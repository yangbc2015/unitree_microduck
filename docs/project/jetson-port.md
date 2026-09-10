# The Jetson port

This fork runs the daemons on a **Jetson Orin Nano Super DevKit** instead of the Radxa Zero 3W
upstream targets. Both are aarch64 Linux, which is where the similarity ends: the capture path, the
video encoder, the servo bus and the IMU all differ, and only the first two have been ported.

Everything here is measured on the board unless it says otherwise, and the numbers are the point -
they are what says whether a difference is a port or a project.

## Where the two boards differ

| | upstream (Radxa Zero 3W) | this fork (Jetson Orin Nano) | state |
|---|---|---|---|
| build | cross-compiled, `cargo zigbuild`, glibc pinned | native aarch64 | done |
| servos | 15x Dynamixel XL330, Protocol 2.0, `/dev/ttyS2` | 15x Unitree S288, 6 Mbps custom protocol | not ported |
| IMU | `imu_to_dxl` board, read on the servo bus | LSM6DSV16X over I2C | not ported |
| capture | rkisp, `v4l2src` on `/dev/video0` | Argus, `nvarguscamerasrc` on a CSI port | ported |
| exposure | `rkaiq_3A_server` plus a software loop | the ISP, which never stops converging | ported |
| H.264 | `mpph264enc`, Rockchip VPU | **no hardware encoder at all**, so `x264enc` | ported |
| WebRTC | `webrtcsink` from the `microduck-gst-plugins` release, patched to know `mpph264enc` | `webrtcsink` from `gst-plugins-rs`, stock | build step |
| detector | RKNN on the NPU (`.rknn`) | none; the `.onnx` path would be CPU | open |
| audio | AIC3104 on I2C3, AIC3x DKMS | nothing chosen | open |
| depth | VL53L5CX/L8CX over I2C | same sensor class would apply | unverified |
| provisioning | `setup-rkaiq.sh`, `setup-npu.sh`, `setup-gstreamer.sh`, `provision-board.sh` | needs its own counterparts | partial |

The detector row is open on both boards, which is worth knowing before treating it as work this
fork introduced: `npu-bringup.md` says nothing on the robot can get a frame yet, and no behaviour
consumes a detection.

## What builds

`cargo check --workspace --all-targets` is clean on the board, and `cargo build --release --bins`
produces thirteen aarch64 binaries including `mediad`, whose GStreamer bindings are the heaviest
dependency in the workspace. Nothing needed a `cfg` to get there.

The reason is that the vendor-specific pieces were already reached at runtime rather than at link
time upstream: `duck-detect` `dlopen`s `librknnrt.so`, `robotd` `dlopen`s the ONNX Runtime, and the
codec plugins are looked up as GStreamer elements. A board with none of them still compiles, which
is exactly the property that makes a port like this one start as a build rather than as a fork of
the whole tree.

One build-time caveat, and it is about the network rather than the board: `tof` pulls
`pollen-robotics/bmi088-rs` as a **git** dependency, so a fresh checkout needs to reach
`github.com` once, and cargo's libgit2 does not honour `http_proxy`. Behind a proxy:

    export https_proxy=http://127.0.0.1:7890 CARGO_NET_GIT_FETCH_WITH_CLI=true
    cargo check --workspace --all-targets

`CARGO_NET_GIT_FETCH_WITH_CLI` is the part that matters - it hands the fetch to the `git` binary,
which does read the environment. With the dependency already in `~/.cargo/git`, `--offline` works
and neither is needed.

The same variable is worth *not* setting around `cargo test`: two of `mediad`'s own tests
(`turn::tests::a_failure_names_its_cause_and_not_just_itself` and
`relay::tests::signing_the_robot_out_drops_the_connection`) exercise connection failures, and an
exported `http_proxy` turns the failure they assert on into a different one - a TLS handshake that
reaches the proxy instead of a refused connection. With the proxy unset, `cargo test -p mediad
--lib` is 95 passed, 0 failed.

## The camera

### What the board reports

    vi-output, imx219 9-0010 (platform:tegra-capture-vi:1) -> /dev/video0

The sensor is an Arducam IMX219 on CAM0 (CSI port B in the RCE trace, `sensor-id=0` to Argus). Its
V4L2 node offers raw `RG10` Bayer at 3280x2464 up to 1920x1080@30 and 1280x720@60, none of which is
a picture anyone wants: the debayer, the white balance and the gamma live in the ISP, behind Argus.

Argus lists what it can offer, which is the list that matters for the geometry:

| mode | resolution | rate |
|---|---|---|
| 0 | 3280x2464 | 21 |
| 1 | 3280x1848 | 28 |
| 2 | 1920x1080 | 30 |
| 3 | 1640x1232 | 30 |
| 4 | 1280x720 | 60 |

**Asking for 1280x720@30 selects mode 2 and scales**, not mode 4: the frame rate asked for is what
picks the mode, and 720p is only a mode of its own at 60. So the sensor runs 1920x1080 at both of
the configured rungs, which is what makes reporting `SensorMode::PINNED` correct for both rather
than a convenience - the same reasoning upstream reaches through a different route, where the rkisp
scaler does the same downscale from the mode `media-ctl` pinned.

A single frame, through the ISP:

    gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 \
      ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' \
      ! nvjpegenc quality=90 ! filesink location=/tmp/camtest/snap.jpg

335 KB, and 98.7% of the first 200 KB is non-zero bytes, which is what says the frame is a picture
rather than a black rectangle. `num-buffers=1` matters: without it the pipeline never reaches EOS
and the file grows until someone kills it.

### The raw tap

The tee carries raw frames for the detector and for a snapshot on demand, and the format is not
negotiable: `duck-detect` wants `UYVY`. Argus does not produce it, so the port converts, once:

    nvarguscamerasrc ! NVMM/NV12 ! nvvidconv ! video/x-raw,format=UYVY

Ten frames at 1280x720 come out as exactly 1280x720x2 bytes each, all of it non-zero. Upstream
carries `UYVY` for a different reason - rkisp's two-plane `NV12` cannot be driven at full rate by
`v4l2src` - but the format it settled on is the one this board can produce, so the detector, the
console and the WebRTC branch are untouched by the difference.

### Software H.264, and what it costs

This JetPack's `nvvideo4linux2` plugin registers `nvv4l2decoder` and nothing else: the Orin Nano
has no NVENC block, so there is no `nvv4l2h264enc`, no `nvv4l2vp8enc` and no `nvv4l2av1enc` to fall
back to. `webrtcsink` therefore reaches `x264enc`, which it already knows how to drive - so the
patch upstream carries for `mpph264enc` is not needed here.

**A second plugin, `nvenc`, is installed and looks like it contradicts that - it does not.** It loads
and tries to register its encoders, fails, and logs the failure at startup:

    ERROR nvenc gstnvenc.c:685:gst_nv_enc_register: NvEncOpenEncodeSessionEx failed: codec h265, device 0, error code 2
    WARN  nvh264encoder:gst_nv_h264_encoder_register_cuda:<cudacontext0> Failed to open session
    WARN  nvh265encoder:gst_nv_h265_encoder_register_cuda:<cudacontext0> Failed to open session

What matters is the consequence, not the noise: an element that fails to register **is not there at
all**, so `gst-inspect-1.0 nvh264enc` answers "no such element" and `webrtcsink` cannot pick it.
Check that rather than the log - the logs read like a fault and are not one:

    gst-inspect-1.0 nvh264enc     # no such element: nothing to fall back to
    gst-inspect-1.0 x264enc | grep Rank    # primary (256), the best H.264 encoder on the board

Measured, 720p30, capture through `nvvidconv`, `x264enc speed-preset=ultrafast tune=zerolatency`,
300 frames:

| | value |
|---|---|
| wall | 11.78 s |
| user + sys | 5.63 s + 1.97 s |
| implied CPU | about 0.65 of a core |
| output | 2.56 MB, about 2 Mbps |

Upstream's `mpph264enc` costs 0.076 of a core, so this is roughly eight times the CPU for the same
picture. It is affordable on six cores at 720p30 next to `robotd`'s 50 Hz loop, and it is the
number to re-measure before anyone raises `[media] quality`: 1080p30 software H.264 is a different
proposition, and the cores are shared with the control loop.

### webrtcsink: which branch of gst-plugins-rs, and why the branch is load-bearing

`webrtcsink` is not in any Ubuntu suite, and `mediad` looks it up as a GStreamer element at run
time. Build it from the **0.14** branch:

    git clone --depth 1 -b 0.14 https://github.com/GStreamer/gst-plugins-rs
    cd gst-plugins-rs
    cargo build --release -p gst-plugin-webrtc --no-default-features
    sudo cp target/release/libgstrswebrtc.so /usr/lib/aarch64-linux-gnu/gstreamer-1.0/
    gst-inspect-1.0 webrtcsink

**The branch is not cosmetic, and this cost an evening.** The 0.13 series moved the signalling
server out of the sink and into a separate `gst-webrtc-signalling-server` binary, so
`run-signalling-server`, `signalling-server-host` and `signalling-server-port` do not exist there.
`mediad` sets all three, and setting a property an element does not have panics inside a GObject
call - so a 0.13 build produces a daemon that cannot start, with a message about a missing property
rather than about the version. The three properties came back in 0.14 (`Since:
plugins-rs-0.14.0`), which is also the branch pinned to `gst-rs` **0.24** - the same bindings
version the daemons build against. The 0.13 branch is `gst-rs` 0.23, which is how the mistake is
made in the first place: it builds cleanly, installs, and registers `webrtcsink`.

Verified after installing 0.14.5: `gst-inspect-1.0 webrtcsink` answers for every property the
daemon touches - `run-signalling-server`, `signalling-server-host`, `signalling-server-port`,
`meta`, `video-caps`, `start-bitrate`, `congestion-control`, `stun-server`, `turn-servers` - and
for the `encoder-setup` and `consumer-added` signals.

`--no-default-features` drops the Janus, WHIP and `web_server` signallers, which pull in `warp`,
`reqwest` and a web server this robot does not run. The in-process signalling server is part of the
plugin itself and needs no feature.

Bootstrapping `gst-plugins-rs` needs network access, `libgstreamer1.0-dev`,
`libgstreamer-plugins-bad1.0-dev` (for `gstreamer-webrtc-1.0`) and a Rust toolchain. The build takes
about five minutes on the Orin Nano's six cores.

### The tee's format, and why the encoder branches convert

Upstream pins the tee to `UYVY`, and this fork keeps it, for three reasons that are all upstream's:
rkisp pushes it fastest (29.3 fps against 19.7 for the same capture with a `videoconvert` in front of
the tee), the detector reads luma straight out of it, and `mpph264enc` takes it and converts to
4:2:0 on the RGA for nothing.

**`x264enc` has no `UYVY` sink at all.** `gst-inspect-1.0 x264enc` lists `Y444, Y42B, I420, YV12,
NV12, GRAY8, Y444_10LE, I422_10LE, I420_10LE` - and nothing 4:2:2 packed. So on this board every
encoder branch needs a conversion, and the first run failed at the tee:

    ERROR mediad: mediad cannot start error=could not attach the H.264 branch to the tee:
        linking a tee branch failed: Noformat

`Noformat` names the tee rather than the format, which is why it read as a caps mystery rather than
as an encoder that cannot take the frame it was handed. Reproduced outside the daemon, where
GStreamer names both ends:

    WARNING: erroneous pipeline: could not link videoscale0 to x264enc0 with caps
        video/x-raw, width=(int)640, height=(int)360

The fix is one `videoconvert` per encoder branch, **after the tee**: `queue ! videoconvert !
webrtcsink`, and `... ! capsfilter ! videoconvert ! x264enc` in the frame-stream branch. Both are
conditional on `gst::ElementFactory::find("mpph264enc").is_none()`, so a Radxa's pipeline is
byte-for-byte what it was - upstream's measurement is why that matters, and the raw branch's
consumers want the unconverted frame in any case.

Measured on this board, 720p30, 300 frames, capture and the tee included:

| chain | wall | CPU |
|---|---|---|
| `nvarguscamerasrc ! nvvidconv ! UYVY ! tee ! queue ! videoconvert ! x264enc` | 11.84 s | 0.75 of a core |
| the same with `webrtcsink` in place of the encoder | 12.63 s | 2.76 of a core |

Both hold 30 fps, so neither the conversion nor `webrtcsink` throttles the capture - the daemon logs
one `capture is below its target rate fps=6.8` for its first second and nothing after, which is
startup and not a steady state. The higher CPU of the `webrtcsink` row is the next thing worth
measuring properly: it is with **no consumer connected**, and the same run as the daemon (also with
no consumer) costs **0.02 of a core**, so the two are not measuring the same thing yet. What the
steady cost is with a browser actually receiving is still open.

### The consumer

A browser has now connected to this board and **shown the picture**: signalling negotiation, ICE, DTLS and the H.264 stream, end to end, with the console served from `:8080` and the session arriving from `webrtcbin` on `:8443`. Two failures were in the way and both are in "What bit on the way" below - item 10 (the pipeline never started, so the signalling server had no producer at all) and item 11 (`webrtcbin` had no ICE, so every session ended the moment it began). The camera path underneath was working for both; what neither of them had was a session that could carry it.

The evidence, in the order the page walks it:

    list            -> [{"id": "2669f8ae-...", "meta": {"api_version": "27", "release": "0.11.0-jetson"}}]
    startSession    -> sessionStarted
                    -> peer {sdp: {type: offer, ...H264/90000, sendonly, BUNDLE video0+application1}}
                    -> peer {ice: candidate ... 10.65.32.235 ... } and a STUN srflx candidate
    /run/mediad/camera.json -> {"fps":30.0,"targetFps":30,"width":1280,"height":720,"dropped":0}

What is still unverified is the rest of the console rather than the video: the control datachannel, the telemetry it carries, and the raw-frame request. `Source::Test` is still how a session problem is told from a capture problem - it exercises the same session with no camera involved.

One thing about the console worth knowing before calling it broken: **the page opens its WebSocket when `connect` is clicked, not when it loads.** A reload alone leaves the page idle and the video black, and that is the page working as designed.

## What the port changed

Four files, one of them new. The shape is deliberate: everything additive went into a new module,
and the existing files got an arm each rather than a rewrite.

| file | change |
|---|---|
| `mediad/src/platform.rs` | **new.** `is_argus()`, `owns_exposure()`, and `camera_source()`: a bin of `nvarguscamerasrc` -> capsfilter (`NVMM/NV12`, the configured geometry and rate) -> `nvvidconv` -> capsfilter (system memory, `UYVY`), exposed with a ghost pad |
| `mediad/src/pipeline.rs` | `camera_source` dispatches to the bin and reports `SensorMode::PINNED`; `Camera` gains `sensor_id`; `wire_encoder_setup` gains the `x264enc` arm; the two encoder branches gain a `videoconvert`, conditional on there being no `mpph264enc`; `build_stream_branch`'s `AppSink` gains `async(false)` - see item 10, a shut `valve` upstream of an async sink stops the pipeline from ever starting |
| `mediad/src/main.rs` | `--csi-port`; the software exposure loop is skipped where the ISP owns metering |
| `mediad/src/lib.rs` | declares the module |

`wire_encoder_setup`'s arm is worth reading before writing another one. It runs inside a GObject
signal handler, where **a panic does not unwind - it aborts the process**, and `set_property` panics
on both a property an element does not have and a value of the wrong type. Two runs went down that
way here:

    property 'profile' of type 'GstX264Enc' not found
    property 'key-int-max' of type 'GstX264Enc' can't be set from the given type (expected: 'guint', got: 'gint')

The first was a misreading of `gst-inspect`: `x264enc`'s caps carry a `profile` *field*, and the
encoder has no such property, so there is nothing to set - a lower H.264 profile is chosen through
caps, which `video-caps` already restricts to H.264. The second is a real property typed `guint`.
Hence the `has_property` checks in that arm: the names are checked because the alternative is a
daemon that will not start, twice, with a backtrace through `g_closure_invoke` and nothing about
which property was wrong.

Three things are worth knowing about those choices:

**The sensor mode is pinned by caps, not by `media-ctl`.** Argus negotiates the mode from what is
asked of it, so asking for 1920x1080@30 is what keeps the sensor out of its 3280x2464 boot mode and
its 21 fps ceiling. Upstream does the same job through a subdev ioctl on an entity whose name
embeds an I2C bus and address, which exists on rkisp and has no counterpart here. The mode reported
to `media.video` is the pinned one, on the reasoning `camera.rs` already writes down: the field of
view is 62 degrees in every mode, so the intrinsics belong to the geometry rather than to the mode.

**Nothing meters the picture.** `mediad::exposure` exists because `rkaiq` converges once and then
stops responding to the scene; Argus does not stop. Starting that loop here would put two
controllers on one sensor fighting for the same registers, so on this board it is skipped and the
rkisp-only `--exposure` and `--analogue-gain` are ignored - Argus has its own compensation and gain
ranges.

**The capture meter still lands where it did.** `meter_capture_rate` attaches to the source's `src`
pad, and a bin answers `static_pad("src")` with its ghost pad, so the probe sits on the same point
in the pipeline: nothing between it and the driver.

## Running it

    cargo build --release --bins

and then, with the camera on CAM0 and the test pattern as the fallback:

    [media]
    camera = true
    quality = "720p30"      # 720p30, 1080p30, 720p15 or 360p30; 1080p costs roughly 2.5x the encode

`mediad --csi-port 1` moves to CAM1; `--camera-device` is unused on this board. As upstream, the
default source is a test pattern, which is what makes a session verifiable with no camera attached
- it is the right first run on a new board, because it exercises signalling, the datachannel and
the encoder with the capture path out of the picture.

## What bit on the way

Eleven failures, in the order they were hit. Each one cost more than it should have, which is the
reason they are written down: nine of the eleven had a symptom that pointed somewhere else.

**1. `no webrtcsink`, then `property 'run-signalling-server' not found`.** `gst-plugins-rs` builds
`webrtcsink` for whichever series matches the GStreamer underneath. The 0.13 series moved the
in-process signalling server out into a separate `gst-webrtc-signalling-server` binary, so the three
properties `mediad` sets are gone there and a `set_property` on a missing name panics. 0.14 is the
series pinned to `gst-rs` 0.24, the same one the daemons use, and it still has them. Check the
properties with `gst-inspect-1.0 webrtcsink`, not the branch name.

**2. `linking a tee branch failed: Noformat`.** The tee carries `UYVY` and `x264enc` has no `UYVY`
sink. `Noformat` names the tee, so it reads as a caps mystery. Reproducing it in `gst-launch` names
both ends: `could not link videoscale0 to x264enc0`.

**3. `property 'profile' of type 'GstX264Enc' not found`, and the process aborted.** The `profile` in
`x264enc`'s caps is a *caps field*; the encoder has no such property. Worse, the panic was inside a
GObject signal handler, where it cannot unwind: it takes the daemon down with a backtrace that never
mentions the property.

**4. `property 'key-int-max' ... expected: 'guint', got: 'gint'`.** Same abort, same handler, a
value of the wrong type. Both of these are why `wire_encoder_setup`'s arm checks names before it sets
them.

**5. `ERROR nvenc gstnvenc.c: NvEncOpenEncodeSessionEx failed`, every start.** The `nvenc` plugin is
installed, has no encode hardware to open a session on, and says so while failing to register its
encoders. It looks like a fault and is not: an element that fails to register is not there at all,
so `gst-inspect-1.0 nvh264enc` answers "no such element" and `webrtcsink` cannot pick it. Ask
`gst-inspect`, not the journal.

**6. The expensive one: the session came up, the browser connected, and there was no picture.**

    (Argus) Error 0x00030003: Connecting to nvargus-daemon failed: No such file or directory
    Error generated. gstnvarguscamerasrc.cpp, execute:940 Failed to create CameraProvider

`nvargus-daemon` listens on `/tmp/argus_socket`, and upstream's unit sets `PrivateTmp=yes`. Inside
the unit's private `/tmp` the socket does not exist, so `nvarguscamerasrc` cannot reach the daemon
and never captures a frame - while the pipeline builds, the signalling server listens, the port
answers, and the failure surfaces in the browser as a video problem. `deploy/jetson/10-argus-socket.conf`
binds that one path in, keeping the private `/tmp`; see the file for why it is a drop-in and why it
is `After=nvargus-daemon.service`.

**The lesson under it: a built pipeline is not a frame.** `mediad::platform`'s "head camera through
Argus" line is emitted when the capture bin is *built*, and "signalling server listening" when the
pipeline reaches PLAYING. Both were true for hours while nothing was being captured, and both were
read as proof that the camera worked. What answers the question is `sudo lsof /dev/video0` - with
the socket hidden, nothing holds the camera - or a count of the frames that came out.

**7. `Failed to create CaptureSession` when probing the camera by hand.** Argus allows one session at
a time, so while `mediad` holds it, the obvious sanity check (`gst-launch-1.0 nvarguscamerasrc ...`)
fails and looks like the camera is broken. `sudo systemctl stop mediad` first.

**8. A second `gst-launch` was not needed to prove the second point, but the first attempt proved
nothing either.** It ran as `sudo -u mediad`, which does not carry the supplementary groups the unit
grants (`SupplementaryGroups=video render robot`), so NVMM failed to initialise and the test failed
for a reason that had nothing to do with the question. `sudo setpriv --reuid=mediad --regid=mediad
--groups=44,993,978` is the faithful equivalent; `systemd-run --uid=mediad` was not.

**9. Environment leaks between the proxy and the tests.** With `http_proxy` exported, `curl` to a
local service answers the *proxy's* 502 (which reads as "the console is down"), and two of `mediad`'s
own tests fail because they assert on connection *errors* that a proxy changes. Unset the proxy for
local checks and for `cargo test`; `curl --noproxy '*'` for the console. `cargo`'s git dependencies
need the opposite: `https_proxy=... CARGO_NET_GIT_FETCH_WITH_CLI=true`, because `libgit2` ignores
`http_proxy` and otherwise the build sits at 0% CPU forever.

**10. The console said "no producers", and the pipeline had never started.** The page loaded, opened
its socket, and stopped on the line that names this exact possibility:

    !! no producers. mediad registers as one when its pipeline reaches PLAYING -
       check its journal for a pipeline that would not start.

The journal said the opposite: the camera bin was built, the signalling server was listening on
`8443`, `curl` on `:8080` answered, and the daemon sat at 0.0 of a core with no error anywhere. Two
facts settled it, and neither is a log line: `/run/mediad/camera.json` - the file `meter_capture_rate`
rewrites once a second while frames arrive - **did not exist**, and the pipeline reported

    pipeline state: ready -> paused        <- and no paused -> playing, ever
    tee=8  sink=1                          <- the preroll buffer, then silence

So the source had delivered its preroll buffer and stopped. `webrtcsink`'s codec discovery never saw
a second one, which matters because its signaller only starts once discovery is done
(`should_start_signaller`): no signaller, no producer registration, no producer in `list`, no
session, no video. Every consequence of that looked like a client-side problem, which is why the
console's own message is honest and misleading at the same time.

The cause is the valved H.264 branch, and it is general rather than a `mediad` bug: **a `valve` with
`drop=true` starves the sink downstream of it, and a sink that goes asynchronously to PAUSED
(`async=true`, the default for both `appsink` and `fakesink`) waits for a buffer that will never
arrive - so the pipeline never leaves PREROLLING.** GStreamer says so in one line, with no `mediad`
in sight:

    $ gst-launch-1.0 -e videotestsrc num-buffers=60 ! tee name=t \
        t. ! queue ! fakesink sync=false t. ! queue ! valve drop=true ! fakesink sync=false
    Setting pipeline to PAUSED ...
    Pipeline is PREROLLING ...              <- and there it stays

`drop=false`, or `async=false` on that branch's sink, and it prerolls and plays. This branch is shut
on purpose ("shut until something asks"), so it must not hold up preroll: `build_stream_branch` now
sets `async(false)` on its `AppSink`, beside the `drop(false)` it already had.

How it was found, because the first attempts were wrong: `gst-launch` cannot express `meta` or
connect `encoder-setup`, so the whole pipeline was rebuilt through the GObject bindings with every
property `mediad` sets, with buffer counters on the tee and on `webrtcsink`'s sink pad, and then
**bisected by dropping one ingredient at a time**. Only the valved branch turned "stuck in PAUSED"
into "60 buffers every two seconds". `gst-launch` remains the right tool for the reduced case, and
the counters are what make a stall visible instead of inferred.

**11. `libnice elements are not available`, and the session died in the millisecond it started.**
With the pipeline playing and the producer in `list`, the console walked to `sessionStarted` and then
got `endSession` from the robot - no SDP offer was ever sent. The producer's side names it:

    WARN gst: error: libnice elements are not available cat=webrtcbin
    ERROR gst: Failed to request pad from webrtcbin cat=webrtcsink
    ERROR mediad::pipeline: pipeline error ... Failed to request pad from webrtcbin

`webrtcbin` needs the `nice` plugin for ICE. This board had `libnice10` (the library) and not
`gstreamer1.0-nice` (the plugin that provides `nicesrc`/`nicesink`), which is half of what
`scripts/setup-gstreamer.sh` installs - and that script says why: "ICE. webrtcbin negotiates nothing
without it". `gst-inspect-1.0 nicesrc` is the check, `sudo apt-get install -y gstreamer1.0-nice` the
fix. From the browser this failure is invisible: the session is announced and then withdrawn, and
nothing on the page says ICE was never available.

## Open items

- **The rest of the console.** The picture is confirmed end to end (see "The consumer"), by a browser
  and by a WebSocket client replaying the page's own sequence. Unverified from here: the control
  datachannel `mediad` creates per consumer, the telemetry it carries, and the raw-frame request
  behind `media.frame`. Both are exercised by the same session the video takes, so a session that
  shows a picture and a control channel that answers are separate questions rather than one.
  `Source::Test` is how to tell a session problem from a capture problem.
- **Servo bus.** `duck-control::RobotIo` is the seam - see `JETSON.md` - and until an impl exists
  for the S288 protocol, `robotd` retries `/dev/ttyS2` and runs without a body.
- **IMU.** Upstream reads it on the servo bus in the same transaction as the joints; the fork's
  LSM6DSV16X is on I2C, so it becomes part of that same `read()` rather than a second source.
- **Rotation and intrinsics.** The mount angle is configuration (`--rotate`), and the family
  calibration in `robotd-params` was solved for the Radxa's module. The Arducam IMX219 is the same
  sensor with the same 62 degree field, so the nominal figures should hold - a calibration is the
  only thing that says so.
- **Audio and depth.** Unported, and unstarted: `deploy/audio` is a Radxa device-tree and codec
  combination, and `tofd` has not been run against a sensor on this board.
