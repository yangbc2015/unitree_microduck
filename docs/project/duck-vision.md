# Duck vision: camera frames to a local VLM

This fork gives the duck a local "describe what you see" loop that does not depend on a browser,
the console, or any cloud API:

    nvarguscamerasrc -> mediad --stream-to -> scripts/duck-vision.py -> llama.cpp (MiniCPM-V)

`mediad` owns the camera as usual. The stream branch taps the tee after `jpeg_encoder`, so the
frames sent to the model are already downscaled, rotated upright and JPEG-encoded - the same branch
the console's video uses, and no second Argus session is opened. `scripts/duck-vision.py` is a
stdlib-only WebSocket server that answers `mediad`, keeps the newest frame, and posts it to an
OpenAI-compatible multimodal endpoint every `VISION_INTERVAL` seconds. `scripts/duck-vision.sh`
starts both halves and stops them together.

Nothing here is specific to this checkout: point `LLAMA_URL` at any reachable endpoint that accepts
`POST /v1/chat/completions` with an `image_url` part, and point `--stream-to` at any host running
`duck-vision.py`. The analyser can run off-board; the robot only needs outbound `ws://` (or `wss://`
plus `ROBOT_TOKEN`, matching `mediad/src/stream.rs`).

## Quick start (Jetson, camera on CAM0)

Build the daemon half once, so `mediad` has the startup flags:

    cargo build --release -p mediad

Have a multimodal llama.cpp server up - on this board that is the user service on `:8081` with
MiniCPM-V, but anything OpenAI-compatible works. Then:

    MEDIAD_ROTATE=0 scripts/duck-vision.sh

`MEDIAD_ROTATE=0` is the Jetson/Arducam case: the IMX219 here is upright, while a real duck's head
camera is mounted a quarter-turn off and should leave mediad's default `--rotate 90` alone. The
streamed JPEGs are turned upright by `jpeg_encoder` from that angle, so a wrong value shows up as a
sideways description rather than an error.

Expected log shape:

    duck-vision listening on ws://127.0.0.1:8765/frames
    hello: microduck frames= jpeg fps=1.0 longest=640 mount_rotate=0
    frame 3 (41822 bytes, 8.1s)
    画面偏暗，中间是桌面，左侧有一条黑色线缆……

A one-shot smoke check, without the launcher:

    scripts/duck-vision.py --once --save-dir /tmp/duck-vision &
    target/release/mediad --stream-to ws://127.0.0.1:8765/frames --rotate 0

`--once` prints the first analysis and exits; `mediad` keeps redialing until it is stopped, which is
the intended best-effort behaviour when the analyser is not there.

## Configuration

Everything is an environment variable, so the same checkout runs on another board without edits:

| variable | default | meaning |
|---|---|---|
| `MEDIAD_BIN` | `target/release/mediad`, then the deployed tree | daemon binary; must have `--stream-to` |
| `VISION_LISTEN` | `127.0.0.1:8765` | where `duck-vision.py` listens |
| `VISION_PATH` | `/frames` | WebSocket path mediad dials |
| `LLAMA_URL` | `http://127.0.0.1:8081/v1` | OpenAI-compatible base URL |
| `VISION_MODEL` | first id from `/models` | model field sent; llama.cpp ignores it |
| `VISION_PROMPT` | built-in Chinese scene prompt | sent with every analysed frame |
| `VISION_INTERVAL` | `5` | minimum seconds between analyses, the model's own latency included; newest frame wins, the rest are dropped |
| `VISION_SAVE_DIR` | unset | save each analysed JPEG here |
| `VISION_FPS` | `1` | frames mediad pushes; analysis does not try to keep up with more |
| `VISION_LONGEST` | `640` | longest edge of the pushed JPEG |
| `VISION_QUALITY` | `70` | JPEG quality |
| `MEDIAD_ROTATE` | unset (mediad default 90) | set `0` on the upright Jetson camera |
| `MEDIAD_CSI_PORT` | unset | `0`/`1` to pick the CSI port |
| `MEDIAD_ARGS` | unset | extra mediad arguments, word-split |

`VISION_FPS` and `VISION_INTERVAL` are deliberately separate: mediad can push at 1 fps while the
model needs several seconds per frame. The analyser always uses the newest frame and reports its
sequence number, so a busy model shows up as skipped sequence numbers, not as a growing queue -
`mediad`'s own channel is 8 frames deep and drops with a log line past that.

## Wire protocol

For anyone replacing the analyser, the socket is intentionally boring:

1. mediad connects to `ws://HOST:PORT/PATH` (no subprotocols; `authorization: Bearer $ROBOT_TOKEN`
   only when the URL is `wss://` and the variable is set).
2. It sends one text frame, a JSON hello with `robot` and `frames` (`encoding`, `fps`, `longest`,
   `width`, `height`, `mount_rotate`, `sequence_start`).
3. It sends binary frames, one complete JPEG per message, in capture order.
4. Text frames back are counted and logged by mediad and otherwise ignored; `duck-vision.py` sends
   `{"type":"vision","seq":N,"analysis":"..."}` so the daemon log can corroborate the loop.

The JPEG is produced by the same `jpeg_encoder` used for the console's frame-stream branch, so
`--stream-longest`/`--stream-quality` trade bytes for detail before anything reaches the model.

## Running it at boot (this board)

Both halves come up with no browser and nobody logged in: `mediad` is the system service it always
was, and the analyser is a *user* service (`duck-vision.service`, `WantedBy=default.target`, with
`loginctl enable-linger` in place) so it can reach the same llama-server that owns the model on
`:8081`.

The video path is untouched - that is the whole point of tapping the tee rather than opening a second
source. The flags are added by a drop-in, `deploy/jetson/30-stream.conf`, which also repeats
`--rotate 0`: two drop-ins cannot both own `ExecStart` (the one that sorts last wins), so
`20-mount.conf`'s command line is superseded while its reasoning still stands. Install with
`scripts/deploy-jetson-skeleton.sh`, or by hand into `/etc/systemd/system/mediad.service.d/`, then
check what the daemon actually received before believing the file:

    $ systemctl show mediad -p ExecStart
    ExecStart={ path=/opt/robot/daemon/current/bin/mediad ; argv[]=... --rotate 0 --stream-to ws://127.0.0.1:8765/frames ... }

The proof that this is additive is the console's own meter plus a caption, side by side - one mediad,
one Argus session, both consumers fed:

    $ sudo cat /run/mediad/camera.json
    {"fps":30.0,"targetFps":30,"width":1280,"height":720,"format":"UYVY","frames":2164,"dropped":0,"consumers":0}
    $ journalctl --user -u duck-vision -f
    [17:50:34] frame 70 (45879 bytes, 5.9s)
    画面偏紫暗，物体模糊不清，无明显无线缆或障碍，未见人。

A board-specific guard rides along with it. **llama-server does not give back what a multimodal
request allocates** (measured 2026-09-11: a staircase of roughly 16-60 MiB per request - flat
stretches, then jumps - never released for the life of the process; upgrading llama.cpp did not
change it). With the loop on all day that drift reaches the unit's `MemoryMax=6G` and the kernel
kills the model, which on this board means the console's model too. So the rig runs a user timer,
`llama-anon-guard.timer`, every minute: when llama-server's cgroup `anon` crosses 1.25 GiB, or the
board's `MemAvailable` drops under 1 GiB, it restarts the server - about six seconds, the camera never
blinks. Two triggers because either alone can be wrong: the unit's own ceiling catches the drift, and
the system floor catches a loaded board that is still inside its cap. At most one restart per four
minutes (`MIN_GAP_SECONDS`), and `DRY_RUN=1` reports what it would do instead of doing it. A fresh
server sits at ~0.27 GiB, so the ceiling is ~5x the baseline and the restart happens with >1 GiB of
system memory still free.

That guard is a fact about *this* board, where the model and the console share one 8 GB pool. Point
`LLAMA_URL` at a machine that is not also running the console and there is nothing to restart.

The envelope it settles into here, measured: one request every `VISION_INTERVAL` seconds (15 on this
rig), the guard restarting the server about every five minutes, `dropped` at 0 throughout, and the
console's meter reading 29-30 fps of 30 while the vision consumer is attached - the encoder now also
produces one JPEG a second, which is where that last frame goes. **The interval is the lever that
matters for memory**, because a local llama.cpp retains ~50 MiB per request: at `5` the loop runs at
the model's speed and the drift is three times faster, at `30` the restart is rarer and the
description is correspondingly staler. It is also why `--interval` means *minimum spacing between
analyses* - frames arriving inside it are dropped, and the model's own latency counts against it.

## Pitfalls

- **`nvargus-daemon` must be running.** If `mediad` reports an Argus connection failure and produces
  nothing, restart it (`sudo systemctl restart nvargus-daemon`) and start again. A dead Argus gives
  zero-byte captures, not a clear error at the model end.
- **One Argus session.** Do not run a separate `nvarguscamerasrc` pipeline (for example a manual
  `gst-launch` snapshot) while `mediad` is up; the second source fails or starves the first. Take
  frames from `--stream-to` or `--save-dir` instead.
- **Rotation is a mount statement, not a filter.** `--rotate` describes how the camera is mounted;
  `jpeg_encoder` uses it to turn frames upright. On this Jetson the right value is `0`; on a real
  duck head it is mediad's default `90`.
- **Dark frames are purple, and that is real.** The IMX219's auto white balance gives up in near
  darkness and the ISP hands back purple-red frames. The model will describe them as dark/purple;
  add light rather than tuning the prompt away.
- **A thinking model answers in two parts.** MiniCPM-V 4.6's template thinks by default: the reply
  lands in `content` (often one thin line) and the working goes to `reasoning_content`. The analyser
  asks for `enable_thinking: false` in `chat_template_kwargs` - on one frame that turned a
  12-character answer plus 105 characters of monologue into a 33-character description and no
  thinking. The field is llama.cpp's own extension, so a request that gets `HTTP 400` for it is
  retried without it rather than losing the frame. If a reply still carries no `content`, the frame is
  logged as `nothing to report` and skipped: printing the monologue as a description would put the
  model's voice in the console instead of the scene.
- **Ports.** The vision socket (`8765`) is separate from the console (`8080`), signalling (`8443`)
  and the model (`8081`). The launcher fails fast if `VISION_LISTEN` is taken.
- **Model latency.** A Q8_0 MiniCPM-V on an Orin Nano takes several seconds per 640px frame. Keep
  `VISION_INTERVAL` at or above that; raising `VISION_FPS` does not make the model faster, it only
  gives the analyser a fresher frame to pick.

## Verifying changes

Focused checks, not a suite:

    python3 -m py_compile scripts/duck-vision.py
    bash -n scripts/duck-vision.sh
    cargo test -p mediad stream

Then the one-shot smoke check above with a real camera and the model up. The pass criterion is a
printed description that matches the scene; a wrong-colour or sideways description is a camera/rotate
problem, not a vision-socket problem.
