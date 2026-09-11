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
| `VISION_INTERVAL` | `5` | seconds between analyses; newest frame wins, the rest are dropped |
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
