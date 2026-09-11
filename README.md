<p align="center">
  <img src="https://github.com/user-attachments/assets/c2f7c245-8217-46a1-8d1e-e0ba967cd969" alt="microduck" width="820">
</p>

<h1 align="center">Microduck · Jetson Orin Nano · Unitree S288</h1>

<p align="center">
  <em>A tiny biped robot that moves using reinforcement learning policies.</em><br>
  A fork of <a href="https://github.com/pollen-robotics/microduck">pollen-robotics/microduck</a> —
  same brain, different body.
</p>

<p align="center">
  <a href="#what-is-actually-fitted"><b>The robot, as built here</b></a> ·
  <a href="#building-this-one">Building this one</a> ·
  <a href="JETSON.md">The port and its branch rules</a> ·
  <a href="docs/project/jetson-port.md">What bit on the way</a> ·
  <a href="docs/project/s288-servo-port.md">The servo swap</a>
</p>

<p align="center">
  <a href="https://github.com/yangbc2015/unitree_microduck/actions/workflows/ci.yml"><img src="https://github.com/yangbc2015/unitree_microduck/actions/workflows/ci.yml/badge.svg" alt="CI"></a>
</p>

---

**This repo is the duck's brain.** About 25 cm and 800 g of robot, run by a handful of daemons:
a 50 Hz control loop driving fifteen servos from neural policies, the radios and the camera, and the
update machinery that gets new software onto a robot without bricking it.

Upstream that runs on a Rockchip RK3566 and drives Dynamixel XL330s. **This fork runs the same
daemons on a Jetson Orin Nano and drives Unitree S288 servos instead** — it exists because a Jetson
was already on the bench, and because the S288 is the same *kind* of part (a small serial-bus servo
that reports its own position and torque) while being far easier to buy. Everything above the bus
is unchanged, which is what the port was betting on: `RobotIo` is the seam, and swapping the body
underneath it costs one type alias and a second implementation.

The policies are trained next door, in
**[microduck_rl](https://github.com/pollen-robotics/microduck_rl)** — MuJoCo and PPO, the sim2real
recipe, and the export to ONNX that this repo loads. Training is upstream's; what changes here is
the body those policies land on, and since the actuator is different the simulator's actuator had to
be *measured* rather than inherited — [the servo swap](docs/project/s288-servo-port.md) is that
work, and the measurements are in `unitree_servo/bam_data/`.

## This fork: a Jetson Orin Nano

This is a fork of [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) that runs
the same daemons on a **Jetson Orin Nano Super DevKit** instead of the Radxa Zero 3W, and drives
**Unitree S288** servos instead of Dynamixel XL330s. The two boards are both aarch64 Linux and share
nothing else that matters: the capture path, the video encoder, the servo bus and the IMU all differ.

| | upstream (Radxa Zero 3W) | this fork (Jetson Orin Nano) | state |
|---|---|---|---|
| build | cross-compiled, glibc pinned | native aarch64, whole workspace clean | done |
| servos | 15x Dynamixel XL330, Protocol 2.0, `/dev/ttyS2` | 15x Unitree S288, 6 Mbps custom protocol | code done, unverified on hardware |
| IMU | `imu_to_dxl` board on the servo bus | LSM6DSV16X over I2C | not ported |
| capture | rkisp, `v4l2src` on `/dev/video0` | Argus, `nvarguscamerasrc` on a CSI port | ported |
| exposure | `rkaiq` 3A, plus a software loop | the ISP, which never stops converging | ported |
| H.264 | `mpph264enc` on the Rockchip VPU | no encoder at all, so `x264enc` | ported |
| WebRTC | `webrtcsink` from `microduck-gst-plugins`, patched | `webrtcsink` from `gst-plugins-rs`, stock | ported |
| detector | RKNN on the NPU | none yet; unported upstream too | open |
| audio | AIC3104 on I2C3 | nothing chosen | open |

### What is actually fitted

| | |
|---|---|
| servos | 15x **Unitree S288** on IDs 0-14, one half-duplex single bus at 6 Mbps. **Not Dynamixel Protocol 2.0** — a different frame format with a different CRC, and upstream's spec is wrong about it in three places. See [`docs/project/s288-servo-port.md`](docs/project/s288-servo-port.md). |
| servo adapter | Unitree's AT32 single-bus-to-USB module (`2e3c:7640`), addressed by its stable `/dev/serial/by-id` path — a `/dev/ttyACM<n>` number depends on enumeration order, so a second USB serial device would rename the bus out from under a running daemon. |
| IMU | **LSM6DSV16X** over I2C, SFLP fusion running in the chip. Replaces upstream's `imu_to_dxl` board, which rode the servo bus and was read in the same transaction as the joints. |
| camera | Arducam **IMX219**, 22-pin, 62° FOV, on a CSI port. |
| power | the servos want a **12 V rail** and the Jetson takes 9-20 V on its barrel jack, which is why a 3S pack can feed both. On the bench the two are **separate supplies** — a bench supply for the servos, the Jetson's own for the board. Never one supply for both: 19 V straight into an S288 is over its 12.6 V limit, and the Jetson's 45 W cannot cover fifteen servos' stall current on top of its own draw. |

Two measurements from the servo work are worth knowing before touching anything, because both are
invisible until something misbehaves: the S288's torque response is a **0.48 ms pipeline delay plus a
first-order ~0.54 ms**, and there is **1.06° (0.0184 rad) of backlash** between the position the rotor
is commanded to and where the output shaft actually is. A control loop that assumes the output
follows the rotor command within encoder resolution is assuming something this servo does not
deliver.

### Building this one

Two scripts, and they *are* the install — the skeleton is idempotent (the systemd units, the
`robot` group and the other daemon users, the state directory, the journald drop-in, the
`deploy/jetson/*.conf` drop-ins, and `deploy/robotd.toml` to `/etc/robot/` only when the board has
none), and the binaries script builds the workspace and installs **every** binary into one release
tree:

```
scripts/deploy-jetson-skeleton.sh
scripts/deploy-jetson-binaries.sh
```

A release tree is all-or-nothing on purpose. `hello` is answered by `updaterd` and `robot.policies`
by `robotd`, so a half-updated tree answers the console in two versions at once — which is exactly
what happened here once, and is why the binaries script installs the whole workspace rather than the
daemon that changed.

The camera is ported and a browser has been watching it: capture, software H.264, signalling, ICE and
the control datachannel the console opens beside the video are all verified end to end on the board.
The servo bus and the IMU are what is left of the port, so this board can be watched but not driven.

**The build is not the port and the port is not the build.** Everything compiles on the board
because the vendor-specific pieces were already reached at runtime upstream - `librknnrt.so` and the
ONNX Runtime are `dlopen`ed, the codec plugins are looked up as GStreamer elements - so a board with
none of them still builds. What then has to be ported, and what it costs, is in
[`docs/project/jetson-port.md`](docs/project/jetson-port.md): what the camera actually delivers, why
there is no `nvv4l2h264enc` to find, and the measured price of encoding in software (about 0.65 of a
core at 720p30, against upstream's 0.076 on the VPU). [`JETSON.md`](JETSON.md) has the branch rules
for keeping the fork mergeable with upstream.

## It does things

<table>
<tr>
<td width="50%">
  <video src="https://github.com/user-attachments/assets/356a6011-8e0d-4b28-bda9-da78646583a3" controls width="100%"></video>
</td>
<td width="50%">
  <video src="https://github.com/user-attachments/assets/abfbf250-1b1c-42cb-8430-00267e2b148a" controls width="100%"></video>

</td>
</tr>
<tr>
<td><b>It walks.</b> Pick up a gamepad and drive.</td>
<td><b>It rolls.</b> Put wheels on, hold D-pad up, and it loads the other brain.</td>
</tr>
<tr>
<td width="50%">
  <video src="https://github.com/user-attachments/assets/7e70c1da-e120-428f-ae0b-f4de62f25984" controls width="100%"></video>
</td>
<td width="50%">
  <video src="https://github.com/user-attachments/assets/3eef63a5-6f84-47cf-90de-e717e6d7f8f0" controls width="100%"></video>
</td>
</tr>
<tr>
<td><b>It picks things up.</b> Beak to the floor, one button.</td>
<td><b>It gets back up.</b> Knock it over and it stands itself up.</td>
</tr>
</table>

It also sits, kicks a ball, rolls forward on command, and quacks in a voice that is its own.

## Where to find things

### You have a duck

| | |
|---|---|
| [Cheat sheet](docs/robot/cheatsheet.md) | Every `robotctl` command: drive, configure, voice, chorale, theremin, wifi, updates, logs. Start here. |
| [Gamepad](docs/robot/cheatsheet.md#gamepad-configd) | The full button mapping, and pairing a pad — [once per pad](docs/robot/pair-a-gamepad.md), plus what to do when it will not bond. |
| [`duckctl`](docs/robot/duckctl.md) | The robot from a laptop over Bluetooth, with no network and no ssh. |
| [Updates](docs/robot/cheatsheet.md#updates-updaterd) | Install, roll back, pin. Every update is verified, health-gated and reversible. |

### You are building on it

| | |
|---|---|
| [microduck_rl](https://github.com/pollen-robotics/microduck_rl) | Where the policies come from: MuJoCo, PPO, domain randomisation, and the ONNX export this repo loads. |
| [The servo swap](docs/project/s288-servo-port.md) | Fifteen XL330s out, fifteen S288s in — **start here for this fork.** What the protocol actually is, what had to be measured rather than inherited, and what is still bench work. |
| [JETSON.md](JETSON.md) | Why this fork is shaped the way it is, and the branch rules that keep it mergeable with upstream. |
| [What bit on the way](docs/project/jetson-port.md) | The port told as the failures it produced: capture, the encoder, the release tree, the mount angle. |
| [How it works](docs/design/architecture.md) | The whole system on one page — the daemons, the bus, how an update reaches a robot — then a page per part. |
| [Set up a dev board](docs/robot/install-dev.md) | From a blank board to a robot that takes branch builds. |
| [Dev cheat sheet](docs/robot/cheatsheet-dev.md) | Branch builds, release candidates, driving from a laptop, and the restart traps after an update. |
| [Push your branch](docs/robot/dev-push.md) | Build on your machine, install over ssh, about a minute. |
| [The simulated duck](docs/robot/simulation.md) | No robot on the desk? `scripts/duck-sim` runs the real daemons against a body in MuJoCo — one duck in a window, or four as machines you log into. |
| [CONTRIBUTING.md](CONTRIBUTING.md) | Building, testing, layout, conventions, releasing. |
| [Docs index](docs/README.md) | Everything, including the design pages and the open problems. |

## Under the hood

Rust, no framework, one workspace. `robotd` owns the control loop and the motor bus; `updaterd`
installs signed releases and rolls them back when a robot comes up unhealthy; `configd` owns wifi
and identity; `btd` is the Bluetooth path a phone uses; `padd` reads the gamepad; `mediad` streams
the camera over WebRTC; `tofd` serves the depth sensor. They talk over one JSON-RPC contract on
Unix sockets, and every client — the app, the console, the gamepad, your script — sends exactly the
same calls.

The interesting decisions are written down: [`docs/design/`](docs/design/) is why things are the
way they are, and [`docs/project/`](docs/project/) is what has gone wrong and what would close it.

## A note on ducks

No duck was harmed in the making of this robot. Several were consulted.
