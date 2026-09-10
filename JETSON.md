# Running microduck on a Jetson Orin Nano

Notes for the port to a Jetson Orin Nano Super DevKit (aarch64, Ubuntu for Jetson, 8 GB + NVMe).
Upstream targets a Radxa Zero 3W (RK3566, Armbian). This file records what that difference costs,
so the fork stays small enough to keep merging upstream.

## Branch model

- `main` tracks `upstream/main` (pollen-robotics/microduck) and **holds every port change**, committed
  here directly. Sync: `git fetch upstream && git merge upstream/main`.
- `origin` is this fork (yangbc2015/unitree_microduck). These are the two remotes, and `main` is
  deliberately both: one branch to rebase, no long-lived `jetson` branch to drift.
- Ground rules, all of them about keeping the diff cheap to rebase:
  - new files over edited files
  - configuration over code
  - no renames, no repo-wide `cargo fmt` (it rewrites whole files and turns every hunk into a conflict)
  - vendor blobs are reached with `dlopen` (as `duck-detect/src/rknn.rs` and the ONNX Runtime path
    already do), so `cargo check --workspace` keeps working on a machine with no vendor libraries

## Where the port stands

The status table, the measurements behind each row, and the open items live in
[`docs/project/jetson-port.md`](docs/project/jetson-port.md). In one line: the whole workspace
builds natively, the camera path is ported, and the servo bus, IMU, audio and depth are not.

## The seam that already exists

The servo bus is already behind a trait, which is why this port is mostly additive:

- `duck-control/src/io.rs`: `pub trait RobotIo`. Required: `read`, `write`, `set_gain`,
  `set_torque`, `reboot`, `slow_sensors`. `imu_stale` and `imu_ready` have defaults.
- `Sensors` carries joints *and* IMU in one value, because upstream's IMU board shares the servo
  bus and is read in the same transaction. A new bus therefore owns the IMU too.
- Existing impls: `DynamixelIo` (`duck-control/src/bus.rs`), `FakeIo` (`duck-control/src/io.rs`),
  `RemoteIo` (`duck-control/src/sim.rs`).
- `robotd` is generic over `RobotIo` (`fn control_loop<T: RobotIo>`), and `robotd/src/main.rs` has a
  cfg'd `type BusIo` alias plus an `open_bus()` that constructs it. That alias is the whole
  integration point: a new impl plus a new alias target, and the control loop, safety arbiter,
  policies and kinematics do not change.

Dynamixel semantics with no S288 equivalent, to be absorbed inside the new impl:
- `reboot` is the Protocol 2 REBOOT instruction
- `check_registers` / `adopt_missing_servo` are EEPROM register writes
- `slow_sensors` reads registers 144-146 for supply voltage and case temperature

## Delta map

Additive (no core changes):
- `duck-control/src/bus_s288.rs`: a `RobotIo` impl speaking the Unitree S288 protocol
  (docs and examples: github.com/unitreerobotics/digital_servo - `specs/protocol.md`,
  `python/servo_demo.py`). CRC plus the position/speed conversion factors.
- IMU: an LSM6DSV16X reader over I2C, feeding `Sensors.imu` from the same `read()`.
- `deploy/jetson/*.toml`: serial port, camera device, model paths, policy slots.
- `deploy/jetson/10-argus-socket.conf`: **in use now.** A systemd drop-in for `mediad.service`, not
  an edit to it. Upstream's unit sets `PrivateTmp=yes`, which on this board hides
  `/tmp/argus_socket` - where `nvargus-daemon` listens - so `nvarguscamerasrc` cannot reach the
  daemon and the session comes up with no frames. The drop-in binds that one path in and adds
  `After=nvargus-daemon.service`. `docs/project/jetson-port.md` § "What bit on the way" has the two
  error lines it fixes, and `lsof /dev/video0` is how you tell it is fixed.
- `scripts/setup-jetson-*.sh`: the Jetson counterparts of `setup-npu.sh`, `setup-rkaiq.sh`,
  `setup-gstreamer.sh`, `scripts/provision-board.sh`, plus the preinstall hook.

Core patches, kept as small as possible:
- `robotd/src/main.rs`: the `BusIo` alias and `open_bus()`. Not started.
- `mediad/`: **done, and the shape to keep.** `platform.rs` (new) holds the Jetson capture path -
  Argus, the two-element bin, and why nothing meters the picture - while `pipeline.rs` and
  `main.rs` grew one arm each to dispatch to it, and `wire_encoder_setup` gained the `x264enc` arm.
  The wire format stayed `UYVY` and the tee, the detector tap and the WebRTC branch are untouched.
  `pipeline.rs` also carries one fix nothing on this board caused: the valved H.264 branch's
  `AppSink` sets `async(false)`, because a shut `valve` starves its sink, an async sink waits for a
  buffer, and the whole pipeline then never leaves PREROLLING - which is what made the signalling
  server report no producer at all. `docs/project/jetson-port.md` has the measurements, and items 10
  and 11 have the two failures that stood between a working camera and a picture in a browser.

Environment, which is part of the port rather than of the code: `scripts/setup-gstreamer.sh` lists
`gstreamer1.0-nice`, and a board that has only `libnice10` gets a `webrtcbin` with no ICE - every
session ends the instant it starts, and the browser sees nothing. `gst-inspect-1.0 nicesrc` is the
check.

## Hardware

- 15x Unitree S288, IDs 0-14. Custom 6 Mbps protocol, not Dynamixel Protocol 2.0.
- IMU: LSM6DSV16X bare module, read over I2C. Fusion (SFLP) runs inside the chip.
- Camera: Arducam IMX219, 22-pin, 62 degree FOV.
- Power: S288 needs a 12 V rail; the Jetson takes 9-20 V on its barrel jack.

## Candidate upstream contributions

Both of these would shrink this fork permanently, and neither changes behaviour on a Radxa:

- parameterise the bus implementation in `robotd` (a `--bus` argument, or a platform feature)
  instead of hardcoding `DynamixelIo`
- parameterise `mediad`'s capture/encoder element names and who owns auto-exposure

Worth an issue first: the project is deliberately single-board and treats complexity as a cost.

## Upstream references

- `docs/project/npu-bringup.md` - the detector, and what is still missing before a behaviour can use it
- `docs/project/media-bringup.md` - why capture is not a plain `v4l2src`
- `docs/design/architecture.md` - the daemons, the IPC contract, and the raw tee branch
- `docs/robot/dev-push.md` - building here and installing on a board as a gated update
