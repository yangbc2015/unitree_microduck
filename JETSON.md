# Running microduck on a Jetson Orin Nano

Notes for the port to a Jetson Orin Nano Super DevKit (aarch64, Ubuntu for Jetson, 8 GB + NVMe).
Upstream targets a Radxa Zero 3W (RK3566, Armbian). This file records what that difference costs,
so the fork stays small enough to keep merging upstream.

## Branch model

- `main` tracks `upstream/main` (pollen-robotics/microduck) and is never committed to.
- `jetson` holds every port change. Sync: `git fetch upstream && git merge upstream/main`.
- Ground rules, all of them about keeping the diff cheap to rebase:
  - new files over edited files
  - configuration over code
  - no renames, no repo-wide `cargo fmt` (it rewrites whole files and turns every hunk into a conflict)
  - vendor blobs are reached with `dlopen` (as `duck-detect/src/rknn.rs` and the ONNX Runtime path
    already do), so `cargo check --workspace` keeps working on a machine with no vendor libraries

## Where the port stands

| area | state on the Jetson |
|---|---|
| build | clean. `cargo check --workspace --all-targets` passes; 13 release binaries, all aarch64 ELF |
| systemd / updater skeleton | deployed, `robotd` + `updaterd` active |
| servo bus | not connected. `robotd` retries `/dev/ttyS2` forever (the Radxa UART) |
| IMU | not connected. Upstream reads an `imu_to_dxl` board on the servo bus |
| camera / video | not ported. Pipeline is Rockchip MPP (`mpph264enc`) + rkisp + an rkaiq 3A loop |
| detector | upstream is not wired to the detector either (`docs/project/npu-bringup.md`: nothing can get a frame yet) |
| audio | not ported. Upstream is an AIC3104 on the Radxa I2C3 header |
| ToF | unverified. Same VL53L5CX/L8CX class of 8x8 sensor over I2C, so mostly a device-node question |

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
- `scripts/setup-jetson-*.sh`: the Jetson counterparts of `setup-npu.sh`, `setup-rkaiq.sh`,
  `setup-gstreamer.sh`, `scripts/provision-board.sh`, plus the preinstall hook.

Core patches, kept as small as possible:
- `robotd/src/main.rs`: the `BusIo` alias and `open_bus()`.
- `mediad/src/pipeline.rs` and `mediad/src/main.rs`: capture and encoder element names, which are
  hardcoded to `rkisp` and `mpph264enc`. `cfg(target_os = "linux")` cannot separate the two boards,
  so this wants a feature flag or runtime configuration rather than a cfg.
- `mediad/src/exposure.rs`: the rkaiq 3A loop. Jetson's Argus owns exposure and white balance, so
  most of this file has no counterpart rather than a replacement.

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
