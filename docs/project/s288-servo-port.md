# S288 servo port — implementation plan

> **Status, 2026-09-15.** Phase 0 (facts), 1 (`bus_s288.rs`), 2 (the I²C IMU), 3 (the `robotd`
> alias), 4 (the constants) and 6 (metadata and docs) are done — **the software side is whole.**
> Phase 5 is done on the software side too, but its numbers are moved rather than tuned: every
> one of them is *known*-unverified, and the travel limits in particular are the training scene's
> rather than a measurement. Phase 2's reader exists, is wired into `robotd`, and has a bench
> probe (`robotd imu-probe`) — and has still never met a module. Nothing here has run against
> real hardware: the whole implementation has only ever seen a fake transport, and the per-unit
> zero offsets and signs every joint needs have not been measured on the one servo that exists.
> § Validation below is the honest to-do list. `JETSON.md` has the deltas; the git log has the
> arguments; [`s288-mechanical.md`](s288-mechanical.md) has the mounting geometry that the
> bracket is waiting on.

Replacing the 15 Dynamixel XL330s with Unitree S288s. This is the servo row of the
Jetson-port status table (`docs/project/jetson-port.md`), planned in `JETSON.md` § "The seam
that already exists". This file is the working plan: what to write, what to edit, what to
re-measure, and how to know each step worked.

## Why this is mostly additive

The servo bus is behind `duck_control::io::RobotIo` (`duck-control/src/io.rs:113`), and
`robotd` is generic over it. The control loop, the safety arbiter, policy loading,
kinematics, the IPC protocol and every client stay untouched. The entire hardware delta is:

1. one new `RobotIo` impl speaking the S288 protocol,
2. one new IMU source (the IMU rides `Sensors`, so a new bus owns it),
3. a handful of constants and defaults that are XL330 measurements wearing neutral names.

Branch-model rules from `JETSON.md` apply throughout: new files over edited files, no
renames, and the only core patch should be the `BusIo` alias in `robotd`.

## Fact-finding first (phase 0)

Everything in phase 1+ is parameterised by facts about the S288 that must come from the
protocol documentation and a bench servo, not from this codebase. Source:
github.com/unitreerobotics/digital_servo — `specs/protocol.md`, `python/servo_demo.py`.
Fill this table before writing the codec; every row has code downstream of it.

| fact | XL330 value (current code) | S288 value | consumed by |
|---|---|---|---|
| baud rate | 1 Mbps (`model.rs:80`) | 6 Mbps planned | `open()` |
| factory ID / baud | 1 @ 57600 (`model.rs:85`) | ? | replacement adoption |
| position counts/rev | 4096 (`bus.rs:480`) | ? | `read`, `write` |
| position centre | 2048 = 0 rad, one turn | ? | `read`, `write` |
| velocity unit | 0.229 rpm/count (`bus.rs:34`) | ? | `Sensors.velocities` |
| current unit | 1 mA/count, i16 | ? | `Sensors.currents_ma` |
| voltage unit | 0.1 V/count @ reg 144 | ? | `slow_sensors` |
| temperature readout | reg 146, whole °C | ? | `slow_sensors` |
| torque on/off | reg 64 write | ? | `set_torque` |
| position P gain | reg 84, u16, ~unitless | ? | `set_gain` |
| reboot / error clear | Protocol 2 REBOOT | ? | `reboot` |
| broadcast/batch read | `sync_read` | ? | tick budget |
| actuator travel | ±π (`safety.rs:47`) | ? | range clamp |
| supply rail | 2S Li-ion, 6.6–8.2 V usable | 12 V | battery constants |

`JETSON.md` § Hardware already fixes two of these: 15 servos at IDs 0–14, and a 12 V rail.

## Phase 1 — `duck-control/src/bus_s288.rs` (new file)

A `RobotIo` impl, shaped like `bus.rs` but self-contained. Register in
`duck-control/src/lib.rs` as `pub mod bus_s288;`.

- **Serial + framing.** Open the port at 6 Mbps with the same 30 ms read-timeout discipline
  as `READ_TIMEOUT` (`bus.rs:52`). Implement the S288 frame: header, id, command, payload,
  CRC. The CRC belongs in unit tests against vectors computed from the Python demo, not in
  "it worked on the bench".
- **`read()`** — one pass over IDs 0–14 returning positions, velocities, currents in the
  `Sensors` layout (indexed as `JOINT_NAMES`, radians / rad·s⁻¹ / mA). If the protocol has
  no `sync_read` equivalent, fifteen point reads per tick at 6 Mbps must be *measured*
  against the 20 ms tick budget before this design is accepted — see validation below.
- **`write()`** — goal positions for all 15 joints. Position control only; alpha has no
  velocity-mode joints.
- **`set_torque(on)`** — every servo written whatever the others said, collecting failures
  (`bus.rs:357` is the shape; a half-locked robot on power-off was a real incident).
- **`set_gain(kp)`** — write the S288's position gain to every servo. The number arriving
  here is in XL330 register units today; see phase 5 before deciding what this impl does
  with it.
- **`slow_sensors()`** — mean supply voltage + per-joint case temperature, once a second.
- **`reboot(id)`** — whatever clears a latched fault on the S288; if nothing does, document
  that and return an error the caller can log, because `robotd`'s recovery path calls this.
- **`present_positions()` / `interpolate_to()`** — inherent (non-trait) methods, because
  `robotd init` calls them on the concrete type (`robotd/src/main.rs:1028`).
- **What does not port:** `check_registers` and `adopt_replacement` are Dynamixel EEPROM
  machinery (return-delay, baud switching, 57600→1M probing). Absorb the *idea* — a fresh
  servo must be assignable to a joint without a config tool — into whatever the S288's ID
  assignment looks like, or cut it explicitly and say so in `JETSON.md`. Do not leave a
  half-working adoption path.

The IMU lands in this same impl's `read()` (phase 2), keeping `Sensors` atomic.

## Phase 2 — IMU: LSM6DSV16X over I2C (new file)

Replaces the `imu_to_dxl` bus board and `SflpDecoder` (`duck-control/src/imu.rs`).

- New reader (own module, e.g. `duck-control/src/imu_lsm6.rs`), filling
  `io::Sensors.imu: ImuData` from the same `read()` as the joints.
- SFLP fusion runs in the chip, as on the old board, so `ImuData` keeps its shape:
  quaternion, projected gravity, gyro. The limp-fall predictor consumes the gyro
  (`robotd.toml` § limp_fall); a quaternion-only reader silently breaks it.
- Preserve the convergence contract: `imu_ready()` is false until the filter has real
  samples. `safety.rs:210` gates fall detection on it, and the regression test
  `an_unconverged_imu_cannot_declare_a_fall` exists because skipping this once cost an
  afternoon.
- The stale-sample tracking (`StaleImuTracker`, `imu_stale()`) is bus-block-specific;
  decide whether the I2C path needs an equivalent (a frozen chip is the same failure) and
  implement it inside the new reader if so.

## Phase 3 — `robotd` integration (the one core patch)

`robotd/src/main.rs`:

- `type BusIo = duck_control::bus::DynamixelIo` (line 1153) → the new type. Per the fork's
  rules this is a platform choice; a cfg or a `--bus` flag is the upstream-friendly version
  (listed under "Candidate upstream contributions" in `JETSON.md`), a plain edit is the
  cheap one. Pick one and record it.
- `open_bus()` (line 1190): replace `check_registers` / `adopt_missing_servo` with the new
  impl's startup verification. Keep the *behaviour*: wait-and-retry on an unpowered bus,
  loud once then quiet (`open_bus_waiting`, line 1164), and publish
  `startup_bus_failures` so `robot.health` names the cause.
- `run_init` (line 1028) needs `present_positions` / `interpolate_to` on the new type.

## Phase 4 — `duck-control/src/model.rs` constants

The file is one robot's measurements; the new robot gets new numbers, with the comment
header updated to say where they were measured (that provenance is the file's whole point).

- `JOINT_IDS` → 0–14 in `JOINT_NAMES` order. The uniqueness and no-collision tests stay;
  the `imu_id_does_not_collide_with_a_joint` test goes with `IMU_DXL_ID`.
- `BAUD_RATE` → 6 Mbps. `FACTORY_ID` / `FACTORY_BAUD_RATE` → S288 factory defaults, or
  deleted with the adoption path.
- `EXPECTED_REGISTERS` → deleted; nothing on the S288 corresponds.
- `IMU_DXL_ID` → deleted; the IMU is no longer a bus device.
- **Battery constants are wrong for the new rail and must be re-measured, not translated.**
  `BATTERY_FULL_V = 8.2` / `BATTERY_EMPTY_V = 6.6` are usable-under-load numbers for a 2S
  pack read through XL330s. The 12 V rail needs its own pair, derived the same way the
  originals were (run a robot flat, watch where it starts struggling — `model.rs:107`
  explains the method). Consumers: `battery_percent` (robotd health at
  `robotd/src/main.rs:874`, `robotctl monitor`), and `battery_empty_shutdown`
  (`robotd/src/main.rs:2324`). A wrong empty floor either shuts a healthy robot down or
  never shuts a dying one down.
- `DEFAULT_POSITION`, `MOUTH_*`, `NUM_JOINTS`, `JOINT_NAMES` are mechanical, not
  electrical: unchanged **if** the S288s mount with the same zero and direction as the
  XL330s. Verify per joint on the bench (command 0, check the horn); a sign flip here is
  invisible to every test and makes the robot stand crooked.

## Phase 5 — gains, limits, parameters

- `safety.rs` `ACTUATOR_MIN/MAX`: **done on the software side, not on the measurement side.**
  That clamp used to be ±π for every joint; it is now `model::JOINT_RANGE`, the MJCF's own
  `[lo, hi]` per joint. The ±π pair stays as the outer bound those ranges are checked against —
  a test, so a joint needing more travel than the servo has fails with its name on it rather than
  at 50 Hz on a bench. What is still open: **none of it has been measured**, and an S288 has no
  firmware travel limit at all, so nothing between a policy and a mechanical stop knows where
  that stop is. Run § Validation's hand-push procedure; if a stop comes back *narrower* than a
  range, the mechanism or the training scene is what to fix, not the constant.
- **Gain semantics.** `policy.gain = 200`, `gain_limp = 50`, `limp_fall_pose_gain = 160`,
  `standing_gain_ratio = 0.8` are XL330 register values. On the S288 the whole set needs
  re-tuning on hardware: limp gain low enough to yield to the floor, running gain stiff
  enough to walk. Then update: `SafetyConfig::default` (`safety.rs:68`), the registry text
  (`robotd-params/src/registry.rs` — `policy.gain`, `safety.gain_limp`,
  `safety.limp_fall_pose_gain`), and `deploy/robotd.toml` comments.
- `policy.voltage_adapt` assumes effective kP tracks supply voltage the way the XL330's
  does. Leave it `false` (the default) until that assumption is re-examined on the S288;
  update `nominal_voltage` to the 12 V rail regardless.
- `deploy/robotd.toml` `[bus] port`: the Jetson's UART device for the servo bus (the
  current `/dev/ttyS2` comment is the Radxa's wiring).

## Phase 6 — metadata and docs

- Policy manifest `"servos": "xl330"` → `"s288"`: the test fixture at
  `updater/src/policy.rs:665` and the spec table in `docs/policy-manifest.md:49`. The field
  is display-only today; if `hw_rev`/`servos` ever gate loading, that check lands here too.
- Status rows: `README.md` (servos/IMU rows "not ported" → done), `JETSON.md`
  (§ Delta map "not written yet" entries), `docs/project/jetson-port.md` table.
- `docs/design/robotd-design.md:271-276` describes the XL330 register pinning and the
  servo-swap adoption as design facts; annotate or update so the design doc stops
  describing hardware that is no longer fitted.

## Validation

Unit (no hardware):
- Codec round-trips: frame encode/decode, CRC against known vectors, position and velocity
  conversions against the phase-0 table. Mirror `bus.rs`'s conversion tests
  (`position_conversion_round_trips_through_rustypot`, `velocity_scale_matches_the_datasheet_figure`).
- `cargo test -p duck-control -p robotd` stays green; `model.rs`'s table tests catch ID
  collisions automatically once `JOINT_IDS` changes.

Bench (one servo, then a full bus):
- Ping all 15 IDs; command and read back a known angle per joint; confirm zero/direction
  against `DEFAULT_POSITION` (phase 4).
- Torque off → robot limp; torque on → holds. `set_gain` sweep to find the new
  limp/running pair before touching the policy.
- Tick budget: measure a full `read()`+`write()` cycle time at 6 Mbps. The 50 Hz loop
  affords 20 ms; `robotd`'s own five-minute loop summary and `min_achieved_hz` are the
  in-situ check.

Integration:
- `robotd --fake` unchanged; real boot: `open_bus_waiting` retries cleanly with servo power
  off, comes up when power lands.
- `robotctl monitor`: battery %, per-joint temps, IMU gravity, fall verdict.
- Fall path: `imu_ready` gating, then limp-fall end-to-end on a held robot before a free
  one.
- Only then: enable the walking policy. Expect to re-tune `action_scale` and gains — the
  networks were trained against XL330 dynamics, and degraded transfer shows up as a robot
  that stands but walks badly.

## Risks and open questions

- **Policy transfer** is the largest unknown. Obs/action shapes are unchanged, but
  torque/velocity/gain response is not; the actuator model lives in the training side
  (mjlab), outside this repo. Budget for a retrain if re-tuning scales and gains is not
  enough.
- **Mouth servo.** `MOUTH_INDEX` is skipped by every policy but driven by sounds/theremin.
  Confirm the mouth is also an S288 and its −5°..+30° travel still makes sense mechanically.
- **No-sync_read bus.** If the S288 protocol cannot batch reads, 15 point reads + I2C IMU
  inside 20 ms at 50 Hz is the feasibility question of the whole port. Answer it in phase 1,
  not phase 5.
- **Replacement adoption** may have no S288 equivalent. Cutting it is acceptable; pretending
  it works is not.
- **The bracket, which is half answered.** Unitree's catalogue gives an outline and nothing else,
  so a drawing built on it could not place a single screw — that was the largest mechanical
  unknown in this port. The STEP model support supplied on 2026-09-14 closes the mounting half
  (two 6 × Ø1.7 bolt circles, 15.6 mm apart) and leaves the rest: no output shaft, no thread
  spec, no solid to check clearance against. Horn geometry is what the leg links key off, so the
  remaining gap is not small. [`s288-mechanical.md`](s288-mechanical.md) is what settled and what
  did not.
