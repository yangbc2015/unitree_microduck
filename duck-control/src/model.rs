//! The robot, as data.
//!
//! One variant — **alpha** — because that is the only robot that exists. Every shipped
//! policy is `alpha_*`; v1/v1.5/v1.6 are history. A second revision becomes a second set
//! of tables, which is honest until there is a second robot to generalise from.
//!
//! The numeric values here are lifted from `microduck_runtime`'s `motor.rs`, where they
//! were measured against hardware rather than derived. Re-deriving them from a datasheet
//! is exactly the kind of change that looks right and walks wrong.
//!
//! What lives here is what survives a change of servo: how many joints there are, what they
//! are called, where home is, how far the mouth opens, and how flat the pack is. Everything
//! that is a property of a particular *bus* — the addresses, the baud rate, the EEPROM
//! registers, where the IMU is read — lives with the code that speaks it ([`crate::bus`] for
//! the XL330 chain, [`crate::bus_s288`] for the S288). That split is what lets both
//! implementations compile side by side, and it is why swapping fifteen servos for fifteen
//! others changed so little of this file.

/// Left leg (5) · neck/head/mouth (5) · right leg (5).
pub const NUM_JOINTS: usize = 15;

/// Joint names, from the protocol crate — the wire indexes `joints` and `targets`
/// positionally, so that order and this one cannot be allowed to drift apart. The
/// assertion below is what makes "cannot" true.
pub use duck_ipc_proto::JOINT_NAMES;

const _: () = assert!(JOINT_NAMES.len() == NUM_JOINTS);

/// The mouth is absent from every alpha policy — they are all 61-D observation, 14-action,
/// and the action vector skips this index. Named so that omission is deliberate rather
/// than an off-by-one someone has to rediscover.
pub const MOUTH_INDEX: usize = 9;

/// Home pose. The trunk sits ~5 mm further forward than the v1.5 pose so the CoM is over
/// the ankle axis; the old pose biased the robot backwards.
///
/// Must match `HOME_FRAME` in the training env — a policy is trained against these angles
/// and observes joint positions *relative* to them, so a discrepancy here is a constant
/// offset on 14 observation slots.
///
/// The servo swap does not change these *angles*, but it does put a new question in front of
/// them: they are joint-side radians, and each S288's output zero is wherever its horn happens
/// to have been pressed on. That offset is per unit and lives in `bus_s288::Config`, measured
/// on the bench — not here, because it is not a property of the robot model.
pub const DEFAULT_POSITION: [f64; NUM_JOINTS] = [
    0.0,     // left_hip_yaw
    -0.0873, // left_hip_roll
    -0.4579, // left_hip_pitch
    -0.0049, // left_knee
    0.4530,  // left_ankle
    0.3491,  // neck_pitch
    0.3491,  // head_pitch
    0.0,     // head_yaw
    0.0,     // head_roll
    0.0,     // mouth
    0.0,     // right_hip_yaw
    0.0873,  // right_hip_roll
    0.4579,  // right_hip_pitch
    0.0049,  // right_knee
    -0.4530, // right_ankle
];

/// Mouth travel, radians: closed and fully open. The alpha reuses the v1.6 range,
/// −5°..+30°, from `microduck_runtime`'s `variant.rs`.
///
/// The mouth is not part of any policy — every alpha network is 14 actions with this joint
/// skipped — so these two numbers and [`mouth_target`] are the whole of mouth control.
///
/// It is also, per `s288-servo-port.md`, the one joint whose mechanical sense under the new
/// servo nobody has confirmed: whether the S288 can reach −5°..+30° from its own zero, and
/// which way round, is a bench check rather than a computation.
pub const MOUTH_CLOSED: f64 = -5.0 * std::f64::consts::PI / 180.0;
pub const MOUTH_OPEN: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// Joint angle for a mouth opening fraction. 0 is closed, 1 is fully open; anything outside
/// is clamped rather than fed to a servo as an out-of-travel target.
pub fn mouth_target(open: f64) -> f64 {
    let open = if open.is_finite() {
        open.clamp(0.0, 1.0)
    } else {
        0.0
    };
    MOUTH_CLOSED + open * (MOUTH_OPEN - MOUTH_CLOSED)
}

/// Index of a joint by name. Linear scan over 15 entries, used at startup and in tests.
pub fn joint_index(name: &str) -> Option<usize> {
    JOINT_NAMES.iter().position(|n| *n == name)
}

/// Per-joint travel, radians, indexed as [`JOINT_NAMES`] — the same `[lo, hi]` the MJCF
/// declares, which is the range every policy was trained against.
///
/// Lifted from `kinematics/assets/alpha/robot_walk.xml`. `kinematics` already parses that file
/// and exposes `Model::joint_range`, so this is deliberately a *copy*: the crate that drives
/// motors should not have to link forward kinematics to find out where a joint stops. A test
/// compares the two tables by name, so the copy cannot drift quietly.
///
/// **These are the training scene's limits, not a measurement of this robot.** They bound what
/// a policy may ask for; they say nothing about where the metal stops. On the XL330 that
/// distinction was nearly academic — a one-turn position mode, with the servo holding a range
/// of its own. On the S288 it is not: that servo has **no firmware travel limit at all**
/// (multi-turn absolute encoder, turn count reset by every power cycle), so if a range here is
/// wider than the mechanism, nothing downstream will catch it. `docs/project/s288-servo-port.md`
/// § phase 5 has the bench procedure — power the bus, hold torque off so the shaft is free,
/// push the joint to each stop by hand, read the single-turn absolute output encoder. If a
/// measured stop is *narrower* than a range below, the mechanism is the thing to fix, or the
/// training scene: clamping a policy inside the range it was trained on is not a fix, it is a
/// policy asked to walk with one leg shorter than the simulator's.
///
/// The mouth is the one joint the MJCF does not carry — no policy drives it — so its entry is
/// the mouth's own travel rather than a joint limit.
pub const JOINT_RANGE: [(f64, f64); NUM_JOINTS] = [
    (-0.4363323129985824, 0.5235987755982988),  // left_hip_yaw
    (-0.3839724354387516, 0.38397243543875337), // left_hip_roll
    (-1.5707963267949037, 1.5707963267948895),  // left_hip_pitch
    (-1.570796326794901, 1.5707963267948921),   // left_knee
    (-1.5707963267949019, 1.5707963267948912),  // left_ankle
    (-1.5707963267948966, 1.0471975511965976),  // neck_pitch
    (-1.5707963267948966, 1.5707963267948966),  // head_pitch
    (-2.967059728390364, 2.967059728390357),    // head_yaw
    (-0.4363323129986037, 0.43633231299856107), // head_roll
    (MOUTH_CLOSED, MOUTH_OPEN),                 // mouth
    (-0.5235987755982988, 0.4363323129985824),  // right_hip_yaw
    (-0.3839724354387525, 0.3839724354387525),  // right_hip_roll
    (-1.5707963267949, 1.570796326794893),      // right_hip_pitch
    (-1.570796326794901, 1.5707963267948921),   // right_knee
    (-1.5707963267949028, 1.5707963267948903),  // right_ankle
];

// ── battery ──────────────────────────────────────────────────────────────────
//
// There is no fuel gauge and no ADC. The only measurement available is what the servos
// report as their own supply (`crate::bus::DynamixelIo::slow_sensors`, or
// `crate::bus_s288::BusS288::slow_sensors`), which is the pack seen through the bus — so it
// sags under load and recovers when the robot stands still. That is why the span below is
// *usable-under-load*, not the cell chemistry's range.
//
// **These two numbers changed with the servo swap and have not been measured on a robot
// yet.** The XL330 duck ran a 2S NP-F550; the S288 duck runs a 3S pack on its 12 V rail, and
// the pair below is the 3S chemistry's usable span rather than a run-flat measurement. The
// method that produced the old pair — `microduck_runtime`'s `check_battery`: run a duck flat
// and watch where it starts struggling — is the method that should replace these. Until
// someone does that, treat them as an envelope, not a measurement: a wrong empty floor either
// shuts a healthy robot down or never shuts a dying one down.
//
// One more thing the S288 makes explicit and the XL330 did not: its frame reports supply as
// **0.5 V per count** (see `bus_s288`'s feedback parser), so every reading is quantised to
// half a volt. Across the 2.7 V span below that is five or six distinct percentages, and no
// arithmetic in this file will add resolution the bus does not carry. `battery_percent` is
// therefore a coarse indicator on this hardware; anything finer needs an ADC the duck does
// not have.

/// Off a full charge, under load. 3S Li-ion on the 12 V rail.
pub const BATTERY_FULL_V: f64 = 12.6;

/// The sag floor: below this the robot starts struggling, well before the pack's own
/// protection trips. Empty for our purposes, not empty for the cells'.
pub const BATTERY_EMPTY_V: f64 = 9.9;

/// Fraction of a pack, 0–100, for a bus voltage.
///
/// Linear, and the numbers come from `microduck_runtime`'s `check_battery`, where they were
/// arrived at by running a duck flat. It lives here rather than in `robotd` so there is one
/// mapping: the prototype had it in a CLI *and* re-derived in the app, which is how two
/// screens end up disagreeing about the same pack.
///
/// A non-finite or non-positive reading is 0 — those mean "no answer from the bus", and the
/// caller is expected to report that as unknown rather than to display this number.
pub fn battery_percent(volts: f64) -> f64 {
    if !volts.is_finite() || volts <= 0.0 {
        return 0.0;
    }
    ((volts - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V)).clamp(0.0, 1.0) * 100.0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The tables are indexed by the same integer everywhere in the crate. If they ever
    /// diverge in length, every lookup silently reads the wrong joint. (`JOINT_IDS` is not in
    /// this list any more: it is a bus's table now, and `bus` asserts it against this one.)
    #[test]
    fn tables_agree_on_length() {
        assert_eq!(JOINT_NAMES.len(), NUM_JOINTS);
        assert_eq!(DEFAULT_POSITION.len(), NUM_JOINTS);
        assert_eq!(JOINT_RANGE.len(), NUM_JOINTS);
    }

    /// Every entry must be a real interval, because `f64::clamp` **panics** when `min > max` —
    /// and these are indexed by joint inside the control tick. A transposed pair here would take
    /// the loop down rather than clamp anything, which is the opposite of what a limit is for.
    #[test]
    fn every_joint_range_is_an_interval() {
        for (joint, &(lo, hi)) in JOINT_RANGE.iter().enumerate() {
            let name = JOINT_NAMES[joint];
            assert!(lo.is_finite() && hi.is_finite(), "{name}: {lo}..{hi}");
            assert!(lo <= hi, "{name}: {lo} > {hi}");
            assert!(lo < 0.0 && hi > 0.0, "{name} cannot reach its own zero");
        }
    }

    /// `JOINT_RANGE` is a transcription, and this is what keeps it one. `kinematics` parses the
    /// same XML this table was copied from, so the two are compared joint by joint — matching by
    /// *name*, since the MJCF's order and [`JOINT_NAMES`] order are not the same list (the mouth
    /// is absent from the tree).
    ///
    /// This is the whole reason the copy is allowed to exist: without it, a mechanical revision
    /// would update the XML and leave the control loop clamping against last season's robot.
    #[test]
    fn the_joint_ranges_are_the_ones_the_mjcf_declares() {
        let model = kinematics::Model::alpha();
        for (joint, name) in JOINT_NAMES.iter().enumerate() {
            if *name == "mouth" {
                // No policy drives the mouth, so the MJCF has no joint for it and there is
                // nothing to compare against — its entry is the mouth's own travel.
                assert!(
                    model.joint_index(name).is_none(),
                    "the MJCF grew a mouth this table does not bound"
                );
                continue;
            }
            let tree = model
                .joint_index(name)
                .unwrap_or_else(|| panic!("alpha MJCF lost joint {name:?}"));
            let declared = model
                .joint_range(tree)
                .unwrap_or_else(|| panic!("alpha MJCF declares no range for {name:?}"));
            let ours = JOINT_RANGE[joint];
            assert!(
                (ours.0 - declared.0).abs() < 1e-12 && (ours.1 - declared.1).abs() < 1e-12,
                "{name}: table says {ours:?}, MJCF says {declared:?}"
            );
        }
    }

    /// `MOUTH_INDEX` is used to skip a slot when mapping 14 policy actions onto 15 joints.
    /// Pointing it at the wrong joint would shift every action after it by one.
    #[test]
    fn mouth_index_names_the_mouth() {
        assert_eq!(JOINT_NAMES[MOUTH_INDEX], "mouth");
        assert_eq!(joint_index("mouth"), Some(MOUTH_INDEX));
    }

    /// The ends of the span and the middle of it. Getting the direction wrong here would
    /// report a full pack as flat, which is the kind of thing nobody double-checks.
    #[test]
    fn battery_percent_spans_the_usable_range() {
        assert_eq!(battery_percent(BATTERY_FULL_V), 100.0);
        assert_eq!(battery_percent(BATTERY_EMPTY_V), 0.0);
        assert!((battery_percent(11.25) - 50.0).abs() < 0.001);
    }

    /// Voltages outside the span are ordinary — a fresh 3S pack reads over 12.6 V off the
    /// charger, and a robot being run into the ground reads under 9.9 V. Neither may produce
    /// a percentage outside 0–100 for a caller to display.
    #[test]
    fn battery_percent_clamps_rather_than_extrapolating() {
        assert_eq!(battery_percent(13.0), 100.0);
        assert_eq!(battery_percent(8.0), 0.0);
    }

    /// The S288 reports supply in half-volt counts, so a frame carrying 25 of them is 12.5 V —
    /// one count short of the full-charge figure, and it must read as a nearly-full pack rather
    /// than as something over 100%. This is the quantisation the battery comment describes,
    /// pinned so the span and the wire's resolution stay in a sane relationship if either is
    /// edited: the whole span is under six counts wide, which is what "coarse indicator" means
    /// in practice.
    #[test]
    fn a_half_volt_count_lands_inside_the_span() {
        assert!((battery_percent(12.5) - 96.296_296_296_296_32).abs() < 1e-9);
        assert_eq!(battery_percent(10.0), battery_percent(9.9 + 0.1));
        assert!(battery_percent(10.0) > 0.0 && battery_percent(10.0) < 100.0);

        let counts_over_the_span = (BATTERY_FULL_V - BATTERY_EMPTY_V) / 0.5;
        assert!(
            counts_over_the_span < 6.0,
            "the S288's 0.5 V/count leaves only {counts_over_the_span} steps over the span"
        );
    }

    /// A bus that did not answer arrives here as 0.0, and NaN is what a mean over an empty
    /// set produces. Both must be 0 rather than a wild number a caller might print.
    #[test]
    fn battery_percent_treats_no_reading_as_zero() {
        assert_eq!(battery_percent(0.0), 0.0);
        assert_eq!(battery_percent(f64::NAN), 0.0);
        assert_eq!(battery_percent(-1.0), 0.0);
    }

    /// The mouth range is the prototype's: −5° closed, +30° open. A fraction outside 0..1
    /// (or a NaN from a broken client) must clamp rather than command a servo past travel.
    #[test]
    fn mouth_target_spans_the_prototype_range() {
        assert!((mouth_target(0.0) - (-5.0f64.to_radians())).abs() < 1e-12);
        assert!((mouth_target(1.0) - 30.0f64.to_radians()).abs() < 1e-12);
        assert_eq!(mouth_target(-3.0), mouth_target(0.0));
        assert_eq!(mouth_target(7.0), mouth_target(1.0));
        assert_eq!(mouth_target(f64::NAN), mouth_target(0.0));
    }

    /// The legs are mirrored: the roll/pitch/ankle pairs are equal and opposite. A sign
    /// typo in the home pose is invisible by inspection and makes the robot stand crooked.
    #[test]
    fn home_pose_legs_are_mirrored() {
        for (left, right) in [
            ("left_hip_roll", "right_hip_roll"),
            ("left_hip_pitch", "right_hip_pitch"),
            ("left_knee", "right_knee"),
            ("left_ankle", "right_ankle"),
        ] {
            let l = DEFAULT_POSITION[joint_index(left).unwrap()];
            let r = DEFAULT_POSITION[joint_index(right).unwrap()];
            assert!(
                (l + r).abs() < 1e-9,
                "{left} ({l}) and {right} ({r}) should be equal and opposite"
            );
        }
    }
}
