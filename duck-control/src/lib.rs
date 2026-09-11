//! The robot control core: everything between reading the bus and writing it.
//!
//! Deliberately not a daemon. There is no tokio here, no socket, no systemd — `robotd`
//! owns all of that. The boundary is enforced by the compiler rather than by discipline,
//! which is what stops process concerns leaking into the code that drives motors.
//!
//! The control path it holds — model, bus, [`io::RobotIo`], observations, policy, safety — is
//! designed in `docs/design/robotd-design.md` §2.

pub mod bus;
/// The Unitree S288 bus — a second `RobotIo` for a servo that shares no protocol with the
/// Dynamixel one above. Both compile everywhere; `robotd`'s `BusIo` alias chooses which one
/// this robot runs (see `JETSON.md`). Nothing else in the crate knows which is in use.
pub mod bus_s288;
pub mod fall;
pub mod imu;
pub mod io;
pub mod model;
pub mod obs;
pub mod policy;
pub mod safety;
/// A robot in MuJoCo, over TCP — the backend `robotd-design.md` §9 deferred.
pub mod sim;

pub use imu::ImuData;
pub use io::{FakeIo, IoError, JointTargets, RobotIo, Sensors, SlowSensors};
pub use model::{
    BATTERY_EMPTY_V, BATTERY_FULL_V, DEFAULT_POSITION, JOINT_IDS, JOINT_NAMES, NUM_JOINTS,
    battery_percent,
};
pub use obs::{ACTION_LEN, Command, OBS_LEN, Observation};
