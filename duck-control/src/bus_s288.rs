//! The Unitree S288 bus.
//!
//! A second `RobotIo` implementation, for a servo that shares almost nothing with the
//! Dynamixel one in `bus`.
//!
//! ## Where the truth is
//!
//! Not in the upstream spec. `specs/protocol.md` and `python/servo_demo.py` are wrong in three
//! places that all change what a correct implementation does, and each is reproduced against
//! real frames in `unitree_servo/verify_official_issues.py`:
//!
//! - the feedback `vol` field is typed `uint16`; it is `uint8`, and the table then sums to 20
//!   bytes against the 19-byte `fbk` the same document declares — every field after `vol` is
//!   off by one in the spec
//! - `servo_demo.py` packs `pos_des` unsigned, so no negative angle can be commanded
//! - neither documents that the CRC walks 4-byte little-endian words MSB-first rather than the
//!   byte stream, so no standard CRC library reproduces it
//!
//! The reference implementation is therefore `unitree_servo/unitree_servo.py` in this repo — a
//! working host-side implementation checked against frames off the wire — and the two test
//! vectors in this file come from that folder's archived run.
//!
//! ## What the trait assumes that this bus does not do
//!
//! Each of these is a decision, not an oversight:
//!
//! - **One `sync_read`/`sync_write` per tick.** Here there is no sync instruction at all: one
//!   control frame goes to one servo at a fixed ID, and its 26-byte reply *is* that servo's
//!   state. So one tick is one exchange per joint, and the reply is collected on the way past.
//!   That is why [`RobotIo::read`] and [`RobotIo::write`] are not two bus operations here.
//! - **The servos must be spoken to continuously.** The firmware drops torque after a 1 s
//!   silence (the timeout protection, measured working on the bench). A bus that answered
//!   reads from a cache and only transmitted on `write` would drop a standing robot's torque
//!   whenever the loop had nothing new to say. See [`KEEPALIVE`].
//! - **`reboot` is the Protocol 2 REBOOT instruction.** The S288 has no reboot command and no
//!   instruction set in common with Dynamixel. What it has is a latched `MError` that the
//!   control bits clear, so [`RobotIo::reboot`] is an attempt to clear that and it reports
//!   honestly when the servo still refuses — an encoder-class fault cannot be cleared in
//!   software at all, and the caller needs to know that rather than be told "rebooted".
//! - **`slow_sensors` costs a transaction.** Every S288 frame already carries supply voltage
//!   and two temperatures, so by the time it is called the values have arrived for free with
//!   the tick. The cadence the trait documents is still fine; it simply costs nothing.
//! - **`currents_ma`.** The S288 has no current telemetry anywhere in its frame. It reports
//!   rotor torque (256000 counts = 1 N·m), and the conversion to amps needs a torque constant
//!   nobody has measured — the manual's own stall example implies ~1e-3 N·m/A, which is an
//!   order of magnitude below a typical small BLDC and exactly the kind of number that turns
//!   into a fabricated measurement if it is printed as one. So this reports 0.0 and says why,
//!   rather than dividing a torque by an assumed constant. Nothing in `robotd` consumes the
//!   field today; the load signal the safety layer wants is torque, and it is already carried.
//! - **Per-joint zero offsets live in the servo's EEPROM.** The S288 has no user EEPROM: the
//!   offset between its output encoder and the joint's mechanical zero has nowhere to live but
//!   [`Config`], because the servo forgets. The offset and the sign are per unit and per
//!   assembly, and `unitree_servo/` has the bench tools that measure both.
//! - **The IMU is not on this bus.** `Sensors` carries joints and IMU together because
//!   upstream's IMU board shares the servo bus; the port's LSM6DSV16X is on I2C and its reader
//!   does not exist yet. This impl takes an [`ImuSource`], so the reader drops in behind it
//!   later; until then [`NoImu`] reports the zero sample with `ready() == false`, which is
//!   exactly the signal upstream uses for a filter that has not converged yet. A loop keeps
//!   running, and nothing pretends to know which way is down.
//!
//! ## Bus capacity, and why retries are not optional
//!
//! The S288 accepts IDs 0-14, with 15 as broadcast (the manual says 15 servos; upstream's spec
//! says 16 — another B-class inconsistency, and the more conservative number is the one to
//! build against). [`crate::model::NUM_JOINTS`] is 15, so a duck uses *every* address on the
//! bus, with nothing spare. The manual's own troubleshooting table lists "14 servos in a chain
//! lose packets — cable too long or too thin" as expected behaviour, so dropped frames are not
//! a hypothetical at this chain length: [`Config::retries`] exists, and drops are counted in
//! [`BusS288::health`] so that "the cable is marginal" is visible before it is a fall.

use std::time::{Duration, Instant};

use crate::imu::ImuData;
use crate::io::{IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::NUM_JOINTS;

/// 6 Mbit/s, fixed by the firmware. (Upstream's `specs/protocol.md` has this one right.)
pub const BAUD: u32 = 6_000_000;

/// Output turns per rotor turn, from the manual: 288.35 = 70070/243.
///
/// Kept as the fraction rather than 288.35 so that a round trip through it is exact to the
/// last bit, and so a reader can see it is the gearbox's real ratio and not a rounded spec.
pub const RATIO: f64 = 70070.0 / 243.0;

const TWO_PI: f64 = std::f64::consts::TAU;

/// Rotor torque counts per N·m, from the manual.
const TORQUE_COUNTS_PER_NM: f64 = 256_000.0;
/// Rotor position counts per rotor radian: 32768/2π.
const POS_COUNTS_PER_ROTOR_RAD: f64 = 32768.0 / TWO_PI;
/// Rotor speed counts per rotor rad/s: 2.56/2π. Coarse (0.0085 output rad/s per count at
/// 288.35:1) — the position channel is four orders finer, so anything needing velocity should
/// differentiate position, which is what `unitree_servo/` does everywhere.
const SPD_COUNTS_PER_ROTOR_RAD_S: f64 = 2.56 / TWO_PI;

/// Frame sizes: 20 bytes out, 26 bytes back. Both are asserted on every exchange.
const CMD_LEN: usize = 20;
const FBK_LEN: usize = 26;

/// What the CRC covers, in each direction — a range the two packets do *not* share, which is
/// easy to get wrong in either direction and is asserted in the tests below.
const CMD_CRC_RANGE: std::ops::Range<usize> = 0..16;
const FBK_CRC_RANGE: std::ops::Range<usize> = 2..22;

/// The firmware's own timeout protection: 1 s without a frame and it drops torque.
pub const FIRMWARE_TIMEOUT: Duration = Duration::from_secs(1);

/// How stale a sample may be before [`RobotIo::read`] spends a bus pass on a fresh one.
///
/// A pass costs 15 × ~0.17 ms ≈ 2.5 ms, 12% of a 50 Hz tick — affordable once, not twice. A
/// loop that calls `read` then `write` every tick therefore pays one pass, not two, because the
/// `write` in the same tick refreshes everything the next `read` would have asked for. The
/// margin under [`FIRMWARE_TIMEOUT`] is deliberate: three misses of this interval still leave
/// torque on.
pub const KEEPALIVE: Duration = Duration::from_millis(300);

// --------------------------------------------------------------------------- CRC32

/// The polynomial, as the spec gives it.
const CRC32_POLY: u32 = 0x04C1_1DB7;

/// The CRC the firmware actually computes.
///
/// CRC-32/MPEG-2 (init all-ones, no reflection, no final xor) over the校验区 — but not over it
/// as a byte stream: the region is walked as 4-byte little-endian words, and each word is fed
/// most-significant byte first. Feeding the same bytes in memory order gives a different
/// answer, which is the trap: `unitree_servo/docs/verify_official_issues_output.txt` records
/// both values for both real frames, and the tests below pin them down.
///
/// `data.len()` must be a multiple of 4 — both CRC regions are, and a shorter one would
/// silently skip a trailing partial word.
fn crc32(data: &[u8]) -> u32 {
    debug_assert!(data.len() % 4 == 0, "CRC region must be whole 32-bit words");
    let mut crc: u32 = 0xFFFF_FFFF;
    for word in data.chunks_exact(4) {
        // Little-endian word, its bytes highest-index first.
        for byte in [word[3], word[2], word[1], word[0]] {
            crc ^= (byte as u32) << 24;
            for _ in 0..8 {
                crc = if crc & 0x8000_0000 != 0 {
                    (crc << 1) ^ CRC32_POLY
                } else {
                    crc << 1
                };
            }
        }
    }
    crc
}

/// The same bytes in memory order — kept only so the tests can assert what the wrong answer
/// is. A standard CRC-32 library over this region produces exactly this, and it is wrong.
///
/// Not `#[cfg(test)]`: it is part of how the module documents the trap, and a future reader
/// porting a "cleaned up" version needs to find the negative result next to the positive one.
#[allow(dead_code)]
fn crc32_byte_order(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= (byte as u32) << 24;
        for _ in 0..8 {
            crc = if crc & 0x8000_0000 != 0 {
                (crc << 1) ^ CRC32_POLY
            } else {
                crc << 1
            };
        }
    }
    crc
}

// --------------------------------------------------------------------------- frames

/// One control frame, built from joint-side (output) units.
///
/// `pos_rad`, `spd_rad_s` and `torque_nm` are all at the **output** shaft — the same convention
/// as every other number in `duck-control`. The protocol is entirely rotor-side, so each is
/// converted here. (The manual states that all protocol quantities are rotor-side; the upstream
/// Python example gets this right and the upstream *spec* never says it.)
///
/// `mode` 1 is closed-loop FOC, mode 0 stops the servo; there is nothing else. `timeout`
/// enables the firmware's 1 s protection and should stay on: this side already guarantees a
/// frame every [`KEEPALIVE`], and losing the protection would mean a dead host leaves the
/// servos driving at the last command.
#[allow(clippy::too_many_arguments)]
fn build_control(
    id: u8,
    mode: u8,
    timeout: bool,
    torque_nm: f64,
    spd_rad_s: f64,
    pos_rad: f64,
    kp: i16,
    kd: i16,
) -> [u8; CMD_LEN] {
    let mode_byte = (id & 0x0F) | ((mode & 0x07) << 4) | ((timeout as u8) << 7);

    let tor = clamp_i16((torque_nm / RATIO * TORQUE_COUNTS_PER_NM).round());
    let spd = clamp_i16((spd_rad_s * RATIO * SPD_COUNTS_PER_ROTOR_RAD_S).round());
    let pos = clamp_i32((pos_rad * RATIO * POS_COUNTS_PER_ROTOR_RAD).round());

    let mut pkt = [0u8; CMD_LEN];
    pkt[0] = 0xFE;
    pkt[1] = 0xEE;
    pkt[2] = mode_byte;
    pkt[3] = 0x00;
    pkt[4..6].copy_from_slice(&tor.to_le_bytes());
    pkt[6..8].copy_from_slice(&spd.to_le_bytes());
    // Signed, and this is the field upstream's example cannot express at all: it packs `pos_des`
    // with `struct.pack('<I', ...)`, which raises on any negative target, so half of every
    // joint's range is unreachable through the official example.
    pkt[8..12].copy_from_slice(&pos.to_le_bytes());
    pkt[12..14].copy_from_slice(&kp.to_le_bytes());
    pkt[14..16].copy_from_slice(&kd.to_le_bytes());
    let crc = crc32(&pkt[CMD_CRC_RANGE]);
    pkt[16..20].copy_from_slice(&crc.to_le_bytes());
    pkt
}

/// One parsed feedback frame.
///
/// Every field the frame carries is kept, not only the ones `RobotIo` consumes: `mode`,
/// `timed_out` and `ex_flag` are what a bench session needs to tell "the servo stopped
/// answering" from "the servo answered that it is in stop", and `out_pos_rad` is the field the
/// manual says to read after a power cycle. `currents_ma` is absence is documented at the top.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(dead_code)]
struct Feedback {
    id: u8,
    mode: u8,
    timed_out: bool,
    /// Case temperature, °C. (The byte upstream calls `sensor` tracks winding/MOS and runs
    /// ~9 °C cooler than this one on our frames; the trait's `temps_c` means the case.)
    temp_c: i8,
    /// Supply voltage, V. One count is 0.5 V, which is why a 12.0 V bench supply reads 24.
    volts: f64,
    /// Rotor torque, N·m. Sign is kept: it is the sign that says which way the servo is pulling.
    torque_rotor_nm: f64,
    spd_rotor_rad_s: f64,
    pos_rotor_rad: f64,
    merror: u32,
    /// The 13-bit single-turn output encoder. Absolute, and the one to read after a power
    /// cycle: the rotor position is multi-turn and its turn count is what resets.
    out_pos_rad: f64,
    /// ExFlag, the three warning bits.
    ex_flag: u16,
}

/// Parse a 26-byte reply. `None` if it is not one: wrong length, wrong header, or a CRC that
/// does not match — all three are "this is not a frame from the servo I asked", and the caller
/// retries rather than guessing which.
fn parse_feedback(buf: &[u8]) -> Option<Feedback> {
    if buf.len() != FBK_LEN || buf[0] != 0xFC || buf[1] != 0xEE {
        return None;
    }
    let body = &buf[FBK_CRC_RANGE];
    let want = u32::from_le_bytes([buf[22], buf[23], buf[24], buf[25]]);
    if crc32(body) != want {
        return None;
    }

    let mode_byte = buf[2];
    let id = mode_byte & 0x0F;
    let mode = (mode_byte >> 4) & 0x07;
    let timed_out = (mode_byte >> 7) & 0x01 == 1;

    // Offsets are from the wire, not from the spec's table — see the module docs for why the
    // two disagree from `vol` onward.
    let temp_c = buf[3] as i8;
    let volts = buf[5] as f64 / 2.0;
    let torque_rotor = i16::from_le_bytes([buf[6], buf[7]]) as f64 / TORQUE_COUNTS_PER_NM;
    let spd_rotor = i16::from_le_bytes([buf[8], buf[9]]) as f64 / SPD_COUNTS_PER_ROTOR_RAD_S;
    let pos_rotor =
        i32::from_le_bytes([buf[10], buf[11], buf[12], buf[13]]) as f64 / POS_COUNTS_PER_ROTOR_RAD;
    let merror = u32::from_le_bytes([buf[14], buf[15], buf[16], buf[17]]);
    let out_ex = u16::from_le_bytes([buf[18], buf[19]]);

    Some(Feedback {
        id,
        mode,
        timed_out,
        temp_c,
        volts,
        torque_rotor_nm: torque_rotor,
        spd_rotor_rad_s: spd_rotor,
        pos_rotor_rad: pos_rotor,
        merror,
        // 13 bits of position, 3 of warnings, in one 16-bit word.
        out_pos_rad: (out_ex & 0x1FFF) as f64 * TWO_PI / 8192.0,
        ex_flag: out_ex >> 13,
    })
}

/// The "nothing measured yet" state.
///
/// `SlowSensors` deliberately has no `Default`: its fields mean *measured*, and a default of
/// zeros would be a reading nobody took — the same objection this module raises about
/// fabricating `currents_ma`. Before the first pass this bus has measured nothing, and
/// [`RobotIo::slow_sensors`] errors rather than handing this back as if it had.
fn no_slow() -> SlowSensors {
    SlowSensors {
        volts: 0.0,
        temps_c: [0.0; NUM_JOINTS],
    }
}

fn clamp_i16(v: f64) -> i16 {
    v.clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

fn clamp_i32(v: f64) -> i32 {
    v.clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

// --------------------------------------------------------------------------- transport

/// The byte-level seam, so the logic above can be tested without a servo anywhere near it.
pub trait Transport: Send {
    /// Send one control frame and return the reply raw. Exactly one frame goes out per call:
    /// the bus is half-duplex and every request has exactly one answer.
    fn exchange(&mut self, cmd: &[u8; CMD_LEN]) -> std::io::Result<[u8; FBK_LEN]>;

    /// Discard anything buffered. Called after a framing error, where the port is holding the
    /// tail of a frame nobody is going to parse.
    fn flush_input(&mut self);
}

/// The real thing: one serial port, opened by path.
///
/// Opened by path rather than enumerated, for the reason `duck-control`'s `Cargo.toml` gives
/// for turning off serialport's `libudev` feature: the board's cross-build has no pkg-config
/// sysroot for libudev, so `available_ports` cannot be part of this crate.
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
}

impl SerialTransport {
    pub fn open(path: &str, read_timeout: Duration) -> std::io::Result<Self> {
        let port = serialport::new(path, BAUD)
            .timeout(read_timeout)
            .open()?;
        Ok(Self { port })
    }
}

impl Transport for SerialTransport {
    fn exchange(&mut self, cmd: &[u8; CMD_LEN]) -> std::io::Result<[u8; FBK_LEN]> {
        self.port.write_all(cmd)?;
        self.port.flush()?;

        let mut buf = [0u8; FBK_LEN];
        let mut got = 0;
        while got < FBK_LEN {
            match self.port.read(&mut buf[got..]) {
                Ok(0) => break, // timeout, with whatever arrived so far
                Ok(n) => got += n,
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => break,
                Err(e) => return Err(e),
            }
        }
        if got != FBK_LEN {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                format!("short reply: {got} of {FBK_LEN} bytes"),
            ));
        }
        Ok(buf)
    }

    fn flush_input(&mut self) {
        let _ = self.port.clear(serialport::ClearBuffer::Input);
    }
}

// --------------------------------------------------------------------------- config

/// Everything about this robot that the servo cannot be asked about.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Bus address per joint, indexed as [`crate::model::JOINT_NAMES`].
    ///
    /// Must be 0-14 and all distinct: **15 is broadcast** (every servo acts, none answers), so a
    /// joint addressed 15 would look dead while every other joint twitched. The default takes
    /// the first 15 addresses for exactly that reason — the Dynamixel IDs in
    /// [`crate::model::JOINT_IDS`] (10-34) are not merely a different numbering, they are
    /// unreachable here. Assigning them is a Windows-上位机 job: the protocol has no documented
    /// set-ID command, which is worth its own question to Unitree.
    pub ids: [u8; NUM_JOINTS],
    /// Joint angle = `sign * (output_angle - offset_rad)`.
    ///
    /// Two per-unit numbers the servo cannot store. `offset_rad` is the output angle the servo
    /// reports when the joint is mechanically at its designed zero; `sign` is which way of the
    /// shaft is the joint's positive direction. Measure, do not guess: a wrong sign drives a
    /// joint into the chassis the first time a policy moves it.
    pub sign: [f64; NUM_JOINTS],
    pub offset_rad: [f64; NUM_JOINTS],
    /// Position gain, in the unit `unitree_servo/`'s bench used — **not** the firmware's own
    /// counts.
    ///
    /// 10-20 with `kd` 1 is what held ±0.2 rad steps without ringing on the bench; 10 is the
    /// default because it is the soft end of a range that was measured. [`KP_COUNTS`] turns it
    /// into what the frame carries, using the reference implementation's own conversion rather
    /// than a constant this file invented — an invented one would have been plausible and
    /// wrong, which is the failure mode the module header spends three paragraphs on.
    ///
    /// The trait's `kp` arrives as a `u16` in *Dynamixel* units, where the design doc's running
    /// value is 200 and going limp is 50. Nothing has measured the two firmware's gain units
    /// against each other, so [`RobotIo::set_gain`] treats the number as already being in this
    /// unit and says so. If 200 turns out to oscillate where 10 was firm, the translation
    /// belongs at the call site — a measurement, not a guess made here.
    pub kp: u16,
    pub kd: u16,
    /// Extra attempts per joint within one pass when a reply is short or unparseable.
    ///
    /// Not optional at this chain length: the manual lists packet loss on a 14-servo chain as
    /// ordinary. One retry costs 0.17 ms; a failed tick costs the policy's observation.
    pub retries: u32,
    pub read_timeout: Duration,
}

/// Joint-side gain -> the firmware's `k_pos` counts, per unit.
///
/// From `unitree_servo/unitree_servo.py`: `Kp / RATIO² × 1_280_000`, the conversion that was on
/// the wire when 10-20 came out a firm hold. Deriving it again here would only be a second
/// guess at the same firmware unit; inheriting it means the bench's numbers stay meaningful.
pub const KP_COUNTS: f64 = 1_280_000.0 / (RATIO * RATIO);
/// The same for `k_spd`, which the reference scales by `1_280_000_00`.
pub const KD_COUNTS: f64 = 128_000_000.0 / (RATIO * RATIO);

/// `kd` 1 held steps without ringing on the bench. See [`KP_COUNTS`] for the units.
pub const DEFAULT_KD: u16 = 1;

impl Default for Config {
    fn default() -> Self {
        let mut ids = [0u8; NUM_JOINTS];
        for (i, id) in ids.iter_mut().enumerate() {
            // 0..=14: every address the bus has, and none of them is broadcast.
            *id = i as u8;
        }
        Self {
            ids,
            sign: [1.0; NUM_JOINTS],
            offset_rad: [0.0; NUM_JOINTS],
            kp: 10,
            kd: DEFAULT_KD,
            retries: 1,
            read_timeout: Duration::from_millis(20),
        }
    }
}

/// Every address a duck occupies, in [`crate::model::JOINT_NAMES`] order.
///
/// The robot's addresses rather than a bus instance's, for the one caller that needs them
/// before it has a handle: `robot.rebootMotors` with no names means "all of them". It reads
/// [`Config`]'s default rather than keeping a second table, so the two cannot drift apart; if
/// the robot ever learns to read its addresses from configuration, this becomes a method on
/// the live bus and this function goes away.
pub fn all_ids() -> [u8; NUM_JOINTS] {
    Config::default().ids
}

impl BusS288<SerialTransport> {
    /// Open the port and build the bus a duck runs.
    ///
    /// Nothing is verified here, and that is deliberate: opening a tty says nothing about
    /// whether anything is on the other end of it, and a chain that has not been powered yet
    /// must not look like a broken port. `robotd` waits for power rather than giving up, and
    /// its retry loop needs "could not open" and "nothing answered" to be different answers
    /// (the first is a wiring or device problem, the second is a switch). [`Self::verify_chain`]
    /// is the second half.
    pub fn open(port: &str) -> Result<Self> {
        let cfg = Config::default();
        let transport =
            SerialTransport::open(port, cfg.read_timeout).map_err(|e| IoError::Port {
                path: port.to_owned(),
                source: e,
            })?;
        Self::with_transport(transport, cfg)
    }
}

// --------------------------------------------------------------------------- IMU seam

/// Where `Sensors.imu` comes from, since it is not on this bus.
pub trait ImuSource: Send {
    fn sample(&mut self) -> Result<ImuData>;
    fn ready(&self) -> bool;
}

/// The honest placeholder until the LSM6DSV16X reader exists: a zero sample, and `ready()`
/// false so nothing downstream believes it.
pub struct NoImu;

impl ImuSource for NoImu {
    fn sample(&mut self) -> Result<ImuData> {
        Ok(ImuData::default())
    }
    fn ready(&self) -> bool {
        false
    }
}

// --------------------------------------------------------------------------- the bus

/// Diagnostics the bus keeps about itself — the S288's version of `ImuStale`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BusHealth {
    /// Frames that did not arrive intact, retried or not.
    pub dropped: u64,
    /// Retries spent. A count that climbs without `dropped` climbing is a marginal cable.
    pub retries: u64,
    /// Frames whose `MError` was non-zero, cumulative.
    pub errors: u64,
    /// Passes: one frame per joint, per pass.
    pub passes: u64,
}

pub struct BusS288<T: Transport = SerialTransport> {
    transport: T,
    imu: Box<dyn ImuSource>,
    cfg: Config,
    /// Last commanded joint targets, re-sent when the loop only asks for state. That is what
    /// keeps a standing robot's torque on between `write`s — see [`KEEPALIVE`].
    targets: JointTargets,
    /// Latest per-joint state, and the instant the pass that produced it finished.
    sample: Sensors,
    /// Voltage and case temperature from that same pass. Free, because they ride in every
    /// frame; kept so [`RobotIo::slow_sensors`] has something to answer with.
    slow: SlowSensors,
    sampled_at: Option<Instant>,
    /// Gains in [`Config::kp`]'s unit, converted to wire counts per frame by [`KP_COUNTS`].
    gain_kp: u16,
    gain_kd: u16,
    torque_on: bool,
    health: BusHealth,
}

impl<T: Transport> BusS288<T> {
    /// Build a bus, refusing a configuration that would misbehave on the wire.
    ///
    /// The checks are not defensive padding: `ids` comes from an operator assigning addresses
    /// through the Windows上位机, and broadcast or duplicate addresses are silent failures
    /// (every servo acts and none answers; two joints share one state) rather than errors
    /// anything downstream could detect.
    pub fn with_transport(transport: T, cfg: Config) -> Result<Self> {
        Self::check_config(&cfg)?;
        Ok(Self {
            transport,
            imu: Box::new(NoImu),
            gain_kp: cfg.kp,
            gain_kd: cfg.kd,
            cfg,
            targets: JointTargets::new([0.0; NUM_JOINTS]),
            sample: Sensors::default(),
            slow: no_slow(),
            sampled_at: None,
            torque_on: false,
            health: BusHealth::default(),
        })
    }

    /// The per-unit calibration, adjustable after construction — the offsets are measured on an
    /// assembled robot, which is after the bus object exists.
    pub fn config_mut(&mut self) -> &mut Config {
        &mut self.cfg
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Swap the IMU source, for when the I2C reader lands (or a fake, in tests).
    pub fn set_imu(&mut self, imu: Box<dyn ImuSource>) {
        self.imu = imu;
    }

    pub fn health(&self) -> BusHealth {
        self.health
    }

    /// The servo addresses this bus is talking to, in [`crate::model::JOINT_NAMES`] order.
    pub fn ids(&self) -> &[u8; NUM_JOINTS] {
        &self.cfg.ids
    }

    /// Every servo answered. This is the S288's answer to the XL330's `check_registers`, and
    /// the two are deliberately not the same shape.
    ///
    /// `check_registers` asserts EEPROM values, because an XL330 ships with a `return_delay_time`
    /// that quietly eats bus budget and a `shutdown` mask that quietly latches. The S288 exposes
    /// no user EEPROM — there is nothing to assert and nothing a factory reset could have
    /// changed — so a boot verifies the one thing the wire *can* say: that the whole chain is
    /// there. One pass tells it, because each frame's reply is that joint's state, so fifteen
    /// answers is a complete census. A short chain fails here, loudly, instead of waking up as a
    /// robot with a dead leg.
    ///
    /// **Torque is off at this point and this cannot move the robot**: nothing enables torque
    /// before a human asks, and a frame with zero gains and no feedforward holds no current
    /// (see [`BusS288::exchange_joint`]). A boot check that twitched the robot would be worse
    /// than no check at all.
    ///
    /// A latched `MError` is *reported, not fatal*: a servo with a fault still answers frames,
    /// and the way out of it is `robot.rebootMotors`, which clears the control bits. Failing
    /// here would turn "one servo has a latched fault" into "the robot never comes up", since
    /// the caller's response to a failure is to wait and try again.
    pub fn verify_chain(&mut self) -> Result<()> {
        self.pass()?;
        if self.health.errors > 0 {
            tracing::warn!(
                errors = self.health.errors,
                joints = NUM_JOINTS,
                "servos answered with MError set; the chain is up, but a latched fault holds \
                 torque off until `robot.rebootMotors` clears it"
            );
        }
        Ok(())
    }

    /// Present positions only — the lighter read `robotd init` uses to learn the pose the robot
    /// is already in before it ramps anywhere. Joint-side radians with [`Config::sign`] and
    /// [`Config::offset_rad`] applied, i.e. the same units [`RobotIo::read`] reports and
    /// [`Self::interpolate_to`] consumes.
    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        self.pass()?;
        Ok(self.sample.positions)
    }

    /// Ramp every joint from where it is now to `target`, linearly.
    ///
    /// Only ever called by an explicit `init` — the control loop must never move the robot on
    /// its own, because that would make an update restart a fall risk. Blocking, and
    /// deliberately so: nothing else should be talking to the bus while this runs.
    ///
    /// Each step is a full pass, so a ramp is `steps` × 15 frames at ~0.17 ms each. That is the
    /// price of a bus with no broadcast write, and it is paid once per `init`, not per tick.
    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.present_positions()?;
        let steps = (duration.as_secs_f64() / step.as_secs_f64())
            .ceil()
            .max(1.0) as u32;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mut next = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                next[j] = start[j] + (target[j] - start[j]) * t;
            }
            self.write(&JointTargets::new(next))?;
            std::thread::sleep(step);
        }
        Ok(())
    }

    fn check_config(cfg: &Config) -> Result<()> {
        for (joint, &id) in cfg.ids.iter().enumerate() {
            if id > 14 {
                return Err(IoError::Bus(format!(
                    "joint {joint} is configured as servo id {id}, but 15 is the S288's \
                     broadcast address -- every servo would act and none would answer"
                )));
            }
            if cfg.ids[..joint].contains(&id) {
                return Err(IoError::Bus(format!(
                    "servo id {id} is assigned to joint {joint} and to an earlier joint; \
                     duplicate ids corrupt the whole bus, not just this joint"
                )));
            }
        }
        Ok(())
    }

    /// One frame out, one frame back, for one joint, with retries.
    fn exchange_joint(&mut self, joint: usize, torque_nm: f64) -> Result<Feedback> {
        let id = self.cfg.ids[joint];
        let (kp, kd) = if self.torque_on {
            (
                clamp_i16((self.gain_kp as f64 * KP_COUNTS).round()),
                clamp_i16((self.gain_kd as f64 * KD_COUNTS).round()),
            )
        } else {
            // "Torque off" for a servo with no torque-enable register is a closed-loop frame
            // with both gains and the feedforward at zero: the FOC holds no current, so the
            // shaft is free, while frames keep arriving so the watchdog never trips and no
            // surprise comes back with it. (Whether mode 0 would be a *brake* instead is not
            // stated anywhere in the manual — "锁定" is all it says. It matters for a falling
            // robot, and it is a two-minute bench test to settle.)
            (0, 0)
        };
        let target_pos = self.targets.positions[joint];
        let pos_out = self.cfg.sign[joint] * target_pos + self.cfg.offset_rad[joint];

        let cmd = build_control(
            id,
            1,
            /* timeout */ true,
            if self.torque_on { torque_nm } else { 0.0 },
            0.0,
            pos_out,
            kp,
            kd,
        );

        let mut attempts = 0;
        loop {
            match self.transport.exchange(&cmd) {
                Ok(buf) => match parse_feedback(&buf) {
                    Some(fb) => {
                        // A frame that answers as a *different* joint is worse than no frame:
                        // it would silently mix two joints' state.
                        if fb.id != id {
                            self.health.dropped += 1;
                            if attempts < self.cfg.retries {
                                attempts += 1;
                                self.health.retries += 1;
                                continue;
                            }
                            return Err(IoError::ShortRead {
                                what: "S288 frame id",
                                expected: id as usize,
                                got: fb.id as usize,
                            });
                        }
                        if fb.merror != 0 {
                            self.health.errors += 1;
                        }
                        return Ok(fb);
                    }
                    None => {
                        self.transport.flush_input();
                    }
                },
                Err(_) => {}
            }
            self.health.dropped += 1;
            if attempts < self.cfg.retries {
                attempts += 1;
                self.health.retries += 1;
                continue;
            }
            // The trait's `ShortRead` is the right shape here too: a device did not answer,
            // and returning stale values for it would be the silent half-array the trait's
            // own comment warns about.
            return Err(IoError::ShortRead {
                what: "S288 reply",
                expected: FBK_LEN,
                got: 0,
            });
        }
    }

    /// One pass: every joint gets a frame, and each reply updates that joint's state.
    fn pass(&mut self) -> Result<()> {
        let mut sensors = Sensors::default();
        let mut slow = no_slow();
        let mut volts_sum = 0.0;
        for joint in 0..NUM_JOINTS {
            // No feedforward: `JointTargets` is position-only, and the S288's torque
            // feedforward has no field on it. A policy that wants torque feedforward needs a
            // field there, not a bus that guesses one.
            let fb = self.exchange_joint(joint, 0.0)?;
            let out_pos = fb.pos_rotor_rad / RATIO;
            let out_spd = fb.spd_rotor_rad_s / RATIO;
            sensors.positions[joint] =
                self.cfg.sign[joint] * (out_pos - self.cfg.offset_rad[joint]);
            sensors.velocities[joint] = self.cfg.sign[joint] * out_spd;
            // No current telemetry on this bus, and no measured torque constant to invent it
            // from — see the module docs. Left at zero, which is a number nobody can mistake
            // for a measurement.
            sensors.currents_ma[joint] = 0.0;
            // Case temperature and supply voltage, free with the frame that just arrived.
            slow.temps_c[joint] = fb.temp_c as f64;
            volts_sum += fb.volts;
        }
        slow.volts = volts_sum / NUM_JOINTS as f64;
        sensors.imu = self.imu.sample()?;
        self.sample = sensors;
        self.slow = slow;
        self.sampled_at = Some(Instant::now());
        self.health.passes += 1;
        Ok(())
    }
}

impl<T: Transport> RobotIo for BusS288<T> {
    /// State, refreshed from the bus when the last pass has aged past [`KEEPALIVE`].
    ///
    /// Sending nothing when the sample is fresh is the point: the loop calls `read` and `write`
    /// back to back, and this bus needs one frame per joint per tick either way. The `write`
    /// that follows refreshes the sample; the next `read` finds it fresh and costs nothing.
    fn read(&mut self) -> Result<Sensors> {
        let stale = match self.sampled_at {
            None => true,
            Some(t) => t.elapsed() >= KEEPALIVE,
        };
        if stale {
            self.pass()?;
        }
        Ok(self.sample)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        self.targets = *targets;
        self.pass()
    }

    /// Stored in [`Config::kp`]'s unit; the number is taken as already being in it. See that
    /// field's docs for why the cross-firmware translation is *not* a factor invented here.
    fn set_gain(&mut self, kp: u16) -> Result<()> {
        self.gain_kp = kp;
        Ok(())
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        self.torque_on = on;
        // Applied now rather than at the next tick: "go limp" that waits 20 ms is 20 ms of a
        // robot still pushing.
        self.pass()
    }

    fn reboot(&mut self, id: u8) -> Result<()> {
        // Not a reboot. The S288 has no such instruction; what it has is a latched `MError`
        // that clearing the control bits is documented to clear. So: stop, re-arm closed loop,
        // and report whether the servo came back clean. A servo that still reports an error has
        // one that software cannot clear (the encoder class), and saying so is more useful than
        // claiming a reboot happened.
        let joint = self
            .cfg
            .ids
            .iter()
            .position(|&x| x == id)
            .ok_or_else(|| IoError::Bus(format!("servo id {id} is not on this robot")))?;

        let stop = build_control(id, 0, true, 0.0, 0.0, 0.0, 0, 0);
        let _ = self.transport.exchange(&stop);
        std::thread::sleep(Duration::from_millis(50));

        let fb = self.exchange_joint(joint, 0.0)?;
        if fb.merror != 0 {
            return Err(IoError::Bus(format!(
                "servo {id} still reports MError {:#010x} after clearing: this one needs a \
                 power cycle",
                fb.merror
            )));
        }
        Ok(())
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        // Free here: voltage and temperature ride in every frame, so this is telling the trait
        // what already arrived rather than buying anything. Before the first pass there is
        // nothing to report, and an error is the honest answer -- the same choice `FakeIo`
        // makes with `slow: None`.
        if self.sampled_at.is_none() {
            return Err(IoError::Bus("no frames yet: nothing to report".into()));
        }
        Ok(self.slow)
    }

    fn imu_ready(&self) -> bool {
        self.imu.ready()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The control frame the hardware was asked to send, and the CRC it carried. From
    /// `unitree_servo/docs/verify_official_issues_output.txt`.
    const CTRL_BODY: [u8; 16] = [
        0xfe, 0xee, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ];
    /// The real feedback frame's CRC region, and the CRC the hardware put on the wire.
    const FBK_BODY: [u8; 20] = [
        0x00, 0x36, 0x2d, 0x18, 0xf4, 0xff, 0x00, 0x00, 0x1b, 0x94, 0x29, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x3f, 0x09, 0x00, 0x00,
    ];

    #[test]
    fn crc_matches_both_real_frames() {
        assert_eq!(crc32(&CTRL_BODY), 0x7920_4680);
        assert_eq!(crc32(&FBK_BODY), 0x42DC_F4D4);
    }

    #[test]
    fn crc_over_the_byte_stream_is_wrong() {
        // The negative result, pinned so nobody "simplifies" the word walk away: a standard
        // CRC-32/MPEG-2 over the same bytes gives these, and they do not match the wire.
        assert_eq!(crc32_byte_order(&CTRL_BODY), 0xFCA_DA9FB);
        assert_eq!(crc32_byte_order(&FBK_BODY), 0x29CB_30E1);
    }

    #[test]
    fn parses_a_frame_the_hardware_actually_sent() {
        let mut frame = [0u8; FBK_LEN];
        frame[0] = 0xFC;
        frame[1] = 0xEE;
        frame[2..22].copy_from_slice(&FBK_BODY);
        frame[22..26].copy_from_slice(&0x42DC_F4D4u32.to_le_bytes());

        let fb = parse_feedback(&frame).expect("real frame must parse");
        assert_eq!(fb.id, 0);
        assert_eq!(fb.mode, 0);
        assert!(!fb.timed_out);
        assert_eq!(fb.temp_c, 54); // 0x36, the case temperature logged that day
        assert_eq!(fb.volts, 12.0); // 0x18 = 24 counts of 0.5 V
        assert_eq!(fb.torque_rotor_nm, -12.0 / TORQUE_COUNTS_PER_NM); // 0xfff4
        // The position the servo reported: 0x0029941b counts of 32768/2π per rad.
        assert!((fb.pos_rotor_rad - 2724891.0 / POS_COUNTS_PER_ROTOR_RAD).abs() < 1e-12);
        assert_eq!(fb.merror, 0);
        // ExPos 2367 of 8192 per turn, from the 0x093f word.
        assert!((fb.out_pos_rad - 2367.0 * TWO_PI / 8192.0).abs() < 1e-12);
        assert_eq!(fb.ex_flag, 0);
    }

    #[test]
    fn rejects_frames_that_are_not_frames() {
        let mut good = [0u8; FBK_LEN];
        good[0] = 0xFC;
        good[1] = 0xEE;
        good[2..22].copy_from_slice(&FBK_BODY);
        good[22..26].copy_from_slice(&0x42DC_F4D4u32.to_le_bytes());

        assert!(parse_feedback(&good).is_some());
        assert!(parse_feedback(&good[..FBK_LEN - 1]).is_none(), "short frame");
        let mut wrong_header = good;
        wrong_header[0] = 0xFE;
        assert!(parse_feedback(&wrong_header).is_none(), "not a feedback header");
        let mut bad_crc = good;
        bad_crc[25] ^= 0xFF;
        assert!(parse_feedback(&bad_crc).is_none(), "corrupted CRC must not pass");
    }

    #[test]
    fn control_frame_is_rotor_side_and_signed() {
        // +0.2 rad at the output is 0.2 × 288.35 rotor rad on the wire.
        let pkt = build_control(3, 1, true, 0.0, 0.0, 0.2, 20, 1);
        assert_eq!(pkt[0], 0xFE);
        assert_eq!(pkt[1], 0xEE);
        assert_eq!(pkt[2], (3 & 0x0F) | (1 << 4) | (1 << 7));
        let pos = i32::from_le_bytes([pkt[8], pkt[9], pkt[10], pkt[11]]) as f64;
        assert!((pos - (0.2 * RATIO * POS_COUNTS_PER_ROTOR_RAD).round()).abs() < 1.0);
        assert_eq!(i16::from_le_bytes([pkt[12], pkt[13]]), 20);
        assert_eq!(
            u32::from_le_bytes([pkt[16], pkt[17], pkt[18], pkt[19]]),
            crc32(&pkt[CMD_CRC_RANGE])
        );

        // And the field the official example cannot send: a negative target must encode, not
        // panic and not wrap to a huge unsigned number.
        let neg = build_control(0, 1, true, 0.0, 0.0, -0.2, 0, 0);
        let npos = i32::from_le_bytes([neg[8], neg[9], neg[10], neg[11]]);
        assert!(npos < 0);
    }

    #[test]
    fn torque_and_speed_convert_through_the_ratio() {
        let pkt = build_control(0, 1, true, 0.1, 0.5, 0.0, 0, 0);
        let tor = i16::from_le_bytes([pkt[4], pkt[5]]) as f64;
        let spd = i16::from_le_bytes([pkt[6], pkt[7]]) as f64;
        // 0.1 N·m at the output is 0.1/288.35 N·m at the rotor, in 1/256000 N·m counts.
        assert!((tor - (0.1 / RATIO * TORQUE_COUNTS_PER_NM).round()).abs() < 1.0);
        assert!((spd - (0.5 * RATIO * SPD_COUNTS_PER_ROTOR_RAD_S).round()).abs() < 1.0);
    }

    /// A servo that answers every frame, remembering what it was last told.
    struct FakeServo {
        id: u8,
        /// Joint-space angle the servo will report, i.e. already through RATIO and the config.
        out_pos: f64,
        merror: u32,
        seen: Vec<[u8; CMD_LEN]>,
        /// How many replies to swallow before answering again. Models a marginal cable.
        drop_next: u32,
        /// Answer as this id instead — the case that must never be mixed up silently.
        answer_as: Option<u8>,
    }

    impl FakeServo {
        fn reply(&self, owner: u8) -> [u8; FBK_LEN] {
            let mut body = [0u8; 20];
            body[0] = self.answer_as.unwrap_or(owner) & 0x0F | 0x10; // mode 1
            body[1] = 54; // case temperature
            body[3] = 24; // 12.0 V
            let pos = (self.out_pos * RATIO * POS_COUNTS_PER_ROTOR_RAD).round() as i32;
            body[8..12].copy_from_slice(&pos.to_le_bytes());
            body[12..16].copy_from_slice(&self.merror.to_le_bytes());
            body[16..18].copy_from_slice(&2367u16.to_le_bytes());

            let mut frame = [0u8; FBK_LEN];
            frame[0] = 0xFC;
            frame[1] = 0xEE;
            frame[2..22].copy_from_slice(&body);
            let crc = crc32(&frame[FBK_CRC_RANGE]);
            frame[22..26].copy_from_slice(&crc.to_le_bytes());
            frame
        }
    }

    /// One fake servo per address, so a test can see the whole pass.
    struct FakeBus {
        servos: Vec<FakeServo>,
        /// Addresses that answer nothing at all.
        dead: Vec<u8>,
    }

    impl Transport for FakeBus {
        fn exchange(&mut self, cmd: &[u8; CMD_LEN]) -> std::io::Result<[u8; FBK_LEN]> {
            let id = cmd[2] & 0x0F;
            if self.dead.contains(&id) {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "no answer",
                ));
            }
            for s in self.servos.iter_mut() {
                if s.id == id {
                    s.seen.push(*cmd);
                    if s.drop_next > 0 {
                        s.drop_next -= 1;
                        return Err(std::io::Error::new(
                            std::io::ErrorKind::UnexpectedEof,
                            "dropped",
                        ));
                    }
                    return Ok(s.reply(id));
                }
            }
            Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "no such servo",
            ))
        }

        fn flush_input(&mut self) {}
    }

    /// Every address answers, all at the same angle, so the joint math is visible.
    fn full_bus() -> BusS288<FakeBus> {
        full_bus_with_drop(0, 0)
    }

    /// A full bus where `joint` swallows its first `drops` replies — a marginal cable.
    fn full_bus_with_drop(joint: usize, drops: u32) -> BusS288<FakeBus> {
        let servos = (0..15u8)
            .map(|id| FakeServo {
                id,
                out_pos: 0.25,
                merror: 0,
                seen: Vec::new(),
                drop_next: if id as usize == joint { drops } else { 0 },
                answer_as: None,
            })
            .collect();
        BusS288::with_transport(FakeBus { servos, dead: vec![] }, Config::default())
            .expect("the default config is valid by construction")
    }

    #[test]
    fn read_reports_the_joint_angle_through_sign_and_offset() {
        let mut bus = full_bus();
        // The per-joint calibration, the two numbers the servo cannot store.
        bus.config_mut().sign[0] = -1.0;
        bus.config_mut().offset_rad[0] = 0.05;

        let s = bus.read().expect("a full bus answers");
        // One rotor position count is 2π/32768 rad at the rotor, so 6.6e-7 rad at the output —
        // the frame quantises, and a tolerance tighter than that would be asserting on the
        // encoder's LSB rather than on the conversion. (The error here is 3e-8: a tenth of a
        // count, which is what a round-trip through the counts should cost.)
        const LSB: f64 = TWO_PI / 32768.0 / RATIO;
        // joint = sign * (output - offset) = -(0.25 - 0.05)
        assert!((s.positions[0] + 0.20).abs() < LSB, "{}", s.positions[0]);
        assert!((s.positions[14] - 0.25).abs() < LSB, "{}", s.positions[14]);
        assert!(!bus.imu_ready(), "no IMU: nothing may claim orientation");
    }

    #[test]
    fn read_is_free_while_the_sample_is_fresh_but_never_lets_the_bus_go_quiet() {
        let mut bus = full_bus();
        bus.write(&JointTargets::new([0.0; NUM_JOINTS])).unwrap();
        let after_write = bus.transport_mut().servos[0].seen.len();
        assert_eq!(after_write, 1);

        // A second read inside KEEPALIVE must cost nothing: the loop's read+write per tick is
        // one pass, not two.
        bus.read().unwrap();
        assert_eq!(bus.transport_mut().servos[0].seen.len(), after_write);

        // Past KEEPALIVE it must speak, or the firmware's 1 s watchdog drops torque.
        bus.sampled_at = Some(Instant::now() - (KEEPALIVE + Duration::from_millis(1)));
        bus.read().unwrap();
        assert_eq!(bus.transport_mut().servos[0].seen.len(), after_write + 1);
    }

    #[test]
    fn a_dropped_frame_is_retried_and_then_counted() {
        let mut bus = full_bus_with_drop(0, 1);
        bus.read().expect("one drop is retried by the default config");
        let h = bus.health();
        assert_eq!(h.dropped, 1);
        assert_eq!(h.retries, 1);
    }

    #[test]
    fn a_dead_servo_fails_the_read_rather_than_returning_stale_values() {
        // Config with retries 0, so the first miss is the failure.
        let mut cfg = Config::default();
        cfg.retries = 0;
        let mut bus = BusS288::with_transport(
            FakeBus {
                servos: vec![],
                dead: (0..15).collect(),
            },
            cfg,
        )
        .expect("the addresses are valid: none is broadcast or duplicated");
        match bus.read() {
            Err(IoError::ShortRead { what, expected, .. }) => {
                assert_eq!(what, "S288 reply");
                assert_eq!(expected, FBK_LEN);
            }
            other => panic!("expected a short read, got {other:?}"),
        }
    }

    #[test]
    fn the_gain_reaches_the_wire_through_the_reference_conversion() {
        let mut bus = full_bus();
        // 20 is the bench's firm hold. `unitree_servo/unitree_servo.py` turns it into 308 counts
        // (`Kp / RATIO² × 1.28e6`) and `Kd` 1 into 1539. Literals on purpose: this test exists to
        // catch the bus inventing a plausible factor of its own. A 1:1 pass-through would send
        // 20 and be 15x too soft, and the number on the wire would look perfectly reasonable.
        bus.set_gain(20).unwrap();
        bus.set_torque(true).unwrap();
        let last = bus.transport_mut().servos[0].seen.last().copied().unwrap();
        assert_eq!(i16::from_le_bytes([last[12], last[13]]), 308);
        assert_eq!(i16::from_le_bytes([last[14], last[15]]), 1539);

        // The top of the u16 range converts past i16::MAX: it must clamp, not wrap into a
        // negative gain and drive the joint the other way.
        bus.set_gain(u16::MAX).unwrap();
        bus.set_torque(true).unwrap();
        let last = bus.transport_mut().servos[0].seen.last().copied().unwrap();
        assert_eq!(i16::from_le_bytes([last[12], last[13]]), i16::MAX);
    }

    #[test]
    fn torque_off_sends_zero_gains_and_no_feedforward() {
        let mut bus = full_bus();
        bus.set_torque(false).unwrap();
        let last = bus.transport_mut().servos[0].seen.last().copied().unwrap();
        assert_eq!(i16::from_le_bytes([last[12], last[13]]), 0);
        assert_eq!(i16::from_le_bytes([last[14], last[15]]), 0);
        assert_eq!(i16::from_le_bytes([last[4], last[5]]), 0);
        // Mode stays closed-loop and the watchdog stays enabled: limp is not unplugged.
        assert_eq!((last[2] >> 4) & 0x07, 1);
        assert_eq!((last[2] >> 7) & 0x01, 1);
    }

    #[test]
    fn reboot_reports_a_latched_error_instead_of_claiming_success() {
        let servos = (0..15u8)
            .map(|id| FakeServo {
                id,
                out_pos: 0.0,
                merror: if id == 3 { 0x0000_2000 } else { 0 },
                seen: Vec::new(),
                drop_next: 0,
                answer_as: None,
            })
            .collect();
        let mut bus = BusS288::with_transport(
            FakeBus {
                servos,
                dead: vec![],
            },
            Config::default(),
        )
        .expect("config is fine");
        match bus.reboot(3) {
            Err(IoError::Bus(msg)) => assert!(msg.contains("power cycle"), "{msg}"),
            other => panic!("a stuck MError must be reported, got {other:?}"),
        }
        assert!(bus.reboot(0).is_ok(), "a clean servo reboots clean");
    }

    #[test]
    fn a_reply_from_the_wrong_servo_is_refused() {
        let servos = vec![FakeServo {
            id: 0,
            out_pos: 0.0,
            merror: 0,
            seen: Vec::new(),
            drop_next: 0,
            answer_as: Some(7), // says it is joint 7 while addressed as 0
        }];
        let mut cfg = Config::default();
        cfg.retries = 0;
        let mut bus = BusS288::with_transport(
            FakeBus {
                servos,
                dead: vec![],
            },
            cfg,
        )
        .expect("the addresses are valid");
        match bus.read() {
            Err(IoError::ShortRead { what, expected, got }) => {
                assert_eq!(what, "S288 frame id");
                assert_eq!((expected, got), (0, 7));
            }
            other => panic!("expected an id mismatch, got {other:?}"),
        }
    }

    #[test]
    fn config_rejects_broadcast_and_duplicate_ids() {
        // 15 is broadcast: every servo would act and none would answer, so the constructor
        // refuses rather than letting a robot twitch in response to a joint nobody can read.
        let mut cfg = Config::default();
        cfg.ids[3] = 15;
        assert!(
            BusS288::with_transport(FakeBus { servos: vec![], dead: vec![] }, cfg).is_err(),
            "15 is broadcast"
        );

        // Two joints on one address: they would share a state, and one would never be commanded.
        let mut cfg = Config::default();
        cfg.ids[3] = 7;
        assert!(
            BusS288::with_transport(FakeBus { servos: vec![], dead: vec![] }, cfg).is_err(),
            "duplicate addresses"
        );

        assert!(full_bus().health() == BusHealth::default(), "the default config is accepted");
    }

    #[test]
    fn slow_sensors_are_free_and_carry_the_case_temperature() {
        let mut bus = full_bus();
        assert!(bus.slow_sensors().is_err(), "nothing to report before a pass");
        bus.read().unwrap();
        let slow = bus.slow_sensors().unwrap();
        assert_eq!(slow.temps_c[0], 54.0);
        assert_eq!(slow.volts, 12.0);
    }
}
