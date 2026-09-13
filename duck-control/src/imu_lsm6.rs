//! The LSM6DSV16X read directly over I²C — no `imu_to_dxl` board.
//!
//! Upstream the chip is soldered to a board that impersonates a Dynamixel servo, so the IMU
//! rides the motor bus and arrives in the same `sync_read` as the joints. This fork has no
//! such board: the chip is a module on the 40-pin I²C header, and this module is what replaces
//! the board's half of the bargain.
//!
//! **What is deliberately reused.** The board was forwarding the chip's *own* SFLP output —
//! the `imu.rs` block layout documents a quaternion in IEEE half-precision, and that is SFLP's
//! native format, not something the board invented. So [`SflpDecoder`] keeps its job: this
//! module gathers the same twelve bytes the board used to hand over and gives them to it.
//! Mount rotation, spike rejection, gravity-from-quaternion and the `ready()` contract all
//! stay in one place, and the I²C path cannot drift from the Dynamixel path without a test
//! noticing.
//!
//! **Where the two differ, and why it matters.** The board read the IMU in the same
//! transaction as the joints; here it is a second bus with its own timing. Two consequences:
//!
//!  - **Gyro is read straight from `OUTX_L_G` every tick, not out of the FIFO.** The
//!    fall predictor differentiates projected gravity using the *current* angular rate
//!    (`fall.rs`: `ġ = −ω × g`), so a gyro sample queued in a batch some milliseconds ago is
//!    the wrong number — it would make the predictor look a tick or two into the past and
//!    react late, which is the one thing a fall predictor cannot do.
//!  - **The quaternion comes out of the FIFO, because SFLP has no register to read.** This is
//!    not a design choice; it is the chip. ST's own driver (`lsm6dsv16x_reg.c`) reaches the
//!    SFLP outputs only through `fifo_out_raw_get`, and the datasheet gives SFLP no direct
//!    output register. ST's application note for the block is AN5277 if this ever needs
//!    re-deriving.
//!
//! ## Register map
//!
//! Every address, bit position and enumeration value below was transcribed from ST's official
//! driver, `lsm6dsv16x_reg.h` / `.c` (`STMicroelectronics/lsm6dsv16x-pid`), not from a
//! datasheet summary and not from memory. The first version of the bench script in this
//! project's skill got `WHO_AM_I` wrong (`0x6B`, which is the ISM330DHCX, not this part) and
//! three addresses besides, so the citations are here to make the next reader's check cheap.
//!
//! Note what generation this part is: it has **no `CTRL3_C`** — the interface-configuration
//! register is `IF_CFG` at `0x03` and the one carrying `if_inc`/`bdu` is `CTRL3` at `0x12`.
//! Addresses familiar from an LSM6DS3 or an LSM6DSO do not transfer.

use crate::bus_s288::ImuSource;
use crate::imu::{ImuData, IMU_BLOCK_LEN, SflpDecoder};
use crate::io::{IoError, Result};

// ── register addresses (main page) ────────────────────────────────────────────────

/// Embedded-function page selector. Bit 0 opens the embedded page (`mem_bank_set` in ST's
/// driver writes exactly this bit and nothing else).
const FUNC_CFG_ACCESS: u8 = 0x01;
/// `FIFO_CTRL3` — per-sensor batching rates, low nibble accel, high nibble gyro.
const FIFO_CTRL3: u8 = 0x09;
/// `FIFO_CTRL4` — `fifo_mode` in bits 2:0.
const FIFO_CTRL4: u8 = 0x0A;
/// Accelerometer ODR in bits 3:0, `op_mode_xl` in bits 6:4.
const CTRL1: u8 = 0x10;
/// Gyroscope ODR in bits 3:0, `op_mode_g` in bits 6:4.
const CTRL2: u8 = 0x11;
/// `sw_reset` bit 0, `if_inc` bit 2, `bdu` bit 6.
const CTRL3: u8 = 0x12;
/// Gyroscope full scale in bits 3:0.
const CTRL6: u8 = 0x15;
/// Accelerometer full scale in bits 1:0.
const CTRL8: u8 = 0x17;
/// FIFO occupancy, low byte.
const FIFO_STATUS1: u8 = 0x1B;
/// FIFO occupancy, high bits and status flags.
const FIFO_STATUS2: u8 = 0x1C;
/// Gyroscope output, X low byte — six bytes of `i16` little-endian follow.
const OUTX_L_G: u8 = 0x22;
const WHO_AM_I: u8 = 0x0F;
/// FIFO read port: a tag byte, then that word's payload. Reading here pops the FIFO.
const FIFO_DATA_OUT_TAG: u8 = 0x78;

// ── register addresses (embedded-function page; open `FUNC_CFG_ACCESS` first) ──────

/// `sflp_game_en` is bit 1.
const EMB_FUNC_EN_A: u8 = 0x04;
/// `sflp_game_fifo_en` is bit 1, `sflp_gravity_fifo_en` bit 4, `sflp_gbias_fifo_en` bit 5.
const EMB_FUNC_FIFO_EN_A: u8 = 0x44;
/// `sflp_game_init` is bit 1.
const EMB_FUNC_INIT_A: u8 = 0x66;
/// `sflp_game_odr` is bits 5:3.
const SFLP_ODR: u8 = 0x5E;

// ── values ────────────────────────────────────────────────────────────────────────

/// The part this module knows how to drive. `0x71` would be an LSM6DSV16B — same SFLP
/// registers, different identity, so it is rejected by name rather than mis-driven.
const WHO_AM_I_LSM6DSV16X: u8 = 0x70;

const FUNC_CFG_EMB_FUNC_ACCESS: u8 = 1 << 0;
const CTRL3_IF_INC: u8 = 1 << 2;
const CTRL3_BDU: u8 = 1 << 6;
const EMB_FUNC_EN_A_SFLP_GAME: u8 = 1 << 1;
const EMB_FUNC_FIFO_EN_A_SFLP_GAME: u8 = 1 << 1;
const EMB_FUNC_INIT_A_SFLP_GAME: u8 = 1 << 1;

/// 120 Hz, `LSM6DSV16X_ODR_AT_120Hz` in the shared accel/gyro ODR enumeration. Four times the
/// control loop, so a tick never starves, and far below the 960 Hz the part can do — the
/// policy was trained on nothing finer.
const ODR_120_HZ: u8 = 0x6;
/// `LSM6DSV16X_500dps`. Chosen to match the `imu_to_dxl` board rather than the part: the
/// board fixed 17.5 mdps/LSB, and [`SflpDecoder`]'s `GYRO_RAD_PER_LSB` is that number. A
/// different scale here would silently mis-scale every angular rate the policy sees.
const FS_G_500_DPS: u8 = 0x2;
/// `LSM6DSV16X_4g`.
const FS_XL_4_G: u8 = 0x1;
/// `LSM6DSV16X_SFLP_60Hz` — comfortably above the control loop, and inside the part's range.
const SFLP_ODR_60_HZ: u8 = 0x2;
/// `LSM6DSV16X_STREAM_MODE`: oldest words are overwritten rather than the FIFO filling and
/// stalling. A stall here would freeze the quaternion for as long as the robot took to notice.
const FIFO_MODE_STREAM: u8 = 0x6;

/// `LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG`. The 16B uses `0x16` for the same thing, which
/// is one more reason to identify the part before trusting a tag.
const TAG_SFLP_GAME_ROTATION: u8 = 0x13;
/// One FIFO word: a tag byte and six bytes of payload. Six is what this module batches —
/// see [`Lsm6dsv16x::configure`] for why nothing else is in there.
const FIFO_WORD_LEN: usize = 7;
/// Where the tag sits in a word: the register is `{unused:1, tag_cnt:2, tag_sensor:5}`.
const FIFO_TAG_SHIFT: u8 = 3;

/// Bytes of quaternion payload in an SFLP game-rotation word (three IEEE halfs).
const QUAT_LEN: usize = 6;

/// `smbus_read_i2c_block_data` and friends are capped at 32 bytes by SMBus itself, and the
/// FIFO can hold far more than that. Reads are chunked; `if_inc` walks the address pointer
/// across the boundary, which is why it is set before anything else.
const MAX_I2C_READ: usize = 32;

// ── the bus, abstracted ───────────────────────────────────────────────────────────

/// A register-addressed I²C device. Abstracted so the driver's logic — the init sequence, the
/// word walk, the reuse of [`SflpDecoder`] — can be exercised with no hardware present, which
/// matters because this code was written before the module it drives was in hand.
pub trait I2c {
    /// Read `buf.len()` bytes starting at `reg`. The device must auto-increment.
    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<()>;
    fn write(&mut self, reg: u8, value: u8) -> Result<()>;
}

// ── the driver ────────────────────────────────────────────────────────────────────

/// What the driver has seen that it could not account for. Counts rather than silence: an
/// unknown FIFO tag means this module's picture of the chip is incomplete, and the only honest
/// thing to do is say so where someone will read it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ImuHealth {
    /// Reads that did not return the requested length.
    pub short_reads: u64,
    /// FIFO words whose tag is not in this module's vocabulary. Non-zero means the FIFO is
    /// carrying something `configure` did not ask for, and the latest quaternion may be stale.
    pub unknown_tags: u64,
    /// Samples where the FIFO held no SFLP quaternion at all.
    pub empty_fifo: u64,
}

pub struct Lsm6dsv16x<I: I2c> {
    i2c: I,
    /// The same decoder the Dynamixel board path uses — see the module docs. Its
    /// `GYRO_RAD_PER_LSB` is why [`FS_G_500_DPS`] is not negotiable.
    decoder: SflpDecoder,
    health: ImuHealth,
}

impl<I: I2c> Lsm6dsv16x<I> {
    /// `mount` is the sensor→trunk rotation, in the same scalar-first form
    /// [`SflpDecoder::new`] takes. The module's own mounting is a property of how it is bolted
    /// to the duck and has not been measured yet; pass
    /// [`SflpDecoder::DEFAULT_MOUNT`] until it has.
    pub fn new(i2c: I, mount: [f64; 4]) -> Self {
        Self {
            i2c,
            decoder: SflpDecoder::new(mount),
            health: ImuHealth::default(),
        }
    }

    pub fn health(&self) -> ImuHealth {
        self.health
    }

    /// Bring the chip up, in the order ST's driver does it.
    ///
    /// **Nothing else is batched into the FIFO on purpose.** Gyro and accel batching stay off
    /// (`FIFO_CTRL3` written to zero) because the gyro is read live from `OUTX_L_G` and the
    /// accel is not read at all — SFLP consumes it inside the chip. That leaves the FIFO
    /// carrying exactly one kind of word, which is what makes
    /// [`newest_quaternion`]'s fixed-stride walk correct rather than a guess.
    pub fn configure(&mut self) -> Result<()> {
        let mut who = [0u8; 1];
        self.i2c.read(WHO_AM_I, &mut who)?;
        if who[0] != WHO_AM_I_LSM6DSV16X {
            return Err(IoError::Bus(format!(
                "LSM6DSV16X: WHO_AM_I is {:#04x}, expected {:#04x} — an LSM6DSV16B answers \
                 {:#04x} and has the same SFLP registers, so a module that is otherwise fine \
                 would land here",
                who[0], WHO_AM_I_LSM6DSV16X, 0x71
            )));
        }

        // Address auto-increment first: every multi-byte read below depends on it, and ST's
        // driver sets it in the same breath as BDU.
        self.i2c.write(CTRL3, CTRL3_IF_INC | CTRL3_BDU)?;

        // Both sensing chains have to run for SFLP to have anything to fuse. High-performance
        // mode is the `op_mode_*` field's zero value, so writing the ODR alone leaves it there.
        self.i2c.write(CTRL1, ODR_120_HZ)?;
        self.i2c.write(CTRL2, ODR_120_HZ)?;
        self.i2c.write(CTRL6, FS_G_500_DPS)?;
        self.i2c.write(CTRL8, FS_XL_4_G)?;

        // SFLP's own registers live on the embedded-function page.
        self.i2c.write(FUNC_CFG_ACCESS, FUNC_CFG_EMB_FUNC_ACCESS)?;
        self.i2c.write(SFLP_ODR, SFLP_ODR_60_HZ << FIFO_TAG_SHIFT)?;
        // Reinitialise before enabling, and read-modify-write both registers: they also hold the
        // pedometer, tilt and significant-motion enables, so a blind write would switch off
        // functions someone turns on later.
        //
        // The reinit is not ceremony. The chip does not lose power when `robotd` restarts, so a
        // filter left running by a previous process would be picked up mid-flight carrying an
        // attitude from before the restart — and `SflpDecoder::ready` would then be counting
        // samples from two different runs. ST's driver exposes this bit for exactly this.
        let mut init_a = [0u8; 1];
        self.i2c.read(EMB_FUNC_INIT_A, &mut init_a)?;
        self.i2c
            .write(EMB_FUNC_INIT_A, init_a[0] | EMB_FUNC_INIT_A_SFLP_GAME)?;
        let mut en_a = [0u8; 1];
        self.i2c.read(EMB_FUNC_EN_A, &mut en_a)?;
        self.i2c
            .write(EMB_FUNC_EN_A, en_a[0] | EMB_FUNC_EN_A_SFLP_GAME)?;
        let mut fifo_en_a = [0u8; 1];
        self.i2c.read(EMB_FUNC_FIFO_EN_A, &mut fifo_en_a)?;
        self.i2c.write(
            EMB_FUNC_FIFO_EN_A,
            fifo_en_a[0] | EMB_FUNC_FIFO_EN_A_SFLP_GAME,
        )?;
        // Leave the way we came in. Every other access after this one assumes the main page.
        self.i2c.write(FUNC_CFG_ACCESS, 0)?;

        // Only now arm the FIFO. Stream mode drops the oldest word when it fills, so a stalled
        // reader costs history rather than freezing the newest sample.
        self.i2c.write(FIFO_CTRL3, 0)?;
        self.i2c.write(FIFO_CTRL4, FIFO_MODE_STREAM)?;

        Ok(())
    }

    /// How many words the FIFO is holding.
    fn fifo_words(&mut self) -> Result<usize> {
        // Occupancy is ten bits: eight in STATUS1, and STATUS2's low bit is the ninth. The
        // rest of STATUS2 is latched flags and the overflow counter — reading it as part of a
        // wider count would over-report and stall the walk.
        //
        // Read as one transaction: `if_inc` carries the address pointer from STATUS1 to
        // STATUS2, which `the_two_fifo_status_registers_are_adjacent` is what holds true.
        let mut status = [0u8; (FIFO_STATUS2 - FIFO_STATUS1 + 1) as usize];
        self.i2c.read(FIFO_STATUS1, &mut status)?;
        Ok(status[0] as usize | (((status[1] & 0x01) as usize) << 8))
    }

    /// Drain the FIFO and keep the newest SFLP quaternion in it.
    ///
    /// Draining matters even though only the newest is wanted: the FIFO is a queue, and a
    /// reader that leaves words behind reads progressively further into the past until it
    /// fills and stream mode starts dropping the ones just written.
    fn newest_quat(&mut self) -> Result<Option<[u8; QUAT_LEN]>> {
        let words = self.fifo_words()?;
        if words == 0 {
            self.health.empty_fifo += 1;
            return Ok(None);
        }

        // Cap what one tick will read. If something has gone wrong and the FIFO is enormous,
        // reading all of it would blow the tick budget — and the newest word is at the end,
        // so a partial read is worse than useless unless it starts at the end. This reads
        // forward and keeps the last one, which needs the whole queue; the cap is therefore
        // generous and the overflow is reported.
        let words = words.min(MAX_FIFO_WORDS_PER_TICK);
        let mut buf = vec![0u8; words * FIFO_WORD_LEN];
        let mut at = 0;
        while at < buf.len() {
            let end = (at + MAX_I2C_READ).min(buf.len());
            self.i2c.read(FIFO_DATA_OUT_TAG, &mut buf[at..end])?;
            at = end;
        }

        let (newest, unknown) = newest_quaternion(&buf);
        self.health.unknown_tags += unknown;
        Ok(newest)
    }
}

/// The most FIFO this will read in one tick. At 60 Hz SFLP and a 50 Hz loop, a healthy queue
/// is one or two words; this is two orders of magnitude of headroom above that, and its only
/// job is to bound a pathological tick.
const MAX_FIFO_WORDS_PER_TICK: usize = 64;

impl<I: I2c + Send> ImuSource for Lsm6dsv16x<I> {
    /// One tick: live gyro, newest fused quaternion, and the same twelve bytes the
    /// `imu_to_dxl` board used to produce.
    fn sample(&mut self) -> Result<ImuData> {
        // Live, not queued — the fall predictor's `ġ = −ω × g` is about *now*, and a batched
        // gyro would be a few milliseconds stale. See the module docs.
        let mut gyro = [0u8; 6];
        self.i2c.read(OUTX_L_G, &mut gyro)?;

        // A tick with no quaternion yet is normal for the first fraction of a second, and is
        // not an error: `SflpDecoder` holds its last value and `ready()` stays false, which is
        // exactly the signal `NoImu` gives and exactly what upstream's board did while its
        // filter converged. Zeros here take that same path.
        let quat = self.newest_quat()?.unwrap_or([0u8; QUAT_LEN]);

        let mut block = [0u8; IMU_BLOCK_LEN];
        block[..6].copy_from_slice(&gyro);
        block[6..].copy_from_slice(&quat);
        Ok(self.decoder.decode(&block))
    }

    fn ready(&self) -> bool {
        self.decoder.ready()
    }
}

/// Walk a FIFO fetch and return the newest SFLP quaternion, plus how many words carried a tag
/// this module does not know.
///
/// Pure, and deliberately strict: a word of an unrecognised tag has a length this module cannot
/// know, so the walk **stops** there rather than stepping past it by the assumed seven bytes
/// and reading every subsequent word out of alignment. Reporting the unknown and returning what
/// was found is the honest failure; silently inventing a stride would turn one odd word into a
/// stream of plausible nonsense.
fn newest_quaternion(fifo: &[u8]) -> (Option<[u8; QUAT_LEN]>, u64) {
    let mut newest = None;
    let mut unknown = 0u64;
    let mut at = 0;
    while at + FIFO_WORD_LEN <= fifo.len() {
        let tag = fifo[at] >> FIFO_TAG_SHIFT;
        if tag != TAG_SFLP_GAME_ROTATION {
            unknown += 1;
            break;
        }
        let mut q = [0u8; QUAT_LEN];
        q.copy_from_slice(&fifo[at + 1..at + 1 + QUAT_LEN]);
        newest = Some(q);
        at += FIFO_WORD_LEN;
    }
    (newest, unknown)
}

// ── the real bus ──────────────────────────────────────────────────────────────────

/// A `/dev/i2c-*` device. Linux only, and gated rather than portable because there is nowhere
/// else for this to run: the same boundary `tof/src/imu.rs` draws for the head IMU, and the
/// reason `cargo test --workspace` still has to pass on a machine with no I²C at all.
///
/// The driver logic above is not gated, so every test in this file runs everywhere.
#[cfg(target_os = "linux")]
pub mod linux {
    use super::{I2c, Result, MAX_I2C_READ};
    use crate::io::IoError;
    use i2cdev::core::I2CDevice;
    use i2cdev::linux::LinuxI2CDevice;

    pub struct LinuxI2c {
        dev: LinuxI2CDevice,
    }

    impl LinuxI2c {
        /// Open `path` (e.g. `/dev/i2c-1`) and address `addr` (`0x6A` or `0x6B` — the module's
        /// SA0 strap decides, so try both rather than trusting a datasheet default).
        pub fn open(path: &str, addr: u8) -> Result<Self> {
            // `LinuxI2CError` carries the errno; `IoError::Port` wants an `io::Error`, and the
            // crate provides the conversion.
            let dev = LinuxI2CDevice::new(path, addr as u16).map_err(|e| IoError::Port {
                path: path.to_owned(),
                source: e.into(),
            })?;
            Ok(Self { dev })
        }
    }

    impl I2c for LinuxI2c {
        fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<()> {
            // `smbus_read_i2c_block_data` issues the write-then-read as one transaction with a
            // repeated start, which is what the register-addressed read wants. SMBus caps the
            // payload at 32 bytes, so anything longer is chunked — `if_inc` carries the address
            // pointer across the boundary.
            let mut at = 0;
            while at < buf.len() {
                let end = (at + MAX_I2C_READ).min(buf.len());
                let chunk = self
                    .dev
                    .smbus_read_i2c_block_data(reg, (end - at) as u8)
                    .map_err(|e| IoError::Bus(format!("i2c read {reg:#04x}: {e}")))?;
                if chunk.len() != end - at {
                    return Err(IoError::ShortRead {
                        what: "i2c block read",
                        expected: end - at,
                        got: chunk.len(),
                    });
                }
                buf[at..end].copy_from_slice(&chunk);
                at = end;
            }
            Ok(())
        }

        fn write(&mut self, reg: u8, value: u8) -> Result<()> {
            self.dev
                .smbus_write_byte_data(reg, value)
                .map_err(|e| IoError::Bus(format!("i2c write {reg:#04x}: {e}")))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Records what was written and answers reads from a script, so the init sequence and the
    /// word walk can be checked without a chip. Deliberately dumb: a real register file would
    /// be a second implementation of the thing under test.
    #[derive(Default)]
    struct FakeI2c {
        writes: Vec<(u8, u8)>,
        words: Vec<u8>,
        gyro: [u8; 6],
        who: u8,
    }

    impl I2c for FakeI2c {
        fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<()> {
            match reg {
                WHO_AM_I => buf[0] = self.who,
                OUTX_L_G => buf.copy_from_slice(&self.gyro),
                FIFO_STATUS1 => {
                    let words = self.words.len() / FIFO_WORD_LEN;
                    buf[0] = words as u8;
                    buf[1] = ((words >> 8) & 0x01) as u8;
                }
                FIFO_DATA_OUT_TAG => {
                    let n = buf.len().min(self.words.len());
                    buf[..n].copy_from_slice(&self.words[..n]);
                }
                // Embedded-page read-modify-write targets.
                _ => buf.fill(0),
            }
            Ok(())
        }
        fn write(&mut self, reg: u8, value: u8) -> Result<()> {
            self.writes.push((reg, value));
            Ok(())
        }
    }

    fn word(tag: u8, payload: [u8; QUAT_LEN]) -> Vec<u8> {
        let mut w = vec![tag << FIFO_TAG_SHIFT];
        w.extend_from_slice(&payload);
        w
    }

    /// A quaternion the chip could plausibly emit: x = 0.125 in half precision.
    const HALF_0125: [u8; 2] = [0x00, 0x30];

    fn payload() -> [u8; QUAT_LEN] {
        [HALF_0125[0], HALF_0125[1], 0, 0, 0, 0]
    }

    #[test]
    fn a_wrong_part_is_named_rather_than_driven() {
        let mut dev = Lsm6dsv16x::new(
            FakeI2c { who: 0x6B, ..Default::default() },
            SflpDecoder::DEFAULT_MOUNT,
        );
        let err = dev.configure().unwrap_err().to_string();
        // 0x6B is the ISM330DHCX's WHO_AM_I — the value the project's own bench script had
        // wrong, so the message names both parts a reader might actually be holding.
        assert!(err.contains("0x6b"), "{err}");
        assert!(err.contains("0x70"), "{err}");
        assert!(err.contains("0x71"), "{err}");
    }

    /// `fifo_words` reads occupancy as two bytes from `FIFO_STATUS1`, which is only correct
    /// while `FIFO_STATUS2` sits immediately after it. Nothing else in the crate would notice
    /// if a future revision moved one, and the failure would be a plausible-looking occupancy
    /// count rather than an error — so the adjacency is asserted rather than assumed.
    #[test]
    fn the_two_fifo_status_registers_are_adjacent() {
        assert_eq!(FIFO_STATUS2, FIFO_STATUS1 + 1);
    }

    #[test]
    fn the_init_sequence_sets_auto_increment_before_any_multi_byte_read() {
        let mut dev = Lsm6dsv16x::new(FakeI2c { who: 0x70, ..Default::default() },
                                     SflpDecoder::DEFAULT_MOUNT);
        dev.configure().unwrap();
        let writes = &dev.i2c.writes;

        let ctrl3 = writes
            .iter()
            .position(|(r, _)| *r == CTRL3)
            .expect("CTRL3 is written");
        assert_eq!(writes[0].0, CTRL3, "auto-increment goes first, nothing else");
        assert_eq!(writes[ctrl3].1 & CTRL3_IF_INC, CTRL3_IF_INC);

        // The embedded page is opened and closed exactly once each — a page left open would
        // make every later read of a main-page address land somewhere else entirely.
        let opens = writes
            .iter()
            .filter(|(r, v)| *r == FUNC_CFG_ACCESS && *v == FUNC_CFG_EMB_FUNC_ACCESS)
            .count();
        let closes = writes
            .iter()
            .filter(|(r, v)| *r == FUNC_CFG_ACCESS && *v == 0)
            .count();
        assert_eq!((opens, closes), (1, 1), "page opened {opens}, closed {closes}");
    }

    #[test]
    fn the_gyro_scale_is_the_one_the_decoder_assumes() {
        // `SflpDecoder` scales raw counts at 17.5 mdps/LSB, which is the ±500 dps range. Configuring
        // any other range here would mis-scale every rate the policy sees, and no test in the
        // decoder could notice — it never talks to the chip.
        let mut dev = Lsm6dsv16x::new(FakeI2c { who: 0x70, ..Default::default() },
                                     SflpDecoder::DEFAULT_MOUNT);
        dev.configure().unwrap();
        let fs_g = dev
            .i2c
            .writes
            .iter()
            .find(|(r, _)| *r == CTRL6)
            .map(|(_, v)| *v)
            .expect("CTRL6 is written");
        assert_eq!(fs_g & 0x0F, FS_G_500_DPS);
    }

    #[test]
    fn the_newest_word_wins_not_the_first() {
        let mut dev = Lsm6dsv16x::new(
            FakeI2c {
                who: 0x70,
                gyro: [1, 0, 2, 0, 3, 0],
                words: [word(TAG_SFLP_GAME_ROTATION, [0, 0, 0, 0, 0, 0]),
                        word(TAG_SFLP_GAME_ROTATION, payload())].concat(),
                ..Default::default()
            },
            SflpDecoder::DEFAULT_MOUNT,
        );
        dev.configure().unwrap();
        let out = dev.sample().unwrap();
        // The second word's x = 0.125 came through, not the first word's zero.
        assert_ne!(out.quat, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(dev.health().unknown_tags, 0);
    }

    #[test]
    fn an_empty_fifo_is_a_sample_not_a_failure() {
        let mut dev = Lsm6dsv16x::new(
            FakeI2c { who: 0x70, gyro: [0, 0, 0, 0, 0, 0], ..Default::default() },
            SflpDecoder::DEFAULT_MOUNT,
        );
        dev.configure().unwrap();
        // Startup: SFLP has not written anything yet. This must be an ordinary tick that
        // reports "not ready", never an error — `robotd`'s first ticks are exactly this.
        let out = dev.sample().unwrap();
        assert_eq!(out.gravity, [0.0, 0.0, -1.0]);
        assert!(!dev.ready());
        assert_eq!(dev.health().empty_fifo, 1);
    }

    #[test]
    fn not_ready_until_the_chip_has_fused_something() {
        let mut dev = Lsm6dsv16x::new(
            FakeI2c {
                who: 0x70,
                ..Default::default()
            },
            SflpDecoder::DEFAULT_MOUNT,
        );
        dev.configure().unwrap();
        for _ in 0..25 {
            dev.i2c.words = word(TAG_SFLP_GAME_ROTATION, payload());
            dev.sample().unwrap();
        }
        assert!(dev.ready(), "25 live quaternions is the decoder's threshold");
    }

    #[test]
    fn an_unknown_tag_stops_the_walk_instead_of_drifting() {
        // A tag this module has no length for. Stepping past it by the assumed seven bytes
        // would read the rest of the buffer out of alignment and invent quaternions.
        let buf = [
            word(TAG_SFLP_GAME_ROTATION, payload()),
            word(0x1A, [0; 6]), // MLC_RESULT_TAG — never batched here
            word(TAG_SFLP_GAME_ROTATION, payload()),
        ]
        .concat();
        let (newest, unknown) = newest_quaternion(&buf);
        assert!(newest.is_some(), "the word before the unknown one is still usable");
        assert_eq!(unknown, 1);
    }

    #[test]
    fn fifo_occupancy_uses_only_the_bits_that_are_occupancy() {
        // STATUS2's low bit is the ninth occupancy bit; the rest is latched flags and the
        // overflow counter. Reading the whole byte as a count over-reports and stalls the walk.
        let mut dev = Lsm6dsv16x::new(
            FakeI2c { who: 0x70, words: vec![0; FIFO_WORD_LEN * 3], ..Default::default() },
            SflpDecoder::DEFAULT_MOUNT,
        );
        assert_eq!(dev.fifo_words().unwrap(), 3);
    }
}
