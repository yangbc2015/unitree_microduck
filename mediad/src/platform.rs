//! Where this board differs from the one upstream targets.
//!
//! Upstream runs on a Radxa Zero 3W: rkisp is the capture node, `rkaiq` owns the 3A engine, and
//! the Rockchip VPU encodes H.264 through `mpph264enc`.
//!
//! On a Jetson Orin Nano the capture path is Argus - `nvarguscamerasrc`, over the CSI port the
//! IMX219 is wired to - the ISP owns exposure and white balance for as long as the stream lives,
//! and **there is no hardware encoder at all**: this JetPack's `nvvideo4linux2` plugin registers
//! `nvv4l2decoder` and nothing else, because the Orin Nano has no NVENC block. `webrtcsink` falls
//! through to `x264enc`, which it already knows how to drive and therefore needs no patch - see
//! `wire_encoder_setup` in [`crate::pipeline`], and note that upstream's patch to
//! `microduck-gst-plugins` exists only because `webrtcsink` does not know `mpph264enc`.
//!
//! Measured here, 720p30, capture through `nvvidconv` and `x264enc`: 300 frames in 11.8 s wall,
//! 5.6 s user + 2.0 s sys, so about 0.65 of a core for the whole chain. Upstream's hardware
//! encoder costs 0.076, so this is roughly eight times the CPU - affordable on six cores at
//! 720p30, and worth measuring again before anyone raises the quality rung.
//!
//! Two consequences shape this file:
//!
//! - **Capture is two elements.** Argus hands back `NV12` in NVMM memory, and everything from the
//!   tee downwards wants an ordinary system-memory buffer, so `nvarguscamerasrc` picks the sensor
//!   mode from the caps it is offered and `nvvidconv` converts. What leaves the bin is
//!   [`crate::pipeline::CAPTURE_FORMAT`], so the detector, the console and the encoder are all
//!   untouched by the difference.
//! - **Nothing needs metering.** [`crate::exposure`] exists because `rkaiq` converges once and
//!   then stops responding; Argus keeps converging for as long as the stream is alive. Running
//!   that loop here would fight the ISP for the same sensor registers, so it is skipped.
//!
//! Nothing here is behind a `cfg`: both boards are `target_os = "linux"`, and a build-time gate
//! cannot tell them apart. The dispatch is at runtime, on whether the Argus element exists.

use anyhow::{Context, Result, anyhow};
use gstreamer as gst;
use gstreamer::prelude::*;

/// Is this the Argus board?
///
/// Keyed on the element rather than on a flag, for the reason a `--camera-device` of
/// `/dev/video0` is not enough: on the Rockchip board that node is rkisp, on this one it is the
/// tegra capture node, and the two need different pipelines around them.
pub fn is_argus() -> bool {
    gst::ElementFactory::find("nvarguscamerasrc").is_some()
}

/// Does the ISP keep exposure and white balance converged by itself?
///
/// True here, and the reason [`crate::exposure`] is not started: Argus keeps metering the scene,
/// where `rkaiq` stops after its first convergence. The starting `--exposure` and
/// `--analogue-gain` are the rkisp V4L2 controls and mean nothing to Argus, which has its own
/// exposure compensation and gain ranges.
pub fn owns_exposure() -> bool {
    is_argus()
}

/// The head camera, through Argus, as one element.
///
/// A bin rather than two elements because the caller links one thing to the tee: the ghost pad
/// carries what the rest of the pipeline already expects, and `bin.static_pad("src")` is where the
/// capture meter attaches, exactly as it does to a `v4l2src`.
///
/// `sensor_id` is the CSI port - 0 for CAM0 - and it is the one thing that cannot be discovered
/// from the topology on this board: Argus numbers the ports where the device tree names them.
pub fn camera_source(sensor_id: u32, width: u32, height: u32, fps: u32) -> Result<gst::Element> {
    let bin = gst::Bin::new();

    let src = gst::ElementFactory::make("nvarguscamerasrc")
        .property("sensor-id", sensor_id as i32)
        .build()
        .map_err(|_| {
            anyhow!(
                "no nvarguscamerasrc; it comes with JetPack and needs the ISP, so a board \
                 without it has no capture path at all"
            )
        })?;

    // **The mode is pinned here rather than by `media-ctl`.** Argus negotiates the sensor mode
    // from the caps it is handed, so asking for 1920x1080 at the configured rate is what makes the
    // sensor leave its 3280x2464 boot mode - upstream's `pin_sensor_mode` does the same job
    // through a subdev ioctl, which exists on rkisp and has no counterpart here. It matters for
    // the same reason: the boot mode caps capture at 21 fps, and `media.video` publishes
    // intrinsics for the pinned mode only.
    let mode = gst::ElementFactory::make("capsfilter")
        .property_from_str(
            "caps",
            &format!(
                "video/x-raw(memory:NVMM),format=NV12,width={width},height={height},framerate={fps}/1"
            ),
        )
        .build()
        .map_err(|_| anyhow!("no capsfilter element; gstreamer core is incomplete"))?;

    let convert = gst::ElementFactory::make("nvvidconv")
        .build()
        .map_err(|_| {
            anyhow!("no nvvidconv; it ships with JetPack and is what leaves NVMM memory")
        })?;

    // What leaves the bin: NVMM in, an ordinary buffer out, in the format the rest of the
    // pipeline is pinned to.
    let out = gst::ElementFactory::make("capsfilter")
        .property_from_str(
            "caps",
            &format!(
                "video/x-raw,format={},width={width},height={height}",
                crate::pipeline::CAPTURE_FORMAT
            ),
        )
        .build()
        .map_err(|_| anyhow!("no capsfilter element; gstreamer core is incomplete"))?;

    bin.add_many([&src, &mode, &convert, &out])
        .context("could not add the Argus capture elements")?;
    gst::Element::link_many([&src, &mode, &convert, &out])
        .context("could not link the Argus capture path")?;

    let pad = out
        .static_pad("src")
        .ok_or_else(|| anyhow!("the capture capsfilter has no src pad"))?;
    let ghost = gst::GhostPad::with_target(&pad).context("could not expose the capture bin")?;
    bin.add_pad(&ghost)
        .context("could not publish the capture bin's src pad")?;

    tracing::info!(
        sensor_id,
        width,
        height,
        fps,
        "head camera through Argus; the ISP owns exposure and white balance"
    );
    Ok(bin.upcast())
}
