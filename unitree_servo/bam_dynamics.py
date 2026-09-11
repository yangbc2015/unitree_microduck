#!/usr/bin/env python3
"""S288 dynamics identification: command delay, drivetrain compliance/backlash, inertia.

Sibling of bam_friction.py — same rule: everything comes from the servo's own feedback,
no instruments. The measured friction (bam_friction.py) is what makes the inertia test
possible: J = (tau_applied - F(v)) / alpha, and F(v) is now known.

Sub-commands
  delay       command -> torque-applied latency, using a torque step SMALLER than the
              breakaway torque so the shaft never moves (isolates the transport/processing
              delay from any mechanical response)
  compliance  wind-up vs torque: apply +/-tau below breakaway and compare the output angle
              implied by the rotor encoder with the independent output-side encoder.
              Gives the drivetrain torsional stiffness and the hysteresis gap
  backlash    approach the same position from both directions, settled, and compare the
              two encoders -- the classic M7 test, done with the settling the first pass
              was missing
  inertia     torque step -> angular acceleration; J = (tau - F(v)) / alpha

Common trap inherited from bam_friction.py: the geared drivetrain stores elastic energy.
Always recentre and sit at zero torque before a measurement, or the unwind is read as motion.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import time
from datetime import datetime
from typing import NamedTuple

import serial

from unitree_servo import (
    RATIO, build_control_packet, parse_feedback_packet,
)

BAUD = 6_000_000
DEFAULT_PORT = "/dev/unitree_servo"
OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bam_data")

TOR_COUNTS_PER_NM = 256000.0          # rotor N.m
POS_COUNTS_PER_ROT_RAD = 32768.0 / (2 * math.pi)
OUTPOS_COUNTS_PER_TURN = 8192.0

# Fitted in bam_fit.py from the paired velocity sweep (output-side N.m). Used only to
# subtract friction in the inertia fit; the fit is self-consistent in reported-torque units.
FRICTION = dict(Fc=0.01870, Fs=0.014193, vs=0.009459, alpha=3.4775, Fv=0.010859)


def friction_nm(v_out: float) -> float:
    a = abs(v_out)
    return FRICTION["Fc"] + FRICTION["Fs"] * math.exp(
        -(a / FRICTION["vs"]) ** FRICTION["alpha"]) + FRICTION["Fv"] * a


def tor_out(counts: float) -> float:
    return counts / TOR_COUNTS_PER_NM * RATIO


def out_pos(fb) -> float:
    return fb["outputPos"]


class Link:
    """Blocking request/response with per-frame timestamps."""

    def __init__(self, port, timeout=0.05):
        self.ser = serial.Serial(port, BAUD, timeout=timeout)

    def frame(self, mid, mode, tmo, tor, spd, pos, kp, kd):
        cmd = build_control_packet(mid, mode, tmo, tor, spd, pos, kp, kd)
        t0 = time.perf_counter()
        self.ser.write(cmd)
        buf = b""
        while len(buf) < 26:
            chunk = self.ser.read(26 - len(buf))
            if not chunk:
                break
            buf += chunk
        t1 = time.perf_counter()
        fb = parse_feedback_packet(buf) if len(buf) == 26 else None
        return t0, t1, fb

    def close(self):
        self.ser.close()


def classify(fb) -> dict:
    """feedback dict -> the raw-and-physical pair we log"""
    return dict(pos_raw=fb["pos_raw"], tor_raw=fb["torque_raw"], spd_raw=fb["speed_raw"],
                out_pos=fb["outputPos"], ex_pos=fb["ExPos"], tor_out=fb["outputTor"],
                spd_out=fb["outputSpd"], vol=fb["vol"], temp=fb["Temp"],
                merr=fb["MError"], exflag=fb["MWarn"], tmo=fb["timeout"])


class Log:
    def __init__(self, name):
        os.makedirs(OUTDIR, exist_ok=True)
        self.path = os.path.join(OUTDIR, f"{name}_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self.fh = open(self.path, "w", newline="")
        self.w = csv.writer(self.fh)

    def header(self, cols):
        self.w.writerow([f"# {c}" for c in cols])

    def row(self, *vals):
        self.w.writerow(vals)

    def close(self):
        self.fh.close()
        return self.path


def settle(m, mid, p_ref, kp=5.0, kd=1.0, secs=1.5, quiet=0.8):
    """Slow move back to p_ref, then release to zero torque so the wind-up unwinds."""
    t_end = time.perf_counter() + secs
    while time.perf_counter() < t_end:
        m.frame(mid, 1, 0, 0.0, 0.0, p_ref, kp, kd)
    t_end = time.perf_counter() + quiet
    fb = None
    while time.perf_counter() < t_end:
        _, _, fb = m.frame(mid, 1, 0, 0.0, 0.0, 0.0, 0.0, kd)
    return classify(fb)


def park(m, mid):
    for _ in range(3):
        try:
            m.frame(mid, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
        time.sleep(0.02)


# ---------------------------------------------------------------- delay (M2)

class DelayFit(NamedTuple):
    td: float                       # transport delay, ms
    tau: float                      # first-order rise time constant, ms
    A: float                        # fitted step amplitude, torque counts
    rms: float                      # residual, torque counts
    n: int                          # samples fitted
    td_band: tuple[float, float] | None   # td values whose SSE stays within 10%


def fit_delay_response(T, Y, step_n) -> DelayFit | None:
    """Fit  y = A*(1 - exp(-(t-td)/tau)) for t>td  to raw (t, y) samples.

    A is linear given (td, tau), so the grid is only over those two. Returns the best
    parameters, the rms residual, and the td band whose SSE stays within 10% of the
    minimum (how sharply the delay is actually pinned down).
    """
    import numpy as np
    T = np.asarray(T, dtype=float)
    Y = np.asarray(Y, dtype=float)
    best = None
    for td in np.arange(0.0, 1.5, 0.01):
        x = T - td
        lag = np.where(x > 0, 1.0, 0.0)
        for tau in np.arange(0.03, 1.2, 0.01):
            f = np.where(x > 0, (1.0 - np.exp(-x / tau)) * lag, 0.0)
            den = float(f @ f)
            if den <= 0:
                continue
            A = float(f @ Y) / den
            sse = float(((Y - A * f) ** 2).sum())
            if best is None or sse < best[0]:
                best = (sse, td, tau, A)
    if best is None:
        return None
    sse, td, tau, A = best
    band = []
    for td2 in np.arange(max(0.0, td - 0.5), td + 0.5, 0.01):
        x = T - td2
        f = np.where(x > 0, 1.0 - np.exp(-x / tau), 0.0)
        den = float(f @ f)
        A2 = float(f @ Y) / den if den > 0 else 0.0
        if float(((Y - A2 * f) ** 2).sum()) < sse * 1.10:
            band.append(td2)
    return DelayFit(float(td), float(tau), float(A), math.sqrt(sse / len(T)), len(T),
                    (float(min(band)), float(max(band))) if band else None)


class SineFit(NamedTuple):
    J: float
    r2: float
    amp: float
    drift: float
    alpha_rms: float
    v_rms: float
    resid_rms: float
    n: int


def fit_sine_inertia(t, pos, tor, w) -> SineFit:
    """Fit J from a sinusoidally driven record:  J = (tau - F(v)) / alpha.

    alpha and v come from fitting the recorded position to sin/cos of the *known* drive
    frequency plus a drift term -- far less noisy than double-differencing the encoder.
    F(v) is the measured friction model, so this only works once friction is known.
    """
    import numpy as np
    t = np.asarray(t, dtype=float)
    pos = np.asarray(pos, dtype=float)
    tor = np.asarray(tor, dtype=float)
    M = np.column_stack([np.ones_like(t), np.sin(w * t), np.cos(w * t), t])
    (c0, c1, c2, c3), *_ = np.linalg.lstsq(M, pos, rcond=None)
    alpha = -(w * w) * (c1 * np.sin(w * t) + c2 * np.cos(w * t))
    v = w * (c1 * np.cos(w * t) - c2 * np.sin(w * t)) + c3
    y = tor - np.array([friction_nm(x) for x in v])
    J = float(alpha @ y) / float(alpha @ alpha)
    resid = y - J * alpha
    ss = float(((y - y.mean()) ** 2).sum())
    return SineFit(J, 1 - float(resid @ resid) / ss if ss > 0 else float("nan"),
                   math.hypot(c1, c2), float(c3),
                   float(np.sqrt((alpha ** 2).mean())), float(np.sqrt((v ** 2).mean())),
                   math.sqrt(float(resid @ resid) / len(resid)), len(t))


def cmd_delay(m, a):
    """Torque step, magnitude below breakaway -> the shaft cannot move, so what we time
    is the command reaching the servo and showing up in its own reported torque.

    Two different quantities, and the first run showed they are not the same:
      * when the reported torque first departs from the previous steady value
        -> transport + processing (the "command_delay" a sim wants)
      * when it reaches the new value -> the actuator's own torque rise
    The trace is logged frame by frame so both can be fitted, not just detected."""
    log = Log(f"M2_delay_id{a.id}")
    log.header(["trial", "sign", "k", "want_out", "tor_raw", "t_rel_ms", "frame_ms"])
    tau = a.tau_delay                       # output N.m, must stay < breakaway
    n_hold, n_tail = a.hold_frames, a.tail_frames
    print(f"command delay: tor +{tau} -> -{tau} N.m (below the {a.breakaway} N.m breakaway "
          f"torque, so nothing moves), {a.trials} trials, "
          f"{n_hold} frames before / {n_hold + n_tail} after the step")
    traces = []
    for trial in range(a.trials):
        for sign in (+1, -1):
            rec = []
            for k in range(n_hold + n_hold + n_tail):
                want = (tau if k < n_hold else -tau) * sign
                t0, t1, fb = m.frame(a.id, 1, 0, want, 0.0, 0.0, 0.0, 0.0)
                if fb is None:
                    continue
                d = classify(fb)
                rec.append((k, want, d["tor_raw"], t0, t1))
            if len(rec) > n_hold + 8:
                # timestamps are relative to the send of the first post-step frame
                t_step = next(t0 for k, _, _, t0, _ in rec if k == n_hold)
                rec = [(k, w, t, (t1 - t_step) * 1e3, (t1 - t0) * 1e3)
                       for k, w, t, t0, t1 in rec]
                traces.append((trial, sign, rec))
                for k, w, tor, tr_, fm in rec:
                    log.row(trial, sign, k, f"{w:+.5f}", tor, f"{tr_:.4f}", f"{fm:.4f}")
    log.close()
    if not traces:
        print("  no usable traces")
        return

    cadence = statistics.median(f[4] for _, _, r in traces for f in r)
    print(f"\n  frame cadence: {cadence:.4f} ms  ({1e3/cadence:.0f} Hz)")

    def analyse(rec):
        base = statistics.fmean(f[2] for f in rec if n_hold - 6 <= f[0] < n_hold)
        final = statistics.fmean(f[2] for f in rec if f[0] >= rec[-1][0] - 4)
        span = final - base
        if abs(span) < 8:
            return None
        t_start = t_90 = None
        for k, want, tor, tr_, _ in rec:
            if k < n_hold:
                continue
            if t_start is None and abs(tor - base) > 3:
                t_start = tr_
            if t_start is not None and t_90 is None and abs((tor - base) / span) >= 0.9:
                t_90 = tr_
                break
        if t_start is None or t_90 is None:
            return None
        return base, final, t_start, t_90

    stats = [s for s in (analyse(r) for _, _, r in traces) if s]
    if not stats:
        print("  no complete transitions -- raise --tail-frames")
        return
    spans = [abs(s[1] - s[0]) for s in stats]
    starts = [s[2] for s in stats]
    t90s = [s[3] for s in stats]
    print(f"  step seen by the feedback: {statistics.fmean(spans):.1f} counts = "
          f"{statistics.fmean(spans)/256000*RATIO*1000:.2f} mN.m output "
          f"(commanded {2*a.tau_delay*1000:.0f} mN.m -- so the reported torque tracks the "
          f"command to within {abs(statistics.fmean(spans)/256000*RATIO-2*a.tau_delay)*1000:.2f} mN.m)")
    print(f"  first departure  : min {min(starts):.4f}  median {statistics.median(starts):.4f}"
          f"  mean {statistics.fmean(starts):.4f}  max {max(starts):.4f}  ms"
          f"   (one frame = {cadence:.3f} ms)")
    print(f"  reaches 90%      : min {min(t90s):.4f}  median {statistics.median(t90s):.4f}"
          f"  mean {statistics.fmean(t90s):.4f}  max {max(t90s):.4f}  ms")

    # Fit the response instead of binning it. The frames land at nearly the same phase
    # relative to the step on every trial (frame interval ~8.1 bins), so a histogram
    # aliases into a comb; a 3-parameter fit on all raw points does not care about phase.
    # Model: y = 0 for t < td, then A*(1 - exp(-(t-td)/tau)).
    import numpy as np
    T, Y = [], []
    fitted = [(t, s) for t, s in zip(traces, stats) if s]
    for (_, _, rec), s in fitted:
        d = 1.0 if s[1] > s[0] else -1.0        # the +phase steps down, the -phase steps up
        for k, want, tor, tr_, _ in rec:        # normalise so every transition is a rise
            if -0.4 <= tr_ <= 2.5:
                T.append(tr_)
                Y.append((tor - s[0]) * d)
    T = np.array(T)
    Y = np.array(Y)
    step_n = 2 * a.tau_delay / RATIO * 256000
    fit = fit_delay_response(T, Y, step_n)
    if fit is None:
        print("  fit failed")
        return
    print(f"\n  fit on {fit.n} raw samples from {len(fitted)} transitions")
    print(f"    transport delay td  = {fit.td:.3f} ms"
          + (f"   (within 10% of best: {fit.td_band[0]:.2f}-{fit.td_band[1]:.2f} ms)"
             if fit.td_band else ""))
    print(f"    torque rise tau     = {fit.tau:.3f} ms   -> 10-90% rise {2.2*fit.tau:.3f} ms")
    print(f"    amplitude A         = {fit.A:.2f} counts = {fit.A/256000*RATIO*1000:.2f} mN.m "
          f"output (commanded {2*a.tau_delay*1000:.0f} mN.m)")
    print(f"    residual rms        = {fit.rms:.2f} counts (feedback noise is ~1 count)")
    print("  -> a first-order torque rise of this shape is what a BAM actuator model needs; "
          "the delay is the transport part in front of it.")
    print(f"\n  raw: {log.path}")
    print("  NOTE: measured host-side, so it includes USB-CDC and scheduling jitter. "
          "That is the end-to-end figure a sim wants, not a firmware-only constant.")


# ---------------------------------------------------------------- compliance / backlash

def cmd_compliance(m, a):
    """Wind-up vs torque. Both encoders are read at the same instant: the rotor one says
    where the output *should* be (rotor/RATIO), the output one says where it is. The gap is
    drivetrain compliance + tooth play. Below breakaway, so the output shaft never turns."""
    log = Log(f"M7_compliance_id{a.id}")
    log.header(["phase", "tau_cmd_out", "tau_rep_out", "delta_rot_out", "delta_ex_pos",
                "gap_rad", "gap_deg", "tor_raw"])
    taus = []
    t = a.step
    while t <= a.tmax + 1e-9:
        taus.append(round(t, 5))
        t += a.step
    seq = [0.0] + taus + [0.0] + [-x for x in taus] + [0.0]
    print(f"compliance/hysteresis: sweep tau 0 -> +{a.tmax} -> 0 -> -{a.tmax} -> 0 N.m in "
          f"{a.step} steps (all below breakaway {a.breakaway} N.m, shaft stays put)")
    fb = m.frame(a.id, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0)[2]
    p_ref = out_pos(fb)
    base_rot = base_ex = None
    pts = []
    for i, tau in enumerate(seq):
        t_end = time.perf_counter() + a.hold
        rec = []
        while time.perf_counter() < t_end:
            _, _, fb = m.frame(a.id, 1, 0, tau, 0.0, 0.0, 0.0, 0.0)
            if fb:
                d = classify(fb)
                rec.append(d)
                log.row(f"t{i:02d}", f"{tau:+.5f}", f"{d['tor_out']:+.6f}", "", "", "", "", d["tor_raw"])
        if not rec:
            continue
        rot = statistics.fmean(d["out_pos"] for d in rec)
        ex = statistics.fmean(d["ex_pos"] for d in rec)
        thr = statistics.fmean(d["tor_out"] for d in rec)
        if base_rot is None and tau == 0.0:
            base_rot, base_ex = rot, ex
        dr = rot - base_rot
        de = ex - base_ex
        gap = dr - de
        pts.append((tau, thr, dr, de, gap))
        print(f"  tau_cmd={tau:+.4f}  reported={thr:+.5f} Nm  rotor->out {dr:+.6f} rad  "
              f"output enc {de:+.6f} rad  gap {gap:+.6f} rad ({math.degrees(gap):+.4f} deg)")
    log.close()
    pos = [(g, t) for t, thr, dr, de, g in pts if t > 0]
    neg = [(g, t) for t, thr, dr, de, g in pts if t < 0]
    if len(pos) > 1 and len(neg) > 1:
        dtau_p = pos[-1][1] - pos[0][1]
        dgap_p = pos[-1][0] - pos[0][0]
        dtau_n = neg[-1][1] - neg[0][1]
        dgap_n = neg[-1][0] - neg[0][0]
        for name, dtau, dgap in (("+", dtau_p, dgap_p), ("-", dtau_n, dgap_n)):
            if abs(dgap) > 1e-6:
                k = dtau / dgap
                print(f"  {name} direction: {dtau:+.5f} Nm over {dgap:+.6f} rad "
                      f"-> k_torsion ~ {k:+.3f} Nm/rad (output side)"
                      f"  [= {k/RATIO/RATIO:.4g} Nm/rad rotor-side, {k:.3f} Nm/rad output]")
    print(f"  backlash estimate (gap at tau=0, +approach minus -approach): "
          f"{pts[0][4]:+.6f} rad")
    print(f"  raw: {log.path}")


def cmd_backlash(m, a):
    """M7 done properly. The gap between the rotor-derived output angle and the output-side
    encoder has two parts, and they have to be separated:
      * a constant offset (encoder zero alignment) -- the same whichever way you arrived
      * a hysteresis (drivetrain play) -- it flips when you reverse
    So: reach each target from below, then from above, settle both times, and compare."""
    log = Log(f"M7_backlash_id{a.id}")
    log.header(["target", "approach", "rot_out_rad", "ex_pos_rad", "gap_rad", "gap_deg"])
    targets = [0.0, a.delta, -a.delta]
    print(f"backlash: for each target, arrive from below and from above "
          f"(via target-/+{a.delta} rad), {a.settle}s settle each time; "
          f"kp={a.kp} kd={a.kd}")

    def goto(tgt, secs):
        t_end = time.perf_counter() + secs
        rec = []
        while time.perf_counter() < t_end:
            _, _, fb = m.frame(a.id, 1, 0, 0.0, 0.0, tgt, a.kp, a.kd)
            if fb:
                rec.append(classify(fb))
        tail = rec[len(rec) // 2:]
        rot = statistics.fmean(d["out_pos"] for d in tail)
        ex = statistics.fmean(d["ex_pos"] for d in tail)
        spd = statistics.fmean(abs(d["spd_out"]) for d in tail)
        return rot, ex, spd, rec[-1]["pos_raw"]

    rows = []
    for tgt in targets:
        for label, pre in (("from_below", tgt - a.delta), ("from_above", tgt + a.delta)):
            goto(pre, a.settle * 0.6)
            rot, ex, spd, praw = goto(tgt, a.settle)
            gap = (rot - ex + math.pi) % (2 * math.pi) - math.pi   # OutPos is single-turn
            print(f"  target {math.degrees(tgt):+8.3f}deg {label:11s} -> rotor "
                  f"{math.degrees(rot):+8.4f}deg  output enc {math.degrees(ex):+8.4f}deg  "
                  f"gap {math.degrees(gap):+7.4f}deg   (|v|={spd:.5f} rad/s)")
            rows.append((tgt, label, rot, ex, gap))
            log.row(f"{tgt:+.5f}", label, f"{rot:.7f}", f"{ex:.7f}",
                    f"{gap:+.7f}", f"{math.degrees(gap):+.4f}")
    log.close()
    print()
    hyst = []
    offs = []
    for tgt in targets:
        b = next((r for r in rows if r[0] == tgt and r[1] == "from_below"), None)
        u = next((r for r in rows if r[0] == tgt and r[1] == "from_above"), None)
        if not b or not u:
            continue
        h = b[4] - u[4]
        hyst.append(abs(h))
        offs.append((b[4] + u[4]) / 2)
        print(f"  target {math.degrees(tgt):+8.3f}deg: hysteresis {math.degrees(h):+7.4f}deg "
              f"({abs(h):+.6f} rad)   offset {math.degrees((b[4]+u[4])/2):+7.4f}deg")
    if hyst:
        print(f"\n  backlash (|hysteresis|): mean {math.degrees(statistics.fmean(hyst)):.4f}deg "
              f"= {statistics.fmean(hyst):.6f} rad, spread "
              f"{min(hyst):.6f}-{max(hyst):.6f} rad")
        print(f"  constant encoder offset: {math.degrees(statistics.fmean(offs)):+.4f}deg")
        print("  (the offset is alignment, the hysteresis is play -- only the second one is "
              "a backlash term in a sim)")
    print(f"  raw: {log.path}")


# ---------------------------------------------------------------- inertia

def cmd_sine(m, a):
    """Independent inertia estimate: drive a position sine and regress tau - F(v) = J*alpha.

    Alpha is obtained by fitting the recorded position to a sinusoid of the *known* drive
    frequency (plus a linear drift term), which is far less noisy than double-differencing
    the encoder. Uses the friction model, so it only works now that F(v) is measured."""
    import numpy as np
    log = Log(f"M6b_inertia_sine_id{a.id}")
    log.header(["t_ms", "pos_out", "tor_out", "spd_out"])
    fb = m.frame(a.id, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0)[2]
    p0 = out_pos(fb)
    w = 2 * math.pi * a.freq
    print(f"sine inertia: pos = {p0:+.4f} + {a.amp} sin(2pi*{a.freq}t), kp={a.kp} kd={a.kd}, "
          f"{a.secs}s")
    print("!! the servo body must be RESTRAINED for this number to mean anything\n")
    rec = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < a.secs:
        now = time.perf_counter() - t0
        tgt = p0 + a.amp * math.sin(w * now)
        _, _, fb = m.frame(a.id, 1, 0, 0.0, 0.0, tgt, a.kp, a.kd)
        if fb:
            d = classify(fb)
            d["t"] = time.perf_counter() - t0
            rec.append(d)
            log.row(f"{d['t']*1e3:.3f}", f"{d['out_pos']:.9f}", f"{d['tor_out']:.7f}",
                    f"{d['spd_out']:.6f}")
    log.close()
    if len(rec) < 200:
        print("  too few frames")
        return
    t = np.array([d["t"] for d in rec])
    p = np.array([d["out_pos"] for d in rec])
    tor = np.array([d["tor_out"] for d in rec])
    fit = fit_sine_inertia(t, p, tor, w)
    print(f"  fitted amplitude {fit.amp:.4f} rad (commanded {a.amp}), "
          f"drift {fit.drift:+.5f} rad/s")
    print(f"  frames {fit.n} over {t[-1]:.2f}s, rms alpha {fit.alpha_rms:.4f} "
          f"rad/s^2, rms v {fit.v_rms:.4f} rad/s")
    print(f"  J_out   = {fit.J:.5f} kg.m^2  (R^2 of the tau-vs-alpha fit: {fit.r2:.4f})")
    print(f"  J_rotor = {fit.J/RATIO/RATIO:.3e} kg.m^2")
    print(f"  residual rms {fit.resid_rms*1000:.3f} mN.m")
    print(f"  raw: {log.path}")


def cmd_inertia(m, a):
    """Torque step, kp=kd=0. The shaft breaks away and accelerates at alpha = (tau -
    F(v))/J, so J falls out once F(v) is known. Several torques, several windows.

    `--vmax` matters: F(v) was fitted over 0.02-1.5 rad/s, and a torque step runs far past
    that within milliseconds (0.12 N.m reaches 13 rad/s inside the record). Windows above
    vmax are skipped, because there J is being computed from an extrapolated friction."""
    log = Log(f"M6b_inertia_id{a.id}")
    log.header(["tau_cmd_out", "window", "v_out", "tor_rep_out", "alpha", "J_out",
                "J_rotor", "n_frames"])
    taus = [float(x) for x in a.taus.split(",")]
    fb = m.frame(a.id, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0)[2]
    p_ref = out_pos(fb)
    print(f"inertia: torque steps {taus} N.m (output side) from a settled, unwound state; "
          f"windows above v={a.vmax} rad/s are discarded (outside the friction fit)")
    print("!! the shaft accelerates during each step -- nothing may be on the horn")
    est = []
    for tau in taus:
        settle(m, a.id, p_ref)
        rec = []
        t_end = time.perf_counter() + a.secs
        p0 = None
        while time.perf_counter() < t_end:
            t0, t1, fb = m.frame(a.id, 1, 0, tau, 0.0, 0.0, 0.0, 0.0)
            if fb is None:
                continue
            d = classify(fb)
            if p0 is None:
                p0 = d["pos_raw"]
            d["t"] = t1
            rec.append(d)
            if abs(d["spd_out"]) > a.vmax or \
               abs((d["pos_raw"] - p0) / POS_COUNTS_PER_ROT_RAD / RATIO) > a.limit:
                break
        if len(rec) < a.win * 2:
            print(f"  tau={tau:+.4f}: too few frames ({len(rec)}), skipped")
            continue
        # find the frame where motion starts
        k0 = next((i for i, d in enumerate(rec)
                   if abs(d["pos_raw"] - rec[0]["pos_raw"]) > 5), len(rec))
        if k0 >= len(rec) - a.win - 1:
            print(f"  tau={tau:+.4f}: never broke away, raise the torque")
            continue
        print(f"  tau={tau:+.4f} Nm: motion from frame {k0}, {len(rec)} frames "
              f"({(rec[-1]['t']-rec[0]['t'])*1e3:.0f} ms, |v| <= {a.vmax})")

        js = []
        for s in range(k0, len(rec) - a.win, a.win):
            w = rec[s:s + a.win]
            ts = [d["t"] - w[0]["t"] for d in w]
            ps = [d["out_pos"] for d in w]
            # quadratic fit pos = a + b t + c t^2  ->  alpha = 2c
            n = len(ts)
            st = sum(ts); st2 = sum(x * x for x in ts); st3 = sum(x ** 3 for x in ts)
            st4 = sum(x ** 4 for x in ts); sp = sum(ps); stp = sum(x * y for x, y in zip(ts, ps))
            st2p = sum(x * x * y for x, y in zip(ts, ps))
            # solve 3x3 by hand (small, and scipy is not a dependency of this folder)
            import numpy as np
            A = np.array([[n, st, st2], [st, st2, st3], [st2, st3, st4]], dtype=float)
            b = np.array([sp, stp, st2p], dtype=float)
            try:
                c2 = np.linalg.solve(A, b)[2]
            except np.linalg.LinAlgError:
                continue
            alpha = 2 * c2
            v_mean = (ps[-1] - ps[0]) / (ts[-1] - ts[0])
            tor_rep = statistics.fmean(d["tor_out"] for d in w)
            j = (tor_rep - friction_nm(v_mean)) / alpha if abs(alpha) > 1e-3 else float("nan")
            js.append(j)
            log.row(f"{tau:+.5f}", f"{s}-{s+a.win}", f"{v_mean:+.6f}", f"{tor_rep:+.6f}",
                    f"{alpha:+.5f}", f"{j:.6f}", f"{j/RATIO/RATIO:.3e}", n)
        if js:
            good = [j for j in js if j == j and j > 0]
            if good:
                med = statistics.median(good)
                est.append(med)
                print(f"      J_out = {med:.5f} kg.m^2 (median of {len(good)} windows, "
                      f"spread {min(good):.4f}-{max(good):.4f})  "
                      f"-> J_rotor = {med/RATIO/RATIO:.3e} kg.m^2")
    if est:
        med = statistics.median(est)
        print(f"\n  across torque steps: J_out = {med:.5f} kg.m^2 "
              f"(spread {min(est):.4f}-{max(est):.4f})")
        print(f"  rotor equivalent   : J_rotor = J_out / RATIO^2 = {med/RATIO/RATIO:.3e} kg.m^2")
        print(f"  (RATIO^2 = {RATIO*RATIO:.1f})")
        print("  Caveat: this is the joint-space inertia of THIS assembly (reflected rotor "
              "inertia + whatever is on the shaft), and it is only meaningful with the body "
              "restrained. Re-run with the body clamped and compare -- if the number moves, "
              "the free-body reaction was in the path.")
        print(f"  raw: {log.path}")
    else:
        print("  no usable windows -- raise --taus or lower --limit")
    park(m, a.id)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["delay", "compliance", "backlash", "inertia", "sine"])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--breakaway", type=float, default=0.0365,
                    help="measured stiction, output N.m (bam_friction.py breakaway)")
    ap.add_argument("--tau-delay", type=float, default=0.02)
    ap.add_argument("--trials", type=int, default=25)
    ap.add_argument("--hold-frames", type=int, default=20)
    ap.add_argument("--tail-frames", type=int, default=30,
                    help="frames logged after the step, per phase")
    ap.add_argument("--tmax", type=float, default=0.032, help="compliance sweep limit")
    ap.add_argument("--step", type=float, default=0.004)
    ap.add_argument("--hold", type=float, default=0.25)
    ap.add_argument("--delta", type=float, default=0.1745, help="backlash step, rad")
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--kp", type=float, default=10.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--taus", default="0.06,0.09,0.12")
    ap.add_argument("--amp", type=float, default=0.25)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--secs", type=float, default=0.25)
    ap.add_argument("--win", type=int, default=120, help="frames per acceleration fit")
    ap.add_argument("--vmax", type=float, default=1.5,
                    help="discard windows above this speed (rad/s) -- the friction fit's range")
    ap.add_argument("--limit", type=float, default=2.0, help="abort a step past this, rad")
    a = ap.parse_args()

    m = Link(a.port)
    print(f"{a.port} @ {BAUD}, id {a.id}")
    try:
        {"delay": cmd_delay, "compliance": cmd_compliance, "backlash": cmd_backlash,
         "inertia": cmd_inertia, "sine": cmd_sine}[a.cmd](m, a)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        park(m, a.id)
        m.close()
        print("servo parked (mode 0)")


if __name__ == "__main__":
    main()
