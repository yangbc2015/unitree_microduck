#!/usr/bin/env python3
"""Per-unit S288 battery + cross-unit comparison.

One servo, one record. Run this for every unit that comes to the bench:

    python3 bam_unit.py run --id 0 --label S288-01 --voltage 12.0 --ambient 24 \
        --psu "KORAD KA3005P" --operator 你的名字
    python3 bam_unit.py compare

`run` shells out to the very commands `docs/S288_个体差异测试方法.md` lists, so the document
and the tool cannot drift apart, and reads each step's result back from its single
`@@RESULT@@` line instead of parsing prose. Everything it produces lands in
`bam_data/units/<label>/` with a sha256 per artifact, so a record can be checked later
without trusting this script.

Two things the protocol cannot give us, and the record says so explicitly:
  * there is no serial number anywhere in the feedback frame -- identity is the operator's
    label, nothing more
  * there is no current field either, so the M4 current limit needs a supply that displays
    amps (`bam_unit.py limit` takes the reading; the torque side comes from feedback)

`compare` is deliberately about the difference between units AND the difference between
repeats of one unit: unit-to-unit spread means nothing until the within-unit noise floor is
known, which is why the protocol says to run one unit twice before trusting any of it.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import platform
import re
import shutil
import statistics
import subprocess
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "bam_data")
UNITS = os.path.join(DATA, "units")
PY = sys.executable
MARK = "@@RESULT@@"

# Artifact paths in a record are stored relative to the repository root, so they read the same
# way as every other path in this repo. Fall back to this directory if git is unavailable.
ROOT = subprocess.run(["git", "rev-parse", "--show-toplevel"], cwd=HERE, capture_output=True,
                      text=True).stdout.strip() or HERE

# what `run` executes, in order. Each entry: (name, argv-template)
#   {id} {curve} {friction} {label} {stiction} {unitdir} are substituted
# The speed set is fixed here on purpose: units are only comparable if they were excited
# the same way. It is the 9-point set behind s288_bam.json (0.02 < 0.04 < 0.07 < 0.12 <
# 0.2 < 0.35 < 0.6 < 1.0 < 1.5 output rad/s).
SPEEDS = "0.02,0.04,0.07,0.12,0.2,0.35,0.6,1.0,1.5"
BATTERY = [
    ("encoder", ["bam_friction.py", "encoder", "--id", "{id}", "--hold", "0.6"]),
    ("friction_sweep", ["bam_friction.py", "sweep", "--id", "{id}",
                        "--speeds", SPEEDS, "--rep", "2"]),
    ("breakaway", ["bam_friction.py", "breakaway", "--id", "{id}"]),
    ("friction_fit", ["bam_fit.py", "--curve", "{curve}", "--out-dir", "{unitdir}",
                      "--tag", "{label}", "--stiction", "{stiction}"]),
    ("delay", ["bam_dynamics.py", "delay", "--id", "{id}", "--tau-delay", "0.02",
               "--trials", "40", "--hold-frames", "20", "--tail-frames", "40",
               "--friction", "{friction}"]),
    ("compliance", ["bam_dynamics.py", "compliance", "--id", "{id}",
                    "--tmax", "0.032", "--step", "0.004", "--hold", "0.25"]),
    ("backlash", ["bam_dynamics.py", "backlash", "--id", "{id}", "--delta", "0.1745",
                  "--settle", "1.8", "--kp", "10", "--kd", "1.0"]),
    ("inertia", ["bam_dynamics.py", "inertia", "--id", "{id}", "--taus", "0.05,0.06,0.07",
                 "--secs", "0.6", "--win", "20", "--vmax", "1.5", "--friction", "{friction}"]),
]


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def to_repo_path(p):
    """Absolute path from a tool's output -> path relative to the repo root.

    Records mix paths from several scripts; keeping them all relative (and stating the base in
    `paths_relative_to`) means a record stays readable if the tree is moved or cloned.
    """
    if isinstance(p, str) and os.path.isabs(p) and p.startswith(ROOT + os.sep):
        return os.path.relpath(p, ROOT)
    return p


# Steps that consume another step's output. If the upstream result is missing (failed, or
# never run for this unit), the downstream step must not quietly fall back to the built-in
# constants -- J computed against another unit's friction is worse than no J at all.
NEEDS = {"friction_fit": ("friction_sweep",), "inertia": ("friction_fit",)}

PATHISH = ("raw_log", "curve_csv", "png", "json", "markdown")


def keep_raw(path_repo_rel, unitdir):
    """Copy a step's raw log into the unit directory, gzipped, and return its repo path.

    Every step writes its raw log into the shared bam_data/ scratch name, where the next unit's
    run will gzip or overwrite it -- so a record has to carry its own copy or its provenance
    evaporates. mtime=0 keeps the compressed bytes reproducible; the artifacts list hashes this
    copy, so a corrupted or edited log would show up.
    """
    src = os.path.join(ROOT, path_repo_rel)
    if not os.path.exists(src):
        return path_repo_rel
    rawdir = os.path.join(unitdir, "raw")
    os.makedirs(rawdir, exist_ok=True)
    dst = os.path.join(rawdir, os.path.basename(src) + ".gz")
    with open(src, "rb") as fi, gzip.GzipFile(dst, "wb", compresslevel=9, mtime=0) as fo:
        shutil.copyfileobj(fi, fo)
    return os.path.relpath(dst, ROOT)


def step(name, argv, logdir, timeout=900):
    """Run one step, tee its output to a log, return (result, note)."""
    log = os.path.join(logdir, f"{name}.log")
    print(f"  [{name}] {' '.join(argv)}", flush=True)
    t0 = time.time()
    with open(log, "w") as lf:
        p = subprocess.run([PY, os.path.join(HERE, argv[0])] + list(argv[1:]),
                           cwd=HERE, stdout=lf, stderr=subprocess.STDOUT, timeout=timeout)
    dur = time.time() - t0
    text = open(log, errors="replace").read()
    hits = [ln for ln in text.splitlines() if ln.startswith(MARK)]
    if p.returncode != 0:
        return None, f"exit {p.returncode} after {dur:.0f}s -- see {log}"
    if not hits:
        return None, f"no {MARK} line after {dur:.0f}s -- see {log}"
    if len(hits) > 1:
        return None, f"{len(hits)} result lines, expected 1 -- see {log}"
    r = json.loads(hits[0][len(MARK):])
    for k in PATHISH:
        if k in r:
            r[k] = to_repo_path(r[k])
    print(f"  [{name}] ok in {dur:.0f}s -> {json.dumps(r, sort_keys=True)[:150]}", flush=True)
    return r, None


def sub_of(name, r, *path, default=None):
    """Pull a field out of a step's result, tolerating a step that failed or was skipped."""
    cur = r
    for p in path:
        if not isinstance(cur, dict) or p not in cur or cur[p] is None:
            return default
        cur = cur[p]
    return cur


def run(a):
    unitdir = os.path.join(UNITS, a.label)
    logdir = os.path.join(unitdir, "logs")
    os.makedirs(logdir, exist_ok=True)
    print(f"unit {a.label} (motor id {a.id}) -> {unitdir}")
    print("!! nothing may be attached to the horn; the shaft will spin up in the "
          "breakaway and inertia steps")

    steps, failed = {}, []
    friction = None          # this unit's own fit, once we have it
    # A single-step re-run (the usual `--only friction_fit` after a failure) has to take its
    # inputs from the existing record, or it would silently use the built-in constants and
    # produce a fit for a different unit's stiction.
    prev_steps, prev_path = {}, os.path.join(unitdir, "record.json")
    if os.path.exists(prev_path):
        with open(prev_path) as f:
            prev_steps = json.load(f).get("steps", {})
    for name, tmpl in BATTERY:
        if a.skip and name in a.skip:
            print(f"  [{name}] skipped by request")
            continue
        if a.only and name not in a.only:
            continue
        lack = [d for d in NEEDS.get(name, ()) if not (steps.get(d) or prev_steps.get(d))]
        if lack:
            why = f"needs {', '.join(lack)} for this unit, which produced no result"
            failed.append((name, why))
            print(f"  [{name}] NOT RUN: {why}")
            continue
        curve = os.path.join(unitdir, "M6a_friction_curve.csv")
        if friction is None:
            pf = prev_steps.get("friction_fit") or {}
            friction = ({k: pf[k] for k in ("Fc", "Fs", "vs", "alpha", "Fv")}
                        if all(k in pf for k in ("Fc", "Fs", "vs", "alpha", "Fv")) else None)
        sub = {
            "id": str(a.id), "label": a.label, "curve": curve, "unitdir": unitdir,
            "stiction": f"{sub_of('breakaway', steps.get('breakaway') or prev_steps.get('breakaway'), 'onset_out_nm', '+', default=0.0365):.5f}",
            "friction": ",".join(f"{friction[k]:.6g}" for k in
                                 ("Fc", "Fs", "vs", "alpha", "Fv")) if friction
                        else "0.0187,0.014193,0.009459,3.4775,0.010859",
        }
        if name == "friction_fit" and not (steps.get("friction_sweep")
                                           or prev_steps.get("friction_sweep")):
            failed.append((name, "no friction_sweep result to fit"))
            print(f"  [{name}] skipped: {failed[-1][1]}")
            continue
        argv = [x.format(**sub) for x in tmpl]
        try:
            r, note = step(name, argv, logdir)
        except subprocess.TimeoutExpired:
            r, note = None, f"timed out (>{900}s) -- see {logdir}/{name}.log"
            print(f"  [{name}] {note}")
        steps[name] = r
        if note:
            failed.append((name, note))
            continue
        if name == "friction_sweep":
            # bam_friction writes the curve into the shared bam_data/ (same name for every
            # unit at id 0), so lift each unit's copy out before the next unit overwrites it
            src = os.path.join(DATA, f"M6a_friction_curve_id{a.id}.csv")
            if os.path.exists(src):
                with open(src) as si, open(curve, "w") as di:
                    di.write(si.read())
        if name == "friction_fit":
            friction = {k: r[k] for k in ("Fc", "Fs", "vs", "alpha", "Fv")}
        if isinstance(r, dict) and r.get("raw_log"):
            r["raw_log"] = keep_raw(r["raw_log"], unitdir)

    # artifacts: hash everything this run produced, so the record can be verified later
    arts = []
    for dirpath, _, files in os.walk(unitdir):
        for f in sorted(files):
            if f == "record.json":
                continue
            p = os.path.join(dirpath, f)
            arts.append(dict(path=os.path.relpath(p, ROOT), bytes=os.path.getsize(p),
                             sha256=sha256(p)))

    # A record for this label may already exist: the usual reason to be here again is one
    # failed step (`--only`), and clobbering the other steps' results would throw away the
    # run that worked. So merge: new results win, steps this run did not touch survive.
    out = os.path.join(unitdir, "record.json")
    prev = {}
    if os.path.exists(out):
        with open(out) as f:
            prev = json.load(f)
        print(f"  merging into the existing record ({len(prev.get('steps', {}))} steps already "
              f"there, {len(prev.get('steps_failed', []))} previously failed)")
    merged = dict(prev.get("steps", {}))
    merged.update(steps)
    kept_failed = [w for w in prev.get("steps_failed", []) if w.get("step") not in steps]
    all_failed = {w["step"]: w for w in kept_failed}          # one entry per step, newest wins
    all_failed.update({n: dict(step=n, why=w) for n, w in failed})
    all_failed = list(all_failed.values())

    rec = {
        "schema": "s288-unit-record/1",
        "label": a.label,
        "motor_id": a.id,
        "when": datetime.now().isoformat(timespec="seconds"),
        "how_identified": "operator label. The protocol exposes no serial number, so this "
                          "identity is only as good as the sticker someone put on the case.",
        "context": {
            "supply_v": a.voltage, "ambient_c": a.ambient, "psu": a.psu,
            "adapter": "Unitree AT32 single-bus->USB (2e3c:7640)",
            "firmware": a.firmware, "operator": a.operator, "notes": a.notes,
            "torque_source": "servo's own rotor torque feedback, 256000 counts = 1 N.m",
        },
        "steps": merged,
        "steps_failed": all_failed,
        "friction_used_for_inertia": friction or prev.get("friction_used_for_inertia"),
        "artifacts": arts,
        "paths_relative_to": ROOT,
        "software": {
            "git_head": subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=HERE,
                                       capture_output=True, text=True).stdout.strip(),
            "host": platform.node(), "kernel": platform.release(),
        },
    }
    with open(out, "w") as f:
        json.dump(rec, f, indent=2, ensure_ascii=False)

    print(f"\n{len(merged) - len(all_failed)}/{len(merged)} steps have a result; record: {out}")
    if all_failed:
        print("failed steps (recorded, not hidden):")
        for w in all_failed:
            print(f"  {w['step']}: {w['why']}")
    print("\nnow repeat the SAME unit with --label %s-r2 so compare can tell the "
          "within-unit noise floor from real unit-to-unit variation" % a.label)
    return 0 if not all_failed else 1


# ---------------------------------------------------------------- comparison

# (label, how to pull the number out of a record, unit)
METRICS = [
    ("stiction", lambda r: _get(r, "breakaway", "onset_out_nm", "+"), "N.m"),
    ("Fc", lambda r: _get(r, "friction_fit", "Fc"), "N.m"),
    ("Fs", lambda r: _get(r, "friction_fit", "Fs"), "N.m"),
    ("Fv", lambda r: _get(r, "friction_fit", "Fv"), "N.m*s/rad"),
    ("stall plateau", lambda r: _get(r, "friction_fit", "robust", "stall_plateau"), "N.m"),
    ("friction dip", lambda r: _get(r, "friction_fit", "robust", "dip"), "N.m"),
    ("viscous slope", lambda r: _get(r, "friction_fit", "robust", "viscous_slope"), "N.m*s/rad"),
    ("command delay", lambda r: _get(r, "delay", "td_ms"), "ms"),
    ("torque rise tau", lambda r: _get(r, "delay", "tau_ms"), "ms"),
    ("backlash", lambda r: _get(r, "backlash", "hysteresis_rad"), "rad"),
    ("encoder offset", lambda r: _get(r, "backlash", "offset_deg"), "deg"),
    ("stiffness +dir", lambda r: _get(r, "compliance", "stiffness_out_nm_rad", "+"), "N.m/rad"),
    ("armature", lambda r: _get(r, "inertia", "J_out"), "kg.m^2"),
    ("case temp", lambda r: _get(r, "delay", "case_c"), "C"),
    ("supply", lambda r: _get(r, "delay", "supply_v"), "V"),
]


def _get(r, *path):
    cur = r.get("steps", {})
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return None
        cur = cur[p]
    return cur


def load_records(d):
    recs = []
    if not os.path.isdir(d):
        return recs
    for name in sorted(os.listdir(d)):
        p = os.path.join(d, name, "record.json")
        if os.path.exists(p):
            with open(p) as f:
                recs.append(json.load(f))
    return recs


def compare(a):
    recs = load_records(a.dir)
    if not recs:
        sys.exit(f"no records under {a.dir}; run `bam_unit.py run` first")
    labels = [r["label"] for r in recs]
    print(f"{len(recs)} records: {', '.join(labels)}\n")

    # a label repeated (S288-01 and S288-01-r2) is a repeat of the same unit: its difference
    # is the noise floor, and NOTHING below that is a real unit-to-unit difference
    base = {}
    for r in recs:
        b = re.sub(r"-r\d+$", "", r["label"])
        base.setdefault(b, []).append(r)
    repeats = {b: v for b, v in base.items() if len(v) > 1}

    hdr = f"{'metric':<17}{'unit':<10}" + "".join(f"{l:>13}" for l in labels)
    print(hdr)
    print("-" * len(hdr))
    for name, fn, unit in METRICS:
        vals = [fn(r) for r in recs]
        if all(v is None for v in vals):
            continue
        cells = "".join(f"{v:>13.6g}" if isinstance(v, (int, float)) else f"{'--':>13}"
                        for v in vals)
        print(f"{name:<17}{unit:<10}{cells}")
        got = [v for v in vals if isinstance(v, (int, float))]
        if len(got) > 1:
            mean = statistics.fmean(got)
            sd = statistics.pstdev(got)
            print(f"{'':<27}spread {min(got):.6g}..{max(got):.6g}  "
                  f"mean {mean:.6g}  sd {sd:.3g} ({100*sd/mean if mean else 0:.1f}%)")
            for r, v in zip(recs, vals):
                if isinstance(v, (int, float)) and sd > 0 and abs(v - mean) > 2 * sd:
                    print(f"{'':<27}!! {r['label']} is >2sd from the mean "
                          f"({v:.6g}) -- possible real outlier, check its log")

    print("\nwithin-unit repeatability (the floor below which nothing is a difference):")
    if not repeats:
        print("  no repeated labels. Run one unit twice (--label X-r2) before reading any "
              "of the spread above as unit-to-unit variation.")
    for b, rs in repeats.items():
        print(f"  {b}: {len(rs)} runs")
        for name, fn, unit in METRICS:
            vs = [fn(r) for r in rs]
            if all(isinstance(v, (int, float)) for v in vs) and len(vs) > 1:
                d = max(vs) - min(vs)
                rel = 100 * d / statistics.fmean(vs) if statistics.fmean(vs) else 0
                print(f"    {name:<17} {min(vs):.6g} .. {max(vs):.6g} {unit:<10} "
                      f"diff {d:.3g} ({rel:.1f}% of mean)")

    print("\nStribeck identifiability (the Fc/Fs split is weakly constrained by design -- if "
          "corr is near -1, read the robust rows above, not Fc/Fs):")
    for r in recs:
        c = _get(r, "friction_fit", "corr_Fc_Fs")
        rmsv = _get(r, "friction_fit", "rms_residual_nm")
        print(f"  {r['label']:<14} corr(Fc,Fs) {'--' if c is None else f'{c:+.3f}'}"
              f"   rms residual {'--' if rmsv is None else f'{rmsv:.5f}'} N.m")

    print("\ncontext (must match across units for the table above to mean anything):")
    for r in recs:
        c = r["context"]
        print(f"  {r['label']:<14} {c['supply_v']}V  ambient {c['ambient_c']}C  "
              f"psu {c['psu']}  fw {c['firmware']}  {r['when']}")
    if len({r["context"]["supply_v"] for r in recs}) > 1:
        print("  !! supply voltage differs between units -- friction and current limit both "
              "scale with it, so the spread above is partly that")


# ---------------------------------------------------------------- M4 limit

def limit(a):
    """Blocked-shaft torque ramp: the reported torque saturates at the current limit.

    The operator clamps or holds the horn; the torque side comes from the servo's own
    feedback, the current side has to come from the supply display (there is no current
    field in the frame). Stop the ramp as soon as the torque stops rising.
    """
    import bam_dynamics as bd

    m = bd.Link(a.port)
    print(f"{a.port} @ {bd.BAUD}, id {a.id}\nM4 torque limit: BOTH the shaft and the body "
          f"must be held. Ramp 0 -> {a.tmax} N.m in {a.step} N.m steps, {a.hold}s each.")
    print("!! stall is hard on the motor: keep the whole ramp under 5 s per the handbook")
    fb = m.frame(a.id, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0)[2]
    p_ref = fb["outputPos"]
    bd.settle(m, a.id, p_ref)
    rows = []
    t = a.step
    while t <= a.tmax + 1e-9:
        rec = []
        t_end = time.perf_counter() + a.hold
        while time.perf_counter() < t_end:
            _, _, fb = m.frame(a.id, 1, 0, t, 0.0, 0.0, 0.0, 0.0)
            if fb:
                rec.append(bd.classify(fb))
        tor = statistics.fmean(d["tor_out"] for d in rec) if rec else float("nan")
        moved = (statistics.fmean(d["out_pos"] for d in rec) - p_ref) if rec else 0.0
        rows.append((t, tor, moved))
        print(f"  tau_cmd={t:+.4f} Nm -> feedback {tor:+.5f} Nm   moved {moved:+.5f} rad")
        t += a.step
    envv = bd.env(m, a.id)
    bd.park(m, a.id)
    m.close()

    # the limit is where the feedback stops following the command
    tail = [r for r in rows if r[0] > rows[-1][0] - 3 * a.step]
    plateau = statistics.fmean(r[1] for r in tail) if tail else float("nan")
    followed = [r for r in rows if r[1] > 0.9 * r[0]]
    knee = followed[-1][0] if followed else None
    print(f"\n  feedback plateau at the top of the ramp : {plateau:+.5f} N.m")
    print(f"  last command the feedback still followed: {knee} N.m")
    if a.current:
        kt_rotor = plateau / a.current / bd.RATIO
        print(f"  with I = {a.current} A from the supply: kt_rotor ~ {kt_rotor:.3e} N.m/A "
              f"(manual's own stall table implies ~1.06e-3)")
    else:
        print("  pass --current <A> with the supply's reading to get kt out of this")
    print("  NOTE: a hand-held shaft is not rigid -- if the shaft crept during the ramp, "
          "the plateau is a lower bound, not the limit")
    bd.emit_result("limit", **envv, motor_id=a.id, tmax=a.tmax, step=a.step,
                   current_a=a.current, plateau_out_nm=plateau,
                   last_followed_nm=knee, ramp=[[r[0], r[1]] for r in rows])
    return 0


def main():
    ap = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="measure one unit end to end")
    r.add_argument("--id", type=int, default=0)
    r.add_argument("--label", required=True, help="sticker on the case, e.g. S288-01")
    r.add_argument("--voltage", type=float, default=None, help="supply setpoint, V")
    r.add_argument("--ambient", type=float, default=None, help="room temperature, C")
    r.add_argument("--psu", default="unknown")
    r.add_argument("--firmware", default="unknown (needs the Windows GUI)")
    r.add_argument("--operator", default=os.environ.get("USER", "unknown"))
    r.add_argument("--notes", default="")
    r.add_argument("--skip", default="", help="comma-separated step names to skip")
    r.add_argument("--only", default="", help="comma-separated step names to run")
    r.set_defaults(func=run)

    c = sub.add_parser("compare", help="cross-unit table + within-unit repeatability")
    c.add_argument("--dir", default=UNITS)
    c.set_defaults(func=compare)

    l = sub.add_parser("limit", help="M4: blocked-shaft torque ramp (needs a hand or clamp)")
    l.add_argument("--id", type=int, default=0)
    l.add_argument("--port", default="/dev/unitree_servo")
    l.add_argument("--tmax", type=float, default=0.30)
    l.add_argument("--step", type=float, default=0.02)
    l.add_argument("--hold", type=float, default=0.35)
    l.add_argument("--current", type=float, default=None,
                   help="supply current reading at the plateau, A")
    l.set_defaults(func=limit)

    a = ap.parse_args()
    if getattr(a, "skip", ""):
        a.skip = {s.strip() for s in a.skip.split(",") if s.strip()}
    if getattr(a, "only", ""):
        a.only = {s.strip() for s in a.only.split(",") if s.strip()}
    sys.exit(a.func(a) or 0)


if __name__ == "__main__":
    main()
