# What the S288 actually bolts to

Unitree support supplied a mounting model on 2026-09-14 — `舵机S288简化模型.STEP`, SolidWorks 2023,
AP203. This page is what came out of it.

It matters because until then the bracket was being designed against a **catalogue number**. The
S288 is listed as 20 × 34 × 26 mm, which is enough to know it fits where an XL330 was (24 × 33.5 ×
19.5) and **not enough to put a hole anywhere**. "Official STL not published" was the largest
mechanical unknown in this port, and the reason nothing structural had been drawn.

## The short version

| | |
|---|---|
| outline | **26.20 × 34.13 × 20.15 mm** — the catalogue `20 × 34 × 26` is confirmed, axes listed in a different order |
| shaft axis | along **X**, at **(y, z) = (3.59, 0.61)** |
| the holes to design to | **two 6-hole flanges, 6 × Ø1.7 on a Ø10.5 bolt circle, 60° apart**, 15.6 mm apart |
| units | millimetres (`SI_UNIT(.MILLI., .METRE.)`) |
| what it is **not** | a solid — so no interference or section work, and **the output shaft is not in it** |

## It is a surface model, not a solid

`MANIFOLD_SOLID_BREP` appears **zero** times. What the file has is one `SHELL_BASED_SURFACE_MODEL`
holding twelve shells and 707 `ADVANCED_FACE`s. That is SolidWorks' "simplified" export, and the
name on the file is 简化模型 — the simplification is not a surprise, it is the label.

So the distinction this page rests on:

- **Measuring it works**, and is what everything below does.
- **Sectioning it, checking clearance against it, or handing it to a CAE tool does not.** There is
  no enclosed volume, so there is nothing to be inside or outside of.

## Outline, and where the axis sits

Every one of the 121 cylindrical faces is axial along ±X, so the shaft does not wander and the
outline is a box around a prism.

```
X   −15.38 ..  10.82      span 26.20   (axial)
Y   −20.98 ..  13.15      span 34.13
Z    −9.46 ..  10.69      span 20.15
```

**The axis is not on the centreline, and that is the part worth not assuming.** In Z it is dead
centre — 10.07 mm to one face, 10.08 to the other. In Y it is not: **9.56 mm to one side, 24.57 to
the other.** The S288 is a gearbox on one side of a motor, not a symmetric cylinder. A bracket laid
out around the geometric centre would be off by 7.5 mm in Y.

**Body diameters along the axis**, for pocketing and clearance:

| x | Ø |
|---|---|
| −16.76 | **16.6** — the largest circle in the file |
| −11.98 | 14.0 / 10.0 |
| −9.28 | 6.0 |
| +7.92 | 6.5 |
| +11.59 | 3.8 / 3.0 |

## The mounting holes

**Two 6-hole flanges, identical, one at each end of the body.**

| plane | holes | Ø | bolt circle | angles |
|---|---|---|---|---|
| `x = −10.08` | 6 | 1.7 | **R5.25 (Ø10.5)** | 0° / 60° / … / 300° |
| `x = +5.52` | 6 | 1.7 | **R5.25 (Ø10.5)** | 0° / 60° / … / 300° |

They are **15.6 mm apart** and sit **5.3 mm** from their respective ends of the outline — the pair
is symmetric about X. Chord between adjacent holes is 5.25 mm.

**That is the answer the bracket needed.** It is also the one thing that could not have been
guessed: a catalogue drawing gives you a box, and this gives you where the screws go.

Other holes, none of which sit on a common circle about the axis (so they are shell and structure,
not a second flange):

| plane | holes | Ø |
|---|---|---|
| `x = −11.43` | 4 | **2.2** |
| `x = −5.88` | 8 | 1.7 |
| `x = +9.05` | 8 | 1.7 / 1.8 |
| `x = −13.17` | 4 | 1.7 |
| `x = +3.32` | 4 | 1.6 |
| `x = −11.60` | 3 | 1.4 |
| `x = +1.94` | 3 | 1.4 |
| `x = +9.72` | 2 | 1.4 |
| `x = −11.88` | 2 | 1.4 |

## What it does not answer

**The output shaft.** The largest cylinder in the file is Ø16.6 on the body, and the front end only
reaches Ø3.8 / Ø3.0 — neither is a shaft a horn bolts onto. Either the simplification dropped it, or
it is not where this reading looks for it. **Measure the shaft on the unit; do not take it from
here.** The horn geometry is what the leg links key off, so this is not a small gap.

**Thread.** Ø1.7 is a *clearance* size. Whether the flange takes M1.6 or M2 is not in the geometry —
thread form is one of the things 简化 removed. Check against a fastener, or ask.

**Mass and inertia.** 19.5 g is the catalogue figure and it is not derivable from this file: a
surface model has no volume. It is worth caring about, because the S288 is ~7 % lighter than the
XL330 it replaces and the joint inertias the policies were trained against come from the old
number.

**Anything about the inside.** No reduction ratio, no rotor, no bearing arrangement. The one
mechanical number that *is* known about the inside came off the bench instead — see
[`s288-servo-port.md`](s288-servo-port.md) for the backlash and the torque response.

## Reproducing this

STEP is ASCII (ISO 10303-21), so this needs no CAD — a text editor and a regex will do. The script
that produced every number above:

```python
import re, math, collections

txt = open('S288-简化模型.step','rb').read().decode('gbk', errors='replace')

# The outline: only points that a VERTEX_POINT refers to are real geometry.
pt = {int(i): (float(a), float(b), float(c)) for i, a, b, c in re.findall(
    r"#(\d+)\s*=\s*CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\)\s*\)", txt)}
vid = [int(m) for m in re.findall(r"VERTEX_POINT\s*\(\s*'[^']*'\s*,\s*#(\d+)\s*\)", txt)]
verts = [pt[i] for i in vid if i in pt]

# Holes: CYLINDRICAL_SURFACE -> its AXIS2_PLACEMENT_3D -> the point on the axis.
axloc = {int(i): pt.get(int(loc)) for i, loc in re.findall(
    r"#(\d+)\s*=\s*AXIS2_PLACEMENT_3D\s*\(\s*'[^']*'\s*,\s*#(\d+)", txt)}
holes = [(float(r), axloc.get(int(pl))) for pl, r in re.findall(
    r"CYLINDRICAL_SURFACE\s*\(\s*'[^']*'\s*,\s*#(\d+)\s*,\s*([\d.eE+-]+)\s*\)", txt)]
```

Four things went wrong on the way, all of them silently:

1. **The header is GBK.** `decode('utf-8')` mangles the file name and the product name into
   replacement characters — decode as `gbk`.
2. **Do not take extremes over every `CARTESIAN_POINT`.** Constructive geometry is in there, and
   doing that reports a **56.8 mm** span where the truth is 26.20. Filter to points reachable from
   a `VERTEX_POINT` first.
3. **`AXIS2_PLACEMENT_3D` gives a point *on* the axis, not the face.** Using it to bound the axial
   extent overstates it — the cylinder positions run from −16.76 to +11.59 while the body is
   −15.38 to +10.82.
4. **One hole is one to four `CYLINDRICAL_SURFACE`s**, depending on how coarse the simplification
   was. Counting faces counts the four-hole group at `x = −11.43` as sixteen. Deduplicate on
   `(x, y, z)`.
