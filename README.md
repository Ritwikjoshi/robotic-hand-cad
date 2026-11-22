# Biomimetic Tendon-Driven Robotic Hand Platform (v9.0 CAD & 27-DoF Reference)

An open-source anthropomorphic robotic hand repository combining **print-verified v9.0 underactuated mechanical CAD hardware** (`cad_v9/`, `stl_exports_v9/`) with an **interactive 27-DoF musculoskeletal simulation and expansion architecture** (`index.html`, `BIOMIMETIC_SPECIFICATION.md`).

---

## 1. Two-Tier Architectural Overview (Printable v9.0 Hardware vs. 27-DoF Platform Reference)

Earlier documentation conflated the full theoretical 27-DoF biomimetic roadmap with the single-axis 3D-printable STL files. Beginning in **Iteration 9 (v9.0)**, the repository explicitly separates what is physically built into the printable STLs today from the multi-axis expansion roadmap:

| Subsystem / Parameter | Tier 1: v9.0 Printable Mechanical Hardware (`stl_exports_v9/`) | Tier 2: 27-DoF Full Platform Reference (`index.html` & Expansion Spec) |
| :--- | :--- | :--- |
| **Primary Purpose** | Ready-to-print FDM/SLA underactuated tendon hand (16 printable parts + 1 assembly reference) | Full human-parity kinematic simulator (`index.html`) & multi-servo expansion blueprint |
| **Mechanical Joints / DoF** | **14 single-axis M3 pin-hinged joints** (Index/Middle/Ring/Pinky MCP, PIP, DIP + Thumb CMC, IP) | **27 DoF** (adds active MCP abduction, 2-DoF palmar arch cupping, 3-DoF thumb circumduction, 4-DoF wrist/forearm gimbal) |
| **Extension Hard Stops** | **Rigid `0°` dorsal mechanical stops** on all 14 joints (`0°→95°` finger flexion; `0°→90°` thumb flexion) | Simulated joint limits across flexion, abduction, cupping, and wrist pitch/yaw/roll |
| **Forearm Servo Capacity** | **6-bay open-access MG90S rack** (`forearm_servo_adapter_v9.stl`: 3 left + 3 right side-loading bays) | Modular **16–24 channel** high-density forearm/chassis servo bank (20 mapped tendon channels) |
| **Tendon Moment Arm** | **`3.50 mm` palmar moment arm** (`TENDON_Z = -3.50 mm`, clearing the M3 hinge pin by `≥ 0.70 mm` solid plastic) | Variable moment-arm / multi-tendon Bowden routing across wrist gimbal |
| **Realistic Fingertip Force** | **`~1.5–1.8 N` / digit** with 6V MG90S (`2.2 kg·cm`); **`~3.5–5.5 N` / digit** with 7.4–8.4V HV digital servos (`3.7–5.3 kg·cm` on `Ø10 mm` spools) | Up to `~18–28 N` (`~1.8–2.8 kgf`) cumulative 5-digit fingertip pinch (`~3–4 kgf` wrapped power grasp) with HV coreless servos |
| **Joint Fasteners** | **M3 screws + captive 6-sided M3 hex nut sockets** (`5.65 mm` across flats) with **`≥ 1.25 mm` clamping walls** | Precision M3 shoulder bolts / dowel pins + miniature sleeve bearings |
| **Antagonistic Return** | **Profile-following dorsal snap-fit pinch-lip channel** (`1.8 mm` mouth, `45°` chamfers, `2.6 mm` cavity, stopping cleanly before nail cuticle) + **captive `Ø2.0 mm` cross-pin palm anchors** | Dorsal elastomer bands or miniature helical torsion springs |
| **Sensing & Feedback** | Open-loop PWM (`PCA9685`) + optional board-level current shunt stall detection *(no magnet pockets in v9.0 STLs)* | External contactless `AS5600` magnetic joint encoders + per-channel current telemetry |

---

## 2. Iteration 9 (v9.0) Key Mechanical Engineering Upgrades

Iteration 9 (`cad_v9/` and `generate_hand_stl_v9.py`) resolves every geometric, structural, and assembly defect identified in v8:

1. **Unobstructed `3.50 mm` Flexor Moment Arm & Recessed Knot Pocket**:
   - Relocated phalanx palmar tendon bores (`Ø2.20 mm`) to `z = -3.50 mm`, leaving a solid `≥ 0.70 mm` bridge below the `Ø3.40 mm` M3 hinge pin hole (eliminating v8's screw-to-tendon blockage while nearly doubling flexion torque over v8's `1.79 mm` arm).
   - Added a `Ø4.0 mm × 2.5 mm` axial counterbore inside each distal fingertip apex so knotted Dyneema seats flush beneath the grip pad.
2. **True Captive 6-Sided M3 Hex Nut Sockets & Reinforced Clamping Webs**:
   - Replaced circular nut recesses with anti-rotation **6-sided hexagonal sockets** (`5.65 mm` across flats) on `+X` clevis walls, paired with `Ø6.30 mm` M3 head counterbores on `-X` walls and **`≥ 1.25 mm` solid clamping walls** across all 14 phalanges (including locally widened base condyles on narrow digits).
3. **Profile-Following Dorsal Snap-Fit Pinch-Lip Channels + Cuticle Termination**:
   - Lofted the `1.8 mm → 2.6 mm` snap-fit undercut channel (`45°` support-free chamfers, `0.50 mm` lips) along the true curved dorsal crown of each phalanx, terminating the overhang lips cleanly at `0.55 × L` just before the `0.70 mm` anatomical fingernail recess (`0.58 × L` cuticle) so press-on nail plates conceal the `Ø2.0 mm` distal elastic knot.
4. **Male Thumb CMC Opposition Pedestal, Splayed MCP Stops & Through-Palm Conduits**:
   - Replaced v8's colliding female-to-female thumb slot with a sculpted **male CMC hinge tongue (`4.4 mm`)** at compound opposition Euler angles (`+48.7°, -20.1°, +28.6°`), rotated all 4 finger knuckle tongues to match anatomical finger splay (`rotZ`), added rigid `0°` dorsal MCP/CMC hard stops, replaced blind rubber-band pits with **captive `Ø2.0 mm` cross-pin dorsal anchors**, and carved unbroken **`Ø2.80 mm` through-palm tendon conduits** (`1` watertight shell per part, zero trapped voids).
5. **Y-Axis 4-Bolt Wrist Bulkhead & Open-Access 6-Bay Forearm Servo Rack**:
   - Eliminated v8's vertical Z-axis hole through the palm in favor of a flat **Y-axis 4× M3 wrist bulkhead flange** with an axial carpal tendon passage mating directly to an **open-sided dual-column 6× MG90S forearm servo rack** (`forearm_servo_adapter_v9.stl`).

---

## 3. Biomechanical Force Selection Summary (v9.0 Hardware)

Because flexor tendons act at `r_moment = 3.50 mm` against an extended finger length of `~85 mm` (`~1:24.3` anatomical speed-multiplying lever ratio), builders should choose servos according to target grasp payload:

- **Budget / Classroom Build — 6× TowerPro MG90S (6.0V, `~2.2 kg·cm`)**: Delivers **`~1.5–1.8 N` (`~150–185 gf`) per fingertip** (`~0.75–0.92 kgf` 5-digit pinch; `~1.2–1.6 kgf` wrapped cylindrical hook grasp). Suitable for kinematics demos, gesturing, and lightweight objects.
- **High-Performance Manipulation Build — 6× KST DS215MG / KST X08 Plus / KingMax CLS0612W (7.4V–8.4V HV, `3.7–5.3 kg·cm` with `Ø10 mm` spools)**: Delivers **`~3.5–5.5 N` (`~355–560 gf`) per digit** (`~1.8–2.8 kgf` 5-digit pinch; `~3.0–4.0 kgf` wrapped grasp). Full derivations are provided in [BIOMIMETIC_SPECIFICATION.md](BIOMIMETIC_SPECIFICATION.md) and [ENGINEERING_GUIDE.md](ENGINEERING_GUIDE.md).

---

## 4. Repository Contents

- [`cad_v9/`](cad_v9/): Modular Iteration 9 Python/Trimesh CAD package (`digits.py` for phalanges, joints, hex sockets, and snap-fit channels; `chassis.py` for monolithic palm, thumb CMC pedestal, through-palm conduits, Y-axis wrist flange, and 6-servo forearm rack).
- [`generate_hand_stl_v9.py`](generate_hand_stl_v9.py): Production v9.0 CAD build & automated geometric/kinematic verification suite (`verify_v9_complete()`).
- [`stl_exports_v9/`](stl_exports_v9/): All 17 production watertight v9.0 STL meshes (`16` printable parts + `robotic_hand_full_assembly_v9.stl`).
- [`V9_SPECIFICATION_REPORT.md`](V9_SPECIFICATION_REPORT.md): Iteration 9 engineering specification, root-cause changelog, STL volume/size inventory, and automated verification report.
- [`3D_PRINTING_GUIDE.md`](3D_PRINTING_GUIDE.md): Calibrated v9.0 FDM slicing parameters, support blocker placement, print orientations, and 4-phase bed batch guide (optimized for Anycubic Kobra 2 Neo & standard 220×220 mm printers).
- [`ENGINEERING_GUIDE.md`](ENGINEERING_GUIDE.md): Complete v9.0 BOM, torque/spool engineering math, captive M3 hex-nut assembly, snap-fit elastic routing, and 6-servo rigging manual.
- [`BIOMIMETIC_SPECIFICATION.md`](BIOMIMETIC_SPECIFICATION.md): Two-tier platform specification, exact tendon lever physics, 6-channel v9.0 mapping, and 20-channel / 27-DoF expansion table.
- [`index.html`](index.html): Interactive Three.js WebGL musculoskeletal visualizer featuring live 27-DoF articulation, dynamic tendon & dorsal elastic routing, active ulnar palmar arch cupping, stationary 6-servo forearm rack with rotating winch horns, wrist gimbal controls, and anatomical fingernail plates.
- [`robotic_hand.scad`](robotic_hand.scad): *Legacy / deprecated v0 OpenSCAD concept exploration* (retained for historical reference only; superseded by `cad_v9/` and `generate_hand_stl_v9.py`).

---

## 5. Quick Start

Regenerate all 17 v9.0 STLs and run the full manifold, collision, hard-stop, tendon-ray, and servo-fit verification suite:
```bash
./venv/bin/python3 generate_hand_stl_v9.py
```

Launch the interactive 3D WebGL musculoskeletal simulator:
```bash
open index.html
```
