# Biomimetic Tendon-Driven Robotic Hand Platform (v10.0 Full-Actuation & v9.0 CAD)

An open-source anthropomorphic robotic hand repository combining **print-verified v10.0 full-actuation 11-servo mechanical CAD + ESP32-S3 firmware** (`cad_v10/`, `stl_exports_v10/`, `firmware/hand_controller_v10/`), the **v9.0 6-servo underactuated baseline** (`cad_v9/`, `stl_exports_v9/`), and an **interactive 27-DoF musculoskeletal WebGL simulator** (`index.html`, `BIOMIMETIC_SPECIFICATION.md`).

---

## 1. Platform Releases (v10.0 Full-Actuation vs. v9.0 Baseline vs. 27-DoF Reference)

| Subsystem / Parameter | **v10.0 Full-Actuation Production (`stl_exports_v10/`)** | **v9.0 Underactuated Baseline (`stl_exports_v9/`)** | **27-DoF Platform Reference (`index.html`)** |
| :--- | :--- | :--- | :--- |
| **Primary Purpose** | Ready-to-print **11-servo** multi-DoF hand + rotating forearm + ESP32-S3/PCA9685 firmware (`26` printable parts + `1` assembly = `27` STLs) | Compact **6-servo** single-axis tendon hand (`16` printable parts + `1` assembly = `17` STLs) | Full human-parity kinematic simulator & high-DoF expansion blueprint |
| **Mechanical Joints / DoF** | **14 phalanx joints + 4× 2-DoF MCP Cardan yokes (`±15°` coupled abduction) + split cupping palm (`0°–30°`) + 2-DoF hollow Cardan wrist (`±45°` Pitch, `±35°` Yaw) + axial forearm pronation (`±85°`)** | **14 single-axis M3 pin-hinged joints** (Index/Middle/Ring/Pinky MCP, PIP, DIP + Thumb CMC, IP) | **27 DoF** simulated musculoskeletal workspace |
| **Actuators & Electronics** | **`8×` `MG90S`/`KST DS215MG` + `3×` `DS3218`/`MG996R`** (`11` channels on `PCA9685` + `ESP32-S3` + `6V–7.4V 10–15A UBEC`) | **`6×` `MG90S`/`KST DS215MG`** in open-sided dual-column forearm rack | Modular 16–24 channel servo bank |
| **Extension Hard Stops** | Rigid `0°` dorsal mechanical stops on all 14 digit joints (`0°→95°` fingers; `0°→90°` thumb) + cupping `0°` stop | Rigid `0°` dorsal mechanical stops on all 14 joints | Simulated joint limits across all axes |
| **Tendon & Bowden Routing** | `3.50 mm` palmar moment arm (`TENDON_Z = -3.50 mm`) + `7×` `2 mm OD` PTFE Bowden liners through `Ø23 mm` hollow Cardan wrist core (zero twist during pronation) | `3.50 mm` palmar moment arm + unbroken `Ø2.80 mm` through-palm conduits | Variable moment-arm / multi-tendon Bowden routing |
| **Joint Fasteners & Return** | Captive 6-sided M3 hex nut sockets (`5.65 mm` AF, `≥ 1.25 mm` walls) + profile-following `45°` dorsal snap-fit pinch-lip channels + M3 ball-joint wrist pushrods | Captive 6-sided M3 hex nut sockets + profile-following `45°` dorsal snap-fit pinch-lip channels | Precision shoulder bolts + dorsal elastomers |
| **Control Software** | **`firmware/hand_controller_v10/`**: ESP32-S3 C++ sketch (differential wrist mixer, 50 Hz slew-rate engine, 8 grasp presets, `RELAX` E-stop) + `hand_cli.py` | Open-loop PWM (`PCA9685`) | Interactive Three.js WebGL visualizer |

---

## 2. Iteration 10 (v10.0) Full-Actuation Engineering Highlights

Iteration 10 (`cad_v10/`, `generate_hand_stl_v10.py`, and `firmware/hand_controller_v10/`) retains all 14 certified v9.0 phalanges (`cad_v9/digits.py`) and adds full wrist, palm, and forearm articulation—engineered to fit on a standard **`220×220×250 mm`** Anycubic Kobra 2 Neo build plate:

1. **2-DoF MCP Cardan Yokes & Coupled In-Palm Abduction (`cad_v10/palm_assembly.py`)**:
   - Four single-body MCP Cardan yokes (`*_mcp_yoke_v10.stl`) provide `0°–95°` flexion with `0°` hard stops AND `±15°` lateral abduction yaw driven simultaneously by an in-palm `MG90S`/`SG90` servo (`CH07`) via `abduction_linkage_v10.stl`.
2. **Split Cupping Palm (`palm_radial_v10.stl` + `palm_ulnar_v10.stl`)**:
   - Splits cleanly along `X = 0` on an axial M3 hinge (`[0.0, 42.0, +3.0] mm`) so the ulnar palm (Ring + Pinky + hypothenar pad) cups `0°–30°` palmarly (`CH06`) for ergonomic cylindrical/spherical power grasps, while preserving the certified Thumb CMC opposition pedestal and dedicated opposition fairlead (`CH05`).
3. **Hollow-Core 2-DoF Cardan Wrist & Differential Pushrods (`cad_v10/wrist_forearm.py`)**:
   - `wrist_cardan_ring_v10.stl` (`Ø23.0 mm` open Bowden core) delivers `±45°` Pitch and `±35°` Yaw driven by dual `DS3218` high-torque servos (`CH08–CH09`) via M3 ball-joint pushrods—passing all 7 PTFE Bowden tubes straight through the gimbal center without pinching.
4. **Rotating 9-Servo Forearm Barrel & 2:1 Pronation Gear Train**:
   - `forearm_inner_barrel_v10.stl` (`79.8 × 177.6 × 79.8 mm`) houses `7× MG90S` tendon servos + `2× DS3218` wrist servos and terminates in a `Ø68.0 mm` printed bearing journal and integral **`44T` spur gear** (`m = 1.75 mm`).
   - `forearm_outer_sleeve_v10.stl` mounts the stationary **`CH10` `DS3218` pronation servo** driving a **`22T` pinion** (`pronation_pinion_v10.stl`, `57.75 mm` center distance) for `±85°` axial roll without twisting a single tendon, plus an integrated tray for the **ESP32-S3** and **PCA9685**.
5. **Production ESP32-S3 + PCA9685 Firmware & Python CLI (`firmware/hand_controller_v10/`)**:
   - Complete `CH00–CH10` configuration (`config.h`), differential wrist pitch/yaw mixer with anti-saturation clamping, `50 Hz` non-blocking slew-rate trajectory engine, 8 built-in grasp presets (`REST, OPEN, FIST, PINCH, CYLINDER, TRIPOD, SPREAD, WAVE`), `RELAX` E-stop, and `hand_cli.py` (with `--dry-run` simulation).

See **[V10_BUILD_AND_WIRING_GUIDE.md](V10_BUILD_AND_WIRING_GUIDE.md)** for the full 11-servo BOM, PCA9685 wiring diagram, 27-file STL inventory, phased print plan, assembly steps, and firmware bring-up.

---

## 3. Iteration 9 (v9.0) Baseline Mechanical Engineering Upgrades

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

## 4. Biomechanical Force Selection Summary (v9.0 & v10.0 Digits)

Because flexor tendons act at `r_moment = 3.50 mm` against an extended finger length of `~85 mm` (`~1:24.3` anatomical speed-multiplying lever ratio), builders should choose servos according to target grasp payload:

- **Budget / Classroom Build — `MG90S` (6.0V, `~2.2 kg·cm`)**: Delivers **`~1.5–1.8 N` (`~150–185 gf`) per fingertip** (`~0.75–0.92 kgf` 5-digit pinch; `~1.2–1.6 kgf` wrapped cylindrical hook grasp). Suitable for kinematics demos, gesturing, and lightweight objects.
- **High-Performance Manipulation Build — `KST DS215MG` / `KST X08 Plus` / `KingMax CLS0612W` (7.4V–8.4V HV, `3.7–5.3 kg·cm` with `Ø10 mm` spools)**: Delivers **`~3.5–5.5 N` (`~355–560 gf`) per digit** (`~1.8–2.8 kgf` 5-digit pinch; `~3.0–4.0 kgf` wrapped grasp). Full derivations are provided in [BIOMIMETIC_SPECIFICATION.md](BIOMIMETIC_SPECIFICATION.md) and [ENGINEERING_GUIDE.md](ENGINEERING_GUIDE.md).
- **v10.0 Wrist & Forearm Pronation Servos**: `3× DS3218` (`20 kg·cm` metal-gear) or `MG996R` (`10–13 kg·cm`) powered from a dedicated `6.0V–7.4V 10A–15A` UBEC (never from the ESP32-S3 5V pin).

---

## 5. Repository Contents

### Iteration 10 (v10.0 — Full-Actuation 11-Servo Production Release)
- [`cad_v10/`](cad_v10/): Modular Iteration 10 Python/Trimesh CAD package (`palm_assembly.py` for 2-DoF MCP Cardan yokes, coupled abduction linkage, and split cupping palm; `wrist_forearm.py` for hollow Cardan wrist ring, 9-servo rotating barrel with 44T gear, stationary outer sleeve, 22T pronation pinion, and winch spools).
- [`generate_hand_stl_v10.py`](generate_hand_stl_v10.py): Production v10.0 CAD build & automated multi-DoF verification suite (`verify_v10_complete()`).
- [`stl_exports_v10/`](stl_exports_v10/): All **27** production watertight v10.0 STL meshes (`26` single-body printable parts + `robotic_hand_full_assembly_v10.stl`).
- [`firmware/hand_controller_v10/`](firmware/hand_controller_v10/): Complete **ESP32-S3 + PCA9685** 11-servo controller firmware (`config.h`, `hand_controller_v10.ino`), interactive Python Serial CLI (`hand_cli.py` with `--dry-run`), and firmware flashing/calibration guide.
- [`V10_BUILD_AND_WIRING_GUIDE.md`](V10_BUILD_AND_WIRING_GUIDE.md): Complete v10.0 11-servo BOM, PCA9685 `CH00–CH10` wiring diagram, 27-file STL inventory, phased Kobra 2 Neo print plan, Bowden routing, mechanical assembly, and bring-up manual.

### Iteration 9 (v9.0 — 6-Servo Underactuated Baseline) & Platform Docs
- [`cad_v9/`](cad_v9/): Modular Iteration 9 Python/Trimesh CAD package (`digits.py` for certified phalanges, joints, hex sockets, and snap-fit channels — reused directly by v10.0; `chassis.py` for monolithic palm, thumb CMC pedestal, through-palm conduits, Y-axis wrist flange, and 6-servo forearm rack).
- [`generate_hand_stl_v9.py`](generate_hand_stl_v9.py): Production v9.0 CAD build & automated geometric/kinematic verification suite (`verify_v9_complete()`).
- [`stl_exports_v9/`](stl_exports_v9/): All 17 production watertight v9.0 STL meshes (`16` printable parts + `robotic_hand_full_assembly_v9.stl`).
- [`V9_SPECIFICATION_REPORT.md`](V9_SPECIFICATION_REPORT.md): Iteration 9 engineering specification, root-cause changelog, STL volume/size inventory, and automated verification report.
- [`3D_PRINTING_GUIDE.md`](3D_PRINTING_GUIDE.md): Calibrated v9.0 FDM slicing parameters, support blocker placement, print orientations, and 4-phase bed batch guide (optimized for Anycubic Kobra 2 Neo & standard 220×220 mm printers).
- [`ENGINEERING_GUIDE.md`](ENGINEERING_GUIDE.md): Complete v9.0 BOM, torque/spool engineering math, captive M3 hex-nut assembly, snap-fit elastic routing, and 6-servo rigging manual.
- [`BIOMIMETIC_SPECIFICATION.md`](BIOMIMETIC_SPECIFICATION.md): Two-tier platform specification, exact tendon lever physics, 6-channel v9.0 mapping, and 20-channel / 27-DoF expansion table.
- [`index.html`](index.html): Interactive Three.js WebGL musculoskeletal visualizer featuring live 27-DoF articulation, dynamic tendon & dorsal elastic routing, active ulnar palmar arch cupping, stationary 6-servo forearm rack with rotating winch horns, wrist gimbal controls, and anatomical fingernail plates.
- [`robotic_hand.scad`](robotic_hand.scad): *Legacy / deprecated v0 OpenSCAD concept exploration* (retained for historical reference only; superseded by `cad_v9/` and `cad_v10/`).

---

## 6. Quick Start

Regenerate all **27 v10.0 STLs** and run the full multi-DoF collision, hard-stop, and 11-servo fit verification suite:
```bash
./venv/bin/python3 generate_hand_stl_v10.py
```

Test the v10.0 Python Serial CLI & differential wrist kinematics mixer in simulation (`--dry-run`, no hardware required):
```bash
./venv/bin/python3 firmware/hand_controller_v10/hand_cli.py --dry-run
```

Regenerate all **17 v9.0 baseline STLs** and run `verify_v9_complete()`:
```bash
./venv/bin/python3 generate_hand_stl_v9.py
```

Launch the interactive 3D WebGL musculoskeletal simulator:
```bash
open index.html
```
