# Anthropomorphic Robotic Hand CAD — Iteration 9 (v9.0) Specification & Verification Report

## Overview & Engineering Summary

Iteration 9 (**v9.0**) transitions the CAD codebase to a modular Python/Trimesh package (`cad_v9/digits.py`, `cad_v9/chassis.py`, and `generate_hand_stl_v9.py`) exporting **17 watertight manifold STLs** in `stl_exports_v9/` (`16` single-body printable components + `1` full assembly reference). Iteration 9 resolves every mechanical, structural, and kinematic loophole identified in the v8 audit:

1. **Unobstructed `3.50 mm` Palmar Tendon Moment Arm & Distal Knot Pocket**: Relocates the `Ø2.20 mm` phalanx flexor bore to `z = -3.50 mm`, providing a `≥ 0.70 mm` solid plastic bridge below the `Ø3.40 mm` M3 hinge pin (eliminating v8's screw-to-tendon collision and nearly doubling flexion torque from `1.79 mm` to `3.50 mm`), plus a recessed `Ø4.0 × 2.5 mm` fingertip knot counterbore.
2. **True Captive 6-Sided M3 Hex Nut Sockets & `≥ 1.25 mm` Clamping Walls**: Replaces circular nut counterbores with anti-rotation `6-sided` hexagonal sockets (`5.65 mm` across flats) on `+X` fork walls and enforces `≥ 1.25 mm` solid clamping webs across all digits (including locally widened condyle bosses on narrow phalanges such as `pinky_distal_v9.stl`).
3. **Profile-Following Dorsal Snap-Fit Pinch-Lip Channels + Clean Cuticle Termination**: Lofts the `1.80 mm → 2.60 mm` (`45°` chamfer, `0.50 mm` lip) undercut cutter along the curved dorsal crown `z(0, y)` of each phalanx and terminates the overhang lips cleanly at `0.55 × length` prior to the `0.70 mm` anatomical fingernail bed (`0.58 × length` cuticle).
4. **Male Thumb CMC Opposition Pedestal, Splayed MCP Stops & Single-Shell Through-Palm Conduits**: Replaces v8's colliding female-to-female thumb slot (`1,903–2,387 mm³` interference) with a sculpted male CMC hinge tongue (`4.4 mm`) at compound Euler pose `(+0.85, -0.35, +0.50) rad`, rotates all 4 finger knuckle tongues by their exact anatomical `rotZ` splay, adds rigid `0°` dorsal extension stops on all MCP/CMC joints, replaces open dorsal dimples with captive `Ø2.0 mm` cross-pin elastic anchors, and carves continuous `Ø2.80 mm` through-palm tendon conduits with zero trapped internal voids (`bodies == 1`).
5. **Y-Axis 4-Bolt Wrist Bulkhead & Open-Access 6-Bay MG90S Forearm Servo Rack**: Eliminates v8's vertical Z-axis hole through the palm, terminating at a planar Y-axis 4× M3 wrist bulkhead mated (`0.000 mm³` collision) to a side-loading dual-column 6× MG90S forearm servo rack (`forearm_servo_adapter_v9.stl`).

---

## 1. Root-Cause Defect Resolution Matrix (v8.0 → v9.0)

| Subsystem | Legacy v8 Defect | v9.0 Engineering Fix (`cad_v9/`) |
| :--- | :--- | :--- |
| **Palmar Tendon vs. M3 Pin** | Bore at `z = -1.79 mm` intersected M3 pin (`z ∈ [-1.50, +1.50] mm`); tiny `1.79 mm` moment arm | Bore (`Ø2.20 mm`) moved to `z = -3.50 mm` (`z ∈ [-4.60, -2.40] mm`); `≥ 0.70 mm` solid bridge & `3.50 mm` moment arm |
| **Fingertip Tendon Knot** | Exposed knot protruded from distal fingertip grip surface | Recessed `Ø4.0 mm × 2.5 mm` axial counterbore inside distal palmar apex |
| **M3 Nut Retention & Web** | Circular `Ø6.5 mm` pocket allowed nut spin; `pinky_distal` clamping wall thinned to `0.70 mm` | **6-sided hex socket** (`5.65 mm` AF) on `+X`; reinforced condyle bosses guarantee **`≥ 1.25 mm` clamping walls** |
| **Dorsal Snap-Fit Lips** | Flat `z_surf = 0.44H` caused `1.42 mm` mid-shaft lips & shaved lips inside distal nail bed | Crown-tracking lofted cutter (`lip_t = 0.50 mm`, `45°` chamfer); lips terminate at `0.55L` before `0.58L` cuticle |
| **Thumb CMC Joint** | Both palm and `thumb_proximal` were female slots (`1,903–2,387 mm³` collision) | Sculpted radial-palmar pedestal with protruding `4.4 mm` male CMC tongue, `0°–90°` clear sweep & `0°` dorsal stop |
| **Palm Conduits & Voids** | Tendon tunnels dead-ended `34 mm` short of wrist; `-2,685.9 mm³` sealed void (`bodies == 2`) | Continuous `Ø2.80 mm` through-palm carpal conduits from all 5 digits out axial wrist funnel (`bodies == 1`) |
| **MCP Splay & Hard Stops** | Unrotated knuckle tongues bound on Index/Pinky (`~0.5–0.6 mm³`); zero MCP hyperextension stop | Knuckle tongues rotated by `rotZ = [+0.07, +0.02, -0.04, -0.10] rad` + dorsal `0°` hard-stop overhangs |
| **Palm Elastic Anchors** | Smooth `3×4 mm` blind dimples allowed stretched bands to pop out | Recessed dorsal pockets crossed by `Ø2.0 mm` transverse tie-holes across all 4 knuckles + thumb CMC |
| **Wrist & Forearm Adapter** | Vertical Z-hole pierced palm center; hollow cup held `0` MG90S servos | Planar Y-axis 4× M3 wrist bulkhead + open-sided dual-column **6× MG90S forearm rack** (`12.6 × 23.2 mm` bays) |

---

## 2. Detailed v9.0 Mechanical Specifications

### 2.1 Phalanx, Fastener & Tendon Channel Parameters (`cad_v9/digits.py`)

| Parameter | Constant / Value | Functional Specification |
| :--- | :--- | :--- |
| **Hinge Pin Bore** | `PIN_RADIUS = 1.70 mm` (`Ø3.40 mm`) | Concentric `X`-axis bore at `z = 0.00 mm` for smooth M3 bolt clearance |
| **Palmar Flexor Bore** | `TENDON_RADIUS = 1.10 mm` (`Ø2.20 mm`) at `TENDON_Z = -3.50 mm` | `3.50 mm` flexion moment arm; `≥ 0.70 mm` bridge below pin; `≥ 1.10 mm` palmar floor |
| **Distal Knot Pocket** | `Ø4.0 mm × 2.5 mm` counterbore | Conceals knotted `Ø0.8–1.0 mm` UHMWPE Dyneema bead flush inside fingertip apex |
| **Clevis Fit** | `CLEVIS_SLOT_W = 5.4 mm`, `CLEVIS_TONGUE_W = 4.4 mm` | `0.50 mm` lateral air gap per side; `FDM_CLEARANCE = 0.25 mm` radial hub clearance |
| **Left Fork Screw Seat (`-X`)** | `SCREW_HEAD_R = 3.15 mm` (`Ø6.30 mm`), depth `≥ 1.80 mm` | Flush circular counterbore for M3 socket or button head cap screws |
| **Right Fork Nut Seat (`+X`)** | `6-sided` hex (`5.65 mm` AF, circumradius `3.262 mm`), depth `1.93–2.45 mm` | True captive anti-rotation M3 hex nut socket; **`1.32–2.20 mm` (`≥ 1.25 mm`) clamping web** on all digits (local condyle boss `11.90 mm` on narrow pinky) |
| **Dorsal Snap-Fit Channel** | `w_mouth = 1.8 mm`, `w_cavity = 2.6 mm`, `lip_t = 0.50 mm`, `45°` chamfer | Crown-following undercut groove; `100%` support-free FDM overhang; stops at `0.55L` on tips |
| **Anatomical Nail Bed** | `NAIL_DEPTH = 0.70 mm`, width `0.72W`, origin `0.58L` | Shallow fingernail plate seat concealing the `Ø2.0 mm` transverse elastic bore (`0.78L`) |

### 2.2 Chassis, Wrist & 6-Servo Forearm Parameters (`cad_v9/chassis.py`)

| Parameter | Constant / Value | Functional Specification |
| :--- | :--- | :--- |
| **Palm Envelope** | `PALM_W = 76.0 mm`, `PALM_L = 86.0 mm`, `PALM_H = 22.0 mm` | Monolithic single-body (`bodies == 1`) palm with sculpted thenar & hypothenar arches |
| **Finger Knuckle Array** | `fx = [-23.0, -8.0, 8.0, 23.0] mm`, `rotZ = [+0.07, +0.02, -0.04, -0.10] rad` | Splayed `4.4 mm` male tongues with `0°` dorsal stops and `0°→95°` collision-free flexion |
| **Thumb CMC Pose** | `THUMB_CMC_POS = [-33.0, 24.0, -2.0] mm`, `Euler XYZ = (+0.85, -0.35, +0.50) rad` | Protruding `4.4 mm` male CMC tongue on radial-palmar pedestal; `0°–90°` clear opposition |
| **Through-Palm Conduits** | `Ø2.80 mm` (`r = 1.40 mm`) continuous bores | Unbroken tendon passages from all 5 digits out the axial `Y`-axis wrist funnel |
| **Wrist Bulkhead** | `WRIST_INTERFACE_Y = -14.0 mm`, `46.0 × 30.0 mm` flange, 4× M3 `Y`-axis bolts at `(±14.0, ±9.5) mm` | Planar XZ bulkhead with `Ø22.0 mm` central carpal opening; zero vertical `Z`-axis palm hole |
| **6-Servo Forearm Rack** | `62.0 × 82.0 × 34.0 mm` envelope, 6× `12.7 × 23.3 mm` side-loading bays (`y = -24, -43, -62 mm`) | Dual-column (`3` on `-X`, `3` on `+X`) rack for `12.2 × 22.8 × 22.0 mm` MG90S/HV micro-servos + 12× `Ø1.8 mm` M2 pilot bores (`z = ±13.75 mm`) |

---

## 3. Biomechanical Torque & Fingertip Force Summary

With `r_moment = 3.50 mm` and primary finger length `L_finger ≈ 85 mm` (`~1:24.3` anatomical speed-multiplying lever ratio):
- **Standard 6.0V MG90S (`~2.2 kg·cm`, `Ø11–12 mm` horn)**: **`~1.5–1.8 N` per fingertip** (`~0.75–0.92 kgf` 5-digit pinch; `~1.2–1.6 kgf` wrapped grasp).
- **HV Digital Coreless (`KST DS215MG / KST X08 Plus / KingMax CLS0612W`, `3.7–5.3 kg·cm` on `Ø10 mm` spools)**: **`~3.5–5.5 N` per digit** (`~1.8–2.8 kgf` 5-digit pinch; `~3.0–4.0 kgf` wrapped power grasp).

---

## 4. Solid Manifold, Volume & Automated Verification Metrics (`stl_exports_v9/`)

All **17 STL meshes** exported by `generate_hand_stl_v9.py` into `stl_exports_v9/` are certified **100% watertight solid manifolds**, with every printable component consisting of **exactly 1 connected body** (`len(m.split(only_watertight=False)) == 1`, zero trapped internal voids):

| Part Name | File Name (`stl_exports_v9/`) | File Size (`KB` = KiB / Exact Bytes) | Triangles | Volume ($\text{mm}^3$) | Bodies | Watertight Solid? |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **Index Distal** | `index_distal_v9.stl` | `241.8 KB` (`247,584 B`) | 4,950 | `2,332.5` (`2,332.45`) | 1 | **True** |
| **Index Intermediate** | `index_intermediate_v9.stl` | `344.8 KB` (`353,084 B`) | 7,060 | `2,037.4` (`2,037.42`) | 1 | **True** |
| **Index Proximal** | `index_proximal_v9.stl` | `357.1 KB` (`365,684 B`) | 7,312 | `3,020.8` (`3,020.76`) | 1 | **True** |
| **Middle Distal** | `middle_distal_v9.stl` | `244.4 KB` (`250,284 B`) | 5,004 | `2,629.4` (`2,629.44`) | 1 | **True** |
| **Middle Intermediate** | `middle_intermediate_v9.stl` | `353.0 KB` (`361,484 B`) | 7,228 | `2,394.8` (`2,394.79`) | 1 | **True** |
| **Middle Proximal** | `middle_proximal_v9.stl` | `367.8 KB` (`376,584 B`) | 7,530 | `3,666.1` (`3,666.12`) | 1 | **True** |
| **Ring Distal** | `ring_distal_v9.stl` | `242.3 KB` (`248,084 B`) | 4,960 | `2,282.7` (`2,282.72`) | 1 | **True** |
| **Ring Intermediate** | `ring_intermediate_v9.stl` | `345.9 KB` (`354,184 B`) | 7,082 | `2,121.5` (`2,121.53`) | 1 | **True** |
| **Ring Proximal** | `ring_proximal_v9.stl` | `359.8 KB` (`368,384 B`) | 7,366 | `3,174.8` (`3,174.77`) | 1 | **True** |
| **Pinky Distal** | `pinky_distal_v9.stl` | `236.8 KB` (`242,484 B`) | 4,848 | `1,841.6` (`1,841.63`) | 1 | **True** |
| **Pinky Intermediate** | `pinky_intermediate_v9.stl` | `341.1 KB` (`349,284 B`) | 6,984 | `1,611.9` (`1,611.94`) | 1 | **True** |
| **Pinky Proximal** | `pinky_proximal_v9.stl` | `352.7 KB` (`361,184 B`) | 7,222 | `2,447.8` (`2,447.83`) | 1 | **True** |
| **Thumb Distal** | `thumb_distal_v9.stl` | `239.0 KB` (`244,784 B`) | 4,894 | `3,258.6` (`3,258.58`) | 1 | **True** |
| **Thumb Proximal** | `thumb_proximal_v9.stl` | `374.8 KB` (`383,784 B`) | 7,674 | `3,712.7` (`3,712.68`) | 1 | **True** |
| **Palm** | `palm_v9.stl` | `847.7 KB` (`868,084 B`) | 17,360 | `103,209.9` (`103,209.94`) | 1 | **True** |
| **Forearm Servo Adapter** | `forearm_servo_adapter_v9.stl` | `158.9 KB` (`162,684 B`) | 3,252 | `83,168.6` (`83,168.59`) | 1 | **True** |
| **Full Hand Assembly** | `robotic_hand_full_assembly_v9.stl` | `5,406.6 KB` (`5,536,384 B`) | 110,726 | `222,911.2` (`222,911.19`) | 16 | **True** |

---

## 5. `verify_v9_complete()` Automated Engineering Audit Summary

Running `./venv/bin/python3 generate_hand_stl_v9.py` executes the strict `verify_v9_complete()` suite across all reloaded binary STLs (`exit code 0`):

1. **Manifold & Single-Shell Verification**:
   - All 17 `*_v9.stl` files pass `is_watertight == True` and `volume > 0`.
   - All 16 printable part STLs pass `len(m.split(only_watertight=False)) == 1`, confirming complete elimination of v8's `-2,685.9 mm³` trapped internal palm void; `robotic_hand_full_assembly_v9.stl` comprises `16` non-overlapping bodies whose sum (`222,911.19 mm³`) equals the exact sum of the 16 individual components.
2. **Full-Hand Kinematic Collision Sweep (`0.0000 mm³` Overlap)**:
   - **Digits II–V (`index, middle, ring, pinky`)**: `0.0000 mm³` boolean intersection across `0°, 15°, 30°, 45°, 60°, 75°, 90°, 95°` at **MCP** (with exact `rotZ` applied), **PIP**, and **DIP**.
   - **Digit I (`thumb`)**: `0.0000 mm³` boolean intersection across `0°, 15°, 30°, 45°, 60°, 75°, 90°` at **Thumb CMC** (`THUMB_CMC_POS = [-33.0, 24.0, -2.0] mm`, Euler `(+0.85, -0.35, +0.50) rad`) and **Thumb IP**.
   - **Wrist Interface**: `0.0000 mm³` overlap between `palm_v9.stl` and `forearm_servo_adapter_v9.stl` at `ADAPTER_PLACEMENT = [0.0, -14.1, 0.0] mm` with 4 concentric `Y`-axis M3 bolt bores (`WRIST_BOLT_XZ = [(±14.0, ±9.5)] mm`).
3. **One-Directional `0°` Dorsal Extension Hard Stops**:
   - All **14 joints** (`4× MCP`, `4× PIP`, `4× DIP`, `Thumb CMC`, `Thumb IP`) engage rigid mechanical hard-stop shoulders at `-5°` (`≥ 0.363 mm³` interference) and `-10°` (`≥ 1.003 mm³` interference).
4. **Tendon, Fastener, Snap-Fit & Servo Clearance Verification**:
   - **Through-Palm Ray/Path Check**: Pure-NumPy Moller-Trumbore ray tracing confirms unbroken `Ø2.80 mm` internal tendon conduits from all 5 digits out `WRIST_INTERFACE_Y = -14.0 mm`, solid palm center at `[0, 4, z]` (zero vertical `Z`-hole), and 5 captive `Ø2.0 mm` transverse dorsal elastic cross-pin anchors.
   - **M3 Pin vs. Tendon Bridge**: Zero intersection between M3 hinge pins (`z ∈ [-1.50, +1.50] mm`) and palmar tendon channels (`max z_top = -2.40 mm ≤ -2.20 mm`), plus enclosed `Ø4.0 × 2.5 mm` distal palmar knot pockets on all 5 fingertips.
   - **Hex Nut Sockets & Clamping Webs**: Verified 6-sided planar hex nut sockets (`5.65 mm` across flats, `r_max / r_min = 1.155`) on `+X` and solid clamping walls **`≥ 1.32 mm`** (`1.32–2.20 mm`, exceeding the `1.25 mm` CAD spec) even on `pinky_distal_v9.stl` and `index_distal_v9.stl`.
   - **Dorsal Snap-Fit Pinch Lips & Nail Bed**: Overhanging pinch lips verified present along mid-shafts (`x = ±1.05 mm`) on all 14 phalanges and cleanly terminated before the `0.58L` fingernail cuticle (`0` overhang hits across `y ≥ 0.60L` on all 5 distal tips).
   - **6-Bay MG90S Forearm Fit**: Six `12.2 × 22.8 × 22.0 mm` MG90S clearance boxes slide radially from outside (`30 → 0 mm`) into the 6 side-loading bays (`yc ∈ [-24, -43, -62] mm`) with `0.0000 mm³` collision, zero mutual overlap, and 12 unobstructed `Ø1.8 mm` M2 pilot bores.
