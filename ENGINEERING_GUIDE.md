# Biomechanical Anthropomorphic Robotic Hand: Engineering & Fabrication Manual (v9.0)

## 1. Executive Summary & v9.0 Mechanical Architecture

This guide covers mechanical fabrication, actuator selection, fastener assembly, snap-fit dorsal return routing, and tendon rigging for **Iteration 9 (v9.0)** of the anthropomorphic robotic hand (`cad_v9/`, `generate_hand_stl_v9.py`, `stl_exports_v9/`).

### Architectural Scope (v9.0 Printable Hardware vs. 27-DoF Simulator)
- **v9.0 Printable Hardware (`stl_exports_v9/`)**: A self-contained, 16-part 3D-printable underactuated hand with **14 single-axis M3 pin-hinged joints**, **rigid `0°` dorsal mechanical extension hard stops**, **`3.50 mm` palmar tendon moment arms**, **captive 6-sided M3 hex nut sockets**, **profile-following dorsal snap-fit elastic channels**, **continuous through-palm tendon conduits**, a **Y-axis 4-bolt wrist bulkhead**, and an **open-access 6-bay MG90S forearm servo rack**.
- **27-DoF Expansion Reference (`index.html`, `BIOMIMETIC_SPECIFICATION.md`)**: Models active MCP abduction, ulnar palmar arch cupping, multi-axis thumb circumduction, and 3-axis wrist gimbals for builders scaling beyond the 6-servo v9.0 baseline.

### Core v9.0 Geometric & Kinematic Specifications
- **Digits II–V (Index, Middle, Ring, Pinky — 12 Hinges)**:
  - 3 single-axis pin-clevis joints per finger (MCP, PIP, DIP) with unobstructed **`0° → 95°` palmar flexion** and **rigid `0°` dorsal hyperextension stops**.
  - Splayed metacarpal knuckle tongues (`rotZ`: Index `+0.07 rad [+4.0°]`, Middle `+0.02 rad [+1.1°]`, Ring `-0.04 rad [-2.3°]`, Pinky `-0.10 rad [-5.7°]`) for natural multi-finger grasp convergence.
- **Digit I (Thumb — 2 Hinges)**:
  - Sculpted radial-palmar thenar pedestal with a protruding `4.4 mm` male CMC hinge tongue oriented at intrinsic XYZ Euler angles:
    - $\alpha_x = +0.85\text{ rad } (+48.7^\circ)$: Palmar abduction into grasping space.
    - $\beta_y = -0.35\text{ rad } (-20.1^\circ)$: Medial pronation so thumb pulp opposes digits II–IV.
    - $\gamma_z = +0.50\text{ rad } (+28.6^\circ)$: Radial abduction clearing the index MCP knuckle.
  - Unobstructed **`0° → 90°` flexion** at both CMC and IP joints, with **rigid `0°` dorsal stops** on both.
- **Clevis & Captive Fastener Geometry**:
  - Female clevis slot width: **`5.40 mm`**; male tongue width: **`4.40 mm`** (`0.50 mm` lateral clearance per side); radial hub clearance: `0.25 mm` (`FDM_CLEARANCE`).
  - Concentric hinge bore: **`Ø3.40 mm`** (`PIN_RADIUS = 1.70 mm`) for smooth M3 bolt/shoulder-pin rotation.
  - Left fork wall (`-X`): **`Ø6.30 mm`** (`SCREW_HEAD_R = 3.15 mm`) round counterbore for M3 socket/button heads.
  - Right fork wall (`+X`): **6-sided hexagonal socket (`5.65 mm` across flats, circumradius `3.262 mm`)** locking M3 hex nuts against rotation without miniature wrenches.
  - Solid clamping web: **`≥ 1.25 mm`** minimum wall between the `5.40 mm` clevis slot and both counterbore floors across all 14 phalanges (narrow phalanges such as `pinky_distal_v9.stl` feature locally reinforced condyle bosses).

---

## 2. Biomechanical Force, Torque & Servo Selection Math

### 2.1 Why Moment Arm Geometry Governs Servo Choice
In v9.0, every phalanx places the hinge pin at `z = 0.00 mm` (`z ∈ [-1.50, +1.50] mm` for an M3 shaft) and the `Ø2.20 mm` palmar flexor bore at **`z = -3.50 mm`** (`z ∈ [-4.60, -2.40] mm`). This creates a **`≥ 0.70 mm` solid plastic bridge** separating the tendon from the metal pin while establishing a constant flexion moment arm:

$$r_{\text{moment}} = 3.50\text{ mm}$$

Against an extended finger length of $L_{\text{finger}} \approx 85\text{ mm}$ (Index `83.5 mm`, Middle `91.5 mm`, Ring `84.5 mm`, Pinky `72.0 mm`), the tendon operates at an anatomical lever ratio of:

$$\frac{r_{\text{moment}}}{L_{\text{finger}}} = \frac{3.50\text{ mm}}{85.0\text{ mm}} \approx \frac{1}{24.3}$$

Accounting for conduit friction ($\eta \approx 0.80\text{–}0.85$) and dorsal elastic return preload ($F_{\text{elastic}} \approx 0.15\text{–}0.25\text{ N}$ at the tip), fingertip pinch force is:

$$F_{\text{digit}} = \eta \cdot \frac{\tau_{\text{servo}}}{r_{\text{spool}}} \cdot \frac{r_{\text{moment}}}{L_{\text{eff}}} - F_{\text{elastic}}$$

### 2.2 Realistic Actuator Comparison for the 6-Bay Forearm Rack

| Build Tier | Recommended Micro-Servo | Operating Voltage | Stall Torque | Winch Spool (`Ø`) | Cable Pull ($T_{\text{cable}}$) | Realistic Force / Digit | 5-Digit Pinch / Wrap Grasp |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Entry / Educational** | **TowerPro MG90S** (Metal Gear) | `6.0 V` | `~2.2 kg·cm` | `Ø11–12 mm` (`r = 5.5–6 mm`) | `36–39 N` | **`~1.5–1.8 N`** (`150–185 gf`) | `~0.75–0.92 kgf` pinch (`~1.2–1.6 kgf` wrap) |
| **Mid-Range HV Digital** | **KST DS215MG V3** | `7.4 V` (2S LiPo) | `3.7 kg·cm` | `Ø10 mm` (`r = 5.0 mm`) | `72.6 N` | **`~3.5–4.1 N`** (`355–420 gf`) | `~1.8–2.1 kgf` pinch (`~2.8–3.3 kgf` wrap) |
| **High-Force Coreless HV** | **KingMax CLS0612W / KST X08 Plus** | `8.4 V` (HV BEC) | `5.2–5.3 kg·cm` | `Ø10 mm` (`r = 5.0 mm`) | `102–104 N` | **`~4.5–5.5 N`** (`460–560 gf`) | `~2.3–2.8 kgf` pinch (`~3.2–4.0 kgf` wrap) |

- **Stroke Budget**: Full `0° → 95°` 3-joint finger curl requires **`~16.8 mm`** of linear tendon pull (`3.50 mm × 4.80 rad`); full `0° → 90°` 2-joint thumb curl requires **`~11.0 mm`** (`3.50 mm × π rad`). A **`Ø10–12 mm` winch spool** delivers `15.7–18.8 mm` of stroke in a single `180°` sweep.

---

## 3. Bill of Materials (v9.0 Printable Build)

### A. 3D Printed Components (`stl_exports_v9/`)
1. **Palm Chassis**: `palm_v9.stl` (Qty: 1) – Monolithic palm with splayed MCP tongues, `0°` dorsal MCP stops, male Thumb CMC opposition pedestal, continuous `Ø2.80 mm` through-palm tendon conduits, captive `Ø2.0 mm` cross-pin dorsal elastic anchors, and Y-axis 4-bolt wrist bulkhead.
2. **Forearm 6-Servo Rack**: `forearm_servo_adapter_v9.stl` (Qty: 1) – Open-sided dual-column 6× MG90S servo rack (`82.0 mm` length × `62.0 mm` outer width, `12.7 × 23.3 mm` side-entry bays at `y = -24, -43, -62 mm`, `16.0 mm` wide central tendon channel) with matching `46.0 × 30.0 mm` 4× M3 Y-axis wrist flange (`WRIST_BOLT_XZ = (±14.0, ±9.5) mm`) and central `Ø22.0 mm` tendon funnel.
3. **Finger Phalanges** (Qty: 12 total):
   - Index: `index_proximal_v9.stl`, `index_intermediate_v9.stl`, `index_distal_v9.stl`
   - Middle: `middle_proximal_v9.stl`, `middle_intermediate_v9.stl`, `middle_distal_v9.stl`
   - Ring: `ring_proximal_v9.stl`, `ring_intermediate_v9.stl`, `ring_distal_v9.stl`
   - Pinky: `pinky_proximal_v9.stl`, `pinky_intermediate_v9.stl`, `pinky_distal_v9.stl`
4. **Thumb Phalanges** (Qty: 2 total):
   - `thumb_proximal_v9.stl`, `thumb_distal_v9.stl`
5. **Assembly Reference** *(do not print)*: `robotic_hand_full_assembly_v9.stl`

### B. Hardware, Actuation & Electronics
| Component | Specification / Dimensions | Qty | Functional Role |
| :--- | :--- | :---: | :--- |
| **Micro-Servos** | MG90S (`12.2 × 22.8 mm`, 6V) *or* HV Digital (`KST DS215MG / CLS0612W`, 7.4–8.4V) | **6 pcs** | 4 fingers + 1 thumb + 1 thumb-split / auxiliary channel |
| **Servo Mounting Screws** | M2 × 6 mm self-tapping pan-head screws (for `Ø1.8 mm` rack pilot holes on `27.5 mm` centers) | **12 pcs** | Secures 6 servos laterally into `forearm_servo_adapter_v9.stl` |
| **Winch Spools / Horns** | `Ø10 mm – Ø12 mm` grooved servo spools (or standard single/double horns drilled at `r = 5–6 mm`) | **6 pcs** | Converts `180°` servo rotation into `16–19 mm` tendon pull |
| **Joint Hinge Screws** | M3 × 12 mm (distal/pinky) & M3 × 14–16 mm (proximal/MCP/thumb) socket or button head screws | **14 pcs** | Single-axis pivot pins across all 14 clevis joints |
| **Captive Hinge Nuts** | Standard M3 hex nuts (`5.5 mm` nominal across flats; fits `5.65 mm` 6-sided sockets) | **14 pcs** | Anti-rotation pin retention inside `+X` fork walls |
| **Wrist Flange Bolts** | M3 × 16–20 mm socket head cap screws + 4× M3 hex/lock nuts | **4 sets** | Bolts `palm_v9.stl` to `forearm_servo_adapter_v9.stl` along Y-axis |
| **Flexor Tendon Line** | Braided UHMWPE Dyneema / Spectra (`Ø0.8–1.0 mm`, `50–80 lb` / `220–350 N` test) | **~4 m** | Low-creep active palmar flexion cables |
| **Dorsal Elastic Bands** | Silicone, nitrile, or TPU orthodontic/loom elastic strips (`1.5–2.0 mm` wide × `0.6–1.0 mm` thick) | **5–10 pcs** | Passive extension return inside dorsal snap-fit channels |
| **PTFE Liner (Optional)** | `2.0 mm OD × 1.0 mm ID` PTFE tubing (inside `Ø2.8 mm` palm conduits for HV builds) | **~1 m** | Minimizes capstan friction under high-voltage servo loads |
| **Cosmetic Nail Plates** | Optional press-on acrylic or `0.65 mm` 3D-printed nail plates | **5 pcs** | Bonds flush into `0.70 mm` fingernail beds, concealing elastic knots |
| **PWM Driver & MCU** | PCA9685 16-Ch I2C PWM Driver + ESP32-S3 / RP2040 + 6V–8.4V (5–10A) BEC | **1 set** | Coordinated multi-channel servo trajectory control |

---

## 4. Repository & CAD Architecture (`cad_v9/`)

- `cad_v9/digits.py`: Shared parametric phalanx generator producing organic lofted shafts, `0°` extension hard-stop clevises, `6-sided` M3 hex nut sockets with `≥ 1.25 mm` clamping walls, `z = -3.50 mm` palmar tendon bores, `Ø4.0 × 2.5 mm` distal knot pockets, `0.70 mm` fingernail beds, and crown-following dorsal snap-fit pinch-lip cutters terminating cleanly at `0.55 × length` before the cuticle (`0.58 × length`).
- `cad_v9/chassis.py`: Parametric generator for `palm_v9.stl` (splayed `rotZ` knuckle tongues with dorsal MCP stops, male Thumb CMC opposition pedestal, unbroken `Ø2.80 mm` through-palm carpal conduits, `Ø2.0 mm` transverse cross-pin dorsal elastic anchors, and Y-axis wrist bulkhead) and `forearm_servo_adapter_v9.stl` (6-bay open-access MG90S rack).
- `generate_hand_stl_v9.py`: Exports all 17 `*_v9.stl` meshes and runs `verify_v9_complete()` (watertightness, single-body `bodies == 1` check, `0°–95°` / `0°–90°` zero-collision sweeps, `-5°/-10°` hard-stop assertions, Moller-Trumbore through-palm tendon ray checks, clamping-wall gauges, and 6-servo box clearance tests).

---

## 5. Step-by-Step v9.0 Assembly Procedure

### Phase 1: Post-Processing & Bore Verification
1. Remove build-plate tree supports from clevis throats, palmar arches, and the thumb CMC pedestal. Do **not** cut or sand the `0.50 mm` inward pinch lips along the dorsal snap-fit channels.
2. Hand-twist a `3.2 mm` or `3.4 mm` drill bit (or M3 screw shaft) through all 14 `Ø3.40 mm` hinge bores to clear any FDM seam wisps so every male tongue (`4.4 mm`) pivots effortlessly inside its `5.4 mm` female clevis.
3. Pass a stiff `Ø1.0–1.5 mm` music wire through each phalanx's `Ø2.20 mm` palmar bore (`z = -3.50 mm`), each `Ø2.80 mm` through-palm conduit, and all 5 `Ø2.0 mm` transverse cross-pin elastic anchor holes on the dorsal palm and distal nail beds.

### Phase 2: Captive M3 Hex Nut Insertion & Phalanx Pinning
1. Press a standard M3 hex nut into the **6-sided hexagonal socket** (`5.65 mm` across flats) on the right (`+X`) outer clevis wall of each distal, intermediate, and proximal phalanx. A gentle squeeze with smooth-jaw pliers seats the nut flush against the `≥ 1.25 mm` clamping wall.
2. Align `*_distal_v9` onto `*_intermediate_v9` (DIP) and `*_intermediate_v9` onto `*_proximal_v9` (PIP). Insert an M3 socket/button-head screw from the left (`-X`) round counterbore (`Ø6.30 mm`), threading into the captive hex nut on `+X`.
3. **Critical Torque Check**: Snug the M3 screw until the head sits recessed inside the `-X` counterbore, then back off `1/8` turn (or use a drop of blue threadlocker on the nut threads) so the clevis prongs do **not** pinch the `4.4 mm` tongue. Verify each joint stops solidly at `0°` extension and swings freely to `95°` flexion (`90°` on thumb).
4. Pin the 4 assembled fingers onto their splayed palm knuckle tongues and pin `thumb_proximal_v9` + `thumb_distal_v9` onto the palm's male Thumb CMC pedestal.

### Phase 3: Snap-Fit Dorsal Elastic Routing & Cross-Pin Tie-Off
1. Thread a `1.5–2.0 mm` wide silicone/nitrile elastic strip through the **`Ø2.0 mm` transverse retention bore** (`y = 0.78 × L`) inside the distal phalanx fingernail bed and tie a compact overhand stopper knot. *(Optional: glue a cosmetic nail plate into the `0.70 mm` nail bed recess to conceal the knot completely.)*
2. Guide the band proximally through the open `1.8 mm` dorsal slot across the nail bed (`0.58L → 0.55L`), then **press the band straight down** with a fingernail or flat spudger through the **`1.8 mm` snap-fit pinch-lip mouth** along the distal, intermediate, and proximal mid-shafts. The band compresses elastically past the `45°` chamfers and expands captive inside the `2.6 mm` internal chamber—zero needle-threading required.
3. At the knuckle base on the dorsal surface of `palm_v9.stl`, feed the proximal end of the elastic band into the recessed anchor pocket, pass it around the **`Ø2.0 mm` transverse cross-pin hole**, pull to slight pre-tension so all 3 joints rest firmly against their `0°` dorsal hard stops, and tie off securely. Repeat for all 4 fingers and the dorsal thumb CMC anchor.

### Phase 4: Palmar Flexor Tendon Knot Seating & Through-Palm Threading
1. Cut `~65 cm` lengths of `Ø0.8–1.0 mm` braided UHMWPE Dyneema for each digit. Tie a figure-eight stopper knot (or crimp a `Ø2.5 mm` micro brass bead) at the distal end and melt-seal the tip.
2. Thread the free end proximally into the distal fingertip's palmar bore and pull taut until the knot seats completely flush inside the **`Ø4.0 mm × 2.5 mm` recessed apex knot pocket** (nothing protruding from the fingertip pad).
3. Feed the tendon sequentially through the intermediate and proximal `Ø2.20 mm` palmar bores (`z = -3.50 mm`—notice the line passes cleanly *underneath* every M3 hinge pin with zero rubbing!), straight into the matching `Ø2.80 mm` palm entry port, and out through the central axial wrist opening.

### Phase 5: Y-Axis Wrist Mating & Side-Loading 6-Servo Forearm Calibration
1. Feed all 5 (or 6) Dyneema lines from the palm's axial wrist exit into the central `Ø22 mm` front funnel of `forearm_servo_adapter_v9.stl`.
2. Mate the flat wrist bulkhead of `palm_v9.stl` against the front flange (`y = 0`) of `forearm_servo_adapter_v9.stl` and fasten with **4× M3 × 16–20 mm bolts** through the aligned Y-axis wrist bolt holes.
3. Slide the **6× micro-servos** (`MG90S`, `KST DS215MG`, or `CLS0612W`) radially inward from the open left (`-X`) and right (`+X`) sides of the forearm rack into the `12.7 × 23.3 mm` bays, routing servo leads neatly along the rear channel. Secure each servo tab with two M2 × 6 mm screws into the `Ø1.8 mm` pilot holes (`27.5 mm` centers).
4. Command all 6 servos to their fully extended `0°` neutral pulse (`e.g., 500–600 µs` or `1000 µs` depending on horn direction) BEFORE attaching spools. Wrap and anchor each Dyneema line onto its `Ø10–12 mm` winch spool with zero slack while the digits rest against their `0°` dorsal stops.

---

## 6. Testing & Calibration Protocol

1. **Passive Return Check**: With servos unpowered or slack, flex each finger to `95°` by hand and release; verify crisp snap-back to `0°` with zero elastic pop-out from the dorsal pinch-lip channels.
2. **Incremental PWM Sweep**: Ramp each channel slowly from `0%` to `100%` stroke (`~16.8 mm` for fingers, `~11.0 mm` for thumb), verifying full palmar fist closure and thumb-to-index tip pinch without driving the servo past mechanical bottom-out.
3. **Thermal & Stall Protection**: Never leave hobby-grade MG90S servos stalled at 100% PWM against rigid objects for >5 seconds; configure software PWM backoff or board-level current limiting (`800–1,200 mA` per channel).
