# TECHNICAL SPECIFICATION: BIOMIMETIC TENDON-DRIVEN SERVO HAND PLATFORM (v9.0 & 27-DoF REFERENCE)

---

## 1. Two-Tier System Architecture & Core Specifications

To maintain strict engineering honesty between printable CAD geometry and full-platform research simulation, this specification defines **two explicit architectural tiers**:

1. **Tier 1 — v9.0 Printable Mechanical Baseline (`cad_v9/`, `stl_exports_v9/`)**: A print-ready, self-contained **14-joint underactuated tendon hand** driven by an **open-access 6-bay MG90S-footprint forearm servo rack**. Every joint uses precision M3 pin-clevis hinges with captive 6-sided hex nut sockets, rigid `0°` dorsal extension stops, a `3.50 mm` palmar flexor moment arm, and snap-fit dorsal elastic return channels.
2. **Tier 2 — 27-DoF Full Biomimetic Platform Reference (`index.html` & Expansion Roadmap)**: The complete human-parity kinematic superset modeled in the Three.js WebGL simulator (`index.html`) and mapped across the 20-channel actuation table below—intended for builders adding multi-axis wrist gimbals, active MCP abduction, active ulnar palmar cupping, external AS5600 magnetic joint encoders, and 16–24 channel HV digital servo banks.

| Technical Parameter | Tier 1: v9.0 Printable CAD Baseline (`stl_exports_v9/`) | Tier 2: 27-DoF Reference / Expansion Platform (`index.html`) |
| :--- | :--- | :--- |
| **Printable Meshes** | **16 individual single-body STLs** + 1 full assembly (`17` watertight files) | Visualized interactively in `index.html`; requires custom gimbal/abduction sub-assemblies |
| **Degrees of Freedom (DoF)** | **14 hinged DoF** (4 fingers × 3 flexion joints + Thumb CMC & IP flexion at fixed compound opposition pose) | **27 DoF** (16 finger flex/abduction + 5 thumb + 2 palmar arch cupping + 4 wrist/forearm) |
| **Joint Range & Stops** | Fingers: `0° → 95°` flexion; Thumb CMC & IP: `0° → 90°` flexion; **rigid `0°` dorsal hard stops** on all 14 joints | Full multi-axis flexion, radial/ulnar abduction, `0°–30°` palmar cupping, and wrist pitch/yaw/roll |
| **Actuator Capacity** | **6× micro-servos** (`12.2 × 22.8 mm` MG90S / KST / KingMax class) in open-sided dual-column forearm rack | **16–24 channel** modular forearm & chassis HV digital coreless servo array |
| **Palmar Flexor Moment Arm** | **`r_moment = 3.50 mm`** (`Ø2.20 mm` phalanx bore at `z = -3.50 mm`, clearing M3 pin by `≥ 0.70 mm`) | `3.50–5.0 mm` flexor/abductor fairleads + Bowden PTFE wrist liners |
| **Fingertip Force (6.0V MG90S)** | **`~1.5–1.8 N` per fingertip** (`~0.75–0.92 kgf` 5-digit pinch; `~1.2–1.6 kgf` wrapped grasp) | Scales linearly with servo torque & spool diameter (see Section 2) |
| **Fingertip Force (7.4–8.4V HV)** | **`~3.5–5.5 N` per digit** (`~1.8–2.8 kgf` 5-digit pinch; `~3.0–4.0 kgf` wrapped grasp using KST/KingMax HV servos) | **`~2.0–4.0 kgf`** cumulative multi-digit grasp (replaces unphysical legacy `6.8 kg` estimate) |
| **Structural Material** | PLA+, PETG, PA12-CF, ABS, or Tough Engineering Resin (`≥ 1.25 mm` clevis clamping walls) | Carbon-fiber reinforced nylon (PA12-CF) / PEEK / CNC aluminum wrist gimbal |
| **Tendon Line Rating** | `50–80 lb` (`220–350 N`) braided UHMWPE Dyneema (`Ø0.8–1.0 mm`) with recessed `Ø4.0 × 2.5 mm` distal knot pocket | Braided UHMWPE Dyneema (`Ø0.8–1.0 mm`) inside `2.0 mm OD × 1.0 mm ID` PTFE Bowden sleeves |
| **Dorsal Antagonistic Return** | `1.5–2.0 mm` silicone/nitrile bands pressed into **profile-following snap-fit pinch-lip channels** + `Ø2.0 mm` cross-pin palm anchors | High-cycle elastomer return bands or dual-rate helical torsion springs |
| **Position & Force Sensing** | Open-loop 12-bit PWM (`PCA9685`) + optional driver-board shunt current sensing *(no internal encoder cavities in v9 STLs)* | External contactless `AS5600` magnetic angle encoders + per-channel current feedback |

---

## 2. Biomechanical Force, Torque & Tendon Excursion Physics

Human and anthropomorphic tendon hands operate as compact **Class-3 / speed-multiplying lever mechanisms**: the flexor tendon passes close to the joint pivot (`r_moment`), trading mechanical advantage for compact knuckle geometry and rapid fingertip closure.

### 2.1 Lever Ratio Geometry in v9.0 CAD
- **Joint Pivot Axis**: Centered at `z = 0.00 mm` (`Ø3.40 mm` smooth clearance bore for an M3 hinge pin spanning `z ∈ [-1.50, +1.50] mm`).
- **Palmar Flexor Bore**: Centered at **`z = -3.50 mm`** (`Ø2.20 mm` bore spanning `z ∈ [-4.60, -2.40] mm`), guaranteeing a **`≥ 0.70 mm` solid bridge** below the pin hole and an exact tendon flexion moment arm of:
  $$r_{\text{moment}} = 3.50\text{ mm}$$
  *(Note: In legacy v8 CAD, the bore sat at `-1.79 mm`, intersecting the M3 screw shaft and halving available joint torque.)*
- **Calibrated Digit Lengths (`L_finger` from MCP pivot to apex)**:
  - **Index**: `34.0 + 25.5 + 24.0 = 83.5 mm`
  - **Middle**: `38.0 + 28.0 + 25.5 = 91.5 mm`
  - **Ring**: `35.0 + 26.0 + 23.5 = 84.5 mm`
  - **Pinky**: `30.0 + 22.0 + 20.0 = 72.0 mm`
  - **Thumb** (CMC to apex): `33.0 + 27.0 = 60.0 mm`
- **Effective Lever Ratio**: For a primary finger (`L_finger ≈ 85 mm` extended; `L_eff ≈ 60–75 mm` in a semi-curled pinch):
  $$\frac{r_{\text{moment}}}{L_{\text{finger}}} = \frac{3.50\text{ mm}}{85.0\text{ mm}} \approx \frac{1}{24.3} \approx 0.0412$$

### 2.2 Actuator Torque-to-Fingertip Force Formulation
Given servo stall torque $\tau_{\text{servo}}$, winch spool radius $r_{\text{spool}}$, conduit/fairlead transmission efficiency $\eta \approx 0.80\text{–}0.85$, and dorsal elastic band return preload $F_{\text{elastic}} \approx 0.15\text{–}0.25\text{ N}$ at the fingertip:

$$T_{\text{cable}} = \frac{\tau_{\text{servo}}}{r_{\text{spool}}}, \qquad \tau_{\text{MCP}} = \eta \cdot T_{\text{cable}} \cdot r_{\text{moment}}, \qquad F_{\text{digit}} = \frac{\tau_{\text{MCP}}}{L_{\text{eff}}} - F_{\text{elastic}}$$

| Servo Model & Voltage | Stall Torque ($\tau_{\text{servo}}$) | Spool / Horn Radius ($r_{\text{spool}}$) | Peak Tendon Pull ($T_{\text{cable}}$) | Net MCP Torque ($\eta=0.82$) | Realistic Force per Digit ($F_{\text{digit}}$) | Cumulative 5-Digit Pinch / Wrap Grasp |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **TowerPro MG90S** (6.0V Standard) | `2.2 kg·cm` (`216 N·mm`) | `5.5 mm` (`Ø11 mm`) | `39.3 N` | `112.8 N·mm` | **`~1.5 – 1.8 N`** (`150–185 gf`) | `~7.5–9.0 N` (`0.75–0.92 kgf` pinch; `~1.2–1.6 kgf` wrap) |
| **KST DS215MG V3** (7.4V HV Digital) | `3.7 kg·cm` (`363 N·mm`) | `5.0 mm` (`Ø10 mm`) | `72.6 N` | `208.4 N·mm` | **`~3.5 – 4.1 N`** (`355–420 gf`) | `~17.5–20.5 N` (`1.8–2.1 kgf` pinch; `~2.8–3.3 kgf` wrap) |
| **KingMax CLS0612W / KST X08 Plus** (8.4V HV Coreless) | `5.2–5.3 kg·cm` (`515 N·mm`) | `5.0 mm` (`Ø10 mm`) | `103.0 N` | `295.6 N·mm` | **`~4.5 – 5.5 N`** (`460–560 gf`) | `~22.5–27.5 N` (`2.3–2.8 kgf` pinch; `~3.2–4.0 kgf` wrap) |

> **Engineering Note on Servo Choice**: All three servo families above share the micro-servo mounting class (`~12 × 23 mm` body; check tab height on sub-micro X08 shims if used) and keep cable tension (`39–103 N`) safely below the `220–350 N` breaking strength of `Ø0.8–1.0 mm` Dyneema and within the structural shear capacity of 4–5 wall PETG/PA12-CF phalanges (`≥ 1.25 mm` clevis clamping webs).

### 2.3 Linear Tendon Stroke & Winch Spool Sizing
- **3-Joint Finger (`0° → 95°` at MCP, PIP, DIP; $\Sigma\theta \approx 275^\circ = 4.80\text{ rad}$)**:
  $$\Delta L_{\text{finger}} \approx 3 \times r_{\text{moment}} \times \bar{\theta} = 3.50\text{ mm} \times 4.80\text{ rad} \approx \mathbf{16.8\text{ mm}} \quad (\text{design budget: } 16\text{–}19\text{ mm})$$
- **2-Joint Thumb (`0° → 90°` at CMC & IP; $\Sigma\theta = 180^\circ = \pi\text{ rad}$)**:
  $$\Delta L_{\text{thumb}} \approx 3.50\text{ mm} \times \pi \approx \mathbf{11.0\text{ mm}} \quad (\text{design budget: } 10\text{–}13\text{ mm})$$
- **Spool Sweep Compatibility**: A compact **`Ø10 mm` (`r = 5.0 mm`)** to **`Ø12 mm` (`r = 6.0 mm`)** winch spool yields `15.7 mm` to `18.8 mm` of linear tendon pull over a standard `180°` servo arc—achieving complete fist closure without multi-turn cable overlap.

---

## 3. Kinematic Distribution: v9.0 Hardware vs. 27-DoF Reference

### 3.1 Tier 1 — v9.0 Printable Hardware (14 Single-Axis Hinges)
- **Digits II–V (12 Hinges — 3 DoF per finger)**:
  - Single-axis MCP (`0°–95°`), PIP (`0°–95°`), and DIP (`0°–95°`) pin-clevis hinges with rigid `0°` dorsal extension hard stops.
  - Static anatomical finger splay baked into the palm knuckle tongues (`rotZ`: Index `+0.07 rad`, Middle `+0.02 rad`, Ring `-0.04 rad`, Pinky `-0.10 rad`) for natural grasp convergence without extra abduction servos.
- **Digit I / Thumb (2 Hinges — 2 DoF)**:
  - Male CMC hinge tongue mounted on a sculpted radial-palmar thenar pedestal at intrinsic XYZ Euler angles $\alpha_x = +0.85\text{ rad } (+48.7^\circ)$, $\beta_y = -0.35\text{ rad } (-20.1^\circ)$, $\gamma_z = +0.50\text{ rad } (+28.6^\circ)$, sweeping the thumb directly across the palm in true opposition (`0°–90°`), paired with a distal IP hinge (`0°–90°`) and rigid `0°` extension stops on both joints.
- **Wrist Interface (Rigid 4-Bolt Y-Axis Bulkhead)**:
  - Planar XZ mating flange at `WRIST_INTERFACE_Y` with 4× M3 Y-axis bolts and an unobstructed central carpal tendon passage into `forearm_servo_adapter_v9.stl`.

### 3.2 Tier 2 — 27-DoF Full Reference Architecture (`index.html`)
- **Thumb (5 DoF)**: CMC 2-DoF (palmar abduction + opposition circumduction), MCP 2-DoF (flexion + abduction), IP 1-DoF.
- **Digits II–V (16 DoF — 4 DoF × 4)**: MCP 2-DoF (flexion/extension + active interossei abduction/adduction), PIP 1-DoF, DIP 1-DoF.
- **Palmar Arch (2 DoF)**: Active `ulnarPalmGroup` 4th/5th metacarpal cupping (`0°–30°`) + adaptive thenar compliance.
- **Wrist & Forearm (4 DoF)**: Wrist flexion/extension pitch (`±45°`), radial/ulnar deviation yaw (`±35°`), and forearm pronation/supination (`±85°`).

---

## 4. Control Systems & Electronics

- **v9.0 Baseline Controller**: 1× PCA9685 16-channel 12-bit I2C PWM driver board controlled by an ESP32-S3, RP2040, or Arduino/Teensy MCU, powered by a dedicated 6.0V (5A) or 7.4V–8.4V HV (8–10A) buck regulator (BEC).
- **Tier 2 Closed-Loop Expansion**:
  - **Current-Shunt Force Regulation**: Per-channel INA219/INA3221 high-side current sensors detect object contact and cap steady-state stall current (`800–1,200 mA` per micro-servo) to prevent thermal overload.
  - **External Joint Encoders**: Builders implementing closed-loop joint telemetry mount diametric magnets and `AS5600` I2C Hall boards externally on custom knuckle caps or wrist gimbals.

---

## 5. Tendon & Servo Channel Mapping Tables

### 5.1 Tier 1: v9.0 Printable 6-Bay Forearm Rack Mapping (`forearm_servo_adapter_v9.stl`)

The printable v9.0 forearm rack houses **6 side-loading micro-servos** arranged in two columns (`3` on `-X` radial side, `3` on `+X` ulnar side):

| Bay ID | Column / Side | Assigned Tendon Conduit | Actuated v9.0 Joints (Underactuated Coupled) | Motion Envelope | Required Stroke |
| :--- | :--- | :--- | :--- | :---: | :---: |
| **BAY-L1** | Radial (`-X`, Front) | Thumb CMC/IP Carpal Conduit | Thumb CMC Opposition + IP Flexion (Coupled FPL) | `0° – 90°` / joint | `~11–13 mm` |
| **BAY-L2** | Radial (`-X`, Mid) | Index Through-Palm Conduit | Index MCP + PIP + DIP Flexion (Coupled FDP) | `0° – 95°` / joint | `~16–18 mm` |
| **BAY-L3** | Radial (`-X`, Rear) | Middle Through-Palm Conduit | Middle MCP + PIP + DIP Flexion (Coupled FDP) | `0° – 95°` / joint | `~17–19 mm` |
| **BAY-R1** | Ulnar (`+X`, Front) | Ring Through-Palm Conduit | Ring MCP + PIP + DIP Flexion (Coupled FDP) | `0° – 95°` / joint | `~16–18 mm` |
| **BAY-R2** | Ulnar (`+X`, Mid) | Pinky Through-Palm Conduit | Pinky MCP + PIP + DIP Flexion (Coupled FDP) | `0° – 95°` / joint | `~15–17 mm` |
| **BAY-R3** | Ulnar (`+X`, Rear) | Auxiliary Central Conduit | Independent Thumb CMC vs. IP Split *or* Pinch Boost | `0° – 90°` | `~10–14 mm` |

### 5.2 Tier 2: Full 20-Channel / 27-DoF Reference Expansion Mapping (`index.html`)

| Channel ID | Muscle Group Equivalence | Target Location | Joint / DoF Actuated | Motion Range | Status in v9.0 Printable STLs |
| :--- | :--- | :--- | :--- | :---: | :--- |
| **CH01** | Flexor Digitorum Profundus (Index) | Forearm Bay L2 | Index PIP & DIP (+ Coupled MCP) Flexion | $0^\circ - 95^\circ$ / jt | **Built-in (`BAY-L2`)** |
| **CH02** | 1st Dorsal Interosseous (Index) | Palm / Expansion Bank | Index MCP Abduction / Lateral Spread | $-15^\circ - +15^\circ$ | Fixed `+4.0°` splay (`rotZ = +0.07`) |
| **CH03** | Flexor Digitorum Superficialis (Index) | Expansion Bank | Independent Index MCP Flexion | $0^\circ - 95^\circ$ | Coupled with CH01 (or `BAY-R3`) |
| **CH04** | Flexor Digitorum Profundus (Middle) | Forearm Bay L3 | Middle PIP & DIP (+ Coupled MCP) Flexion | $0^\circ - 95^\circ$ / jt | **Built-in (`BAY-L3`)** |
| **CH05** | 2nd & 3rd Interossei (Middle) | Palm / Expansion Bank | Middle Radial/Ulnar Deviation | $-10^\circ - +10^\circ$ | Fixed `+1.1°` splay (`rotZ = +0.02`) |
| **CH06** | Flexor Digitorum Superficialis (Middle) | Expansion Bank | Independent Middle MCP Flexion | $0^\circ - 95^\circ$ | Coupled with CH04 |
| **CH07** | Flexor Digitorum Profundus (Ring) | Forearm Bay R1 | Ring PIP & DIP (+ Coupled MCP) Flexion | $0^\circ - 95^\circ$ / jt | **Built-in (`BAY-R1`)** |
| **CH08** | Interossei (Ring) | Palm / Expansion Bank | Ring MCP Abduction / Spread | $-12^\circ - +12^\circ$ | Fixed `-2.3°` splay (`rotZ = -0.04`) |
| **CH09** | Flexor Digitorum Superficialis (Ring) | Expansion Bank | Independent Ring MCP Flexion | $0^\circ - 95^\circ$ | Coupled with CH07 |
| **CH10** | Flexor Digitorum Profundus (Pinky) | Forearm Bay R2 | Pinky PIP & DIP (+ Coupled MCP) Flexion | $0^\circ - 95^\circ$ / jt | **Built-in (`BAY-R2`)** |
| **CH11** | Abductor Digiti Minimi (Pinky) | Expansion Bank | Pinky MCP Abduction / Spread | $-15^\circ - +15^\circ$ | Fixed `-5.7°` splay (`rotZ = -0.10`) |
| **CH12** | Opponens Digiti Minimi (Palmar Arch) | Palm / Expansion Bank | 4th/5th Metacarpal Cupping (`ulnarPalmGroup`) | $0^\circ - 30^\circ$ | Simulated in `index.html` (monolithic v9 palm) |
| **CH13** | Flexor Pollicis Longus (Thumb IP) | Forearm Bay L1 | Thumb Terminal IP (+ Coupled CMC) Flexion | $0^\circ - 90^\circ$ | **Built-in (`BAY-L1`)** |
| **CH14** | Flexor Pollicis Brevis (Thumb CMC/MCP) | Forearm Bay R3 | Independent Thumb Proximal Flexion | $0^\circ - 90^\circ$ | **Optional Split (`BAY-R3`)** |
| **CH15** | Opponens Pollicis (Thumb CMC) | Expansion Bank | Active Thumb Pronation / Opposition | $-30^\circ - +40^\circ$ | Fixed compound CMC pedestal (`+48.7°, -20.1°, +28.6°`) |
| **CH16** | Abductor Pollicis Brevis (Thumb CMC) | Expansion Bank | Active Thumb Palmar Abduction | $0^\circ - 60^\circ$ | Fixed compound CMC pedestal |
| **CH17** | Adductor Pollicis (Thumb Power) | Expansion Bank | Thumb Transverse Adduction Pinch | $0^\circ - 35^\circ$ | Simulated in `index.html` |
| **CH18** | Flexor / Extensor Carpi | External Wrist Gimbal | Wrist Flexion / Extension Pitch | $-45^\circ - +45^\circ$ | Rigid 4-bolt Y-axis flange in v9 STLs |
| **CH19** | Carpi Radialis / Ulnaris | External Wrist Gimbal | Wrist Radial / Ulnar Yaw | $-35^\circ - +35^\circ$ | Rigid 4-bolt Y-axis flange in v9 STLs |
| **CH20** | Pronator Teres / Quadratus | External Forearm Roll | Forearm Pronation / Supination Roll | $-85^\circ - +85^\circ$ | Simulated in `index.html` |
