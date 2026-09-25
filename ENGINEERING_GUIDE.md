# Biomechanical Anthropomorphic Robotic Hand: Engineering & Fabrication Guide

## 1. Executive Summary & Kinematic Architecture
This guide details the complete mechanical fabrication, tendon-driven servo actuation, and assembly procedures for the 27-DoF biomimetic anthropomorphic robotic hand.

### Key Biomechanical Specifications:
- **Kinematic Degrees of Freedom (DoF)**:
  - **Digits II-V (Index, Middle, Ring, Pinky)**: 4 DOF per finger (MCP flex/ext, MCP abduction/adduction, PIP flex/ext, DIP flex/ext). Active tendon flexion with passive elastic extension.
  - **Digit I (Thumb)**: 5 DOF with dual-axis CMC saddle-joint motion (palmar abduction/adduction + pronation/opposition), condyloid MCP (flex/ext + ab/adduction), and terminal IP flexion.
  - **Palmar Arch**: 2 DOF active bowl-shaped cupping (4th & 5th metacarpal flexion + thenar translation).
  - **Wrist & Forearm**: 4 DOF multi-axis gimbal (flexion/extension, radial/ulnar deviation, and forearm pronation/supination).
- **Thumb CMC Mounting Euler Angles (Intrinsic XYZ)**:
  - $\alpha_x = +0.85\text{ rad } (+48.7^\circ)$: Palmar abduction (anterior tilt towards grasping space).
  - $\beta_y = -0.35\text{ rad } (-20.1^\circ)$: Pronation (orients thumb pulp medially towards digits II-IV).
  - $\gamma_z = +0.50\text{ rad } (+28.6^\circ)$: Radial abduction (radial clearance from the index knuckle).
- **Fasteners & Pinning**:
  - Precision 3.0 mm Dowel Pins or M3 metric socket-head screws for joint hinges.
  - Low-friction brass/PTFE sleeve bushings or direct reamed polymer bores.

---

## 2. Bill of Materials (BOM)

### A. 3D Printed / Composite Components
1. **Palm Chassis** (`palm.stl`) – Monolithic palm chassis with carved thenar/hypothenar mounds, 4-digit metacarpal knuckle clevises, and thumb CMC saddle receptacle.
2. **Forearm Servo Bay & Bracket Array** (`forearm_servo_adapter.stl`) – Modular high-torque micro-servo mounting rack with low-friction tendon guide conduits.
3. **Digit Phalanges** (Per finger: Proximal, Intermediate, Distal):
   - Index: `index_proximal.stl`, `index_intermediate.stl`, `index_distal.stl`
   - Middle: `middle_proximal.stl`, `middle_intermediate.stl`, `middle_distal.stl`
   - Ring: `ring_proximal.stl`, `ring_intermediate.stl`, `ring_distal.stl`
   - Pinky: `pinky_proximal.stl`, `pinky_intermediate.stl`, `pinky_distal.stl`
4. **Thumb Phalanges**:
   - `thumb_metacarpal.stl` (or proximal saddle link)
   - `thumb_proximal.stl`
   - `thumb_distal.stl`

### B. Actuation & Fasteners
| Part | Spec / Dimensions | Quantity | Purpose |
|---|---|---|---|
| Servos | High-Torque Digital Metal-Gear Micro Servos (MG90S / KST DS215MG / CLS0612W) | 16–20 pcs | Precision tendon actuation (Digits, Thumb, Wrist) |
| Tendon Wire | UHMWPE Dyneema / Braided Spectra line (0.8 - 1.0 mm, 50-80 lb test) | ~6 meters | Flexor & abductor tendon actuation lines |
| Extension Springs / Elastic Bands | High-cycle silicone bands / 0.4mm wire dia helical torsion springs | 15–20 pcs | Dorsal joint extension return mechanics |
| Joint Pivot Pins | M3 x 14mm / M3 x 18mm Dowel Pins or Button Head Screws | ~20 pcs | Precision hinge pivots |
| Locknuts / Retaining Clips | M3 nylon locknuts | ~20 pcs | Pin retention |
| PTFE Tubing | 2mm OD x 1mm ID | ~2 meters | Low-friction Bowden conduit sleeve routing |
| Servo Driver Board | PCA9685 16-Channel 12-Bit I2C PWM or 24-Ch Serial Bus Controller | 1–2 pcs | Multichannel servo PWM control |
| Micro Controller | ESP32-S3 or Teensy 4.1 (ARM Cortex-M7) | 1 pc | Motion kinematics & trajectory execution |

---

## 3. Printing Recommendations & Slicing Setup

- **Material Selection**:
  - Primary Structural Parts (Palm, Phalanges, Brackets): **Carbon-Fiber Reinforced PETG, ABS, Nylon (PA12-CF), or Tough Resin** for rigid load bearing under cable tension.
  - Soft Grip Pads: **TPU 85A/95A** for high friction fingertips and palmar pads.
- **Layer Height**: 0.12 mm - 0.16 mm (critical for clean joint pin bores and smooth eyelets).
- **Perimeters / Walls**: Minimum 4-5 walls (1.6 - 2.0 mm total wall thickness) to withstand tendon pull forces (up to 35 N per finger).
- **Infill**: 40% - 50% Gyroid or Honeycomb.
- **Support**: Normal or Tree supports enabled. Block supports inside the $\varnothing 2.0\text{ mm}$ cable guide channels; clean channels post-print with a 1.5mm drill bit or heated wire.

---

## 4. Tendon-Driven Servo Routing & Forearm Integration

### A. Routing Channels & Fairleads
1. **Digits II to V (Flexors & Abductors)**:
   - Each phalanx contains an internal or palmar routing bore ($\varnothing 1.5 - 2.0\text{ mm}$).
   - The flexor cable terminates inside the **Distal Phalanx** anchored via an internal knot, brass crimp bead, or M2 grub screw.
   - Secondary lateral tendons pass through the metacarpal bases to dedicated abduction servos for side-to-side spread.
   - Tendons route over joint pivot axes through smooth radiused fairleads down into the palm cavity and into PTFE Bowden guide tubes.
2. **Thumb Routing (Triple Tendon Multi-Axis)**:
   - **Tendon 1 (Flexor)**: Originates at the thumb distal phalanx, traverses IP and MCP palmar eyelets into Servo 13.
   - **Tendon 2 (Pronation / Opposition)**: Anchored to the lateral tubercle of the thumb metacarpal/proximal phalanx, routed through the ulnar thenar guide into Servo 15 to sweep the thumb across the palm.
   - **Tendon 3 (Palmar Abductor)**: Routed to Servo 16 for radial/palmar clearance.
3. **Forearm Servo Bay**:
   - High-density modular servo rack mounted directly behind the wrist gimbal.
   - Each servo output features an aluminum or printed mini-winch spool ($\varnothing 10–14\text{ mm}$) to translate angular rotation into linear tendon travel.

---

## 5. Assembly Step-by-Step

### Phase 1: Post-Processing & Bore Reaming
1. Carefully remove support material around all joint clevises.
2. Run a 3.0 mm drill bit by hand through every pivot hinge hole to ensure silky-smooth rotation without radial slop.
3. Clean out cable passages using a 1.2 mm - 1.5 mm music wire or flexible guitar string.

### Phase 2: Finger Assembly & Return Springs
1. Connect Distal to Intermediate phalanx with a 3mm dowel pin.
2. Connect Intermediate to Proximal phalanx.
3. Insert dorsal elastic band or miniature torsion spring along the dorsal groove.
4. Verify instant spring return motion under zero tendon load.
5. Repeat for Index, Middle, Ring, and Pinky digits.

### Phase 3: Palm & Thumb Assembly
1. Pin each assembled finger into its corresponding palm knuckle clevis.
2. Assemble the thumb saddle joint onto the thenar mount using the specified compound angle.
3. Attach the thumb proximal and distal links. Ensure full collision-free range of motion across the palm.

### Phase 4: Servo Mounting & Tendon Rigging
1. Install servos into the forearm rack bays. Center all servos at neutral PWM ($1500\,\mu\text{s}$).
2. Thread Dyneema lines through the distal termination anchor.
3. Feed down through intermediate and proximal guides, through the palm internal routing tubes, and through PTFE sleeve conduits into the forearm bay.
4. Secure lines onto the servo spools with slight pre-tensioning using brass crimps or tensioner screws.

---

## 6. Testing, Calibration, and Control Integration

1. **Manual Articulation**: Verify full closure into a cylindrical grasp, spherical grasp, and tip-to-tip pinch.
2. **Servo Stroke Calibration**:
   - Typical stroke required per digit: 18 - 26 mm of tendon travel.
   - On a 12mm radius winch horn, this corresponds to approximately $90^\circ - 130^\circ$ of servo rotation.
3. **Current-Based Stall Detection**:
   - Calibrate the microcontroller current sense threshold to 800–1200 mA per servo to avoid motor burnout upon firm object contact.
4. **Trajectory Mapping**:
   - Program coordinated synergies (e.g., synchronous MCP, PIP, and DIP curling) to create smooth, natural human-like grasps.
