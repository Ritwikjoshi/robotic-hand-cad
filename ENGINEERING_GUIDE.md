# Biomechanical Anthropomorphic Robotic Hand: Engineering & Fabrication Guide

## 1. Executive Summary & Kinematic Architecture
This guide details the complete mechanical fabrication, tendon-driven actuation, and assembly procedures for the 5-digit anthropomorphic robotic hand.

### Key Biomechanical Specifications:
- **Kinematic Degrees of Freedom (DOF)**:
  - Digits II-V (Index, Middle, Ring, Pinky): 3 DOF per finger (MCP, PIP, DIP). Active tendon flexion with passive elastic extension.
  - Digit I (Thumb): Complex 3 DOF with dual-axis CMC saddle-joint motion (abduction/opposition + pronation/circumduction) and coupled MCP/IP flexion.
- **Thumb CMC Mounting Euler Angles (Intrinsic XYZ)**:
  - $\alpha_x = +0.85\text{ rad } (+48.7^\circ)$: Palmar abduction (anterior tilt towards the grasping space).
  - $\beta_y = -0.35\text{ rad } (-20.1^\circ)$: Pronation (orients thumb pad medially towards digits II-IV).
  - $\gamma_z = +0.50\text{ rad } (+28.6^\circ)$: Radial abduction (radial clearance from the index knuckle).
- **Fasteners & Pinning**:
  - Precision 3.0 mm Dowel Pins or M3 metric socket-head screws for joint hinges.
  - Low-friction brass/PTFE sleeve bushings (optional) or direct reamed polymer bores.

---

## 2. Bill of Materials (BOM)

### A. 3D Printed Components (FDM or SLA/SLS)
1. **Palm Chassis** () – Monolithic palm chassis with carved thenar/hypothenar mounds, 4-digit metacarpal knuckle knuckles, and thumb CMC saddle receptacle.
2. **Forearm Servo Adapter** () – 6-servo mounting bracket (2x3 array) with integrated cable feed channels.
3. **Digit Phalanges** (Per finger: Proximal, Intermediate, Distal):
   - Index: , , 
   - Middle: , , 
   - Ring: , , 
   - Pinky: , , 
4. **Thumb Phalanges**:
   -  (or proximal saddle link)
   - 
   - 

### B. Actuation & Fasteners
| Part | Spec / Dimensions | Quantity | Purpose |
|---|---|---|---|
| Servos | TowerPro MG90S (Metal Gear) or SG90 | 6 pcs | Actuation (Digits I-V + Thumb Opp.) |
| Tendon Wire | UHMWPE Dyneema / Braided Spectra line (0.8 - 1.0 mm, 50-80 lb test) | ~3 meters | Flexor tendon actuation |
| Extension Springs / Elastic Band | 0.4mm wire dia, 3mm OD, 15-25mm length | 5-9 pcs | Joint return/extension mechanics |
| Joint Pivot Pins | M3 x 14mm / M3 x 18mm Dowel Pins or Button Head Screws | ~15 pcs | Hinge joints |
| Locknuts / Retaining Clips | M3 nylon locknuts | ~15 pcs | Pin retention |
| PTFE Tubing | 2mm OD x 1mm ID | ~1 meter | Low friction tendon sleeve routing |
| Micro Bearings (Optional) | MR63ZZ (3x6x2.5mm) | Optional | Ultra-smooth joint articulation |

---

## 3. Printing Recommendations & Slicing Setup

- **Material Selection**:
  - Primary Structural Parts (Palm, Phalanges, Adapter): **PETG, ABS, or Nylon (PA12)** for fatigue resistance under cable tension.
  - Soft Grip Pads (Optional): **TPU 85A/95A** for high friction fingertips.
- **Layer Height**: 0.12 mm - 0.16 mm (critical for clean joint pin bores and smooth eyelets).
- **Perimeters / Walls**: Minimum 4-5 walls (1.6 - 2.0 mm total wall thickness) to withstand tendon pull forces (up to 30 N per finger).
- **Infill**: 35% - 50% Gyroid or Honeycomb.
- **Support**: Normal or Tree supports enabled. Block supports inside the $\varnothing 2.0\text{ mm}$ cable guide channels; clean channels post-print with a 1.5mm drill bit or heated wire.

---

## 4. Tendon Routing & Servo Integration

### A. Routing Channels
1. **Digits II to V**:
   - Each phalanx contains an internal or palmar routing bore ($\varnothing 1.5 - 2.0\text{ mm}$).
   - The cable terminates inside the **Distal Phalanx** anchored via an internal knot, brass crimp bead, or M2 grub screw.
   - Tendons route over the joint pivot axes through smooth radiused fairleads down into the palm cavity.
2. **Thumb Routing (Dual Tendon)**:
   - **Tendon 1 (Flexor)**: Originates at the thumb distal phalanx, traverses the IP and MCP palmar eyelets, passes through the thenar conduit into Servo 1.
   - **Tendon 2 (Opposition)**: Anchored to the lateral tubercle of the thumb metacarpal/proximal phalanx, routed through the ulnar thenar guide into Servo 2 to sweep the thumb into opposition across the palm.
3. **Forearm Integration**:
   - The forearm adapter connects directly to the wrist flange of the palm using 4x M3 countersunk screws.
   - Each servo output pulley features a dual-hole horn or printed mini-winch spool ($\varnothing 12\text{ mm}$).

---

## 5. Assembly Step-by-Step

### Phase 1: Post-Processing & Bore Reaming
1. Carefully remove support material around all joint clevises.
2. Run a 3.0 mm drill bit by hand through every pivot hinge hole to ensure silky-smooth rotation without radial slop.
3. Clean out cable passages using a 1.2 mm - 1.5 mm music wire or flexible guitar string.

### Phase 2: Finger Assembly
1. Connect Distal to Intermediate phalanx with a 3mm dowel pin.
2. Connect Intermediate to Proximal phalanx.
3. Insert extension return spring or elastic cord along the dorsal groove/hooks.
4. Verify smooth return motion under gravity and spring action.
5. Repeat for Index, Middle, Ring, and Pinky digits.

### Phase 3: Palm & Thumb Assembly
1. Pin each assembled finger into its corresponding palm knuckle clevis.
2. Assemble the thumb saddle joint onto the thenar mount using the specified compound angle.
3. Attach the thumb proximal and distal links. Ensure free range of motion from neutral resting radial clearance to deep ulnar opposition.

### Phase 4: Tendon Rigging & Tensioning
1. Thread Dyneema lines through the distal termination anchor.
2. Feed down through intermediate and proximal guides, through the palm internal routing tubes, and out through the wrist guide.
3. Slide PTFE guide sleeves into the wrist conduits to prevent friction groove wear.
4. Mount the 6 servos into the forearm bracket. Set servos to neutral (0 degrees).
5. Fasten lines onto the servo horns with slight pre-tensioning using tensioner adjustment screws or crimp beads.

---

## 6. Testing, Calibration, and Kinematics Validation

1. **Manual Articulation**: Verify full closure into a cylindrical grasp, spherical grasp, and tip-to-tip pinch.
2. **Servo Stroke Calibration**:
   - Typical stroke required per digit: 18 - 25 mm of tendon travel.
   - On a 12mm radius winch horn, this corresponds to approximately ^\circ - 120^\circ$ of servo rotation.
3. **Opposition Clearance**: Confirm that during opposition, the thumb clears the palm surface by 8-12 mm before contacting the index or middle fingertips.

---

# SECTION 7: 27-DoF BIOMIMETIC HYDRAULIC MUSCULOSKELETAL SPECIFICATION OVERHAUL

## 7.1 Overview of 27-DoF Anatomical Kinematic Breakdown
To achieve total anatomical parity with the human musculoskeletal system, the hand is elevated from single-cable tendon flexion to a full 27-DoF dual-acting antagonistic architecture:

1. **Digits II–V (Index, Middle, Ring, Pinky) – 16 DoF Total (4 DoF / Finger)**:
   - **MCP Joint (2 DoF)**: Metacarpophalangeal Flexion/Extension (zsh^\circ$ to ^\circ$) + Lateral Abduction/Adduction (569X15^\circ$ to 0^\circ$).
   - **PIP Joint (1 DoF)**: Proximal Interphalangeal Flexion/Extension (zsh^\circ$ to ^\circ$).
   - **DIP Joint (1 DoF)**: Distal Interphalangeal Flexion/Extension (zsh^\circ$ to ^\circ$).
2. **Digit I (Thumb) – 5 DoF Total**:
   - **CMC Joint (2 DoF)**: Trapeziometacarpal Saddle articulation with palmar abduction/adduction (zsh^\circ$ to ^\circ$) and true opposition/circumduction rotation (569X30^\circ$ to 0^\circ$).
   - **MCP Joint (2 DoF)**: Condyloid articulation featuring flexion/extension (zsh^\circ$ to ^\circ$) and supplementary radial/ulnar abduction (569X10^\circ$ to 0^\circ$).
   - **IP Joint (1 DoF)**: Interphalangeal terminal flexion/extension (zsh^\circ$ to ^\circ$).
3. **Flexible Palmar Arch (2 DoF Total)**:
   - **Ulnar Palmar Cupping (1 DoF)**: Articulation of the 4th & 5th metacarpal rays allowing the palm to collapse into a concave cup during spherical/power grasping.
   - **Thenar Eminence Translation (1 DoF)**: Adaptive compliance of the 1st metacarpal base along the transverse carpal arch.
4. **Multi-Axis Wrist & Forearm (4 DoF Total)**:
   - **Flexion / Extension (1 DoF)**: 569X70^\circ$ to 0^\circ$ via antagonistic longitudinal hydraulic pairs.
   - **Radial / Ulnar Deviation (1 DoF)**: 569X20^\circ$ to 0^\circ$ side-to-side carpal deviation.
   - **Forearm Pronation / Supination (2 DoF / Coupled Rotational Axes)**: 569X85^\circ$ to 0^\circ$ full axial rotation.

---

## 7.2 McKibben Hydraulic Artificial Muscle Fibers (36 Units)
- **Construction**:
  - Inner elastomeric silicone/viton micro-bladder (wall thickness zsh.4\text{ mm}$).
  - Outer braided Kevlar / ultra-high molecular weight polyethylene (UHMWPE) helical sleeve.
  - Active braided diameter: $\varnothing 4.0\text{ mm}$ (unpressurized) expands radially to $\varnothing 6.5\text{ mm}$ upon fluid injection.
- **Performance Characteristics**:
  - Unit weight: $\sim 3.0\text{ g}$ per muscle unit.
  - Linear tensile force: $\sim 10\text{ N}$ (.0\text{ kgf}$) per fiber at 6-8 bar (0.6-0.8 MPa) operating hydraulic pressure.
  - Response time: $< 45\text{ ms}$ for 15% longitudinal stroke contraction.
  - Mechanical compliance: Intrinsic viscoelastic fluid damping eliminates gearbox backlash, prevents shock transmission to bone structures, and delivers natural biomimetic back-drivability.

---

## 7.3 Carbon-Fiber Composite Skeletal Structures & Ligament Tethers
1. **Bone Matrix**: Short-fiber carbon-fiber reinforced PEEK / SLA composite resin providing high stiffness-to-weight ratio ( > 18\text{ GPa}$, density $\sim 1.3\text{ g/cm}^3$).
2. **Ligamentous Capsule Suspension**:
   - Rather than rigid sliding pins alone, joint hinges incorporate cross-woven UHMWPE ligament tethers replicating the volar plate, collateral ligaments, and flexor retinaculum.
   - Provides multi-axis compliance, self-centering articulation, and prevents joint subluxation under multi-directional impact loads.

---

## 7.4 Hydraulic Manifold, Valve Subsystem & Neural Control
1. **Electro-Hydraulic Micro-Valve Array**:
   - High-speed 3/2-way piezo-actuated or micro-solenoid valves switching at up to 120 Hz.
   - Miniature high-pressure micro-gear pump integrated into the proximal forearm housing with sealed deaerated distilled water/glycol hydraulic fluid.
2. **Closed-Loop Proprioceptive Sensory Integration**:
   - Piezo-resistive pressure sensors at each valve outlet monitoring instantaneous muscle fluid tension.
   - Magnetic absolute rotary encoders (AS5600 / hall-effect) at MCP and CMC joints for angular telemetry.
3. **Deep Neural Network (DNN) Control Pipeline**:
   - Direct intent-to-pressure policy network bypassing conventional singular matrix inversions.
   - Real-time vision-based hand-tracking translates human teleoperation or grasp targets directly into antagonistic chamber pressures.
