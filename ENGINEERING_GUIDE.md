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