# Biomimetic Musculoskeletal Hand Platform (27 DoF - Servo Actuated)

A fully biomimetic end-effector platform designed to mirror human musculoskeletal anatomy. It integrates **high-torque digital coreless micro-servo motors with low-friction Dyneema/UHMWPE tendon cables and passive elastic return structures**, combined with **carbon-fiber composite skeletal frameworks and ligamentous capsule suspension**.

---

## 1. System Architecture & Core Specifications

| Technical Parameter | Specification Value |
| --- | --- |
| **Degrees of Freedom (DoF)** | **27 DoF** (Exact anatomical parity with the human hand) |
| **Actuation Mechanism** | High-Torque Micro-Servo Actuators with Tendon Bowden-Routing |
| **Servo Units / Channels** | Modular 16–24 Channel Forearm / Chassis Servo Bank |
| **Total Assembly Weight** | < 2 lbs (~880 g) including motors and brackets |
| **Max Grip Force** | ~6.8 kg (15 lbs) cumulative fingertip power grasp |
| **Structural Framework** | Molded carbon-fiber/PEEK or tough engineering resin bones |
| **Tendon Tensile Rating** | 50–80 lb (220–350 N) braided UHMWPE Dyneema (Ø0.8–1.0 mm) |
| **Fatigue Threshold** | 650,000+ active operational cycles |
| **Response Latency** | < 40 ms high-speed PWM / CAN-bus command execution |

---

## 2. 27-DoF Kinematic Distribution

- **Digit I / Thumb (5 DoF)**: Carpometacarpal (CMC 2-DoF: palmar abduction + true opposition/circumduction), Metacarpophalangeal (MCP 2-DoF: flex/ext + ab/adduction), and Interphalangeal (IP 1-DoF).
- **Digits II–V (16 DoF - 4 DoF per finger)**: MCP Flexion/Extension + Abduction/Adduction, PIP joint flexion, DIP terminal curling.
- **Palmar Arch Geometry (2 DoF)**: Flexible bowl-shaped palmar arch allowing active 4th/5th ray cupping and thenar compliance around curved objects.
- **Wrist & Forearm Interface (4 DoF)**: Multi-axis flexion/extension, radial/ulnar deviation, combined with structural forearm pronation/supination.

---

## 3. Actuation, Sensing & Control Systems

- **Servo Actuation**: Digital coreless metal-gear micro servos (e.g., MG90S, KST DS215MG, KingMax CLS0612W) delivering high power density and sub-degree precision.
- **Tendon Rigging**: Ultra-high molecular weight polyethylene (UHMWPE Dyneema) lines passing through PTFE Bowden conduit liners across the wrist gimbal.
- **Antagonistic Compliance**: Dedicated active flexor tendons coupled with high-cycle silicone/torsion dorsal return elements providing passive shock absorption and back-drivability.
- **Closed-Loop Feedback**: AS5600 absolute magnetic angle sensors on major joints paired with per-channel current shunt sensing for real-time contact detection.
- **Neural Motion Processing**: Intent-to-motion policy network translates real-time human tracking streams directly into coordinated multi-channel PWM duty cycles.

---

## 4. CAD Iteration Status

- **Iteration 7 (v7.0 - Latest Production Release)**:
  - **Fingernail Bed Carving**: 0.70 mm deep anatomical nail plate recess on all distal tips (Index, Middle, Ring, Pinky, Thumb) with curved proximal cuticle arc (eponychium) and lateral folds.
  - **One-Directional Articulation**: Rigid 0° mechanical extension hard stops with full unobstructed 0°→95° palmar flexion.
  - **Continuous Dorsal Groove & Concealed Bridges**: 2.4 mm wide open groove for passive return elastic band, secured with 1.3 mm thick flush concealed bridges (zero external protrusion).
  - **Generator**: [`generate_hand_stl_v7.py`](generate_hand_stl_v7.py)
  - **Export Directory**: [`stl_exports_v7/`](stl_exports_v7/) (17 watertight solid manifold STLs)
  - **Full Documentation**: See [V7_SPECIFICATION_REPORT.md](V7_SPECIFICATION_REPORT.md)

---

## 5. Repository Contents

- `V7_SPECIFICATION_REPORT.md`: Comprehensive engineering report and verification metrics for Iteration 7.
- `generate_hand_stl_v7.py`: Iteration 7 procedural CAD compilation script with fingernail bed and one-directional hard stop generation.
- `stl_exports_v7/`: Production STL files for all 17 components of Iteration 7.
- `index.html`: Interactive Three.js WebGL visualizer featuring live 27-DoF joint controls, modular micro-servo bank, Dyneema tendon routing, real-time palmar cupping, wrist articulation, and carbon matrix shaders.
- `BIOMIMETIC_SPECIFICATION.md`: Complete technical specification and 20-channel servo-to-joint tendon mapping table.
- `ENGINEERING_GUIDE.md`: Comprehensive fabrication, 3D printing parameters, tendon rigging, and servo calibration manual.
- `robotic_hand.scad`: Parametric CAD source with tendon conduit fairleads, clevis hinge tolerances, and composite bone geometries.

---

## 6. Quick Start & Visualizer

Launch the interactive 3D WebGL musculoskeletal simulator:
```bash
open index.html
```
