# Biomimetic Musculoskeletal Hand Platform (27 DoF)

A fully biomimetic end-effector platform designed to mirror human musculoskeletal anatomy. It replaces traditional motor-and-gear configurations with a **hydraulic artificial muscle network (36 McKibben units)** integrated directly with **carbon-fiber composite skeletal structures and ligamentous suspension**.

---

## 1. System Architecture & Core Specifications

| Technical Parameter | Specification Value |
| --- | --- |
| **Degrees of Freedom (DoF)** | **27 DoF** (Exact anatomical parity with human hand) |
| **Actuation Mechanism** | Hydraulic Artificial Muscle Fibers (~36 units) |
| **Total Assembly Weight** | < 2 lbs (~900 g) |
| **Max Grip Force** | ~6.8 kg (15 lbs) antagonistic grip |
| **Structural Framework** | Molded carbon-fiber/PEEK composite bones + ligament tethers |
| **Fatigue Threshold** | 650,000+ active operational cycles |
| **Contraction Dynamics** | ~3g per fiber, < 50 ms response time |

---

## 2. 27-DoF Kinematic Distribution

- **Digit I / Thumb (5 DoF)**: Carpometacarpal (CMC 2-DoF: palmar abduction + opposition), Metacarpophalangeal (MCP 2-DoF: flex/ext + ab/adduction), and Interphalangeal (IP 1-DoF).
- **Digits II–V (16 DoF - 4 DoF per finger)**: MCP Flexion/Extension + Abduction/Adduction, PIP joint flexion, DIP terminal curling.
- **Palmar Arch Geometry (2 DoF)**: Flexible bowl-shaped palmar arch allowing active 4th/5th ray cupping and thenar compliance.
- **Wrist & Forearm Interface (4 DoF)**: Multi-axis flexion/extension, radial/ulnar deviation, combined with structural forearm pronation/supination.

---

## 3. Actuation, Sensing & Neural Control

- **McKibben Fluid-Driven Actuators**: Antagonistic flexor/extensor pairs provide inherent viscoelastic compliance, passive shock absorption, and back-drivability without heavy software impedance loops.
- **Fluid Power Subsystem**: Compact high-pressure internal pump routing deaerated hydraulic fluid through high-speed electro-hydraulic micro-valve arrays.
- **Proprioceptive Feedback**: Integrated piezoresistive pressure sensors and absolute angle encoders stream closed-loop metrics for adaptive dynamic grip modulation.
- **Neural Motion Processing**: Intent-to-motion policy network translates real-time human tracking streams directly into antagonistic hydraulic valve pressures.

---

## 4. Repository Contents

- `index.html`: Interactive Three.js WebGL visualizer featuring live 27-DoF joint controls, 36-muscle hydraulic overlay, real-time palmar cupping, wrist articulation, and biological/carbon-matrix shader modes.
- `BIOMIMETIC_SPECIFICATION.md`: Complete technical specification and 36-muscle antagonistic anatomical mapping table.
- `ENGINEERING_GUIDE.md`: Comprehensive fabrication, carbon-fiber composite layups, hydraulic manifold plumbing, and assembly procedures.
- `robotic_hand.scad`: Parametric CAD source with antagonistic hydraulic conduit fairleads and composite bone geometries.
- `generate_hand_stl.py`: Procedural STL compilation script for 3D printing and mold machining.

---

## 5. Quick Start & Visualizer

Launch the interactive 3D WebGL musculoskeletal simulator:
```bash
open index.html
```
