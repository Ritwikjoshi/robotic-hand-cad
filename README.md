# Parametric Anthropomorphic 5-Finger Robotic Hand

A fully parametric, 3D-printable anthropomorphic robotic hand designed for tendon-driven actuation, underactuated compliant grasping, and robotic arm mounting.

## Project Structure

- **`robotic_hand.scad`**: Fully parametric OpenSCAD source file. Change pin tolerances, hand dimensions, finger scale factors, or render isolated components.
- **`generate_hand_stl.py`**: Python script using `trimesh` & `manifold3d` to procedurally compile watertight CAD models and export STLs.
- **`index.html`**: Interactive WebGL (Three.js) 3D model viewer with live forward/inverse kinematics, grasp presets (Fist, Pinch, Point, Open), exploded view, and wireframe analysis.
- **`stl_exports/`**:
  - `palm.stl`: Main chassis with knuckle clevises, thumb abductor socket, internal servo routing cavity, and ISO 4x M3 wrist flange.
  - `proximal_phalanx.stl`: Base knuckle finger segment with flexor/extensor tendon bores.
  - `intermediate_phalanx.stl`: Mid segment with dual clevis joints.
  - `distal_phalanx.stl`: Fingertip with high-friction pad surface and tendon termination knot pocket.
  - `robotic_hand_full_assembly.stl`: Complete pre-assembled 5-finger articulated hand.

## Hardware & Fabrication Specifications

| Specification | Dimension / Value |
|---|---|
| **Degrees of Freedom (DOF)** | 16 kinematic joints (10-15 active via tendon cables) |
| **Joint Hinge Pins** | 3.0 mm stainless steel dowel pins or M3 bolts (L=16–22mm) |
| **Tendon Channel Dia** | 1.5 mm (compatible with 0.8–1.2mm Dyneema / Spectra / steel wire) |
| **Palm Base Size** | 72 mm (W) × 82 mm (L) × 20 mm (H) |
| **Wrist Mounting Interface** | ISO robotic circular flange pattern (4x M3 on Ø36mm BCD) |
| **Recommended Materials** | PETG, PLA-CF, or PA12 Nylon (FDM) / Tough Resin (SLA) |
| **Infill & Walls** | 4 perimeters, 40-50% gyroid infill for structural stiffness |

## Viewing & Customizing

### 1. View the 3D Model Interactively
Simply open `index.html` in your browser (e.g. `open index.html` on macOS) to test finger movement and grasp kinematics.

### 2. Regenerate STLs
To adjust parametric dimensions, edit `generate_hand_stl.py` or `robotic_hand.scad` and run:
```bash
./venv/bin/python generate_hand_stl.py
```
