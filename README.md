# Anthropomorphic 5-Finger Robotic Hand (Organic / Biomimetic CAD)

A parametric, 3D-printable anthropomorphic robotic hand designed with **ergonomically curved, human-like finger phalanxes, rounded palmar contours, and tactile pulp cushions**, combining biomimetic aesthetics with precise mechanical pin joints and tendon routing.

## Key Features

- **Ergonomic Finger Curvature:** Tapered ellipsoid profiles with subtle resting flexion curves and palmar cushions mimicking human fingers.
- **Biomimetic Palm:** Fleshy thenar (thumb) and hypothenar (pinky) contours with anatomical metacarpal arching.
- **Tendon Channels:** Ø1.5 mm longitudinal bores for low-friction Dyneema / Spectra / Bowden cable actuation.
- **Pin Joints:** Clean 3.0 mm stainless dowel / M3 bolt hinge clearances (0.30mm tolerance).
- **Robotic Flange Mount:** Standard 4x M3 circular mounting pattern for robotic arms.
- **Interactive 3D WebGL Viewer:** Live kinematics, grasping presets, wireframe and exploded views in `index.html`.

## Project Directory

- **`index.html`**: Interactive Three.js WebGL visualizer with live joint control and grasp presets.
- **`robotic_hand.scad`**: Parametric OpenSCAD design with organic hull profiles.
- **`generate_hand_stl.py`**: Procedural watertight STL compilation script using `trimesh`, `manifold3d`, and `scipy`.
- **`stl_exports/`**:
  - `robotic_hand_full_assembly.stl`: Complete 5-finger articulated hand assembly.
  - `palm.stl`: Contoured palm base with wrist flange.
  - `proximal_phalanx.stl`: Ergonomically rounded base segment.
  - `intermediate_phalanx.stl`: Mid segment with double clevises.
  - `distal_phalanx.stl`: Fingertip with tactile pad and tendon knot anchor.

## Quick Start

Open `index.html` in your browser to inspect and manipulate the hand model in real time:
```bash
open index.html
```
