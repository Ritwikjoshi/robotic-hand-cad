# Robotic Hand CAD Design & Surface Geometry Rules

## 1. Zero Valleys, Creases, or Notches on Outer Contours
- **Seamless Tangent Lofting**: All bone segments (proximal, intermediate, distal phalanges, and palm knuckles) must have smooth, continuously curved outer profiles without valleys, re-entrant notches, or waist pinch lines between joints and shafts.
- **Convex Hull / Smooth Lofting**: Construct solid phalanx bodies using tangent convex hulls of proximal hub, mid-shaft volume, and distal hub to guarantee continuous curvature and zero surface dips.
- **Never Use Abrupt Stepping**: Do not introduce sudden diameter drops, jagged shoulder cuts, or multi-step profile narrowing along the bone shaft.

## 2. Harmonized Finger Proportions & Unified Aesthetics
- **Proportional Sizing**: All three phalanges (proximal, intermediate, distal) of a digit must feel like cohesive components of a single organic finger.
- **Smooth Tapering**: Finger thickness and width must gently and naturally taper from proximal base to distal tip (e.g. ~12.8 mm proximal -> ~12.2 mm intermediate -> ~12.0 mm distal width).
- **Matched Joint Thickness**: The knuckle height of the proximal phalanx must closely match the intermediate phalanx knuckle height so joints transition smoothly.

## 3. Joint Clevis Fit & Hinge Alignment
- **Concentric Radii**: The distal male tongue knuckle radius must match the tongue hub radius (`hub_r = height * HUB_R_FRAC - FDM_CLEARANCE`), ensuring the tongue tip never protrudes past the socket bottom.
- **Zero Joint Interference**: The assembled joint must have 0.000 mm³ collision overlap at the hinge axis while preserving full solid wall thickness around the screw head and nut counterbores.
- **Smooth Articulation**: The hinge pin hole (Ø3.4 mm) must align with the M3 screw head counterbore (Ø6.5 mm, 2.6 mm deep) and nut pocket (Ø6.5 mm, 2.4 mm deep).

## 4. Functional Integrity Preservation
- **Do Not Alter Functional Carvings**: Any smoothing, lofting, or rounding must not modify or reduce clearance of functional features such as screw head counterbores, nut pockets, hinge pin holes, interlocking tongue/socket geometry, tendon bores, etc.
- **Maintain Clearance Margins**: Preserve the designed clearances (e.g., `FDM_CLEARANCE`, `PIN_RADIUS`, `SCREW_HEAD_R`, `NUT_R`) after aesthetic changes.
- **Validate Fit After Aesthetic Changes**: Run boolean intersection or clearance checks to verify zero overlap at joints and that all counterbore volumes remain intact.
