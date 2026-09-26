# Anycubic Kobra 2 Neo: Phased 3D Printing & Fabrication Guide
### Biomechanical Anthropomorphic Robotic Hand (Iteration 2 - Calibrated Scale)

---

## 1. Changelog: Iteration 2 (v2) Physical Calibration
Based on real physical test print feedback on the Anycubic Kobra 2 Neo:
1. **Finger Width & Diameter Enlarged**:
   - Previous index finger printed at ~8 mm (too slender).
   - **Iteration 2 Width**: Calibrated to **12.0 mm – 13.5 mm** across all phalanges for realistic anthropomorphic anatomy and structural strength.
2. **Internal Holes & Tendon Pass-Throughs Widened**:
   - Internal tendon bores enlarged from 1.5–1.8 mm to **2.5 mm – 3.0 mm** (zero friction, easy threading of Dyneema line / Bowden PTFE liners without needle threading tools).
   - Hinge pin bores enlarged to **3.5 mm diameter** (specifically accommodates thermal FDM shrinkage so 3.0 mm dowel pins / M3 screws rotate smoothly without binding).
3. **Clevis Hinge Clearance Adjusted**:
   - Increased female clevis pocket width to **5.4 mm** with **4.4 mm** male tongue (0.5 mm lateral air gap on each side).
4. **Version Safety**:
   - Previous files remain safe in `stl_exports/`.
   - All newly generated, calibrated parts are in **`stl_exports_v2/`** with `_v2.stl` suffixes.

---

## 2. Machine & Slicer Overview: Anycubic Kobra 2 Neo
- **Print Volume**: $220 \times 220 \times 250\text{ mm}$ (Entire hand fits comfortably on this bed).
- **Extruder Type**: Direct Drive Extruder (ideal for precise retractions and optional flexible TPU finger pads).
- **Build Plate**: Magnetic Spring Steel PEI Sheet (textured).
- **Recommended Slicer**: Anycubic Slicer, PrusaSlicer, or Cura (Anycubic Kobra 2 Neo profile).

---

## 3. Global Slicer Parameters for Kobra 2 Neo

| Setting | Structural Parts (PETG / PLA+) | Flexible Pads (Optional TPU 95A) |
|---|---|---|
| **Nozzle Temperature** | $210^\circ - 215^\circ\text{C}$ (PLA+) / $235^\circ\text{C}$ (PETG) | $220^\circ - 225^\circ\text{C}$ |
| **Bed Temperature** | $60^\circ\text{C}$ (PLA+) / $75^\circ\text{C}$ (PETG) | $50^\circ\text{C}$ |
| **Layer Height** | **0.12 mm – 0.16 mm** (Critical for hinge pins and tendon bores) | 0.20 mm |
| **Wall Loops / Perimeters** | **4 to 5 Walls** (Minimum 1.6 - 2.0 mm wall thickness) | 3 Walls |
| **Top / Bottom Solid Layers** | 5 Top / 5 Bottom | 4 Top / 4 Bottom |
| **Infill Density & Pattern** | **40% – 50% Gyroid** (resists multi-axial shear) | 25% Gyroid |
| **Print Speed** | $60 - 80\text{ mm/s}$ (Outer walls: $40\text{ mm/s}$) | $25 - 35\text{ mm/s}$ |
| **Retraction** | $1.5 - 2.0\text{ mm}$ @ $45\text{ mm/s}$ (Direct Drive) | $1.0\text{ mm}$ @ $20\text{ mm/s}$ |
| **Supports** | **Tree Supports (Organic)** on build plate only | None / Minimal |
| **Support Blockers** | Place cylindrical support blockers over the internal $\varnothing 2.5\text{ mm}$ tendon bores | N/A |

---

## 4. Phased Printing Plan (Bed Batches)

Print in the following **4 chronological phases** to ensure clean post-processing and easy verification:

```
Phase 1: Palm Core & Servo Adapter
└── palm_v2.stl (Qty: 1)
└── forearm_servo_adapter_v2.stl (Qty: 1)

Phase 2: Thumb Biomechanical Digits
└── thumb_proximal_v2.stl (Qty: 1)
└── thumb_distal_v2.stl (Qty: 1)

Phase 3: Primary Grasp Digits (Index & Middle)
└── index_proximal_v2.stl (Qty: 1)
└── index_intermediate_v2.stl (Qty: 1)
└── index_distal_v2.stl (Qty: 1)
└── middle_proximal_v2.stl (Qty: 1)
└── middle_intermediate_v2.stl (Qty: 1)
└── middle_distal_v2.stl (Qty: 1)

Phase 4: Ulnar Support Digits (Ring & Pinky)
└── ring_proximal_v2.stl (Qty: 1)
└── ring_intermediate_v2.stl (Qty: 1)
└── ring_distal_v2.stl (Qty: 1)
└── pinky_proximal_v2.stl (Qty: 1)
└── pinky_intermediate_v2.stl (Qty: 1)
└── pinky_distal_v2.stl (Qty: 1)
```

---

### Detailed Phase Specifications

### Phase 1: Structural Chassis & Servo Bay
*Estimated Print Time: ~6.0 - 8.0 hours*

| File Name (in `stl_exports_v2/`) | Qty | Bed Orientation | Support Required |
|---|---|---|---|
| `palm_v2.stl` | **1** | Lay flat on dorsal back surface (palm facing up) | Tree supports under thenar overhang & knuckle clevises |
| `forearm_servo_adapter_v2.stl` | **1** | Lay vertical on flat circular base | Minimal tree supports under servo flange tabs |

---

### Phase 2: Thumb Biomechanical Digits
*Estimated Print Time: ~1.5 - 2.0 hours*

| File Name (in `stl_exports_v2/`) | Qty | Bed Orientation | Support Required |
|---|---|---|---|
| `thumb_proximal_v2.stl` | **1** | Horizontal on lateral flat edge | Tree supports under joint clevis |
| `thumb_distal_v2.stl` | **1** | Upright on base clevis or angled $45^\circ$ | Support under fingertip pulp pad |

---

### Phase 3: Primary Grasp Digits (Index & Middle)
*Estimated Print Time: ~3.5 - 4.5 hours*

| File Name (in `stl_exports_v2/`) | Qty | Calibrated Width | Support Required |
|---|---|---|---|
| `index_proximal_v2.stl` | **1** | **13.5 mm** | Tree supports under clevis |
| `index_intermediate_v2.stl` | **1** | **12.0 mm** | Tree supports under clevis |
| `index_distal_v2.stl` | **1** | **12.0 mm** | Support under tactile pad |
| `middle_proximal_v2.stl` | **1** | **14.5 mm** | Tree supports under clevis |
| `middle_intermediate_v2.stl` | **1** | **13.0 mm** | Tree supports under clevis |
| `middle_distal_v2.stl` | **1** | **12.8 mm** | Support under tactile pad |

---

### Phase 4: Ulnar Support Digits (Ring & Pinky)
*Estimated Print Time: ~3.0 - 4.0 hours*

| File Name (in `stl_exports_v2/`) | Qty | Calibrated Width | Support Required |
|---|---|---|---|
| `ring_proximal_v2.stl` | **1** | **13.5 mm** | Tree supports under clevis |
| `ring_intermediate_v2.stl` | **1** | **12.2 mm** | Tree supports under clevis |
| `ring_distal_v2.stl` | **1** | **12.0 mm** | Support under tactile pad |
| `pinky_proximal_v2.stl` | **1** | **12.5 mm** | Tree supports under clevis |
| `pinky_intermediate_v2.stl` | **1** | **11.2 mm** | Tree supports under clevis |
| `pinky_distal_v2.stl` | **1** | **11.0 mm** | Support under tactile pad |

---

## 5. Master Parts Inventory (Iteration 2)

| Part Category | STL File Name (`stl_exports_v2/`) | Calibrated Width | Required Qty |
|---|---|---|:---:|
| **Chassis** | `palm_v2.stl` | 76.0 mm | **1** |
| **Forearm** | `forearm_servo_adapter_v2.stl` | 46.0 mm dia | **1** |
| **Thumb** | `thumb_proximal_v2.stl` | 14.5 mm | **1** |
| **Thumb** | `thumb_distal_v2.stl` | 13.5 mm | **1** |
| **Index** | `index_proximal_v2.stl` | **13.5 mm** | **1** |
| **Index** | `index_intermediate_v2.stl` | **12.0 mm** | **1** |
| **Index** | `index_distal_v2.stl` | **12.0 mm** | **1** |
| **Middle** | `middle_proximal_v2.stl` | 14.5 mm | **1** |
| **Middle** | `middle_intermediate_v2.stl` | 13.0 mm | **1** |
| **Middle** | `middle_distal_v2.stl` | 12.8 mm | **1** |
| **Ring** | `ring_proximal_v2.stl` | 13.5 mm | **1** |
| **Ring** | `ring_intermediate_v2.stl` | 12.2 mm | **1** |
| **Ring** | `ring_distal_v2.stl` | 12.0 mm | **1** |
| **Pinky** | `pinky_proximal_v2.stl` | 12.5 mm | **1** |
| **Pinky** | `pinky_intermediate_v2.stl` | 11.2 mm | **1** |
| **Pinky** | `pinky_distal_v2.stl` | 11.0 mm | **1** |
| *Reference* | `robotic_hand_full_assembly_v2.stl` | Assembly CAD check | — |

**Total physical pieces to print**: **16 individual parts**.

---

## 6. Post-Processing & Calibration for Anycubic Kobra 2 Neo

1. **Hinge Pin Verification**:
   - The holes are modeled at $3.5\text{ mm}$ to yield a net $3.1 - 3.2\text{ mm}$ after typical PEI/PETG shrinkage.
   - Test-insert your 3.0 mm dowel pin or M3 screw. If slightly snug, run a 3.0 mm drill bit through by hand once.
2. **Tendon Bore Threading**:
   - Internal bores are $2.5\text{ mm}$ wide, making it easy to pass Dyneema line (0.8–1.0 mm) straight through without snagging.
3. **Bed Leveling (LeviQ 2.0)**:
   - Run auto-leveling before Phase 1 to ensure a clean, unwarped palm base.
