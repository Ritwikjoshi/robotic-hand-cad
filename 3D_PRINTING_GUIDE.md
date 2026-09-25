# Anycubic Kobra 2 Neo: Phased 3D Printing & Fabrication Guide
### Biomechanical Anthropomorphic Robotic Hand (Servo & Tendon Actuated)

---

## 1. Machine & Slicer Overview: Anycubic Kobra 2 Neo
- **Print Volume**: $220 \times 220 \times 250\text{ mm}$ (Entire hand fits comfortably on this bed).
- **Extruder Type**: Direct Drive Extruder (ideal for precise retractions and optional flexible TPU finger pads).
- **Build Plate**: Magnetic Spring Steel PEI Sheet (textured).
- **Recommended Slicer**: Anycubic Slicer, PrusaSlicer, or Cura (Anycubic Kobra 2 Neo profile).

---

## 2. Global Slicer Parameters for Kobra 2 Neo

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
| **Support Blockers** | Place support blockers over $\varnothing 1.5 - 2.0\text{ mm}$ internal tendon bores | N/A |

---

## 3. Phased Printing Plan (Bed Batches)

To prevent failures, make post-processing easy, and maintain perfect organization, print in the following **4 chronological phases**:

```
Phase 1: Palm Core & Servo Adapter
└── palm.stl (Qty: 1)
└── forearm_servo_adapter.stl (Qty: 1)

Phase 2: Thumb Biomechanical Digits
└── thumb_proximal.stl (Qty: 1)
└── thumb_distal.stl (Qty: 1)

Phase 3: Primary Grasp Digits (Index & Middle)
└── index_proximal.stl (Qty: 1)
└── index_intermediate.stl (Qty: 1)
└── index_distal.stl (Qty: 1)
└── middle_proximal.stl (Qty: 1)
└── middle_intermediate.stl (Qty: 1)
└── middle_distal.stl (Qty: 1)

Phase 4: Ulnar Support Digits (Ring & Pinky)
└── ring_proximal.stl (Qty: 1)
└── ring_intermediate.stl (Qty: 1)
└── ring_distal.stl (Qty: 1)
└── pinky_proximal.stl (Qty: 1)
└── pinky_intermediate.stl (Qty: 1)
└── pinky_distal.stl (Qty: 1)
```

---

### Detailed Phase Specifications

### Phase 1: Structural Chassis & Servo Bay
*Estimated Print Time: ~5.5 - 7.5 hours*

| File Name | Qty | Bed Orientation | Support Required |
|---|---|---|---|
| `palm.stl` | **1** | Lay flat on dorsal back surface (wrist flange facing down or palm facing up) | Tree supports under thenar overhang & knuckle clevises |
| `forearm_servo_adapter.stl` | **1** | Lay vertical on flat circular base | Minimal tree supports under servo flange tabs |

> **Pro Tip**: Use a brim of 4–5 mm on the Kobra 2 Neo textured PEI sheet to prevent corners of the palm from lifting.

---

### Phase 2: Thumb Biomechanical Digits
*Estimated Print Time: ~1.5 - 2.0 hours*

| File Name | Qty | Bed Orientation | Support Required |
|---|---|---|---|
| `thumb_proximal.stl` | **1** | Horizontal on lateral flat edge | Tree supports under joint clevis |
| `thumb_distal.stl` | **1** | Upright on base clevis or angled $45^\circ$ | Support under fingertip pulp pad |

> **Verification**: Test-fit a 3.0 mm dowel pin or M3 screw through the thumb hinge immediately after printing.

---

### Phase 3: Primary Grasp Digits (Index & Middle)
*Estimated Print Time: ~3.0 - 4.0 hours*

| File Name | Qty | Bed Orientation | Support Required |
|---|---|---|---|
| `index_proximal.stl` | **1** | Lateral side down on bed | Tree supports under female knuckle clevis |
| `index_intermediate.stl` | **1** | Lateral side down on bed | Tree supports under male clevis tongue |
| `index_distal.stl` | **1** | Angled $45^\circ$ or upright | Support under tactile pad |
| `middle_proximal.stl` | **1** | Lateral side down on bed | Tree supports under female knuckle clevis |
| `middle_intermediate.stl` | **1** | Lateral side down on bed | Tree supports under male clevis tongue |
| `middle_distal.stl` | **1** | Angled $45^\circ$ or upright | Support under tactile pad |

---

### Phase 4: Ulnar Support Digits (Ring & Pinky)
*Estimated Print Time: ~2.5 - 3.5 hours*

| File Name | Qty | Bed Orientation | Support Required |
|---|---|---|---|
| `ring_proximal.stl` | **1** | Lateral side down on bed | Tree supports under clevis |
| `ring_intermediate.stl` | **1** | Lateral side down on bed | Tree supports under clevis |
| `ring_distal.stl` | **1** | Angled $45^\circ$ or upright | Support under tactile pad |
| `pinky_proximal.stl` | **1** | Lateral side down on bed | Tree supports under clevis |
| `pinky_intermediate.stl` | **1** | Lateral side down on bed | Tree supports under clevis |
| `pinky_distal.stl` | **1** | Angled $45^\circ$ or upright | Support under tactile pad |

---

## 4. Master Parts Inventory & Quantity Check

| Part Category | STL File Name | Required Quantity | Cumulative Total |
|---|---|---|---|
| **Chassis** | `palm.stl` | **1** | 1 |
| **Forearm** | `forearm_servo_adapter.stl` | **1** | 2 |
| **Thumb** | `thumb_proximal.stl` | **1** | 3 |
| **Thumb** | `thumb_distal.stl` | **1** | 4 |
| **Index** | `index_proximal.stl` | **1** | 5 |
| **Index** | `index_intermediate.stl` | **1** | 6 |
| **Index** | `index_distal.stl` | **1** | 7 |
| **Middle** | `middle_proximal.stl` | **1** | 8 |
| **Middle** | `middle_intermediate.stl` | **1** | 9 |
| **Middle** | `middle_distal.stl` | **1** | 10 |
| **Ring** | `ring_proximal.stl` | **1** | 11 |
| **Ring** | `ring_intermediate.stl` | **1** | 12 |
| **Ring** | `ring_distal.stl` | **1** | 13 |
| **Pinky** | `pinky_proximal.stl` | **1** | 14 |
| **Pinky** | `pinky_intermediate.stl` | **1** | 15 |
| **Pinky** | `pinky_distal.stl` | **1** | 16 |
| **Reference** | `robotic_hand_full_assembly.stl` | *(CAD reference only)* | — |
| **Reference** | `thumb_assembly.stl` | *(CAD reference only)* | — |

**Total printable structural parts**: **16 individual pieces**.

---

## 5. Post-Processing & Calibration for Anycubic Kobra 2 Neo

1. **Pin Hole Post-Processing**:
   - 3D printer shrinkage typically tightens small holes by $0.1 - 0.2\text{ mm}$.
   - Take a **3.0 mm drill bit** and ream every hinge pin hole by hand (do not use high-speed power drills which can melt the polymer).
2. **Tendon Bore Clearing**:
   - Run a $1.2\text{ mm}$ - $1.5\text{ mm}$ guitar string (G or D string) or rigid floral wire through every internal cable routing channel to ensure zero burrs or snag points.
3. **Clevis Dressing**:
   - Use 400-grit wet sandpaper on the sides of the male hinge tongues for friction-free movement.
4. **Bed Leveling (Kobra 2 Neo LeviQ 2.0)**:
   - Run the automatic LeviQ 2.0 bed leveling calibration before Phase 1 to guarantee uniform first-layer adhesion across the entire palm surface.
