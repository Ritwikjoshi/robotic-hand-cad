# Anycubic Kobra 2 Neo & FDM 3D Printing Guide (Iteration 9 — v9.0)
### Biomechanical Anthropomorphic Robotic Hand (`stl_exports_v9/`)

---

## 1. Changelog: Upgrade from Legacy v2 to Iteration 9 (v9.0)

All production print files now reside in **`stl_exports_v9/`** (`*_v9.stl`), replacing legacy `stl_exports_v2/` (`*_v2.stl`). Key printability and mechanical upgrades in **v9.0**:

1. **Unobstructed `3.50 mm` Palmar Tendon Bores + Recessed Fingertip Knot Pocket**:
   - Phalanx flexor bores (`Ø2.20 mm` smooth clearance for `Ø0.8–1.0 mm` Dyneema) are positioned at `z = -3.50 mm`, providing a **`≥ 0.70 mm` solid plastic bridge** below the `Ø3.40 mm` M3 hinge pin (fixing v8's pin-to-tendon collision and nearly doubling flexion torque).
   - Distal fingertips include a recessed **`Ø4.0 mm × 2.5 mm` palmar knot pocket** so stopper knots sit flush inside the tip.
2. **True 6-Sided Captive M3 Hex Nut Sockets & `≥ 1.25 mm` Clamping Walls**:
   - Right (`+X`) clevis walls feature precision **6-sided hexagonal sockets (`5.65 mm` across flats)** that lock standard M3 hex nuts against spinning, paired with `Ø6.30 mm` screw-head counterbores on `-X`.
   - Narrow digits (such as `pinky_distal_v9.stl`) feature locally reinforced base condyle bosses guaranteeing **`≥ 1.25 mm` solid clamping walls** (`1.32–2.20 mm` measured) and `≥ 1.80 mm` pocket depth without thin-wall breakout.
3. **Support-Free `45°` Profile-Following Dorsal Snap-Fit Pinch-Lip Channels**:
   - Replaces fragile closed dorsal bridges with an open **`1.80 mm` mouth slit** expanding via **`45°` self-supporting chamfers** into a **`2.60 mm` captive undercut chamber** (`0.50 mm` retaining lips) that tracks the curved dorsal crown of every phalanx.
   - On all 5 distal phalanges, the overhanging pinch lips **terminate cleanly at `0.55 × length`** just before the **`0.70 mm` anatomical fingernail recess (`0.58 × length` cuticle)**, transitioning into a clean open `1.80 mm` slot to the `Ø2.0 mm` distal elastic knot bore.
4. **Single-Shell Monolithic Palm (`bodies == 1`), Male Thumb CMC & Cross-Pin Anchors**:
   - Continuous **`Ø2.80 mm` through-palm tendon conduits** run unbroken from all 4 finger knuckles and the male Thumb CMC opposition pedestal (`[-33.0, 24.0, -2.0] mm`) straight out the axial wrist funnel—eliminating trapped internal voids.
   - Dorsal rubber-band mounts use recessed pockets pierced by **`Ø2.0 mm` transverse cross-pin tie-holes** for positive knot retention.
5. **Y-Axis 4-Bolt Wrist Bulkhead & Side-Loading 6-Servo Forearm Rack**:
   - `palm_v9.stl` terminates in a flat `46.0 × 30.0 mm` Y-axis 4× M3 wrist bulkhead (`WRIST_BOLT_XZ = (±14.0, ±9.5) mm`) mating directly to `forearm_servo_adapter_v9.stl`—an **open-sided dual-column 6× MG90S servo rack** (`62.0 × 82.0 × 34.0 mm` bounding envelope, `12.7 × 23.3 mm` side-loading bays with `Ø1.8 mm` M2 pilot holes on `27.5 mm` centers).

---

## 2. Target Machine & Slicer Compatibility: Anycubic Kobra 2 Neo

- **Build Volume**: $220 \times 220 \times 250\text{ mm}$ (all 16 parts fit in 1–4 clean batches).
- **Nozzle**: Standard `0.40 mm` brass/hardened nozzle (`0.12–0.16 mm` layer height required for crisp `0.50 mm` snap-fit pinch lips and 6-sided M3 hex flats).
- **Extruder & Plate**: Direct-drive extruder + textured PEI spring steel sheet.
- **Recommended Slicers**: OrcaSlicer, PrusaSlicer, or AnycubicSlicer Next.

---

## 3. Global Slicer Parameters for v9.0 (`0.4 mm` Nozzle)

| Parameter | Structural Parts (PLA+ / PETG / PA12-CF) | Optional Grip / Nail Pads (TPU 95A / Acrylic) |
| :--- | :--- | :--- |
| **Nozzle Temperature** | $210^\circ\text{–}215^\circ\text{C}$ (PLA+) / $235^\circ\text{–}240^\circ\text{C}$ (PETG) | $220^\circ\text{–}225^\circ\text{C}$ (TPU 95A) |
| **Bed Temperature** | $60^\circ\text{C}$ (PLA+) / $75^\circ\text{C}$ (PETG) | $50^\circ\text{C}$ |
| **Layer Height** | **`0.12 mm` (digits) – `0.16 mm` (palm & forearm)** | `0.16 mm` |
| **Wall Generator** | **Arachne** (preserves thin `0.50 mm` snap-fit pinch lips cleanly) | Arachne |
| **Wall Loops / Perimeters** | **`4 – 5 Walls`** (`1.6–2.0 mm` shell for `35–105 N` tendon loads) | `3 Walls` |
| **Top / Bottom Solid Layers** | `5 Top / 5 Bottom` | `4 Top / 4 Bottom` |
| **Infill Density & Pattern** | **`40% – 50% Gyroid`** (isotropic shear strength at clevis forks) | `25% Gyroid` |
| **Outer Wall Speed** | `35 – 45 mm/s` (`60–80 mm/s` inner/infill) | `25 – 30 mm/s` |
| **Supports** | **Tree (Organic) — On Build Plate Only** (`Threshold angle: 35°`) | None |
| **Mandatory Support Blockers** | Block internal **`Ø2.20 mm` phalanx bores**, **`Ø2.80 mm` palm conduits**, and **dorsal `1.80 mm` snap-fit channels** | N/A |

> **Critical Slicer Rule — "On Build Plate Only" + Support Blockers**: Never allow "Everywhere" supports! In v9.0, the dorsal snap-fit undercut cavity (`2.6 mm → 1.8 mm`) uses **`45°` self-supporting chamfers** specifically designed to bridge cleanly in mid-air without internal supports when phalanges are printed dorsal-side up.

---

## 4. Phased Printing Plan & Optimal v9.0 Bed Orientations

Print the **16 physical parts** across **4 chronological phases** for rapid tolerance verification:

```text
Phase 1: Monolithic Palm & 6-Servo Forearm Rack (stl_exports_v9/)
├── palm_v9.stl (Qty: 1)
└── forearm_servo_adapter_v9.stl (Qty: 1)

Phase 2: Opposable Thumb Digits (stl_exports_v9/)
├── thumb_proximal_v9.stl (Qty: 1)
└── thumb_distal_v9.stl (Qty: 1)

Phase 3: Primary Pinch & Power Digits — Index & Middle (stl_exports_v9/)
├── index_proximal_v9.stl, index_intermediate_v9.stl, index_distal_v9.stl (Qty: 1 each)
└── middle_proximal_v9.stl, middle_intermediate_v9.stl, middle_distal_v9.stl (Qty: 1 each)

Phase 4: Ulnar Grasp Digits — Ring & Pinky (stl_exports_v9/)
├── ring_proximal_v9.stl, ring_intermediate_v9.stl, ring_distal_v9.stl (Qty: 1 each)
└── pinky_proximal_v9.stl, pinky_intermediate_v9.stl, pinky_distal_v9.stl (Qty: 1 each)
```

---

### Phase 1: Palm Chassis & Open-Access 6-Servo Forearm Rack
*Estimated Print Time (`0.16 mm` layers): ~6.5 – 8.5 hours*

| File (`stl_exports_v9/`) | Qty | Recommended Bed Orientation | Support & Blocker Strategy |
| :--- | :---: | :--- | :--- |
| `palm_v9.stl` | **1** | **Option A (Recommended)**: Stand upright on the flat `WRIST_INTERFACE_Y` XZ wrist bulkhead flange (`5 mm` outer brim).<br>**Option B**: Lay flat on dorsal back surface (palm up). | **Option A**: Vertical `Ø2.80 mm` carpal conduits print 100% support-free; add build-plate tree support under the radial Thumb CMC pedestal.<br>**Option B**: Tree supports on build plate only + cylindrical blockers over all `Ø2.80 mm` internal conduits. |
| `forearm_servo_adapter_v9.stl` | **1** | Stand **upright on the flat front wrist flange (`y = 0` face down)** on the build plate (`5 mm` outer brim). | Central `Ø22.0 mm` tendon passage prints vertically without supports; use **build-plate-only tree supports** for the horizontal side-bay servo tab ledges (`12.7 × 23.3 mm` side openings). |

---

### Phase 2: Opposable Thumb Phalanges
*Estimated Print Time (`0.12 mm` layers): ~1.5 hours*

| File (`stl_exports_v9/`) | Qty | Nominal Shaft (`L × W × H`) | Recommended Bed Orientation & Support Strategy |
| :--- | :---: | :---: | :--- |
| `thumb_proximal_v9.stl` | **1** | `33.0 × 14.5 × 13.0 mm` | **Horizontal, palmar side down / dorsal crown UP (`+Z`)**. Build-plate-only tree supports under palmar curve & distal tongue; `45°` snap-fit pinch lips print upward 100% support-free; block `Ø2.20 mm` bore. |
| `thumb_distal_v9.stl` | **1** | `27.0 × 13.5 × 12.0 mm` | **Horizontal, palmar side down / dorsal fingernail bed UP (`+Z`)**. Preserves razor-sharp `0.70 mm` nail bed & cuticle termination (`0.55L`) on top with zero support marks; block `Ø2.20 mm` bore & `Ø4.0 mm` knot pocket. |

---

### Phase 3: Primary Grasp Digits (Index & Middle)
*Estimated Print Time (`0.12 mm` layers): ~3.5 – 4.5 hours*

Print all 6 phalanges **horizontally with palmar side down and dorsal crown facing UP (`+Z`)** using **Arachne walls**, **build-plate-only tree supports**, and **support blockers** over internal `Ø2.20 mm` bores:

| File (`stl_exports_v9/`) | Qty | Nominal Shaft (`L × W × H`) | Key v9.0 Print Features to Inspect |
| :--- | :---: | :---: | :--- |
| `index_proximal_v9.stl` | **1** | `34.0 × 12.6 × 12.0 mm` | `1.8 mm` snap-fit mouth, 6-sided M3 hex socket (`+X`), `Ø3.4 mm` pin bore |
| `index_intermediate_v9.stl` | **1** | `25.5 × 12.2 × 11.6 mm` | `0°` dorsal hard-stop ceiling, `z = -3.50 mm` tendon bore below pin |
| `index_distal_v9.stl` | **1** | `24.0 × 12.0 × 11.2 mm` | Lips stop cleanly at `0.55L` before `0.70 mm` nail bed (`0.58L`), `Ø4.0 mm` knot pocket |
| `middle_proximal_v9.stl` | **1** | `38.0 × 13.4 × 12.0 mm` | `1.8 mm` snap-fit mouth, 6-sided M3 hex socket (`+X`), `Ø3.4 mm` pin bore |
| `middle_intermediate_v9.stl` | **1** | `28.0 × 12.8 × 11.6 mm` | `0°` dorsal hard-stop ceiling, `z = -3.50 mm` tendon bore below pin |
| `middle_distal_v9.stl` | **1** | `25.5 × 12.6 × 11.2 mm` | Clean cuticle lip termination, `Ø2.0 mm` nail-bed cross-bore, `Ø4.0 mm` knot pocket |

---

### Phase 4: Ulnar Support Digits (Ring & Pinky)
*Estimated Print Time (`0.12 mm` layers): ~3.0 – 4.0 hours*

Print **horizontally with dorsal crown UP (`+Z`)** (same slicer profile as Phase 3):

| File (`stl_exports_v9/`) | Qty | Nominal Shaft (`L × W × H`) | Key v9.0 Print Features to Inspect |
| :--- | :---: | :---: | :--- |
| `ring_proximal_v9.stl` | **1** | `35.0 × 12.8 × 12.0 mm` | `1.8 mm` snap-fit mouth, 6-sided M3 hex socket (`+X`) |
| `ring_intermediate_v9.stl` | **1** | `26.0 × 12.4 × 11.6 mm` | `0°` dorsal hard-stop ceiling, `z = -3.50 mm` tendon bore |
| `ring_distal_v9.stl` | **1** | `23.5 × 12.0 × 11.2 mm` | `0.70 mm` nail bed recess, `Ø4.0 × 2.5 mm` palmar knot pocket |
| `pinky_proximal_v9.stl` | **1** | `30.0 × 11.8 × 12.0 mm` | Reinforced base condyles (`≥ 1.25 mm` clamping walls around M3 pockets) |
| `pinky_intermediate_v9.stl` | **1** | `22.0 × 11.4 × 11.6 mm` | Reinforced base condyles (`≥ 1.25 mm` clamping walls), `0°` stop |
| `pinky_distal_v9.stl` | **1** | `20.0 × 11.0 × 11.2 mm` | Reinforced base condyles (`≥ 1.25 mm` clamping walls), clean nail bed |

---

## 5. Master Parts Inventory (Iteration 9 — `stl_exports_v9/`)

| Category | STL File Name (`stl_exports_v9/`) | Nominal Dimensions (`L × W × H`) | Print Qty |
| :--- | :--- | :--- | :---: |
| **Palm Chassis** | `palm_v9.stl` | `86.0 × 76.0 × 22.0 mm` (plus wrist bulkhead & CMC pedestal) | **1** |
| **Forearm Rack** | `forearm_servo_adapter_v9.stl` | `82.0 × 62.0 × 34.0 mm` (6× MG90S rack + `46×30 mm` flange) | **1** |
| **Thumb** | `thumb_proximal_v9.stl` | `33.0 × 14.5 × 13.0 mm` | **1** |
| **Thumb** | `thumb_distal_v9.stl` | `27.0 × 13.5 × 12.0 mm` | **1** |
| **Index** | `index_proximal_v9.stl` | `34.0 × 12.6 × 12.0 mm` | **1** |
| **Index** | `index_intermediate_v9.stl` | `25.5 × 12.2 × 11.6 mm` | **1** |
| **Index** | `index_distal_v9.stl` | `24.0 × 12.0 × 11.2 mm` | **1** |
| **Middle** | `middle_proximal_v9.stl` | `38.0 × 13.4 × 12.0 mm` | **1** |
| **Middle** | `middle_intermediate_v9.stl` | `28.0 × 12.8 × 11.6 mm` | **1** |
| **Middle** | `middle_distal_v9.stl` | `25.5 × 12.6 × 11.2 mm` | **1** |
| **Ring** | `ring_proximal_v9.stl` | `35.0 × 12.8 × 12.0 mm` | **1** |
| **Ring** | `ring_intermediate_v9.stl` | `26.0 × 12.4 × 11.6 mm` | **1** |
| **Ring** | `ring_distal_v9.stl` | `23.5 × 12.0 × 11.2 mm` | **1** |
| **Pinky** | `pinky_proximal_v9.stl` | `30.0 × 11.8 × 12.0 mm` | **1** |
| **Pinky** | `pinky_intermediate_v9.stl` | `22.0 × 11.4 × 11.6 mm` | **1** |
| **Pinky** | `pinky_distal_v9.stl` | `20.0 × 11.0 × 11.2 mm` | **1** |
| *Reference Only* | `robotic_hand_full_assembly_v9.stl` | Complete articulated CAD check *(do not slice/print)* | — |

**Total physical parts to print**: **16 components**.

---

## 6. Post-Print Calibration & Hardware Fit-Check (v9.0)

1. **First-Article Tolerance Test (`index_distal_v9.stl` + `index_intermediate_v9.stl`)**:
   - Before printing the full hand, print these two parts (`~35 minutes`) and verify:
     - **M3 Hinge Pin (`Ø3.40 mm` bore)**: An M3 bolt slides through smoothly; ream lightly by hand with a `3.2 mm` bit only if Z-seam elephants-foot is present.
     - **Captive 6-Sided Hex Nut (`5.65 mm` AF)**: A standard M3 hex nut (`5.5 mm` flats) presses snugly into the `+X` hexagonal socket and cannot spin when tightened.
     - **Snap-Fit Pinch Lips (`1.80 mm` mouth)**: Press a `1.5–2.0 mm` wide rubber band straight down into the dorsal slit—it should pop cleanly past the `0.50 mm` lips into the `2.60 mm` chamber and stay captive through `0° → 95°` DIP bending.
     - **Tendon Clearance (`Ø2.20 mm` bore at `z = -3.50 mm`)**: With the M3 hinge screw installed, thread `Ø0.8–1.0 mm` Dyneema straight through the palmar bore—confirm zero contact with the metal M3 shaft and flush knot seating inside the `Ø4.0 × 2.5 mm` tip pocket.
2. **6-Servo Forearm Fit-Check (`forearm_servo_adapter_v9.stl`)**:
   - Test-slide an MG90S (`12.2 × 22.8 mm` body) into each of the 6 side-loading bays (`12.7 × 23.3 mm`) and verify alignment of the `27.5 mm` mounting tab centers over the `Ø1.8 mm` M2 pilot holes before bolting the Y-axis wrist bulkhead to `palm_v9.stl`.
3. **Full Rigging & Calibration**: Follow Section 5 of [ENGINEERING_GUIDE.md](ENGINEERING_GUIDE.md) for dorsal cross-pin tie-offs and `Ø10–12 mm` servo spool tensioning.
