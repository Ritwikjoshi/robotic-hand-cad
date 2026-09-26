"""
Procedural 3D Mesh Generator for 5-Finger Anthropomorphic Robotic Hand
========================================================================
Iteration 2 (v2 - Calibrated Physical Printing Scale):
- User Feedback Integration:
  1. Width of finger calibrated: Target width = 12.0 mm (distal/intermediate) to 13.5 mm (proximal).
  2. Pin hole enlarged from radius 1.65 mm (3.3 mm hole) to radius 1.75 mm (3.5 mm hole)
     so a standard 3.0 mm dowel pin / M3 screw fits smoothly with zero binding after FDM shrinkage.
  3. Tendon bore enlarged from radius 0.75-0.9 mm (1.5-1.8 mm dia) to radius 1.25 mm (2.5 mm dia)
     and 1.5 mm (3.0 mm dia) for easy threading of Dyneema cables without snagging.
  4. Hinge clevis gaps increased from 4.8 mm to 5.4 mm with 4.5 mm tongue for silky smooth rotation.
  5. Kept previous STL files intact in `stl_exports/` and saved all updated files to `stl_exports_v2/`.
"""

import os
import numpy as np
import trimesh
from trimesh.creation import box, cylinder, icosphere

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
STL_DIR_V2 = os.path.join(OUTPUT_DIR, "stl_exports_v2")
os.makedirs(STL_DIR_V2, exist_ok=True)

# THUMB CMC EULER ANGLES (XYZ intrinsic order)
THUMB_CMC_X = 0.85    # +49deg palmar abduction
THUMB_CMC_Y = -0.35   # -20deg pronation
THUMB_CMC_Z = 0.50    # +29deg radial abduction

# Base palm dimensions
PALM_W = 76.0
PALM_L = 86.0
PALM_H = 22.0

# Calibrated hole and clevis tolerances for FDM printing (Kobra 2 Neo)
PIN_RADIUS = 1.75        # 3.5 mm diameter hole (clears 3.0 mm pin / M3 bolt)
TENDON_RADIUS = 1.25     # 2.5 mm diameter internal cable bore (effortless threading)
CABLE_PORT_RADIUS = 1.50 # 3.0 mm entrance/exit funnel ports
CLEVIS_SLOT_W = 5.4      # Female clevis pocket width
CLEVIS_TONGUE_W = 4.4    # Male clevis tongue width (0.5 mm clearance each side)


def generate_distal_phalanx(length=25.0, width=12.0, height=10.5):
    """
    Fingertip phalanx:
    - Width calibrated to 12.0 mm
    - Enlarged tendon bore (dia 2.5 mm) & anchor pocket
    - Enlarged pin hole (dia 3.5 mm)
    """
    # Male hinge base cylinder that fits into intermediate female clevis
    k_base = cylinder(radius=height * 0.48, height=CLEVIS_TONGUE_W, sections=32)
    k_base.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    shaft = cylinder(radius=height * 0.46, height=length * 0.70, sections=32)
    shaft.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    shaft.apply_translation([0, length * 0.35, -height * 0.04])

    pulp = icosphere(subdivisions=3, radius=1.0)
    pulp.apply_scale([width * 0.46, length * 0.34, height * 0.50])
    pulp.apply_translation([0, length * 0.72, -height * 0.16])

    apex = icosphere(subdivisions=3, radius=1.0)
    apex.apply_scale([width * 0.42, height * 0.40, height * 0.40])
    apex.apply_translation([0, length - 1.5, 0])

    body = trimesh.boolean.union([k_base, shaft, pulp, apex])

    # Cutters
    pin_cutter = cylinder(radius=PIN_RADIUS, height=width + 6.0, sections=32)
    pin_cutter.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    # Generous tendon slot & anchor knot pocket
    tendon_slot = box(extents=[width * 0.45, 5.0, height * 0.65])
    tendon_slot.apply_translation([0, length * 0.55, -height * 0.15])

    anchor_pocket = box(extents=[6.0, 5.5, 7.0])
    anchor_pocket.apply_translation([0, length * 0.75, -height * 0.08])

    tendon_bore = cylinder(radius=TENDON_RADIUS, height=length + 6.0, sections=20)
    tendon_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    tendon_bore.apply_translation([0, length / 2, -height * 0.22])

    cutters = trimesh.boolean.union([pin_cutter, tendon_slot, anchor_pocket, tendon_bore])
    return body.difference(cutters)


def generate_intermediate_phalanx(length=28.0, width=12.5, height=11.0):
    """
    Intermediate phalanx:
    - Width calibrated to 12.5 mm
    - Proximal female clevis (slot 5.4 mm)
    - Distal female clevis (slot 5.4 mm)
    - Enlarged flexor & extensor tendon bores (dia 2.5 mm)
    """
    k_prox = cylinder(radius=height * 0.48, height=width, sections=32)
    k_prox.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    shaft = cylinder(radius=height * 0.46, height=length * 0.85, sections=32)
    shaft.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    shaft.apply_translation([0, length * 0.48, -height * 0.06])

    cushion = icosphere(subdivisions=3, radius=1.0)
    cushion.apply_scale([width * 0.40, length * 0.36, height * 0.44])
    cushion.apply_translation([0, length * 0.50, -height * 0.30])

    k_dist = cylinder(radius=height * 0.46, height=width * 0.92, sections=32)
    k_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    k_dist.apply_translation([0, length, 0])

    body = trimesh.boolean.union([k_prox, shaft, cushion, k_dist])

    # Proximal clevis slot (receives proximal tongue)
    clevis_cut = box(extents=[CLEVIS_SLOT_W, height * 1.5, height * 1.5])

    # Distal clevis cut (receives distal phalanx tongue)
    dist_clevis_cut = box(extents=[CLEVIS_SLOT_W, height * 1.5, height * 1.5])
    dist_clevis_cut.apply_translation([0, length, 0])

    # Hinge Pin Bores
    pin_prox = cylinder(radius=PIN_RADIUS, height=width + 6.0, sections=32)
    pin_prox.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    pin_dist = cylinder(radius=PIN_RADIUS, height=width + 6.0, sections=32)
    pin_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    pin_dist.apply_translation([0, length, 0])

    # Enlarged Tendon Bores
    t_flex = cylinder(radius=TENDON_RADIUS, height=length + 6.0, sections=20)
    t_flex.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_flex.apply_translation([0, length / 2, -height * 0.28])

    t_ext = cylinder(radius=TENDON_RADIUS, height=length + 6.0, sections=20)
    t_ext.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_ext.apply_translation([0, length / 2, height * 0.28])

    # Chamfered cable guide eyelets at ends
    guide_prox = cylinder(radius=CABLE_PORT_RADIUS, height=width * 0.65, sections=18)
    guide_prox.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    guide_prox.apply_translation([0, 2.5, -height * 0.28])

    guide_dist = cylinder(radius=CABLE_PORT_RADIUS, height=width * 0.65, sections=18)
    guide_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    guide_dist.apply_translation([0, length - 2.5, -height * 0.28])

    cutters = trimesh.boolean.union([
        clevis_cut, dist_clevis_cut, pin_prox, pin_dist,
        t_flex, t_ext, guide_prox, guide_dist
    ])
    return body.difference(cutters)


def generate_proximal_phalanx(length=38.0, width=13.5, height=12.0):
    """
    Proximal phalanx:
    - Width calibrated to 13.5 mm (smooth anatomical taper to 12.5 mm intermediate)
    - Proximal female clevis to knuckle (slot 5.4 mm)
    - Distal male tongue (width 4.4 mm)
    - Enlarged tendon bores (dia 2.5 mm) & cable eyelets
    """
    k_base = cylinder(radius=height * 0.48, height=width, sections=32)
    k_base.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    shaft = cylinder(radius=height * 0.46, height=length * 0.88, sections=32)
    shaft.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    shaft.apply_translation([0, length * 0.48, -height * 0.06])

    cushion = icosphere(subdivisions=3, radius=1.0)
    cushion.apply_scale([width * 0.42, length * 0.38, height * 0.45])
    cushion.apply_translation([0, length * 0.50, -height * 0.32])

    tongue = cylinder(radius=height * 0.44, height=CLEVIS_TONGUE_W, sections=32)
    tongue.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    tongue.apply_translation([0, length, 0])

    body = trimesh.boolean.union([k_base, shaft, cushion, tongue])

    # Proximal base clevis slot (mounts to palm knuckle)
    base_clevis = box(extents=[CLEVIS_SLOT_W, height * 1.5, height * 1.5])

    # Hinge Pin Bores
    pin_base = cylinder(radius=PIN_RADIUS, height=width + 6.0, sections=32)
    pin_base.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    pin_dist = cylinder(radius=PIN_RADIUS, height=width + 6.0, sections=32)
    pin_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    pin_dist.apply_translation([0, length, 0])

    # Enlarged Tendon Bores
    t_flex = cylinder(radius=TENDON_RADIUS, height=length + 6.0, sections=20)
    t_flex.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_flex.apply_translation([0, length / 2, -height * 0.28])

    t_ext = cylinder(radius=TENDON_RADIUS, height=length + 6.0, sections=20)
    t_ext.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_ext.apply_translation([0, length / 2, height * 0.28])

    cable_eyelet = cylinder(radius=CABLE_PORT_RADIUS, height=width * 0.65, sections=18)
    cable_eyelet.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    cable_eyelet.apply_translation([0, 3.5, -height * 0.28])

    cutters = trimesh.boolean.union([base_clevis, pin_base, pin_dist, t_flex, t_ext, cable_eyelet])
    return body.difference(cutters)


def generate_forearm_adapter(adapter_length=65.0, outer_radius=23.0):
    """Forearm servo adapter for 6x micro servos with enlarged tendon pass-throughs."""
    shell = cylinder(radius=outer_radius, height=adapter_length, sections=48)
    shell.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    shell.apply_translation([0, -adapter_length / 2, 0])

    cap = icosphere(subdivisions=3, radius=outer_radius * 0.98)
    cap_scale = np.eye(4)
    cap_scale[0, 0] = 1.0
    cap_scale[1, 1] = 0.35
    cap_scale[2, 2] = 1.0
    cap.apply_transform(cap_scale)
    cap.apply_translation([0, -adapter_length, 0])

    body = trimesh.boolean.union([shell, cap])
    cutters = []

    servo_positions = [
        (-8.0, -14.0), (-8.0, -34.0), (-8.0, -54.0),
        ( 8.0, -14.0), ( 8.0, -34.0), ( 8.0, -54.0)
    ]

    for sx, sy in servo_positions:
        pocket = box(extents=[12.8, 23.8, 28.0])
        pocket.apply_translation([sx, sy, 0])
        cutters.append(pocket)

        t_ch = cylinder(radius=1.8, height=adapter_length + 10.0, sections=16)
        t_ch.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
        t_ch.apply_translation([sx, -adapter_length / 2, -10.0])
        cutters.append(t_ch)

    # 4x M3 mount bolt holes
    for ang in [np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4]:
        bh = cylinder(radius=1.75, height=adapter_length * 2, sections=16)
        bh.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
        bh.apply_translation([18.0 * np.cos(ang), -adapter_length / 2, 18.0 * np.sin(ang)])
        cutters.append(bh)

    center_bore = cylinder(radius=7.5, height=adapter_length * 2, sections=24)
    center_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    center_bore.apply_translation([0, -adapter_length / 2, 0])
    cutters.append(center_bore)

    return body.difference(trimesh.boolean.union(cutters))


def generate_palm(palm_w=PALM_W, palm_l=PALM_L, palm_h=PALM_H):
    """Anatomical palm with calibrated clevises, enlarged cable tunnels, and thumb socket."""
    wrist_base = icosphere(subdivisions=3, radius=1.0)
    wrist_base.apply_scale([palm_w * 0.38, 14.0, palm_h * 0.44])
    wrist_base.apply_translation([0, 10.0, 0])

    thenar = icosphere(subdivisions=3, radius=1.0)
    thenar.apply_scale([18.0, 22.0, 14.0])
    rot_thenar = trimesh.transformations.euler_matrix(0.40, -0.30, 0.45)
    thenar.apply_transform(rot_thenar)
    thenar.apply_translation([-palm_w * 0.36, palm_l * 0.28, -2.0])

    hypo = icosphere(subdivisions=3, radius=1.0)
    hypo.apply_scale([13.5, 26.0, 10.5])
    hypo.apply_translation([palm_w * 0.36, palm_l * 0.40, -1.8])

    finger_x = [-23.0, -8.0, 8.0, 23.0]
    knuckle_y = [palm_l - 2.5, palm_l, palm_l - 1.2, palm_l - 3.8]
    knuckle_z = [0.4, 0.8, 0.3, -0.4]

    knuckle_spheres = []
    for fx, fy, fz in zip(finger_x, knuckle_y, knuckle_z):
        ks = icosphere(subdivisions=3, radius=1.0)
        ks.apply_scale([9.0, 12.0, palm_h * 0.46])
        ks.apply_translation([fx, fy - 6.0, fz])
        knuckle_spheres.append(ks)

    distal_ridge = cylinder(radius=palm_h * 0.36, height=palm_w * 0.78, sections=32)
    distal_ridge.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    distal_ridge.apply_translation([0, palm_l - 16.0, -palm_h * 0.16])

    dorsal_core = icosphere(subdivisions=3, radius=1.0)
    dorsal_core.apply_scale([palm_w * 0.44, palm_l * 0.42, palm_h * 0.36])
    dorsal_core.apply_translation([0, palm_l * 0.48, palm_h * 0.12])

    all_palm_parts = [wrist_base, thenar, hypo, distal_ridge, dorsal_core] + knuckle_spheres
    palm_hull = trimesh.boolean.union(all_palm_parts).convex_hull

    cutters = []

    # 4 Knuckle clevis slots & pin holes
    for fx, fy, fz in zip(finger_x, knuckle_y, knuckle_z):
        # Male tongue on proximal phalanx is 4.4 mm wide -> slot is 5.4 mm
        c_slot = box(extents=[CLEVIS_SLOT_W, 17.0, palm_h + 6.0])
        c_slot.apply_translation([fx, fy - 1.0, fz])
        cutters.append(c_slot)

        # Pin hole 3.5 mm dia
        p_hole = cylinder(radius=PIN_RADIUS, height=20.0, sections=32)
        p_hole.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
        p_hole.apply_translation([fx, fy - 4.0, fz])
        cutters.append(p_hole)

        # Enlarged tendon conduit (dia 3.0 mm)
        t_tun = cylinder(radius=1.5, height=36.0, sections=20)
        t_tun.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
        t_tun.apply_translation([fx, fy - 14.0, fz - 3.5])
        cutters.append(t_tun)

        eyelet = cylinder(radius=1.5, height=9.0, sections=20)
        eyelet.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
        eyelet.apply_translation([fx, fy - 8.0, fz - 3.0])
        cutters.append(eyelet)

    # Corrected Thumb CMC Joint Socket
    th_pos = [-palm_w * 0.36 - 2.5, palm_l * 0.28, -2.0]
    rot_thumb_cmc = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)

    th_slot = box(extents=[CLEVIS_SLOT_W + 0.4, 20.0, 24.0])
    th_slot.apply_transform(rot_thumb_cmc)
    th_slot.apply_translation(th_pos)
    cutters.append(th_slot)

    th_pin = cylinder(radius=PIN_RADIUS, height=26.0, sections=32)
    th_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    th_pin.apply_transform(rot_thumb_cmc)
    th_pin.apply_translation(th_pos)
    cutters.append(th_pin)

    th_tendon = cylinder(radius=1.5, height=34.0, sections=20)
    th_tendon.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    th_tendon.apply_transform(rot_thumb_cmc)
    th_tendon.apply_translation([th_pos[0] + 5, th_pos[1] - 8, th_pos[2]])
    cutters.append(th_tendon)

    th_opp_bore = cylinder(radius=1.5, height=30.0, sections=20)
    th_opp_bore.apply_transform(trimesh.transformations.rotation_matrix(0.4, [0, 0, 1]))
    th_opp_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    th_opp_bore.apply_translation([th_pos[0] + 8, th_pos[1] - 12, th_pos[2] + 2])
    cutters.append(th_opp_bore)

    cavity = box(extents=[palm_w * 0.56, palm_l * 0.48, palm_h * 0.65])
    cavity.apply_translation([0, palm_l * 0.42, 0])
    cutters.append(cavity)

    palm_cup = icosphere(subdivisions=3, radius=1.0)
    palm_cup.apply_scale([palm_w * 0.22, palm_l * 0.22, 6.0])
    palm_cup.apply_translation([0, palm_l * 0.45, -palm_h * 0.50])
    cutters.append(palm_cup)

    wrist_bore = cylinder(radius=14.0, height=palm_h + 12.0, sections=36)
    wrist_bore.apply_translation([0, 4.0, 0])
    cutters.append(wrist_bore)

    for ang in [np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4]:
        bolt = cylinder(radius=1.75, height=palm_h + 12.0, sections=18)
        bx = 18.0 * np.cos(ang)
        by = 4.0 + 18.0 * np.sin(ang)
        bolt.apply_translation([bx, by, 0])
        cutters.append(bolt)

    cutter_all = trimesh.boolean.union(cutters)
    return palm_hull.difference(cutter_all)


def build_iteration_2():
    print("=" * 65)
    print("BUILDING ITERATION 2 (v2) - CALIBRATED PRINTING SCALE")
    print("  Finger width target : 12.0 mm (distal/intermediate), 13.5 mm (proximal)")
    print("  Pin hole diameter   : 3.5 mm (smooth clearance for 3.0 mm pins)")
    print("  Tendon bore diameter: 2.5 - 3.0 mm (wide, frictionless cable threading)")
    print("  Output Directory    : " + STL_DIR_V2)
    print("=" * 65)

    print("\n[1/5] Generating Calibrated Palm (v2)...")
    palm = generate_palm()
    palm.export(os.path.join(STL_DIR_V2, "palm_v2.stl"))

    print("[2/5] Generating Forearm Servo Adapter (v2)...")
    adapter = generate_forearm_adapter()
    adapter.export(os.path.join(STL_DIR_V2, "forearm_servo_adapter_v2.stl"))

    components = [palm]

    # Digit configurations
    # Index finger calibrated specifically to 12.0 mm width
    # Scales relative to index baseline
    digit_configs = [
        {"name": "index",  "w_prox": 13.5, "w_mid": 12.0, "w_dist": 12.0, "len_p": 36.0, "len_i": 26.0, "len_d": 24.0, "x": -23.0, "y": PALM_L - 2.5, "z": 0.4, "rotZ": 0.07, "fm": 0.32, "fp": 0.42, "fd": 0.25},
        {"name": "middle", "w_prox": 14.5, "w_mid": 13.0, "w_dist": 12.8, "len_p": 40.0, "len_i": 29.0, "len_d": 25.5, "x":  -8.0, "y": PALM_L,       "z": 0.8, "rotZ": 0.02, "fm": 0.28, "fp": 0.38, "fd": 0.22},
        {"name": "ring",   "w_prox": 13.5, "w_mid": 12.2, "w_dist": 12.0, "len_p": 37.0, "len_i": 26.5, "len_d": 23.5, "x":   8.0, "y": PALM_L - 1.2, "z": 0.3, "rotZ": -0.04, "fm": 0.30, "fp": 0.40, "fd": 0.25},
        {"name": "pinky",  "w_prox": 12.5, "w_mid": 11.2, "w_dist": 11.0, "len_p": 31.0, "len_i": 22.5, "len_d": 20.0, "x":  23.0, "y": PALM_L - 3.8, "z": -0.4, "rotZ": -0.10, "fm": 0.34, "fp": 0.44, "fd": 0.28}
    ]

    print("[3/5] Generating & Exporting All 4 Finger Digits (v2)...")
    for d in digit_configs:
        dname = d["name"]
        p = generate_proximal_phalanx(length=d["len_p"], width=d["w_prox"], height=12.0)
        ip = generate_intermediate_phalanx(length=d["len_i"], width=d["w_mid"], height=11.0)
        dp = generate_distal_phalanx(length=d["len_d"], width=d["w_dist"], height=10.5)

        # Export individual cleanly named v2 parts
        p.export(os.path.join(STL_DIR_V2, f"{dname}_proximal_v2.stl"))
        ip.export(os.path.join(STL_DIR_V2, f"{dname}_intermediate_v2.stl"))
        dp.export(os.path.join(STL_DIR_V2, f"{dname}_distal_v2.stl"))

        # Assembly copy
        p_c = p.copy()
        ip_c = ip.copy()
        dp_c = dp.copy()

        dp_c.apply_transform(trimesh.transformations.rotation_matrix(d["fd"], [1, 0, 0]))
        dp_c.apply_translation([0, d["len_i"], 0])

        tip = trimesh.util.concatenate([ip_c, dp_c])
        tip.apply_transform(trimesh.transformations.rotation_matrix(d["fp"], [1, 0, 0]))
        tip.apply_translation([0, d["len_p"], 0])

        f_full = trimesh.util.concatenate([p_c, tip])
        f_full.apply_transform(trimesh.transformations.rotation_matrix(d["fm"], [1, 0, 0]))
        f_full.apply_transform(trimesh.transformations.rotation_matrix(d["rotZ"], [0, 0, 1]))
        f_full.apply_translation([d["x"], d["y"] - 4.0, d["z"]])
        components.append(f_full)

    print("[4/5] Generating & Exporting Opposable Thumb (v2)...")
    th_p = generate_proximal_phalanx(length=33.0, width=14.5, height=13.0)
    th_d = generate_distal_phalanx(length=27.0, width=13.5, height=12.0)

    th_p.export(os.path.join(STL_DIR_V2, "thumb_proximal_v2.stl"))
    th_d.export(os.path.join(STL_DIR_V2, "thumb_distal_v2.stl"))

    th_p_c = th_p.copy()
    th_d_c = th_d.copy()
    th_d_c.apply_transform(trimesh.transformations.rotation_matrix(0.35, [1, 0, 0]))
    th_d_c.apply_translation([0, 33.0, 0])

    th_full = trimesh.util.concatenate([th_p_c, th_d_c])
    th_full.apply_transform(trimesh.transformations.rotation_matrix(0.30, [1, 0, 0]))

    rot_cmc = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)
    th_full.apply_transform(rot_cmc)
    th_full.apply_translation([-PALM_W * 0.36 - 2.5, PALM_L * 0.28, -2.0])
    components.append(th_full)

    adapter_c = adapter.copy()
    adapter_c.apply_translation([0, -2.0, 0])
    components.append(adapter_c)

    print("[5/5] Merging Full Assembly (v2)...")
    full_assembly = trimesh.util.concatenate(components)
    full_assembly.export(os.path.join(STL_DIR_V2, "robotic_hand_full_assembly_v2.stl"))

    print("\nSUCCESS: All Iteration 2 (v2) STLs exported to:", STL_DIR_V2)


if __name__ == "__main__":
    build_iteration_2()
