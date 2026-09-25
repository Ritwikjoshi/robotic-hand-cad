"""
Procedural 3D Mesh Generator for 5-Finger Anthropomorphic Robotic Hand
========================================================================
Revision 3 — Biomechanical Expert Corrections:

THUMB DIRECTION FIX (Root-Cause Analysis):
  Previous Euler angles (0.55, 0.45, -0.75) caused the thumb to:
    1. Project dorsally instead of palmarly (X rotation too small)
    2. Pronate the pad AWAY from fingers (Y rotation wrong sign)
    3. Swing toward the ulnar side instead of radially (Z rotation wrong sign)

  Corrected angles (0.85, -0.35, 0.50):
    X = +0.85 rad (+49deg) -> Palmar abduction: projects thumb forward out of palm (+Z)
    Y = -0.35 rad (-20deg) -> Pronation: thumb pad faces medially toward finger pads
    Z = +0.50 rad (+29deg) -> Radial abduction: natural resting clearance from index

SERVO INTEGRATION:
  - Forearm adapter block (60mm cylinder) with 6x SG90/MG90S servo pockets
  - Tendon routing bores through palm interior into each finger channel
  - Cable anchor pockets in each distal phalanx

COORDINATE SYSTEM (Right hand, palmar view):
  +X = ulnar (toward pinky)     -X = radial (toward thumb)
  +Y = distal (toward fingertips)  -Y = proximal (toward wrist)
  +Z = palmar (toward viewer)   -Z = dorsal (back of hand)
"""

import os
import numpy as np
import trimesh
from trimesh.creation import box, cylinder, icosphere

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(OUTPUT_DIR, "stl_exports")
os.makedirs(STL_DIR, exist_ok=True)

# CORRECTED THUMB CMC EULER ANGLES (XYZ intrinsic order)
THUMB_CMC_X = 0.85    # +49deg palmar abduction (out of palm toward +Z)
THUMB_CMC_Y = -0.35   # -20deg pronation (pad faces medially toward +X)
THUMB_CMC_Z = 0.50    # +29deg radial abduction (natural clearance toward -X)

# Palm dimensions
PALM_W = 74.0
PALM_L = 84.0
PALM_H = 21.0

# Servo dimensions (SG90 micro servo)
SERVO_W = 12.5
SERVO_D = 23.5
SERVO_H = 30.0
SERVO_EAR_H = 2.5


def generate_distal_phalanx(length=24.0, width=12.0, height=10.0):
    """Fingertip phalanx with rounded pulp pad, tendon anchor pocket, and cable bore."""
    k_base = cylinder(radius=height * 0.46, height=width - 3.4, sections=32)
    k_base.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    shaft = cylinder(radius=height * 0.45, height=length * 0.7, sections=32)
    shaft.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    shaft.apply_translation([0, length * 0.35, -height * 0.05])

    pulp = icosphere(subdivisions=3, radius=1.0)
    pulp.apply_scale([width * 0.42, length * 0.32, height * 0.48])
    pulp.apply_translation([0, length * 0.72, -height * 0.16])

    apex = icosphere(subdivisions=3, radius=1.0)
    apex.apply_scale([width * 0.38, height * 0.38, height * 0.38])
    apex.apply_translation([0, length - 1.5, 0])

    body = trimesh.boolean.union([k_base, shaft, pulp, apex])

    pin_cutter = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_cutter.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    tendon_slot = box(extents=[width * 0.4, 4.5, height * 0.6])
    tendon_slot.apply_translation([0, length * 0.55, -height * 0.15])

    anchor_pocket = box(extents=[5.0, 4.0, 6.0])
    anchor_pocket.apply_translation([0, length * 0.75, -height * 0.10])

    tendon_bore = cylinder(radius=0.75, height=length + 6.0, sections=16)
    tendon_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    tendon_bore.apply_translation([0, length / 2, -height * 0.22])

    cutters = trimesh.boolean.union([pin_cutter, tendon_slot, anchor_pocket, tendon_bore])
    return body.difference(cutters)


def generate_intermediate_phalanx(length=28.0, width=13.0, height=11.0):
    """Intermediate segment with dual clevis joints and tendon routing channels."""
    clevis_w = 4.2

    k_prox = cylinder(radius=height * 0.48, height=width, sections=32)
    k_prox.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    shaft = cylinder(radius=height * 0.46, height=length * 0.85, sections=32)
    shaft.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    shaft.apply_translation([0, length * 0.48, -height * 0.06])

    cushion = icosphere(subdivisions=3, radius=1.0)
    cushion.apply_scale([width * 0.38, length * 0.36, height * 0.42])
    cushion.apply_translation([0, length * 0.50, -height * 0.32])

    k_dist = cylinder(radius=height * 0.44, height=width * 0.86, sections=32)
    k_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    k_dist.apply_translation([0, length, 0])

    body = trimesh.boolean.union([k_prox, shaft, cushion, k_dist])

    clevis_cut = box(extents=[clevis_w + 0.8, height * 1.4, height * 1.4])
    dist_clevis_cut = box(extents=[width - 3.0, height * 1.3, height * 1.3])
    dist_clevis_cut.apply_translation([0, length, 0])

    pin_prox = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_prox.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    pin_dist = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    pin_dist.apply_translation([0, length, 0])

    t_flex = cylinder(radius=0.9, height=length + 6.0, sections=16)
    t_flex.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_flex.apply_translation([0, length / 2, -height * 0.28])

    t_ext = cylinder(radius=0.9, height=length + 6.0, sections=16)
    t_ext.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_ext.apply_translation([0, length / 2, height * 0.28])

    guide_prox = cylinder(radius=1.25, height=width * 0.5, sections=16)
    guide_prox.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    guide_prox.apply_translation([0, 2.0, -height * 0.28])

    guide_dist = cylinder(radius=1.25, height=width * 0.5, sections=16)
    guide_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    guide_dist.apply_translation([0, length - 2.0, -height * 0.28])

    cutters = trimesh.boolean.union([
        clevis_cut, dist_clevis_cut, pin_prox, pin_dist,
        t_flex, t_ext, guide_prox, guide_dist
    ])
    return body.difference(cutters)


def generate_proximal_phalanx(length=38.0, width=15.0, height=13.0):
    """Proximal phalanx with anatomical curvature, cable guide eyelet, and dual tendon bores."""
    clevis_w = 4.0

    k_base = cylinder(radius=height * 0.48, height=width, sections=32)
    k_base.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    shaft = cylinder(radius=height * 0.46, height=length * 0.88, sections=32)
    shaft.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    shaft.apply_translation([0, length * 0.48, -height * 0.08])

    cushion = icosphere(subdivisions=3, radius=1.0)
    cushion.apply_scale([width * 0.40, length * 0.38, height * 0.45])
    cushion.apply_translation([0, length * 0.50, -height * 0.34])

    tongue = cylinder(radius=height * 0.42, height=clevis_w, sections=32)
    tongue.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    tongue.apply_translation([0, length, 0])

    body = trimesh.boolean.union([k_base, shaft, cushion, tongue])

    base_clevis = box(extents=[5.2, height * 1.5, height * 1.5])

    pin_base = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_base.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))

    pin_dist = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    pin_dist.apply_translation([0, length, 0])

    t_flex = cylinder(radius=0.9, height=length + 6.0, sections=16)
    t_flex.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_flex.apply_translation([0, length / 2, -height * 0.3])

    t_ext = cylinder(radius=0.9, height=length + 6.0, sections=16)
    t_ext.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_ext.apply_translation([0, length / 2, height * 0.3])

    cable_eyelet = cylinder(radius=1.25, height=width * 0.6, sections=16)
    cable_eyelet.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    cable_eyelet.apply_translation([0, 3.0, -height * 0.30])

    cutters = trimesh.boolean.union([base_clevis, pin_base, pin_dist, t_flex, t_ext, cable_eyelet])
    return body.difference(cutters)


def generate_forearm_adapter(adapter_length=65.0, outer_radius=22.0):
    """
    Cylindrical forearm adapter block housing 6 servo pockets (2x3 grid).
    Mounts to the palm wrist flange via 4x M3 bolt pattern.
    """
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
        (-8.0, -12.0), (-8.0, -32.0), (-8.0, -52.0),
        ( 8.0, -12.0), ( 8.0, -32.0), ( 8.0, -52.0),
    ]

    for sx, sy in servo_positions:
        pocket = box(extents=[SERVO_W + 1.0, SERVO_D + 1.0, SERVO_H + 1.0])
        pocket.apply_translation([sx, sy, 0])
        cutters.append(pocket)

        for ear_offset in [-SERVO_D/2 - 2.0, SERVO_D/2 + 2.0]:
            screw = cylinder(radius=1.1, height=outer_radius * 2, sections=16)
            screw.apply_translation([sx, sy + ear_offset, 0])
            cutters.append(screw)

        cable_exit = cylinder(radius=1.0, height=20.0, sections=16)
        cable_exit.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
        cable_exit.apply_translation([sx, -2.0, 0])
        cutters.append(cable_exit)

    central_bore = cylinder(radius=8.0, height=adapter_length + 10.0, sections=32)
    central_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    central_bore.apply_translation([0, -adapter_length / 2, 0])
    cutters.append(central_bore)

    for ang in [np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4]:
        bolt = cylinder(radius=1.65, height=18.0, sections=16)
        bx = 18.0 * np.cos(ang)
        bz = 18.0 * np.sin(ang)
        bolt.apply_translation([bx, -3.0, bz])
        cutters.append(bolt)

    wire_pass = cylinder(radius=3.0, height=outer_radius * 3, sections=24)
    wire_pass.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    wire_pass.apply_translation([0, -adapter_length * 0.75, 0])
    cutters.append(wire_pass)

    cutter_all = trimesh.boolean.union(cutters)
    return body.difference(cutter_all)


def generate_palm(palm_w=PALM_W, palm_l=PALM_L, palm_h=PALM_H):
    """Anatomical palm with corrected thumb CMC orientation and cable routing."""
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
        ks.apply_scale([8.5, 11.0, palm_h * 0.44])
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

    for fx, fy, fz in zip(finger_x, knuckle_y, knuckle_z):
        c_slot = box(extents=[4.8, 16.0, palm_h + 4.0])
        c_slot.apply_translation([fx, fy - 1.0, fz])
        cutters.append(c_slot)

        p_hole = cylinder(radius=1.65, height=18.0, sections=24)
        p_hole.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
        p_hole.apply_translation([fx, fy - 4.0, fz])
        cutters.append(p_hole)

        t_tun = cylinder(radius=1.3, height=34.0, sections=16)
        t_tun.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
        t_tun.apply_translation([fx, fy - 14.0, fz - 3.5])
        cutters.append(t_tun)

        eyelet = cylinder(radius=1.25, height=8.0, sections=16)
        eyelet.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
        eyelet.apply_translation([fx, fy - 8.0, fz - 3.0])
        cutters.append(eyelet)

    # CORRECTED Thumb CMC joint socket
    th_pos = [-palm_w * 0.36 - 2.5, palm_l * 0.28, -2.0]
    rot_thumb_cmc = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)

    th_slot = box(extents=[5.5, 18.0, 22.0])
    th_slot.apply_transform(rot_thumb_cmc)
    th_slot.apply_translation(th_pos)
    cutters.append(th_slot)

    th_pin = cylinder(radius=1.65, height=24.0, sections=24)
    th_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    th_pin.apply_transform(rot_thumb_cmc)
    th_pin.apply_translation(th_pos)
    cutters.append(th_pin)

    th_tendon = cylinder(radius=1.0, height=30.0, sections=16)
    th_tendon.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    th_tendon.apply_transform(rot_thumb_cmc)
    th_tendon.apply_translation([th_pos[0] + 5, th_pos[1] - 8, th_pos[2]])
    cutters.append(th_tendon)

    th_opp_bore = cylinder(radius=1.0, height=28.0, sections=16)
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
        bolt = cylinder(radius=1.65, height=palm_h + 12.0, sections=16)
        bx = 18.0 * np.cos(ang)
        by = 4.0 + 18.0 * np.sin(ang)
        bolt.apply_translation([bx, by, 0])
        cutters.append(bolt)

    cutter_all = trimesh.boolean.union(cutters)
    return palm_hull.difference(cutter_all)


def assemble_hand():
    """Generates the full assembly with biomechanically corrected opposable thumb."""
    print("=" * 60)
    print("ROBOTIC HAND STL GENERATOR - Revision 3")
    print("Thumb CMC angles: X=+{:.2f} Y={:.2f} Z=+{:.2f} rad".format(
        THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z))
    print("=" * 60)

    print("\n[1/5] Generating Anatomical Palm with cable routing...")
    palm = generate_palm()
    palm.export(os.path.join(STL_DIR, "palm.stl"))

    print("[2/5] Generating Rounded Phalanx Modules...")
    p_phalanx = generate_proximal_phalanx()
    i_phalanx = generate_intermediate_phalanx()
    d_phalanx = generate_distal_phalanx()

    p_phalanx.export(os.path.join(STL_DIR, "proximal_phalanx.stl"))
    i_phalanx.export(os.path.join(STL_DIR, "intermediate_phalanx.stl"))
    d_phalanx.export(os.path.join(STL_DIR, "distal_phalanx.stl"))

    print("[3/5] Generating Forearm Servo Adapter (6x servo pockets)...")
    adapter = generate_forearm_adapter()
    adapter.export(os.path.join(STL_DIR, "forearm_servo_adapter.stl"))

    components = [palm]

    print("[4/5] Assembling 4 Finger Digits...")
    scales = [0.90, 1.00, 0.92, 0.78]
    x_positions = [-23.0, -8.0, 8.0, 23.0]
    knuckle_y = [PALM_L - 2.5, PALM_L, PALM_L - 1.2, PALM_L - 3.8]
    knuckle_z = [0.4, 0.8, 0.3, -0.4]
    angles_z = [0.07, 0.02, -0.04, -0.10]
    flex_mcp = [0.32, 0.28, 0.30, 0.34]
    flex_pip = [0.42, 0.38, 0.40, 0.44]
    flex_dip = [0.25, 0.22, 0.25, 0.28]

    for i in range(4):
        s = scales[i]
        p = generate_proximal_phalanx(length=38.0*s, width=15.0*s, height=13.0*s)
        ip = generate_intermediate_phalanx(length=28.0*s, width=13.0*s, height=11.0*s)
        dp = generate_distal_phalanx(length=24.0*s, width=12.0*s, height=10.0*s)

        dp.apply_transform(trimesh.transformations.rotation_matrix(flex_dip[i], [1, 0, 0]))
        dp.apply_translation([0, 28.0*s, 0])

        finger_tip = trimesh.util.concatenate([ip, dp])
        finger_tip.apply_transform(trimesh.transformations.rotation_matrix(flex_pip[i], [1, 0, 0]))
        finger_tip.apply_translation([0, 38.0*s, 0])

        finger_full = trimesh.util.concatenate([p, finger_tip])
        finger_full.apply_transform(trimesh.transformations.rotation_matrix(flex_mcp[i], [1, 0, 0]))
        finger_full.apply_transform(trimesh.transformations.rotation_matrix(angles_z[i], [0, 0, 1]))
        finger_full.apply_translation([x_positions[i], knuckle_y[i] - 4.0, knuckle_z[i]])
        components.append(finger_full)

    # CORRECTED Opposable Thumb
    print("[5/5] Assembling Corrected Opposable Thumb...")
    th_p = generate_proximal_phalanx(length=32.0, width=15.0, height=13.0)
    th_d = generate_distal_phalanx(length=26.0, width=14.0, height=11.5)

    th_d.apply_transform(trimesh.transformations.rotation_matrix(0.35, [1, 0, 0]))
    th_d.apply_translation([0, 32.0, 0])

    thumb_full = trimesh.util.concatenate([th_p, th_d])
    thumb_full.apply_transform(trimesh.transformations.rotation_matrix(0.30, [1, 0, 0]))

    rot_cmc = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)
    thumb_full.apply_transform(rot_cmc)
    thumb_full.apply_translation([-PALM_W * 0.36 - 2.5, PALM_L * 0.28, -2.0])
    components.append(thumb_full)

    adapter_mesh = generate_forearm_adapter()
    adapter_mesh.apply_translation([0, -2.0, 0])
    components.append(adapter_mesh)

    print("\nMerging complete assembly...")
    full_assembly = trimesh.util.concatenate(components)
    full_assembly.export(os.path.join(STL_DIR, "robotic_hand_full_assembly.stl"))

    thumb_solo_p = generate_proximal_phalanx(length=32.0, width=15.0, height=13.0)
    thumb_solo_d = generate_distal_phalanx(length=26.0, width=14.0, height=11.5)
    thumb_full_solo = trimesh.util.concatenate([thumb_solo_p, thumb_solo_d])
    thumb_full_solo.export(os.path.join(STL_DIR, "thumb_assembly.stl"))

    print("\nAll STL files exported to:", STL_DIR)
    print("  - palm.stl")
    print("  - proximal_phalanx.stl")
    print("  - intermediate_phalanx.stl")
    print("  - distal_phalanx.stl")
    print("  - forearm_servo_adapter.stl")
    print("  - thumb_assembly.stl")
    print("  - robotic_hand_full_assembly.stl")


if __name__ == "__main__":
    assemble_hand()
