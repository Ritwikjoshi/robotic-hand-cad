"""
Procedural 3D Mesh Generator for 5-Finger Anthropomorphic Robotic Hand
Featuring organic, anatomically realistic human palm geometry:
- Carpal cup (palmar concavity)
- Thenar eminence (prominent thumb ball muscle mound)
- Hypothenar eminence (pinky side muscle mound)
- Transverse metacarpal arch (knuckle cascade curve)
- Distal palmar cushions (metacarpal pads below knuckles)
- Smooth organic convex hulls & watertight CSG
"""

import os
import numpy as np
import trimesh
from trimesh.creation import box, cylinder, icosphere

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(OUTPUT_DIR, "stl_exports")
os.makedirs(STL_DIR, exist_ok=True)

def generate_distal_phalanx(length=24.0, width=12.0, height=10.0):
    """Fingertip phalanx with realistic rounded pulp pad and fingernail curve."""
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

    cutters = trimesh.boolean.union([pin_cutter, tendon_slot])
    return body.difference(cutters)

def generate_intermediate_phalanx(length=28.0, width=13.0, height=11.0):
    """Intermediate segment with rounded anatomical silhouette and dual clevis joints."""
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

    cutters = trimesh.boolean.union([clevis_cut, dist_clevis_cut, pin_prox, pin_dist, t_flex, t_ext])
    return body.difference(cutters)

def generate_proximal_phalanx(length=38.0, width=15.0, height=13.0):
    """Proximal phalanx with anatomical curvature, knuckle base, and distal tongue."""
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

    cutters = trimesh.boolean.union([base_clevis, pin_base, pin_dist, t_flex, t_ext])
    return body.difference(cutters)

def generate_palm(palm_w=74.0, palm_l=84.0, palm_h=21.0):
    """
    Sculpts an anatomically realistic human palm:
    - Transverse Metacarpal Knuckle Arch: Index/Pinky slope backward & curve.
    - Thenar Mound (ball of thumb muscle): Full, rounded, bulbous palm base.
    - Hypothenar Mound: Lateral pinky muscular cushion.
    - Metacarpal Heads: Smooth rounded pads across base of digits.
    - Central Palmar Hollow: Natural ergonomic concavity.
    """
    # 1. Carpal base (wrist transition bulb)
    wrist_base = icosphere(subdivisions=3, radius=1.0)
    wrist_base.apply_scale([palm_w * 0.38, 14.0, palm_h * 0.44])
    wrist_base.apply_translation([0, 10.0, 0])

    # 2. Thenar Eminence (Thumb muscle fleshy dome)
    thenar = icosphere(subdivisions=3, radius=1.0)
    thenar.apply_scale([17.0, 24.0, 12.0])
    rot_thenar = trimesh.transformations.euler_matrix(0.22, 0.48, -0.58)
    thenar.apply_transform(rot_thenar)
    thenar.apply_translation([-palm_w * 0.34, palm_l * 0.36, -1.5])

    # 3. Hypothenar Eminence (Pinky side cushion)
    hypo = icosphere(subdivisions=3, radius=1.0)
    hypo.apply_scale([13.5, 26.0, 10.5])
    hypo.apply_translation([palm_w * 0.36, palm_l * 0.40, -1.8])

    # 4. Transverse Knuckle Arch (4 rounded metacarpal knuckle mounds)
    finger_x = [-23.0, -8.0, 8.0, 23.0]
    knuckle_y = [palm_l - 2.5, palm_l, palm_l - 1.2, palm_l - 3.8] # Natural human cascade
    knuckle_z = [0.4, 0.8, 0.3, -0.4]

    knuckle_spheres = []
    for fx, fy, fz in zip(finger_x, knuckle_y, knuckle_z):
        ks = icosphere(subdivisions=3, radius=1.0)
        ks.apply_scale([8.5, 11.0, palm_h * 0.44])
        ks.apply_translation([fx, fy - 6.0, fz])
        knuckle_spheres.append(ks)

    # 5. Distal Palmar Cushion (fleshy transverse ridge just below knuckles)
    distal_ridge = cylinder(radius=palm_h * 0.36, height=palm_w * 0.78, sections=32)
    distal_ridge.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    distal_ridge.apply_translation([0, palm_l - 16.0, -palm_h * 0.16])

    # 6. Anatomical Dorsal Core (dorsal skin of hand)
    dorsal_core = icosphere(subdivisions=3, radius=1.0)
    dorsal_core.apply_scale([palm_w * 0.44, palm_l * 0.42, palm_h * 0.36])
    dorsal_core.apply_translation([0, palm_l * 0.48, palm_h * 0.12])

    # Build anatomical palm body via union and convex hull
    all_palm_parts = [wrist_base, thenar, hypo, distal_ridge, dorsal_core] + knuckle_spheres
    palm_hull = trimesh.boolean.union(all_palm_parts).convex_hull

    # Subtractive Features (Joint Clevises, Tendon tunnels, Cavity, Wrist mount)
    cutters = []

    # 4 Finger knuckle clevis slots & pin holes
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

    # Thumb CMC Clevis Socket
    th_slot = box(extents=[5.5, 18.0, 22.0])
    th_slot.apply_transform(rot_thenar)
    th_slot.apply_translation([-palm_w * 0.34 - 3, palm_l * 0.36, -1.5])
    cutters.append(th_slot)

    th_pin = cylinder(radius=1.65, height=24.0, sections=24)
    th_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    th_pin.apply_transform(rot_thenar)
    th_pin.apply_translation([-palm_w * 0.34 - 3, palm_l * 0.36, -1.5])
    cutters.append(th_pin)

    # Internal hollow cavity for servo wires & tendon routing
    cavity = box(extents=[palm_w * 0.58, palm_l * 0.46, palm_h * 0.68])
    cavity.apply_translation([0, palm_l * 0.44, 0])
    cutters.append(cavity)

    # Central palmar shallow depression / hollow (cup of the hand)
    palm_cup = icosphere(subdivisions=3, radius=1.0)
    palm_cup.apply_scale([palm_w * 0.22, palm_l * 0.22, 6.0])
    palm_cup.apply_translation([0, palm_l * 0.45, -palm_h * 0.50])
    cutters.append(palm_cup)

    # Wrist Mount Interface
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
    """Generates the full assembly with anatomically realistic human palm & curved fingers."""
    print("Generating Anatomical Human Palm...")
    palm = generate_palm()
    palm.export(os.path.join(STL_DIR, "palm.stl"))

    print("Generating Rounded Phalanx Modules...")
    p_phalanx = generate_proximal_phalanx()
    i_phalanx = generate_intermediate_phalanx()
    d_phalanx = generate_distal_phalanx()

    p_phalanx.export(os.path.join(STL_DIR, "proximal_phalanx.stl"))
    i_phalanx.export(os.path.join(STL_DIR, "intermediate_phalanx.stl"))
    d_phalanx.export(os.path.join(STL_DIR, "distal_phalanx.stl"))

    components = [palm]

    # 4 Main fingers with anatomical curvature matching the palm's metacarpal arch
    scales = [0.90, 1.00, 0.92, 0.78]   # Index, Middle, Ring, Pinky
    x_positions = [-23.0, -8.0, 8.0, 23.0]
    knuckle_y = [84.0 - 2.5, 84.0, 84.0 - 1.2, 84.0 - 3.8]
    knuckle_z = [0.4, 0.8, 0.3, -0.4]
    angles_z = [0.07, 0.02, -0.04, -0.10]
    flex_mcp = [0.35, 0.30, 0.32, 0.38]
    flex_pip = [0.45, 0.40, 0.42, 0.48]
    flex_dip = [0.28, 0.25, 0.28, 0.30]

    for i in range(4):
        s = scales[i]
        p = generate_proximal_phalanx(length=38.0 * s, width=15.0 * s, height=13.0 * s)
        ip = generate_intermediate_phalanx(length=28.0 * s, width=13.0 * s, height=11.0 * s)
        dp = generate_distal_phalanx(length=24.0 * s, width=12.0 * s, height=10.0 * s)

        dp.apply_transform(trimesh.transformations.rotation_matrix(flex_dip[i], [1, 0, 0]))
        dp.apply_translation([0, 28.0 * s, 0])

        finger_tip = trimesh.util.concatenate([ip, dp])
        finger_tip.apply_transform(trimesh.transformations.rotation_matrix(flex_pip[i], [1, 0, 0]))
        finger_tip.apply_translation([0, 38.0 * s, 0])

        finger_full = trimesh.util.concatenate([p, finger_tip])
        finger_full.apply_transform(trimesh.transformations.rotation_matrix(flex_mcp[i], [1, 0, 0]))
        finger_full.apply_transform(trimesh.transformations.rotation_matrix(angles_z[i], [0, 0, 1]))
        finger_full.apply_translation([x_positions[i], knuckle_y[i] - 4.0, knuckle_z[i]])

        components.append(finger_full)

    # Opposable Thumb (located on the thenar mound)
    print("Assembling Rounded Opposable Thumb on Thenar Mound...")
    th_p = generate_proximal_phalanx(length=32.0, width=15.0, height=13.0)
    th_d = generate_distal_phalanx(length=26.0, width=14.0, height=11.5)

    th_d.apply_transform(trimesh.transformations.rotation_matrix(0.4, [1, 0, 0]))
    th_d.apply_translation([0, 32.0, 0])

    thumb_full = trimesh.util.concatenate([th_p, th_d])
    thumb_full.apply_transform(trimesh.transformations.euler_matrix(0.32, 0.52, -0.62))
    thumb_full.apply_translation([-74.0 * 0.34 - 3, 84.0 * 0.36, -1.5])
    components.append(thumb_full)

    print("Merging complete full assembly...")
    full_assembly = trimesh.util.concatenate(components)
    full_assembly.export(os.path.join(STL_DIR, "robotic_hand_full_assembly.stl"))
    print("Success! All realistic human hand STL files regenerated in:", STL_DIR)

if __name__ == "__main__":
    assemble_hand()
