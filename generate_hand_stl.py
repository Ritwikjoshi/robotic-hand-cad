"""
Procedural 3D Mesh Generator for 5-Finger Anthropomorphic Robotic Hand
Generates watertight, 3D-printable STL files for each individual component
and complete articulated hand assembly.
"""

import os
import numpy as np
import trimesh
from trimesh.creation import box, cylinder, icosphere

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(OUTPUT_DIR, "stl_exports")
os.makedirs(STL_DIR, exist_ok=True)

def create_pin_joint_pocket(base_mesh, pin_pos, pin_dir, pin_radius=1.65, pin_len=25):
    """Subtractive pin hole for hinge joints (M3 dowel clearance)."""
    cyl = cylinder(radius=pin_radius, height=pin_len, sections=32)
    # Align cylinder with pin_dir
    rot = trimesh.geometry.align_vectors([0, 0, 1], pin_dir)
    cyl.apply_transform(rot)
    cyl.apply_translation(pin_pos)
    return base_mesh.difference(cyl)

def generate_distal_phalanx(length=24.0, width=12.0, height=10.0):
    """Fingertip phalanx with curved pad, tendon anchor socket, and knuckle hinge tongue."""
    # Main tip body
    b = box(extents=[width, length, height])
    b.apply_translation([0, length/2, 0])
    
    # Rounded fingertip pad
    tip_sphere = icosphere(subdivisions=3, radius=height*0.48)
    tip_sphere.apply_translation([0, length - height*0.4, -height*0.1])
    
    # Hinge boss at base
    hinge_boss = cylinder(radius=height*0.46, height=width - 3.4, sections=32)
    hinge_boss.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    
    combined = trimesh.boolean.union([b, tip_sphere, hinge_boss])
    
    # Pin hole (DIP joint)
    pin_cutter = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_cutter.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    
    # Tendon anchor slot
    tendon_slot = box(extents=[width*0.4, 5.0, height*0.65])
    tendon_slot.apply_translation([0, length*0.6, -height*0.15])
    
    final_mesh = combined.difference(pin_cutter)
    final_mesh = final_mesh.difference(tendon_slot)
    return final_mesh

def generate_intermediate_phalanx(length=28.0, width=13.0, height=11.0):
    """Intermediate segment with clevis female joint on proximal and male tongue on distal."""
    clevis_w = 4.2
    
    b = box(extents=[width, length, height])
    b.apply_translation([0, length/2, 0])
    
    # Knuckle ends
    k_prox = cylinder(radius=height*0.48, height=width, sections=32)
    k_prox.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    
    k_dist = cylinder(radius=height*0.44, height=width*0.85, sections=32)
    k_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    k_dist.apply_translation([0, length, 0])
    
    combined = trimesh.boolean.union([b, k_prox, k_dist])
    
    # Proximal clevis slot (female joint receives proximal tongue)
    clevis_cut = box(extents=[clevis_w + 0.8, height*1.4, height*1.4])
    
    # Distal clevis cut (receives distal phalanx tongue)
    dist_clevis_cut = box(extents=[width - 3.0, height*1.3, height*1.3])
    dist_clevis_cut.apply_translation([0, length, 0])
    
    # Pin holes
    pin_prox = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_prox.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    
    pin_dist = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    pin_dist.apply_translation([0, length, 0])
    
    # Tendon path channels (flexor & extensor)
    t_flex = cylinder(radius=0.9, height=length + 6.0, sections=16)
    t_flex.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_flex.apply_translation([0, length/2, -height*0.28])
    
    t_ext = cylinder(radius=0.9, height=length + 6.0, sections=16)
    t_ext.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_ext.apply_translation([0, length/2, height*0.28])
    
    cutters = [clevis_cut, dist_clevis_cut, pin_prox, pin_dist, t_flex, t_ext]
    cutter_union = trimesh.boolean.union(cutters)
    return combined.difference(cutter_union)

def generate_proximal_phalanx(length=38.0, width=15.0, height=13.0):
    """Proximal phalanx attached to palm MCP joint."""
    clevis_w = 4.0
    b = box(extents=[width, length, height])
    b.apply_translation([0, length/2, 0])
    
    # Base knuckle
    k_base = cylinder(radius=height*0.48, height=width, sections=32)
    k_base.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    
    # Distal tongue boss (inserts into intermediate phalanx)
    tongue = cylinder(radius=height*0.42, height=clevis_w, sections=32)
    tongue.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    tongue.apply_translation([0, length, 0])
    
    combined = trimesh.boolean.union([b, k_base, tongue])
    
    # Base clevis slot (for palm clevis)
    base_clevis = box(extents=[5.2, height*1.5, height*1.5])
    
    # Pin holes
    pin_base = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_base.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    
    pin_dist = cylinder(radius=1.65, height=width + 6.0, sections=32)
    pin_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
    pin_dist.apply_translation([0, length, 0])
    
    # Tendon channels
    t_flex = cylinder(radius=0.9, height=length + 6.0, sections=16)
    t_flex.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_flex.apply_translation([0, length/2, -height*0.3])
    
    t_ext = cylinder(radius=0.9, height=length + 6.0, sections=16)
    t_ext.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    t_ext.apply_translation([0, length/2, height*0.3])
    
    cutters = [base_clevis, pin_base, pin_dist, t_flex, t_ext]
    cutter_union = trimesh.boolean.union(cutters)
    return combined.difference(cutter_union)

def generate_palm(palm_w=72.0, palm_l=82.0, palm_h=20.0):
    """Palm chassis with 4 MCP knuckle clevises, thumb abductor boss, and wrist mount."""
    palm_body = box(extents=[palm_w, palm_l, palm_h])
    palm_body.apply_translation([0, palm_l/2, 0])
    
    # Thumb mount angled boss
    thumb_boss = box(extents=[18.0, 22.0, 16.0])
    rot_thumb = trimesh.transformations.euler_matrix(0.26, 0.52, -0.61)
    thumb_boss.apply_transform(rot_thumb)
    thumb_boss.apply_translation([-palm_w/2 - 2, palm_l*0.35, -2])
    
    palm_combined = trimesh.boolean.union([palm_body, thumb_boss])
    
    cutters = []
    
    # 4 Finger knuckle clevis slots & pin holes
    finger_x = [-23.0, -8.0, 8.0, 23.0]
    for fx in finger_x:
        # Clevis cut
        c_slot = box(extents=[4.8, 16.0, palm_h + 4.0])
        c_slot.apply_translation([fx, palm_l, 0])
        cutters.append(c_slot)
        
        # MCP pin hole
        p_hole = cylinder(radius=1.65, height=18.0, sections=24)
        p_hole.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0]))
        p_hole.apply_translation([fx, palm_l - 4.0, 0])
        cutters.append(p_hole)
        
        # Tendon guide tunnel
        t_tun = cylinder(radius=1.3, height=30.0, sections=16)
        t_tun.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
        t_tun.apply_translation([fx, palm_l - 12.0, -4.0])
        cutters.append(t_tun)
    
    # Thumb clevis cut & pin
    th_slot = box(extents=[5.5, 16.0, 22.0])
    th_slot.apply_transform(rot_thumb)
    th_slot.apply_translation([-palm_w/2 - 2, palm_l*0.35, -2])
    cutters.append(th_slot)
    
    # Internal hollow cavity for servo lines / tendon routing
    cavity = box(extents=[palm_w*0.68, palm_l*0.55, palm_h - 6.0])
    cavity.apply_translation([0, palm_l*0.42, 0])
    cutters.append(cavity)
    
    # Wrist mount central bore & 4x M3 screw pattern
    wrist_bore = cylinder(radius=14.0, height=palm_h + 8.0, sections=36)
    wrist_bore.apply_translation([0, 0, 0])
    cutters.append(wrist_bore)
    
    for ang in [np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4]:
        bolt = cylinder(radius=1.65, height=palm_h + 8.0, sections=16)
        bx = 18.0 * np.cos(ang)
        by = 18.0 * np.sin(ang)
        bolt.apply_translation([bx, by, 0])
        cutters.append(bolt)
    
    cutter_all = trimesh.boolean.union(cutters)
    return palm_combined.difference(cutter_all)

def assemble_hand():
    """Generates the full assembly with all 5 fingers articulated into natural grasp posture."""
    print("Generating Palm...")
    palm = generate_palm()
    palm.export(os.path.join(STL_DIR, "palm.stl"))
    
    print("Generating Phalanx Modules...")
    p_phalanx = generate_proximal_phalanx()
    i_phalanx = generate_intermediate_phalanx()
    d_phalanx = generate_distal_phalanx()
    
    p_phalanx.export(os.path.join(STL_DIR, "proximal_phalanx.stl"))
    i_phalanx.export(os.path.join(STL_DIR, "intermediate_phalanx.stl"))
    d_phalanx.export(os.path.join(STL_DIR, "distal_phalanx.stl"))
    
    components = [palm]
    
    # Assemble 4 main fingers with anthropomorphic lengths and flexion
    scales = [0.90, 1.00, 0.92, 0.78]   # Index, Middle, Ring, Pinky
    x_positions = [-23.0, -8.0, 8.0, 23.0]
    angles_z = [0.07, 0.02, -0.04, -0.10]
    flex_mcp = [0.45, 0.40, 0.42, 0.48]
    flex_pip = [0.55, 0.50, 0.52, 0.58]
    flex_dip = [0.35, 0.32, 0.35, 0.38]
    
    for i in range(4):
        s = scales[i]
        p = generate_proximal_phalanx(length=38.0*s, width=15.0*s, height=13.0*s)
        ip = generate_intermediate_phalanx(length=28.0*s, width=13.0*s, height=11.0*s)
        dp = generate_distal_phalanx(length=24.0*s, width=12.0*s, height=10.0*s)
        
        # Position DIP
        dp.apply_transform(trimesh.transformations.rotation_matrix(flex_dip[i], [1, 0, 0]))
        dp.apply_translation([0, 28.0*s, 0])
        
        # Merge PIP + DIP
        finger_tip = trimesh.util.concatenate([ip, dp])
        finger_tip.apply_transform(trimesh.transformations.rotation_matrix(flex_pip[i], [1, 0, 0]))
        finger_tip.apply_translation([0, 38.0*s, 0])
        
        # Merge Proximal
        finger_full = trimesh.util.concatenate([p, finger_tip])
        finger_full.apply_transform(trimesh.transformations.rotation_matrix(flex_mcp[i], [1, 0, 0]))
        finger_full.apply_transform(trimesh.transformations.rotation_matrix(angles_z[i], [0, 0, 1]))
        finger_full.apply_translation([x_positions[i], 82.0 - 4.0, 0])
        
        components.append(finger_full)
    
    # Thumb (opposable 2-segment configuration)
    print("Assembling Opposable Thumb...")
    th_p = generate_proximal_phalanx(length=32.0, width=15.0, height=13.0)
    th_d = generate_distal_phalanx(length=26.0, width=14.0, height=11.5)
    
    th_d.apply_transform(trimesh.transformations.rotation_matrix(0.5, [1, 0, 0]))
    th_d.apply_translation([0, 32.0, 0])
    
    thumb_full = trimesh.util.concatenate([th_p, th_d])
    thumb_full.apply_transform(trimesh.transformations.euler_matrix(0.35, 0.55, -0.65))
    thumb_full.apply_translation([-72.0/2 - 2, 82.0*0.35, -2])
    components.append(thumb_full)
    
    print("Merging complete full assembly...")
    full_assembly = trimesh.util.concatenate(components)
    full_assembly.export(os.path.join(STL_DIR, "robotic_hand_full_assembly.stl"))
    print("Success! All STL files written to:", STL_DIR)

if __name__ == "__main__":
    assemble_hand()
