#!/usr/bin/env python3
"""
Iteration 9 (v9.0) Chassis Module: Biomimetic Palm & 6-Servo Forearm Rack
=========================================================================
Resolves all v8 palm, thumb CMC, MCP knuckle, dorsal anchor, wrist flange,
and forearm servo adapter engineering defects:
1. Sculpted radial-palmar thenar pedestal with protruding 4.4 mm male CMC
   hinge tongue, 0° dorsal extension hard stop, and 0°–90° unobstructed flexion.
2. Continuous Ø2.8 mm internal flexor tendon conduits from all 5 digits into
   an axial carpal routing manifold exiting the Y-oriented wrist opening,
   with 100% single-shell manifold integrity (bodies == 1, zero trapped voids).
3. Splayed MCP knuckle male tongues rotated by exact digit rotZ angles with
   integral 0° dorsal hard-stop shoulders and 0°–95° collision-free flexion.
4. Captive dorsal rubber-band anchor pockets crossed by Ø2.0 mm transverse
   tie-pin bores across raised metacarpal/thenar anchor bosses (all 5 digits).
5. Flat XZ proximal wrist bulkhead at WRIST_INTERFACE_Y = -14.0 with 4x M3
   Y-axis bolt bores and axial Ø22.0 mm carpal passage (no Z-hole through palm).
6. Buildable open-sided dual-column 6x MG90S micro-servo forearm rack
   (12.7 x 23.3 mm side-entry bays, Ø1.8 mm M2 pilot holes on 27.5 mm centers,
   unobstructed central tendon routing channel, single watertight solid).
"""

import numpy as np
import trimesh
from trimesh.creation import box, cylinder, icosphere

# =============================================================================
# REQUIRED V9 CHASSIS CONSTANTS & KINEMATIC INTERFACES
# =============================================================================
PALM_W = 76.0
PALM_L = 86.0
PALM_H = 22.0

THUMB_CMC_X = 0.85     # +48.7 deg palmar abduction
THUMB_CMC_Y = -0.35    # -20.1 deg pronation
THUMB_CMC_Z = 0.50     # +28.6 deg radial abduction
THUMB_CMC_POS = [-33.0, 24.0, -2.0]

WRIST_INTERFACE_Y = -14.0
ADAPTER_PLACEMENT = [0.0, WRIST_INTERFACE_Y - 0.1, 0.0]

WRIST_BOLT_XZ = [
    (-14.0, -9.5),
    ( 14.0, -9.5),
    (-14.0,  9.5),
    ( 14.0,  9.5),
]

KNUCKLE_SPECS = [
    {
        "name": "index",
        "x": -23.0,
        "y": PALM_L - 2.5,
        "py": PALM_L - 6.5,
        "z": 0.4,
        "rotZ": 0.07,
        "w_prox": 12.6,
        "h_p": 12.0,
    },
    {
        "name": "middle",
        "x": -8.0,
        "y": PALM_L,
        "py": PALM_L - 4.0,
        "z": 0.8,
        "rotZ": 0.02,
        "w_prox": 13.4,
        "h_p": 12.0,
    },
    {
        "name": "ring",
        "x": 8.0,
        "y": PALM_L - 1.2,
        "py": PALM_L - 5.2,
        "z": 0.3,
        "rotZ": -0.04,
        "w_prox": 12.8,
        "h_p": 12.0,
    },
    {
        "name": "pinky",
        "x": 23.0,
        "y": PALM_L - 3.8,
        "py": PALM_L - 7.8,
        "z": -0.4,
        "rotZ": -0.10,
        "w_prox": 11.8,
        "h_p": 12.0,
    },
]

# Shared hardware constants aligned with cad_v9/digits.py
PIN_RADIUS = 1.70
TENDON_Z = -3.50
PALM_CONDUIT_R = 1.40      # Ø2.80 mm internal through-palm tendon conduit
CLEVIS_SLOT_W = 5.4
CLEVIS_TONGUE_W = 4.4
FDM_CLEARANCE = 0.25
HUB_R_FRAC = 0.38

WRIST_FLANGE_W = 46.0      # +/-23.0 mm along X
WRIST_FLANGE_H = 30.0      # +/-15.0 mm along Z
WRIST_FLANGE_CORNER_R = 7.0
WRIST_BORE_R = 11.0        # Ø22.0 mm central carpal tendon funnel

ADAPTER_LENGTH = 82.0
SERVO_BAY_Y_CENTERS = [-24.0, -43.0, -62.0]
SERVO_BAY_W_Y = 12.7       # > 12.6 mm spec (clears 12.2-12.4 mm MG90S width)
SERVO_BAY_H_Z = 23.3       # > 23.2 mm spec (clears 22.8-23.0 mm MG90S length)
SERVO_RACK_HALF_X = 31.0   # +/-31.0 mm outer mounting tab face (23.0 mm clear bay depth to |x|=8.0)
SERVO_CHANNEL_HALF_X = 8.0
SERVO_TAB_SCREW_Z = 13.75  # 27.5 mm center-to-center M2 pilot hole spacing
SERVO_PILOT_R = 0.90       # Ø1.8 mm M2 pilot bore


# =============================================================================
# GEOMETRIC PRIMITIVE HELPERS
# =============================================================================
def _rounded_rect_section_xz(half_x, half_z, r_corner, n_per_corner=12):
    """Returns CCW 2D points (x, z) for a rounded rectangle centered at (0, 0)."""
    r = min(r_corner, half_x - 0.2, half_z - 0.2)
    cx = half_x - r
    cz = half_z - r
    corners = [
        ( cx,  cz, 0.0, np.pi / 2),
        (-cx,  cz, np.pi / 2, np.pi),
        (-cx, -cz, np.pi, 3 * np.pi / 2),
        ( cx, -cz, 3 * np.pi / 2, 2 * np.pi),
    ]
    pts = []
    for ox, oz, a0, a1 in corners:
        angs = np.linspace(a0, a1, n_per_corner, endpoint=False)
        for a in angs:
            pts.append((ox + r * np.cos(a), oz + r * np.sin(a)))
    return np.array(pts, dtype=float)


def _loft_y_stations(y_values, xz_rings):
    """
    Builds a closed watertight Trimesh lofted along Y through congruent 2D (x, z) rings.
    Each ring must have the same vertex count N ordered CCW in the XZ plane (looking from +Y to -Y).
    """
    n_slices = len(y_values)
    n_pts = len(xz_rings[0])
    assert all(len(r) == n_pts for r in xz_rings), "All loft rings must have identical point counts!"
    verts = []
    for y, ring in zip(y_values, xz_rings):
        for x, z in ring:
            verts.append([x, y, z])

    faces = []
    # Side quads: for CCW ring in (x, z), cross((x_next-x, 0, z_next-z), (0, dy>0, 0))
    # = (-dz*dy, 0, dx*dy) -> outward normal requires winding [v0, v3, v2], [v0, v2, v1]
    for i in range(n_slices - 1):
        row0 = i * n_pts
        row1 = (i + 1) * n_pts
        for j in range(n_pts):
            jn = (j + 1) % n_pts
            v0 = row0 + j
            v1 = row0 + jn
            v2 = row1 + jn
            v3 = row1 + j
            faces.append([v0, v3, v2])
            faces.append([v0, v2, v1])

    # Proximal cap (y = y_values[0], normal -Y)
    c0_idx = len(verts)
    c0 = np.mean(xz_rings[0], axis=0)
    verts.append([c0[0], y_values[0], c0[1]])
    for j in range(n_pts):
        jn = (j + 1) % n_pts
        faces.append([c0_idx, j, jn])

    # Distal cap (y = y_values[-1], normal +Y)
    c1_idx = len(verts)
    c1 = np.mean(xz_rings[-1], axis=0)
    verts.append([c1[0], y_values[-1], c1[1]])
    last = (n_slices - 1) * n_pts
    for j in range(n_pts):
        jn = (j + 1) % n_pts
        faces.append([c1_idx, last + jn, last + j])

    mesh = trimesh.Trimesh(vertices=np.array(verts, dtype=float), faces=np.array(faces, dtype=int), process=True)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def _straight_bore(p0, p1, radius, sections=28):
    """Creates a clean cylindrical bore cutter between 3D points p0 and p1."""
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    vec = p1 - p0
    dist = float(np.linalg.norm(vec))
    cyl = cylinder(radius=radius, height=dist, sections=sections)
    z_axis = np.array([0.0, 0.0, 1.0])
    u = vec / dist
    cross = np.cross(z_axis, u)
    c_norm = np.linalg.norm(cross)
    dot = float(np.clip(np.dot(z_axis, u), -1.0, 1.0))
    if c_norm > 1e-7:
        cyl.apply_transform(trimesh.transformations.rotation_matrix(np.arccos(dot), cross / c_norm))
    elif dot < 0:
        cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1.0, 0.0, 0.0]))
    cyl.apply_translation((p0 + p1) * 0.5)
    return cyl


def _apply_pose(mesh, rot_mat, pos):
    """Applies a 4x4 rotation matrix followed by 3D translation in place."""
    mesh.apply_transform(rot_mat)
    mesh.apply_translation(pos)
    return mesh


def _build_joint_male_stages(
    r_hub,
    stop_z_top,
    fork_clear_half_x,
    r_condyle_clear,
    anchor_y=-10.0,
    anchor_boss_z_top=9.2,
    pedestal_y_back=-18.0,
    pedestal_w=12.4,
):
    """
    Constructs non-coplanar 3-stage CSG components in local joint frame
    (pivot at (0,0,0), pin along X, distal along +Y, dorsal +Z):
      1. base_adds   : structural root pedestal + rounded dorsal elastic anchor boss
      2. env_cuts    : full-width clevis & 0..95 deg palmar flexion clearance envelope
      3. tongue_adds : exact 4.4 mm wide circular male hub + 0 deg dorsal stop shoulder
      4. detail_cuts : Ø3.4 mm pin bore, palmar tendon pulley groove, dorsal guide,
                       and captive dorsal anchor pocket + Ø2.0 mm transverse X cross-bore
    Eliminating coplanar keep-cylinders prevents sub-micron float32 STL sliver triangles.
    """
    base_adds = []
    env_cuts = []
    tongue_adds = []
    detail_cuts = []

    # --- STAGE 1: Structural Root Pedestal & Dorsal Anchor Boss ---
    y_ped_front = -r_condyle_clear + 0.5
    ped_len = y_ped_front - pedestal_y_back
    ped_z_bot = -r_hub - 2.6
    ped_h = anchor_boss_z_top - ped_z_bot
    pedestal = box(extents=[pedestal_w, ped_len, ped_h])
    pedestal.apply_translation([0.0, 0.5 * (y_ped_front + pedestal_y_back), 0.5 * (anchor_boss_z_top + ped_z_bot)])
    base_adds.append(pedestal)

    boss_len = 6.2
    boss_w = 10.4
    boss_cyl = cylinder(radius=boss_len * 0.5, height=boss_w + 1.2, sections=32)
    boss_cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    boss_cyl.apply_translation([0.0, anchor_y, anchor_boss_z_top - 1.2])
    base_adds.append(boss_cyl)

    # --- STAGE 2: Wide Joint & Flexion Clearance Cutters (carves clean pocket in palm/pedestal) ---
    span_x = fork_clear_half_x * 2.0
    # Concentric condyle clearance cylinder across full clevis width
    c_condyle = cylinder(radius=r_condyle_clear, height=span_x, sections=44)
    c_condyle.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    env_cuts.append(c_condyle)

    # Open forward & dorsal space ahead of y = -r_condyle_clear
    b_front_dors = box(extents=[span_x, r_condyle_clear + 22.0, 32.0])
    b_front_dors.apply_translation([0.0, -0.5 * r_condyle_clear + 11.0, 8.0])
    env_cuts.append(b_front_dors)

    # Full-width palmar flexion sweep (clears 0..95 deg shaft & palmar bulge below z = -3.8 mm)
    palmar_pts = np.array([
        [-fork_clear_half_x,  8.0, -3.6],
        [ fork_clear_half_x,  8.0, -3.6],
        [-fork_clear_half_x, -5.6, -3.8],
        [ fork_clear_half_x, -5.6, -3.8],
        [-fork_clear_half_x, -9.2, -7.8],
        [ fork_clear_half_x, -9.2, -7.8],
        [-fork_clear_half_x, -9.8, -22.0],
        [ fork_clear_half_x, -9.8, -22.0],
        [-fork_clear_half_x,  8.0, -22.0],
        [ fork_clear_half_x,  8.0, -22.0],
    ])
    env_cuts.append(trimesh.convex.convex_hull(palmar_pts))

    # --- STAGE 3: Clean 4.4 mm Male Tongue + 0-Deg Dorsal Hard Stop Shoulder ---
    hub_cyl = cylinder(radius=r_hub, height=CLEVIS_TONGUE_W, sections=48)
    hub_cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    tongue_adds.append(hub_cyl)

    # Tongue neck + dorsal stop block penetrating cleanly 2.2 mm into the solid pedestal wall
    y_stop_front = 0.25
    y_neck_back = -r_condyle_clear - 2.2
    hw_t = CLEVIS_TONGUE_W * 0.5
    # Polygonal profile in YZ keeping bottom above z = -3.5 mm back to pedestal so it never enters palmar_sweep
    neck_pts = np.array([
        [-hw_t, y_stop_front, -3.2],
        [ hw_t, y_stop_front, -3.2],
        [-hw_t, y_stop_front,  stop_z_top],
        [ hw_t, y_stop_front,  stop_z_top],
        [-hw_t, y_neck_back,   stop_z_top],
        [ hw_t, y_neck_back,   stop_z_top],
        [-hw_t, y_neck_back,  -3.5],
        [ hw_t, y_neck_back,  -3.5],
    ])
    tongue_adds.append(trimesh.convex.convex_hull(neck_pts))

    # --- STAGE 4: Precision Bore, Pulley & Captive Anchor Cutters ---
    pin_bore = cylinder(radius=PIN_RADIUS, height=span_x + 6.0, sections=36)
    pin_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    detail_cuts.append(pin_bore)

    # Smooth 2.1 mm wide palmar tendon pulley groove under the pin bridge (z in [-6.5, -2.45])
    pulley_slot = box(extents=[2.10, 9.0, 4.2])
    pulley_slot.apply_translation([0.0, 0.8, -4.55])
    detail_cuts.append(pulley_slot)

    # Coaxial tendon entry bore at z = -3.50 mm from distal hub face into tongue root
    t_entry = cylinder(radius=1.25, height=11.0, sections=24)
    t_entry.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    t_entry.apply_translation([0.0, -0.5, TENDON_Z])
    detail_cuts.append(t_entry)

    # Dorsal elastic guide groove across stop shoulder (1.8 mm wide, leaving two 1.3 mm solid stop rails!)
    z_guide_floor = r_hub + 0.35
    guide_len = abs(anchor_y) + 2.0
    dorsal_guide = box(extents=[1.80, guide_len, 14.0])
    dorsal_guide.apply_translation([0.0, 0.5 * (anchor_y + 2.0), z_guide_floor + 7.0])
    detail_cuts.append(dorsal_guide)

    # Captive Dorsal Elastic Band Anchor: recessed pocket + Ø2.0 mm transverse X tie-bore
    z_pocket_floor = anchor_boss_z_top - 4.4
    anchor_pocket = box(extents=[3.2, 4.6, 12.0])
    anchor_pocket.apply_translation([0.0, anchor_y, z_pocket_floor + 6.0])
    detail_cuts.append(anchor_pocket)

    z_cross_pin = z_pocket_floor + 1.65
    cross_pin = cylinder(radius=1.00, height=pedestal_w + 10.0, sections=24)
    cross_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    cross_pin.apply_translation([0.0, anchor_y, z_cross_pin])
    detail_cuts.append(cross_pin)

    side_rel_w = max(2.2, (pedestal_w - boss_w) * 0.5 + 3.2)
    for sign in (-1.0, 1.0):
        side_rel = box(extents=[side_rel_w, 5.4, 12.0])
        side_rel.apply_translation([
            sign * (boss_w * 0.5 + side_rel_w * 0.5),
            anchor_y,
            z_cross_pin + 4.5,
        ])
        detail_cuts.append(side_rel)

    return base_adds, env_cuts, tongue_adds, detail_cuts


# =============================================================================
# BIOMIMETIC PALM GENERATOR (v9.0)
# =============================================================================
def generate_palm(palm_w=PALM_W, palm_l=PALM_L, palm_h=PALM_H):
    """
    Generates the v9.0 watertight single-body biomimetic palm:
      - Sculpted radial-palmar thenar mount + 4.4 mm male Thumb CMC hinge tongue + 0 deg stop.
      - 4 splayed MCP knuckle male tongues rotated by rotZ + 0 deg dorsal stops.
      - 5 captive dorsal rubber-band anchor pockets with Ø2.0 mm transverse cross-pin bores.
      - Unbroken Ø2.8 mm internal flexor conduits exiting an axial Ø22.0 mm wrist passage.
      - Flat XZ wrist flange at WRIST_INTERFACE_Y = -14.0 with 4x M3 Y-axis bolt bores.
    """
    sx = palm_w / PALM_W
    dy_l = palm_l - PALM_L

    # 1. Smooth organic metacarpal palm core (terminating before knuckle fork zones)
    core_blobs = []

    carpal_arch = icosphere(subdivisions=3, radius=1.0)
    carpal_arch.apply_scale([25.5 * sx, 22.0, palm_h * 0.54])
    carpal_arch.apply_translation([0.0, 10.0, 0.0])
    core_blobs.append(carpal_arch)

    mid_palm = icosphere(subdivisions=3, radius=1.0)
    mid_palm.apply_scale([33.5 * sx, 30.0, palm_h * 0.50])
    mid_palm.apply_translation([0.0, 42.0 + 0.5 * dy_l, 0.0])
    core_blobs.append(mid_palm)

    # Distal metacarpal bar stopping safely behind MCP condyles (y <= py - 7.0)
    for fx0, py0, fz0 in [(-23.0, 79.5, 0.4), (-8.0, 82.0, 0.8), (8.0, 80.8, 0.3), (23.0, 78.2, -0.4)]:
        mc_node = icosphere(subdivisions=3, radius=1.0)
        mc_node.apply_scale([9.2 * sx, 11.0, palm_h * 0.43])
        mc_node.apply_translation([fx0 * sx, (py0 + dy_l) - 16.5, fz0 - 0.4])
        core_blobs.append(mc_node)

    thenar_eminence = icosphere(subdivisions=3, radius=1.0)
    thenar_eminence.apply_scale([15.5 * sx, 21.0, 11.2])
    thenar_eminence.apply_transform(trimesh.transformations.euler_matrix(0.30, -0.22, 0.35))
    thenar_eminence.apply_translation([-22.0 * sx, 24.5, -1.5])
    core_blobs.append(thenar_eminence)

    hypothenar_eminence = icosphere(subdivisions=3, radius=1.0)
    hypothenar_eminence.apply_scale([13.5 * sx, 26.0, 10.6])
    hypothenar_eminence.apply_translation([24.5 * sx, 38.0 + 0.4 * dy_l, -1.4])
    core_blobs.append(hypothenar_eminence)

    palm_core = trimesh.util.concatenate(core_blobs).convex_hull

    # 2. Proximal Y-Axis Wrist Flange & Carpal Loft (flat mating face at WRIST_INTERFACE_Y = -14.0)
    ring_flange = _rounded_rect_section_xz(
        WRIST_FLANGE_W * 0.5, WRIST_FLANGE_H * 0.5, WRIST_FLANGE_CORNER_R, n_per_corner=12
    )
    ring_carpal = _rounded_rect_section_xz(
        26.0 * sx, palm_h * 0.56, 8.5, n_per_corner=12
    )
    wrist_loft = _loft_y_stations(
        [WRIST_INTERFACE_Y - 1.5, -7.5, 4.0, 16.0],
        [ring_flange, ring_flange, ring_carpal, ring_carpal],
    )

    stage1_adds = [palm_core, wrist_loft]
    stage2_env_cuts = []
    stage3_tongues = []
    stage4_detail_cuts = []

    # 3. Four Splayed MCP Knuckle Male Modules (rotated by exact rotZ!)
    r_hub_mcp = 12.0 * HUB_R_FRAC - FDM_CLEARANCE  # 4.31 mm
    for spec in KNUCKLE_SPECS:
        fx = spec["x"] * sx
        py = spec["py"] + dy_l
        fz = spec["z"]
        rot_z = trimesh.transformations.rotation_matrix(spec["rotZ"], [0, 0, 1])

        b_adds, e_cuts, t_adds, d_cuts = _build_joint_male_stages(
            r_hub=r_hub_mcp,
            stop_z_top=5.80,
            fork_clear_half_x=7.35,   # Clears up to 14.7 mm wide proximal fork condyles
            r_condyle_clear=6.45,     # Clears r_z = 5.28 mm condyles by 1.17 mm
            anchor_y=-10.2,
            anchor_boss_z_top=8.8,
            pedestal_y_back=-18.5,
            pedestal_w=12.2,
        )
        for m in b_adds:
            stage1_adds.append(_apply_pose(m, rot_z, [fx, py, fz]))
        for m in e_cuts:
            stage2_env_cuts.append(_apply_pose(m, rot_z, [fx, py, fz]))
        for m in t_adds:
            stage3_tongues.append(_apply_pose(m, rot_z, [fx, py, fz]))
        for m in d_cuts:
            stage4_detail_cuts.append(_apply_pose(m, rot_z, [fx, py, fz]))

    # 4. Sculpted Radial-Palmar Thenar Pedestal + Protruding 4.4 mm Male Thumb CMC Tongue
    th_pos = np.array([THUMB_CMC_POS[0] * sx, THUMB_CMC_POS[1], THUMB_CMC_POS[2]], dtype=float)
    rot_cmc = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)
    r_hub_cmc = 13.0 * HUB_R_FRAC - FDM_CLEARANCE  # 4.69 mm

    th_b_adds, th_e_cuts, th_t_adds, th_d_cuts = _build_joint_male_stages(
        r_hub=r_hub_cmc,
        stop_z_top=6.25,
        fork_clear_half_x=9.20,   # Generous lateral clearance for 14.5 mm wide thumb clevis + hardware
        r_condyle_clear=6.65,     # Clears r_z = 5.72 mm thumb condyles by 0.93 mm
        anchor_y=-9.6,
        anchor_boss_z_top=8.6,
        pedestal_y_back=-12.2,
        pedestal_w=13.0,
    )
    for m in th_b_adds:
        stage1_adds.append(_apply_pose(m, rot_cmc, th_pos))
    for m in th_e_cuts:
        stage2_env_cuts.append(_apply_pose(m, rot_cmc, th_pos))
    for m in th_t_adds:
        stage3_tongues.append(_apply_pose(m, rot_cmc, th_pos))
    for m in th_d_cuts:
        stage4_detail_cuts.append(_apply_pose(m, rot_cmc, th_pos))

    # Sculpted organic bridge fusing the CMC pedestal root (y_cmc <= -7.2) smoothly into palm_core
    cmc_root_collar = box(extents=[12.4, 4.6, 12.2])
    cmc_root_collar.apply_translation([0.0, -9.8, 1.5])
    _apply_pose(cmc_root_collar, rot_cmc, th_pos)

    palm_anchor_blob = icosphere(subdivisions=3, radius=1.0)
    palm_anchor_blob.apply_scale([10.5, 15.5, 9.8])
    palm_anchor_blob.apply_translation([-19.5 * sx, 22.5, -1.5])
    thenar_bridge = trimesh.util.concatenate([cmc_root_collar, palm_anchor_blob]).convex_hull
    stage1_adds.append(thenar_bridge)

    # Wide-envelope thumb flexion clearance cutter (y_cmc >= -6.65 across 0..95 deg sweep)
    th_sweep_box = box(extents=[22.0, 42.0, 42.0])
    th_sweep_box.apply_translation([0.0, 14.35, -14.5])
    stage2_env_cuts.append(_apply_pose(th_sweep_box, rot_cmc, th_pos))

    # Trim any stray bottom corner below z = -15.0 mm and proximal plane behind WRIST_INTERFACE_Y
    bottom_trim = box(extents=[palm_w + 40.0, palm_l + 60.0, 20.0])
    bottom_trim.apply_translation([0.0, 35.0, -25.0])
    stage2_env_cuts.append(bottom_trim)

    wrist_trim = box(extents=[palm_w + 40.0, 30.0, palm_h + 40.0])
    wrist_trim.apply_translation([0.0, WRIST_INTERFACE_Y - 15.0, 0.0])
    stage2_env_cuts.append(wrist_trim)

    # 6. Axial Y-Oriented Carpal Tendon Manifold + Unbroken 5-Digit Ø2.8 mm Conduits
    angs_c = np.linspace(0.0, 2.0 * np.pi, 48, endpoint=False)
    ring_w_circ = np.column_stack([WRIST_BORE_R * np.cos(angs_c), WRIST_BORE_R * np.sin(angs_c)])
    ring_w_oval = np.column_stack([10.5 * np.cos(angs_c), 6.2 * np.sin(angs_c)])
    ring_w_tip = np.column_stack([5.0 * np.cos(angs_c), 3.0 * np.sin(angs_c)])
    carpal_manifold = _loft_y_stations(
        [WRIST_INTERFACE_Y - 2.0, -8.5, 0.0, 14.0, 19.5],
        [ring_w_circ, ring_w_circ, ring_w_oval, ring_w_oval, ring_w_tip],
    )
    stage4_detail_cuts.append(carpal_manifold)

    # Four unbroken straight Ø2.8 mm finger flexor bores from open MCP pulley slot into open carpal manifold
    wrist_finger_target = np.array([0.0, WRIST_INTERFACE_Y - 2.0, -1.2])
    for spec in KNUCKLE_SPECS:
        fx = spec["x"] * sx
        py = spec["py"] + dy_l
        fz = spec["z"]
        rot_z3 = trimesh.transformations.rotation_matrix(spec["rotZ"], [0, 0, 1])[:3, :3]
        p_mcp_start = np.array([fx, py, fz]) + rot_z3 @ np.array([0.0, 2.5, TENDON_Z])
        stage4_detail_cuts.append(_straight_bore(p_mcp_start, wrist_finger_target, PALM_CONDUIT_R, sections=28))

    # Thumb CMC unbroken 2-segment Ø2.8 mm flexor conduit joined via slightly oversized Ø3.2 mm rounded elbow
    R_cmc3 = rot_cmc[:3, :3]
    p_th_start = th_pos + 2.5 * R_cmc3[:, 1] + TENDON_Z * R_cmc3[:, 2]
    p_th_elbow = th_pos - 4.8 * R_cmc3[:, 1] + TENDON_Z * R_cmc3[:, 2]
    p_th_wrist = np.array([-4.5, WRIST_INTERFACE_Y - 2.0, -2.5])
    elbow_sphere = icosphere(subdivisions=2, radius=PALM_CONDUIT_R + 0.22)
    elbow_sphere.apply_translation(p_th_elbow)
    stage4_detail_cuts.append(_straight_bore(p_th_start, p_th_elbow, PALM_CONDUIT_R, sections=28))
    stage4_detail_cuts.append(elbow_sphere)
    stage4_detail_cuts.append(_straight_bore(p_th_elbow, p_th_wrist, PALM_CONDUIT_R, sections=28))

    # 7. Four Y-Oriented M3 Wrist Bolt Bores (r = 1.70 mm, 12.5 mm deep from WRIST_INTERFACE_Y)
    for bx, bz in WRIST_BOLT_XZ:
        bolt_y = cylinder(radius=PIN_RADIUS, height=13.5, sections=28)
        bolt_y.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
        bolt_y.apply_translation([bx, WRIST_INTERFACE_Y + 5.75, bz])
        stage4_detail_cuts.append(bolt_y)

    # Execute clean 3-stage non-coplanar boolean pipeline
    palm_cleared = trimesh.boolean.union(stage1_adds).difference(trimesh.boolean.union(stage2_env_cuts))
    palm_with_tongues = trimesh.boolean.union([palm_cleared] + stage3_tongues)
    return palm_with_tongues.difference(trimesh.boolean.union(stage4_detail_cuts))


# =============================================================================
# 6-SERVO OPEN-ACCESS FOREARM RACK GENERATOR (v9.0)
# =============================================================================
def generate_forearm_adapter(adapter_length=ADAPTER_LENGTH):
    """
    Generates the v9.0 buildable 6x MG90S open-access forearm servo rack:
      - Front mating face at y = 0.0 extending along -Y to -adapter_length (-82.0 mm).
      - Front wrist flange (y in [-5.0, 0.0]) matching the palm's 4x M3 Y-axis bolt
        pattern (WRIST_BOLT_XZ) and central Ø22.0 mm tendon passage.
      - Open-sided dual-column 6x MG90S rack (3 bays on -X, 3 bays on +X):
        * Each bay 12.7 x 23.3 mm opening directly outward along +/-X with 23.0 mm radial depth.
        * 12x Ø1.8 mm M2 pilot screw holes on exact 27.5 mm mounting tab centers (z = +/-13.75 mm).
        * Unobstructed 16.0 x 22.0 mm central tendon routing corridor + angled guide ports.
      - Watertight single manifold solid (bodies == 1).
    """
    rack_half_z = 17.0
    ring_front = _rounded_rect_section_xz(
        WRIST_FLANGE_W * 0.5, WRIST_FLANGE_H * 0.5, WRIST_FLANGE_CORNER_R, n_per_corner=12
    )
    ring_rack = _rounded_rect_section_xz(
        SERVO_RACK_HALF_X, rack_half_z, 3.2, n_per_corner=12
    )

    # Smooth monolithic outer chassis lofted from front flange (y = 0) to rear frame (y = -82.0)
    outer_body = _loft_y_stations(
        [-adapter_length, -12.5, -5.0, 0.0],
        [ring_rack, ring_rack, ring_front, ring_front],
    )

    cutters = []

    # 1. Central Tendon Routing Passage:
    # Ø22.0 mm circular funnel through front flange & neck (y in [-14.0, +1.0])
    front_funnel = cylinder(radius=WRIST_BORE_R, height=16.0, sections=48)
    front_funnel.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    front_funnel.apply_translation([0.0, -6.5, 0.0])
    cutters.append(front_funnel)

    # Full-length central rectangular tendon corridor (16.0 x 22.0 mm) through rear frame
    central_corridor = box(extents=[SERVO_CHANNEL_HALF_X * 2.0, adapter_length + 4.0, 22.0])
    central_corridor.apply_translation([0.0, -0.5 * adapter_length - 2.0, 0.0])
    cutters.append(central_corridor)

    # 2. Four Y-Axis M3 Wrist Flange Bolt Thru-Holes + Rear Bolt-Head Access Windows
    for bx, bz in WRIST_BOLT_XZ:
        flange_bolt = cylinder(radius=PIN_RADIUS, height=10.0, sections=28)
        flange_bolt.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
        flange_bolt.apply_translation([bx, -2.0, bz])
        cutters.append(flange_bolt)

        # Counterbore / wrench clearance pocket behind the 4.8 mm front flange (y in [-12.3, -4.8])
        head_access = cylinder(radius=3.6, height=7.5, sections=28)
        head_access.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
        head_access.apply_translation([bx, -8.55, bz])
        cutters.append(head_access)

    # Side access windows in neck (y in [-12.2, -5.3]) so M3 flange screws insert easily from outside
    for sign in (-1.0, 1.0):
        neck_win = box(extents=[26.0, 6.9, 23.0])
        neck_win.apply_translation([sign * (10.5 + 13.0), -8.75, 0.0])
        cutters.append(neck_win)

    # 3. Six Open-Sided MG90S Micro-Servo Bays (3 on -X, 3 on +X) + 12x M2 Pilot Screw Bores
    bay_span_x = (SERVO_RACK_HALF_X - SERVO_CHANNEL_HALF_X) + 12.0  # Extends from |x|=8.0 out past |x|=31.0
    for sign in (-1.0, 1.0):
        bx_center = sign * (SERVO_CHANNEL_HALF_X + bay_span_x * 0.5)
        pilot_x_center = sign * (SERVO_RACK_HALF_X - 4.5)  # 10.0 mm deep pilot bore from |x|=31.0 to |x|=21.0
        for yc in SERVO_BAY_Y_CENTERS:
            bay_box = box(extents=[bay_span_x, SERVO_BAY_W_Y, SERVO_BAY_H_Z])
            bay_box.apply_translation([bx_center, yc, 0.0])
            cutters.append(bay_box)

            for sz in (-SERVO_TAB_SCREW_Z, SERVO_TAB_SCREW_Z):
                pilot = cylinder(radius=SERVO_PILOT_R, height=11.0, sections=20)
                pilot.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
                pilot.apply_translation([pilot_x_center, yc, sz])
                cutters.append(pilot)

    # 4. Rear weight-relief / wire-harness side windows behind Bay 3 (y in [-76.5, -70.5])
    for sign in (-1.0, 1.0):
        rear_win = box(extents=[bay_span_x, 6.0, 20.0])
        rear_win.apply_translation([sign * (SERVO_CHANNEL_HALF_X + bay_span_x * 0.5), -73.5, 0.0])
        cutters.append(rear_win)

    return outer_body.difference(trimesh.boolean.union(cutters))
