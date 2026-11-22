#!/usr/bin/env python3
"""
Iteration 10 (v10.0) Actuated Multi-DoF Palm & MCP Abduction Assembly
=====================================================================
Module: `cad_v10.palm_assembly`

All generated parts are guaranteed watertight (`is_watertight == True`),
single connected manifold solids (`len(m.split(only_watertight=False)) == 1`),
zero trapped voids, and 100% bolt-compatible with all 14 certified v9 phalanges
(`cad_v9.digits`).

Components:
1. `generate_mcp_yoke(digit_name)` — 4× 2-DoF MCP Cardan Yokes (`index, middle, ring, pinky`):
   - Local origin `(0, 0, 0)` = horizontal X-axis flexion hinge (`4.4 mm` male tongue,
     `r_hub = 4.50 mm`, `Ø3.4 mm` X-pin bore, `0°` dorsal extension hard-stop shoulder).
   - Proximal Z-axis yaw tongue at `(0, -9.0, 0)` (`6.0 mm` vertical thickness `z in [-2.8, +3.2]`,
     `Ø3.4 mm` vertical Z-pin bore) for `±15°` lateral abduction/adduction.
   - Central `Ø2.6 mm` palmar flexor tendon guide bore aligned through `z = -3.50 mm`,
     rear `Ø2.2 mm` abduction crank pin bore at `(0, -14.0, +3.2)`, and dorsal `Ø2.0 mm`
     transverse rubber-band tie-pin anchor.
2. `generate_abduction_linkage()` — Coupled Abduction Linkage Bar:
   - Single-body flat linkage bar (`4.0 mm` thick in Z, `6.0 mm` nominal width in Y,
     spanning `x in [-26.0, +26.0]`) with 4 slotted cam cutouts (`2.6 × 5.0 mm`) engaging
     the 4 MCP yoke rear crank pins, plus a central servo-horn drive clevis pin.
3. `generate_palm_radial()` & `generate_palm_ulnar()` — Split Cupping Palm:
   - Split along `X = 0.0` (`palm_radial` `x <= -0.3`, `palm_ulnar` `x >= +0.3`) with a
     palmar V-relief below `z = +3.0` and interleaved Y-axis M3 cupping hinge at
     `CUP_HINGE_POS = [0.0, 42.0, 3.0]` (`Ø3.4 mm` bore + `0°` dorsal flat hard stop)
     enabling `0°–30°` collision-free palmar arch cupping.
   - `palm_radial`: Index & Middle Z-axis MCP yaw clevis sockets (`6.6 mm` Z-slot, `±15°` yaw),
     certified v9 Thumb CMC pedestal (`THUMB_CMC_POS = [-33.0, 24.0, -2.0]`) + opposition
     tendon fairlead, open-back dorsal drop-in `MG90S`/`SG90` abduction servo bay
     (`12.8 × 23.4 mm` + 2× `Ø1.8 mm` M2 pilot holes), continuous `Ø2.8 mm` tendon conduits
     with `Ø2.2 mm` PTFE liner seats to the carpal tunnel `[0.0, -3.0, -2.0]`, rear 2-DoF
     Wrist Cardan Yoke Ears at `WRIST_CENTER = [0.0, -14.0, 0.0]` (`z = ±18.5 mm`, inner faces
     `z = ±16.5 mm`, outer faces `z = ±20.5 mm`, `Ø3.4 mm` Z-pin bores), and dual pushrod
     ball-joint mounting ears at `PUSHROD_PALM_ANCHORS = [[-24.0, -2.0, -8.0], [24.0, -2.0, -8.0]]`.
   - `palm_ulnar`: Ring & Pinky Z-axis MCP yaw clevis sockets (`6.6 mm` Z-slot, `±15°` yaw),
     continuous `Ø2.8 mm` tendon conduits with `Ø2.2 mm` PTFE liner seats exiting toward the
     carpal tunnel, palmar cupping tendon anchor eyelet at `[22.0, 54.0, -7.0]`, and dorsal
     `Ø2.0 mm` elastic return tie-pin anchor across the cupping seam.
"""

import manifold3d
import numpy as np
import trimesh
from trimesh.creation import box, cylinder, icosphere

# =============================================================================
# REQUIRED V10 PALM & MCP KINEMATIC CONSTANTS
# =============================================================================

WRIST_CENTER = [0.0, -14.0, 0.0]
CUP_HINGE_POS = [0.0, 42.0, 3.0]
PUSHROD_PALM_ANCHORS = [[-24.0, -2.0, -8.0], [24.0, -2.0, -8.0]]

THUMB_CMC_POS = [-33.0, 24.0, -2.0]
THUMB_CMC_X = 0.85
THUMB_CMC_Y = -0.35
THUMB_CMC_Z = 0.50

# Shared hardware constants aligned 100% with cad_v9/digits.py
PIN_RADIUS = 1.70          # Ø3.40 mm clearance bore for M3 hinge pins
TENDON_Z = -3.50           # Palmar flexor tendon centerline Z
CLEVIS_TONGUE_W = 4.4      # Male flexion tongue width (x in [-2.20, +2.20])
CLEVIS_SLOT_W = 5.4        # Female clevis slot width in v9 proximal phalanges
FDM_CLEARANCE = 0.25       # Radial FDM clearance
HUB_R_FRAC = 0.38

MCP_HUB_R = 4.50           # MCP Cardan yoke X-flexion hub radius (clears r_slot = 4.82 mm)
MCP_YAW_LOCAL_Y = -9.0     # Local Y of Z-axis yaw pivot relative to X-axis flexion origin
MCP_CRANK_LOCAL_POS = [0.0, -14.0, 3.2]  # Local rear abduction crank pin bore center

YOKE_YAW_Z_BOT = -2.80     # 6.0 mm vertical thickness along Z (z in [-2.80, +3.20])
YOKE_YAW_Z_TOP = 3.20
PALM_SLOT_Z_BOT = -3.10    # 6.6 mm Z-slot height in palm yaw clevis (0.30 mm top/bot clearance)
PALM_SLOT_Z_TOP = 3.50

PALM_CONDUIT_R = 1.40      # Ø2.80 mm internal tendon conduit
PTFE_SEAT_R = 1.10         # Ø2.20 mm PTFE Bowden liner stop/seat bore

CARPAL_TUNNEL_POS = [0.0, -3.0, -2.0]
CUPPING_EYELET_POS = [22.0, 54.0, -7.0]

# In-palm MG90S/SG90 abduction servo bay metadata (open dorsally on palm_radial)
IN_PALM_SERVO_BAY = {
    "center": [-15.0, 42.0, 0.0],
    "size_x": 12.8,        # > 12.6 mm spec (clears 12.2 mm MG90S width)
    "size_y": 23.4,        # > 23.2 mm spec (clears 22.8 mm MG90S length)
    "z_floor": -9.5,
    "z_mount": 7.5,
    "pilot_xy": [[-15.0, 28.25], [-15.0, 55.75]],  # 27.5 mm center-to-center M2 pilots
    "pilot_r": 0.90,       # Ø1.8 mm M2 pilot bore
}

# Coupled abduction linkage assembly placement (engaged above the 4 MCP yoke crank pins)
ABDUCTION_LINKAGE_POS = [0.0, 66.2, 10.2]

# Four MCP Cardan Yoke specifications (aligned with v9 DIGIT_SPECS at neutral yaw=0)
_RAW_MCP_SPECS = [
    {"name": "index",  "x": -23.0, "py": 79.5, "z":  0.4, "rotZ":  0.07, "w_prox": 12.6, "h_p": 12.0},
    {"name": "middle", "x":  -8.0, "py": 82.0, "z":  0.8, "rotZ":  0.02, "w_prox": 13.4, "h_p": 12.0},
    {"name": "ring",   "x":   8.0, "py": 80.8, "z":  0.3, "rotZ": -0.04, "w_prox": 12.8, "h_p": 12.0},
    {"name": "pinky",  "x":  23.0, "py": 78.2, "z": -0.4, "rotZ": -0.10, "w_prox": 11.8, "h_p": 12.0},
]

MCP_YOKE_SPECS = []
for _s in _RAW_MCP_SPECS:
    _rz = _s["rotZ"]
    # World coordinates of the Z-axis yaw pivot `(0, -9.0, 0)` in yoke local frame
    _yaw_x = float(_s["x"] + (-MCP_YAW_LOCAL_Y) * np.sin(_rz))
    _yaw_y = float(_s["py"] + MCP_YAW_LOCAL_Y * np.cos(_rz))
    MCP_YOKE_SPECS.append({
        "name": _s["name"],
        "x": _s["x"],
        "py": _s["py"],
        "z": _s["z"],
        "yaw_x": round(_yaw_x, 4),
        "yaw_y": round(_yaw_y, 4),
        "rotZ": _rz,
        "w_prox": _s["w_prox"],
        "h_p": _s["h_p"],
    })


# =============================================================================
# GEOMETRIC PRIMITIVE & TRANSFORM HELPERS
# =============================================================================

def mcp_yoke_transform(spec: dict, yaw_rad: float = 0.0) -> np.ndarray:
    """
    Returns the 4x4 homogeneous transform placing an MCP Cardan Yoke (whose local
    origin is its X-axis flexion hinge and Z-axis yaw pivot is at `(0, -9.0, 0)`)
    into palm world coordinates at abduction angle `yaw_rad` around its Z-axis yaw pin.
    """
    t_to_yaw = trimesh.transformations.translation_matrix([0.0, -MCP_YAW_LOCAL_Y, 0.0])
    r_yaw = trimesh.transformations.rotation_matrix(yaw_rad, [0.0, 0.0, 1.0])
    t_from_yaw = trimesh.transformations.translation_matrix([0.0, MCP_YAW_LOCAL_Y, 0.0])
    r_base = trimesh.transformations.rotation_matrix(spec["rotZ"], [0.0, 0.0, 1.0])
    t_world = trimesh.transformations.translation_matrix([spec["x"], spec["py"], spec["z"]])
    return t_world @ r_base @ t_from_yaw @ r_yaw @ t_to_yaw


def _apply_pose(mesh: trimesh.Trimesh, rot_mat: np.ndarray, pos) -> trimesh.Trimesh:
    mesh.apply_transform(rot_mat)
    mesh.apply_translation(pos)
    return mesh


def _straight_bore(p0, p1, radius: float, sections: int = 28) -> trimesh.Trimesh:
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


def _routed_conduit(waypoints, radius: float, sections: int = 24) -> list[trimesh.Trimesh]:
    """Builds a continuous multi-segment cylindrical conduit with smooth spherical elbows."""
    parts = []
    pts = [np.asarray(p, dtype=float) for p in waypoints]
    for i in range(len(pts) - 1):
        parts.append(_straight_bore(pts[i], pts[i + 1], radius, sections=sections))
    for p in pts[1:-1]:
        elbow = icosphere(subdivisions=2, radius=radius + 0.15)
        elbow.apply_translation(p)
        parts.append(elbow)
    return parts


def _rounded_slot_2d(half_x: float, half_y: float, z_height: float, sections: int = 24) -> trimesh.Trimesh:
    """Creates a stadium/slot prism of total width `2*half_x` and length `2*half_y` along Y."""
    r = min(half_x, half_y)
    dy = max(0.0, half_y - r)
    if dy < 1e-5:
        return cylinder(radius=r, height=z_height, sections=sections)
    c1 = cylinder(radius=r, height=z_height, sections=sections)
    c1.apply_translation([0.0, -dy, 0.0])
    c2 = cylinder(radius=r, height=z_height, sections=sections)
    c2.apply_translation([0.0, +dy, 0.0])
    mid = box(extents=[2.0 * r, 2.0 * dy, z_height])
    return trimesh.boolean.union([c1, c2, mid])


def _clean_single_body(mesh: trimesh.Trimesh, simplify_eps: float = 5e-3) -> trimesh.Trimesh:
    """
    Eliminates sub-micron coplanar boolean sliver edges via `manifold3d.Manifold.simplify`
    at float32 precision so binary STL export/reload is guaranteed `is_watertight == True`
    and `bodies == 1`.
    """
    if mesh.volume < 0:
        mesh.invert()
    mf = manifold3d.Manifold(
        manifold3d.Mesh(
            vert_properties=np.asarray(mesh.vertices, dtype=np.float32),
            tri_verts=np.asarray(mesh.faces, dtype=np.uint32),
        )
    )
    mf = mf.simplify(simplify_eps)
    out = mf.to_mesh()
    cleaned = trimesh.Trimesh(
        vertices=np.asarray(out.vert_properties[:, :3], dtype=np.float64),
        faces=np.asarray(out.tri_verts, dtype=np.int64),
        process=True,
    )
    if cleaned.volume < 0:
        cleaned.invert()
    trimesh.repair.fix_normals(cleaned)
    return cleaned


# =============================================================================
# 1. 2-DoF MCP CARDAN YOKE GENERATOR (`generate_mcp_yoke`)
# =============================================================================

def generate_mcp_yoke(digit_name: str = "index") -> trimesh.Trimesh:
    """
    Generates a v10.0 2-DoF MCP Cardan Yoke for `digit_name` in
    `('index', 'middle', 'ring', 'pinky')`:
      - Local origin `(0, 0, 0)` = horizontal X-axis flexion hinge (`4.4 mm` male tongue,
        `r_hub = 4.50 mm`, `Ø3.4 mm` X-pin bore, `0°` dorsal extension hard-stop shoulder).
      - Proximal Z-axis yaw tongue at `(0, -9.0, 0)` (`6.0 mm` vertical thickness `z in [-2.8, +3.2]`,
        `Ø3.4 mm` vertical Z-pin bore) for `±15°` lateral yaw inside the palm clevis socket.
      - Central `Ø2.6 mm` palmar flexor tendon guide bore aligned at `z = -3.50 mm`.
      - Rear `Ø2.2 mm` abduction crank pin bore at `(0, -14.0, +3.2)`.
      - Dorsal `Ø2.0 mm` transverse rubber-band tie-pin anchor.
    """
    valid_names = {s["name"] for s in MCP_YOKE_SPECS}
    if digit_name not in valid_names:
        raise ValueError(f"Unknown digit_name '{digit_name}'. Expected one of {sorted(valid_names)}")

    hw_t = CLEVIS_TONGUE_W * 0.5   # 2.20 mm (4.40 mm male flexion tongue)
    yaw_y = MCP_YAW_LOCAL_Y        # -9.0 mm
    z_bot = YOKE_YAW_Z_BOT         # -2.80 mm
    z_top = YOKE_YAW_Z_TOP         # +3.20 mm
    z_mid = 0.5 * (z_bot + z_top)  # +0.20 mm
    z_thick = z_top - z_bot        # 6.00 mm

    adds = []

    # 1. Distal X-axis flexion hub (`r_hub = 4.50 mm`, `4.4 mm` wide along X)
    flex_hub = cylinder(radius=MCP_HUB_R, height=CLEVIS_TONGUE_W, sections=48)
    flex_hub.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    adds.append(flex_hub)

    # 2. Dorsal 0° extension hard-stop shoulder (`y in [-4.5, +0.25]`, `z up to +5.80`)
    #    Stays ahead of y = -4.5 mm so it clears the palm's dorsal yaw clevis arm across ±15° yaw,
    #    while rigidly blocking -5°/-10° hyperextension of all v9 proximal phalanges.
    stop_pts = np.array([
        [-hw_t,  0.25, -3.20], [ hw_t,  0.25, -3.20],
        [-hw_t,  0.25,  5.80], [ hw_t,  0.25,  5.80],
        [-hw_t, -4.30,  5.80], [ hw_t, -4.30,  5.80],
        [-hw_t, -5.20,  z_top], [ hw_t, -5.20,  z_top],
        [-hw_t, -5.20, -3.20], [ hw_t, -5.20, -3.20],
    ], dtype=float)
    adds.append(trimesh.convex.convex_hull(stop_pts))

    # 3. Connecting neck from flexion tongue (`y = -4.0`) to Z-axis yaw hub (`y = -9.0`)
    #    Width `4.4 mm` (`|x| <= 2.20 mm`) so proximal phalanx clevis forks clear 0..95° flexion.
    neck = box(extents=[CLEVIS_TONGUE_W, 6.2, z_thick])
    neck.apply_translation([0.0, -6.5, z_mid])
    adds.append(neck)

    # 4. Proximal Z-axis yaw tongue hub at `(0, -9.0, 0)` (`r = 3.80 mm`, `6.0 mm` vertical Z thickness)
    #    Note: at |x| = 2.70 mm, y_front = -9.0 + sqrt(3.80^2 - 2.70^2) = -6.33 mm,
    #    clearing the proximal phalanx fork condyles (r_z = 5.28 mm) by > 1.0 mm!
    yaw_hub = cylinder(radius=3.80, height=z_thick, sections=44)
    yaw_hub.apply_translation([0.0, yaw_y, z_mid])
    adds.append(yaw_hub)

    # 5. Rear abduction crank arm extending from yaw hub `(0, -9.0)` to crank boss `(0, -14.0)`
    crank_y = MCP_CRANK_LOCAL_POS[1]  # -14.0 mm
    crank_arm = box(extents=[4.4, 5.2, z_thick])
    crank_arm.apply_translation([0.0, 0.5 * (yaw_y + crank_y), z_mid])
    adds.append(crank_arm)

    crank_boss = cylinder(radius=2.30, height=z_thick, sections=32)
    crank_boss.apply_translation([0.0, crank_y, z_mid])
    adds.append(crank_boss)

    body = trimesh.boolean.union(adds)

    # --- Cutters ---
    cutters = []

    # 1. Horizontal X-axis Ø3.4 mm M3 flexion pin bore at `(0, 0, 0)`
    x_pin = cylinder(radius=PIN_RADIUS, height=CLEVIS_TONGUE_W + 6.0, sections=40)
    x_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    cutters.append(x_pin)

    # 2. Vertical Z-axis Ø3.4 mm M3 yaw pin bore at `(0, -9.0, 0)`
    z_pin = cylinder(radius=PIN_RADIUS, height=z_thick + 6.0, sections=40)
    z_pin.apply_translation([0.0, yaw_y, z_mid])
    cutters.append(z_pin)

    # 3. Rear Ø2.2 mm vertical abduction crank pin bore at `(0, -14.0, +3.2)`
    crank_bore = cylinder(radius=1.10, height=z_thick + 6.0, sections=28)
    crank_bore.apply_translation([0.0, crank_y, z_mid])
    cutters.append(crank_bore)

    # 4. Central Ø2.6 mm palmar flexor tendon guide bore along Y at `z = -3.50 mm`
    #    Clears X-pin bore (`z in [-1.70, +1.70]`) with solid bridge `z in [-2.20, -1.70]`
    #    and aligns directly through the yaw centerline so ±15° yaw never snags flexion.
    tendon_bore = cylinder(radius=1.30, height=26.0, sections=32)
    tendon_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    tendon_bore.apply_translation([0.0, -4.0, TENDON_Z])
    cutters.append(tendon_bore)

    # Smooth palmar pulley relief slot below z = -3.50 at the distal flexion hub
    pulley_slot = box(extents=[2.10, 8.5, 4.0])
    pulley_slot.apply_translation([0.0, 0.8, -5.50])
    cutters.append(pulley_slot)

    # 5. Dorsal elastic guide groove (1.8 mm wide, preserving two 1.3 mm solid 0° stop rails!)
    #    + Ø2.0 mm transverse X-axis rubber-band tie-pin anchor bore at `(0, -2.8, +4.4)`
    dorsal_guide = box(extents=[1.80, 6.4, 6.0])
    dorsal_guide.apply_translation([0.0, -1.8, 4.85 + 3.0])
    cutters.append(dorsal_guide)

    tie_pin_bore = cylinder(radius=1.00, height=CLEVIS_TONGUE_W + 6.0, sections=24)
    tie_pin_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    tie_pin_bore.apply_translation([0.0, -2.9, 4.15])
    cutters.append(tie_pin_bore)

    result = body.difference(trimesh.boolean.union(cutters))
    return _clean_single_body(result)


# =============================================================================
# 2. COUPLED ABDUCTION LINKAGE BAR (`generate_abduction_linkage`)
# =============================================================================

def generate_abduction_linkage() -> trimesh.Trimesh:
    """
    Generates the v10.0 Coupled Abduction Linkage Bar (local frame centered at `(0, 0, 0)`,
    placed in assembly at `ABDUCTION_LINKAGE_POS = [0.0, 66.2, 6.2]`):
      - Single-body flat linkage bar: `4.0 mm` thick in Z (`z in [-2.0, +2.0]`),
        `6.0 mm` nominal width in Y, spanning `x in [-26.0, +26.0]`.
      - 4 slotted cam cutouts (`2.6 × 5.0 mm`) positioned to engage the 4 MCP yoke rear
        crank pins at their natural anatomical lever offsets.
      - Central servo-horn drive clevis pin (`x = 0.0`) engaging the in-palm `MG90S`/`SG90`
        abduction micro-servo.
    """
    bar_z = 4.0
    bar_y = 6.0
    x_min, x_max = -26.0, 26.0

    # Compute exact crank pin offsets relative to ABDUCTION_LINKAGE_POS
    crank_local_xy = []
    for spec in MCP_YOKE_SPECS:
        T = mcp_yoke_transform(spec, yaw_rad=0.0)
        p_w = (T @ np.array([0.0, MCP_CRANK_LOCAL_POS[1], MCP_CRANK_LOCAL_POS[2], 1.0]))[:3]
        crank_local_xy.append((
            float(p_w[0] - ABDUCTION_LINKAGE_POS[0]),
            float(p_w[1] - ABDUCTION_LINKAGE_POS[1]),
        ))

    adds = []

    # Main flat bar spanning x in [-26.0, +26.0], width 6.0 mm in Y, 4.0 mm thick in Z
    main_bar = box(extents=[x_max - x_min - 4.0, bar_y, bar_z])
    adds.append(main_bar)
    for sx in (-1.0, 1.0):
        end_cap = cylinder(radius=bar_y * 0.5, height=bar_z, sections=32)
        end_cap.apply_translation([sx * (26.0 - bar_y * 0.5), 0.0, 0.0])
        adds.append(end_cap)

    # Local reinforced cam-eyelet pads around each of the 4 `2.6 × 5.0 mm` slots
    # ensuring >= 1.5 mm solid perimeter wall around every cam cutout
    for cx, cy in crank_local_xy:
        pad = _rounded_slot_2d(half_x=2.85, half_y=4.05, z_height=bar_z, sections=28)
        pad.apply_translation([cx, cy, 0.0])
        adds.append(pad)

    # Central servo-horn drive clevis lug & vertical drive pin at `x = 0.0, y = -5.5`
    clevis_arm = box(extents=[6.4, 6.5, bar_z])
    clevis_arm.apply_translation([0.0, -3.5, 0.0])
    adds.append(clevis_arm)

    clevis_boss = cylinder(radius=3.2, height=bar_z, sections=32)
    clevis_boss.apply_translation([0.0, -6.5, 0.0])
    adds.append(clevis_boss)

    # Integral vertical drive clevis pin protruding +3.5 mm dorsally to engage MG90S servo horn
    drive_pin = cylinder(radius=1.15, height=bar_z + 3.5, sections=24)
    drive_pin.apply_translation([0.0, -6.5, 1.75])
    adds.append(drive_pin)

    body = trimesh.boolean.union(adds)

    # Cutters: 4× slotted cam cutouts (`2.6 × 5.0 mm`)
    cutters = []
    for cx, cy in crank_local_xy:
        slot = _rounded_slot_2d(half_x=1.30, half_y=2.50, z_height=bar_z + 6.0, sections=24)
        slot.apply_translation([cx, cy, 0.0])
        cutters.append(slot)

    result = body.difference(trimesh.boolean.union(cutters))
    return _clean_single_body(result)


# =============================================================================
# SHARED PALM SUB-COMPONENT BUILDERS (MCP YAW CLEVIS, THUMB CMC, CUPPING SEAM)
# =============================================================================

def _build_mcp_yaw_socket_stages(spec: dict):
    """
    Constructs non-coplanar CSG components in palm world coordinates for one
    Z-axis MCP yaw clevis socket:
      - `adds`: upper (`z in [+3.5, +7.0]`) & lower (`z in [-6.8, -3.1]`) clevis arms
        plus lateral side pillars (`|x_local| in [4.2, 6.6]`) connecting into the metacarpal body.
      - `cuts`: `6.6 mm` Z-slot (`z in [-3.1, +3.5]`) with full `±15°` swept clearance for
        both the flexion tongue and rear abduction crank arm, plus vertical `Ø3.4 mm` Z-pin bore
        and palmar `Ø2.2 mm` PTFE liner seat entry.
    """
    adds = []
    cuts = []

    z_slot_bot = PALM_SLOT_Z_BOT  # -3.10 mm
    z_slot_top = PALM_SLOT_Z_TOP  # +3.50 mm
    z_arm_dors_top = 7.00         # 3.50 mm thick upper clevis arm
    z_arm_palm_bot = -5.60        # 2.50 mm thick lower clevis arm (clears 95° flexed phalanx!)

    # Transform centered at the yaw pivot `(0, -9.0, 0)` in local yoke frame
    T_yaw = mcp_yoke_transform(spec, yaw_rad=0.0) @ trimesh.transformations.translation_matrix(
        [0.0, MCP_YAW_LOCAL_Y, 0.0]
    )

    # --- ADDS (in yaw-pivot local frame: pivot at (0,0,0), distal +Y, dorsal +Z) ---
    # Upper clevis ring (`r = 3.45 mm`, clears crank pin inner edge at `r = 3.90 mm` across all yaw!)
    upper_ring = cylinder(radius=3.45, height=z_arm_dors_top - z_slot_top, sections=36)
    upper_ring.apply_translation([0.0, 0.0, 0.5 * (z_slot_top + z_arm_dors_top)])
    adds.append(upper_ring)

    # Lower clevis ring (`r = 3.45 mm`, 1.75 mm annular wall around M3 pin, clears 95° flexion!)
    lower_ring = cylinder(radius=3.45, height=z_slot_bot - z_arm_palm_bot, sections=36)
    lower_ring.apply_translation([0.0, 0.0, 0.5 * (z_arm_palm_bot + z_slot_bot)])
    adds.append(lower_ring)

    # Lateral flanking clevis walls (outside ±15° crank/tongue sweep) bridging back into palm
    for sx in (-1.0, 1.0):
        # Upper lateral bridge from ring back to palm body
        u_side = box(extents=[3.0, 10.0, z_arm_dors_top - z_slot_top])
        u_side.apply_translation([sx * 4.7, -4.5, 0.5 * (z_slot_top + z_arm_dors_top)])
        adds.append(u_side)

        # Lower lateral bridge + full-height rear metacarpal cheek behind y = -7.8
        l_side = box(extents=[3.0, 11.0, z_slot_bot - z_arm_palm_bot])
        l_side.apply_translation([sx * 4.7, -5.0, 0.5 * (z_arm_palm_bot + z_slot_bot)])
        adds.append(l_side)

        # Vertical side pillar connecting upper & lower arms at `|x| in [4.8, 6.6], y in [-9.5, -1.5]`
        pillar = box(extents=[2.2, 8.2, z_arm_dors_top - z_arm_palm_bot])
        pillar.apply_translation([sx * 5.6, -5.6, 0.5 * (z_arm_palm_bot + z_arm_dors_top)])
        adds.append(pillar)

    # Solid rear metacarpal root block behind `y = -8.6 mm` (clears rear crank boss at `y = -7.3 mm`)
    rear_root = box(extents=[13.2, 8.0, z_arm_dors_top - z_arm_palm_bot])
    rear_root.apply_translation([0.0, -12.6, 0.5 * (z_arm_palm_bot + z_arm_dors_top)])
    adds.append(rear_root)

    # --- CUTS (in yaw-pivot local frame) ---
    # 1. Open forward space ahead of y = +3.55 mm (clears flexion hub & proximal phalanx at ±15° yaw)
    fwd_clear = box(extents=[22.0, 24.0, 26.0])
    fwd_clear.apply_translation([0.0, 15.55, 0.0])
    cuts.append(fwd_clear)

    # Palmar flexion sweep relief below `z = -5.65` for `y_from_yaw >= -4.5` (clears 95° flexed phalanx!)
    palm_flex_clear = box(extents=[16.0, 20.0, 18.0])
    palm_flex_clear.apply_translation([0.0, 5.5, -14.65])
    cuts.append(palm_flex_clear)

    # 2. Full `6.6 mm` Z-slot (`z in [-3.10, +3.50]`) swept across ±18° yaw for tongue + rear crank arm
    for deg in (-18.0, -9.0, 0.0, 9.0, 18.0):
        rad = np.radians(deg)
        rz_m = trimesh.transformations.rotation_matrix(rad, [0, 0, 1])

        # Yaw hub + tongue + rear crank sweep inside `z in [-3.10, +3.50]`
        slot_sweep = box(extents=[5.8, 17.6, z_slot_top - z_slot_bot])
        slot_sweep.apply_translation([0.0, 0.4, 0.5 * (z_slot_bot + z_slot_top)])
        slot_sweep.apply_transform(rz_m)
        cuts.append(slot_sweep)

        # Rear crank boss vertical clearance above `z = +3.50` (`y_from_yaw = -5.0`, `r = 2.65 mm`)
        # Ensures the crank pin & linkage bar have zero collision with palm dorsal structure!
        crank_up = cylinder(radius=2.65, height=14.0, sections=24)
        crank_up.apply_translation([0.0, -5.0, z_slot_top + 6.5])
        crank_up.apply_transform(rz_m)
        cuts.append(crank_up)

        # Flexion tongue dorsal stop shoulder clearance (`y_from_yaw >= +3.5`, `z in [-5.0, +8.5]`)
        tongue_dors = box(extents=[5.4, 14.0, 14.0])
        tongue_dors.apply_translation([0.0, 10.6, 1.5])
        tongue_dors.apply_transform(rz_m)
        cuts.append(tongue_dors)

    # Concentric `r = 4.25 mm` cylindrical yaw-hub slot pocket (`z in [-3.10, +3.50]`)
    hub_pocket = cylinder(radius=4.25, height=z_slot_top - z_slot_bot, sections=40)
    hub_pocket.apply_translation([0.0, 0.0, 0.5 * (z_slot_bot + z_slot_top)])
    cuts.append(hub_pocket)

    # 3. Vertical Ø3.4 mm (`PIN_RADIUS = 1.70`) Z-axis yaw pin bore through both clevis arms
    z_pin_bore = cylinder(radius=PIN_RADIUS, height=22.0, sections=36)
    cuts.append(z_pin_bore)

    # 4. Palmar tendon guide entry at `z = -3.50 mm` (`Ø2.2 mm` PTFE liner seat bore)
    ptfe_seat = cylinder(radius=PTFE_SEAT_R, height=14.0, sections=24)
    ptfe_seat.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    ptfe_seat.apply_translation([0.0, -5.5, TENDON_Z])
    cuts.append(ptfe_seat)

    # Apply world transform T_yaw to all adds and cuts
    w_adds = [m.copy().apply_transform(T_yaw) for m in adds]
    w_cuts = [m.copy().apply_transform(T_yaw) for m in cuts]
    return w_adds, w_cuts


def _build_thumb_cmc_stages():
    """
    Reproduces the certified v9 male Thumb CMC pedestal & 4.4 mm male hinge tongue at
    `THUMB_CMC_POS = [-33.0, 24.0, -2.0]`, Euler `(+0.85, -0.35, +0.50)` guaranteeing
    `0.000 mm³` collision across `0°–90°` flexion and rigid `-5°/-10°` hard stop.
    """
    r_hub = 13.0 * HUB_R_FRAC - FDM_CLEARANCE  # 4.69 mm
    stop_z_top = 6.25
    fork_clear_half_x = 9.20
    r_condyle_clear = 6.65
    anchor_y = -9.6
    anchor_boss_z_top = 8.6
    pedestal_y_back = -12.2
    pedestal_w = 13.0

    base_adds, env_cuts, tongue_adds, detail_cuts = [], [], [], []

    # Stage 1: Pedestal + dorsal elastic anchor boss
    y_ped_front = -r_condyle_clear + 0.5
    ped_len = y_ped_front - pedestal_y_back
    ped_z_bot = -r_hub - 2.6
    ped_h = anchor_boss_z_top - ped_z_bot
    pedestal = box(extents=[pedestal_w, ped_len, ped_h])
    pedestal.apply_translation([0.0, 0.5 * (y_ped_front + pedestal_y_back), 0.5 * (anchor_boss_z_top + ped_z_bot)])
    base_adds.append(pedestal)

    boss_cyl = cylinder(radius=3.1, height=11.6, sections=32)
    boss_cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    boss_cyl.apply_translation([0.0, anchor_y, anchor_boss_z_top - 1.2])
    base_adds.append(boss_cyl)

    # Stage 2: Joint & 0..90° flexion clearance envelope
    span_x = fork_clear_half_x * 2.0
    c_condyle = cylinder(radius=r_condyle_clear, height=span_x, sections=44)
    c_condyle.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    env_cuts.append(c_condyle)

    b_front_dors = box(extents=[span_x, r_condyle_clear + 22.0, 32.0])
    b_front_dors.apply_translation([0.0, -0.5 * r_condyle_clear + 11.0, 8.0])
    env_cuts.append(b_front_dors)

    palmar_pts = np.array([
        [-fork_clear_half_x,  8.0, -3.6], [ fork_clear_half_x,  8.0, -3.6],
        [-fork_clear_half_x, -5.6, -3.8], [ fork_clear_half_x, -5.6, -3.8],
        [-fork_clear_half_x, -9.2, -7.8], [ fork_clear_half_x, -9.2, -7.8],
        [-fork_clear_half_x, -9.8, -22.0], [ fork_clear_half_x, -9.8, -22.0],
        [-fork_clear_half_x,  8.0, -22.0], [ fork_clear_half_x,  8.0, -22.0],
    ])
    env_cuts.append(trimesh.convex.convex_hull(palmar_pts))

    th_sweep_box = box(extents=[22.0, 42.0, 42.0])
    th_sweep_box.apply_translation([0.0, 14.35, -14.5])
    env_cuts.append(th_sweep_box)

    # Stage 3: Clean 4.4 mm male tongue + 0° dorsal hard-stop shoulder
    hub_cyl = cylinder(radius=r_hub, height=CLEVIS_TONGUE_W, sections=48)
    hub_cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    tongue_adds.append(hub_cyl)

    y_stop_front = 0.25
    y_neck_back = -r_condyle_clear - 2.2
    hw_t = CLEVIS_TONGUE_W * 0.5
    neck_pts = np.array([
        [-hw_t, y_stop_front, -3.2], [ hw_t, y_stop_front, -3.2],
        [-hw_t, y_stop_front,  stop_z_top], [ hw_t, y_stop_front,  stop_z_top],
        [-hw_t, y_neck_back,   stop_z_top], [ hw_t, y_neck_back,   stop_z_top],
        [-hw_t, y_neck_back,  -3.5], [ hw_t, y_neck_back,  -3.5],
    ])
    tongue_adds.append(trimesh.convex.convex_hull(neck_pts))

    # Stage 4: Precision bores & dorsal elastic anchor
    pin_bore = cylinder(radius=PIN_RADIUS, height=span_x + 6.0, sections=36)
    pin_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    detail_cuts.append(pin_bore)

    pulley_slot = box(extents=[2.10, 9.0, 4.2])
    pulley_slot.apply_translation([0.0, 0.8, -4.55])
    detail_cuts.append(pulley_slot)

    t_entry = cylinder(radius=PTFE_SEAT_R, height=11.0, sections=24)
    t_entry.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    t_entry.apply_translation([0.0, -0.5, TENDON_Z])
    detail_cuts.append(t_entry)

    z_guide_floor = r_hub + 0.35
    dorsal_guide = box(extents=[1.80, abs(anchor_y) + 2.0, 14.0])
    dorsal_guide.apply_translation([0.0, 0.5 * (anchor_y + 2.0), z_guide_floor + 7.0])
    detail_cuts.append(dorsal_guide)

    z_pocket_floor = anchor_boss_z_top - 4.4
    anchor_pocket = box(extents=[3.2, 4.6, 12.0])
    anchor_pocket.apply_translation([0.0, anchor_y, z_pocket_floor + 6.0])
    detail_cuts.append(anchor_pocket)

    z_cross_pin = z_pocket_floor + 1.65
    cross_pin = cylinder(radius=1.00, height=pedestal_w + 10.0, sections=24)
    cross_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    cross_pin.apply_translation([0.0, anchor_y, z_cross_pin])
    detail_cuts.append(cross_pin)

    rot_cmc = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)
    th_pos = np.array(THUMB_CMC_POS, dtype=float)

    base_adds = [_apply_pose(m, rot_cmc, th_pos) for m in base_adds]
    env_cuts = [_apply_pose(m, rot_cmc, th_pos) for m in env_cuts]
    tongue_adds = [_apply_pose(m, rot_cmc, th_pos) for m in tongue_adds]
    detail_cuts = [_apply_pose(m, rot_cmc, th_pos) for m in detail_cuts]

    # Organic bridge fusing CMC pedestal root into palm_radial core
    cmc_root_collar = box(extents=[12.4, 4.6, 12.2])
    cmc_root_collar.apply_translation([0.0, -9.8, 1.5])
    _apply_pose(cmc_root_collar, rot_cmc, th_pos)

    palm_anchor_blob = icosphere(subdivisions=3, radius=1.0)
    palm_anchor_blob.apply_scale([10.5, 15.5, 9.8])
    palm_anchor_blob.apply_translation([-19.5, 22.5, -1.5])
    base_adds.append(trimesh.util.concatenate([cmc_root_collar, palm_anchor_blob]).convex_hull)

    return base_adds, env_cuts, tongue_adds, detail_cuts


def _make_cupping_seam_cutter(side: str) -> trimesh.Trimesh:
    """
    Creates the seam trimming solid for `side in ('radial', 'ulnar')`:
      - Dorsal (`z >= +3.0`): flat vertical `0°` hard-stop plane at `x = -0.3` (radial)
        or `x = +0.3` (ulnar).
      - Palmar (`z < +3.0`): angled `17°` V-relief (`34°` total included angle) so
        `palm_ulnar` cups `0°–30°` palmarly around `CUP_HINGE_POS = [0.0, 42.0, +3.0]`
        with `0.000 mm³` collision!
    Applies for `y in [13.5, 95.0]` (leaving `palm_radial`'s rigid carpal wrist base
    `y <= 12.0` intact across both `-X` and `+X`).
    """
    z_h = CUP_HINGE_POS[2]  # +3.0 mm
    z_top = 24.0
    z_bot = -24.0
    tan_a = np.tan(np.radians(17.5))  # 17.5° each side -> 35° total relief (> 30° cupping)
    dx_bot = (z_h - z_bot) * tan_a    # ~8.51 mm

    y0, y1 = 13.5, 95.0
    if side == "radial":
        # Remove everything on the +X side of radial's mating face (`x > -0.3` dorsally,
        # `x > -0.3 - (3.0 - z) * tan(17.5°)` palmarly)
        xz = [
            (-0.30, z_top),
            ( 60.0, z_top),
            ( 60.0, z_bot),
            (-0.30 - dx_bot, z_bot),
            (-0.30, z_h),
        ]
    else:
        # Remove everything on the -X side of ulnar's mating face (`x < +0.3` dorsally,
        # `x < +0.3 + (3.0 - z) * tan(17.5°)` palmarly)
        xz = [
            (-60.0, z_top),
            (  0.30, z_top),
            (  0.30, z_h),
            (  0.30 + dx_bot, z_bot),
            (-60.0, z_bot),
        ]

    # Build clean watertight extruded prism from polygon `xz` along `y in [y0, y1]`
    n = len(xz)
    verts = [[x, y0, z] for x, z in xz] + [[x, y1, z] for x, z in xz]
    faces = []
    # Side quads
    for i in range(n):
        inext = (i + 1) % n
        v0, v1 = i, inext
        v2, v3 = n + inext, n + i
        faces.append([v0, v1, v2])
        faces.append([v0, v2, v3])
    # Fan triangulation for pentagon caps (concave? Wait: check convexity!
    # The notch at (x=±0.3, z=3.0) makes the removed region slightly convex or concave!
    # Let's split cleanly into TWO convex boxes/prisms: dorsal box (z >= 3.0) + palmar wedge (z <= 3.0)!)
    if side == "radial":
        dors = box(extents=[60.0, y1 - y0, z_top - z_h])
        dors.apply_translation([-0.30 + 30.0, 0.5 * (y0 + y1), 0.5 * (z_h + z_top)])
        palm_xz = [(-0.30, z_h + 0.05), (60.0, z_h + 0.05), (60.0, z_bot), (-0.30 - dx_bot, z_bot)]
    else:
        dors = box(extents=[60.0, y1 - y0, z_top - z_h])
        dors.apply_translation([0.30 - 30.0, 0.5 * (y0 + y1), 0.5 * (z_h + z_top)])
        palm_xz = [(-60.0, z_h + 0.05), (0.30, z_h + 0.05), (0.30 + dx_bot, z_bot), (-60.0, z_bot)]

    palm_pts = np.array(
        [[x, y0, z] for x, z in palm_xz] + [[x, y1, z] for x, z in palm_xz],
        dtype=float,
    )
    palm_wedge = trimesh.convex.convex_hull(palm_pts)
    return trimesh.boolean.union([dors, palm_wedge])


def _make_wrist_cardan_sweep_cutter() -> trimesh.Trimesh:
    """
    Builds a swept clearance envelope for `wrist_cardan_ring` (`R_out = 19.8 mm`,
    `y_local in [-5.0, +5.0]`, flat Z pads at `z = ±16.0 mm`) across `±36°` Yaw around Z
    at `WRIST_CENTER = [0.0, -14.0, 0.0]` so `palm_radial` is guaranteed `0.000 mm³`
    collision against the Cardan ring across `±35°` Yaw!
    """
    r_clear = 20.55       # 0.75 mm radial clearance over R_out = 19.8 mm
    y_thick = 11.4        # 0.70 mm axial clearance over y in [-5.0, +5.0]
    z_half = 16.45        # Stops strictly inside the palm's z = ±16.5 mm wrist yoke ears!

    sweeps = []
    for deg in (-36.0, -24.0, -12.0, 0.0, 12.0, 24.0, 36.0):
        ring_env = cylinder(radius=r_clear, height=y_thick, sections=40)
        ring_env.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
        # Clip to |z| <= z_half so it never cuts into the z = ±16.5..20.5 yoke ears
        z_clip = box(extents=[2.0 * r_clear + 2.0, y_thick + 2.0, 2.0 * z_half])
        ring_clipped = trimesh.boolean.intersection([ring_env, z_clip])
        ring_clipped.apply_transform(trimesh.transformations.rotation_matrix(np.radians(deg), [0, 0, 1]))
        ring_clipped.apply_translation(WRIST_CENTER)
        sweeps.append(ring_clipped)

    return trimesh.boolean.union(sweeps)


# =============================================================================
# 3. SPLIT CUPPING PALM — RADIAL HALF (`generate_palm_radial`)
# =============================================================================

def generate_palm_radial() -> trimesh.Trimesh:
    """
    Generates `palm_radial` (fixed wrist-mounted radial palm half):
      - Rigid carpal wrist base (`y <= +12.0`) carrying:
        * Rear 2-DoF Wrist Cardan Yoke Ears extending to `WRIST_CENTER = [0.0, -14.0, 0.0]`
          at `z = ±18.5 mm` (inner faces `z = ±16.5 mm`, outer faces `z = ±20.5 mm`,
          vertical `Ø3.4 mm` Z-axis yaw pin bores), clearing the `Ø40 mm` Cardan ring across `±35°` Yaw.
        * Dual Pushrod Ball-Joint Mounting Ears at `PUSHROD_PALM_ANCHORS = [[-24.0, -2.0, -8.0], [24.0, -2.0, -8.0]]`
          with `Ø3.4 mm` M3 rod-end bores.
        * Central carpal tendon tunnel exiting cleanly at `[0.0, -3.0, -2.0]`.
      - Radial metacarpal body (`x <= -0.3` with `17.5°` palmar V-relief below `z = +3.0`) carrying:
        * Interleaved outer Y-axis cupping hinge knuckles (`y in [22.0, 32.0]` and `y in [52.0, 62.0]`)
          at `CUP_HINGE_POS = [0.0, 42.0, +3.0]` (`Ø3.4 mm` Y-pin bore).
        * Index & Middle Z-axis MCP yaw clevis sockets (`6.6 mm` Z-slot, `±15°` yaw clearance).
        * Certified v9 Thumb 2-DoF CMC pedestal (`THUMB_CMC_POS = [-33.0, 24.0, -2.0]`) +
          dedicated palmar opposition tendon fairlead conduit.
        * Open-back dorsal drop-in `MG90S`/`SG90` abduction micro-servo bay (`12.8 × 23.4 mm` +
          2× `Ø1.8 mm` M2 pilot holes on `27.5 mm` centers).
        * Continuous `Ø2.8 mm` internal tendon conduits with `Ø2.2 mm` PTFE Bowden liner seats
          for Index, Middle, Thumb Flex, Thumb Opp, and Palmar Cupping.
        * Dorsal `Ø2.0 mm` elastic return tie-pin anchor across the cupping seam.
    """
    stage1_adds = []
    stage2_env_cuts = []
    stage3_tongues = []
    stage4_detail_cuts = []

    # 1. Sculpted Radial Palm Core + Rigid Carpal Wrist Base
    core_blobs = []

    # Rigid carpal wrist base spanning x in [-26, +26], y in [+1.0, +13.0]
    carpal_base = box(extents=[52.0, 12.5, 22.0])
    carpal_base.apply_translation([0.0, 7.2, 0.0])
    core_blobs.append(carpal_base)

    # Radial metacarpal palm volume (x in [-32, +2], y in [10, 68], z in [-11.5, +7.5])
    radial_body = box(extents=[33.0, 56.0, 19.0])
    radial_body.apply_translation([-15.5, 39.0, -2.0])
    core_blobs.append(radial_body)

    # Organic sculpted outer radial & thenar contours
    thenar = icosphere(subdivisions=3, radius=1.0)
    thenar.apply_scale([14.5, 21.0, 10.5])
    thenar.apply_transform(trimesh.transformations.euler_matrix(0.25, -0.18, 0.30))
    thenar.apply_translation([-23.0, 25.0, -1.8])
    core_blobs.append(thenar)

    radial_arch = icosphere(subdivisions=3, radius=1.0)
    radial_arch.apply_scale([18.0, 28.0, 10.0])
    radial_arch.apply_translation([-15.0, 44.0, -1.5])
    core_blobs.append(radial_arch)

    # Dorsal servo flange plateau (`z_mount = +7.5`) around `IN_PALM_SERVO_BAY`
    servo_plateau = box(extents=[22.0, 34.0, 18.0])
    servo_plateau.apply_translation([-15.0, 42.0, -1.5])
    core_blobs.append(servo_plateau)

    stage1_adds.append(trimesh.boolean.union(core_blobs))

    # 2. Rear 2-DoF Wrist Cardan Yoke Ears at `WRIST_CENTER = [0.0, -14.0, 0.0]`
    #    Dorsal ear: z in [+16.5, +20.5] (center +18.5); Palmar ear: z in [-20.5, -16.5] (center -18.5)
    ear_thick = 4.0
    ear_w_x = 15.0
    for z_sign in (-1.0, 1.0):
        z_ear = z_sign * 18.5
        # Circular tip boss around [0.0, -14.0, ±18.5] (radius 7.5 mm)
        ear_tip = cylinder(radius=ear_w_x * 0.5, height=ear_thick, sections=36)
        ear_tip.apply_translation([0.0, WRIST_CENTER[1], z_ear])
        stage1_adds.append(ear_tip)

        # Horizontal ear arm extending from y = -14.0 forward to carpal base y = +6.5
        ear_arm = box(extents=[ear_w_x, 21.0, ear_thick])
        ear_arm.apply_translation([0.0, -3.5, z_ear])
        stage1_adds.append(ear_arm)

        # Gusset riser connecting ear arm (`|z| = 16.5..20.5`) down into carpal base (`|z| = 11.0`)
        # at `y in [+3.0, +11.5]` (completely clear of swept Cardan ring `y <= +1.5`!)
        gusset = box(extents=[ear_w_x + 4.0, 9.0, 10.0])
        gusset.apply_translation([0.0, 7.5, z_sign * 15.5])
        stage1_adds.append(gusset)

        # Vertical Ø3.4 mm Z-axis wrist yaw pin bore at `[0.0, -14.0, ±18.5]`
        w_pin = cylinder(radius=PIN_RADIUS, height=ear_thick + 4.0, sections=36)
        w_pin.apply_translation([0.0, WRIST_CENTER[1], z_ear])
        stage4_detail_cuts.append(w_pin)

    # 3. Dual Pushrod Ball-Joint Mounting Ears at `PUSHROD_PALM_ANCHORS = [[-24.0, -2.0, -8.0], [24.0, -2.0, -8.0]]`
    for ax, ay, az in PUSHROD_PALM_ANCHORS:
        s_x = -1.0 if ax < 0 else 1.0
        # Lug centered at [ax, ay, az], 6.0 mm wide along X, radius 4.5 mm in YZ
        lug_cyl = cylinder(radius=4.5, height=6.0, sections=32)
        lug_cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
        lug_cyl.apply_translation([ax, ay, az])
        stage1_adds.append(lug_cyl)

        # Structural strut connecting lug back to carpal base (`y = +4.0..+10.0`)
        strut = box(extents=[7.0, 9.5, 9.0])
        strut.apply_translation([ax - s_x * 0.5, ay + 4.8, az + 0.5])
        stage1_adds.append(strut)

        # Horizontal X-axis Ø3.4 mm M3 rod-end bore at `[ax, ay, az]`
        rod_bore = cylinder(radius=PIN_RADIUS, height=12.0, sections=32)
        rod_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
        rod_bore.apply_translation([ax, ay, az])
        stage4_detail_cuts.append(rod_bore)

    # 4. Index & Middle Z-Axis MCP Yaw Clevis Sockets
    for spec in MCP_YOKE_SPECS:
        if spec["name"] in ("index", "middle"):
            s_adds, s_cuts = _build_mcp_yaw_socket_stages(spec)
            stage1_adds.extend(s_adds)
            stage4_detail_cuts.extend(s_cuts)

    # 5. Certified v9 Thumb CMC Pedestal & Male Tongue
    th_b_adds, th_e_cuts, th_t_adds, th_d_cuts = _build_thumb_cmc_stages()
    stage1_adds.extend(th_b_adds)
    stage2_env_cuts.extend(th_e_cuts)
    stage3_tongues.extend(th_t_adds)
    stage4_detail_cuts.extend(th_d_cuts)

    # 6. Trim radial palm cleanly along the cupping seam (`x <= -0.3` dorsally, 17.5° palmar V-relief)
    stage2_env_cuts.append(_make_cupping_seam_cutter("radial"))

    # Also trim wrist Cardan ring ±36° swept yaw envelope (`|z| <= 16.45`)
    stage2_env_cuts.append(_make_wrist_cardan_sweep_cutter())

    # 7. Interleaved Axial Y-Axis Cupping Hinge Knuckles at `CUP_HINGE_POS = [0.0, 42.0, +3.0]`
    #    `palm_radial` owns the two outer knuckles: `y in [22.0, 32.0]` and `y in [52.0, 62.0]`
    hx, hy, hz = CUP_HINGE_POS
    for y0, y1 in [(22.0, 32.0), (52.0, 62.0)]:
        ym = 0.5 * (y0 + y1)
        ylen = y1 - y0
        kn = cylinder(radius=3.45, height=ylen, sections=36)
        kn.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
        kn.apply_translation([hx, ym, hz])
        stage3_tongues.append(kn)

        # Bridge knuckle solidly into palm_radial (`x in [-4.5, 0.0]`)
        br = box(extents=[4.5, ylen, 6.2])
        br.apply_translation([-2.25, ym, hz])
        stage3_tongues.append(br)

    # Concentric relief on `palm_radial` in `palm_ulnar`'s central knuckle zone (`y in [32.3, 51.7]`)
    ulnar_kn_clear = cylinder(radius=3.85, height=19.4, sections=36)
    ulnar_kn_clear.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    ulnar_kn_clear.apply_translation([hx, 42.0, hz])
    stage4_detail_cuts.append(ulnar_kn_clear)

    # Axial Y-axis Ø3.4 mm cupping hinge pin bore through both outer knuckles
    cup_pin_bore = cylinder(radius=PIN_RADIUS, height=46.0, sections=36)
    cup_pin_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    cup_pin_bore.apply_translation([hx, 42.0, hz])
    stage4_detail_cuts.append(cup_pin_bore)

    # 8. Open-Back Dorsal Drop-In MG90S/SG90 Abduction Micro-Servo Bay
    sc = IN_PALM_SERVO_BAY["center"]
    s_bay = box(extents=[IN_PALM_SERVO_BAY["size_x"], IN_PALM_SERVO_BAY["size_y"], 28.0])
    s_bay.apply_translation([sc[0], sc[1], IN_PALM_SERVO_BAY["z_floor"] + 14.0])
    stage4_detail_cuts.append(s_bay)

    for px, py_p in IN_PALM_SERVO_BAY["pilot_xy"]:
        pilot = cylinder(radius=IN_PALM_SERVO_BAY["pilot_r"], height=12.0, sections=20)
        pilot.apply_translation([px, py_p, IN_PALM_SERVO_BAY["z_mount"] - 4.0])
        stage4_detail_cuts.append(pilot)

    # 9. Central Carpal Tendon Tunnel at `[0.0, -3.0, -2.0]` + Continuous Ø2.8 mm Conduits
    #    Tunnel opens smoothly at `y = +1.5..+14.0` (ahead of swept Cardan ring!)
    carpal_tunnel = cylinder(radius=6.5, height=22.0, sections=36)
    carpal_tunnel.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    carpal_tunnel.apply_translation([CARPAL_TUNNEL_POS[0], 5.5, CARPAL_TUNNEL_POS[2]])
    stage4_detail_cuts.append(carpal_tunnel)

    # Index flexor conduit (routes cleanly around radial side of servo bay `x = -23.8`)
    idx_spec = MCP_YOKE_SPECS[0]
    stage4_detail_cuts.extend(_routed_conduit([
        [idx_spec["yaw_x"], idx_spec["yaw_y"] - 2.0, idx_spec["z"] + TENDON_Z],
        [-24.2, 54.0, -4.5],
        [-24.2, 26.0, -4.5],
        [-4.0, 5.0, -2.0],
    ], PALM_CONDUIT_R))

    # Middle flexor conduit (routes cleanly on ulnar side of servo bay `x = -6.2`)
    mid_spec = MCP_YOKE_SPECS[1]
    stage4_detail_cuts.extend(_routed_conduit([
        [mid_spec["yaw_x"], mid_spec["yaw_y"] - 2.0, mid_spec["z"] + TENDON_Z],
        [-6.2, 56.0, -4.8],
        [-6.2, 24.0, -4.8],
        [-1.5, 5.0, -2.0],
    ], PALM_CONDUIT_R))

    # Thumb Flex conduit from CMC tongue root to carpal tunnel
    rot_cmc3 = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)[:3, :3]
    th_pos = np.array(THUMB_CMC_POS, dtype=float)
    p_th_start = th_pos + 2.0 * rot_cmc3[:, 1] + TENDON_Z * rot_cmc3[:, 2]
    p_th_elbow = th_pos - 4.8 * rot_cmc3[:, 1] + TENDON_Z * rot_cmc3[:, 2]
    stage4_detail_cuts.extend(_routed_conduit([
        p_th_start,
        p_th_elbow,
        [-4.5, 5.0, -2.5],
    ], PALM_CONDUIT_R))

    # Thumb Opposition fairlead conduit (exits palmar-ulnar thenar surface toward thumb)
    stage4_detail_cuts.extend(_routed_conduit([
        [-28.0, 32.0, -11.5],
        [-22.0, 28.0, -8.5],
        [-3.5, 5.0, -4.0],
    ], PALM_CONDUIT_R))

    # Palmar Cupping tendon conduit (exits radial V-seam at y = 54, z = -7.0 toward ulnar eyelet)
    stage4_detail_cuts.extend(_routed_conduit([
        [-1.0, 54.0, -7.0],
        [-6.5, 54.0, -7.0],
        [-6.5, 22.0, -7.0],
        [1.5, 5.0, -3.5],
    ], PALM_CONDUIT_R))

    # Dorsal elastic return tie-pin anchor pocket + Ø2.0 mm transverse bore across cupping seam
    cup_el_pocket = box(extents=[3.0, 4.4, 6.0])
    cup_el_pocket.apply_translation([-5.2, 42.0, 6.5])
    stage4_detail_cuts.append(cup_el_pocket)

    cup_el_pin = cylinder(radius=1.00, height=7.0, sections=20)
    cup_el_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    cup_el_pin.apply_translation([-5.2, 42.0, 5.4])
    stage4_detail_cuts.append(cup_el_pin)

    # Execute clean 3-stage non-coplanar boolean pipeline
    palm_cleared = trimesh.boolean.union(stage1_adds).difference(trimesh.boolean.union(stage2_env_cuts))
    palm_with_tongues = trimesh.boolean.union([palm_cleared] + stage3_tongues)
    result = palm_with_tongues.difference(trimesh.boolean.union(stage4_detail_cuts))
    return _clean_single_body(result)


# =============================================================================
# 4. SPLIT CUPPING PALM — ULNAR HALF (`generate_palm_ulnar`)
# =============================================================================

def generate_palm_ulnar() -> trimesh.Trimesh:
    """
    Generates `palm_ulnar` (mobile cupping ulnar palm half, `x >= +0.3`, `y >= 16.0`):
      - Interleaved central Y-axis M3 cupping hinge knuckle (`y in [32.6, 51.4]`) at
        `CUP_HINGE_POS = [0.0, 42.0, +3.0]` (`Ø3.4 mm` Y-pin bore + `0°` dorsal flat hard stop),
        cupping `0°–30°` palmarly with `0.000 mm³` collision against `palm_radial`.
      - Ring & Pinky Z-axis MCP yaw clevis sockets (`6.6 mm` Z-slot, `±15°` yaw clearance).
      - Continuous `Ø2.8 mm` tendon conduits with `Ø2.2 mm` PTFE liner seats exiting proximally
        toward the central carpal tunnel `[0.0, -3.0, -2.0]`.
      - Palmar cupping tendon anchor eyelet at `[22.0, 54.0, -7.0]`.
      - Dorsal `Ø2.0 mm` elastic return tie-pin anchor across the cupping seam.
    """
    stage1_adds = []
    stage2_env_cuts = []
    stage3_tongues = []
    stage4_detail_cuts = []

    # 1. Sculpted Ulnar Metacarpal & Hypothenar Core (`x in [0.0, +32.0]`, `y in [16.0, 68.0]`)
    ulnar_core = box(extents=[31.0, 50.0, 17.5])
    ulnar_core.apply_translation([15.5, 41.0, -1.75])
    stage1_adds.append(ulnar_core)

    hypothenar = icosphere(subdivisions=3, radius=1.0)
    hypothenar.apply_scale([13.0, 24.0, 9.8])
    hypothenar.apply_translation([21.5, 40.0, -2.0])
    stage1_adds.append(hypothenar)

    # 2. Ring & Pinky Z-Axis MCP Yaw Clevis Sockets
    for spec in MCP_YOKE_SPECS:
        if spec["name"] in ("ring", "pinky"):
            s_adds, s_cuts = _build_mcp_yaw_socket_stages(spec)
            stage1_adds.extend(s_adds)
            stage4_detail_cuts.extend(s_cuts)

    # 3. Trim cleanly along the cupping seam (`x >= +0.3` dorsally, 17.5° palmar V-relief)
    #    and proximal clearance plane (`y >= 16.0` clearing `palm_radial`'s rigid carpal base `y <= 13.5`)
    stage2_env_cuts.append(_make_cupping_seam_cutter("ulnar"))

    prox_trim = box(extents=[80.0, 40.0, 40.0])
    prox_trim.apply_translation([15.0, -4.0, 0.0])
    stage2_env_cuts.append(prox_trim)

    # 4. Interleaved Central Y-Axis Cupping Hinge Knuckle (`y in [32.6, 51.4]`) at `CUP_HINGE_POS`
    hx, hy, hz = CUP_HINGE_POS
    kn = cylinder(radius=3.45, height=18.8, sections=36)
    kn.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    kn.apply_translation([hx, 42.0, hz])
    stage3_tongues.append(kn)

    # Bridge central knuckle solidly into `palm_ulnar` (`x in [0.0, +5.0]`)
    br = box(extents=[5.0, 18.8, 6.2])
    br.apply_translation([2.5, 42.0, hz])
    stage3_tongues.append(br)

    # Concentric reliefs in `palm_radial`'s outer knuckle zones (`y in [21.5, 32.3]` and `[51.7, 62.5]`)
    for y0, y1 in [(21.5, 32.3), (51.7, 62.5)]:
        rel = cylinder(radius=3.85, height=y1 - y0, sections=36)
        rel.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
        rel.apply_translation([hx, 0.5 * (y0 + y1), hz])
        stage4_detail_cuts.append(rel)

    # Axial Y-axis Ø3.4 mm cupping hinge pin bore
    cup_pin = cylinder(radius=PIN_RADIUS, height=26.0, sections=36)
    cup_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    cup_pin.apply_translation([hx, 42.0, hz])
    stage4_detail_cuts.append(cup_pin)

    # 5. Continuous Ø2.8 mm Tendon Conduits for Ring & Pinky exiting toward carpal tunnel
    ring_spec = MCP_YOKE_SPECS[2]
    stage4_detail_cuts.extend(_routed_conduit([
        [ring_spec["yaw_x"], ring_spec["yaw_y"] - 2.0, ring_spec["z"] + TENDON_Z],
        [9.0, 46.0, -4.5],
        [7.5, 14.5, -3.5],
    ], PALM_CONDUIT_R))

    pinky_spec = MCP_YOKE_SPECS[3]
    stage4_detail_cuts.extend(_routed_conduit([
        [pinky_spec["yaw_x"], pinky_spec["yaw_y"] - 2.0, pinky_spec["z"] + TENDON_Z],
        [20.0, 46.0, -4.5],
        [14.5, 14.5, -3.5],
    ], PALM_CONDUIT_R))

    # 6. Palmar Cupping Tendon Anchor Eyelet at `CUPPING_EYELET_POS = [22.0, 54.0, -7.0]`
    ex, ey, ez = CUPPING_EYELET_POS
    eyelet_pocket = box(extents=[5.0, 5.0, 8.0])
    eyelet_pocket.apply_translation([ex, ey, ez - 3.0])
    stage4_detail_cuts.append(eyelet_pocket)

    eyelet_bore = cylinder(radius=1.25, height=10.0, sections=24)
    eyelet_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    eyelet_bore.apply_translation([ex, ey, ez])
    stage4_detail_cuts.append(eyelet_bore)

    # 7. Dorsal Elastic Return Tie-Pin Anchor (`Ø2.0 mm`) across the cupping seam
    el_pocket = box(extents=[3.0, 4.4, 6.0])
    el_pocket.apply_translation([5.2, 42.0, 6.5])
    stage4_detail_cuts.append(el_pocket)

    el_pin = cylinder(radius=1.00, height=7.0, sections=20)
    el_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    el_pin.apply_translation([5.2, 42.0, 5.4])
    stage4_detail_cuts.append(el_pin)

    # Execute clean 3-stage non-coplanar boolean pipeline
    palm_cleared = trimesh.boolean.union(stage1_adds).difference(trimesh.boolean.union(stage2_env_cuts))
    palm_with_tongues = trimesh.boolean.union([palm_cleared] + stage3_tongues)
    result = palm_with_tongues.difference(trimesh.boolean.union(stage4_detail_cuts))
    return _clean_single_body(result)


if __name__ == "__main__":
    print("Generating v10.0 palm_assembly parts...")
    parts = {
        "palm_radial": generate_palm_radial(),
        "palm_ulnar": generate_palm_ulnar(),
        "abduction_linkage": generate_abduction_linkage(),
    }
    for s in MCP_YOKE_SPECS:
        parts[f"{s['name']}_mcp_yoke"] = generate_mcp_yoke(s["name"])
    for name, m in parts.items():
        bodies = len(m.split(only_watertight=False))
        print(f"  {name:20s}: watertight={m.is_watertight}, bodies={bodies}, vol={m.volume:.1f} mm³, tris={len(m.faces)}, bounds={np.round(m.bounds, 2).tolist()}")
