#!/usr/bin/env python3
"""
Iteration 9 (v9.0) Parametric Digit & Phalanx CAD Module (`cad_v9.digits`)
==========================================================================
Fixes all phalanx and inter-phalangeal joint loopholes identified in v8:
1. Palmar Flexor Bore & Moment Arm (`TENDON_Z = -3.50 mm`, `TENDON_RADIUS = 1.10 mm`):
   - Relocates the Ø2.20 mm Dyneema tendon bore well below the Ø3.40 mm (`PIN_RADIUS = 1.70`)
     M3 hinge pin axis (`z = 0`), guaranteeing a >= 0.70 mm solid plastic bridge (`z in [-2.40, -1.70]`)
     with zero M3 bolt interference and nearly doubling the flexion moment arm to 3.50 mm.
   - Maintains >= 1.20 mm (>= 1.10 mm spec) of solid palmar floor beneath `z = -4.60 mm` along
     every phalanx shaft while providing a smooth 2.20 mm wide palmar tendon guide slot on
     distal male tongues and clean throat reliefs for 0°–95° collision-free flexion.
2. True Captive 6-Sided M3 Hex Nut Sockets & >= 1.25 mm Clamping Walls:
   - Replaces the circular nut pocket with a 6-sided hexagonal prism (`sections=6`, 5.65 mm
     across flats -> circumradius 3.262 mm) on the `+X` fork wall, locking M3 nut rotation.
   - Retains a Ø6.30 mm (`SCREW_HEAD_R = 3.15`) round socket/button-head counterbore on `-X`.
   - Widens narrow condyle hardware bosses locally where needed (e.g. pinky) so BOTH counterbore
     depth >= 1.85 mm AND solid clamping web >= 1.30 mm (>= 1.25 mm spec) hold simultaneously.
3. Profile-Following Dorsal Snap-Fit Pinch-Lip Channel + Clean Cuticle Termination:
   - Multi-station lofted undercut cutter (`w_mouth = 1.8 mm`, `w_cavity = 2.6 mm`, `lip_t = 0.50 mm`,
     45° self-supporting `0.50 mm` chamfer) whose `z_surf(y)` tracks the exact dorsal crown of
     each phalanx mesh.
   - On distal phalanges, overhanging pinch-lips terminate cleanly by `y = 0.55 * length`
     (prior to the `0.58 * length` fingernail cuticle), transitioning smoothly into an open
     `1.8 mm` dorsal slot across the `0.70 mm` fingernail bed through the Ø2.0 mm transverse
     elastic retention bore at `y = 0.78 * length`.
4. Distal Palmar Tendon Knot Pocket:
   - Recessed Ø4.0 mm × 2.5 mm enclosed axial counterbore inside the fingertip apex so a knotted
     Dyneema stopper bead seats flush inside the fingertip with zero grip-surface protrusion.
5. One-Directional 0° Extension Hard Stops & Consolidated Hinged-Phalanx Builder:
   - Rigid dorsal stop shoulders block `-5°` and `-10°` hyperextension (`> 0.15 mm³` overlap)
     while maintaining `0.000 mm³` collision across `0°–95°` flexion (`0°–90°` on Thumb IP).
   - Proximal and intermediate phalanges share a single parametric `_build_hinged_phalanx` builder.
"""

import numpy as np
import trimesh
from trimesh.creation import box, cylinder, icosphere

# =============================================================================
# Required Exported Mechanical & Geometric Constants
# =============================================================================

PIN_RADIUS = 1.70          # Ø3.40 mm clearance bore for M3 hinge pin
TENDON_RADIUS = 1.10       # Ø2.20 mm smooth bore for Ø0.8–1.0 mm braided Dyneema
TENDON_Z = -3.50           # Palmar tendon centerline Z (z in [-4.60, -2.40], clears M3 pin!)
CLEVIS_SLOT_W = 5.4        # Female clevis slot width (x in [-2.70, +2.70])
CLEVIS_TONGUE_W = 4.4      # Male clevis tongue width (x in [-2.20, +2.20])
FDM_CLEARANCE = 0.25       # Radial clearance on male hinge hubs
HUB_R_FRAC = 0.38          # Nominal hinge hub radius fraction of phalanx height
NAIL_DEPTH = 0.70          # Shallow 0.70 mm anatomical fingernail recess depth

# Hardware seat & snap-fit dorsal channel constants
NAIL_WIDTH_FRAC = 0.72
NAIL_Y_PROX_FRAC = 0.58    # Cuticle eponychium start fraction
SCREW_HEAD_R = 3.15        # Ø6.30 mm round M3 socket/button head counterbore (-X)
NUT_ACROSS_FLATS = 5.65    # M3 hex nut across-flats (5.50 mm ISO + 0.15 mm FDM clearance)
NUT_CIRCUMRADIUS = NUT_ACROSS_FLATS / np.sqrt(3.0)  # 3.2620 mm circumradius (sections=6)
MIN_CLAMP_WALL = 1.32      # Minimum solid clamping web (>= 1.25 mm requirement)
MIN_CB_DEPTH = 1.88        # Minimum hardware pocket depth (>= 1.80 mm requirement)

RB_MOUTH_W = 1.80          # Top pinch slit width (mm)
RB_CAVITY_W = 2.60         # Internal captive chamber width (mm)
RB_LIP_THICKNESS = 0.50    # Uniform profile-following top lip thickness (mm)
RB_CHAMFER_H = 0.50        # Self-supporting ~45° undercut chamfer height (mm)
RB_GROOVE_DEPTH = 2.55     # Total dorsal channel depth along mid-shaft (mm)

KNOT_POCKET_RADIUS = 2.00  # Ø4.0 mm distal palmar Dyneema knot pocket radius
KNOT_POCKET_DEPTH = 2.50   # 2.5 mm axial counterbore depth inside fingertip apex

# =============================================================================
# Required Exported Anthropomorphic Digit & Thumb Specifications
# =============================================================================

DIGIT_SPECS = [
    {
        "name": "index",
        "w_prox": 12.6, "w_mid": 12.2, "w_dist": 12.0,
        "len_p": 34.0,  "len_i": 25.5, "len_d": 24.0,
        "h_p": 12.0,    "h_i": 11.6,   "h_d": 11.2,
        "x": -23.0, "y": 83.5, "py": 79.5, "z": 0.4,
        "rotZ": 0.07, "fm": -0.35, "fp": -0.48, "fd": -0.30,
    },
    {
        "name": "middle",
        "w_prox": 13.4, "w_mid": 12.8, "w_dist": 12.6,
        "len_p": 38.0,  "len_i": 28.0, "len_d": 25.5,
        "h_p": 12.0,    "h_i": 11.6,   "h_d": 11.2,
        "x": -8.0,  "y": 86.0, "py": 82.0, "z": 0.8,
        "rotZ": 0.02, "fm": -0.30, "fp": -0.42, "fd": -0.26,
    },
    {
        "name": "ring",
        "w_prox": 12.8, "w_mid": 12.4, "w_dist": 12.0,
        "len_p": 35.0,  "len_i": 26.0, "len_d": 23.5,
        "h_p": 12.0,    "h_i": 11.6,   "h_d": 11.2,
        "x": 8.0,   "y": 84.8, "py": 80.8, "z": 0.3,
        "rotZ": -0.04, "fm": -0.32, "fp": -0.44, "fd": -0.28,
    },
    {
        "name": "pinky",
        "w_prox": 11.8, "w_mid": 11.4, "w_dist": 11.0,
        "len_p": 30.0,  "len_i": 22.0, "len_d": 20.0,
        "h_p": 12.0,    "h_i": 11.6,   "h_d": 11.2,
        "x": 23.0,  "y": 82.2, "py": 78.2, "z": -0.4,
        "rotZ": -0.10, "fm": -0.38, "fp": -0.50, "fd": -0.32,
    },
]

THUMB_SPEC = {
    "name": "thumb",
    "w_prox": 14.5, "w_dist": 13.5,
    "len_p": 33.0,  "len_d": 27.0,
    "h_p": 13.0,    "h_d": 12.0,
    "fm": -0.35,    "fd": -0.40,
    "f_cmc": -0.35, "f_ip": -0.40,
}


# =============================================================================
# Internal Ray & Surface Profiling Utilities
# =============================================================================

def _effective_condyle_width(width: float) -> float:
    """
    Computes the local proximal condyle boss width so narrow digits (e.g. pinky 11.0 mm)
    simultaneously achieve >= 1.32 mm solid clamping walls AND >= 1.88 mm deep M3 pockets.
    """
    min_w = CLEVIS_SLOT_W + 2.0 * (MIN_CLAMP_WALL + MIN_CB_DEPTH)  # 11.80 mm
    return max(float(width), min_w + 0.10)  # >= 11.90 mm


def _vertical_surface_z(triangles: np.ndarray, x: float, y: float, prefer_top: bool = True) -> float | None:
    """
    Pure vectorized NumPy Moller-Trumbore Z-ray intersection at `(x, y)` returning
    the highest (`prefer_top=True`) or lowest (`prefer_top=False`) Z coordinate on `triangles`.
    """
    v0 = triangles[:, 0, :]
    v1 = triangles[:, 1, :]
    v2 = triangles[:, 2, :]

    # Ray origin = [x, y, -25.0], direction = [0, 0, +1]
    e1 = v1 - v0
    e2 = v2 - v0
    # h = cross([0, 0, 1], e2) = [-e2_y, e2_x, 0]
    hx = -e2[:, 1]
    hy = e2[:, 0]
    a = e1[:, 0] * hx + e1[:, 1] * hy

    valid = np.abs(a) > 1e-10
    if not np.any(valid):
        return None

    f = 1.0 / a[valid]
    sx = x - v0[valid, 0]
    sy = y - v0[valid, 1]
    sz = -25.0 - v0[valid, 2]

    u = f * (sx * hx[valid] + sy * hy[valid])
    u_ok = (u >= -1e-7) & (u <= 1.0 + 1e-7)
    if not np.any(u_ok):
        return None

    idx = np.flatnonzero(valid)[u_ok]
    f_u = f[u_ok]
    sx_u, sy_u, sz_u = sx[u_ok], sy[u_ok], sz[u_ok]
    u_u = u[u_ok]
    e1_u = e1[idx]

    # q = cross(s, e1); d . q = q_z = sx * e1_y - sy * e1_x
    qz = sx_u * e1_u[:, 1] - sy_u * e1_u[:, 0]
    v = f_u * qz
    v_ok = (v >= -1e-7) & (u_u + v <= 1.0 + 1e-7)
    if not np.any(v_ok):
        return None

    idx_v = idx[v_ok]
    f_v = f_u[v_ok]
    sx_v, sy_v, sz_v = sx_u[v_ok], sy_u[v_ok], sz_u[v_ok]
    e1_v = e1[idx_v]
    e2_v = e2[idx_v]

    qx = sy_v * e1_v[:, 2] - sz_v * e1_v[:, 1]
    qy = sz_v * e1_v[:, 0] - sx_v * e1_v[:, 2]
    qz_v = qz[v_ok]

    t = f_v * (e2_v[:, 0] * qx + e2_v[:, 1] * qy + e2_v[:, 2] * qz_v)
    z_vals = -25.0 + t
    return float(np.max(z_vals)) if prefer_top else float(np.min(z_vals))


def _axial_y_exit(triangles: np.ndarray, x: float, z: float) -> float | None:
    """
    Finds the distalmost `+Y` surface coordinate of `triangles` along the line `(x, y, z)`.
    """
    v0 = triangles[:, 0, :]
    v1 = triangles[:, 1, :]
    v2 = triangles[:, 2, :]

    # Ray origin = [x, -25.0, z], direction = [0, +1, 0]
    e1 = v1 - v0
    e2 = v2 - v0
    # h = cross([0, 1, 0], e2) = [e2_z, 0, -e2_x]
    hx = e2[:, 2]
    hz = -e2[:, 0]
    a = e1[:, 0] * hx + e1[:, 2] * hz

    valid = np.abs(a) > 1e-10
    if not np.any(valid):
        return None

    f = 1.0 / a[valid]
    sx = x - v0[valid, 0]
    sy = -25.0 - v0[valid, 1]
    sz = z - v0[valid, 2]

    u = f * (sx * hx[valid] + sz * hz[valid])
    u_ok = (u >= -1e-7) & (u <= 1.0 + 1e-7)
    if not np.any(u_ok):
        return None

    idx = np.flatnonzero(valid)[u_ok]
    f_u = f[u_ok]
    sx_u, sy_u, sz_u = sx[u_ok], sy[u_ok], sz[u_ok]
    u_u = u[u_ok]
    e1_u = e1[idx]

    # d . cross(s, e1) = q_y = sz * e1_x - sx * e1_z
    qy = sz_u * e1_u[:, 0] - sx_u * e1_u[:, 2]
    v = f_u * qy
    v_ok = (v >= -1e-7) & (u_u + v <= 1.0 + 1e-7)
    if not np.any(v_ok):
        return None

    idx_v = idx[v_ok]
    f_v = f_u[v_ok]
    sx_v, sy_v, sz_v = sx_u[v_ok], sy_u[v_ok], sz_u[v_ok]
    e1_v = e1[idx_v]
    e2_v = e2[idx_v]

    qx = sy_v * e1_v[:, 2] - sz_v * e1_v[:, 1]
    qy_v = qy[v_ok]
    qz = sx_v * e1_v[:, 1] - sy_v * e1_v[:, 0]

    t = f_v * (e2_v[:, 0] * qx + e2_v[:, 1] * qy_v + e2_v[:, 2] * qz)
    return float(-25.0 + np.max(t))


# =============================================================================
# Shared Condyle, Clevis, Male Tongue & Dorsal Pinch-Lip Builders
# =============================================================================

def _make_base_condyles(width: float, height: float) -> trimesh.Trimesh:
    """
    Constructs solid anatomical proximal condyles centered at `(0, 0, 0)` with
    reinforced coaxial hardware seating bosses so both the `-X` screw-head counterbore
    and `+X` 6-sided M3 hex-nut pocket remain 360° enclosed with >= 1.32 mm clamping walls.
    """
    condyle_w = _effective_condyle_width(width)
    r_z = height * 0.44

    cyl = cylinder(radius=r_z, height=max(condyle_w - 2.2, 2.0), sections=44)
    cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))

    cap_l = icosphere(subdivisions=3, radius=1.0)
    cap_l.apply_scale([1.1, r_z, r_z])
    cap_l.apply_translation([-(condyle_w / 2.0 - 1.1), 0.0, 0.0])

    cap_r = icosphere(subdivisions=3, radius=1.0)
    cap_r.apply_scale([1.1, r_z, r_z])
    cap_r.apply_translation([(condyle_w / 2.0 - 1.1), 0.0, 0.0])

    # Coaxial tapered hardware boss guaranteeing full annular wall around hex nut corners (r=3.262 mm)
    boss_outer = cylinder(radius=4.05, height=condyle_w, sections=44)
    boss_outer.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    boss_inner = cylinder(radius=min(r_z - 0.15, 4.45), height=max(condyle_w - 0.9, 2.0), sections=44)
    boss_inner.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    boss = trimesh.boolean.union([boss_outer, boss_inner]).convex_hull

    return trimesh.boolean.union([cyl, cap_l, cap_r, boss])


def _create_clevis_cutters(width: float, height: float) -> list[trimesh.Trimesh]:
    """
    Creates v9 female clevis cutters at `y = 0`:
    1. Concentric slot pocket (`r_slot = height * HUB_R_FRAC + 0.26`) clearing all parent hubs
       (including Thumb IP `13.0 -> 12.0` height step) with zero radial rub.
    2. One-directional dorsal mouth (`y <= +0.44 mm`) leaving a solid dorsal ceiling stop for `y > +0.44 mm`.
    3. Calibrated palmar throat relief (`y <= 6.50 mm`, `|x| <= 2.70 mm`) giving `0.000 mm³` collision
       up to 95° flexion while keeping the shaft tendon bore (`y >= 6.60 mm`) 100% enclosed.
    4. `-X` round M3 screw-head counterbore (`Ø6.30 mm`) and `+X` 6-sided captive M3 hex-nut
       socket (`5.65 mm` across flats), both preserving >= 1.32 mm solid clamping walls.
    5. Concentric `Ø3.40 mm` M3 hinge pin bore.
    """
    cutters = []
    condyle_w = _effective_condyle_width(width)
    half_w = condyle_w / 2.0
    r_slot = height * HUB_R_FRAC + 0.26

    # 1. Concentric cylindrical hub slot + slight +Y relief
    slot_cyl_0 = cylinder(radius=r_slot, height=CLEVIS_SLOT_W, sections=48)
    slot_cyl_0.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))

    slot_cyl_fwd = cylinder(radius=r_slot - 0.04, height=CLEVIS_SLOT_W, sections=48)
    slot_cyl_fwd.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    slot_cyl_fwd.apply_translation([0.0, 0.24, 0.0])

    # 2. Rear fork mouth box (y in [-8.0, +0.44], open to +Z=15 and -Z=-12)
    y_stop_face = 0.44
    rear_y_min = -8.0
    rear_len = y_stop_face - rear_y_min
    slot_box_rear = box(extents=[CLEVIS_SLOT_W, rear_len, 27.0])
    slot_box_rear.apply_translation([0.0, (rear_y_min + y_stop_face) / 2.0, 1.5])

    # 3. Exact palmar throat relief prism (|x| <= 2.70 mm, y <= 6.85 mm, z <= -0.45 mm)
    hs = CLEVIS_SLOT_W / 2.0
    yz_throat = [
        (-2.5, -0.45),
        (5.25, -0.45),
        (6.25, -2.40),
        (6.65, TENDON_Z - TENDON_RADIUS),  # Meets bottom of tendon bore (-4.60) cleanly
        (6.85, -12.0),
        (-2.5, -12.0),
    ]
    throat_pts = np.array(
        [[-hs, y, z] for y, z in yz_throat] + [[hs, y, z] for y, z in yz_throat],
        dtype=np.float64,
    )
    palmar_relief = trimesh.convex.convex_hull(throat_pts)

    cutters.append(trimesh.boolean.union([slot_cyl_0, slot_cyl_fwd, slot_box_rear, palmar_relief]))

    # 4. Hardware counterbores with guaranteed >= 1.32 mm clamping web from x = ±2.70 mm
    max_allowed_depth = half_w - (CLEVIS_SLOT_W / 2.0 + MIN_CLAMP_WALL)
    h_depth = min(2.45, max(MIN_CB_DEPTH, max_allowed_depth))
    n_depth = min(2.35, max(MIN_CB_DEPTH, max_allowed_depth))

    # Left (-X): round Ø6.30 mm counterbore for M3 socket/button head
    x_head_floor = -(half_w - h_depth)  # <= -4.02 mm
    head_ext = 4.0
    cb_head_len = x_head_floor - (-(half_w + head_ext))
    cb_head = cylinder(radius=SCREW_HEAD_R, height=cb_head_len, sections=40)
    cb_head.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    cb_head.apply_translation([(x_head_floor - (half_w + head_ext)) / 2.0, 0.0, 0.0])
    cutters.append(cb_head)

    # Right (+X): 6-sided hexagonal prism (5.65 mm across flats) locking M3 nut rotation
    x_nut_floor = half_w - n_depth  # >= +4.02 mm
    nut_ext = 4.0
    cb_nut_len = (half_w + nut_ext) - x_nut_floor
    cb_nut = cylinder(radius=NUT_CIRCUMRADIUS, height=cb_nut_len, sections=6)
    cb_nut.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    cb_nut.apply_translation([(x_nut_floor + (half_w + nut_ext)) / 2.0, 0.0, 0.0])
    cutters.append(cb_nut)

    # 5. Concentric Ø3.40 mm M3 hinge pin hole
    pin_cutter = cylinder(radius=PIN_RADIUS, height=condyle_w + 8.0, sections=40)
    pin_cutter.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    cutters.append(pin_cutter)

    return cutters


def _create_male_tongue_cutters(length: float, width: float, height: float) -> list[trimesh.Trimesh]:
    """
    Creates distal male tongue cutters at `y = length`:
    - Lateral reliefs narrowing the distal hub to `CLEVIS_TONGUE_W = 4.40 mm` back to
      `y = length - 6.35 mm`, plus a 45° palmar-lateral corner chamfer clearing child
      fork walls at 95° flexion.
    - Concentric Ø3.40 mm M3 hinge pin hole at `(0, length, 0)`.
    - Smooth `2.20 mm` wide palmar tendon guide slot below `z = -3.50 mm` at the male tongue,
      preserving the >= 0.70 mm solid pin bridge (`z in [-2.40, -1.70]`) intact!
    """
    cutters = []
    eff_w = _effective_condyle_width(width) + 2.0
    cut_w = (eff_w - CLEVIS_TONGUE_W) / 2.0 + 3.0

    # Lateral reliefs (|x| >= 2.20 mm, y in [length - 6.35, length + 7.65])
    y_back = length - 6.35
    y_front = length + 7.65
    y_len = y_front - y_back
    y_mid = (y_back + y_front) / 2.0

    for sign_x in (-1.0, +1.0):
        xc = sign_x * (CLEVIS_TONGUE_W / 2.0 + cut_w / 2.0)
        side_box = box(extents=[cut_w, y_len, height * 1.6])
        side_box.apply_translation([xc, y_mid, 0.0])
        cutters.append(side_box)

        # Palmar-lateral shoulder bevel (|x| >= 2.20 mm, y in [length - 7.6, length - 6.0], z <= -3.5)
        x0 = sign_x * (CLEVIS_TONGUE_W / 2.0)
        x1 = sign_x * (CLEVIS_TONGUE_W / 2.0 + cut_w)
        xmin, xmax = min(x0, x1), max(x0, x1)
        bevel_yz = [
            (length - 7.6, -10.0),
            (length - 6.0, -10.0),
            (length - 6.0, -3.4),
            (length - 7.6, -5.0),
        ]
        b_pts = np.array(
            [[xmin, y, z] for y, z in bevel_yz] + [[xmax, y, z] for y, z in bevel_yz],
            dtype=np.float64,
        )
        cutters.append(trimesh.convex.convex_hull(b_pts))

    # Concentric distal Ø3.40 mm hinge pin hole
    pin_dist = cylinder(radius=PIN_RADIUS, height=eff_w + 8.0, sections=40)
    pin_dist.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    pin_dist.apply_translation([0.0, length, 0.0])
    cutters.append(pin_dist)

    # Palmar 2.20 mm wide smooth tendon guide slot below z = -3.50 mm at distal male tongue
    # Ramps open cleanly at 45° from z = -4.60 (bottom of enclosed shaft bore) at y = length - 6.6
    # while NEVER cutting above z = -3.50 (preserving cylindrical bore ceiling at z = -2.40)!
    ht = TENDON_RADIUS  # 1.10 mm -> 2.20 mm total slot width
    guide_yz = [
        (length - 6.6, TENDON_Z - TENDON_RADIUS),  # (-4.60)
        (length - 5.2, TENDON_Z),                  # (-3.50)
        (length + 7.0, TENDON_Z),                  # (-3.50)
        (length + 7.0, -10.0),
        (length - 5.0, -10.0),
    ]
    g_pts = np.array(
        [[-ht, y, z] for y, z in guide_yz] + [[ht, y, z] for y, z in guide_yz],
        dtype=np.float64,
    )
    cutters.append(trimesh.convex.convex_hull(g_pts))

    return cutters


def _make_profile_following_dorsal_cutter(
    body_mesh: trimesh.Trimesh,
    length: float,
    height: float,
    is_distal: bool = False,
    z_top: float = 15.0,
) -> tuple[trimesh.Trimesh, float]:
    """
    Constructs a single watertight, multi-station lofted dorsal channel cutter whose
    `z_surf(y)` tracks the exact dorsal crown of `body_mesh` at `x = ±0.90 mm`:
    - Open `1.80 mm` guide slot over the proximal clevis ceiling (`z_fl >= r_slot + 0.25`).
    - Smooth transition into the profile-following snap-fit undercut cavity
      (`w_mouth = 1.80 mm`, `w_cavity = 2.60 mm`, `lip_t = 0.50 mm`, `chamfer_h = 0.50 mm`).
    - On proximal/intermediate phalanges: transitions back to an open `1.80 mm` slot
      over the distal male tongue (`z_fl >= PIN_RADIUS + 1.10`).
    - On distal phalanges: terminates overhanging pinch-lips cleanly by `y = 0.55 * length`
      (before the `0.58 * length` fingernail cuticle), continuing as an open `1.80 mm` slot
      across the `0.70 mm` fingernail bed through the `0.78 * length` retention bore.
    """
    tris = body_mesh.triangles
    hw_m = RB_MOUTH_W / 2.0   # 0.90 mm
    hw_c_max = RB_CAVITY_W / 2.0  # 1.30 mm

    r_slot = height * HUB_R_FRAC + 0.26
    y_clevis_root = r_slot + 0.15  # ~4.67..5.35 mm

    if is_distal:
        y_start = -8.0
        y_end = length + 3.0
        # Ensure full pinch-lip across mid-shaft (including y = 0.42 * length on pinky L=20)
        # and complete termination of overhangs by y = 0.55 * length!
        y_ramp_in_start = min(y_clevis_root, 0.23 * length)
        y_pinch_start = min(y_ramp_in_start + 1.5, 0.31 * length)
        y_pinch_end = 0.50 * length
        y_cuticle_clear = 0.548 * length  # Strictly < 0.55 * length!
        y_stations = np.unique(
            np.concatenate([
                np.linspace(y_start, y_ramp_in_start, 8),
                np.linspace(y_ramp_in_start, y_pinch_start, 8),
                np.linspace(y_pinch_start, y_pinch_end, 16),
                np.linspace(y_pinch_end, y_cuticle_clear, 8),
                np.linspace(y_cuticle_clear, y_end, 22),
            ])
        )
    else:
        y_start = -8.0
        y_end = length + 7.5
        y_ramp_in_start = y_clevis_root
        y_pinch_start = y_clevis_root + 2.0
        y_pinch_end = length - 6.0
        y_ramp_out_end = length - 4.0
        y_stations = np.unique(
            np.concatenate([
                np.linspace(y_start, y_ramp_in_start, 8),
                np.linspace(y_ramp_in_start, y_pinch_start, 8),
                np.linspace(y_pinch_start, y_pinch_end, 24),
                np.linspace(y_pinch_end, y_ramp_out_end, 8),
                np.linspace(y_ramp_out_end, y_end, 10),
            ])
        )

    # Sample exact dorsal surface height along the phalanx body
    y_clamp_min = 0.6
    y_clamp_max = length - (0.6 if is_distal else 0.2)
    z_surfs = np.zeros(len(y_stations), dtype=np.float64)
    for idx, y in enumerate(y_stations):
        yq = float(np.clip(y, y_clamp_min, y_clamp_max))
        zs = _vertical_surface_z(tris, hw_m, yq, prefer_top=True)
        if zs is None:
            zs = _vertical_surface_z(tris, 0.0, yq, prefer_top=True)
        if zs is None:
            zs = height * 0.46
        z_surfs[idx] = zs

    z_prox_joint_floor = max(r_slot + 0.22, z_surfs[0] - 0.80)
    hub_r_dist = height * HUB_R_FRAC - FDM_CLEARANCE
    z_dist_joint_floor = max(PIN_RADIUS + 1.25, hub_r_dist - 0.85)

    verts = []
    retention_floor_z = 1.0

    for idx, y in enumerate(y_stations):
        zs = z_surfs[idx]

        # Compute undercut fraction alpha in [0, 1] (0 = open 1.8 mm slot, 1 = 2.6 mm snap-fit cavity)
        if is_distal:
            if y <= y_ramp_in_start or y >= y_cuticle_clear:
                alpha = 0.0
            elif y < y_pinch_start:
                alpha = (y - y_ramp_in_start) / (y_pinch_start - y_ramp_in_start)
            elif y <= y_pinch_end:
                alpha = 1.0
            else:
                alpha = 1.0 - (y - y_pinch_end) / (y_cuticle_clear - y_pinch_end)

            # Channel floor profile along Y
            if y <= y_ramp_in_start:
                z_fl = z_prox_joint_floor
            elif y < y_pinch_start:
                s = (y - y_ramp_in_start) / (y_pinch_start - y_ramp_in_start)
                z_fl = (1.0 - s) * z_prox_joint_floor + s * (zs - RB_GROOVE_DEPTH)
            elif y <= y_cuticle_clear:
                z_fl = zs - RB_GROOVE_DEPTH
            else:
                # Across fingernail bed (0.55*L -> tip): shallow open 1.8 mm slot ~1.55 mm below nail floor
                s_nail = np.clip((y - y_cuticle_clear) / (0.62 * length - y_cuticle_clear), 0.0, 1.0)
                z_target_nail = max(0.35, (zs - NAIL_DEPTH) - 1.65)
                z_fl = (1.0 - s_nail) * (zs - RB_GROOVE_DEPTH) + s_nail * z_target_nail
        else:
            if y <= y_ramp_in_start or y >= y_ramp_out_end:
                alpha = 0.0
            elif y < y_pinch_start:
                alpha = (y - y_ramp_in_start) / (y_pinch_start - y_ramp_in_start)
            elif y <= y_pinch_end:
                alpha = 1.0
            else:
                alpha = 1.0 - (y - y_pinch_end) / (y_ramp_out_end - y_pinch_end)

            if y <= y_ramp_in_start:
                z_fl = z_prox_joint_floor
            elif y < y_pinch_start:
                s = (y - y_ramp_in_start) / (y_pinch_start - y_ramp_in_start)
                z_fl = (1.0 - s) * z_prox_joint_floor + s * (zs - RB_GROOVE_DEPTH)
            elif y <= y_pinch_end:
                z_fl = zs - RB_GROOVE_DEPTH
            elif y < y_ramp_out_end:
                s = (y - y_pinch_end) / (y_ramp_out_end - y_pinch_end)
                z_fl = (1.0 - s) * (zs - RB_GROOVE_DEPTH) + s * z_dist_joint_floor
            else:
                z_fl = z_dist_joint_floor

        if is_distal and abs(y - 0.78 * length) < 0.8:
            retention_floor_z = z_fl

        hw_c = hw_m + alpha * (hw_c_max - hw_m)

        # Strict monotonic vertical ordering: z_fl < z_cb < z_ct < z_top
        z_ct = max(z_fl + 0.90, zs - RB_LIP_THICKNESS)
        z_cb = max(z_fl + 0.40, z_ct - RB_CHAMFER_H)

        # 8-vertex symmetric cross-section in XZ at station y
        verts.extend([
            [-hw_m, y, z_top],  # 0: top-left
            [-hw_m, y, z_ct],   # 1: left mouth lip underside
            [-hw_c, y, z_cb],   # 2: left chamfer bottom
            [-hw_c, y, z_fl],   # 3: left cavity floor
            [ hw_c, y, z_fl],   # 4: right cavity floor
            [ hw_c, y, z_cb],   # 5: right chamfer bottom
            [ hw_m, y, z_ct],   # 6: right mouth lip underside
            [ hw_m, y, z_top],  # 7: top-right
        ])

    verts_arr = np.asarray(verts, dtype=np.float64)
    n_s = len(y_stations)
    n_p = 8
    faces = []

    for i in range(n_s - 1):
        for j in range(n_p):
            jn = (j + 1) % n_p
            v0 = i * n_p + j
            v1 = i * n_p + jn
            v2 = (i + 1) * n_p + jn
            v3 = (i + 1) * n_p + j
            faces.append([v0, v3, v2])
            faces.append([v0, v2, v1])

    # Proximal & distal rectangular endcaps (both have alpha=0 -> convex rectangle!)
    prox_c_idx = len(verts_arr)
    prox_center = verts_arr[:n_p].mean(axis=0)
    dist_c_idx = prox_c_idx + 1
    dist_center = verts_arr[(n_s - 1) * n_p : n_s * n_p].mean(axis=0)
    verts_arr = np.vstack([verts_arr, prox_center, dist_center])

    for j in range(n_p):
        jn = (j + 1) % n_p
        faces.append([prox_c_idx, j, jn])
        last = (n_s - 1) * n_p
        faces.append([dist_c_idx, last + jn, last + j])

    cutter_mesh = trimesh.Trimesh(vertices=verts_arr, faces=np.asarray(faces, dtype=np.int64), process=True)
    trimesh.repair.fix_normals(cutter_mesh)
    return cutter_mesh, float(retention_floor_z)


# =============================================================================
# Shared Hinged Phalanx Builder (Proximal & Intermediate Phalanges)
# =============================================================================

def _generate_hinged_shaft_body(length: float, width: float, height: float, hub_r_dist: float) -> trimesh.Trimesh:
    """
    Lofts an organic anatomical phalanx shaft from `y = 0` to `y = length`:
    - Blends width smoothly from `_effective_condyle_width(width)` at the base to a tapered distal shaft.
    - Guarantees >= 1.25 mm solid palmar floor (`z <= -5.95 mm`) beneath the `z in [-4.60, -2.40]`
      tendon bore across the enclosed mid-shaft, then tapers smoothly into `hub_r_dist` at the distal joint.
    - Integrates a solid dorsal extension-stop cheek block (`y <= length + 0.30 mm`) on the
      `4.40 mm` wide male tongue so `-5°` and `-10°` hyperextension are rigidly blocked.
    """
    condyle_w = _effective_condyle_width(width)
    n_slices = 36
    n_pts = 44
    y_stations = np.linspace(0.0, length, n_slices)
    vertices = []

    # Target mid-shaft palmar depth guaranteeing >= 1.25 mm floor even at x = ±1.10 mm
    min_mid_palmar_rz = max(height * 0.47, abs(TENDON_Z) + TENDON_RADIUS + 1.32)  # >= 5.92 mm

    for y in y_stations:
        t = y / length

        # Smooth base boss blend over t in [0, 0.28], then gentle anatomical taper
        base_blend = max(0.0, 1.0 - t / 0.28) ** 2
        w_taper = width * (1.0 - 0.08 * t)
        w = w_taper + base_blend * max(0.0, condyle_w - w_taper)

        h = height * (1.0 - 0.05 * t)
        rx = w / 2.0
        rz_dorsal = h * 0.48
        z_offset = -0.15 * np.sin(np.pi * t)

        # Palmar profile: full >= 5.92 mm depth along enclosed tendon shaft, smooth taper near joints
        y_dist_rel = length - y
        if y < 6.2:
            s_prox = y / 6.2
            rz_p_base = height * 0.44
            rz_palmar = (1.0 - s_prox) * rz_p_base + s_prox * min_mid_palmar_rz
        elif y_dist_rel < 8.2:
            # Smoothly taper palmar belly from min_mid_palmar_rz up to hub_r_dist for 95° joint clearance
            s_dist = (8.2 - y_dist_rel) / 8.2
            rz_palmar = (1.0 - s_dist) * min_mid_palmar_rz + s_dist * (hub_r_dist - 0.12)
        else:
            rz_palmar = min_mid_palmar_rz

        theta = np.linspace(0.0, 2.0 * np.pi, n_pts, endpoint=False)
        for th in theta:
            ct = np.cos(th)
            x = rx * np.sin(th)
            z = z_offset + (rz_dorsal * ct if ct >= 0.0 else rz_palmar * ct)
            vertices.append([x, y, z])

    vertices_arr = np.asarray(vertices, dtype=np.float64)
    faces = []
    for i in range(n_slices - 1):
        for j in range(n_pts):
            jn = (j + 1) % n_pts
            v0 = i * n_pts + j
            v1 = i * n_pts + jn
            v2 = (i + 1) * n_pts + jn
            v3 = (i + 1) * n_pts + j
            faces.append([v0, v1, v2])
            faces.append([v0, v2, v3])

    base_c = len(vertices_arr)
    dist_c = base_c + 1
    vertices_arr = np.vstack([vertices_arr, [0.0, 0.0, 0.0], [0.0, length, 0.0]])
    last_row = (n_slices - 1) * n_pts
    for j in range(n_pts):
        jn = (j + 1) % n_pts
        faces.append([base_c, jn, j])
        faces.append([dist_c, last_row + j, last_row + jn])

    shaft_mesh = trimesh.Trimesh(vertices=vertices_arr, faces=np.asarray(faces, dtype=np.int64), process=True)
    trimesh.repair.fix_normals(shaft_mesh)

    base_mesh = _make_base_condyles(width, height)

    distal_hub = cylinder(radius=hub_r_dist, height=condyle_w * 0.92, sections=48)
    distal_hub.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    distal_hub.apply_translation([0.0, length, 0.0])

    # Dorsal hard-stop shoulder block on male tongue (y in [length - 2.0, length + 0.30], |x| <= 2.20 mm)
    z_dorsal_end = height * 0.95 * 0.48 - 0.05
    stop_pts = np.array([
        [-CLEVIS_TONGUE_W / 2.0, length - 2.2, 0.0],
        [ CLEVIS_TONGUE_W / 2.0, length - 2.2, 0.0],
        [-CLEVIS_TONGUE_W / 2.0, length + 0.30, 0.0],
        [ CLEVIS_TONGUE_W / 2.0, length + 0.30, 0.0],
        [-CLEVIS_TONGUE_W / 2.0, length - 2.2, z_dorsal_end],
        [ CLEVIS_TONGUE_W / 2.0, length - 2.2, z_dorsal_end],
        [-CLEVIS_TONGUE_W / 2.0, length + 0.30, z_dorsal_end - 0.12],
        [ CLEVIS_TONGUE_W / 2.0, length + 0.30, z_dorsal_end - 0.12],
    ], dtype=np.float64)
    dorsal_stop_block = trimesh.convex.convex_hull(stop_pts)

    return trimesh.boolean.union([base_mesh, shaft_mesh, distal_hub, dorsal_stop_block])


def _build_hinged_phalanx(length: float, width: float, height: float) -> trimesh.Trimesh:
    """
    Consolidated parametric builder shared by both Proximal and Intermediate phalanges.
    """
    hub_r_dist = height * HUB_R_FRAC - FDM_CLEARANCE
    body = _generate_hinged_shaft_body(length, width, height, hub_r_dist)

    cutters = []
    cutters.extend(_create_clevis_cutters(width, height))
    cutters.extend(_create_male_tongue_cutters(length, width, height))

    # Palmar flexor bore at z = TENDON_Z (-3.50 mm), Ø2.20 mm
    t_flex = cylinder(radius=TENDON_RADIUS, height=length + 24.0, sections=32)
    t_flex.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    t_flex.apply_translation([0.0, length / 2.0, TENDON_Z])
    cutters.append(t_flex)

    # Profile-following dorsal snap-fit pinch-lip undercut channel
    dorsal_cutter, _ = _make_profile_following_dorsal_cutter(body, length, height, is_distal=False)
    cutters.append(dorsal_cutter)

    return body.difference(trimesh.boolean.union(cutters))


# =============================================================================
# Distal Phalanx & Anatomical Fingernail Bed + Knot Pocket Builders
# =============================================================================

def _create_fingernail_bed_cutter(
    body_mesh: trimesh.Trimesh,
    length: float,
    width: float,
    height: float,
    nail_depth: float = NAIL_DEPTH,
    z_top: float = 15.0,
) -> trimesh.Trimesh:
    """
    Creates a shallow (`0.70 mm` deep) anatomical fingernail bed carving cutter on the
    dorsal aspect of a distal phalanx (`y >= 0.58 * length`), tracking `body_mesh`'s
    exact dorsal crown so the recess depth is uniform across all digit sizes.
    """
    tris = body_mesh.triangles
    y_prox = length * NAIL_Y_PROX_FRAC
    y_dist = length + 1.2
    nail_len = y_dist - y_prox
    max_w = width * NAIL_WIDTH_FRAC

    n_y = 30
    n_x = 22
    ys = np.linspace(y_prox, y_dist, n_y)

    grid_bottom = np.zeros((n_y, n_x, 3), dtype=np.float64)
    grid_top = np.zeros((n_y, n_x, 3), dtype=np.float64)

    for i, y in enumerate(ys):
        t = (y - y_prox) / nail_len
        if t < 0.28:
            w_factor = np.sqrt(max(0.002, 1.0 - ((0.28 - t) / 0.28) ** 2))
        else:
            w_factor = 1.0 - 0.06 * (t - 0.28) / 0.72

        half_w = (max_w * w_factor) / 2.0
        yq = float(np.clip(y, y_prox, length - 0.6))
        z_peak = _vertical_surface_z(tris, 0.0, yq, prefer_top=True)
        if z_peak is None:
            z_peak = height * 0.42
        # Gentle natural downward slope past the extreme tip
        if y > length - 0.6:
            z_peak -= 0.35 * (y - (length - 0.6))

        z_floor_peak = z_peak - nail_depth
        r_arch = width * 0.54

        xs = np.linspace(-half_w, half_w, n_x)
        for j, x in enumerate(xs):
            drop = r_arch - np.sqrt(max(0.01, r_arch ** 2 - x ** 2))
            z_cuticle_lift = nail_depth * ((0.12 - t) / 0.12) ** 2 if t < 0.12 else 0.0
            z = z_floor_peak - drop + z_cuticle_lift
            grid_bottom[i, j] = [x, y, z]
            grid_top[i, j] = [x, y, z_top]

    verts = np.vstack([grid_bottom.reshape(-1, 3), grid_top.reshape(-1, 3)])
    faces = []

    def b(i, j):
        return i * n_x + j

    def top(i, j):
        return n_y * n_x + i * n_x + j

    for i in range(n_y - 1):
        for j in range(n_x - 1):
            faces.append([b(i, j), b(i + 1, j), b(i + 1, j + 1)])
            faces.append([b(i, j), b(i + 1, j + 1), b(i, j + 1)])
            faces.append([top(i, j), top(i, j + 1), top(i + 1, j + 1)])
            faces.append([top(i, j), top(i + 1, j + 1), top(i + 1, j)])

    for i in range(n_y - 1):
        faces.append([b(i, 0), top(i, 0), top(i + 1, 0)])
        faces.append([b(i, 0), top(i + 1, 0), b(i + 1, 0)])
        faces.append([b(i, n_x - 1), b(i + 1, n_x - 1), top(i + 1, n_x - 1)])
        faces.append([b(i, n_x - 1), top(i + 1, n_x - 1), top(i, n_x - 1)])

    for j in range(n_x - 1):
        faces.append([b(0, j), b(0, j + 1), top(0, j + 1)])
        faces.append([b(0, j), top(0, j + 1), top(0, j)])
        faces.append([b(n_y - 1, j), top(n_y - 1, j), top(n_y - 1, j + 1)])
        faces.append([b(n_y - 1, j), top(n_y - 1, j + 1), b(n_y - 1, j + 1)])

    nail_mesh = trimesh.Trimesh(vertices=verts, faces=np.asarray(faces, dtype=np.int64), process=True)
    trimesh.repair.fix_normals(nail_mesh)
    return nail_mesh


def generate_distal_phalanx(length: float = 24.0, width: float = 12.0, height: float = 11.2) -> trimesh.Trimesh:
    """
    Generates a v9 distal fingertip phalanx:
    - Reinforced proximal female clevis with captive 6-sided M3 hex nut socket, >= 1.32 mm
      clamping walls, 0° extension hard stop, and unobstructed 0°–95° DIP flexion.
    - Palmar Ø2.20 mm flexor tendon bore at `z = -3.50 mm` with >= 1.25 mm solid shaft floor
      plus a recessed Ø4.0 mm × 2.5 mm enclosed axial knot pocket inside the fingertip apex.
    - Profile-following dorsal snap-fit pinch-lip channel terminating cleanly before `0.55 * length`,
      transitioning into an open `1.80 mm` dorsal slot across the `0.70 mm` fingernail bed
      through the Ø2.0 mm transverse elastic retention bore at `0.78 * length`.
    """
    base = _make_base_condyles(width, height)

    # Cushioned palmar tactile pulp ensuring >= 1.25 mm floor beneath z = -4.60 mm tendon bore
    pulp_z_bot = min(-height * 0.54, TENDON_Z - TENDON_RADIUS - 1.55)  # <= -6.15 mm
    pulp = icosphere(subdivisions=3, radius=1.0)
    pulp.apply_scale([width * 0.47, length * 0.45, abs(pulp_z_bot + 0.6)])
    pulp.apply_translation([0.0, length * 0.56, -0.6])

    dorsal = icosphere(subdivisions=3, radius=1.0)
    dorsal.apply_scale([width * 0.45, length * 0.42, height * 0.45])
    dorsal.apply_translation([0.0, length * 0.52, height * 0.08])

    # Rounded fingertip apex pad sized so the Ø4.0 mm knot pocket at z = -3.50 mm stays 360° enclosed
    apex = icosphere(subdivisions=3, radius=1.0)
    apex.apply_scale([max(width * 0.43, 4.75), height * 0.42, max(height * 0.46, 5.35)])
    apex.apply_translation([0.0, length - 1.5, -0.85])

    smooth_body = trimesh.boolean.union([base, pulp, dorsal, apex]).convex_hull

    cutters = []
    cutters.extend(_create_clevis_cutters(width, height))

    # 1. Palmar Ø2.20 mm flexor tendon bore at z = TENDON_Z (-3.50 mm)
    tendon_bore = cylinder(radius=TENDON_RADIUS, height=length + 24.0, sections=32)
    tendon_bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    tendon_bore.apply_translation([0.0, length / 2.0, TENDON_Z])
    cutters.append(tendon_bore)

    # 2. Recessed Ø4.0 mm × 2.5 mm axial Dyneema knot pocket inside fingertip apex
    y_tip_exit = _axial_y_exit(smooth_body.triangles, 0.0, TENDON_Z)
    if y_tip_exit is None:
        y_tip_exit = length + 1.5
    # Seat pocket floor 2.5 mm inside the distal palmar exit at r = 2.0 mm so bead sits flush
    y_edge_exit = _axial_y_exit(smooth_body.triangles, 0.0, TENDON_Z - KNOT_POCKET_RADIUS + 0.2)
    if y_edge_exit is None:
        y_edge_exit = y_tip_exit - 0.8
    y_pocket_floor = min(y_edge_exit - KNOT_POCKET_DEPTH, y_tip_exit - (KNOT_POCKET_DEPTH + 0.5))
    pocket_cyl_len = (y_tip_exit + 6.0) - y_pocket_floor
    knot_pocket = cylinder(radius=KNOT_POCKET_RADIUS, height=pocket_cyl_len, sections=36)
    knot_pocket.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    knot_pocket.apply_translation([0.0, y_pocket_floor + pocket_cyl_len / 2.0, TENDON_Z])
    cutters.append(knot_pocket)

    # 3. Profile-following dorsal snap-fit pinch-lip channel (terminates < 0.55 * length)
    dorsal_cutter, z_ret_floor = _make_profile_following_dorsal_cutter(
        smooth_body, length, height, is_distal=True
    )
    cutters.append(dorsal_cutter)

    # 4. Transverse Ø2.0 mm elastic retention bore at y = 0.78 * length
    rb_retention = cylinder(radius=1.00, height=_effective_condyle_width(width) + 6.0, sections=28)
    rb_retention.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    rb_retention.apply_translation([0.0, length * 0.78, z_ret_floor + 0.80])
    cutters.append(rb_retention)

    # 5. Shallow 0.70 mm anatomical fingernail bed carving (starts at 0.58 * length)
    nail_cutter = _create_fingernail_bed_cutter(smooth_body, length, width, height, nail_depth=NAIL_DEPTH)
    cutters.append(nail_cutter)

    return smooth_body.difference(trimesh.boolean.union(cutters))


def generate_intermediate_phalanx(length: float = 25.5, width: float = 12.2, height: float = 11.6) -> trimesh.Trimesh:
    """Generates a v9 intermediate phalanx via the shared hinged-phalanx builder."""
    return _build_hinged_phalanx(length=length, width=width, height=height)


def generate_proximal_phalanx(length: float = 34.0, width: float = 12.6, height: float = 12.0) -> trimesh.Trimesh:
    """Generates a v9 proximal phalanx via the shared hinged-phalanx builder."""
    return _build_hinged_phalanx(length=length, width=width, height=height)
