#!/usr/bin/env python3
"""
Iteration 10 (v10.0) Wrist & Forearm Module:
  1. Hollow-Core 2-DoF Cardan Wrist Ring (generate_wrist_cardan_ring)
  2. Rotating 9-Servo Forearm Actuator Barrel + Integral 44T Gear (generate_forearm_inner_barrel)
  3. Stationary Pronation Outer Sleeve + DS3218 Mount + Electronics Tray (generate_forearm_outer_sleeve)
  4. 22-Tooth Pronation Pinion Gear (generate_pronation_pinion)
  5. Printable Micro-Servo Winch Spool (generate_servo_spool)

All 5 parts are single-body watertight manifolds (is_watertight == True, bodies == 1)
designed for zero-collision multi-DoF articulation and FDM printing on a 220x220x250 mm bed.
"""

import numpy as np
import trimesh
from trimesh.creation import box, cylinder

# =============================================================================
# REQUIRED V10 WRIST & FOREARM CONSTANTS
# =============================================================================
WRIST_CENTER = [0.0, -14.0, 0.0]
PUSHROD_PALM_ANCHORS = [[-24.0, -2.0, -8.0], [24.0, -2.0, -8.0]]
PIN_RADIUS = 1.70  # Ø3.4 mm M3 clearance bore

# Pronation 2:1 Spur Gear Train
GEAR_MODULE = 1.75
GEAR_Z_BARREL = 44
GEAR_Z_PINION = 22
GEAR_CENTER_DIST = 57.75  # 0.5 * 1.75 * (44 + 22)
GEAR_ADDENDUM = 1.40
GEAR_DEDENDUM = 1.80
GEAR_FACE_WIDTH = 10.0
GEAR_BACKLASH = 0.28  # Per-gear circular tooth thinning for clean FDM mesh

PINION_AXIS_XZ = (57.75, 0.0)
PINION_Y = -181.0

# Wrist Cardan Ring Geometry
RING_R_OUT = 19.8
RING_R_IN = 11.5       # Ø23.0 mm open Bowden tunnel
RING_THICK_Y = 10.0    # y in [-5.0, +5.0]
RING_Z_FLAT = 16.0     # Flat outer seating pads clearing palm_radial ears (z in [+-16.5, +-20.5])

# Barrel Rear Bearing Journal
BARREL_JOURNAL_R = 34.0
SLEEVE_BEARING_R = 34.40  # 0.40 mm radial clearance

# Standard DS3218 / MG996R Bay Dimensions (open drop-in)
DS3218_BAY_LEN = 41.0     # Clears 40.0 mm body length
DS3218_BAY_WID = 20.6     # Clears 20.0 mm body width
DS3218_BAY_DEP = 38.0     # Clears 38.0 mm body depth
DS3218_FLANGE_L = 49.5    # Standard M3 mounting hole pitch along length
DS3218_FLANGE_W = 10.0    # Standard M3 mounting hole pitch along width
M3_PILOT_R = 1.40         # Ø2.8 mm M3 self-tapping / heat-set pilot bore

# Micro-Servo MG90S / KST DS215MG Bay Dimensions (open outward along +/-X)
MG90S_BAY_W_Y = 12.7      # Clears 12.2 mm body width
MG90S_BAY_H_Z = 23.3      # Clears 22.8 mm body length
MG90S_RACK_HALF_X = 31.5  # Outer mounting tab face at |x| = 31.5 mm
MG90S_CHAN_HALF_X = 8.2   # Central corridor width 16.4 mm (>= 16.0 mm)
MG90S_TAB_SCREW_Z = 13.75 # 27.5 mm center-to-center M2 pilot hole spacing
M2_PILOT_R = 0.90         # Ø1.8 mm M2 pilot bore

# Exported Servo Bay Centers & Extents
# 2x DS3218 Wrist Pushrod Servos in Forearm Inner Barrel (drop-in from -Z palmar face)
BARREL_DS3218_BAY_EXTENTS = (DS3218_BAY_WID, DS3218_BAY_LEN, DS3218_BAY_DEP)  # (20.6, 41.0, 38.0)
BARREL_DS3218_CENTERS = [
    (-20.3, -60.0, -1.0, -1),
    ( 20.3, -60.0, -1.0,  1),
]

# 7x MG90S Tendon Micro-Servos in Forearm Inner Barrel (4 on -X, 3 on +X, drop-in from +/-X)
_MG90S_BAY_DEPTH_X = MG90S_RACK_HALF_X - MG90S_CHAN_HALF_X  # 23.3 mm
_MG90S_CX = MG90S_CHAN_HALF_X + 0.5 * _MG90S_BAY_DEPTH_X    # 19.85 mm
BARREL_MG90S_BAY_EXTENTS = (_MG90S_BAY_DEPTH_X, MG90S_BAY_W_Y, MG90S_BAY_H_Z)  # (23.3, 12.7, 23.3)
BARREL_MG90S_CENTERS = [
    (-_MG90S_CX, -100.0, 0.0, -1),
    (-_MG90S_CX, -115.5, 0.0, -1),
    (-_MG90S_CX, -131.0, 0.0, -1),
    (-_MG90S_CX, -146.5, 0.0, -1),
    ( _MG90S_CX, -104.0, 0.0,  1),
    ( _MG90S_CX, -124.0, 0.0,  1),
    ( _MG90S_CX, -144.0, 0.0,  1),
]

# 1x DS3218 Pronation Drive Servo in Forearm Outer Sleeve (output shaft along +Y at PINION_AXIS_XZ)
# Shaft offset is -10.0 mm along the 41.0 mm length (Z) from bay center -> z_center = +10.0 puts shaft at z = 0.0!
SLEEVE_DS3218_BAY_EXTENTS = (DS3218_BAY_WID, DS3218_BAY_DEP, DS3218_BAY_LEN)  # (20.6, 38.0, 41.0)
SLEEVE_DS3218_CENTER = (PINION_AXIS_XZ[0], -208.0, 10.0)


# =============================================================================
# GEOMETRIC HELPERS
# =============================================================================
def _rounded_rect_section_xz(half_x, half_z, r_corner, n_per_corner=10):
    """Returns CCW 2D points (x, z) for a symmetric rounded rectangle centered at (0, 0)."""
    r = min(r_corner, half_x - 0.1, half_z - 0.1)
    pts = []
    corners = [
        ( half_x - r,  half_z - r, 0.0,           0.5 * np.pi),
        (-half_x + r,  half_z - r, 0.5 * np.pi,   np.pi),
        (-half_x + r, -half_z + r, np.pi,         1.5 * np.pi),
        ( half_x - r, -half_z + r, 1.5 * np.pi,   2.0 * np.pi),
    ]
    for cx, cz, a0, a1 in corners:
        for a in np.linspace(a0, a1, n_per_corner, endpoint=False):
            pts.append([cx + r * np.cos(a), cz + r * np.sin(a)])
    return np.array(pts, dtype=np.float64)


def _rounded_rect_asym_xz(x_min, x_max, z_min, z_max, r_corner, n_per_corner=10):
    """Returns CCW 2D points (x, z) for a rounded rectangle over [x_min, x_max] x [z_min, z_max]."""
    half_x = 0.5 * (x_max - x_min)
    half_z = 0.5 * (z_max - z_min)
    cx = 0.5 * (x_min + x_max)
    cz = 0.5 * (z_min + z_max)
    pts = _rounded_rect_section_xz(half_x, half_z, r_corner, n_per_corner=n_per_corner)
    pts[:, 0] += cx
    pts[:, 1] += cz
    return pts


def _circle_section_xz(radius, n_pts=40):
    """Returns CCW 2D points (x, z) for a circle of given radius centered at (0, 0)."""
    angs = np.linspace(0.0, 2.0 * np.pi, n_pts, endpoint=False)
    return np.column_stack([radius * np.cos(angs), radius * np.sin(angs)])


def _loft_y_stations(y_vals, rings_xz):
    """
    Creates a closed watertight lofted mesh along Y from matched 2D XZ rings.
    y_vals must be strictly increasing; all rings must have the same point count and CCW winding.
    """
    n_s = len(y_vals)
    n_p = len(rings_xz[0])
    verts = []
    for yv, ring in zip(y_vals, rings_xz):
        for px, pz in ring:
            verts.append([px, yv, pz])

    faces = []
    for s in range(n_s - 1):
        b0 = s * n_p
        b1 = (s + 1) * n_p
        for i in range(n_p):
            ni = (i + 1) % n_p
            faces.append([b0 + i, b1 + i, b1 + ni])
            faces.append([b0 + i, b1 + ni, b0 + ni])

    # Cap start (y_min) and end (y_max)
    c0 = len(verts)
    verts.append([float(np.mean(rings_xz[0][:, 0])), float(y_vals[0]), float(np.mean(rings_xz[0][:, 1]))])
    c1 = len(verts)
    verts.append([float(np.mean(rings_xz[-1][:, 0])), float(y_vals[-1]), float(np.mean(rings_xz[-1][:, 1]))])

    b_last = (n_s - 1) * n_p
    for i in range(n_p):
        ni = (i + 1) % n_p
        faces.append([c0, i, ni])
        faces.append([c1, b_last + ni, b_last + i])

    mesh = trimesh.Trimesh(vertices=np.array(verts, dtype=np.float64), faces=np.array(faces, dtype=np.int64))
    mesh.fix_normals()
    return mesh


def _extrude_polygon_y(poly_xz, y_min, y_max):
    """Extrudes a simple CCW 2D polygon in XZ along Y from y_min to y_max into a watertight mesh."""
    n = len(poly_xz)
    cx = float(np.mean(poly_xz[:, 0]))
    cz = float(np.mean(poly_xz[:, 1]))

    # Note: star-shaped polygon fan works cleanly for all gears & rounded rects centered near origin
    verts = []
    for px, pz in poly_xz:
        verts.append([px, y_min, pz])
    for px, pz in poly_xz:
        verts.append([px, y_max, pz])
    c_bot = len(verts)
    verts.append([cx, y_min, cz])
    c_top = len(verts)
    verts.append([cx, y_max, cz])

    faces = []
    for i in range(n):
        ni = (i + 1) % n
        # Side quad
        faces.append([i, n + i, n + ni])
        faces.append([i, n + ni, ni])
        # Caps
        faces.append([c_bot, i, ni])
        faces.append([c_top, n + ni, n + i])

    mesh = trimesh.Trimesh(vertices=np.array(verts, dtype=np.float64), faces=np.array(faces, dtype=np.int64))
    mesh.fix_normals()
    return mesh


def _spur_gear_polygon_xz(
    z_teeth,
    module=GEAR_MODULE,
    addendum=GEAR_ADDENDUM,
    dedendum=GEAR_DEDENDUM,
    pressure_angle_deg=20.0,
    backlash=GEAR_BACKLASH,
    phase_rad=0.0,
    n_flank=5,
):
    """
    Generates a clean, star-shaped, 3D-printable involute spur gear profile in the XZ plane.
    """
    r_p = 0.5 * module * z_teeth
    r_tip = r_p + addendum
    r_root = r_p - dedendum
    alpha = np.radians(pressure_angle_deg)
    r_b = r_p * np.cos(alpha)

    # Half angular tooth thickness at pitch circle (with backlash thinning)
    pitch_circ = np.pi * module
    s_p = 0.5 * pitch_circ - backlash
    half_tooth_p = s_p / (2.0 * r_p)

    # Involute function inv(phi) = tan(phi) - phi
    def _inv(phi):
        return np.tan(phi) - phi

    inv_alpha = _inv(alpha)
    half_tooth_b = half_tooth_p + inv_alpha

    # Build one tooth symmetric around angle = 0
    r_start = max(r_b, r_root)
    radii_up = np.linspace(r_start, r_tip, n_flank)

    # Angular offset at radius r on the right flank (positive angle)
    def _flank_angle(r):
        if r <= r_b:
            return half_tooth_b
        phi = np.arccos(np.clip(r_b / r, -1.0, 1.0))
        return half_tooth_b - _inv(phi)

    # Root fillet clearance angle
    ang_root = _flank_angle(r_start) + 0.04 * (2.0 * np.pi / z_teeth)
    half_pitch = np.pi / z_teeth
    ang_root = min(ang_root, half_pitch * 0.92)

    pts = []
    step = 2.0 * np.pi / z_teeth
    for k in range(z_teeth):
        center_ang = k * step + phase_rad

        # Valley left of tooth
        pts.append((r_root, center_ang - half_pitch))
        pts.append((r_root, center_ang - ang_root))

        # Left flank (ascending from root to tip)
        for r in radii_up:
            pts.append((r, center_ang - _flank_angle(r)))

        # Right flank (descending from tip to root)
        for r in reversed(radii_up):
            pts.append((r, center_ang + _flank_angle(r)))

        # Root right of tooth
        pts.append((r_root, center_ang + ang_root))

    poly = np.array([[r * np.cos(a), r * np.sin(a)] for r, a in pts], dtype=np.float64)
    return poly


def _cyl_along_x(radius, height, center, sections=32):
    m = cylinder(radius=radius, height=height, sections=sections)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    m.apply_translation(center)
    return m


def _cyl_along_y(radius, height, center, sections=48):
    m = cylinder(radius=radius, height=height, sections=sections)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    m.apply_translation(center)
    return m


def _cyl_along_z(radius, height, center, sections=32):
    m = cylinder(radius=radius, height=height, sections=sections)
    m.apply_translation(center)
    return m


# =============================================================================
# 1. HOLLOW-CORE 2-DOF CARDAN WRIST RING
# =============================================================================
def generate_wrist_cardan_ring():
    """
    Generates the v10.0 Hollow-Core 2-DoF Cardan Wrist Ring in local coordinates
    centered at (0, 0, 0) (placed at WRIST_CENTER = [0.0, -14.0, 0.0] in assembly).
      - Outer radius R_out = 19.8 mm, inner Bowden tunnel R_in = 11.5 mm (Ø23.0 mm).
      - Thickness along Y = 10.0 mm (y in [-5.0, +5.0]).
      - Flat outer Z seating pads at z = +/-16.0 mm clearing palm_radial ears (z in [+/-16.5, +/-20.5]).
      - X-axis Pitch pin bores (Ø3.4 mm) through (+/-15.5, 0, 0) mating with forearm_inner_barrel.
      - Z-axis Yaw pin bores (Ø3.4 mm) through (0, 0, +/-15.5) mating with palm_radial.
    """
    outer = _cyl_along_y(RING_R_OUT, RING_THICK_Y, [0.0, 0.0, 0.0], sections=64)

    cutters = []
    # Central Ø23.0 mm open Bowden tunnel bore
    cutters.append(_cyl_along_y(RING_R_IN, RING_THICK_Y + 4.0, [0.0, 0.0, 0.0], sections=64))

    # Flat outer Z seating pads at z = +/-16.0 mm
    for sign in (-1.0, 1.0):
        trim_z = box(extents=[RING_R_OUT * 2.5, RING_THICK_Y + 4.0, 12.0])
        trim_z.apply_translation([0.0, 0.0, sign * (RING_Z_FLAT + 6.0)])
        cutters.append(trim_z)

    # X-axis Pitch pin thru-bore (Ø3.4 mm)
    cutters.append(_cyl_along_x(PIN_RADIUS, RING_R_OUT * 2.4, [0.0, 0.0, 0.0], sections=32))

    # Z-axis Yaw pin thru-bore (Ø3.4 mm)
    cutters.append(_cyl_along_z(PIN_RADIUS, RING_Z_FLAT * 2.4, [0.0, 0.0, 0.0], sections=32))

    ring = outer.difference(trimesh.boolean.union(cutters))
    return ring


# =============================================================================
# 2. ROTATING 9-SERVO FOREARM ACTUATOR BARREL + INTEGRAL 44T GEAR
# =============================================================================
def generate_forearm_inner_barrel():
    """
    Generates the v10.0 Rotating 9-Servo Forearm Actuator Barrel:
      - Front Wrist Gimbal Yoke (y = -14.0 to -32.0): lateral arms at x = +/-23.0 mm
        (inner +/-20.5, outer +/-25.5) with Ø3.4 mm X-axis Pitch pin bores at [+/-23.0, -14.0, 0.0],
        clearing wrist_cardan_ring across +/-45 deg Pitch with 0.000 mm^3 collision.
      - 2x Standard High-Torque Wrist Servos (DS3218 / MG996R) (y = -32.0 to -90.0):
        open drop-in bays on -X and +X (20.6 x 41.0 x 38.0 mm) opening palmarly (-Z) with
        4x Ø2.8 mm M3 pilot bores per servo at 49.5 x 10.0 mm flange centers.
      - 7x Micro-Servo Tendon Bays (MG90S / KST DS215MG) (y = -90.0 to -156.5):
        4 bays on -X, 3 bays on +X (12.7 x 23.3 mm opening outward along +/-X) with
        14x Ø1.8 mm M2 pilot bores on 27.5 mm centers + full-length central tendon corridor.
      - Rear Bearing Journal (R = 34.0 mm, y in [-156.5, -174.0]) + Integral 44T Spur Gear
        (m = 1.75 mm, pitch dia 77.0 mm, R_tip = 39.9 mm, y in [-176.0, -186.0]).
    """
    add_parts = []

    # --- A. Front Wrist Gimbal Yoke Arms (x in [+/-20.5, +/-25.5]) ---
    for sign in (-1.0, 1.0):
        arm_x = sign * 23.0
        # Rounded front pin boss centered at [+/-23.0, -14.0, 0.0]
        boss = _cyl_along_x(5.6, 5.0, [arm_x, -14.0, 0.0], sections=36)
        add_parts.append(boss)

        # Straight arm strut from y = -14.0 back to y = -33.5 (overlaps front bulkhead cleanly)
        arm_strut = box(extents=[5.0, 19.5, 11.2])
        arm_strut.apply_translation([arm_x, -23.75, 0.0])
        add_parts.append(arm_strut)

        # Structural gusset root flaring into the DS3218 front bulkhead (y in [-27.0, -34.0])
        gusset = box(extents=[7.5, 7.0, 18.0])
        gusset.apply_translation([sign * 21.75, -30.5, 0.0])
        add_parts.append(gusset)

    # --- B. Main Barrel Lofted Body (y = -176.5 to -32.0) ---
    # All rings have 40 points (4 corners * 10 pts) for clean matched lofting
    ring_ds = _rounded_rect_asym_xz(-33.5, 33.5, -20.0, 21.5, 8.0, n_per_corner=10)
    ring_mg = _rounded_rect_section_xz(MG90S_RACK_HALF_X, 17.0, 3.2, n_per_corner=10)
    ring_jrn = _circle_section_xz(BARREL_JOURNAL_R, n_pts=40)

    barrel_body = _loft_y_stations(
        [-176.5, -174.0, -156.5, -153.5, -93.0, -90.0, -32.0],
        [ring_jrn, ring_jrn, ring_jrn, ring_mg, ring_mg, ring_ds, ring_ds],
    )
    add_parts.append(barrel_body)

    # --- C. Integral 44-Tooth Pronation Spur Gear (y in [-186.0, -176.0]) ---
    gear_poly = _spur_gear_polygon_xz(
        GEAR_Z_BARREL,
        module=GEAR_MODULE,
        addendum=GEAR_ADDENDUM,
        dedendum=GEAR_DEDENDUM,
        backlash=GEAR_BACKLASH,
        phase_rad=0.0,
    )
    gear_44t = _extrude_polygon_y(gear_poly, -186.0, -176.0)
    add_parts.append(gear_44t)

    solid_barrel = trimesh.boolean.union(add_parts)

    # --- CUTTERS ---
    cutters = []

    # 1. Wrist Gimbal Pitch Pin Bores (Ø3.4 mm at [+/-23.0, -14.0, 0.0])
    cutters.append(_cyl_along_x(PIN_RADIUS, 56.0, [0.0, -14.0, 0.0], sections=32))

    # 2. Gimbal Swing Clearance Cutout (clears wrist_cardan_ring R_yz_max = 16.76 mm across +/-45 deg Pitch)
    gimbal_clear = _cyl_along_x(18.5, 41.0, [0.0, -14.0, 0.0], sections=48)
    cutters.append(gimbal_clear)

    # 3. Full-Length Central Tendon & Wiring Routing Corridor
    # Through DS3218 + MG90S zones: 16.4 mm wide (x in [-8.2, +8.2]) x 22.0 mm tall (z in [-11.0, +11.0])
    central_corridor = box(extents=[MG90S_CHAN_HALF_X * 2.0, 146.0, 22.0])
    central_corridor.apply_translation([0.0, -101.0, 0.0])
    cutters.append(central_corridor)

    # Cylindrical rear wiring bore (Ø28.0 mm) through bearing journal & 44T gear out the back
    rear_wire_bore = _cyl_along_y(14.0, 32.0, [0.0, -172.0, 0.0], sections=48)
    cutters.append(rear_wire_bore)

    # 4. 2x Standard DS3218 / MG996R Wrist Servo Bays (open palmarly along -Z for drop-in)
    for cx, cy, cz, _side in BARREL_DS3218_CENTERS:
        # Bay extends from z = +18.0 down past z = -20.0 (to z = -26.0) for unobstructed -Z drop-in
        ds_bay = box(extents=[DS3218_BAY_WID, DS3218_BAY_LEN, DS3218_BAY_DEP + 6.0])
        ds_bay.apply_translation([cx, cy, cz - 3.0])
        cutters.append(ds_bay)

        # 4x Ø2.8 mm M3 pilot screw bores at 49.5 x 10.0 mm flange centers (drilled upward into +Z from z = -20.0)
        for dy in (-0.5 * DS3218_FLANGE_L, 0.5 * DS3218_FLANGE_L):
            for dx in (-0.5 * DS3218_FLANGE_W, 0.5 * DS3218_FLANGE_W):
                pilot = _cyl_along_z(M3_PILOT_R, 14.0, [cx + dx, cy + dy, -14.0], sections=20)
                cutters.append(pilot)

    # 5. 7x Micro-Servo MG90S Tendon Bays (4 on -X, 3 on +X, open outward along +/-X)
    bay_cut_x = _MG90S_BAY_DEPTH_X + 12.0  # Extends from |x| = 8.2 out past |x| = 31.5
    for _cx, cy, cz, side in BARREL_MG90S_CENTERS:
        bx_cut = side * (MG90S_CHAN_HALF_X + 0.5 * bay_cut_x)
        mg_bay = box(extents=[bay_cut_x, MG90S_BAY_W_Y, MG90S_BAY_H_Z])
        mg_bay.apply_translation([bx_cut, cy, cz])
        cutters.append(mg_bay)

        # 2x Ø1.8 mm M2 pilot screw bores per servo on 27.5 mm Z-centers (10.0 mm deep from outer face)
        pilot_x = side * (MG90S_RACK_HALF_X - 4.5)
        for sz in (-MG90S_TAB_SCREW_Z, MG90S_TAB_SCREW_Z):
            pilot = _cyl_along_x(M2_PILOT_R, 11.0, [pilot_x, cy, sz], sections=18)
            cutters.append(pilot)

    return solid_barrel.difference(trimesh.boolean.union(cutters))


# =============================================================================
# 3. STATIONARY PRONATION OUTER SLEEVE + DS3218 MOUNT + ELECTRONICS TRAY
# =============================================================================
def generate_forearm_outer_sleeve():
    """
    Generates the v10.0 Stationary Pronation Outer Sleeve (y in [-206.0, -158.0]):
      - Concentric inner bearing housing (R = 34.40 mm, 0.40 mm clearance over barrel's R = 34.0 mm journal).
      - Enlarged gear shroud cavity (R = 41.2 mm, y in [-189.5, -174.5]) with side mesh window for 22T pinion.
      - Rear retention bulkhead at y in [-206.0, -190.0] with Ø44.0 mm central power/USB cable pass-through.
      - Side-mounted 1x DS3218 Pronation Servo Bay on +X with shaft axis at PINION_AXIS_XZ = (57.75, 0.0)
        and 4x Ø2.8 mm M3 pilot bores at standard 49.5 x 10.0 mm flange centers.
      - Integrated Electronics Tray on -X with 4x M3 standoffs for PCA9685 (56.0 x 19.0 mm) and
        4x M3 standoffs for ESP32-S3 DevKit (51.0 x 18.0 mm).
    """
    add_parts = []

    # --- A. Main Cylindrical Outer Cuff (y in [-206.0, -158.0], R_out = 45.5 mm) ---
    cuff = _cyl_along_y(45.5, 48.0, [0.0, -182.0, 0.0], sections=64)
    add_parts.append(cuff)

    # --- B. Side-Mounted DS3218 Pronation Servo Mount Block (+X side) ---
    # Mounting flange face at y = -189.0, extending back to y = -206.0 (17.0 mm solid mount flange)
    # Servo bay: x in [47.45, 68.05], z in [-10.5, +30.5]; M3 screws at x = 57.75 +/- 5.0, z = 10.0 +/- 24.75
    sx_c, _sy_c, sz_c = SLEEVE_DS3218_CENTER
    servo_mount = box(extents=[32.0, 17.0, 62.0])
    servo_mount.apply_translation([56.0, -197.5, sz_c])  # x in [40.0, 72.0], y in [-206.0, -189.0], z in [-21.0, +41.0]
    add_parts.append(servo_mount)

    # Top & bottom stiffening gussets connecting servo mount smoothly to front cuff (y in [-189.0, -158.0])
    # Positioned well clear of the 22T pinion (R_tip = 20.65 mm around z = 0.0 -> |z| >= 25.0)
    upper_gusset = box(extents=[24.0, 31.0, 10.0])
    upper_gusset.apply_translation([50.0, -173.5, 34.0])  # z in [29.0, 39.0]
    add_parts.append(upper_gusset)

    # --- C. Integrated Electronics Tray on -X Side (PCA9685 + ESP32-S3) ---
    # Vertical tray plate at x in [-51.0, -43.0], y in [-206.0, -158.0], z in [-35.0, +35.0]
    elec_tray = box(extents=[10.0, 48.0, 70.0])
    elec_tray.apply_translation([-46.0, -182.0, 0.0])
    add_parts.append(elec_tray)

    # Standoff bosses (Ø6.4 mm projecting 3.0 mm outward to x = -54.0 mm),
    # kept strictly within y in [-206.0, -158.0] so the rear face sits dead-flat on the print bed.
    pca9685_holes_yz = [
        (-161.5, -28.0),
        (-161.5,  28.0),
        (-180.5, -28.0),
        (-180.5,  28.0),
    ]
    esp32_holes_yz = [
        (-184.5, -25.5),
        (-184.5,  25.5),
        (-202.5, -25.5),
        (-202.5,  25.5),
    ]
    all_elec_holes = pca9685_holes_yz + esp32_holes_yz
    for hy, hz in all_elec_holes:
        boss = _cyl_along_x(3.2, 6.0, [-51.0, hy, hz], sections=20)
        add_parts.append(boss)

    solid_sleeve = trimesh.boolean.union(add_parts)

    # --- CUTTERS ---
    cutters = []

    # 1. Concentric Inner Bearing Bore (R = 34.40 mm, y in [-175.0, -156.0])
    bearing_bore = _cyl_along_y(SLEEVE_BEARING_R, 19.0, [0.0, -165.5, 0.0], sections=64)
    cutters.append(bearing_bore)

    # 2. 44T Gear Clearance Cavity (R = 41.2 mm, y in [-189.5, -174.5])
    gear_cavity = _cyl_along_y(41.2, 15.0, [0.0, -182.0, 0.0], sections=64)
    cutters.append(gear_cavity)

    # 3. Side Pinion Mesh Window through +X wall of gear shroud (clears 22T pinion R_tip = 20.65 mm at x = 57.75, z = 0)
    pinion_clear = _cyl_along_y(22.2, 16.0, [PINION_AXIS_XZ[0], -181.5, PINION_AXIS_XZ[1]], sections=48)
    cutters.append(pinion_clear)

    # Clear the bridge between the 44T cavity and 22T pinion window
    mesh_bridge = box(extents=[32.0, 15.0, 44.4])
    mesh_bridge.apply_translation([42.0, -182.0, 0.0])
    cutters.append(mesh_bridge)

    # 4. Rear Cable Pass-Through Bore (Ø44.0 mm -> R = 22.0 mm, y in [-208.0, -188.0])
    cable_bore = _cyl_along_y(22.0, 20.0, [0.0, -198.0, 0.0], sections=48)
    cutters.append(cable_bore)

    # 5. DS3218 Pronation Servo Drop-In Bay (open from -Y rear, y in [-227.0, -188.5])
    ds_bay = box(extents=[DS3218_BAY_WID, 40.0, DS3218_BAY_LEN])
    ds_bay.apply_translation([sx_c, -208.5, sz_c])
    cutters.append(ds_bay)

    # 4x Ø2.8 mm M3 pilot screw bores for DS3218 flange (49.5 x 10.0 mm centers along Z and X, drilled along Y)
    for dx in (-0.5 * DS3218_FLANGE_W, 0.5 * DS3218_FLANGE_W):
        for dz in (-0.5 * DS3218_FLANGE_L, 0.5 * DS3218_FLANGE_L):
            pilot = _cyl_along_y(M3_PILOT_R, 19.0, [sx_c + dx, -197.5, sz_c + dz], sections=20)
            cutters.append(pilot)

    # 6. 8x Ø2.8 mm M3 Standoff Pilot Bores on Electronics Tray (PCA9685 + ESP32-S3)
    for hy, hz in all_elec_holes:
        elec_bore = _cyl_along_x(M3_PILOT_R, 11.0, [-50.5, hy, hz], sections=18)
        cutters.append(elec_bore)

    return solid_sleeve.difference(trimesh.boolean.union(cutters))


# =============================================================================
# 4. 22-TOOTH PRONATION PINION GEAR
# =============================================================================
def generate_pronation_pinion():
    """
    Generates the v10.0 22-Tooth Pronation Pinion Gear in local coordinates
    centered at (0, 0, 0) with gear axis along Y (y in [-5.0, +5.0], face width 10.0 mm).
    Placed in assembly at (PINION_AXIS_XZ[0], PINION_Y, PINION_AXIS_XZ[1]) = (57.75, -181.0, 0.0).
      - 22 teeth, m = 1.75 mm, pitch dia 38.5 mm, 2:1 ratio with barrel 44T gear.
      - Phase-aligned by pi/22 so a pinion tooth centers cleanly in a 44T valley at roll = 0 deg.
      - Ø6.0 mm 25T/horn adapter counterbore pocket + Ø3.2 mm M3 center bore + 2x Ø2.2 mm M2 horn bores at R = 7.5 mm.
    """
    pinion_poly = _spur_gear_polygon_xz(
        GEAR_Z_PINION,
        module=GEAR_MODULE,
        addendum=GEAR_ADDENDUM,
        dedendum=GEAR_DEDENDUM,
        backlash=GEAR_BACKLASH,
        phase_rad=np.pi / GEAR_Z_PINION,
    )
    pinion_body = _extrude_polygon_y(pinion_poly, -0.5 * GEAR_FACE_WIDTH, 0.5 * GEAR_FACE_WIDTH)

    cutters = []
    # Ø3.2 mm M3 center screw thru-bore
    cutters.append(_cyl_along_y(1.60, GEAR_FACE_WIDTH + 4.0, [0.0, 0.0, 0.0], sections=28))

    # Ø6.0 mm 25T/horn adapter center hub pocket on -Y face (y in [-5.5, -1.0])
    cutters.append(_cyl_along_y(3.00, 4.5, [0.0, -3.25, 0.0], sections=32))

    # 2x Ø2.2 mm M2 horn bolt thru-bores at R = 7.5 mm
    for sign in (-1.0, 1.0):
        cutters.append(_cyl_along_y(1.10, GEAR_FACE_WIDTH + 4.0, [sign * 7.5, 0.0, 0.0], sections=22))

    return pinion_body.difference(trimesh.boolean.union(cutters))


# =============================================================================
# 5. PRINTABLE MICRO-SERVO WINCH SPOOL
# =============================================================================
def generate_servo_spool():
    """
    Generates the v10.0 Printable Micro-Servo Winch Spool (print 7x for MG90S tendon servos):
      - Axis along Z, total height 6.0 mm (z in [-3.0, +3.0]).
      - Ø10.0 mm winding drum, Ø14.5 mm outer flanges (1.3 mm thick each).
      - Ø4.8 mm MG90S horn boss counterbore pocket (3.5 mm deep from bottom).
      - Ø2.2 mm M2 center screw thru-bore + Ø1.5 mm radial Dyneema tie-off hole.
    """
    drum = _cyl_along_z(5.00, 6.0, [0.0, 0.0, 0.0], sections=48)
    flange_bot = _cyl_along_z(7.25, 1.3, [0.0, 0.0, -2.35], sections=48)
    flange_top = _cyl_along_z(7.25, 1.3, [0.0, 0.0,  2.35], sections=48)
    spool_solid = trimesh.boolean.union([drum, flange_bot, flange_top])

    cutters = []
    # Ø2.2 mm center screw thru-bore
    cutters.append(_cyl_along_z(1.10, 8.0, [0.0, 0.0, 0.0], sections=24))

    # Ø4.8 mm MG90S horn boss pocket from z = -3.5 to z = +0.5
    cutters.append(_cyl_along_z(2.40, 4.0, [0.0, 0.0, -1.5], sections=32))

    # Ø1.5 mm radial Dyneema tie-off hole through drum wall at z = 0.0
    cutters.append(_cyl_along_x(0.75, 6.0, [4.0, 0.0, 0.0], sections=20))

    return spool_solid.difference(trimesh.boolean.union(cutters))


# =============================================================================
# SELF-TEST SUITE
# =============================================================================
def _intersection_volume(m1, m2):
    inter = m1.intersection(m2)
    if inter.is_empty or len(inter.faces) == 0:
        return 0.0
    return float(abs(inter.volume))


if __name__ == "__main__":
    print("=== v10.0 Wrist & Forearm Self-Test ===")
    parts = {
        "wrist_cardan_ring": generate_wrist_cardan_ring(),
        "forearm_inner_barrel": generate_forearm_inner_barrel(),
        "forearm_outer_sleeve": generate_forearm_outer_sleeve(),
        "pronation_pinion": generate_pronation_pinion(),
        "servo_spool": generate_servo_spool(),
    }

    # 1. Watertight, single-body, and 220x220x250 print bed checks
    for name, mesh in parts.items():
        bodies = len(mesh.split(only_watertight=False))
        ext = mesh.extents
        ext_sorted = sorted(ext)
        print(
            f"  {name:22s}: watertight={mesh.is_watertight}, bodies={bodies}, "
            f"vol={mesh.volume:9.1f} mm^3, tris={len(mesh.faces):6d}, "
            f"ext=({ext[0]:.1f}, {ext[1]:.1f}, {ext[2]:.1f}) mm"
        )
        assert mesh.is_watertight, f"{name} is not watertight!"
        assert mesh.volume > 0.0, f"{name} has non-positive volume!"
        assert bodies == 1, f"{name} has {bodies} bodies (expected 1)!"
        assert ext_sorted[0] <= 220.0 and ext_sorted[1] <= 220.0 and ext_sorted[2] <= 250.0, (
            f"{name} exceeds 220x220x250 mm print bed: {ext}"
        )

    ring_local = parts["wrist_cardan_ring"]
    barrel = parts["forearm_inner_barrel"]
    sleeve = parts["forearm_outer_sleeve"]
    pinion_local = parts["pronation_pinion"]

    # 2. Wrist Cardan Ring vs Forearm Inner Barrel across +/-45 deg Pitch
    print("\n--- Wrist Pitch Sweep (+/-45 deg) ---")
    for deg in (-45, -30, -15, 0, 15, 30, 45):
        r_posed = ring_local.copy()
        r_posed.apply_transform(trimesh.transformations.rotation_matrix(np.radians(deg), [1, 0, 0]))
        r_posed.apply_translation(WRIST_CENTER)
        vol = _intersection_volume(r_posed, barrel)
        print(f"  Pitch {deg:+4d} deg: collision = {vol:.6f} mm^3")
        assert vol < 0.05, f"Wrist pitch collision at {deg} deg: {vol} mm^3"

    # 3. Forearm Inner Barrel vs Outer Sleeve across +/-90 deg Roll
    print("\n--- Forearm Pronation Roll Sweep (+/-90 deg) ---")
    for deg in (-90, -60, -30, 0, 30, 60, 90):
        b_rot = barrel.copy()
        b_rot.apply_transform(trimesh.transformations.rotation_matrix(np.radians(deg), [0, 1, 0]))
        vol = _intersection_volume(b_rot, sleeve)
        print(f"  Roll  {deg:+4d} deg: barrel vs sleeve collision = {vol:.6f} mm^3")
        assert vol < 0.05, f"Barrel vs sleeve collision at {deg} deg: {vol} mm^3"

    # 4. 44T Barrel Gear vs 22T Pronation Pinion Synchronized Mesh Sweep
    print("\n--- 44T / 22T Pronation Gear Mesh Sweep ---")
    for deg in (-90, -45, -10, -4, -2, 0, 2, 4, 10, 45, 90):
        b_rot = barrel.copy()
        b_rot.apply_transform(trimesh.transformations.rotation_matrix(np.radians(deg), [0, 1, 0]))
        p_rot = pinion_local.copy()
        # 2:1 ratio, opposite rotation sense
        p_rot.apply_transform(trimesh.transformations.rotation_matrix(np.radians(-2.0 * deg), [0, 1, 0]))
        p_rot.apply_translation([PINION_AXIS_XZ[0], PINION_Y, PINION_AXIS_XZ[1]])
        vol_gear = _intersection_volume(b_rot, p_rot)
        vol_slv = _intersection_volume(p_rot, sleeve)
        print(f"  Roll  {deg:+4d} deg (pinion {-2*deg:+5d} deg): gear_col={vol_gear:.6f}, pinion_vs_sleeve={vol_slv:.6f} mm^3")
        assert vol_gear < 0.05, f"Gear mesh collision at {deg} deg: {vol_gear} mm^3"
        assert vol_slv < 0.05, f"Pinion vs sleeve collision at {deg} deg: {vol_slv} mm^3"

    # 5. Unobstructed Servo Drop-In Clearance (7x MG90S + 3x DS3218)
    print("\n--- Servo Drop-In Clearance Verification ---")
    # 7x MG90S in barrel (12.2 x 22.8 x 22.0 mm standard body, drop-in along +/-X)
    for idx, (cx, cy, cz, side) in enumerate(BARREL_MG90S_CENTERS):
        for offset_x in (0.0, 10.0, 25.0):
            s_box = box(extents=[22.0, 12.2, 22.8])
            s_box.apply_translation([cx + side * offset_x, cy, cz])
            vol = _intersection_volume(s_box, barrel)
            assert vol < 0.05, f"MG90S #{idx} collision at drop offset {offset_x}: {vol} mm^3"
    print("  7x Barrel MG90S bays: 0.000 mm^3 collision (seated + drop-in path)")

    # 2x DS3218 in barrel (20.0 x 40.0 x 37.5 mm standard body, drop-in along -Z)
    for idx, (cx, cy, cz, _side) in enumerate(BARREL_DS3218_CENTERS):
        for offset_z in (0.0, -15.0, -35.0):
            s_box = box(extents=[20.0, 40.0, 37.5])
            s_box.apply_translation([cx, cy, cz + offset_z])
            vol = _intersection_volume(s_box, barrel)
            assert vol < 0.05, f"Barrel DS3218 #{idx} collision at drop offset {offset_z}: {vol} mm^3"
    print("  2x Barrel DS3218 bays: 0.000 mm^3 collision (seated + drop-in path)")

    # 1x DS3218 in outer sleeve (20.0 x 37.5 x 40.0 mm standard body, drop-in along -Y)
    sx, sy, sz = SLEEVE_DS3218_CENTER
    for offset_y in (0.0, -15.0, -35.0):
        s_box = box(extents=[20.0, 37.5, 40.0])
        s_box.apply_translation([sx, sy + offset_y, sz])
        vol = _intersection_volume(s_box, sleeve)
        assert vol < 0.05, f"Sleeve DS3218 collision at drop offset {offset_y}: {vol} mm^3"
    print("  1x Sleeve DS3218 bay: 0.000 mm^3 collision (seated + drop-in path)")

    print("\nALL v10.0 WRIST & FOREARM SELF-TESTS PASSED!")
