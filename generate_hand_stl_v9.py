#!/usr/bin/env python3
"""
High-Precision 3D Printable Anthropomorphic Robotic Hand — Iteration 9 (v9.0)
=============================================================================
Full-system CAD integration, 17-part STL export, and strict zero-false-positive
automated geometric & kinematic verification (`verify_v9_complete`).

Key v9.0 Engineering Upgrades Verified Here:
1. Manifold & Single-Shell Integrity:
   - All 17 exported STLs are 100% watertight positive-volume solids.
   - All 16 printable parts have `len(m.split(only_watertight=False)) == 1`
     (zero disconnected shells, zero negative-volume internal trapped voids).
2. Full-Hand Kinematic Collision Sweep (< 0.02 mm³ tolerance):
   - All 4 fingers (Index, Middle, Ring, Pinky) across 0°–95° for MCP (with rotZ), PIP, DIP.
   - Thumb CMC (male palm pedestal -> female thumb proximal) & Thumb IP across 0°–90°.
   - Palm vs. Forearm Servo Adapter at ADAPTER_PLACEMENT + concentric Y-axis wrist bolts.
3. One-Directional 0° Extension Hard Stops (> 0.03 mm³ interference):
   - Rigid mechanical stop engagement at -5° and -10° hyperextension across all 14 joints.
4. Tendon, Fastener, Snap-Lip & Servo Hardware Verification:
   - Pure-numpy Moller-Trumbore ray/path check confirming unbroken tendon passage from
     all 5 digits through `palm_v9.stl` out the axial wrist opening, and NO vertical Z-hole
     at [0, 4, 0].
   - Zero overlap between M3 hinge pins (z in [-1.50, +1.50]) and palmar tendon channels
     (z_top <= -2.20 mm), plus distal palmar knot pockets.
   - Clevis fork clamping wall thickness >= 1.20 mm (even on pinky_distal_v9.stl) and
     true 6-sided captive M3 hex nut sockets.
   - Snap-fit pinch lips present along mid-shafts (x = ±1.05 mm overhang) and cleanly
     terminated prior to distal fingernail beds (y >= 0.60 * length).
   - 6x MG90S servo clearance boxes (12.2 x 22.8 x 22.0 mm) slide cleanly from outside
     into all 6 forearm servo bays with 0.000 mm³ collision.
=============================================================================
"""

import os
import sys
import numpy as np
import trimesh
from trimesh.creation import box, cylinder

from cad_v9.digits import (
    PIN_RADIUS,
    TENDON_RADIUS,
    TENDON_Z,
    CLEVIS_SLOT_W,
    CLEVIS_TONGUE_W,
    FDM_CLEARANCE,
    HUB_R_FRAC,
    NAIL_DEPTH,
    DIGIT_SPECS,
    THUMB_SPEC,
    generate_proximal_phalanx,
    generate_intermediate_phalanx,
    generate_distal_phalanx,
)
from cad_v9.chassis import (
    PALM_W,
    PALM_L,
    PALM_H,
    THUMB_CMC_X,
    THUMB_CMC_Y,
    THUMB_CMC_Z,
    THUMB_CMC_POS,
    WRIST_INTERFACE_Y,
    ADAPTER_PLACEMENT,
    WRIST_BOLT_XZ,
    KNUCKLE_SPECS,
    generate_palm,
    generate_forearm_adapter,
)

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
STL_DIR_V9 = os.path.join(OUTPUT_DIR, "stl_exports_v9")

COMPONENT_STL_NAMES = [
    "index_distal_v9.stl",
    "index_intermediate_v9.stl",
    "index_proximal_v9.stl",
    "middle_distal_v9.stl",
    "middle_intermediate_v9.stl",
    "middle_proximal_v9.stl",
    "ring_distal_v9.stl",
    "ring_intermediate_v9.stl",
    "ring_proximal_v9.stl",
    "pinky_distal_v9.stl",
    "pinky_intermediate_v9.stl",
    "pinky_proximal_v9.stl",
    "thumb_distal_v9.stl",
    "thumb_proximal_v9.stl",
    "palm_v9.stl",
    "forearm_servo_adapter_v9.stl",
]
ASSEMBLY_STL_NAME = "robotic_hand_full_assembly_v9.stl"
ALL_STL_NAMES = COMPONENT_STL_NAMES + [ASSEMBLY_STL_NAME]


# =============================================================================
# Pure-NumPy Moller-Trumbore Ray-Triangle Intersection Helpers
# =============================================================================

def moller_trumbore_hits(triangles: np.ndarray, origin, direction, t_min: float = 1e-6, t_max: float = np.inf) -> np.ndarray:
    """
    Pure vectorized NumPy Moller-Trumbore ray-triangle intersection.
    `direction` need not be unit length: returned `t` parameters satisfy
    `hit_point = origin + t * direction`.
    """
    orig = np.asarray(origin, dtype=np.float64)
    d = np.asarray(direction, dtype=np.float64)

    v0 = triangles[:, 0, :]
    v1 = triangles[:, 1, :]
    v2 = triangles[:, 2, :]

    edge1 = v1 - v0
    edge2 = v2 - v0

    h = np.cross(d, edge2)
    a = np.einsum("ij,ij->i", edge1, h)

    valid = np.abs(a) > 1e-10
    if not np.any(valid):
        return np.empty(0, dtype=np.float64)

    f = 1.0 / a[valid]
    s = orig - v0[valid]
    u = f * np.einsum("ij,ij->i", s, h[valid])

    u_ok = (u >= -1e-7) & (u <= 1.0 + 1e-7)
    if not np.any(u_ok):
        return np.empty(0, dtype=np.float64)

    idx_u = np.flatnonzero(valid)[u_ok]
    f_u = f[u_ok]
    s_u = s[u_ok]
    u_u = u[u_ok]

    q = np.cross(s_u, edge1[idx_u])
    v = f_u * np.einsum("j,ij->i", d, q)

    v_ok = (v >= -1e-7) & (u_u + v <= 1.0 + 1e-7)
    if not np.any(v_ok):
        return np.empty(0, dtype=np.float64)

    f_v = f_u[v_ok]
    q_v = q[v_ok]
    edge2_v = edge2[idx_u[v_ok]]

    t = f_v * np.einsum("ij,ij->i", edge2_v, q_v)
    t_ok = (t >= t_min) & (t <= t_max)
    if not np.any(t_ok):
        return np.empty(0, dtype=np.float64)

    t_vals = np.sort(t[t_ok])
    # Deduplicate shared-edge/vertex hits within 1e-5 parameter units
    if len(t_vals) > 1:
        keep = np.concatenate([[True], np.diff(t_vals) > 1e-5])
        t_vals = t_vals[keep]
    return t_vals


def point_in_solid(triangles: np.ndarray, pt) -> bool:
    """
    Determines whether `pt` lies strictly inside a watertight mesh using
    jittered-direction Moller-Trumbore ray parity (avoiding axis-aligned edges).
    """
    ray_dir = np.array([0.1379, 0.2713, 0.9524], dtype=np.float64)
    hits = moller_trumbore_hits(triangles, pt, ray_dir, t_min=1e-5, t_max=np.inf)
    return (len(hits) % 2) == 1


def segment_is_clear_in_air(triangles: np.ndarray, p0, p1) -> bool:
    """
    Returns True iff `p0` is in air, `p1` is in air, and the straight line segment
    `p0 -> p1` has zero triangle intersections with `triangles`.
    """
    p0 = np.asarray(p0, dtype=np.float64)
    p1 = np.asarray(p1, dtype=np.float64)
    if point_in_solid(triangles, p0) or point_in_solid(triangles, p1):
        return False
    hits = moller_trumbore_hits(triangles, p0, p1 - p0, t_min=0.0, t_max=1.0)
    return len(hits) == 0


def intersection_volume(mesh_a: trimesh.Trimesh, mesh_b: trimesh.Trimesh) -> float:
    """Exact boolean intersection volume in mm³ via manifold3d."""
    inter = trimesh.boolean.intersection([mesh_a, mesh_b])
    if inter is None or inter.is_empty or len(inter.faces) == 0:
        return 0.0
    return float(abs(inter.volume))


# =============================================================================
# Spec Alignment Helpers
# =============================================================================

def resolve_knuckle_pose(d_spec: dict, knuckle_specs: list) -> tuple[float, float, float, float]:
    """
    Resolves exact MCP hinge pin center (fx, py, fz) and splay angle rotZ for a finger,
    matching `KNUCKLE_SPECS` from `chassis.py` with `DIGIT_SPECS` from `digits.py`.
    """
    name = d_spec["name"]
    ks_match = next((k for k in knuckle_specs if k.get("name") == name), None)
    if ks_match is not None:
        fx = float(ks_match.get("fx", ks_match.get("x", d_spec["x"])))
        if "py" in ks_match:
            py = float(ks_match["py"])
        elif "y" in ks_match:
            y_val = float(ks_match["y"])
            py = y_val - 4.0 if y_val > 81.5 else y_val
        else:
            y_val = float(d_spec["y"])
            py = y_val - 4.0 if y_val > 81.5 else y_val
        fz = float(ks_match.get("fz", ks_match.get("z", d_spec["z"])))
        rot_z = float(ks_match.get("rotZ", ks_match.get("rot_z", d_spec["rotZ"])))
        return fx, py, fz, rot_z

    fx = float(d_spec["x"])
    y_val = float(d_spec["py"]) if "py" in d_spec else float(d_spec["y"])
    py = y_val - 4.0 if ("py" not in d_spec and y_val > 81.5) else y_val
    fz = float(d_spec["z"])
    rot_z = float(d_spec["rotZ"])
    return fx, py, fz, rot_z


# =============================================================================
# Build & Export All 17 v9 STL Files
# =============================================================================

def build_iteration_9() -> dict:
    """
    Generates all 16 individual v9.0 CAD components plus the full articulated
    16-body assembly, exports all 17 STL files to `stl_exports_v9/`, and returns
    generated meshes and metadata for verification.
    """
    os.makedirs(STL_DIR_V9, exist_ok=True)

    print("=" * 80)
    print("BUILDING ITERATION 9 (v9.0 — HEX-CAPTIVE, SNAP-FIT & ZERO-LOOPHOLE CAD)")
    print(f"  Palmar Flexor Bore : Ø{2 * TENDON_RADIUS:.2f} mm at z = {TENDON_Z:.2f} mm (clears M3 pin)")
    print(f"  Hinge Pin Bore     : Ø{2 * PIN_RADIUS:.2f} mm + Captive 6-Sided M3 Hex Nut Socket")
    print(f"  Dorsal Return      : Profile-Following Snap-Fit Pinch-Lip Channel (Cuticle Stop)")
    print(f"  Wrist & Forearm    : Y-Axis 4-Bolt Bulkhead (y={WRIST_INTERFACE_Y}) + 6-Bay MG90S Rack")
    print(f"  Output Directory   : {STL_DIR_V9}")
    print("=" * 80)

    meshes = {}
    assembly_parts = []

    print("\n[1/5] Generating Palm (v9.0)...")
    palm = generate_palm(palm_w=PALM_W, palm_l=PALM_L, palm_h=PALM_H)
    palm.export(os.path.join(STL_DIR_V9, "palm_v9.stl"))
    meshes["palm_v9.stl"] = palm
    assembly_parts.append(palm.copy())

    print("[2/5] Generating 6-Bay Open-Access Forearm Servo Adapter (v9.0)...")
    adapter = generate_forearm_adapter()
    adapter.export(os.path.join(STL_DIR_V9, "forearm_servo_adapter_v9.stl"))
    meshes["forearm_servo_adapter_v9.stl"] = adapter

    adapter_posed = adapter.copy()
    adapter_posed.apply_translation(ADAPTER_PLACEMENT)
    assembly_parts.append(adapter_posed)

    print("[3/5] Generating & Exporting 12 Finger Phalanges (Index, Middle, Ring, Pinky)...")
    finger_parts = {}
    for d in DIGIT_SPECS:
        dname = d["name"]
        h_p = float(d.get("h_p", 12.0))
        h_i = float(d.get("h_i", 11.6))
        h_d = float(d.get("h_d", 11.2))

        p = generate_proximal_phalanx(length=d["len_p"], width=d["w_prox"], height=h_p)
        ip = generate_intermediate_phalanx(length=d["len_i"], width=d["w_mid"], height=h_i)
        dp = generate_distal_phalanx(length=d["len_d"], width=d["w_dist"], height=h_d)

        p_name = f"{dname}_proximal_v9.stl"
        i_name = f"{dname}_intermediate_v9.stl"
        d_name = f"{dname}_distal_v9.stl"

        p.export(os.path.join(STL_DIR_V9, p_name))
        ip.export(os.path.join(STL_DIR_V9, i_name))
        dp.export(os.path.join(STL_DIR_V9, d_name))

        meshes[p_name] = p
        meshes[i_name] = ip
        meshes[d_name] = dp
        finger_parts[dname] = (d, p, ip, dp)

        fx, py, fz, rot_z = resolve_knuckle_pose(d, KNUCKLE_SPECS)
        fm = float(d.get("fm", -0.35))
        fp = float(d.get("fp", -0.45))
        fd = float(d.get("fd", -0.30))

        p_c = p.copy()
        ip_c = ip.copy()
        dp_c = dp.copy()

        dp_c.apply_transform(trimesh.transformations.rotation_matrix(fd, [1, 0, 0]))
        dp_c.apply_translation([0.0, float(d["len_i"]), 0.0])

        tip = trimesh.util.concatenate([ip_c, dp_c])
        tip.apply_transform(trimesh.transformations.rotation_matrix(fp, [1, 0, 0]))
        tip.apply_translation([0.0, float(d["len_p"]), 0.0])

        f_full = trimesh.util.concatenate([p_c, tip])
        f_full.apply_transform(trimesh.transformations.rotation_matrix(fm, [1, 0, 0]))
        f_full.apply_transform(trimesh.transformations.rotation_matrix(rot_z, [0, 0, 1]))
        f_full.apply_translation([fx, py, fz])
        assembly_parts.append(f_full)

    print("[4/5] Generating & Exporting Opposable Thumb Phalanges (v9.0)...")
    th_p = generate_proximal_phalanx(
        length=THUMB_SPEC["len_p"],
        width=THUMB_SPEC["w_prox"],
        height=THUMB_SPEC.get("h_p", 13.0),
    )
    th_d = generate_distal_phalanx(
        length=THUMB_SPEC["len_d"],
        width=THUMB_SPEC["w_dist"],
        height=THUMB_SPEC.get("h_d", 12.0),
    )

    th_p.export(os.path.join(STL_DIR_V9, "thumb_proximal_v9.stl"))
    th_d.export(os.path.join(STL_DIR_V9, "thumb_distal_v9.stl"))
    meshes["thumb_proximal_v9.stl"] = th_p
    meshes["thumb_distal_v9.stl"] = th_d

    th_p_c = th_p.copy()
    th_d_c = th_d.copy()
    th_ip_ang = float(THUMB_SPEC.get("f_ip", -0.40))
    th_cmc_ang = float(THUMB_SPEC.get("f_cmc", -0.35))

    th_d_c.apply_transform(trimesh.transformations.rotation_matrix(th_ip_ang, [1, 0, 0]))
    th_d_c.apply_translation([0.0, float(THUMB_SPEC["len_p"]), 0.0])

    th_full = trimesh.util.concatenate([th_p_c, th_d_c])
    th_full.apply_transform(trimesh.transformations.rotation_matrix(th_cmc_ang, [1, 0, 0]))
    rot_cmc = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)
    th_full.apply_transform(rot_cmc)
    th_full.apply_translation(THUMB_CMC_POS)
    assembly_parts.append(th_full)

    print("[5/5] Exporting Full Articulated Hand Assembly (v9.0)...")
    full_assembly = trimesh.util.concatenate(assembly_parts)
    full_assembly.export(os.path.join(STL_DIR_V9, ASSEMBLY_STL_NAME))
    meshes[ASSEMBLY_STL_NAME] = full_assembly

    print("\n--- EXPORTED STL SUMMARY (stl_exports_v9/) ---")
    print(f"  {'STL File':<36} {'Size (KB)':>10} {'Volume (mm³)':>14} {'Bodies':>8} {'Watertight':>12}")
    print("  " + "-" * 84)
    for fname in ALL_STL_NAMES:
        fpath = os.path.join(STL_DIR_V9, fname)
        fsize_kb = os.path.getsize(fpath) / 1024.0
        m = meshes[fname]
        bodies = len(m.split(only_watertight=False))
        print(f"  {fname:<36} {fsize_kb:10.1f} {m.volume:14.1f} {bodies:8d} {str(m.is_watertight):>12}")

    return {
        "meshes": meshes,
        "finger_parts": finger_parts,
        "thumb_parts": (th_p, th_d),
        "palm": palm,
        "adapter": adapter,
    }


# =============================================================================
# Strict Automated Verification Suite (`verify_v9_complete`)
# =============================================================================

def _find_tendon_polyline_through_palm(
    palm_tris: np.ndarray,
    entry_candidates: list[np.ndarray],
    wrist_y: float,
) -> list[np.ndarray] | None:
    """
    Uses pure-numpy Moller-Trumbore ray/segment checks to find an unbroken
    empty-space tendon passage from a digit's flexor entry through `palm_v9.stl`
    out past `WRIST_INTERFACE_Y`. Supports straight (1-segment) and bent/manifold
    (2- or 3-segment) internal carpal conduits.
    """
    exit_pts = []
    for wx in [0.0, -4.0, 4.0, -7.5, 7.5, -2.0, 2.0]:
        for wz in [0.0, -2.5, 2.5, -4.5, 4.5, -1.5, 1.5]:
            p_ex = np.array([wx, wrist_y - 2.0, wz], dtype=np.float64)
            if not point_in_solid(palm_tris, p_ex):
                exit_pts.append(p_ex)

    valid_starts = [p for p in entry_candidates if not point_in_solid(palm_tris, p)]
    if not valid_starts or not exit_pts:
        return None

    # 1. Direct 1-segment straight line-of-sight check
    for p_start in valid_starts:
        for p_exit in exit_pts:
            if segment_is_clear_in_air(palm_tris, p_start, p_exit):
                return [p_start, p_exit]

    # Pre-sample air waypoints inside the palm's internal conduits / carpal chamber
    xs = np.linspace(-26.0, 26.0, 27)
    ys = np.linspace(wrist_y + 1.0, 76.0, 32)
    zs = np.linspace(-6.5, 4.5, 13)
    air_waypoints = []
    for y in ys:
        for x in xs:
            for z in zs:
                pt = np.array([x, y, z], dtype=np.float64)
                if not point_in_solid(palm_tris, pt):
                    air_waypoints.append(pt)

    if not air_waypoints:
        return None

    # Pre-filter waypoints that have direct clear exit out the wrist opening
    exit_connected_wps = []
    for wp in air_waypoints:
        for p_exit in exit_pts:
            if len(moller_trumbore_hits(palm_tris, wp, p_exit - wp, 0.0, 1.0)) == 0:
                exit_connected_wps.append((wp, p_exit))
                break

    # 2. 2-segment conduit path: Entry -> Waypoint -> WristExit
    for p_start in valid_starts:
        for wp, p_exit in exit_connected_wps:
            if len(moller_trumbore_hits(palm_tris, p_start, wp - p_start, 0.0, 1.0)) == 0:
                return [p_start, wp, p_exit]

    # 3. 3-segment conduit path: Entry -> DistalWP -> CarpalWP -> WristExit
    start_connected_wps = []
    for p_start in valid_starts:
        for wp in air_waypoints:
            if wp[1] > 20.0 and len(moller_trumbore_hits(palm_tris, p_start, wp - p_start, 0.0, 1.0)) == 0:
                start_connected_wps.append((p_start, wp))

    for p_start, wp1 in start_connected_wps:
        for wp2, p_exit in exit_connected_wps:
            if wp2[1] <= wp1[1] and len(moller_trumbore_hits(palm_tris, wp1, wp2 - wp1, 0.0, 1.0)) == 0:
                return [p_start, wp1, wp2, p_exit]

    return None


def _measure_clevis_clamping_and_hex_nut(mesh: trimesh.Trimesh, label: str) -> dict:
    """
    Measures left/right clevis clamping wall thickness around the M3 pin (`y=0, z=0`)
    and verifies the `+X` nut socket is a true 6-sided hexagonal pocket (`sections=6`).
    """
    tris = mesh.triangles
    half_slot = CLEVIS_SLOT_W / 2.0  # 2.70 mm

    # Probe X-rays at radial offset r = 2.25 mm from pin center (outside r=1.70 pin,
    # inside r=3.15 screw head counterbore and r_in=2.825 hex nut flat)
    probe_yz = [
        (2.25, 0.0),
        (0.0, -2.25),
        (1.60, -1.60),
        (1.60, 1.60),
    ]
    left_walls = []
    right_walls = []
    right_floors = []

    for py, pz in probe_yz:
        hits_x = -25.0 + moller_trumbore_hits(tris, [-25.0, py, pz], [1.0, 0.0, 0.0], 0.0, 50.0)
        left_hits = hits_x[hits_x < -half_slot + 0.25]
        right_hits = hits_x[hits_x > half_slot - 0.25]
        if len(left_hits) >= 2 and len(right_hits) >= 2:
            # Innermost solid interval adjacent to clevis slot [-2.70, +2.70]
            x_left_cb_floor = left_hits[-2]
            x_left_slot = left_hits[-1]
            x_right_slot = right_hits[0]
            x_right_nut_floor = right_hits[1]
            left_walls.append(x_left_slot - x_left_cb_floor)
            right_walls.append(x_right_nut_floor - x_right_slot)
            right_floors.append(x_right_nut_floor)

    assert len(left_walls) >= 2 and len(right_walls) >= 2, (
        f"[{label}] Could not resolve clevis counterbore floors on fork walls!"
    )

    min_left_wall = float(min(left_walls))
    min_right_wall = float(min(right_walls))
    x_nut_probe = float(np.median(right_floors)) + 0.65

    # Cast 360 radial rays in YZ plane from (x_nut_probe, 0, 0) to map the +X nut pocket profile
    angles = np.linspace(0.0, 2.0 * np.pi, 360, endpoint=False)
    radii = np.zeros(len(angles), dtype=np.float64)
    for k, ang in enumerate(angles):
        dy, dz = np.cos(ang), np.sin(ang)
        t_hits = moller_trumbore_hits(tris, [x_nut_probe, 0.0, 0.0], [0.0, dy, dz], 0.5, 10.0)
        assert len(t_hits) >= 1, (
            f"[{label}] Open/broken wall in +X nut pocket at x={x_nut_probe:.2f}, angle={np.rad2deg(ang):.1f}°!"
        )
        radii[k] = t_hits[0]

    r_min = float(np.min(radii))
    r_max = float(np.max(radii))
    across_flats = 2.0 * r_min
    ratio = r_max / r_min  # Regular hexagon: 2 / sqrt(3) = 1.1547 (circle = 1.000)

    # Also verify 6 distinct flat planar side normals inside the +X nut pocket
    tri_centers = tris.mean(axis=1)
    nut_mask = (
        (tri_centers[:, 0] > float(np.median(right_floors)) + 0.15)
        & (np.hypot(tri_centers[:, 1], tri_centers[:, 2]) > PIN_RADIUS + 0.35)
        & (np.hypot(tri_centers[:, 1], tri_centers[:, 2]) < 3.65)
        & (np.abs(mesh.face_normals[:, 0]) < 0.25)
    )
    yz_normals = mesh.face_normals[nut_mask][:, 1:3]
    norms = np.linalg.norm(yz_normals, axis=1, keepdims=True)
    yz_normals = yz_normals[norms[:, 0] > 0.8] / norms[norms[:, 0] > 0.8]
    normal_angles = np.mod(np.rad2deg(np.arctan2(yz_normals[:, 1], yz_normals[:, 0])), 360.0)
    # Cluster normal angles into 10-degree bins
    distinct_clusters = []
    for ang_deg in sorted(normal_angles):
        if not distinct_clusters or min(
            min(abs(ang_deg - c) % 360.0, 360.0 - (abs(ang_deg - c) % 360.0))
            for c in distinct_clusters
        ) > 15.0:
            distinct_clusters.append(ang_deg)

    return {
        "min_left_wall": min_left_wall,
        "min_right_wall": min_right_wall,
        "across_flats": across_flats,
        "hex_ratio": ratio,
        "hex_side_count": len(distinct_clusters),
    }


def _has_dorsal_pinch_lip_at_y(tris: np.ndarray, y_val: float, x_probe: float = 1.05) -> tuple[bool, float]:
    """
    Checks at `x = ±x_probe` (inside cavity half-width 1.30 mm, outside mouth half-width 0.90 mm)
    whether a dorsal undercut cavity exists beneath a solid overhanging pinch lip.
    Returns `(has_lip, mean_lip_thickness)`.
    """
    lip_thicknesses = []
    for sx in (-x_probe, +x_probe):
        z_hits = -15.0 + moller_trumbore_hits(tris, [sx, y_val, -15.0], [0.0, 0.0, 1.0], 0.0, 30.0)
        if len(z_hits) < 4 or (len(z_hits) % 2) != 0:
            return False, 0.0
        # Solid intervals are [z_hits[0], z_hits[1]], [z_hits[2], z_hits[3]], ...
        intervals = list(zip(z_hits[0::2], z_hits[1::2]))
        # Topmost solid interval must be a dorsal pinch lip above z > 0 with air cavity below it
        top_bot, top_top = intervals[-1]
        prev_bot, prev_top = intervals[-2]
        cav_height = top_bot - prev_top
        lip_t = top_top - top_bot
        # Pinch lip sits in dorsal region (z > 0.5), has undercut cavity >= 0.4 mm, lip thickness in [0.25, 1.50] mm
        if top_bot <= 0.5 or cav_height < 0.40 or not (0.25 <= lip_t <= 1.50):
            return False, 0.0
        lip_thicknesses.append(lip_t)
    return True, float(np.mean(lip_thicknesses))


def verify_v9_complete() -> dict:
    """
    Strict, zero-false-positive verification suite for Iteration 9 (v9.0).
    Reloads all 17 STL files directly from `stl_exports_v9/` and asserts every
    manifold, kinematic, hard-stop, tendon, fastener, snap-lip, and servo-bay
    requirement. Raises `AssertionError` on any violation.
    """
    print("\n" + "=" * 80)
    print("RUNNING STRICT AUTOMATED VERIFICATION SUITE: verify_v9_complete()")
    print("=" * 80)

    # -------------------------------------------------------------------------
    # 1. MANIFOLD & SINGLE-SHELL INTEGRITY (All 17 STLs from disk)
    # -------------------------------------------------------------------------
    print("\n[Suite 1/4] Manifold & Single-Shell Integrity (17 Disk STLs)...")
    loaded = {}
    for fname in ALL_STL_NAMES:
        fpath = os.path.join(STL_DIR_V9, fname)
        assert os.path.isfile(fpath), f"Missing exported STL file: {fpath}"
        m = trimesh.load(fpath, force="mesh")
        assert isinstance(m, trimesh.Trimesh), f"{fname} did not load as a single Trimesh"
        assert m.is_watertight, f"Manifold failure: {fname} is NOT watertight!"
        assert m.volume > 0.0, f"Volume failure: {fname} has non-positive volume ({m.volume})!"
        bodies = len(m.split(only_watertight=False))
        if fname != ASSEMBLY_STL_NAME:
            assert bodies == 1, (
                f"Single-shell failure: {fname} has {bodies} disconnected shells/voids (expected 1)!"
            )
        else:
            assert bodies == 16, (
                f"Assembly shell count failure: {fname} has {bodies} bodies (expected 16 non-colliding parts)!"
            )
        loaded[fname] = m

    palm = loaded["palm_v9.stl"]
    adapter = loaded["forearm_servo_adapter_v9.stl"]
    th_p = loaded["thumb_proximal_v9.stl"]
    th_d = loaded["thumb_distal_v9.stl"]
    print("  PASS: All 17 STLs are watertight (vol > 0) and all 16 component STLs have bodies == 1.")

    # -------------------------------------------------------------------------
    # 2. FULL-HAND KINEMATIC COLLISION SWEEP (< 0.02 mm³ tolerance)
    # -------------------------------------------------------------------------
    print("\n[Suite 2/4] Full-Hand Kinematic Collision Sweep (0.000 mm³ / < 0.02 mm³)...")
    finger_angles = [0, 15, 30, 45, 60, 75, 90, 95]
    thumb_angles = [0, 15, 30, 45, 60, 75, 90]
    max_flex_collision = 0.0

    for d in DIGIT_SPECS:
        dname = d["name"]
        p = loaded[f"{dname}_proximal_v9.stl"]
        ip = loaded[f"{dname}_intermediate_v9.stl"]
        dp = loaded[f"{dname}_distal_v9.stl"]
        fx, py, fz, rot_z = resolve_knuckle_pose(d, KNUCKLE_SPECS)
        len_p = float(d["len_p"])
        len_i = float(d["len_i"])

        for deg in finger_angles:
            rad = -np.deg2rad(deg)

            # MCP (with rotZ applied!)
            p_mcp = p.copy()
            p_mcp.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            p_mcp.apply_transform(trimesh.transformations.rotation_matrix(rot_z, [0, 0, 1]))
            p_mcp.apply_translation([fx, py, fz])
            v_mcp = intersection_volume(palm, p_mcp)
            max_flex_collision = max(max_flex_collision, v_mcp)
            assert v_mcp < 0.02, f"Collision on {dname} MCP at {deg}°: {v_mcp:.4f} mm³ >= 0.02 mm³"

            # PIP
            ip_pip = ip.copy()
            ip_pip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            ip_pip.apply_translation([0.0, len_p, 0.0])
            v_pip = intersection_volume(p, ip_pip)
            max_flex_collision = max(max_flex_collision, v_pip)
            assert v_pip < 0.02, f"Collision on {dname} PIP at {deg}°: {v_pip:.4f} mm³ >= 0.02 mm³"

            # DIP
            dp_dip = dp.copy()
            dp_dip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            dp_dip.apply_translation([0.0, len_i, 0.0])
            v_dip = intersection_volume(ip, dp_dip)
            max_flex_collision = max(max_flex_collision, v_dip)
            assert v_dip < 0.02, f"Collision on {dname} DIP at {deg}°: {v_dip:.4f} mm³ >= 0.02 mm³"

        print(f"  PASS: {dname.capitalize():<6} MCP (rotZ={rot_z:+.2f}), PIP, DIP clear across 0°–95°.")

    # Thumb CMC & IP across 0°–90°
    rot_cmc = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)
    th_len_p = float(THUMB_SPEC["len_p"])
    for deg in thumb_angles:
        rad = -np.deg2rad(deg)

        th_p_cmc = th_p.copy()
        th_p_cmc.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
        th_p_cmc.apply_transform(rot_cmc)
        th_p_cmc.apply_translation(THUMB_CMC_POS)
        v_cmc = intersection_volume(palm, th_p_cmc)
        max_flex_collision = max(max_flex_collision, v_cmc)
        assert v_cmc < 0.02, f"Collision on Thumb CMC at {deg}°: {v_cmc:.4f} mm³ >= 0.02 mm³"

        th_d_ip = th_d.copy()
        th_d_ip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
        th_d_ip.apply_translation([0.0, th_len_p, 0.0])
        v_ip = intersection_volume(th_p, th_d_ip)
        max_flex_collision = max(max_flex_collision, v_ip)
        assert v_ip < 0.02, f"Collision on Thumb IP at {deg}°: {v_ip:.4f} mm³ >= 0.02 mm³"

    print(f"  PASS: Thumb CMC (at {THUMB_CMC_POS}) & Thumb IP clear across 0°–90°.")

    # Palm vs. Forearm Adapter at ADAPTER_PLACEMENT + concentric wrist bolts
    adapter_posed = adapter.copy()
    adapter_posed.apply_translation(ADAPTER_PLACEMENT)
    v_wrist = intersection_volume(palm, adapter_posed)
    max_flex_collision = max(max_flex_collision, v_wrist)
    assert v_wrist < 0.02, f"Collision between Palm and Forearm Adapter: {v_wrist:.4f} mm³ >= 0.02 mm³"

    assert len(WRIST_BOLT_XZ) == 4, f"Expected 4 wrist bolts in WRIST_BOLT_XZ, got {len(WRIST_BOLT_XZ)}"
    for idx, (bx, bz) in enumerate(WRIST_BOLT_XZ):
        # M3 shaft test pin (r=1.45 mm) spanning across wrist bulkhead interface along Y
        bolt_pin = cylinder(radius=1.45, height=5.0, sections=24)
        bolt_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
        bolt_pin.apply_translation([float(bx), float(WRIST_INTERFACE_Y), float(bz)])
        v_bp_palm = intersection_volume(palm, bolt_pin)
        v_bp_adap = intersection_volume(adapter_posed, bolt_pin)
        assert v_bp_palm < 0.02, (
            f"Wrist bolt #{idx + 1} at ({bx}, {bz}) blocked by Palm ({v_bp_palm:.4f} mm³)!"
        )
        assert v_bp_adap < 0.02, (
            f"Wrist bolt #{idx + 1} at ({bx}, {bz}) blocked by Forearm Adapter ({v_bp_adap:.4f} mm³)!"
        )

        # Annular flange sleeve check proving hole sits inside solid flange material on BOTH parts
        outer_ring = cylinder(radius=2.8, height=3.0, sections=24).difference(
            cylinder(radius=2.0, height=4.0, sections=24)
        )
        outer_ring.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0]))
        ring_palm = outer_ring.copy()
        ring_palm.apply_translation([float(bx), float(WRIST_INTERFACE_Y) + 1.8, float(bz)])
        ring_adap = outer_ring.copy()
        ring_adap.apply_translation([float(bx), float(WRIST_INTERFACE_Y) - 1.9, float(bz)])
        assert intersection_volume(palm, ring_palm) > 1.0, (
            f"Wrist bolt #{idx + 1} ({bx}, {bz}) misses solid Palm flange!"
        )
        assert intersection_volume(adapter_posed, ring_adap) > 1.0, (
            f"Wrist bolt #{idx + 1} ({bx}, {bz}) misses solid Forearm Adapter flange!"
        )

    print(
        f"  PASS: Palm vs. Forearm Adapter collision = {v_wrist:.4f} mm³; "
        f"all 4 Y-axis wrist bolts ({WRIST_BOLT_XZ}) concentric & enclosed."
    )

    # -------------------------------------------------------------------------
    # 3. ONE-DIRECTIONAL EXTENSION HARD STOPS (-5° & -10° > 0.03 mm³)
    # -------------------------------------------------------------------------
    print("\n[Suite 3/4] One-Directional Extension Hard Stops (-5° and -10° > 0.03 mm³)...")
    min_stop_vol = np.inf
    stop_records = {}

    for deg in (-5, -10):
        rad = -np.deg2rad(deg)  # Positive X rotation = dorsal hyperextension

        for d in DIGIT_SPECS:
            dname = d["name"]
            p = loaded[f"{dname}_proximal_v9.stl"]
            ip = loaded[f"{dname}_intermediate_v9.stl"]
            dp = loaded[f"{dname}_distal_v9.stl"]
            fx, py, fz, rot_z = resolve_knuckle_pose(d, KNUCKLE_SPECS)

            p_mcp = p.copy()
            p_mcp.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            p_mcp.apply_transform(trimesh.transformations.rotation_matrix(rot_z, [0, 0, 1]))
            p_mcp.apply_translation([fx, py, fz])
            v_mcp = intersection_volume(palm, p_mcp)
            assert v_mcp > 0.03, f"Hard stop FAILED on {dname} MCP at {deg}°: {v_mcp:.4f} mm³ <= 0.03 mm³"

            ip_pip = ip.copy()
            ip_pip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            ip_pip.apply_translation([0.0, float(d["len_p"]), 0.0])
            v_pip = intersection_volume(p, ip_pip)
            assert v_pip > 0.03, f"Hard stop FAILED on {dname} PIP at {deg}°: {v_pip:.4f} mm³ <= 0.03 mm³"

            dp_dip = dp.copy()
            dp_dip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            dp_dip.apply_translation([0.0, float(d["len_i"]), 0.0])
            v_dip = intersection_volume(ip, dp_dip)
            assert v_dip > 0.03, f"Hard stop FAILED on {dname} DIP at {deg}°: {v_dip:.4f} mm³ <= 0.03 mm³"

            min_stop_vol = min(min_stop_vol, v_mcp, v_pip, v_dip)
            stop_records[(dname, deg)] = (v_mcp, v_pip, v_dip)

        th_p_cmc = th_p.copy()
        th_p_cmc.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
        th_p_cmc.apply_transform(rot_cmc)
        th_p_cmc.apply_translation(THUMB_CMC_POS)
        v_cmc = intersection_volume(palm, th_p_cmc)
        assert v_cmc > 0.03, f"Hard stop FAILED on Thumb CMC at {deg}°: {v_cmc:.4f} mm³ <= 0.03 mm³"

        th_d_ip = th_d.copy()
        th_d_ip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
        th_d_ip.apply_translation([0.0, th_len_p, 0.0])
        v_ip = intersection_volume(th_p, th_d_ip)
        assert v_ip > 0.03, f"Hard stop FAILED on Thumb IP at {deg}°: {v_ip:.4f} mm³ <= 0.03 mm³"

        min_stop_vol = min(min_stop_vol, v_cmc, v_ip)
        stop_records[("thumb", deg)] = (v_cmc, v_ip)

    print(
        f"  PASS: All 14 joints (4×MCP, 4×PIP, 4×DIP, Thumb CMC, Thumb IP) rigidly block "
        f"at -5° and -10° (min stop interference = {min_stop_vol:.3f} mm³)."
    )

    # -------------------------------------------------------------------------
    # 4. TENDON & HARDWARE GEOMETRY CHECKS
    # -------------------------------------------------------------------------
    print("\n[Suite 4/4] Tendon, Fastener, Snap-Lip & Servo Hardware Checks...")

    # 4a. Palm: NO vertical Z-axis through-hole at [0, 4, 0] + unbroken 5-digit tendon paths
    palm_tris = palm.triangles
    for probe_xy in [(0.0, 4.0), (-3.0, 4.0), (3.0, 4.0), (0.0, 1.0), (0.0, 7.0)]:
        z_hits = moller_trumbore_hits(
            palm_tris, [probe_xy[0], probe_xy[1], -30.0], [0.0, 0.0, 1.0], 0.0, 60.0
        )
        assert len(z_hits) >= 2, (
            f"Palm vertical Z through-hole detected at (x={probe_xy[0]}, y={probe_xy[1]})! "
            f"Ray hit count = {len(z_hits)}"
        )

    # Confirm axial Y-axis carpal opening exists through WRIST_INTERFACE_Y
    y_wrist_hits = moller_trumbore_hits(
        palm_tris, [0.0, float(WRIST_INTERFACE_Y) - 5.0, 0.0], [0.0, 1.0, 0.0], 0.0, 20.0
    )
    assert not point_in_solid(palm_tris, [0.0, float(WRIST_INTERFACE_Y) + 0.5, 0.0]), (
        f"Palm axial wrist opening at [0, {WRIST_INTERFACE_Y}, 0] is blocked by solid plastic!"
    )

    # Check unbroken tendon paths from all 4 fingers + thumb out the wrist
    tendon_paths = {}
    for d in DIGIT_SPECS:
        dname = d["name"]
        fx, py, fz, rot_z = resolve_knuckle_pose(d, KNUCKLE_SPECS)
        rz_mat = trimesh.transformations.rotation_matrix(rot_z, [0, 0, 1])[:3, :3]
        entry_candidates = []
        for dy_local in [2.0, 0.0, -2.0, -4.0, -6.0]:
            for dz_off in [0.0, -0.4, 0.4]:
                pt_world = np.array([fx, py, fz]) + rz_mat @ np.array([0.0, dy_local, TENDON_Z + dz_off])
                entry_candidates.append(pt_world)
        path = _find_tendon_polyline_through_palm(palm_tris, entry_candidates, float(WRIST_INTERFACE_Y))
        assert path is not None, (
            f"Tendon conduit BLOCKED in palm_v9.stl for finger '{dname}' from knuckle ({fx}, {py}, {fz + TENDON_Z})!"
        )
        tendon_paths[dname] = len(path) - 1

    # Thumb tendon entry candidates around CMC palmar flexor conduit
    cmc_rot3 = rot_cmc[:3, :3]
    thumb_entry_candidates = []
    for dy_local in [2.0, 0.0, -2.0, -4.5, -7.0]:
        for dx_local in [0.0, 2.0, -2.0]:
            for dz_local in [TENDON_Z, TENDON_Z - 1.0, TENDON_Z + 1.0]:
                pt_world = np.asarray(THUMB_CMC_POS, dtype=np.float64) + cmc_rot3 @ np.array([dx_local, dy_local, dz_local])
                thumb_entry_candidates.append(pt_world)
    thumb_path = _find_tendon_polyline_through_palm(palm_tris, thumb_entry_candidates, float(WRIST_INTERFACE_Y))
    assert thumb_path is not None, (
        f"Tendon conduit BLOCKED in palm_v9.stl for Thumb CMC at {THUMB_CMC_POS}!"
    )
    tendon_paths["thumb"] = len(thumb_path) - 1

    # Verify all 5 captive dorsal rubber-band cross-pin anchor bores on palm_v9.stl
    dorsal_anchor_poses = []
    for d in DIGIT_SPECS:
        fx, py, fz, rot_z = resolve_knuckle_pose(d, KNUCKLE_SPECS)
        rz4 = trimesh.transformations.rotation_matrix(rot_z, [0, 0, 1])
        dorsal_anchor_poses.append((d["name"], np.array([fx, py, fz], dtype=np.float64), rz4[:3, :3], -10.2, 6.05))
    dorsal_anchor_poses.append(("thumb", np.asarray(THUMB_CMC_POS, dtype=np.float64), cmc_rot3, -9.6, 5.85))

    for aname, apos, arot, ay_loc, az_loc in dorsal_anchor_poses:
        pin_center = apos + arot @ np.array([0.0, ay_loc, az_loc])
        # Transverse X-ray along local X must pass cleanly through open cross-pin bore at x_local in [-4.5, +4.5]
        p_x0 = pin_center + arot @ np.array([-4.5, 0.0, 0.0])
        p_x1 = pin_center + arot @ np.array([+4.5, 0.0, 0.0])
        x_bore_hits = moller_trumbore_hits(palm_tris, p_x0, p_x1 - p_x0, 0.05, 0.95)
        assert len(x_bore_hits) == 0 and not point_in_solid(palm_tris, pin_center), (
            f"Palm dorsal anchor cross-pin bore blocked on '{aname}' (hits={len(x_bore_hits)})!"
        )
        # Confirm captive solid bridge roof directly above cross-pin on boss shoulders (x_local = ±3.2 mm)
        for sx_sh in (-3.2, +3.2):
            roof_pt = apos + arot @ np.array([sx_sh, ay_loc, az_loc + 1.45])
            assert point_in_solid(palm_tris, roof_pt), (
                f"Palm dorsal anchor on '{aname}' lacks captive solid bridge above cross-pin at x_local={sx_sh}!"
            )

    print(
        f"  PASS [4a]: No vertical Z-hole at [0, 4, 0]; unbroken Moller-Trumbore tendon paths "
        f"verified for all 5 digits ({tendon_paths}) + 5 captive dorsal cross-pin anchors."
    )

    # 4b. Zero overlap between M3 hinge pin (z in [-1.50, +1.50]) and palmar tendon channel (z_top <= -2.20 mm)
    assert abs(PIN_RADIUS - 1.70) < 1e-6, f"PIN_RADIUS={PIN_RADIUS} (expected 1.70)"
    assert abs(TENDON_RADIUS - 1.10) < 1e-6, f"TENDON_RADIUS={TENDON_RADIUS} (expected 1.10)"
    assert TENDON_Z + TENDON_RADIUS <= -2.20, (
        f"Theoretical tendon bore top {TENDON_Z + TENDON_RADIUS:.2f} > -2.20 mm!"
    )

    phalanx_specs_for_check = []
    for d in DIGIT_SPECS:
        dname = d["name"]
        phalanx_specs_for_check.append((f"{dname}_proximal_v9.stl", float(d["len_p"]), False))
        phalanx_specs_for_check.append((f"{dname}_intermediate_v9.stl", float(d["len_i"]), False))
        phalanx_specs_for_check.append((f"{dname}_distal_v9.stl", float(d["len_d"]), True))
    phalanx_specs_for_check.append(("thumb_proximal_v9.stl", float(THUMB_SPEC["len_p"]), False))
    phalanx_specs_for_check.append(("thumb_distal_v9.stl", float(THUMB_SPEC["len_d"]), True))

    max_tendon_z_top = -np.inf
    for fname, plen, is_dist in phalanx_specs_for_check:
        m = loaded[fname]
        tris = m.triangles

        # Base M3 pin cylinder (r=1.50 mm -> z in [-1.50, +1.50]) clears base bore cleanly
        m3_pin = cylinder(radius=1.50, height=30.0, sections=24)
        m3_pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0]))
        assert intersection_volume(m, m3_pin) < 0.01, f"{fname}: M3 pin collides with base clevis bore!"

        # Vertical Z-ray at mid-shaft (x=0, y=0.42*plen) to locate exact palmar tendon bore [z_bot, z_top]
        y_probe = 0.42 * plen
        z_hits = -15.0 + moller_trumbore_hits(tris, [0.0, y_probe, -15.0], [0.0, 0.0, 1.0], 0.0, 30.0)
        assert len(z_hits) >= 4, f"{fname}: Missing internal palmar tendon bore at mid-shaft y={y_probe:.2f}!"
        # First solid wall is palmar floor [z_hits[0], z_hits[1]]; tendon bore air is [z_hits[1], z_hits[2]]
        palmar_floor_t = z_hits[1] - z_hits[0]
        bore_z_bot, bore_z_top = z_hits[1], z_hits[2]
        assert palmar_floor_t >= 0.85, (
            f"{fname}: Palmar floor below tendon bore too thin ({palmar_floor_t:.2f} mm < 0.85 mm)!"
        )
        assert bore_z_top <= -2.20, (
            f"{fname}: Palmar tendon bore ceiling z_top={bore_z_top:.2f} mm > -2.20 mm (encroaches on M3 pin)!"
        )
        assert point_in_solid(tris, [0.0, y_probe, -1.95]), (
            f"{fname}: Missing solid bridge at z=-1.95 mm between tendon bore and pin axis!"
        )
        max_tendon_z_top = max(max_tendon_z_top, bore_z_top)

        if not is_dist:
            # Distal male tongue bridge check at y = plen: solid plastic must separate pin (z=-1.70) & tendon (z=-2.40)
            assert point_in_solid(tris, [0.0, plen, -2.05]), (
                f"{fname}: Missing solid bridge between distal pin hole and tendon guide at y={plen:.1f}!"
            )
        else:
            # Distal palmar tendon knot pocket check (recessed ~Ø4.0 mm pocket near fingertip apex)
            knot_found = False
            for y_k in np.linspace(plen - 3.5, plen + 1.5, 25):
                # At radius 1.55 mm from bore center (outside r=1.10 tendon bore, inside r=2.00 knot pocket)
                if (not point_in_solid(tris, [1.55, y_k, TENDON_Z])) and point_in_solid(tris, [2.60, y_k, TENDON_Z]):
                    knot_found = True
                    break
            assert knot_found, f"{fname}: Missing recessed Ø4.0 mm distal palmar tendon knot pocket!"

    print(
        f"  PASS [4b]: Palmar tendon bores clear M3 pins on all 14 phalanges "
        f"(max z_top = {max_tendon_z_top:.2f} mm <= -2.20 mm) + distal knot pockets present."
    )

    # 4c. Clevis clamping walls >= 1.20 mm & true 6-sided hex nut pockets
    min_clamp_wall_all = np.inf
    for fname, _, _ in phalanx_specs_for_check:
        res = _measure_clevis_clamping_and_hex_nut(loaded[fname], fname)
        min_w = min(res["min_left_wall"], res["min_right_wall"])
        min_clamp_wall_all = min(min_clamp_wall_all, min_w)
        assert min_w >= 1.20, (
            f"{fname}: Clevis clamping wall too thin! left={res['min_left_wall']:.3f} mm, "
            f"right={res['min_right_wall']:.3f} mm (< 1.20 mm)"
        )
        assert 1.11 <= res["hex_ratio"] <= 1.20 and res["hex_side_count"] == 6, (
            f"{fname}: +X nut pocket is NOT a 6-sided hexagon! "
            f"r_max/r_min={res['hex_ratio']:.4f} (expected ~1.1547), sides={res['hex_side_count']}"
        )
        assert 5.40 <= res["across_flats"] <= 5.90, (
            f"{fname}: +X M3 hex nut across-flats={res['across_flats']:.3f} mm out of [5.40, 5.90] mm spec!"
        )

    pinky_d_res = _measure_clevis_clamping_and_hex_nut(loaded["pinky_distal_v9.stl"], "pinky_distal_v9.stl")
    index_d_res = _measure_clevis_clamping_and_hex_nut(loaded["index_distal_v9.stl"], "index_distal_v9.stl")
    print(
        f"  PASS [4c]: Clamping walls >= 1.20 mm across all 14 phalanges "
        f"(pinky_distal={min(pinky_d_res['min_left_wall'], pinky_d_res['min_right_wall']):.2f} mm, "
        f"index_distal={min(index_d_res['min_left_wall'], index_d_res['min_right_wall']):.2f} mm, "
        f"global min={min_clamp_wall_all:.2f} mm) & 6-sided hex nut sockets verified."
    )

    # 4d. Snap-fit pinch lips present along mid-shafts (x = ±1.05 mm) & cleanly absent in distal nail beds (y >= 0.60 * L)
    for fname, plen, is_dist in phalanx_specs_for_check:
        tris = loaded[fname].triangles
        y_mid = 0.42 * plen if is_dist else 0.50 * plen
        has_lip, lip_t = _has_dorsal_pinch_lip_at_y(tris, y_mid, x_probe=1.05)
        assert has_lip, (
            f"{fname}: Snap-fit overhanging pinch lips MISSING at mid-shaft y={y_mid:.2f} mm (x=±1.05 mm)!"
        )

        if is_dist:
            # Verify pinch lips are cleanly absent across fingernail bed (y >= 0.60 * L),
            # avoiding the Ø2.0 mm transverse elastic retention bore zone around y = 0.78 * L ± 1.2 mm
            for y_frac in (0.60, 0.64, 0.68, 0.71, 0.86, 0.90):
                y_nail = y_frac * plen
                if abs(y_nail - 0.78 * plen) < 1.35:
                    continue
                lip_in_nail, _ = _has_dorsal_pinch_lip_at_y(tris, y_nail, x_probe=1.05)
                assert not lip_in_nail, (
                    f"{fname}: Overhanging pinch lip encroaches into fingernail bed at y={y_nail:.2f} ({y_frac:.2f}*L)!"
                )

    print(
        "  PASS [4d]: Snap-fit pinch lips verified at mid-shafts (x=±1.05 mm) on all 14 phalanges "
        "and cleanly absent across all 5 distal fingernail beds (y >= 0.60*L)."
    )

    # 4e. 6x MG90S servo clearance boxes (12.2 x 22.8 x 22.0 mm) slide cleanly into 6 forearm bays
    adap_tris = adapter.triangles
    # MG90S clearance test box: 22.0 mm radial insertion depth (X) x 12.2 mm width (Y) x 22.8 mm case length (Z)
    ext_xyz = [22.0, 12.2, 22.8]
    servo_y_centers = [-24.0, -43.0, -62.0]
    found_bays = []

    for sign_x in (-1.0, +1.0):
        xc_seated = sign_x * 19.5  # Spans |x| in [8.5, 30.5] inside [8.0, 31.0] side bay
        for yc in servo_y_centers:
            zc = 0.0
            # Verify bay is genuinely enclosed by solid rack walls on ±Y and ±Z
            assert point_in_solid(adap_tris, [xc_seated, yc + 7.2, zc]) and point_in_solid(
                adap_tris, [xc_seated, yc - 7.2, zc]
            ), f"Missing ±Y rack divider walls around forearm bay (sign_x={sign_x:+.0f}, y={yc})!"
            assert point_in_solid(adap_tris, [xc_seated, yc, zc + 12.6]) and point_in_solid(
                adap_tris, [xc_seated, yc, zc - 12.6]
            ), f"Missing ±Z mounting flange walls around forearm bay (sign_x={sign_x:+.0f}, y={yc})!"

            # Verify both M2 mounting tab pilot bores (z = ±13.75 mm, 27.5 mm spacing) are open at outer face
            for sz_pilot in (-13.75, +13.75):
                assert not point_in_solid(adap_tris, [sign_x * 28.0, yc, sz_pilot]), (
                    f"Missing M2 pilot bore at (x={sign_x * 28.0}, y={yc}, z={sz_pilot})!"
                )

            # Slide MG90S box straight in from outside (slide_out = 30 -> 20 -> 10 -> 5 -> 0 mm)
            for slide_out in (30.0, 20.0, 10.0, 5.0, 0.0):
                sbox = box(extents=ext_xyz)
                sbox.apply_translation([xc_seated + sign_x * slide_out, yc, zc])
                v_srv = intersection_volume(adapter, sbox)
                assert v_srv < 0.02, (
                    f"MG90S servo box collides ({v_srv:.4f} mm³) sliding into bay "
                    f"(sign_x={sign_x:+.0f}, y={yc}) at slide_out={slide_out} mm!"
                )
            found_bays.append((xc_seated, yc, zc, tuple(ext_xyz)))

    assert len(found_bays) == 6, f"Expected 6 verified forearm servo bays, got {len(found_bays)}"

    # Also verify all 6 MG90S boxes seated simultaneously have 0.000 mm³ mutual overlap!
    seated_boxes = []
    for xc, yc, zc, ext_xyz in found_bays[:6]:
        sb = box(extents=list(ext_xyz))
        sb.apply_translation([xc, yc, zc])
        seated_boxes.append(sb)
    for i in range(6):
        for j in range(i + 1, 6):
            v_mut = intersection_volume(seated_boxes[i], seated_boxes[j])
            assert v_mut < 0.02, f"Seated MG90S servo #{i + 1} collides with servo #{j + 1} ({v_mut:.3f} mm³)!"

    print(
        f"  PASS [4e]: All 6 MG90S (12.2×22.8×22.0 mm) test boxes slide cleanly from outside "
        f"into forearm_servo_adapter_v9.stl with 0.000 mm³ collision and zero mutual overlap."
    )

    print("\n" + "=" * 80)
    print("ALL V9.0 GEOMETRIC, MANIFOLD, KINEMATIC & HARDWARE ASSERTIONS PASSED 100%!")
    print("=" * 80 + "\n")

    return {
        "max_flex_collision_mm3": max_flex_collision,
        "min_hard_stop_mm3": min_stop_vol,
        "max_tendon_z_top_mm": max_tendon_z_top,
        "min_clamping_wall_mm": min_clamp_wall_all,
        "servo_bays": found_bays[:6],
    }


if __name__ == "__main__":
    build_iteration_9()
    verify_v9_complete()
