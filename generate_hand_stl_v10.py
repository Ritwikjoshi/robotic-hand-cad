#!/usr/bin/env python3
"""
High-Precision 3D Printable Anthropomorphic Robotic Hand — Iteration 10 (v10.0)
===============================================================================
Full-actuation 11-servo CAD integration, 27-part STL export (26 individual
printable components + 1 complete assembled hand & forearm system), and strict
automated geometric, kinematic, hard-stop & actuator-fit verification suite
(`verify_v10_complete`).
"""

import os
import numpy as np
import trimesh
from trimesh.creation import box

from cad_v9.digits import (
    PIN_RADIUS,
    TENDON_RADIUS,
    TENDON_Z,
    DIGIT_SPECS,
    THUMB_SPEC,
    generate_proximal_phalanx,
    generate_intermediate_phalanx,
    generate_distal_phalanx,
)
from cad_v10.palm_assembly import (
    WRIST_CENTER,
    CUP_HINGE_POS,
    PUSHROD_PALM_ANCHORS,
    THUMB_CMC_POS,
    THUMB_CMC_X,
    THUMB_CMC_Y,
    THUMB_CMC_Z,
    MCP_YOKE_SPECS,
    ABDUCTION_LINKAGE_POS,
    IN_PALM_SERVO_BAY,
    mcp_yoke_transform,
    generate_palm_radial,
    generate_palm_ulnar,
    generate_mcp_yoke,
    generate_abduction_linkage,
)
from cad_v10.wrist_forearm import (
    GEAR_MODULE,
    GEAR_Z_BARREL,
    GEAR_Z_PINION,
    GEAR_CENTER_DIST,
    PINION_AXIS_XZ,
    PINION_Y,
    BARREL_MG90S_CENTERS,
    BARREL_MG90S_BAY_EXTENTS,
    BARREL_DS3218_CENTERS,
    BARREL_DS3218_BAY_EXTENTS,
    SLEEVE_DS3218_CENTER,
    SLEEVE_DS3218_BAY_EXTENTS,
    generate_wrist_cardan_ring,
    generate_forearm_inner_barrel,
    generate_forearm_outer_sleeve,
    generate_pronation_pinion,
    generate_servo_spool,
)

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
STL_DIR_V10 = os.path.join(OUTPUT_DIR, "stl_exports_v10")

COMPONENT_STL_NAMES = [
    # 14 certified v9 phalanges re-exported for v10
    "index_distal_v10.stl",
    "index_intermediate_v10.stl",
    "index_proximal_v10.stl",
    "middle_distal_v10.stl",
    "middle_intermediate_v10.stl",
    "middle_proximal_v10.stl",
    "ring_distal_v10.stl",
    "ring_intermediate_v10.stl",
    "ring_proximal_v10.stl",
    "pinky_distal_v10.stl",
    "pinky_intermediate_v10.stl",
    "pinky_proximal_v10.stl",
    "thumb_distal_v10.stl",
    "thumb_proximal_v10.stl",
    # 4 MCP 2-DoF Cardan yokes
    "index_mcp_yoke_v10.stl",
    "middle_mcp_yoke_v10.stl",
    "ring_mcp_yoke_v10.stl",
    "pinky_mcp_yoke_v10.stl",
    # Palm & abduction assembly
    "abduction_linkage_v10.stl",
    "palm_radial_v10.stl",
    "palm_ulnar_v10.stl",
    # 2-DoF wrist & 9+1 servo forearm pronation system
    "wrist_cardan_ring_v10.stl",
    "forearm_inner_barrel_v10.stl",
    "forearm_outer_sleeve_v10.stl",
    "pronation_pinion_v10.stl",
    "servo_spool_v10.stl",
]
ASSEMBLY_STL_NAME = "robotic_hand_full_assembly_v10.stl"
ALL_STL_NAMES = COMPONENT_STL_NAMES + [ASSEMBLY_STL_NAME]

KOBRA2_BED_MM = (220.0, 220.0, 250.0)


def intersection_volume(mesh_a: trimesh.Trimesh, mesh_b: trimesh.Trimesh) -> float:
    """Exact boolean intersection volume in mm³ via manifold3d."""
    inter = trimesh.boolean.intersection([mesh_a, mesh_b])
    if inter is None or inter.is_empty or len(inter.faces) == 0:
        return 0.0
    return float(abs(inter.volume))


def _resolve_mcp_yoke_spec(digit_name: str) -> dict:
    for spec in MCP_YOKE_SPECS:
        if spec["name"] == digit_name:
            return spec
    raise KeyError(f"Missing MCP_YOKE_SPECS entry for {digit_name!r}")


def _place_mcp_yoke(yoke_mesh: trimesh.Trimesh, spec: dict, yaw_deg: float = 0.0) -> trimesh.Trimesh:
    """Places an MCP Cardan yoke in palm coordinates using `mcp_yoke_transform`."""
    m = yoke_mesh.copy()
    m.apply_transform(mcp_yoke_transform(spec, yaw_rad=np.deg2rad(yaw_deg)))
    return m


def _rotate_about_point(mesh: trimesh.Trimesh, angle_rad: float, axis, point) -> trimesh.Trimesh:
    m = mesh.copy()
    pt = np.asarray(point, dtype=np.float64)
    m.apply_translation(-pt)
    m.apply_transform(trimesh.transformations.rotation_matrix(angle_rad, axis))
    m.apply_translation(pt)
    return m


def build_iteration_10() -> dict:
    """
    Generates all 26 individual v10.0 CAD components plus the complete articulated
    assembly, exports all 27 STL files to `stl_exports_v10/`, and prints the
    exact STL inventory table.
    """
    os.makedirs(STL_DIR_V10, exist_ok=True)

    print("=" * 90)
    print("BUILDING ITERATION 10 (v10.0 — 11-SERVO FULL-ACTUATION HAND & FOREARM SYSTEM)")
    print(f"  Output Directory : {STL_DIR_V10}")
    print(f"  Wrist Center     : {WRIST_CENTER} | Cup Hinge : {CUP_HINGE_POS}")
    print(f"  Pronation Gear   : {GEAR_Z_PINION}T -> {GEAR_Z_BARREL}T (m={GEAR_MODULE}, CD={GEAR_CENTER_DIST} mm)")
    print("=" * 90)

    meshes = {}
    assembly_parts = []

    print("\n[1/5] Generating Split Cupping Palm & Abduction Linkage...")
    palm_radial = generate_palm_radial()
    palm_ulnar = generate_palm_ulnar()
    abduction_linkage = generate_abduction_linkage()

    palm_radial.export(os.path.join(STL_DIR_V10, "palm_radial_v10.stl"))
    palm_ulnar.export(os.path.join(STL_DIR_V10, "palm_ulnar_v10.stl"))
    abduction_linkage.export(os.path.join(STL_DIR_V10, "abduction_linkage_v10.stl"))

    meshes["palm_radial_v10.stl"] = palm_radial
    meshes["palm_ulnar_v10.stl"] = palm_ulnar
    meshes["abduction_linkage_v10.stl"] = abduction_linkage

    assembly_parts.append(palm_radial.copy())
    assembly_parts.append(palm_ulnar.copy())
    linkage_placed = abduction_linkage.copy()
    linkage_placed.apply_translation(ABDUCTION_LINKAGE_POS)
    assembly_parts.append(linkage_placed)

    print("[2/5] Generating 4× MCP Cardan Yokes & 12× Finger Phalanges...")
    for d in DIGIT_SPECS:
        dname = d["name"]
        h_p = float(d.get("h_p", 12.0))
        h_i = float(d.get("h_i", 11.6))
        h_d = float(d.get("h_d", 11.2))

        yoke = generate_mcp_yoke(dname)
        p = generate_proximal_phalanx(length=d["len_p"], width=d["w_prox"], height=h_p)
        ip = generate_intermediate_phalanx(length=d["len_i"], width=d["w_mid"], height=h_i)
        dp = generate_distal_phalanx(length=d["len_d"], width=d["w_dist"], height=h_d)

        y_name = f"{dname}_mcp_yoke_v10.stl"
        p_name = f"{dname}_proximal_v10.stl"
        i_name = f"{dname}_intermediate_v10.stl"
        d_name = f"{dname}_distal_v10.stl"

        yoke.export(os.path.join(STL_DIR_V10, y_name))
        p.export(os.path.join(STL_DIR_V10, p_name))
        ip.export(os.path.join(STL_DIR_V10, i_name))
        dp.export(os.path.join(STL_DIR_V10, d_name))

        meshes[y_name] = yoke
        meshes[p_name] = p
        meshes[i_name] = ip
        meshes[d_name] = dp

        spec = _resolve_mcp_yoke_spec(dname)
        fx, py, fz = float(spec["x"]), float(spec["py"]), float(spec["z"])
        rot_z = float(spec.get("rotZ", d["rotZ"]))
        fm = float(d.get("fm", -0.35))
        fp = float(d.get("fp", -0.45))
        fd = float(d.get("fd", -0.30))

        yoke_placed = _place_mcp_yoke(yoke, spec, yaw_deg=0.0)
        assembly_parts.append(yoke_placed)

        p_c, ip_c, dp_c = p.copy(), ip.copy(), dp.copy()
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

    print("[3/5] Generating Opposable Thumb Phalanges...")
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
    th_p.export(os.path.join(STL_DIR_V10, "thumb_proximal_v10.stl"))
    th_d.export(os.path.join(STL_DIR_V10, "thumb_distal_v10.stl"))
    meshes["thumb_proximal_v10.stl"] = th_p
    meshes["thumb_distal_v10.stl"] = th_d

    th_p_c, th_d_c = th_p.copy(), th_d.copy()
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

    print("[4/5] Generating 2-DoF Cardan Wrist, 9-Servo Barrel, Outer Sleeve, Pinion & Winch Spool...")
    wrist_ring = generate_wrist_cardan_ring()
    inner_barrel = generate_forearm_inner_barrel()
    outer_sleeve = generate_forearm_outer_sleeve()
    pinion = generate_pronation_pinion()
    spool = generate_servo_spool()

    wrist_ring.export(os.path.join(STL_DIR_V10, "wrist_cardan_ring_v10.stl"))
    inner_barrel.export(os.path.join(STL_DIR_V10, "forearm_inner_barrel_v10.stl"))
    outer_sleeve.export(os.path.join(STL_DIR_V10, "forearm_outer_sleeve_v10.stl"))
    pinion.export(os.path.join(STL_DIR_V10, "pronation_pinion_v10.stl"))
    spool.export(os.path.join(STL_DIR_V10, "servo_spool_v10.stl"))

    meshes["wrist_cardan_ring_v10.stl"] = wrist_ring
    meshes["forearm_inner_barrel_v10.stl"] = inner_barrel
    meshes["forearm_outer_sleeve_v10.stl"] = outer_sleeve
    meshes["pronation_pinion_v10.stl"] = pinion
    meshes["servo_spool_v10.stl"] = spool

    wrist_placed = wrist_ring.copy()
    wrist_placed.apply_translation(WRIST_CENTER)
    assembly_parts.append(wrist_placed)
    assembly_parts.append(inner_barrel.copy())
    assembly_parts.append(outer_sleeve.copy())

    pinion_placed = pinion.copy()
    px, pz = float(PINION_AXIS_XZ[0]), float(PINION_AXIS_XZ[1])
    pinion_placed.apply_translation([px, float(PINION_Y), pz])
    assembly_parts.append(pinion_placed)

    print("[5/5] Exporting Full Articulated Hand & Forearm Assembly (v10.0)...")
    full_assembly = trimesh.util.concatenate(assembly_parts)
    full_assembly.export(os.path.join(STL_DIR_V10, ASSEMBLY_STL_NAME))
    meshes[ASSEMBLY_STL_NAME] = full_assembly

    print("\n--- EXPORTED STL SUMMARY (stl_exports_v10/) ---")
    print(f"  {'STL File':<38} {'Bytes':>10} {'KB':>8} {'Tris':>8} {'Volume (mm³)':>13} {'Bodies':>7} {'WT':>5}")
    print("  " + "-" * 95)
    for fname in ALL_STL_NAMES:
        fpath = os.path.join(STL_DIR_V10, fname)
        fbytes = os.path.getsize(fpath)
        fkb = fbytes / 1024.0
        m = meshes[fname]
        tris = len(m.faces)
        bodies = len(m.split(only_watertight=False))
        print(f"  {fname:<38} {fbytes:10d} {fkb:8.1f} {tris:8d} {m.volume:13.1f} {bodies:7d} {str(m.is_watertight):>5}")

    return {"meshes": meshes}


def verify_v10_complete() -> dict:
    """
    Strict automated verification suite for Iteration 10 (v10.0).
    Reloads all 27 STL files directly from `stl_exports_v10/` and asserts:
      1. Manifold, positive volume, single-shell integrity, and Kobra 2 Neo bed fit
      2. Full multi-DoF collision sweeps (< 0.05 mm³) across all joints & gears
      3. Hard-stop verification (-5° / -10° hyperextension blocked on all 14 digit joints)
      4. Actuator fit verification for all 8× MG90S + 3× DS3218 servo bays
    """
    print("\n" + "=" * 90)
    print("RUNNING STRICT AUTOMATED VERIFICATION SUITE: verify_v10_complete()")
    print("=" * 90)

    # -------------------------------------------------------------------------
    # 1. MANIFOLD, SINGLE-SHELL & PRINT-BED FIT (All 27 Disk STLs)
    # -------------------------------------------------------------------------
    print("\n[Suite 1/4] Manifold, Single-Shell & 220×220×250 mm Bed Fit (27 Disk STLs)...")
    loaded = {}
    for fname in ALL_STL_NAMES:
        fpath = os.path.join(STL_DIR_V10, fname)
        assert os.path.isfile(fpath), f"Missing exported STL file: {fpath}"
        m = trimesh.load(fpath, force="mesh")
        assert isinstance(m, trimesh.Trimesh), f"{fname} did not load as a Trimesh"
        assert m.is_watertight, f"Manifold failure: {fname} is NOT watertight!"
        assert m.volume > 0.0, f"Volume failure: {fname} has non-positive volume ({m.volume})!"
        bodies = len(m.split(only_watertight=False))
        if fname != ASSEMBLY_STL_NAME:
            assert bodies == 1, f"Single-shell failure: {fname} has {bodies} bodies (expected 1)!"
            ext_sorted = sorted(float(x) for x in m.extents)
            bed_sorted = sorted(KOBRA2_BED_MM)
            assert all(e <= b for e, b in zip(ext_sorted, bed_sorted)), (
                f"Print-bed overflow: {fname} extents {m.extents} exceed {KOBRA2_BED_MM} mm!"
            )
        loaded[fname] = m

    palm_radial = loaded["palm_radial_v10.stl"]
    palm_ulnar = loaded["palm_ulnar_v10.stl"]
    wrist_ring = loaded["wrist_cardan_ring_v10.stl"]
    inner_barrel = loaded["forearm_inner_barrel_v10.stl"]
    outer_sleeve = loaded["forearm_outer_sleeve_v10.stl"]
    pinion = loaded["pronation_pinion_v10.stl"]
    th_p = loaded["thumb_proximal_v10.stl"]
    th_d = loaded["thumb_distal_v10.stl"]
    print("  PASS: All 27 STLs watertight (vol > 0), all 26 components single-body & fit 220×220×250 mm.")

    # -------------------------------------------------------------------------
    # 2. FULL MULTI-DoF COLLISION SWEEPS (< 0.05 mm³ tolerance)
    # -------------------------------------------------------------------------
    print("\n[Suite 2/4] Full Multi-DoF Collision Sweeps (< 0.05 mm³ tolerance)...")
    COLL_TOL = 0.05
    finger_angles = [0, 15, 30, 45, 60, 75, 90, 95]
    thumb_angles = [0, 15, 30, 45, 60, 75, 90]
    max_coll = 0.0

    # 2a. 4 Fingers: MCP flexion (0-95° on yoke), MCP yaw (-15/0/+15° in palm), PIP (0-95°), DIP (0-95°)
    for d in DIGIT_SPECS:
        dname = d["name"]
        yoke = loaded[f"{dname}_mcp_yoke_v10.stl"]
        p = loaded[f"{dname}_proximal_v10.stl"]
        ip = loaded[f"{dname}_intermediate_v10.stl"]
        dp = loaded[f"{dname}_distal_v10.stl"]
        spec = _resolve_mcp_yoke_spec(dname)
        palm_host = palm_radial if dname in ("index", "middle") else palm_ulnar
        len_p = float(d["len_p"])
        len_i = float(d["len_i"])

        for deg in finger_angles:
            rad = -np.deg2rad(deg)

            # MCP flexion on yoke (local yoke origin = X flexion hinge)
            p_mcp = p.copy()
            p_mcp.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            v_mcp = intersection_volume(yoke, p_mcp)
            max_coll = max(max_coll, v_mcp)
            assert v_mcp < COLL_TOL, f"{dname} MCP flexion collision at {deg}°: {v_mcp:.4f} mm³"

            # PIP
            ip_pip = ip.copy()
            ip_pip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            ip_pip.apply_translation([0.0, len_p, 0.0])
            v_pip = intersection_volume(p, ip_pip)
            max_coll = max(max_coll, v_pip)
            assert v_pip < COLL_TOL, f"{dname} PIP collision at {deg}°: {v_pip:.4f} mm³"

            # DIP
            dp_dip = dp.copy()
            dp_dip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            dp_dip.apply_translation([0.0, len_i, 0.0])
            v_dip = intersection_volume(ip, dp_dip)
            max_coll = max(max_coll, v_dip)
            assert v_dip < COLL_TOL, f"{dname} DIP collision at {deg}°: {v_dip:.4f} mm³"

        for yaw_deg in (-15.0, 0.0, 15.0):
            yoke_posed = _place_mcp_yoke(yoke, spec, yaw_deg=yaw_deg)
            v_yaw = intersection_volume(palm_host, yoke_posed)
            max_coll = max(max_coll, v_yaw)
            assert v_yaw < COLL_TOL, f"{dname} MCP yaw collision at {yaw_deg:+.0f}°: {v_yaw:.4f} mm³"

        print(f"  PASS: {dname.capitalize():<6} MCP flex (0–95°), MCP yaw (±15°), PIP & DIP clear.")

    # 2b. Palmar Arch Cupping: palm_ulnar vs palm_radial across 0°, 10°, 20°, 30°
    for cup_deg in (0.0, 10.0, 20.0, 30.0):
        ulnar_cupped = _rotate_about_point(palm_ulnar, np.deg2rad(cup_deg), [0, 1, 0], CUP_HINGE_POS)
        v_cup = intersection_volume(palm_radial, ulnar_cupped)
        max_coll = max(max_coll, v_cup)
        assert v_cup < COLL_TOL, f"Palmar cupping collision at {cup_deg:.0f}°: {v_cup:.4f} mm³"
    print("  PASS: Palmar arch cupping (palm_ulnar vs palm_radial) clear across 0°–30°.")

    # 2c. Thumb CMC (0-90° on palm_radial) & Thumb IP (0-90°)
    rot_cmc = trimesh.transformations.euler_matrix(THUMB_CMC_X, THUMB_CMC_Y, THUMB_CMC_Z)
    th_len_p = float(THUMB_SPEC["len_p"])
    for deg in thumb_angles:
        rad = -np.deg2rad(deg)
        th_p_cmc = th_p.copy()
        th_p_cmc.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
        th_p_cmc.apply_transform(rot_cmc)
        th_p_cmc.apply_translation(THUMB_CMC_POS)
        v_cmc = intersection_volume(palm_radial, th_p_cmc)
        max_coll = max(max_coll, v_cmc)
        assert v_cmc < COLL_TOL, f"Thumb CMC collision at {deg}°: {v_cmc:.4f} mm³"

        th_d_ip = th_d.copy()
        th_d_ip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
        th_d_ip.apply_translation([0.0, th_len_p, 0.0])
        v_ip = intersection_volume(th_p, th_d_ip)
        max_coll = max(max_coll, v_ip)
        assert v_ip < COLL_TOL, f"Thumb IP collision at {deg}°: {v_ip:.4f} mm³"
    print("  PASS: Thumb CMC (0–90° on palm_radial) & Thumb IP (0–90°) clear.")

    # 2d. Wrist 2-DoF Gimbal: Pitch (-45, 0, +45°) & Yaw (-35, 0, +35°)
    wrist_base = wrist_ring.copy()
    wrist_base.apply_translation(WRIST_CENTER)
    for pitch_deg in (-45.0, 0.0, 45.0):
        w_pitched = _rotate_about_point(wrist_base, np.deg2rad(pitch_deg), [1, 0, 0], WRIST_CENTER)
        v_pitch = intersection_volume(inner_barrel, w_pitched)
        max_coll = max(max_coll, v_pitch)
        assert v_pitch < COLL_TOL, f"Wrist pitch collision at {pitch_deg:+.0f}°: {v_pitch:.4f} mm³"

    for yaw_deg in (-35.0, 0.0, 35.0):
        p_yawed = _rotate_about_point(palm_radial, np.deg2rad(yaw_deg), [0, 0, 1], WRIST_CENTER)
        v_wyaw = intersection_volume(wrist_base, p_yawed)
        max_coll = max(max_coll, v_wyaw)
        assert v_wyaw < COLL_TOL, f"Wrist yaw collision at {yaw_deg:+.0f}°: {v_wyaw:.4f} mm³"
    print("  PASS: Wrist 2-DoF Cardan gimbal clear across Pitch ±45° & Yaw ±35°.")

    # 2e. Forearm Pronation: Barrel vs Outer Sleeve (-85, 0, +85°) + Synchronized 22T/44T Gear Mesh
    px, pz = float(PINION_AXIS_XZ[0]), float(PINION_AXIS_XZ[1])
    for roll_deg in (-85.0, 0.0, 85.0):
        b_rolled = _rotate_about_point(inner_barrel, np.deg2rad(roll_deg), [0, 1, 0], [0.0, 0.0, 0.0])
        v_roll = intersection_volume(outer_sleeve, b_rolled)
        max_coll = max(max_coll, v_roll)
        assert v_roll < COLL_TOL, f"Forearm pronation collision at {roll_deg:+.0f}°: {v_roll:.4f} mm³"

        # 2:1 gear ratio: barrel rotates +roll_deg around Y, external meshing pinion rotates -2*roll_deg
        p_rolled = pinion.copy()
        p_rolled.apply_transform(trimesh.transformations.rotation_matrix(np.deg2rad(-2.0 * roll_deg), [0, 1, 0]))
        p_rolled.apply_translation([px, float(PINION_Y), pz])
        v_gear = intersection_volume(b_rolled, p_rolled)
        max_coll = max(max_coll, v_gear)
        assert v_gear < COLL_TOL, f"Pronation 22T/44T gear mesh collision at roll={roll_deg:+.0f}°: {v_gear:.4f} mm³"
    print("  PASS: Forearm pronation (±85° roll) & synchronized 22T/44T gear mesh clear.")

    # -------------------------------------------------------------------------
    # 3. HARD-STOP VERIFICATION (-5° & -10° blocked on all 14 digit joints)
    # -------------------------------------------------------------------------
    print("\n[Suite 3/4] One-Directional Extension Hard Stops (-5° & -10° > 0.03 mm³)...")
    min_stop = np.inf
    for deg in (-5, -10):
        rad = -np.deg2rad(deg)
        for d in DIGIT_SPECS:
            dname = d["name"]
            yoke = loaded[f"{dname}_mcp_yoke_v10.stl"]
            p = loaded[f"{dname}_proximal_v10.stl"]
            ip = loaded[f"{dname}_intermediate_v10.stl"]
            dp = loaded[f"{dname}_distal_v10.stl"]

            p_mcp = p.copy()
            p_mcp.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            v_mcp = intersection_volume(yoke, p_mcp)
            assert v_mcp > 0.03, f"Hard stop FAILED on {dname} MCP at {deg}°: {v_mcp:.4f} mm³"

            ip_pip = ip.copy()
            ip_pip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            ip_pip.apply_translation([0.0, float(d["len_p"]), 0.0])
            v_pip = intersection_volume(p, ip_pip)
            assert v_pip > 0.03, f"Hard stop FAILED on {dname} PIP at {deg}°: {v_pip:.4f} mm³"

            dp_dip = dp.copy()
            dp_dip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
            dp_dip.apply_translation([0.0, float(d["len_i"]), 0.0])
            v_dip = intersection_volume(ip, dp_dip)
            assert v_dip > 0.03, f"Hard stop FAILED on {dname} DIP at {deg}°: {v_dip:.4f} mm³"

            min_stop = min(min_stop, v_mcp, v_pip, v_dip)

        th_p_cmc = th_p.copy()
        th_p_cmc.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
        th_p_cmc.apply_transform(rot_cmc)
        th_p_cmc.apply_translation(THUMB_CMC_POS)
        v_cmc = intersection_volume(palm_radial, th_p_cmc)
        assert v_cmc > 0.03, f"Hard stop FAILED on Thumb CMC at {deg}°: {v_cmc:.4f} mm³"

        th_d_ip = th_d.copy()
        th_d_ip.apply_transform(trimesh.transformations.rotation_matrix(rad, [1, 0, 0]))
        th_d_ip.apply_translation([0.0, th_len_p, 0.0])
        v_ip = intersection_volume(th_p, th_d_ip)
        assert v_ip > 0.03, f"Hard stop FAILED on Thumb IP at {deg}°: {v_ip:.4f} mm³"
        min_stop = min(min_stop, v_cmc, v_ip)

    print(f"  PASS: All 14 digit joints rigidly block -5°/-10° hyperextension (min={min_stop:.3f} mm³).")

    # -------------------------------------------------------------------------
    # 4. ACTUATOR FIT VERIFICATION (8× MG90S + 3× DS3218/MG996R)
    # -------------------------------------------------------------------------
    print("\n[Suite 4/4] Actuator Fit Verification (8× MG90S + 3× DS3218/MG996R)...")
    assert len(BARREL_MG90S_CENTERS) == 7, f"Expected 7 barrel MG90S bays, got {len(BARREL_MG90S_CENTERS)}"
    assert len(BARREL_DS3218_CENTERS) == 2, f"Expected 2 barrel DS3218 bays, got {len(BARREL_DS3218_CENTERS)}"
    assert SLEEVE_DS3218_CENTER is not None, "Missing SLEEVE_DS3218_CENTER"

    # 4a. 7× MG90S tendon bays in forearm_inner_barrel (slide in along ±X)
    mg90s_barrel_env = [22.0, 12.2, 22.8]
    for idx, (cx, cy, cz, side_sign) in enumerate(BARREL_MG90S_CENTERS):
        for slide_out in (30.0, 15.0, 5.0, 0.0):
            sbox = box(extents=mg90s_barrel_env)
            sbox.apply_translation([cx + side_sign * slide_out, cy, cz])
            v = intersection_volume(inner_barrel, sbox)
            assert v < COLL_TOL, (
                f"Barrel MG90S bay #{idx + 1} ({cx}, {cy}, {cz}) collides ({v:.4f} mm³) at slide_out={slide_out} mm!"
            )

    # 4b. 1× In-palm MG90S abduction bay in palm_radial (drop-in from dorsal +Z)
    bay_c = IN_PALM_SERVO_BAY["center"]
    z_floor = float(IN_PALM_SERVO_BAY["z_floor"])
    mg90s_palm_h = 16.0  # Lower body seated inside palm cavity (z_floor=-9.5 -> z_mount=+7.5)
    mg90s_palm_z0 = z_floor + 0.5 * mg90s_palm_h + 0.3
    for slide_out in (25.0, 12.0, 4.0, 0.0):
        sbox = box(extents=[12.2, 22.8, mg90s_palm_h])
        sbox.apply_translation([bay_c[0], bay_c[1], mg90s_palm_z0 + slide_out])
        v = intersection_volume(palm_radial, sbox)
        assert v < COLL_TOL, (
            f"In-palm MG90S abduction bay collides ({v:.4f} mm³) at dorsal slide_out={slide_out} mm!"
        )

    # 4c. 2× DS3218/MG996R wrist pushrod bays in forearm_inner_barrel (drop-in from palmar -Z)
    ds3218_barrel_env = [20.0, 40.0, 37.5]
    for idx, (cx, cy, cz, _side_sign) in enumerate(BARREL_DS3218_CENTERS):
        for slide_out in (45.0, 20.0, 5.0, 0.0):
            sbox = box(extents=ds3218_barrel_env)
            sbox.apply_translation([cx, cy, cz - slide_out])
            v = intersection_volume(inner_barrel, sbox)
            assert v < COLL_TOL, (
                f"Barrel DS3218 bay #{idx + 1} ({cx}, {cy}, {cz}) collides ({v:.4f} mm³) at -Z slide_out={slide_out} mm!"
            )

    # 4d. 1× DS3218/MG996R pronation drive bay in forearm_outer_sleeve (drop-in from rear -Y)
    scx, scy, scz = SLEEVE_DS3218_CENTER
    ds3218_sleeve_env = [20.0, 37.5, 40.0]
    for slide_out in (45.0, 20.0, 5.0, 0.0):
        sbox = box(extents=ds3218_sleeve_env)
        sbox.apply_translation([scx, scy - slide_out, scz])
        v = intersection_volume(outer_sleeve, sbox)
        assert v < COLL_TOL, (
            f"Sleeve DS3218 bay ({scx}, {scy}, {scz}) collides ({v:.4f} mm³) at -Y slide_out={slide_out} mm!"
        )

    print("  PASS: All 11 actuator bays (7× barrel MG90S + 1× palm MG90S + 3× DS3218) clear cleanly from outside.")

    print("\n" + "=" * 90)
    print("ALL V10.0 GEOMETRIC, MANIFOLD, KINEMATIC & ACTUATOR ASSERTIONS PASSED 100%!")
    print("=" * 90 + "\n")

    return {
        "max_collision_mm3": max_coll,
        "min_hard_stop_mm3": min_stop,
    }


if __name__ == "__main__":
    build_iteration_10()
    verify_v10_complete()
