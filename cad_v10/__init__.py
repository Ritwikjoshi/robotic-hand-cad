"""
Anthropomorphic Robotic Hand CAD Package — Iteration 10 (v10.0)
===============================================================
Full-actuation 11-servo parametric CAD package providing:
- Certified v9.0 digit phalanges re-exported from `cad_v9.digits`
- `cad_v10.palm_assembly`: Split cupping palm (`palm_radial`, `palm_ulnar`),
  4× 2-DoF MCP Cardan yokes, and coupled abduction linkage bar with in-palm MG90S bay.
- `cad_v10.wrist_forearm`: Hollow-core 2-DoF Cardan wrist ring, 9-servo rotating
  forearm actuator barrel with integral 44T pronation gear, stationary outer
  pronation sleeve with DS3218 bay + PCA9685/ESP32-S3 tray, 22T pronation pinion,
  and printable micro-servo winch spool.
"""

from importlib import import_module

__version__ = "10.0.0"

_DIGITS_EXPORTS = {
    "PIN_RADIUS",
    "TENDON_RADIUS",
    "TENDON_Z",
    "CLEVIS_SLOT_W",
    "CLEVIS_TONGUE_W",
    "FDM_CLEARANCE",
    "HUB_R_FRAC",
    "NAIL_DEPTH",
    "DIGIT_SPECS",
    "THUMB_SPEC",
    "generate_proximal_phalanx",
    "generate_intermediate_phalanx",
    "generate_distal_phalanx",
}

_PALM_EXPORTS = {
    "WRIST_CENTER",
    "CUP_HINGE_POS",
    "PUSHROD_PALM_ANCHORS",
    "THUMB_CMC_POS",
    "THUMB_CMC_X",
    "THUMB_CMC_Y",
    "THUMB_CMC_Z",
    "MCP_YOKE_SPECS",
    "ABDUCTION_LINKAGE_POS",
    "IN_PALM_SERVO_BAY",
    "mcp_yoke_transform",
    "generate_palm_radial",
    "generate_palm_ulnar",
    "generate_mcp_yoke",
    "generate_abduction_linkage",
}

_WRIST_FOREARM_EXPORTS = {
    "GEAR_MODULE",
    "GEAR_Z_BARREL",
    "GEAR_Z_PINION",
    "GEAR_CENTER_DIST",
    "PINION_AXIS_XZ",
    "PINION_Y",
    "BARREL_MG90S_CENTERS",
    "BARREL_MG90S_BAY_EXTENTS",
    "BARREL_DS3218_CENTERS",
    "BARREL_DS3218_BAY_EXTENTS",
    "SLEEVE_DS3218_CENTER",
    "SLEEVE_DS3218_BAY_EXTENTS",
    "generate_wrist_cardan_ring",
    "generate_forearm_inner_barrel",
    "generate_forearm_outer_sleeve",
    "generate_pronation_pinion",
    "generate_servo_spool",
}

__all__ = sorted(_DIGITS_EXPORTS | _PALM_EXPORTS | _WRIST_FOREARM_EXPORTS)


def __getattr__(name: str):
    if name in _DIGITS_EXPORTS:
        mod = import_module("cad_v9.digits")
        return getattr(mod, name)
    if name in _PALM_EXPORTS:
        mod = import_module(".palm_assembly", __name__)
        return getattr(mod, name)
    if name in _WRIST_FOREARM_EXPORTS:
        mod = import_module(".wrist_forearm", __name__)
        return getattr(mod, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals().keys()) | set(__all__))
