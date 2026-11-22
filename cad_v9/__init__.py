"""
Anthropomorphic Robotic Hand CAD Package — Iteration 9 (v9.0)
=============================================================
Modular parametric CAD package providing:
- `cad_v9.digits`: Phalanx geometry, captive M3 hex-nut clevis joints, profile-following
  snap-fit dorsal pinch-lip channels, palmar flexor bores (TENDON_Z = -3.50 mm), and
  distal fingernail beds + palmar knot pockets.
- `cad_v9.chassis`: Biomimetic palm with splayed knuckle tongues, MCP extension stops,
  male thumb CMC pedestal, unbroken through-palm tendon conduits, captive dorsal elastic
  cross-pin anchors, Y-axis 4-bolt wrist bulkhead, and 6-bay open-sided MG90S forearm rack.
"""

from importlib import import_module

__version__ = "9.0.0"

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

_CHASSIS_EXPORTS = {
    "PALM_W",
    "PALM_L",
    "PALM_H",
    "THUMB_CMC_X",
    "THUMB_CMC_Y",
    "THUMB_CMC_Z",
    "THUMB_CMC_POS",
    "WRIST_INTERFACE_Y",
    "ADAPTER_PLACEMENT",
    "WRIST_BOLT_XZ",
    "KNUCKLE_SPECS",
    "generate_palm",
    "generate_forearm_adapter",
}

__all__ = sorted(_DIGITS_EXPORTS | _CHASSIS_EXPORTS)


def __getattr__(name: str):
    if name in _DIGITS_EXPORTS:
        mod = import_module(".digits", __name__)
        return getattr(mod, name)
    if name in _CHASSIS_EXPORTS:
        mod = import_module(".chassis", __name__)
        return getattr(mod, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals().keys()) | set(__all__))
