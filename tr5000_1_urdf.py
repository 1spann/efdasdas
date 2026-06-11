"""
URDF generator for the TR-5000 Trash Rocket (tr5000_1.py geometry).

Design ledger
-------------
Units/frames: URDF metres/radians; CAD model is millimetres (mesh scale
0.001). base_link frame = CAD model frame: origin at the hitch point on
the ground, +X toward the hitch (travel direction), +Z up, Y=0 on the
trailer centreline. Stowed transport pose = all joints at zero.

Kinematic tree (matches tr5000_1.LINK_FRAMES):
  base_link
   └─ turret_slew    revolute  +Z at the slew ring atop the column;
                     enables the setup guide's perpendicular/parallel
                     orientations (±180°).
   └─ boom_lift      revolute  axis -Y at the boom pivot; positive
                     raises the saddle end (boom points +X).
   └─ chute_pitch    revolute  axis +Y at the saddle; positive raises
                     the hopper end (chute points -X). Because chute_1
                     is the boom's child, chute elevation above horizon
                     = chute_pitch - boom_lift (spec max 59°), so the
                     knuckle sweeps lift + elevation like an excavator
                     boom/stick pair.
   └─ telescope_2/3/4  prismatic along local -X (up the chute toward
                     the hopper end). Travel set so full extension at
                     59° reaches the spec's 50'-5" (15.37 m) hopper lip.

Inertials are documented engineering estimates: total trailer weight
8,740 lbs (3,964 kg) from the spec sheet, distributed by component, with
box/cylinder-approximation inertia tensors about each link COM.

Known limitation: the hydraulic lift cylinder is visual-only geometry in
base_link (URDF trees cannot close the boom/cylinder loop), so it is
drawn correctly only near the stowed pose. Wheels are fixed (no rolling
joints).
"""

import math
import xml.etree.ElementTree as ET

from tr5000_1 import (
    LINK_FRAMES, TUBE_FRONT_X, TUBE_LEN, TUBE_ODS, CHUTE_CL_Z,
    STACK_TOP_Z, HOPPER_SMALL_X, STA_HOPPER_END, inch,
)

MM = 0.001                       # CAD mm -> URDF metres
MESH_SCALE = "0.001 0.001 0.001"
MESH_DIR = "tr5000_1_meshes"

DEG = math.pi / 180.0

# ---------------------------------------------------------------------------
# Spec-sheet physical properties (Rocket Equipment TR-5000 Trash Rocket)
# DWG 75024  |  mfr: Rocket Equipment Inc.
# ---------------------------------------------------------------------------
# Trailer weight (GVW):      8,740 lbs  (3,964 kg)
# Tongue weight:               950 lbs  (431 kg)
# Axles:         6,000 lb tandem axles (2 × 2,722 kg rated)
# Power:         Dual 195 Ah marine deep-cycle batteries (12 V)
# Chute material:  1/8" (3.175 mm) 6061-T6 aluminium sidewalls,
#                  1/16" (1.587 mm) aluminium cover sheet
# Chute liner:     1/4" (6.35 mm) UHMW polyethylene (natural)
# Hopper opening:  54" (1,371.6 mm) wide
# Max hopper lip:  50'-5" (15.367 m) at 59° elevation (4-story setup)
# Frame / column:  Carbon steel, primer + safety orange paint
# Slew ring:       Carbon steel, worm-gear drive, hydraulic motor
# ---------------------------------------------------------------------------

# --- material RGBA colours (r g b a) ----------------------------------------
MATERIALS = {
    # name:               (R,    G,    B,    A)
    "steel_orange":       (0.96, 0.45, 0.05, 1.0),   # safety orange frame
    "steel_dark":         (0.18, 0.18, 0.18, 1.0),   # dark steel (axles, slew)
    "aluminum_raw":       (0.75, 0.75, 0.78, 1.0),   # bare 6061-T6 aluminium
    "uhmw_natural":       (0.95, 0.95, 0.90, 1.0),   # natural UHMW poly liner
}

# per-link material assignment
LINK_MATERIAL = {
    "base_link": "steel_orange",
    "turret":    "steel_dark",
    "boom":      "steel_orange",
    "chute_1":   "aluminum_raw",
    "chute_2":   "aluminum_raw",
    "chute_3":   "aluminum_raw",
    "chute_4":   "aluminum_raw",
}

# --- joint frames, derived from the CAD link frames (metres) ---------------
TURRET_XYZ = tuple(v * MM for v in LINK_FRAMES["turret"])
BOOM_IN_TURRET = tuple(
    (LINK_FRAMES["boom"][i] - LINK_FRAMES["turret"][i]) * MM for i in range(3))
CHUTE_IN_BOOM = tuple(
    (LINK_FRAMES["chute_1"][i] - LINK_FRAMES["boom"][i]) * MM for i in range(3))

SLEW_AXIS = "0 0 1"
LIFT_AXIS = "0 -1 0"             # positive raises the +X (saddle) end
PITCH_AXIS = "0 1 0"             # positive raises the -X (hopper) end
TELESCOPE_AXIS = "-1 0 0"        # extend rearward/up the chute axis

SLEW_LIMIT = math.pi             # ±180°: perpendicular & parallel setups
BOOM_LIFT_RANGE = (0.0, 40 * DEG)
CHUTE_MAX_ELEV = 59 * DEG        # 4-story setup angle from the spec
# knuckle range: elevation = pitch - lift, so full deploy needs 40°+59°
CHUTE_PITCH_RANGE = (-5 * DEG, BOOM_LIFT_RANGE[1] + CHUTE_MAX_ELEV)

# --- telescope travel solved from the 50'-5" (4-story) reach spec ----------
SPEC_MAX_HEIGHT = (50 + 5/12) * 0.3048          # 15.367 m hopper lip height
BOOM_TIP_R = math.hypot(CHUTE_IN_BOOM[0], CHUTE_IN_BOOM[2])
BOOM_STOWED_ANG = math.atan2(CHUTE_IN_BOOM[2], CHUTE_IN_BOOM[0])
_SADDLE_Z_DEPLOYED = (TURRET_XYZ[2] + BOOM_IN_TURRET[2]
                      + BOOM_TIP_R * math.sin(BOOM_STOWED_ANG
                                              + BOOM_LIFT_RANGE[1]))
# hopper lip in chute-local coords, stowed (top rear corner of the opening)
_LIP_LOCAL_X = (STA_HOPPER_END - LINK_FRAMES["chute_4"][0]) * MM
_LIP_LOCAL_Z = (STACK_TOP_Z - LINK_FRAMES["chute_4"][2]) * MM
_ALONG_NEEDED = ((SPEC_MAX_HEIGHT - _SADDLE_Z_DEPLOYED
                  - _LIP_LOCAL_Z * math.cos(CHUTE_MAX_ELEV))
                 / math.sin(CHUTE_MAX_ELEV))
TELESCOPE_TRAVEL = (_ALONG_NEEDED - (-_LIP_LOCAL_X)) / 3.0   # per section

# mechanical cap: keep a 2 ft overlap between nested sections
_MECH_CAP = (TUBE_LEN - inch(24)) * MM - max(
    abs(TUBE_FRONT_X[i+1] - TUBE_FRONT_X[i]) * MM for i in range(3))
TELESCOPE_TRAVEL = min(TELESCOPE_TRAVEL, _MECH_CAP)

# --- inertial estimates (kg, m; spec total 3,964 kg) ------------------------
# (mass, COM in link frame, box dims for the inertia approximation)
INERTIALS = {
    "base_link": (2950.0, (-4.4, 0.0, 0.55), (8.8, 2.6, 1.1)),
    "turret":    (40.0,   (0.0, 0.0, 0.03),  (0.56, 0.56, 0.07)),
    "boom":      (180.0,  (0.82, 0.0, 0.66), (1.7, 0.25, 1.4)),
    "chute_1":   (330.0,  (-1.2, 0.0, 0.46), (4.9, 0.92, 0.92)),
    "chute_2":   (240.0,  (-2.5, 0.0, 0.46), (4.9, 0.84, 0.84)),
    "chute_3":   (210.0,  (-2.9, 0.0, 0.46), (4.9, 0.79, 0.79)),
    "chute_4":   (260.0,  (-3.4, 0.0, 0.40), (6.2, 1.37, 1.0)),
}


def _fmt(vals):
    return " ".join(f"{v:.6g}" for v in vals)


def _box_inertia(mass, dims):
    x, y, z = dims
    ixx = mass / 12.0 * (y*y + z*z)
    iyy = mass / 12.0 * (x*x + z*z)
    izz = mass / 12.0 * (x*x + y*y)
    return ixx, iyy, izz


def _add_link(robot, name):
    link = ET.SubElement(robot, "link", {"name": name})
    mass, com, dims = INERTIALS[name]
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": _fmt(com), "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": f"{mass:.6g}"})
    ixx, iyy, izz = _box_inertia(mass, dims)
    ET.SubElement(inertial, "inertia", {
        "ixx": f"{ixx:.6g}", "iyy": f"{iyy:.6g}", "izz": f"{izz:.6g}",
        "ixy": "0", "ixz": "0", "iyz": "0",
    })
    mat_name = LINK_MATERIAL[name]
    for tag in ("visual", "collision"):
        elem = ET.SubElement(link, tag)
        ET.SubElement(elem, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geom = ET.SubElement(elem, "geometry")
        ET.SubElement(geom, "mesh", {
            "filename": f"{MESH_DIR}/{name}.stl",
            "scale": MESH_SCALE,
        })
        if tag == "visual":
            ET.SubElement(elem, "material", {"name": mat_name})
    return link


def _add_joint(robot, name, jtype, parent, child, xyz, axis=None,
               limit=None, effort=None, velocity=None):
    joint = ET.SubElement(robot, "joint", {"name": name, "type": jtype})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {"xyz": _fmt(xyz), "rpy": "0 0 0"})
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": axis})
    if limit is not None:
        ET.SubElement(joint, "limit", {
            "lower": f"{limit[0]:.6g}", "upper": f"{limit[1]:.6g}",
            "effort": f"{effort:.6g}", "velocity": f"{velocity:.6g}",
        })
    return joint


def gen_urdf():
    robot = ET.Element("robot", {"name": "tr5000_trash_rocket"})

    # global material definitions (referenced by visual blocks on each link)
    for mat_name, (r, g, b, a) in MATERIALS.items():
        mat = ET.SubElement(robot, "material", {"name": mat_name})
        ET.SubElement(mat, "color", {"rgba": f"{r} {g} {b} {a}"})

    for name in INERTIALS:
        _add_link(robot, name)

    _add_joint(robot, "turret_slew", "revolute",
               "base_link", "turret", TURRET_XYZ,
               axis=SLEW_AXIS, limit=(-SLEW_LIMIT, SLEW_LIMIT),
               effort=8000, velocity=0.5)

    _add_joint(robot, "boom_lift", "revolute",
               "turret", "boom", BOOM_IN_TURRET,
               axis=LIFT_AXIS, limit=BOOM_LIFT_RANGE,
               effort=60000, velocity=0.25)

    _add_joint(robot, "chute_pitch", "revolute",
               "boom", "chute_1", CHUTE_IN_BOOM,
               axis=PITCH_AXIS, limit=CHUTE_PITCH_RANGE,
               effort=60000, velocity=0.25)

    for i in (2, 3, 4):
        _add_joint(robot, f"telescope_{i}", "prismatic",
                   f"chute_{i-1}", f"chute_{i}", (0.0, 0.0, 0.0),
                   axis=TELESCOPE_AXIS, limit=(0.0, TELESCOPE_TRAVEL),
                   effort=15000, velocity=0.4)

    return robot


if __name__ == "__main__":
    print(f"telescope travel per section: {TELESCOPE_TRAVEL:.3f} m")
    print(ET.tostring(gen_urdf(), encoding="unicode")[:400])
