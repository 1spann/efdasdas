"""
TR-5000 Trash Rocket — trailer-mounted telescoping debris chute
Rocket Equipment, DWG 75024

4-story fully-extended configuration:
  chute angle   59° from horizontal
  total reach   ~50'-5"  = 15,367 mm

All dimensions in mm; 1 ft = 304.8 mm, 1 in = 25.4 mm
Origin: centre of trailer frame at ground level, +Z up, +X toward hitch/tongue.
"""

from build123d import (
    BuildPart, Box, Cylinder, Location, Compound, Align, Mode,
)
from cadpy.assembly import AssemblyHelper, label_shape
import math


def ft(v):  return v * 304.8
def inch(v): return v * 25.4


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

FRAME_LEN     = ft(29)
FRAME_W       = ft(8.5)
FRAME_H       = inch(10)
WALL_T        = inch(0.375)

AXLE_DIA      = inch(3.5)
WHEEL_DIA     = inch(24)
WHEEL_W       = inch(8)
AXLE_SPACING  = ft(4)
AXLE_X_POS    = ft(-6)

TONGUE_LEN    = ft(8.5)
TONGUE_W      = inch(6)
TONGUE_H      = inch(6)

CHUTE_WIDTHS  = [inch(w) for w in [54, 36, 33, 31, 29.5]]
CHUTE_DEPTH   = inch(24)
CHUTE_WALL    = inch(0.125)
CHUTE_LENGTHS = [ft(l) for l in [14, 13, 12, 11, 10.4]]

UPPER_ANGLE_DEG = 59.0
PIVOT_HEIGHT    = inch(48)

LEG_H   = inch(36)
LEG_DIA = inch(3)


# ---------------------------------------------------------------------------
# Part builders — all centred at origin, caller positions via .moved()
# ---------------------------------------------------------------------------

def make_frame():
    with BuildPart() as p:
        Box(FRAME_LEN, FRAME_W, FRAME_H)
        Box(FRAME_LEN - 2*WALL_T,
            FRAME_W - 2*WALL_T,
            FRAME_H - WALL_T,
            mode=Mode.SUBTRACT)
    return p.part


def make_axle():
    with BuildPart() as p:
        Cylinder(radius=AXLE_DIA/2,
                 height=FRAME_W + inch(8),
                 rotation=(90, 0, 0))
    return p.part


def make_wheel():
    with BuildPart() as p:
        Cylinder(radius=WHEEL_DIA/2,
                 height=WHEEL_W,
                 rotation=(90, 0, 0))
    return p.part


def make_tongue():
    with BuildPart() as p:
        Box(TONGUE_LEN, TONGUE_W, TONGUE_H,
            align=(Align.MAX, Align.CENTER, Align.CENTER))
    return p.part


def make_chute_section(width, depth, length, wall):
    with BuildPart() as p:
        Box(length, width, depth)
        Box(length + 2,
            width - 2*wall,
            depth - wall,
            align=(Align.CENTER, Align.CENTER, Align.MAX),
            mode=Mode.SUBTRACT)
    return p.part


def make_leg():
    with BuildPart() as p:
        Cylinder(radius=LEG_DIA/2, height=LEG_H)
    return p.part


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------

def gen_step():
    asm = AssemblyHelper("tr5000_trash_rocket")

    # Trailer frame — centred at origin
    asm.add(make_frame(), "trailer_frame")

    # Tongue from +X face forward
    asm.add(make_tongue().moved(Location((FRAME_LEN/2, 0, 0))),
            "tongue")

    # Tandem axles + wheels
    wheel_z = -(FRAME_H/2 + WHEEL_DIA/2)
    for i, ax_x in enumerate([AXLE_X_POS, AXLE_X_POS - AXLE_SPACING]):
        asm.add(make_axle().moved(Location((ax_x, 0, wheel_z))),
                f"axle", f"{i+1}")
        for side, y_sign in [("left", 1), ("right", -1)]:
            asm.add(make_wheel().moved(
                        Location((ax_x,
                                  y_sign*(FRAME_W/2 + WHEEL_W/2),
                                  wheel_z))),
                    "wheel", f"{i+1}_{side}")

    # Jack legs at four corners
    leg_z = -(FRAME_H/2 + LEG_H/2)
    for lbl_detail, xf, yf in [
        ("fl",  0.4,  1),
        ("fr",  0.4, -1),
        ("rl", -0.4,  1),
        ("rr", -0.4, -1),
    ]:
        asm.add(make_leg().moved(
                    Location((xf*FRAME_LEN/2, yf*FRAME_W/2, leg_z))),
                "leg", lbl_detail)

    # Telescoping chute sections at 59°
    pivot_x = -FRAME_LEN * 0.35
    pivot_z = FRAME_H/2 + PIVOT_HEIGHT
    rad = math.radians(UPPER_ANGLE_DEG)
    along = 0.0
    for i, (w, l) in enumerate(zip(CHUTE_WIDTHS, CHUTE_LENGTHS)):
        cx = pivot_x + math.cos(rad) * (along + l/2)
        cz = pivot_z + math.sin(rad) * (along + l/2)
        sec = make_chute_section(w, CHUTE_DEPTH, l, CHUTE_WALL)
        asm.add(sec.moved(Location((cx, 0, cz), (0, -UPPER_ANGLE_DEG, 0))),
                "chute_section", f"{i+1}")
        along += l

    return asm.build()


if __name__ == "__main__":
    shape = gen_step()
    print("gen_step() returned:", type(shape))
