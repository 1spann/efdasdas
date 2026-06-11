"""
TR-5000 Trash Rocket — Rocket Equipment (DWG 75024, spec sheet rev 0.04)

Transport / stowed configuration per the spec-sheet 3-view drawing:
  overall length  29'-0"   (hitch to hopper rear)
  overall width    8'-6"   (over fenders)
  overall height  12'-11"  (ground to top of chute stack)

Side-view stations from hitch: front stabilizers 4'-2", rear stabilizers
25'-6" (4'-2" + 21'-4"), frame rear 28'-1", hopper rear 29'-0".

Chute: four nested round telescoping tubes, OD 36" / 33" / 31" / 29.5",
1/8" aluminium wall, on a common centreline; flared intake hopper (54"
opening) at the rear; discharge deflector at the front; knuckle-boom lift
on a centre turret (turret rotation enables the perpendicular/parallel
site orientations shown in the setup guide).

Units: mm.  Origin: hitch point at ground level; -X runs to the rear,
+Z up, Y=0 is the trailer centreline.
"""

from build123d import (
    BuildPart, BuildSketch, Box, Cylinder, Rectangle, Location, Locations,
    Plane, Align, Mode, loft,
)
from cadpy.assembly import AssemblyHelper
import math


def ft(v):  return v * 304.8
def inch(v): return v * 25.4


# ---------------------------------------------------------------------------
# Drawing-controlled dimensions
# ---------------------------------------------------------------------------

OVERALL_LEN   = ft(29)            # 29'-0"  hitch -> hopper rear
OVERALL_HALFW = ft(8.5) / 2       # 8'-6" overall width over fenders
STACK_TOP_Z   = inch(12*12 + 11)  # 12'-11" ground -> top of chute stack

STA_FRONT_LEGS = -ft(4 + 2/12)            # 4'-2"
STA_REAR_LEGS  = -(ft(4+2/12) + ft(21+4/12))   # 4'-2" + 21'-4" = 25'-6"
STA_FRAME_REAR = -ft(28 + 1/12)           # 28'-1"
STA_HOPPER_END = -OVERALL_LEN             # 29'-0"

# Chute tube stack (cross-section view callouts)
TUBE_ODS   = [inch(36), inch(33), inch(31), inch(29.5)]
TUBE_WALL  = inch(0.125)          # 1/8" aluminium sidewall
TUBE_LEN   = ft(16)               # per-section length (scaled from drawing)
CHUTE_CL_Z = STACK_TOP_Z - TUBE_ODS[0] / 2   # common tube centreline

# Stowed tube stagger: x of each section's FRONT end (from side view)
TUBE_FRONT_X = [-inch(32), -ft(6+8/12), -ft(8), -ft(8.5)]

HOPPER_SMALL_X = TUBE_FRONT_X[3] - TUBE_LEN   # = -24'-6", rear of tube 4
HOPPER_OPEN_W  = inch(54)         # flared opening width
HOPPER_WALL    = TUBE_WALL

# Trailer / running gear (scaled from side + front views)
DECK_TOP_Z   = inch(28.6)
FRAME_H      = inch(6)
FRAME_W      = inch(80)
TIRE_OD      = inch(32)
TIRE_W       = inch(9.5)
AXLE_SPACING = inch(36)
AXLE_GROUP_X = -ft(15 + 4/12)     # tandem group centre from hitch
FENDER_W     = inch(10)

TURRET_X = -ft(12)

# Kinematic frames (also consumed by the URDF generator, tr5000_1_urdf.py)
COLUMN_TOP_Z  = DECK_TOP_Z + inch(36)        # slew-ring seat / turret joint
BOOM_PIVOT_Z  = 1700.0                       # boom lift pivot above ground
BOOM_ANGLE_DEG = 38.8                        # stowed boom angle from horizontal
SADDLE_X      = -2012.0                      # chute saddle / chute pitch pivot
SADDLE_TOP_Z  = CHUTE_CL_Z - TUBE_ODS[0]/2   # tube-1 underside at the saddle

# Link frame origins in model coordinates (mm)
LINK_FRAMES = {
    "base_link": (0.0, 0.0, 0.0),
    "turret":    (TURRET_X, 0.0, COLUMN_TOP_Z),
    "boom":      (TURRET_X, 0.0, BOOM_PIVOT_Z),
    "chute_1":   (SADDLE_X, 0.0, SADDLE_TOP_Z),
    "chute_2":   (SADDLE_X, 0.0, SADDLE_TOP_Z),
    "chute_3":   (SADDLE_X, 0.0, SADDLE_TOP_Z),
    "chute_4":   (SADDLE_X, 0.0, SADDLE_TOP_Z),
}


# ---------------------------------------------------------------------------
# Part builders (centred at origin unless noted; caller positions via moved)
# ---------------------------------------------------------------------------

def make_frame():
    length = -(STA_FRAME_REAR + ft(6))            # frame front at -6'
    with BuildPart() as p:
        Box(length, FRAME_W, FRAME_H)
    return p.part


def make_tongue():
    with BuildPart() as p:
        Box(ft(8), inch(6), inch(4))
    return p.part


def make_coupler():
    with BuildPart() as p:
        Box(inch(12), inch(4), FRAME_H)
    return p.part


def make_tongue_jack():
    with BuildPart() as p:
        Cylinder(radius=inch(1.25), height=inch(22))
    return p.part


def make_axle():
    with BuildPart() as p:
        Cylinder(radius=inch(1.75), height=inch(90), rotation=(90, 0, 0))
    return p.part


def make_wheel():
    with BuildPart() as p:
        Cylinder(radius=TIRE_OD/2, height=TIRE_W, rotation=(90, 0, 0))
        Cylinder(radius=inch(4), height=TIRE_W + inch(1.5),
                 rotation=(90, 0, 0), mode=Mode.ADD)
    return p.part


def make_fender():
    with BuildPart() as p:
        Box(inch(80), FENDER_W, inch(2.5))
    return p.part


def make_stabilizer():
    """Square jack post with ground foot, deployed to grade."""
    foot_t = inch(0.5)
    with BuildPart() as p:
        Box(inch(3), inch(3), DECK_TOP_Z,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        Box(inch(8), inch(8), foot_t,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
    return p.part


def make_turret_column():
    with BuildPart() as p:
        Box(inch(16), inch(16), inch(36),
            align=(Align.CENTER, Align.CENTER, Align.MIN))
    return p.part


def make_slew_ring():
    """Rotating turret bearing — the slew (URDF turret) link."""
    with BuildPart() as p:
        Cylinder(radius=inch(11), height=inch(2.5),
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    return p.part


def make_battery_box():
    with BuildPart() as p:
        Box(inch(16), inch(24), inch(20))
    return p.part


def make_brace():
    with BuildPart() as p:
        Box(inch(3), inch(3), 1367)
    return p.part


def make_boom():
    with BuildPart() as p:
        Box(2112, inch(10), inch(8))
    return p.part


def make_cylinder_barrel():
    with BuildPart() as p:
        Cylinder(radius=inch(2), height=1100)
    return p.part


def make_cylinder_rod():
    with BuildPart() as p:
        Cylinder(radius=inch(1), height=1000)
    return p.part


def make_saddle():
    with BuildPart() as p:
        Box(inch(12), inch(36), inch(6))
    return p.part


def make_tube(od, length):
    """Round telescoping chute section, open both ends, axis along X."""
    with BuildPart() as p:
        Cylinder(radius=od/2, height=length, rotation=(0, 90, 0))
        Cylinder(radius=od/2 - TUBE_WALL, height=length + 2,
                 rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    return p.part


def make_chute_rest():
    """Transport rest post under the rear of tube 1."""
    post_h = (CHUTE_CL_Z - TUBE_ODS[0]/2) - DECK_TOP_Z - inch(4)
    with BuildPart() as p:
        Box(inch(4), inch(4), post_h,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((0, 0, post_h)):
            Box(inch(6), inch(36), inch(4),
                align=(Align.CENTER, Align.CENTER, Align.MIN))
    return p.part


def make_hopper():
    """Flared intake hopper lofted from the tube-4 collar to the 54" opening."""
    small_w = inch(30)
    small_h = inch(30)
    large_w = HOPPER_OPEN_W
    large_h = inch(110)
    # top edge exactly at the 12'-11" stack top; body flares downward to
    # the low rear rest as in the side-view drawing
    large_cz = STACK_TOP_Z - large_h/2
    x_small, x_large = HOPPER_SMALL_X, STA_HOPPER_END
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(x_small)):
            with Locations((0, CHUTE_CL_Z)):
                Rectangle(small_w, small_h)
        with BuildSketch(Plane.YZ.offset(x_large)):
            with Locations((0, large_cz)):
                Rectangle(large_w, large_h)
        loft()
        with BuildSketch(Plane.YZ.offset(x_small + 10)):
            with Locations((0, CHUTE_CL_Z)):
                Rectangle(small_w - 2*HOPPER_WALL, small_h - 2*HOPPER_WALL)
        with BuildSketch(Plane.YZ.offset(x_large - 10)):
            with Locations((0, large_cz)):
                Rectangle(large_w - 2*HOPPER_WALL, large_h - 2*HOPPER_WALL)
        loft(mode=Mode.SUBTRACT)
    return p.part


def make_hopper_rest():
    rest_h = (STACK_TOP_Z - inch(110)) - DECK_TOP_Z   # deck to hopper bottom
    with BuildPart() as p:
        Box(inch(6), inch(24), rest_h,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
    return p.part


def make_deflector():
    with BuildPart() as p:
        Box(1000, inch(40), inch(2))
    return p.part


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------

def build_groups():
    """Positioned parts grouped by kinematic link (stowed = zero pose).

    Groups feed both the STEP assembly (gen_step) and the per-link mesh
    export for the URDF. The hydraulic cylinder stays in base_link: URDF
    trees cannot close the boom/cylinder loop, so it is correct only in
    the stowed pose (documented cosmetic limitation).
    """
    groups = {name: [] for name in LINK_FRAMES}

    def add(link, shape, *label):
        groups[link].append((shape, label))

    frame_cz = DECK_TOP_Z - FRAME_H/2

    # --- base_link: trailer and everything that never articulates ---
    frame_len = -(STA_FRAME_REAR + ft(6))
    add("base_link",
        make_frame().moved(Location((-ft(6) - frame_len/2, 0, frame_cz))),
        "trailer_frame")
    add("base_link", make_tongue().moved(Location((-ft(4), 0, frame_cz))),
        "tongue")
    add("base_link", make_coupler().moved(Location((-inch(6), 0, frame_cz))),
        "coupler")
    add("base_link",
        make_tongue_jack().moved(Location((-ft(3), inch(5), inch(17)))),
        "tongue_jack")

    wheel_cz = TIRE_OD/2
    for i, ax in enumerate([AXLE_GROUP_X + AXLE_SPACING/2,
                            AXLE_GROUP_X - AXLE_SPACING/2]):
        add("base_link", make_axle().moved(Location((ax, 0, wheel_cz))),
            "axle", str(i+1))
        for side, ys in [("left", 1), ("right", -1)]:
            add("base_link",
                make_wheel().moved(Location((ax, ys*inch(45.2), wheel_cz))),
                "wheel", f"{i+1}_{side}")

    fender_cy = OVERALL_HALFW - FENDER_W/2
    for side, ys in [("left", 1), ("right", -1)]:
        add("base_link",
            make_fender().moved(
                Location((AXLE_GROUP_X, ys*fender_cy, inch(34.5)))),
            "fender", side)

    for sta_name, sx in [("front", STA_FRONT_LEGS), ("rear", STA_REAR_LEGS)]:
        for side, ys in [("left", 1), ("right", -1)]:
            add("base_link",
                make_stabilizer().moved(
                    Location((sx, ys*(FRAME_W/2 - inch(1.5)), 0))),
                "stabilizer", f"{sta_name}_{side}")

    add("base_link",
        make_turret_column().moved(Location((TURRET_X, 0, DECK_TOP_Z))),
        "turret_column")
    add("base_link",
        make_battery_box().moved(
            Location((TURRET_X + inch(16), 0, DECK_TOP_Z + inch(11)))),
        "battery_box")
    for side, ys, rot in [("left", 1, -48), ("right", -1, 48)]:
        add("base_link",
            make_brace().moved(
                Location((TURRET_X, ys*inch(20), 1183), (rot, 0, 0))),
            "brace", side)

    add("base_link",
        make_cylinder_barrel().moved(Location((-3884, 0, 1256), (0, 43.3, 0))),
        "lift_cylinder_barrel")
    add("base_link",
        make_cylinder_rod().moved(Location((-3242, 0, 1938), (0, 43.3, 0))),
        "lift_cylinder_rod")
    add("base_link",
        make_chute_rest().moved(Location((-ft(18), 0, DECK_TOP_Z))),
        "chute_rest")
    add("base_link",
        make_hopper_rest().moved(
            Location((STA_FRAME_REAR + inch(12), 0, DECK_TOP_Z))),
        "hopper_rest")

    # --- turret: rotating slew ring on top of the column ---
    add("turret",
        make_slew_ring().moved(Location((TURRET_X, 0, COLUMN_TOP_Z))),
        "slew_ring")

    # --- boom: lift arm from turret pivot toward the saddle ---
    add("boom",
        make_boom().moved(Location((-2835, 0, 2362), (0, -BOOM_ANGLE_DEG, 0))),
        "lift_boom")

    # --- chute_1: outer tube + saddle + discharge deflector ---
    add("chute_1",
        make_saddle().moved(Location((SADDLE_X, 0, SADDLE_TOP_Z - inch(3)))),
        "chute_saddle")
    add("chute_1",
        make_deflector().moved(Location((-506, 0, 2685), (0, 50, 0))),
        "discharge_deflector")

    # --- chute_1..chute_4: telescoping sections; hopper rides chute_4 ---
    for i, (od, fx) in enumerate(zip(TUBE_ODS, TUBE_FRONT_X)):
        cx = fx - TUBE_LEN/2
        add(f"chute_{i+1}",
            make_tube(od, TUBE_LEN).moved(Location((cx, 0, CHUTE_CL_Z))),
            "chute_section", str(i+1))
    add("chute_4", make_hopper(), "hopper")

    return groups


def gen_step():
    asm = AssemblyHelper("tr5000_trash_rocket")
    for link, parts in build_groups().items():
        for shape, label in parts:
            asm.add(shape, *label)
    return asm.build()


if __name__ == "__main__":
    shape = gen_step()
    print("gen_step() returned:", type(shape))
