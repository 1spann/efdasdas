"""
Per-link STL mesh export for the TR-5000 URDF (tr5000_1_urdf.py).

Each link's parts are gathered from tr5000_1.build_groups() and shifted
into that link's local frame (LINK_FRAMES) so the URDF can mount every
mesh with a zero visual origin. Meshes are exported in millimetres; the
URDF applies scale 0.001.

Run:  python tr5000_1_meshes.py
"""

from pathlib import Path
from build123d import Compound, Location, export_stl
from tr5000_1 import build_groups, LINK_FRAMES

MESH_DIR = Path(__file__).parent / "tr5000_1_meshes"
LINEAR_TOL_MM = 0.8
ANGULAR_TOL_DEG = 0.4


def export_link_meshes():
    MESH_DIR.mkdir(exist_ok=True)
    written = []
    for link, parts in build_groups().items():
        fx, fy, fz = LINK_FRAMES[link]
        shapes = [shape.moved(Location((-fx, -fy, -fz)))
                  for shape, _label in parts]
        mesh = Compound(children=shapes)
        out = MESH_DIR / f"{link}.stl"
        export_stl(mesh, str(out),
                   tolerance=LINEAR_TOL_MM,
                   angular_tolerance=ANGULAR_TOL_DEG)
        written.append(out)
    return written


if __name__ == "__main__":
    for path in export_link_meshes():
        print(f"wrote {path} ({path.stat().st_size/1024:.0f} KiB)")
