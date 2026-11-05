"""Export SesAnt Jumper ("SesAnt Jumper") as one STEP assembly, e.g. to create a Tnkr project from the GitHub repo.

Usage: python jumper_step.py <work dir> <sesant root>   ->  <root>/SesAnt Jumper.step

Pose: standing, head level, arms in the carry pose (level, claws closed).
Every part is a named, coloured component. The STEP is built from the generated meshes: triangles are sewn into a
solid and coplanar faces merged, so flat-sided printed parts come out as clean planar faces and curved ones stay
faceted. It's meant for viewing, placement and import, not as editable parametric CAD.
Printed parts keep their shape (simplified within 0.1-0.2 mm, see TOL). Servos and electronics are written as
their oriented bounding boxes (enough to show size and position).
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(__file__))
sys.argv = sys.argv[:3]
WORK, ROOT = sys.argv[1], sys.argv[2]
src = open(os.path.join(os.path.dirname(__file__), 'jumper_arms.py')).read()
exec(compile(src[:src.index('# ----------------------------------------------------------------------------- 4. checks')], 'jumper_arms', 'exec'))

from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
from OCP.gp import gp_Pnt
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.TopoDS import TopoDS
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Compound
from OCP.BRep import BRep_Builder
import cadquery as cq

# STEP stores every facet as its own face (~2 KB per triangle), so the file size follows the triangle count. Printed
# parts are simplified by manifold3d within a deviation tolerance (no visible change at 0.1 mm), keeping the file
# around 45 MB, under GitHub's 50 MB warning size.
TOL = [('femur', 0.2), ('blade_foot', 0.2), ('chassis', 0.15)]
DEFAULT_TOL = 0.1
PROXY = ('servo', 'horn', 's2mini', 'battery', 'cores3')      # bought parts: written as their oriented bounding boxes

def mesh_to_shape(m):
    sew = BRepBuilderAPI_Sewing(1e-3)
    V = m.vertices
    for f in m.faces:
        p = BRepBuilderAPI_MakePolygon(gp_Pnt(*V[f[0]]), gp_Pnt(*V[f[1]]), gp_Pnt(*V[f[2]]), True)
        fc = BRepBuilderAPI_MakeFace(p.Wire(), True)
        if fc.IsDone(): sew.Add(fc.Face())
    sew.Perform(); sh = sew.SewedShape()
    comp = TopoDS_Compound(); B = BRep_Builder(); B.MakeCompound(comp)
    exp = TopExp_Explorer(sh, TopAbs_SHELL)
    while exp.More():                                        # every shell becomes a solid (a part may sew into several)
        ms = BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(exp.Current()))
        B.Add(comp, ms.Solid() if ms.IsDone() else exp.Current()); exp.Next()
    u = ShapeUpgrade_UnifySameDomain(comp, True, True, True); u.Build()
    return u.Shape()

def decimate(m, n):
    return m.simplify_quadric_decimation(face_count=n) if len(m.faces) > n else m

def hexcolor(c):
    c = c.lstrip('#'); return cq.Color(*(int(c[i:i + 2], 16) / 255 for i in (0, 2, 4)))

STAND = bot.stand_on_floor()
parts = full(STAND, L=ARM0, R=ARM0)                  # carry pose: arms level (yaw 0, pitch 0), claws closed
asm = cq.Assembly(name='SesAnt Jumper')
t0 = time.time(); nf = 0; seen = {}
for n, m, c in parts:
    if n in ('cores3_screen', 'cores3_camera'): continue
    if any(k in n for k in PROXY):
        m = m.bounding_box_oriented
    elif m.is_watertight:
        m = to_tm(to_mf(m).simplify(next((t for k, t in TOL if k in n), DEFAULT_TOL)))
    m = m.copy(); m.apply_scale(1.0)                         # mm
    nf += len(m.faces)
    seen[n] = seen.get(n, 0) + 1
    asm.add(cq.Shape.cast(mesh_to_shape(m)), name=n if seen[n] == 1 else f'{n}_{seen[n]}', color=hexcolor(c))
out = os.path.join(ROOT, 'SesAnt Jumper.step')
asm.export(out) if hasattr(asm, 'export') else asm.save(out)
print(f'{len(parts)} parts, {nf} triangles in, {time.time() - t0:.0f} s -> {out} ({os.path.getsize(out) / 1e6:.1f} MB)')
