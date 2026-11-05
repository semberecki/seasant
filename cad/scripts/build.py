"""SesAnt build script: generates the remix parts from the original RealAnt + Sesame STL/STEP data.

Coordinate frame (SesAnt): Z up, X = body length (face/OLED at -X = front), Y left/right (left = -Y),
origin at the centre of the Sesame internal frame footprint. All lengths in mm.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(__file__))
from common import *
import manifold3d as mf
from shapely.geometry import Polygon, MultiPolygon, box, Point
from shapely import affinity
from shapely.ops import unary_union

OUT = sys.argv[1] if len(sys.argv) > 1 else SCR + '/out'
os.makedirs(OUT, exist_ok=True)

# ----------------------------------------------------------------------------- parameters
D_SHIFT = 18.0          # hip pods pushed outward by (D,D) along each diagonal -> collision-free over full servo range
PLATE_Z0, PLATE_Z1 = 18.0, 21.97   # X-plate thickness, top flush with Sesame frame top (servo tabs rest at z=21.97)
POD_WALL_Z0 = 8.0       # pod wall bottom (servo body bottom is at z=6.17)
SPINE_HALF_W = 19.0     # keep |y| <= this from the Sesame internal frame
REALANT_SCALE = 0.90    # RealAnt body_v6 outline scale
BLADE_TIP_Z = -66.0     # new shin tip, in knee-local Z (Sesame foot tip is at -47)
SERVO_CLEAR = 0.25

HIP0 = {'FL': (-24.476, -25.384), 'FR': (-24.477, 25.416), 'BR': (23.923, 25.417), 'BL': (23.924, -25.383)}
SG = {'FL': (-1, -1), 'FR': (-1, 1), 'BR': (1, 1), 'BL': (1, -1)}
HIP = {k: (HIP0[k][0] + SG[k][0] * D_SHIFT, HIP0[k][1] + SG[k][1] * D_SHIFT) for k in HIP0}
# STEP component prefixes per leg: hip servo, femur, knee servo, foot
LEG = {'FL': ('01', '12', '14', '13', 'L1', 'L3'), 'FR': ('02', '16', '15', '17', 'R1', 'R3'),
       'BR': ('03', '05', '07', '06', 'R2', 'R4'), 'BL': ('04', '08', '10', '09', 'L2', 'L4')}
KNEE = {'FL': (-46.64, -55.3, 15.41), 'FR': (-46.77, 55.21, 15.41), 'BR': (45.87, 55.56, 15.23), 'BL': (45.87, -55.52, 15.23)}

# ----------------------------------------------------------------------------- helpers
def to_mf(m):
    m = m.copy(); m.merge_vertices(); m.remove_unreferenced_vertices()
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))

def to_tm(x):
    mm = x.to_mesh(); return trimesh.Trimesh(vertices=np.asarray(mm.vert_properties)[:, :3], faces=np.asarray(mm.tri_verts), process=True)

def cs_from(poly):
    polys = list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]
    contours = []
    for p in polys:
        contours.append(np.asarray(p.exterior.coords)[:-1])
        for h in p.interiors: contours.append(np.asarray(h.coords)[:-1])
    return mf.CrossSection(contours, mf.FillRule.EvenOdd)

def extrude(poly, z0, z1):
    return cs_from(poly).extrude(z1 - z0).translate([0, 0, z0])

def mat_mf(M):  # 4x4 -> manifold 3x4
    return M[:3, :4].T.flatten().reshape(4, 3).T if False else [list(M[0, :4]), list(M[1, :4]), list(M[2, :4])]

def xform(x, M):
    return x.transform(np.asarray(M[:3, :4], dtype=np.float64))

def rot_about(a_deg, c):
    return trimesh.transformations.rotation_matrix(np.radians(a_deg), [0, 0, 1], [c[0], c[1], 0])

def trans(v):
    M = np.eye(4); M[:3, 3] = v; return M

def rrect(u0, u1, v0, v1, r):
    return box(u0 + r, v0 + r, u1 - r, v1 - r).buffer(r, resolution=16)

def cyl(x, y, r, z0, z1, n=32):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])

def pod_angle(k):
    sx, sy = SG[k]; return np.degrees(np.arctan2(-sy, -sx))  # servo body points inward along diagonal

def local_to_world_xy(k, u, v):
    a = np.radians(pod_angle(k)); h = HIP[k]
    return h[0] + u * np.cos(a) - v * np.sin(a), h[1] + u * np.sin(a) + v * np.cos(a)

def pod_poly(k, poly_uv):
    a = pod_angle(k); h = HIP[k]
    return affinity.translate(affinity.rotate(poly_uv, a, origin=(0, 0)), h[0], h[1])

# SG90/MG90S geometry in pod-local frame: u = along body (inward), v = across, origin at output shaft
BODY_U = (-6.10, 16.40); BODY_V = (-6.10, 6.10); TAB_HOLES_U = (-8.75, 19.05)

# ----------------------------------------------------------------------------- 1. spine (Sesame frame core)
frame = L(P + 'Internal-Frame-v121.stl')
spine = to_mf(frame) ^ mf.Manifold.cube([100, 2 * SPINE_HALF_W, 40]).translate([-50, -SPINE_HALF_W, 0])

# ----------------------------------------------------------------------------- 2. RealAnt X-plate outline
body = L(R + 'body_v6.stl', None)
ra = max(hsec(body, -3), key=lambda p: p.area)
ra = affinity.scale(affinity.rotate(ra, 45, origin=(0, 0)), REALANT_SCALE, REALANT_SCALE, origin=(0, 0))
# keep only RealAnt outline + its arm hole rows; drop the central lightening windows (spine sits there)
plate2d = ra.difference(box(-39.6, -SPINE_HALF_W + 1.0, 39.6, SPINE_HALF_W - 1.0))
# pods: pad + walls
pads = []; cut2d = []; walls2d = []; slot2d = []; pilots = []
for k in HIP:
    pads.append(pod_poly(k, rrect(-13.5, 24.5, -9.2, 9.2, 4.0)))
    walls2d.append(pod_poly(k, rrect(-11.2, 21.4, -8.45, 8.45, 2.0)))
    cut2d.append(pod_poly(k, box(BODY_U[0] - SERVO_CLEAR, BODY_V[0] - SERVO_CLEAR, BODY_U[1] + SERVO_CLEAR, BODY_V[1] + SERVO_CLEAR)))
    slot2d.append(pod_poly(k, box(BODY_U[1], -3.5, 23, 3.5)))
    for u in TAB_HOLES_U: pilots.append(local_to_world_xy(k, u, 0))
plate2d = unary_union([plate2d] + pads)
# morphological closing of the outline only: fills RealAnt's forked arm-tip slits next to each pod
_outer = Polygon(plate2d.exterior).buffer(3.5, resolution=16).buffer(-3.5, resolution=16)
plate2d = Polygon(_outer.exterior, [h for h in plate2d.interiors if Polygon(h).within(_outer)])
# clear the RealAnt Dynamixel screw holes that land inside a pad (keep the look elsewhere)
plate2d = Polygon(plate2d.exterior, [h for h in plate2d.interiors if not any(Polygon(h).intersects(p.buffer(-0.5)) for p in pads)]) if isinstance(plate2d, Polygon) else plate2d
# femur sweep clearance: subtract the exact swept footprint of each femur (z <= plate top + 0.5) over -92..+92 deg
FEMUR_PRE = {'FL': '12', 'FR': '16', 'BR': '05', 'BL': '08'}
for k in HIP:
    fem = step(FEMUR_PRE[k]); fem.apply_transform(trans([SG[k][0] * D_SHIFT, SG[k][1] * D_SHIFT, 0]))
    foot2d = unary_union([g for z in np.arange(fem.bounds[0, 2] + 0.2, PLATE_Z1 + 0.6, 0.7) for g in hsec(fem, z)])
    h = HIP[k]
    swept = unary_union([affinity.rotate(foot2d, a, origin=h) for a in np.arange(-92, 93, 2)]).buffer(1.0)
    plate2d = plate2d.difference(swept)
plate2d = plate2d.buffer(-0.5, resolution=8).buffer(0.5, resolution=8).simplify(0.02)
if isinstance(plate2d, MultiPolygon):
    print('WARNING plate split into', len(plate2d.geoms), [round(g.area) for g in plate2d.geoms])
    plate2d = max(plate2d.geoms, key=lambda g: g.area)
plate = extrude(plate2d, PLATE_Z0, PLATE_Z1)
walls = extrude(unary_union(walls2d), POD_WALL_Z0, PLATE_Z1)
chassis = spine + plate + walls
chassis = chassis - extrude(unary_union(cut2d), -10, 40)
chassis = chassis - extrude(unary_union(slot2d), POD_WALL_Z0 - 1, PLATE_Z0 - 1.5)
for (x, y) in pilots:
    chassis = chassis - cyl(x, y, 0.85, 10, 23)
# re-drill Sesame top-cover screw holes + counterbores (accessible from below) and keep spine interior free
for (x, y) in [(-33.60, 15.93), (-33.60, -15.91), (36.06, 16.80), (36.06, -16.77)]:
    chassis = chassis - cyl(x, y, 1.15, 0, 23) - cyl(x, y, 2.55, -1, 19.4)
# wire pass-throughs from each pod into the spine (under the plate, through the old bay wall)
chassis_tm = to_tm(chassis)
print('manifold status', chassis.status(), 'genus', chassis.genus(), 'tm euler', chassis_tm.euler_number, 'broken faces', len(trimesh.repair.broken_faces(chassis_tm)))

# ----------------------------------------------------------------------------- 3. trimmed bottom cover
bot = to_mf(L(P + 'Bottom-Cover-v121.stl')) ^ mf.Manifold.cube([100, 2 * SPINE_HALF_W, 20]).translate([-50, -SPINE_HALF_W, -5])
bottom_tm = to_tm(bot)

# ----------------------------------------------------------------------------- 4. RealAnt blade shins on Sesame foot clamps
leg = L(R + 'leg_v8.stl', None)
Mxz = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)
blade_ra = max(psec(leg, [0, 0, 0], [0, 1, 0], Mxz), key=lambda p: p.area)
blade_ra = Polygon(blade_ra.exterior)  # outline only, holes re-added round
blade_ra = blade_ra.intersection(box(-240, -50, -160, 10))   # blade portion (tip at x=-233)
bx0, bz0, bx1, bz1 = blade_ra.bounds
zc = (bz0 + bz1) / 2
root_w = blade_ra.intersection(box(-162, -50, -160, 10)).bounds
sW = 19.0 / (root_w[3] - root_w[1])
CUT_Z = -21.0
sL = (CUT_Z - BLADE_TIP_Z) / (bx1 - bx0)
# map RealAnt (x,z) -> foot-local (X,Z): Z = CUT_Z - (bx1 - x)*sL, X = (z - zc)*sW
blade_loc = affinity.affine_transform(blade_ra, [0, sW, sL, 0, -zc * sW, CUT_Z - bx1 * sL])
# check orientation: x=bx1 -> Z=CUT_Z ; x=bx0 -> Z=CUT_Z-(bx1-bx0)*sL
blade_loc = blade_loc.union(box(-9.5, CUT_Z - 3, 9.5, CUT_Z + 0.35)).buffer(0.6, resolution=8).buffer(-0.6, resolution=8)
holes = [Point(0, CUT_Z - 10).buffer(3.2, 24), Point(0, CUT_Z - 22).buffer(2.4, 24), Point(0, CUT_Z - 32).buffer(1.7, 24)]
blade_loc = blade_loc.difference(unary_union(holes))

def knee_frame(k):
    K = np.array(KNEE[k]); r = np.array([SG[k][0], SG[k][1], 0.0]) / np.sqrt(2); zz = np.array([0, 0, 1.0]); t = np.cross(zz, r)
    W2L = np.eye(4); W2L[:3, :3] = np.vstack([r, t, zz]); W2L[:3, 3] = -W2L[:3, :3] @ K
    return W2L

new_feet_world0 = {}   # in original Sesame STEP pose (before D shift)
for k, (_, _, _, fpre, _, fname) in LEG.items():
    W2L = knee_frame(k); L2W = np.linalg.inv(W2L)
    foot = step(fpre); foot.apply_transform(W2L)
    y0, y1 = foot.bounds[0, 1], foot.bounds[1, 1]
    keep = to_mf(foot) ^ mf.Manifold.cube([60, 60, 60]).translate([-30, -30, CUT_Z - 1.5])
    bl = cs_from(blade_loc).extrude(y1 - y0)       # extrude along +Z, then rotate so Z->Y
    # CrossSection is in (X, Z_local) plane: map cross-section coords (a,b,c) -> (a, c, b)
    Mswap = np.array([[1, 0, 0, 0], [0, 0, 1, y0], [0, 1, 0, 0], [0, 0, 0, 1]], float)
    bl = xform(bl, Mswap)
    nf = keep + bl
    new_feet_world0[k] = (to_tm(nf), L2W, fname)

tc = L(P + 'top-covers/Top-Cover-Enclosed-v117.stl')

# ----------------------------------------------------------------------------- 5. StackChan head: CoreS3 face/camera integrated into the body cover
# Replaces Sesame's top cover + OLED. Same footprint and the same 4 screws from below as the Sesame cover.
# M5Stack CoreS3 = StackChan's head: 54 x 54 x 16 mm, ~73 g; GC0308 camera + proximity sensor on the front face BELOW the screen.
CORE_W, CORE_H, CORE_D, CLR = 54.0, 54.0, 16.0, 0.3
CAM_TILT_DEG = 0.0          # >0 tilts the face/camera down
HEAD_Z0 = 21.97             # cover sits on the spine top (same as Sesame cover)
ROOF_Z = 41.0               # body cover roof; femurs sweep up to z=39.4 next to the head
FACE_FLOOR_Z = 43.0         # underside of the wide face part: 3.6 mm above the femur sweep
WALL = 1.8
RIM = box(-39.6, -18.7, 39.6, 18.7).buffer(-1.5, join_style=2).buffer(1.5, join_style=1)
W, D = CORE_W + 2 * CLR, CORE_D + 2 * CLR
def box3(a0, a1, b0, b1, c0, c1):
    return mf.Manifold.cube([a1 - a0, b1 - b0, c1 - c0]).translate([a0, b0, c0])
# --- body cover (electronics bay over the spine)
cover = extrude(RIM, HEAD_Z0, ROOF_Z + 2.0) - extrude(RIM.buffer(-WALL, join_style=2), HEAD_Z0 - 1, ROOF_Z)
for (x, y) in [(-33.60, 15.93), (-33.60, -15.91), (36.06, 16.80), (36.06, -16.77)]:     # Sesame cover screw bosses
    cover += (cyl(x, y, 2.9, HEAD_Z0, HEAD_Z0 + 9) ^ extrude(RIM, 0, 80))
    cover -= cyl(x, y, 0.85, HEAD_Z0 - 1, HEAD_Z0 + 8)
for xw in (-22.0, 14.0):                                       # servo-lead entries at the bottom edge of each side
    cover -= box3(xw - 7, xw + 7, -30, 30, HEAD_Z0 - 1, HEAD_Z0 + 6)
cover -= box3(36.0, 45.0, -6.5, 6.5, HEAD_Z0 - 1, HEAD_Z0 + 7.5)   # S2 Mini USB-C at the back
cover -= box3(36.9, 38.7, -14.0, 14.0, HEAD_Z0 - 1, HEAD_Z0 + 6.5)    # relief for the S2 Mini PCB edge (board ends at x=38.2)
cover -= box3(22.5, 35.5, -6.5, 6.5, ROOF_Z - 1, ROOF_Z + 3)       # KCD1 rocker switch (cross-shaped cut fits 13 x 19 mm panel)
cover -= box3(26.5, 31.5, -9.6, 9.6, ROOF_Z - 1, ROOF_Z + 3)
# --- face: CoreS3 pocket standing on the roof, screen forward (-X)
FACE_X = -40.0              # CoreS3 front face plane
T = 2.5
face = box3(-T, D + T, -W / 2 - T, W / 2 + T, -3.0, 0.0)   # floor: underside at FACE_FLOOR_Z
face += box3(-T, D + T, -W / 2 - T, -W / 2, 0, CORE_H + 2 * CLR)          # side walls, full height
face += box3(-T, D + T, W / 2, W / 2 + T, 0, CORE_H + 2 * CLR)
face += box3(-T, D + T, -W / 2 - T, W / 2 + T, CORE_H + 2 * CLR, CORE_H + 2 * CLR + 2.0)   # top
for sb in (-1, 1):                                                        # front bezel lips on the sides (4.6 mm)
    face += box3(-T, 0, min(sb * (W / 2 - 4.6), sb * W / 2), max(sb * (W / 2 - 4.6), sb * W / 2), 0, CORE_H + 2 * CLR)
    face += box3(D - 1.0, D, min(sb * (W / 2 - 6), sb * W / 2), max(sb * (W / 2 - 6), sb * W / 2), 4, CORE_H - 4)   # back crush ribs
face += box3(-T, 0, -W / 2, W / 2, CORE_H + 2 * CLR - 3.0, CORE_H + 2 * CLR)   # top bezel lip (3 mm)
for sb in (-1, 1):                                                        # bottom corners only: camera (bottom-centre) stays open
    face += box3(-T, 0, min(sb * (W / 2 - 8), sb * W / 2), max(sb * (W / 2 - 8), sb * W / 2), 0, 4.0)
face -= box3(3.0, D - 3.0, -13, 13, -20, 1)                               # cable pass-through to the body (M-Bus / power)
for sb in (-1, 1):                                                        # rubber-band / zip-tie notches on the open back
    face -= box3(D + T - 1.2, D + T + 1, sb * (W / 2 + T) - 1.5 * sb - 1.5, sb * (W / 2 + T) - 1.5 * sb + 1.5, 20, 34)
Mface = trans([FACE_X, 0.0, FACE_FLOOR_Z + 3.0]) @ trimesh.transformations.rotation_matrix(np.radians(-CAM_TILT_DEG), [0, 1, 0], [0, 0, 0])
face_w = xform(face, Mface)
neck = extrude(box(FACE_X - T + 0.4, -W / 2 - T, FACE_X + D + T - 0.4, W / 2 + T).intersection(RIM.buffer(-0.5)), ROOF_Z, FACE_FLOOR_Z + 2.6)
head = cover + face_w + neck
head -= xform(box3(0, D, -W / 2, W / 2, 0, CORE_H + 2 * CLR), Mface)       # keep the pocket clear
head -= xform(box3(3.0, D - 3.0, -13, 13, -20, 1), Mface) ^ extrude(RIM.buffer(-WALL), ROOF_Z - 1, FACE_FLOOR_Z + 4)  # cable hole through roof
head_tm = to_tm(head)
core = box3(CLR, D - CLR, -CORE_W / 2, CORE_W / 2, 0.01, CORE_H)
core_tm = to_tm(xform(core, Mface))
screen_tm = to_tm(xform(box3(-0.2, 0.3, -20.3, 20.3, 17.5, 48.0), Mface))
camdot_tm = to_tm(xform(mf.Manifold.cylinder(0.5, 2.2, 2.2, 24).rotate([0, -90, 0]).translate([0.3, 0, 9.0]), Mface))
print('head watertight', head_tm.is_watertight, 'vol %.1f cm3' % (head_tm.volume / 1000), 'bounds', head_tm.bounds.round(1).tolist(), 'CoreS3 z %.1f..%.1f' % (core_tm.bounds[0, 2], core_tm.bounds[1, 2]))

# ----------------------------------------------------------------------------- 6. assembly (display + checks)
def leg_T(k):
    return trans([SG[k][0] * D_SHIFT, SG[k][1] * D_SHIFT, 0])

asm = []   # (name, mesh, color, group)
asm.append(('chassis', chassis_tm, '#f28c28', 'print'))
asm.append(('bottom_cover', bottom_tm, '#3a3f4a', 'print'))
asm.append(('stackchan_head', head_tm, '#f5a54a', 'print'))
asm.append(('cores3', core_tm, '#2b2d33', 'elec'))
asm.append(('cores3_screen', screen_tm, '#1f6feb', 'elec'))
asm.append(('cores3_camera', camdot_tm, '#111111', 'elec'))
for nm, k, c in [('s2mini', '18', '#2e7d32'), ('battery', '56', '#7bc96f')]:
    asm.append((nm, step(k), c, 'elec'))
for k, (hs, fe, ks, fpre, fname, ftname) in LEG.items():
    T = leg_T(k)
    h = step(hs); h.apply_transform(rot_about(pod_angle(k) - (0 if SG[k][0] < 0 else 180), HIP0[k])); h.apply_transform(T)
    asm.append((f'hip_servo_{k}', h, '#2b2d33', 'servo'))
    f = step(fe); f.apply_transform(T); asm.append((f'femur_{fname}', f, '#f5a54a', 'sesame'))
    s = step(ks); s.apply_transform(T); asm.append((f'knee_servo_{k}', s, '#2b2d33', 'servo'))
    nf, L2W, ftn = new_feet_world0[k]
    w = nf.copy(); w.apply_transform(L2W); w.apply_transform(T)
    asm.append((f'blade_foot_{ftn}', w, '#e9e2d0', 'print'))

import pickle
pickle.dump({'asm': asm, 'HIP': HIP, 'KNEE': KNEE, 'D': D_SHIFT, 'feet_local': {k: (v[0], v[1], v[2]) for k, v in new_feet_world0.items()},
             'blade_loc': blade_loc}, open(OUT + '/asm.pkl', 'wb'))
for nm, m, c, g in asm:
    if g == 'print' or nm.startswith('blade'):
        pass
print('chassis watertight', chassis_tm.is_watertight, 'vol %.1f cm3' % (chassis_tm.volume / 1000), 'bounds', chassis_tm.bounds.round(1).tolist())
print('bottom watertight', bottom_tm.is_watertight)
for k, v in new_feet_world0.items():
    print('foot', k, v[2], 'watertight', v[0].is_watertight, 'local bounds', v[0].bounds.round(1).tolist())
print('HIP', {k: tuple(np.round(v, 2)) for k, v in HIP.items()})
