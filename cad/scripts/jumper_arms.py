"""SesAnt Jumper (1/2): two Jumper-style front arms, each ending in a two-jaw Jumper claw.

Usage: python jumper_arms.py <work dir with asm.pkl, v2.pkl, v3.pkl> <sesant root>

Like Jumper's front limbs: SesAnt gets a left and a right arm in front of its four walking legs.
Per arm, 3 MG90S:
  * shoulder yaw   - the servo hangs shaft-down in the arm-base housing (on the chassis' front plate tongue)
  * shoulder pitch - lies under the yaw horn on a shoulder bracket, shaft pointing outward; the forearm hangs on its horn
  * gripper        - lies on the inside of the forearm, right behind the shoulder (no servo at the claw). Its horn drives
                     a 1.2 mm wire pushrod along the outside of the forearm to a lever on the finger jaw; a printed gear
                     pair turns the palm jaw the other way, so BOTH jaws swing open together. The claw grabs horizontally
                     (palm on the inside, finger outside); at the claw there is only a slim plate holding the two pivots.
The jaws are Jumper's palm and finger, outline projected from Jumper's URDF meshes (KingKong Robotics, Apache-2.0),
scaled by CLAW_SCALE. Left arm = mirror image of the right arm.
Joint signs: yaw + = arm swings outward, pitch + = claw goes down, jaw + = open (per jaw).
"""
import sys, os, pickle, json
sys.path.insert(0, os.path.dirname(__file__))
import v3_neck_jaw as v3
from common import *
from cadlib import *
from poses import SG, knee_units, support_margin, Robot
from jumper_urdf import JumperURDF
from trimesh.collision import CollisionManager
from shapely.geometry import LineString

WORK, ROOT = sys.argv[1], sys.argv[2]
OUT = ROOT + '/sesant_jumper'
for s in ('stl', 'images', 'assembly'): os.makedirs(f'{OUT}/{s}', exist_ok=True)

# ----------------------------------------------------------------------------- parameters (right arm, arm at yaw 0 / pitch 0)
CLAW_SCALE = 0.34
JAW_OPEN = 35.0            # each jaw opens 35 deg (70 deg between them)
GM, GZ = 1.0, 14           # jaw gear pair: module 1, 14 teeth -> pivots 14 mm apart
RPJ = GM * GZ / 2
JX, SY = -55.75, 16.0      # shoulder yaw axes (x, +-y)
HZ = -7.0 + v3.HORN_TOP    # yaw servo: inverted, horn underside at z = -7
PLATE = (-10.0, -7.0)      # shoulder bracket plate, screwed under the yaw horn
XP = JX - 2.0              # shoulder pitch axis x ...
ZP = PLATE[0] - 0.3 - 6.15 # ... and z (pitch servo lies under the plate, 12.3 mm thick)
YB = 5.0                   # pitch servo bottom face (inner) y; horn face at YB + 30.65
L_ARM = 58.0               # pitch axis -> finger-jaw pivot
YF = YB + v3.HORN_TOP + 0.25 + 1.5      # forearm plate centre plane (on the pitch horn)
XG = XP - L_ARM            # finger-jaw pivot x (gripper shaft) ...
ZB = ZP                    # ... and z: horizontal 5 mm jaws centred on the pitch-axis line
YG = YF                    # finger-jaw pivot y (in line with the forearm plate); the palm-jaw pivot is 2*RPJ further inside
XS = XP - 19.5 - 6.15      # gripper servo shaft x (servo body from XP - 19.5 forward, clear of the pitch servo at any pitch)
YGB = YF - 1.5 - 0.6 - 22.0  # gripper servo body bottom face y (body just inside the forearm plate, horn through it)
RH, RL = 7.0, 7.5          # pushrod linkage: servo horn radius, finger-jaw lever radius
YR = YGB + v3.HORN_TOP + 0.65   # pushrod line y (outside the forearm plate, beside the horn)
LEV0 = 90.0 + JAW_OPEN / 2  # lever direction at jaw 0 (deg, in the jaw plane): perpendicular to the rod at mid-stroke
YJ = YG                    # (kept for the pickle / sim)
CLR = 0.4
SIDES = {'r': +1, 'l': -1}
YAW_RANGE = (-25.0, 40.0)  # inward / outward
PITCH_RANGE = (-30.0, 70.0)

# ----------------------------------------------------------------------------- 1. jaws: Jumper palm + finger, geared
J = JumperURDF()
PIV = J.joints['LF_J4_joint']['M'][:3, 3] * 1000.0

def plane_outline(names):
    polys = []
    for n, m in J.meshes(frame='LF_palm_link', names=names):
        tri = m.vertices[m.faces][:, :, 1:3]
        a = np.abs((tri[:, 1, 0] - tri[:, 0, 0]) * (tri[:, 2, 1] - tri[:, 0, 1]) - (tri[:, 1, 1] - tri[:, 0, 1]) * (tri[:, 2, 0] - tri[:, 0, 0]))
        polys += [Polygon(t) for t in tri[a > 1e-4]]
    return unary_union(polys).buffer(0.05).buffer(-0.05)

palm_j = plane_outline(['LF_palm_link', 'LF_palm_pad_f_link', 'LF_palm_pad_b_link', 'LF_palm_grip_insert_link'])
fing_j = plane_outline(['LF_finger_link', 'LF_finger_tip_link', 'LF_finger_grip_insert_link'])
# claw-local 2D frame: finger pivot at the origin, claw pointing -x, palm on the -y (inner) side; it maps to the horizontal
# plane as world x = XG + x, y = YG + y (right arm), jaws 5 mm thick around z = ZB
to_local = lambda g: affinity.affine_transform(g, [-CLAW_SCALE, 0, 0, -CLAW_SCALE, CLAW_SCALE * PIV[1], CLAW_SCALE * PIV[2]])
PA = (0.0, -2 * RPJ)       # palm-jaw pivot (geared to the finger-jaw pivot at the origin)
TOOTH, RFJ, RAJ = v3.involute_tooth(GM, GZ)
def gear(center_deg, ks, phase=0.0):
    g = [Point(0, 0).buffer(RFJ + 0.01, resolution=64)]
    g += [affinity.rotate(TOOTH, center_deg + phase + k * 360.0 / GZ, origin=(0, 0)) for k in ks]
    return unary_union(g)
nt = int(np.ceil(JAW_OPEN / (360.0 / GZ))) + 1
gearB = gear(-90.0, range(-1, nt + 1))                                          # finger jaw turns clockwise when opening
gearA = affinity.translate(gear(90.0, range(-nt - 1, 2), phase=180.0 / GZ), *PA)   # palm jaw counter-clockwise
openB = lambda g, a: affinity.rotate(g, -a, origin=(0, 0))
openA = lambda g, a: affinity.rotate(g, a, origin=PA)
fing0 = to_local(fing_j).simplify(0.05)
palm0 = to_local(palm_j).simplify(0.05).intersection(box(-80, -40, 4.0, 40))      # drop Jumper's wrist housing behind the pivot
angles = np.linspace(-1.0, JAW_OPEN, 37)
# palm jaw: cut where the finger jaw (with its gear) passes, in the palm jaw's moving frame; then add its own gear
B_full = unary_union([fing0, gearB])
palm_cut = palm0.difference(unary_union([openA(openB(B_full, a), -a) for a in angles]).buffer(CLR))
palm_cut = max(palm_cut.geoms, key=lambda g: g.area) if palm_cut.geom_type == 'MultiPolygon' else palm_cut
jawA = unary_union([palm_cut, gearA]).difference(Point(*PA).buffer(1.1, 16))     # M2 pivot screw
A_body = palm_cut
# finger jaw: cut where the palm jaw body passes (its gear meshes with ours by design)
fing_cut = fing0.difference(unary_union([openB(openA(A_body, a), -a) for a in angles]).buffer(CLR))
fing_cut = max(fing_cut.geoms, key=lambda g: g.area) if fing_cut.geom_type == 'MultiPolygon' else fing_cut
LEVER = LineString([(0, 0), (RL * np.cos(np.radians(LEV0)), RL * np.sin(np.radians(LEV0)))]).buffer(2.6, resolution=16)
jawB = unary_union([fing_cut, gearB, LEVER]).difference(Point(0, 0).buffer(1.1, 16))
jawB = jawB.difference(Point(RL * np.cos(np.radians(LEV0)), RL * np.sin(np.radians(LEV0))).buffer(0.75, 16))   # pushrod Z-bend hole


# ----------------------------------------------------------------------------- 2. arm parts (right arm, then mirrored)
def servo_box(M):
    c = np.array([[x, y, z] for x in v3.SRV_BODY[:2] for y in v3.SRV_BODY[2:4] for z in v3.SRV_BODY[4:]])
    w = trimesh.transform_points(c, M); return np.array([w.min(0), w.max(0)])
R_P = np.array([[-1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)   # servo x -> -x (body forward), y -> z, z (shaft) -> +y
M_pitch_srv = trans([XP, YB, ZP]) @ R_P
M_grip_srv = trans([XS, YGB, ZP]) @ R_P                                         # lying like the pitch servo, shaft outward (+y)
M_yaw_srv = trans([JX, SY, HZ]) @ rotz(180) @ rot(180, [1, 0, 0])               # as the twin-claw design: body lying forward
PB, GB, YBX = servo_box(M_pitch_srv), servo_box(M_grip_srv), servo_box(M_yaw_srv)
tabs = lambda M: [trimesh.transform_points([[tx, 0, sum(v3.TAB_Z) / 2]], M)[0] for tx in v3.TAB_X]

# shoulder bracket (yaws): plate on the yaw horn, inner cheek, two tab posts on the outer side of the pitch servo's tabs
sh = extrude(rrect(PB[0, 0] - 5.0, JX + 9.5, YB, PB[1, 1] + 4.5, 4.0), PLATE[0], PLATE[1])
sh += box3(PB[0, 0], PB[1, 0], YB - 2.4, YB - 0.2, PB[0, 2] - 0.5, PLATE[0] + 0.01)
for tp in tabs(M_pitch_srv):
    sh += box3(tp[0] - 2.6, tp[0] + 2.6, YB + 18.7, PB[1, 1] + 4.5, PB[0, 2] - 0.5, PLATE[0] + 0.01)
    sh -= mf.Manifold.cylinder(10, 0.85, 0.85, 12).rotate([-90, 0, 0]).translate([tp[0], YB + 16, tp[2]])
sh -= box3(PB[0, 0] - 0.3, PB[1, 0] + 0.3, PB[0, 1] - 0.3, PB[1, 1] + 0.3, PB[0, 2] - 0.3, PB[1, 2] + 0.3)
sh -= box3(PB[0, 0] - 5.3, PB[1, 0] + 5.3, YB + 15.7, YB + 18.8, PB[0, 2] - 0.3, PB[1, 2] + 0.3)     # tab slots
sh -= cyl(JX, SY, 1.25, PLATE[0] - 1, PLATE[1] + 1, 16)                                             # yaw horn screw
for hx_ in (-6.0, 6.0):
    sh -= cyl(JX + hx_, SY, 0.8, PLATE[0] - 1, PLATE[1] + 1, 12)
shoulder = sh

# forearm (pitches): plate on the pitch horn, gripper servo on its inside (tab posts), a slim claw plate with both pivots
def xz_plate(poly2d, y0, y1):
    """2D profile in (x, z) extruded between y0 and y1"""
    return extrude(poly2d, 0, y1 - y0).rotate([90, 0, 0]).translate([0, y1, 0])
fa = xz_plate(unary_union([LineString([(XP, ZP), (XG + 15.0, ZP)]).buffer(6.5, cap_style=1),
                           rrect(XS - 12.0, XS + 9.5, ZP - 10.0, ZP + 10.0, 3.0)]), YF - 1.5, YF + 1.5)   # taller round the servo's output boss
fa -= mf.Manifold.cylinder(5, 1.25, 1.25, 16).rotate([-90, 0, 0]).translate([XP, YF - 3, ZP])        # pitch horn screw
for dx in (-6.0, 6.0):
    fa -= mf.Manifold.cylinder(5, 0.8, 0.8, 12).rotate([-90, 0, 0]).translate([XP + dx, YF - 3, ZP])
GTAB_Y = (YGB + v3.TAB_Z[0], YGB + v3.TAB_Z[1])
for tp in tabs(M_grip_srv):                                                                       # tab posts, outside the tabs
    fa += box3(tp[0] - 2.6, tp[0] + 2.6, GTAB_Y[1] + 0.1, YF - 1.4, ZP - 6.15, ZP + 6.15)
    fa -= mf.Manifold.cylinder(9, 0.85, 0.85, 12).rotate([-90, 0, 0]).translate([tp[0], GTAB_Y[0] - 2, ZP])
fa -= box3(GB[0, 0] - 0.3, GB[1, 0] + 0.3, GB[0, 1] - 0.3, GB[1, 1] + 0.3, GB[0, 2] - 0.3, GB[1, 2] + 0.3)
fa -= box3(GB[0, 0] - 5.3, GB[1, 0] + 5.3, GTAB_Y[0] - 0.3, GTAB_Y[1] + 0.1, GB[0, 2] - 0.3, GB[1, 2] + 0.3)        # tab slots
fa -= box3(XS - 8.85, XS + 6.65, YF - 6.0, YF + 6.0, ZP - 6.45, ZP + 6.45)                             # slot for the output boss + horn (rails above and below)
PAW = (XG + PA[0], YG + PA[1])
fa += extrude(rrect(XG - 7.5, XG + 11.0, PAW[1] - 6.0, YG + 3.0, 4.0), ZB + 2.9, ZB + 5.4)            # claw plate over both jaw pivots
for (x, y) in [(XG, YG), PAW]:
    fa -= cyl(x, y, 0.8, ZB, ZB + 6.0, 12)                                                            # M2 pivot screws (self-tapping)
forearm = fa

jA = extrude(affinity.translate(jawA, XG, YG), ZB - 2.5, ZB + 2.5)
jB = extrude(affinity.translate(jawB, XG, YG), ZB - 2.5, ZB + 2.5)
jB -= cyl(XG, YG, 2.4, ZB - 3.5, ZB - 0.7, 24)                  # pivot screw heads (from below)
jA -= cyl(PAW[0], PAW[1], 2.4, ZB - 3.5, ZB - 0.7, 24)

# pushrod linkage (right-arm coordinates, arm at pitch 0): servo horn point H(theta) in the x-z plane, lever point L(jaw) in the jaw plane
SHAFT = np.array([XS, YR, ZP])
def horn_pt(th):  return SHAFT + RH * np.array([-np.sin(np.radians(th)), 0.0, np.cos(np.radians(th))])
def lever_pt(a):
    ang = np.radians(LEV0 - a)
    return np.array([XG + RL * np.cos(ang), YG + RL * np.sin(ang), ZB + 3.3])
ROD_L = np.linalg.norm(horn_pt(0.0) - lever_pt(JAW_OPEN / 2))
def theta_for(a):
    ths = np.linspace(-70, 70, 1401)
    err = np.array([np.linalg.norm(horn_pt(t) - lever_pt(a)) - ROD_L for t in ths])
    i0 = np.argmin(np.abs(err)); return float(ths[i0]), float(err[i0])
JAW_TABLE = np.linspace(0, JAW_OPEN, 36)
TH_TABLE = np.array([theta_for(a)[0] for a in JAW_TABLE])
def horn_M(a):
    th = np.interp(a, JAW_TABLE, TH_TABLE)
    return rot(-(90.0 + th), [0, 1, 0], (XS, 0, ZP))
def pushrod_mesh(a):
    th = np.interp(a, JAW_TABLE, TH_TABLE)
    return trimesh.creation.cylinder(radius=0.6, segment=[horn_pt(th), lever_pt(a)], sections=12)

MIRROR = np.diag([1.0, -1.0, 1.0, 1.0])
def raw_tm(x):
    mm_ = x.to_mesh()
    return trimesh.Trimesh(np.asarray(mm_.vert_properties)[:, :3], np.asarray(mm_.tri_verts), process=False)
def mesh_side(x, s):
    m = raw_tm(x) if not isinstance(x, trimesh.Trimesh) else x.copy()
    if s == 'l': m.apply_transform(MIRROR)
    return m
PARTS = {}
for s in SIDES:
    PARTS[s] = {'shoulder': mesh_side(shoulder, s), 'forearm': mesh_side(forearm, s), 'jaw_palm': mesh_side(jA, s), 'jaw_finger': mesh_side(jB, s),
                'pitch_servo': mesh_side(v3.servo_at(M_pitch_srv), s), 'grip_servo': mesh_side(v3.servo_at(M_grip_srv), s),
                'yaw_servo': mesh_side(v3.servo_at(M_yaw_srv), s),
                'yaw_horn': mesh_side(v3.servo_at(rotz(180, (JX, SY)) @ M_yaw_srv, horn=True), s),
                'pitch_horn': mesh_side(v3.servo_at(M_pitch_srv, horn=True), s),
                'grip_horn': mesh_side(v3.servo_at(M_grip_srv, horn=True), s), 'pushrod': mesh_side(pushrod_mesh(0.0), s)}

# arm-base housing: holds the two shoulder yaw servos lying forward
FW = min(t[0] for t in tabs(M_yaw_srv)) - 2.35 - 2.6
YO = YBX[1, 1] + 2.4
TABS_Y = tabs(M_yaw_srv)
house = extrude(rrect(FW, -39.8, -YO, YO, 5.0), v3.HEAD_Z0, 27.0)
house += extrude(rrect(-42.4, -39.8, -14.0, 14.0, 1.0), 26.0, 38.0)
house += box3(FW, FW + 2.4, -YO, YO, 1.0, 22.5)
for sg in (-1, 1):
    house += box3(FW, -45.0, min(sg * YO, sg * (YO - 2.4)), max(sg * YO, sg * (YO - 2.4)), 1.0, 17.0)
    house += box3(FW, -49.6, min(sg * YO, sg * (YO - 2.4)), max(sg * YO, sg * (YO - 2.4)), 1.0, 22.5)
house += box3(FW, -49.6, -YBX[0, 1] + 0.3, YBX[0, 1] - 0.3, -2.0, 22.5)
TZ_ = trimesh.transform_points([[0, 0, v3.TAB_Z[0]], [0, 0, v3.TAB_Z[1]]], M_yaw_srv)[:, 2]
for sg in (-1, 1):
    y0, y1 = (YBX[0, 1] - 0.6, YBX[1, 1] + 0.6) if sg > 0 else (-YBX[1, 1] - 0.6, -YBX[0, 1] + 0.6)
    house += box3(min(t[0] for t in TABS_Y) - 2.4, YBX[0, 0] - 0.3, y0, y1, TZ_[0] + 0.4, 22.5)
    house += box3(YBX[1, 0] + 0.3, -45.0, y0, y1, TZ_[0] + 0.4, 17.0)
for sg in (-1, 1):
    b = YBX if sg > 0 else np.array([[YBX[0, 0], -YBX[1, 1], YBX[0, 2]], [YBX[1, 0], -YBX[0, 1], YBX[1, 2]]])
    house -= box3(b[0, 0] - 0.3, b[1, 0] + 0.3, b[0, 1] - 0.3, b[1, 1] + 0.3, 1.3, 23.95)
    house -= box3(min(t[0] for t in TABS_Y) - 2.6, max(t[0] for t in TABS_Y) + 2.6, b[0, 1] - 0.3, b[1, 1] + 0.3, TZ_[1] - 0.4, TZ_[0] + 0.4)
    for t in TABS_Y:
        house -= cyl(t[0], sg * SY, 0.85, 3.0, 17.0, 16)
    house -= cyl(JX, sg * SY, 7.5, -8, 1.3, 48)
    house -= box3(-49.0, -44.0, sg * SY - 3, sg * SY + 3, 11, 28)
house -= box3(-50.8, -37.0, -40, 40, 17.6, v3.HEAD_Z0) + box3(-40.0, -37.0, -40, 40, -10, v3.HEAD_Z0)
for y in (-10.0, 10.0):
    house -= mf.Manifold.cylinder(6, 1.1, 1.1, 16).rotate([0, 90, 0]).translate([-44.0, y, 33.0])
_hm = house.to_mesh()
house_tm = trimesh.Trimesh(np.asarray(_hm.vert_properties)[:, :3], np.asarray(_hm.tri_verts), process=False)

# ----------------------------------------------------------------------------- 3. kinematics + robot
def M_yaw(s, a):   return rotz(-SIDES[s] * a, (JX, SIDES[s] * SY))
def M_pitch(a):    return rot(-a, [0, 1, 0], (XP, 0, ZP))
def M_jawB(s, a):  return rotz(-SIDES[s] * a, (XG, SIDES[s] * YG))              # finger jaw swings outward
def M_jawA(s, a):  return rotz(SIDES[s] * a, (XG, SIDES[s] * (YG + PA[1])))    # palm jaw swings inward
COL = {'shoulder': '#f5a54a', 'forearm': '#f5a54a', 'jaw_palm': '#c9ccd2', 'jaw_finger': '#d9a441', 'pitch_servo': '#2b2d33', 'grip_servo': '#2b2d33',
       'yaw_servo': '#2b2d33', 'yaw_horn': '#e8e8e8', 'pitch_horn': '#e8e8e8', 'grip_horn': '#e8e8e8', 'pushrod': '#8d939b'}
LINK = {'yaw_servo': 'base', 'shoulder': 'yaw', 'yaw_horn': 'yaw', 'pitch_servo': 'yaw', 'pitch_horn': 'pitch', 'forearm': 'pitch', 'grip_servo': 'pitch',
        'grip_horn': 'pitch', 'pushrod': 'pitch', 'jaw_finger': 'jawB', 'jaw_palm': 'jawA'}

def arm_M(s, link, yaw, pitch, jaw):
    if link == 'base': return np.eye(4)
    M = M_yaw(s, yaw)
    if link == 'yaw': return M
    M = M @ M_pitch(pitch)
    if link == 'pitch': return M
    return M @ (M_jawB(s, jaw) if link == 'jawB' else M_jawA(s, jaw))

STATIC = [(n, m, c) for n, m, c in v3.STATIC if n not in ('jaw_housing', 'jaw_servo')] + [('arm_housing', house_tm, '#f28c28')] + \
         [(f'yaw_servo_{s}', PARTS[s]['yaw_servo'], '#2b2d33') for s in SIDES]
d = pickle.load(open(WORK + '/asm.pkl', 'rb'))
bot = Robot(d, replace={'stackchan_head': None, 'cores3': None, 'cores3_screen': None, 'cores3_camera': None, 'bottom_cover': v3.cov2_tm},
            extra=STATIC)
ARM0 = {'yaw': 0.0, 'pitch': 0.0, 'jaw': 0.0}

def arms(T=np.eye(4), L=None, R=None, only=None):
    out = []
    for s, q in (('l', L or ARM0), ('r', R or ARM0)):
        for n, m in PARTS[s].items():
            if n == 'yaw_servo' or (only and not only(n)): continue
            if n == 'pushrod':
                mm = mesh_side(pushrod_mesh(q['jaw']), s)
            else:
                mm = m.copy()
                if n == 'grip_horn': mm.apply_transform(horn_M(q['jaw']))     # rotation about the y axis: same for the mirrored arm
            mm.apply_transform(T @ arm_M(s, LINK[n], q['yaw'], q['pitch'], q['jaw'])); out.append((f'arm_{s}_{n}', mm, COL[n]))
    return out

def head(T=np.eye(4), pan=0.0, tilt=0.0):
    out = []
    for n, m, c in v3.NECK:
        mm = m.copy(); mm.apply_transform(T @ v3.neck_M(pan)); out.append((n, mm, c))
    for n, m, c in v3.HEAD:
        mm = m.copy(); mm.apply_transform(T @ v3.head_M(pan, tilt)); out.append((n, mm, c))
    return out

def full(T=np.eye(4), hips=None, knees=None, pan=0.0, tilt=0.0, L=None, R=None):
    return bot.pose(T, hips, knees) + head(T, pan, tilt) + arms(T, L, R)

SERVO_NAMES = ('yaw_servo', 'pitch_servo', 'grip_servo')
def part_mass(n, m):
    return v3.MASS['servo'] if any(n.endswith(x) for x in SERVO_NAMES) or n.startswith('yaw_servo') else v3.part_mass(n, m)
def com(parts):
    ms = np.array([part_mass(n, m) for n, m, c in parts]); cs = np.array([m.center_mass if m.is_watertight else m.bounds.mean(0) for n, m, c in parts])
    return (ms[:, None] * cs).sum(0) / ms.sum(), ms.sum()

# ----------------------------------------------------------------------------- 4. checks
rep = []
def P(*s):
    line = ' '.join(str(x) for x in s); print(line); rep.append(line)
P('== SesAnt Jumper arms verification ==')
P(f'arm: shoulder yaw at ({JX}, +-{SY}), pitch axis ({XP}, z {ZP:.2f}), forearm {L_ARM:.0f} mm, gripper axis ({XG:.2f}, +-{YG:.2f}); '
  f'jaws (horizontal: palm inside, finger outside): Jumper palm + finger x{CLAW_SCALE}, gear pair m{GM} z{GZ} (pivots {2 * RPJ:.0f} mm apart)')
gb0 = gearB; ga0 = gearA
ov, md = [], []
for a in np.linspace(0, JAW_OPEN, 36):
    ga, gb = openA(ga0, a), openB(gb0, a)
    ov.append(ga.intersection(gb).area); md.append(ga.distance(gb))
bodies = [openA(A_body, a).distance(openB(unary_union([fing_cut, gearB]), a)) for a in np.linspace(0, JAW_OPEN, 36)] + \
         [openB(fing_cut, a).distance(openA(unary_union([A_body, gearA]), a)) for a in np.linspace(0, JAW_OPEN, 36)]
tipA = lambda a: np.array(openA(A_body, a).exterior.coords); tipB = lambda a: np.array(openB(fing_cut, a).exterior.coords)
ta, tb = tipA(JAW_OPEN), tipB(JAW_OPEN)
P(f'jaw gears: max overlap {max(ov):.3f} mm2 over 0..{JAW_OPEN:.0f} deg, closest {min(md):.2f} mm -> {"mesh without binding" if max(ov) < 0.05 else "INTERFERENCE"}; '
  f'jaw bodies never closer than {min(bodies):.2f} mm to the other jaw')
P(f'both jaws open together: closed grip gap {A_body.distance(fing_cut):.1f} mm; open {JAW_OPEN:.0f} + {JAW_OPEN:.0f} deg -> tips '
  f'{np.hypot(*(ta[np.argmin(ta[:, 0])] - tb[np.argmin(tb[:, 0])])):.0f} mm apart; claw {np.ptp(np.vstack([np.array(A_body.exterior.coords), np.array(fing_cut.exterior.coords)])[:, 0]):.0f} mm long')

errs = [theta_for(a)[1] for a in JAW_TABLE]
P(f'pushrod linkage (no servo at the claw): horn r {RH:.0f}, lever r {RL:.1f}, rod {ROD_L:.1f} mm; jaws 0..{JAW_OPEN:.0f} deg <- gripper servo '
  f'{TH_TABLE[0]:+.1f}..{TH_TABLE[-1]:+.1f} deg ({"monotonic" if (np.all(np.diff(TH_TABLE) > 0) or np.all(np.diff(TH_TABLE) < 0)) else "NOT MONOTONIC"}, '
  f'about {abs(TH_TABLE[-1] - TH_TABLE[0]) / JAW_OPEN:.2f} servo deg per jaw deg), loop closure within {max(map(abs, errs)):.2f} mm')
rod_hits = []
for a in np.linspace(0, JAW_OPEN, 8):
    th = np.interp(a, JAW_TABLE, TH_TABLE); h, l = horn_pt(th), lever_pt(a); u = (l - h) / np.linalg.norm(l - h)
    rod = trimesh.creation.cylinder(radius=0.6, segment=[h + 2.5 * u, l - 2.5 * u], sections=12)
    cmr = CollisionManager(); cmr.add_object('forearm', raw_tm(forearm)); cmr.add_object('grip_servo', v3.servo_at(M_grip_srv))
    jb_ = raw_tm(jB); jb_.apply_transform(M_jawB('r', a)); ja_ = raw_tm(jA); ja_.apply_transform(M_jawA('r', a))
    cmr.add_object('jaw_finger', jb_); cmr.add_object('jaw_palm', ja_)
    hit, nm = cmr.in_collision_single(rod, return_names=True)
    if hit: rod_hits.append((round(a), sorted(nm)))
P('pushrod (ends trimmed 2.5 mm where it hooks in) vs forearm, gripper servo and both jaws over the jaw range:', 'clear' if not rod_hits else rod_hits)
# fit
cmfit = CollisionManager(); cmfit.add_object('h', house_tm); cmfit.add_object('chassis', d['asm'][0][1]); cmfit.add_object('cover', v3.cover_tm)
fit = {f'yaw_servo_{s}': ('clear' if not cmfit.in_collision_single(PARTS[s]['yaw_servo']) else 'HIT') for s in SIDES}
for nm, part, srv in [('shoulder', 'shoulder', 'pitch_servo'), ('forearm', 'forearm', 'grip_servo')]:
    c1 = CollisionManager(); c1.add_object(nm, PARTS['r'][part])
    fit[f'{srv} in {nm}'] = 'clear' if not c1.in_collision_single(PARTS['r'][srv]) else 'HIT'
c2 = CollisionManager(); c2.add_object('forearm', PARTS['r']['forearm'])
fit['jaws under the claw plate'] = 'clear' if not (c2.in_collision_single(PARTS['r']['jaw_palm']) or c2.in_collision_single(PARTS['r']['jaw_finger'])) else 'HIT'
ovh = (house ^ to_mf(d['asm'][0][1])).volume()
fit['housing vs chassis overlap'] = f'{ovh:.2f} mm3'
P('static fit:', fit)

# self sweeps: each arm through yaw x pitch x jaw against the body and the other arm (at rest)
body_cm = CollisionManager()
for n, m, c in bot.pose():
    if n.startswith(('hip_servo', 'femur', 'knee_servo', 'blade')) or n in ('chassis', 'arm_housing', 'v3_body_cover', 'bottom_cover', 'belly_cone') or n.startswith('yaw_servo'):
        body_cm.add_object(n, m)
for n, m, c in head():
    body_cm.add_object(n, m)
def arm_hits(s, q, other=None):
    parts = arms(L=q if s == 'l' else (other or ARM0), R=q if s == 'r' else (other or ARM0))
    mine = [(n, m) for n, m, c in parts if n.startswith(f'arm_{s}_')]
    theirs = [(n, m) for n, m, c in parts if not n.startswith(f'arm_{s}_')]
    cm = CollisionManager()
    for n, m in theirs: cm.add_object(n, m)
    hits = set()
    for n, m in mine:
        own_mount = {f'yaw_servo_{s}'} if n.endswith(('shoulder', 'yaw_horn')) else set()
        h, nm = body_cm.in_collision_single(m, return_names=True)
        hits |= (set(nm) - own_mount - ({'arm_housing'} if n.endswith('yaw_horn') else set()))
        h2, nm2 = cm.in_collision_single(m, return_names=True)
        hits |= set(nm2)
    return hits
p_up = 0.0                                                                        # how far up the arm can lift before it meets the housing
while p_up - 5 >= -15 and not any(arm_hits('r', {'yaw': 0.0, 'pitch': p_up - 5, 'jaw': j}) for j in (0.0, JAW_OPEN)): p_up -= 5
p_up_hit = sorted(arm_hits('r', {'yaw': 0.0, 'pitch': p_up - 5, 'jaw': 0.0}))
PITCH_RANGE = (p_up, PITCH_RANGE[1])
P(f'shoulder pitch: arm lifts up to {-p_up:.0f} deg' + (f' (then {p_up_hit})' if p_up_hit else ' (capped: enough to clear the puck when docking)') + f', down to {PITCH_RANGE[1]:.0f} deg -> pitch range {PITCH_RANGE}')
free_yaw = {}
for yaw in np.arange(-25.0, 40.1, 5.0):
    hits = []
    for pitch in np.arange(PITCH_RANGE[0], PITCH_RANGE[1] + 0.1, 20):
        for jaw in (0.0, JAW_OPEN):
            h = arm_hits('r', {'yaw': yaw, 'pitch': pitch, 'jaw': jaw})
            if h: hits.append((pitch, jaw, sorted(h)))
    free_yaw[yaw] = hits
lo = 0.0
while lo - 5 in free_yaw and not free_yaw[lo - 5]: lo -= 5
hi = 0.0
while hi + 5 in free_yaw and not free_yaw[hi + 5]: hi += 5
assert not free_yaw[0.0], ('arm at yaw 0 collides', free_yaw[0.0][:3])
YAW_RANGE = (lo, hi)
P(f'arm sweep (right arm; left is its mirror), standing legs, head, other arm at rest, every pitch {PITCH_RANGE[0]:.0f}..+{PITCH_RANGE[1]:.0f} (20 deg steps) '
  f'x jaws closed/open: collision-free for yaw {lo:.0f}..+{hi:.0f} deg (5 deg steps) -> joint range; beyond: inward {free_yaw.get(lo - 5, [[None, None, ["-"]]])[0][2]}, '
  f'outward {free_yaw.get(hi + 5, [[None, None, ["-"]]])[0][2]}')
# both arms swinging together (inward pinch / outward)
both = []
for yaw in np.arange(YAW_RANGE[0], YAW_RANGE[1] + 0.1, 5):
    q = {'yaw': yaw, 'pitch': 20.0, 'jaw': JAW_OPEN}
    both.append((yaw, sorted(arm_hits('r', q, other=q))))
pinch = min((y for y, h in both if not h), default=None)
P(f'both arms together (pitch 20, jaws open): collision-free from yaw {pinch:.0f} (inward) to {max(y for y, h in both if not h):.0f}; '
  f'first contact inward at {max((y for y, h in both if h and y < 0), default=float("nan")):.0f} deg: {next((h for y, h in both[::-1] if h and y < 0), [])}')
YAW_JOINT = (max(YAW_RANGE[0], pinch), YAW_RANGE[1])
P(f'-> yaw joint limit {YAW_JOINT} (both arms may move at once)')
# front legs vs arms at rest / arms swung out
def hip_range(q):
    cm = CollisionManager()
    for n, m, c in arms(L=q, R=q): cm.add_object(n, m)
    cm.add_object('arm_housing', house_tm)
    res = {}
    for k in ('FL', 'FR'):
        legm = [m for n, m, c, l, kd in bot.parts if l == k]
        ok = [a for a in range(-90, 91, 3) if not any(cm.in_collision_single(m, transform=bot.M_hip(k, a)) for m in legm)]
        lo = 0
        while lo - 3 in ok: lo -= 3
        hi = 0
        while hi + 3 in ok: hi += 3
        res[k] = (lo, hi)
    return res
CARRY = {'yaw': 0.0, 'pitch': 0.0, 'jaw': 0.0}                     # walking: arms level (lifting them crowds the knee servos)
DOCK = {'yaw': 0.0, 'pitch': max(-15.0, PITCH_RANGE[0]), 'jaw': 0.0}   # docking: arms lifted clear of the puck (legs don't swing)
P(f'front hips with the arms in the carry pose (yaw 0, pitch {CARRY["pitch"]:.0f}, jaws closed):', hip_range(CARRY))
P(f'front hips with the arms swung out to +{YAW_RANGE[1]:.0f} deg, jaws open:', hip_range({'yaw': YAW_RANGE[1], 'pitch': 0.0, 'jaw': JAW_OPEN}))

# torque on the shoulder pitch servo (arm horizontal)
pitch_parts = [(n, m) for n, m in PARTS['r'].items() if LINK[n] in ('pitch', 'jawA', 'jawB')]
mp = np.array([part_mass(n, m) for n, m in pitch_parts]); cp = np.array([m.center_mass if m.is_watertight else m.bounds.mean(0) for n, m in pitch_parts])
lever = abs(((mp[:, None] * cp).sum(0) / mp.sum())[0] - XP)
P(f'shoulder pitch load, arm horizontal: {mp.sum():.0f} g at {lever:.0f} mm -> {mp.sum() * lever / 10 / 1000:.2f} kg*cm = '
  f'{mp.sum() * lever / 10 / 1000 / 2.2 * 100:.0f} % of MG90S stall; grip per claw ~{2.2 * 9.81 / 2 / 2.2:.1f} N per jaw at 22 mm (est.)')

# reach the floor while standing
STAND = bot.stand_on_floor()
def jaw_low(T, q):
    return min(m.bounds[0, 2] for n, m, c in arms(T, L=q, R=q, only=lambda n: n.startswith('jaw')))
pdn = None
for p in np.arange(0, PITCH_RANGE[1] + 0.1, 0.5):
    if jaw_low(STAND, {'yaw': 0.0, 'pitch': p, 'jaw': JAW_OPEN}) <= 2.0: pdn = p; break
REACH = {'yaw': 0.0, 'pitch': pdn, 'jaw': JAW_OPEN}
reach_parts = full(STAND, tilt=45, L=REACH, R=REACH)
tips_ = np.vstack([m.vertices[np.argmin(m.vertices[:, 2])] for n, m, c in reach_parts if 'jaw' in n])
c_r, m_r = com(reach_parts)
gm = support_margin(c_r, [bot.tip_pts(k, 0, 0, STAND).mean(0) for k in SG])
cmg = CollisionManager(); cmg.add_object('floor', trimesh.creation.box([600, 600, 2], transform=trans([0, 0, -1.0])))
P(f'reach the floor standing: pitch {pdn:.1f} deg down puts both open claws 2 mm above the floor at x {tips_[:, 0].mean():.0f} mm '
  f'(body front -39.6); CoM margin {gm:.0f} mm; floor contacts 0.3 mm lower: {sorted({n for n, m, c in full(trans([0, 0, -0.3]) @ STAND, tilt=45, L=REACH, R=REACH) if cmg.in_collision_single(m)})}')
def cam_to(T, target):
    best_ = None
    for t in np.arange(v3.TILT_RANGE[0], v3.TILT_RANGE[1] + 0.1, 0.5):
        Mh = T @ v3.head_M(0, t); cam = trimesh.transform_points([v3.CAM0], Mh)[0]; fwd = Mh[:3, :3] @ [-1, 0, 0]; v = target - cam
        e = np.degrees(np.arctan2(np.cross(fwd, v)[1], np.dot(fwd, v)))
        if best_ is None or abs(e) < abs(best_[1]): best_ = (t, e, cam)
    return best_
tgt = np.array([tips_[:, 0].mean(), 0.0, tips_[:, 2].mean()])
t_, e_, camw = cam_to(STAND, tgt)
P(f'  head tilt {t_:.1f} deg looks at the claws (residual {e_:.1f} deg); claws at +-{np.degrees(np.arctan2(abs(tips_[:, 1]).mean(), np.linalg.norm(tgt - camw))):.0f} deg either side of the camera axis')
# extremes for stability
worst = 99
for y in (0, YAW_RANGE[1]):
    for p in (PITCH_RANGE[0], 0, 30):
        q = {'yaw': y, 'pitch': p, 'jaw': 0}
        worst = min(worst, support_margin(com(full(STAND, L=q, R=q))[0], [bot.tip_pts(k, 0, 0, STAND).mean(0) for k in SG]))
c0, mtot = com(full(STAND, L=CARRY, R=CARRY))
P(f'mass ~{mtot:.0f} g (est.); standing CoM margin >= {worst:.0f} mm over arm poses (yaw 0/{YAW_RANGE[1]:.0f}, pitch {PITCH_RANGE[0]:.0f}/0/+30)')
v2 = v3.v2
cmp_ = CollisionManager(); cmp_.add_object('puck', v2['puck']); cmp_.add_object('flag', v2['flag'])
seat_kn = {k: bot.knee_for(k, 0.0, trans([0, 0, v2['SEAT_DZ']])) for k in SG}
P(f'on the v2 home puck (arms lifted {-DOCK["pitch"]:.0f} deg for docking), pushed 0.3 mm down: contacts =',
  sorted({n for n, m, c in full(trans([0, 0, v2['SEAT_DZ'] - 0.3]), knees=seat_kn, L=DOCK, R=DOCK) if cmp_.in_collision_single(m)}))

# ----------------------------------------------------------------------------- 5. export
info = {'sesant_jumper_arm_housing': save_stl(house_tm, OUT + '/stl/sesant_jumper_arm_housing.stl', rot(180, [1, 0, 0]))}
for s, nm in (('r', 'right'), ('l', 'left')):
    info[f'sesant_jumper_shoulder_{nm}'] = save_stl(PARTS[s]['shoulder'], OUT + f'/stl/sesant_jumper_shoulder_{nm}.stl', rot(180, [1, 0, 0]))
    info[f'sesant_jumper_forearm_{nm}'] = save_stl(PARTS[s]['forearm'], OUT + f'/stl/sesant_jumper_forearm_{nm}.stl', rot(90 * SIDES[s], [1, 0, 0]))
    info[f'sesant_jumper_jaw_palm_{nm}'] = save_stl(PARTS[s]['jaw_palm'], OUT + f'/stl/sesant_jumper_jaw_palm_{nm}.stl')
    info[f'sesant_jumper_jaw_finger_{nm}'] = save_stl(PARTS[s]['jaw_finger'], OUT + f'/stl/sesant_jumper_jaw_finger_{nm}.stl')
for n, m in info.items():
    e = m.extents; P(f'{n:28s} {e[0]:6.1f} x {e[1]:6.1f} x {e[2]:5.1f} mm  {m.volume / 1000:5.1f} cm3')
open(OUT + '/assembly/sesant_jumper_arms_report.txt', 'w').write('\n'.join(rep) + '\n')
json.dump({'yaw_range_deg': YAW_JOINT, 'pitch_range_deg': PITCH_RANGE, 'jaw_open_deg': JAW_OPEN, 'carry': CARRY, 'dock': DOCK, 'reach_pitch_deg': pdn,
           'reach_head_tilt_deg': t_, 'shoulder_xy': [JX, SY], 'pitch_axis_xz': [XP, ZP], 'gripper_xy': [XG, YG]},
          open(OUT + '/assembly/sesant_jumper_arms.json', 'w'), indent=1)
pickle.dump({'STATIC': STATIC, 'PARTS': PARTS, 'LINK': LINK, 'COL': COL, 'house': house_tm,
             'axes': {'JX': JX, 'SY': SY, 'XP': XP, 'ZP': ZP, 'XG': XG, 'YG': YG, 'ZB': ZB, 'PA': PA, 'YJ': YJ},
             'ranges': {'yaw': YAW_JOINT, 'pitch': PITCH_RANGE, 'jaw': (0.0, JAW_OPEN)}, 'CARRY': CARRY, 'DOCK': DOCK, 'REACH': REACH, 'REACH_TILT': t_,
             'stand': full(STAND, pan=-20, tilt=25, L={'yaw': 0, 'pitch': 5, 'jaw': 30}, R={'yaw': 8, 'pitch': -10, 'jaw': 15}),
             'reach': reach_parts, 'seat': full(trans([0, 0, v2['SEAT_DZ']]), knees=seat_kn, L=DOCK, R=DOCK), 'puck_items': v2['puck_items'],
             'claw_closed': arms(L=ARM0, R=ARM0), 'claw_open': arms(L={'yaw': 0, 'pitch': 0, 'jaw': JAW_OPEN}, R={'yaw': 0, 'pitch': 0, 'jaw': JAW_OPEN}),
             'palm_j': palm_j, 'fing_j': fing_j, 'PIV': PIV, 'jawA2d': jawA, 'jawB2d': jawB, 'STAND_DZ': STAND[2, 3]},
            open(WORK + '/jumper.pkl', 'wb'))
