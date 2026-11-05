"""SesAnt v3: StackChan neck + ant mandibles.

Usage: python v3_neck_jaw.py <work dir with asm.pkl and v2.pkl> <sesant root>

v3 = v1 + v2 belly cone (still docks on the home puck) +
  * a pan/tilt NECK: the CoreS3 head moves off the body cover onto two Feetech SCS0009 serial servos
    (the servos StackChan's own body uses: same IDs, protocol and 0.3125 deg/step mapping), so the camera can look
    around and down while the body walks;
  * a pair of ANT MANDIBLES under the front of the body: one MG90S turns the left mandible, a printed involute gear
    pair mirrors it onto the right one. The robot crouches nose-down to pick small objects off the floor while the
    head looks down at its own mandibles.
Robot frame as v1: Z up, front at -X, z = -1 is the belly. All lengths in mm. SCS0009 is modelled with the SG90
body from the Sesame STEP (same 23 x 12 mm class; check against your servos).
"""
import sys, os, pickle, json
sys.path.insert(0, os.path.dirname(__file__))
from common import *
from cadlib import *
from poses import Robot, SG, knee_units, support_margin
from trimesh.collision import CollisionManager
from shapely.geometry import LineString, MultiPoint

WORK, ROOT = sys.argv[1], sys.argv[2]
OUT = ROOT + '/v3'
for s in ('stl', 'images', 'assembly'): os.makedirs(f'{OUT}/{s}', exist_ok=True)
d = pickle.load(open(WORK + '/asm.pkl', 'rb'))
v2 = pickle.load(open(WORK + '/v2.pkl', 'rb'))
A = {n: m for n, m, c, g in d['asm']}

# ----------------------------------------------------------------------------- servo model (SG90 class), local frame:
# output shaft on the Z axis at x = y = 0, body bottom at z = 0, body extends toward +X, tabs at z 16..18.5, horn top z 30.65
SRV = step('01'); SRV.apply_translation([24.45, 25.35, -6.17])
SRV_BODY = (-6.15, 16.35, -6.2, 6.2, 0.0, 22.0)
TAB_X = (-8.75, 19.05); TAB_Z = (16.0, 18.5); HORN_TOP = 30.65

SRV_HORN = SRV.slice_plane([0, 0, 26.4], [0, 0, 1], cap=True)       # horn: moves with whatever it drives
SRV = SRV.slice_plane([0, 0, 26.4], [0, 0, -1], cap=True)            # case + output cap

def servo_at(M, horn=False):
    m = (SRV_HORN if horn else SRV).copy(); m.apply_transform(M); return m

# ----------------------------------------------------------------------------- parameters
NX = -22.0                 # neck (pan) axis x; also the tilt axis x and the head's centre of mass
PAN_Z0 = 22.5              # pan servo body bottom (just above the spine top)
ROOF_Z = 41.0              # body cover roof underside (v1)
PAN_HORN = PAN_Z0 + HORN_TOP                     # 53.15
TT_Z0, TT_Z1 = PAN_HORN, PAN_HORN + 3.0          # turntable on the pan horn
TZ = TT_Z1 + 6.25          # tilt axis height (tilt servo lies on the turntable, shaft along +Y)
TILT_Y0 = -14.0            # tilt servo body bottom face at y = -14 -> horn face at y = +16.65
HEAD_H = 12.0              # head floor underside above the tilt axis
HB = TZ + HEAD_H           # head floor underside
CORE_W, CORE_H, CORE_D, CLR, T = 54.0, 54.0, 16.0, 0.3, 2.5
W, D = CORE_W + 2 * CLR, CORE_D + 2 * CLR
HX = NX - (D / 2)          # CoreS3 front plane: CoreS3 centred over both axes
TILT_RANGE = (-20, 60)     # + = nod down (look down)
PAN_RANGE = (-120, 120)    # cable twist limit; mechanically free all round
JX = -56.5                 # mandible pivot x
GEAR_M, GEAR_Z = 1.25, 14  # involute gear pair, 20 deg pressure angle
RP = GEAR_M * GEAR_Z / 2   # pitch radius 8.75 -> pivots 17.5 apart
JYA, JYB = -RP, RP         # left mandible (on the servo) / right mandible (on an M3 pivot)
MZ0, MZ1 = -12.0, -7.0     # mandible plane (horn underside at -7)
JAW_OPEN = 40.0            # each mandible opens 40 deg
RIM = box(-39.6, -18.7, 39.6, 18.7).buffer(-1.5, join_style=2).buffer(1.5, join_style=1)
HEAD_Z0 = 21.97
WALL = 1.8

# ----------------------------------------------------------------------------- 1. body cover (v1 cover without the face) + neck collar + jaw seat
cover = extrude(RIM, HEAD_Z0, ROOF_Z + 2.0) - extrude(RIM.buffer(-WALL, join_style=2), HEAD_Z0 - 1, ROOF_Z)
for (x, y) in [(-33.60, 15.93), (-33.60, -15.91), (36.06, 16.80), (36.06, -16.77)]:
    cover += (cyl(x, y, 2.9, HEAD_Z0, HEAD_Z0 + 9) ^ extrude(RIM, 0, 80))
    cover -= cyl(x, y, 0.85, HEAD_Z0 - 1, HEAD_Z0 + 8)
for xw in (-22.0, 14.0):
    cover -= box3(xw - 7, xw + 7, -30, 30, HEAD_Z0 - 1, HEAD_Z0 + 6)
cover -= box3(36.0, 45.0, -6.5, 6.5, HEAD_Z0 - 1, HEAD_Z0 + 7.5)
cover -= box3(36.9, 38.7, -14.0, 14.0, HEAD_Z0 - 1, HEAD_Z0 + 6.5)
cover -= box3(22.5, 35.5, -6.5, 6.5, ROOF_Z - 1, ROOF_Z + 3)
cover -= box3(26.5, 31.5, -9.6, 9.6, ROOF_Z - 1, ROOF_Z + 3)
# neck collar: stadium boss around the pan servo, hides the servo top, carries the tab screws
collar2d = rrect(NX - 12.5, NX + 22.5, -9.5, 9.5, 6.0)
cover += chamfered_slab(collar2d, ROOF_Z, PAN_Z0 + 26.0, 1.5, steps=6)
M_pan = trans([NX, 0, PAN_Z0])
cover -= xform(box3(SRV_BODY[0] - 0.3, SRV_BODY[1] + 0.3, SRV_BODY[2] - 0.3, SRV_BODY[3] + 0.3, -1, 40), M_pan)
for tx in TAB_X:                                            # M2 screws from below through the tabs into the collar
    cover -= cyl(NX + tx, 0, 0.85, ROOF_Z - 1, PAN_Z0 + 25.0, 16)
cover -= box3(NX + 24.5, NX + 30.5, -5, 5, ROOF_Z - 1, ROOF_Z + 3)          # head/neck cable loop into the body
cover -= box3(-40.0, -37.0, -4.0, 4.0, HEAD_Z0 - 1, 26.0)                    # jaw servo lead into the body
for y in (-10.0, 10.0):                                                      # 2x M2 + nuts: jaw housing lip to the cover front wall
    cover -= mf.Manifold.cylinder(6, 1.1, 1.1, 16).rotate([0, 90, 0]).translate([-42.5, y, 33.0])
cover_tm = to_tm(cover)

# ----------------------------------------------------------------------------- 2. jaw housing (separate print, sits on the chassis' front plate tongue)
M_jaw = trans([JX, JYA, MZ1 + HORN_TOP]) @ rotz(90) @ rot(180, [1, 0, 0])    # inverted servo, body toward +Y, horn underside at z = -7
jaw_srv = servo_at(M_jaw)
_c = np.array([[x, y, z] for x in SRV_BODY[:2] for y in SRV_BODY[2:4] for z in SRV_BODY[4:]])
_w = trimesh.transform_points(_c, M_jaw); jb = np.array([_w.min(0), _w.max(0)])     # servo body box (no horn)
TAB_TOP = trimesh.transform_points([[0, 0, TAB_Z[0]]], M_jaw)[0, 2]                 # 7.65
HSG_Y = (-21.5, 21.5)
FW = JX - 8.5                                                 # front wall outer face
house = extrude(rrect(FW, -39.8, HSG_Y[0], HSG_Y[1], 5.0), HEAD_Z0, 27.0)            # top plate on the plate tongue, 0.2 mm off the cover
house += extrude(rrect(-42.4, -39.8, -14.0, 14.0, 1.0), 26.0, 38.0)                  # lip up the cover front wall
house += box3(FW, FW + 2.2, HSG_Y[0] + 3, HSG_Y[1] - 3, 1.0, 22.5)                   # front wall
house += extrude(rrect(FW, FW + 6, HSG_Y[0], HSG_Y[1], 2.5), 1.0, 22.5) - box3(FW + 2.2, FW + 7, HSG_Y[0] + 3, HSG_Y[1] - 3, 0, 23)
house += box3(JX - 5.0, JX + 5.0, -21.5, jb[0, 1] - 1.6, TAB_TOP, 22.5)              # post over the -Y tab
house += box3(JX - 5.0, JX + 5.0, jb[1, 1] + 1.6, 12.8, TAB_TOP, 22.5)               # post over the +Y tab
house += box3(JX - 5.0, JX + 5.0, 12.8, 16.6, -6.6, 22.5)                             # bracket down beside the +Y tab
house += box3(JX - 8.0, JX + 8.0, 1.6, 16.6, -6.6, -3.6)                              # shelf: carries the right mandible's pivot
house -= box3(jb[0, 0] - 0.3, jb[1, 0] + 0.3, jb[0, 1] - 0.3, jb[1, 1] + 0.3, 1.3, 23.95) # servo body
house -= box3(jb[0, 0] - 0.3, jb[1, 0] + 0.3, -20.0, 12.6, 4.9, 7.9)                 # tab slot
for tx in TAB_X:                                                                      # tab screws (M2 from below into the posts)
    p = trimesh.transform_points([[tx, 0, 17.25]], M_jaw)[0]
    house -= cyl(p[0], p[1], 0.85, 4.0, 18.0, 16)
house -= cyl(JX, JYB, 2.0, -7.0, -3.0, 24)                                            # M3 heat-set insert, right mandible pivot
house -= cyl(JX, JYA, 7.5, -8, 1.3, 48)                                               # servo cap + horn clearance
house -= box3(-49.6, -37.0, -30, 30, 0, HEAD_Z0)                                      # keep the plate tongue / spine free
house -= box3(-48.0, -44.0, -4.0, 4.0, HEAD_Z0 - 1, 28)                               # servo lead into the body
for y in (-10.0, 10.0):                                                               # 2x M2 through the lip into the cover front wall
    house -= mf.Manifold.cylinder(6, 1.1, 1.1, 16).rotate([0, 90, 0]).translate([-44.0, y, 33.0])
house_tm = to_tm(house)

# ----------------------------------------------------------------------------- 3. mandibles with involute gear sectors
def involute_tooth(m, z, pa=20.0, bl=0.15):
    rp = m * z / 2; rb = rp * np.cos(np.radians(pa)); ra = rp + m; rf = rp - 1.25 * m
    inv = lambda t: np.array([rb * (np.cos(t) + t * np.sin(t)), rb * (np.sin(t) - t * np.cos(t))])
    ta = np.sqrt((ra / rb) ** 2 - 1); tp = np.sqrt((rp / rb) ** 2 - 1)
    ph_p = np.arctan2(*inv(tp)[::-1]); half = np.pi / (2 * z) - bl / (2 * rp)
    ph0 = -half - ph_p
    R = lambda a: np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    flank = [R(ph0) @ inv(t) for t in np.linspace(0, ta, 14)]
    right = [rf * np.array([np.cos(ph0), np.sin(ph0)])] + flank
    left = [p * [1, -1] for p in right][::-1]
    return Polygon(right + left), rf, ra

TOOTH, RF, RA = involute_tooth(GEAR_M, GEAR_Z)

def gear_sector(center_deg, ks, phase=0.0):
    g = [Point(0, 0).buffer(RF + 0.01, resolution=64)]
    for k in ks:
        g.append(affinity.rotate(TOOTH, center_deg + phase + k * 360.0 / GEAR_Z, origin=(0, 0)))
    return unary_union(g)

def arm2d(mirror=1):
    """left mandible arm, closed, in its pivot frame (+Y = toward the other mandible). mirror=-1 for the right one"""
    P = [(0, 0), (-12, -3.0), (-24, -3.5), (-33, 0.5), (-38.5, 7.2)]
    Wd = [10, 7.5, 6.5, 5.0, 2.6]
    segs = []
    for (p0, w0), (p1, w1) in zip(zip(P, Wd), zip(P[1:], Wd[1:])):
        segs.append(unary_union([Point(p0).buffer(w0 / 2, 24), Point(p1).buffer(w1 / 2, 24)]).convex_hull)
    a = unary_union(segs)
    # three blunt teeth on the inner (+Y) edge of the distal half
    for (x, y) in [(-22.5, -0.6), (-29.0, 1.6), (-33.6, 4.6)]:
        a = a.union(Point(x, y).buffer(1.5, 16))
    return affinity.scale(a, 1, mirror, origin=(0, 0))

KS = [-3, -2, -1, 0, 1]
gearA = gear_sector(90.0, KS)                                  # tooth centred toward the other gear
gearB = gear_sector(-90.0, [-k for k in KS], phase=180.0 / GEAR_Z)   # half a pitch out of phase
mandA2d = unary_union([gearA, arm2d(+1)]).difference(Point(0, 0).buffer(1.1, 16))     # 2.2 mm horn screw
mandB2d = unary_union([gearB, arm2d(-1)]).difference(Point(0, 0).buffer(1.65, 16))    # M3 pivot
mandA = extrude(mandA2d, MZ0, MZ1)
for hx_ in (-6.0, 6.0):                                                               # 2 x M2 into the servo horn arm
    mandA -= cyl(hx_, 0, 0.8, MZ0 - 1, MZ1 + 1, 12)
mandA -= cyl(0, 0, 2.6, MZ0 - 1, MZ0 + 2.0, 24)                                      # horn screw head recess (from below)
mandB = extrude(mandB2d, MZ0, MZ1) - cyl(0, 0, 3.0, MZ0 - 1, MZ0 + 1.8, 24)          # M3 head recess from below
mandA_tm = to_tm(mandA); mandB_tm = to_tm(mandB)

def mand_M(which, open_deg):
    """mandible placement in the robot frame; open_deg 0 = closed, + = open (A turns CCW, B CW)"""
    return trans([JX, JYA if which == 'A' else JYB, 0]) @ rotz(open_deg if which == 'A' else -open_deg)

# ----------------------------------------------------------------------------- 4. neck (turntable + tilt servo cradle + pivot cheek)
R_TILT = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float)   # servo x -> x, y -> -z, z (shaft) -> +y
tilt_srv = servo_at(trans([NX, TILT_Y0, TZ]) @ R_TILT)
tilt_horn = servo_at(trans([NX, TILT_Y0, TZ]) @ R_TILT, horn=True)
jaw_horn = servo_at(M_jaw, horn=True)
neck = extrude(unary_union([rrect(NX - 13.5, NX + 23.5, -18.0, 10.5, 5.0), Point(NX, 0).buffer(16.0, resolution=64)]), TT_Z0, TT_Z1)
for tx in TAB_X:                                                     # cradle posts behind the tabs (+Y side), M2 along Y into them
    neck += box3(NX + tx - 2.4, NX + tx + 2.4, TILT_Y0 + TAB_Z[1] + 0.1, TILT_Y0 + TAB_Z[1] + 4.6, TT_Z1 - 0.01, TZ + 3.6)
    neck -= xform(mf.Manifold.cylinder(8, 0.85, 0.85, 12).rotate([-90, 0, 0]).translate([NX + tx, TILT_Y0 + TAB_Z[1] - 1, TZ]), np.eye(4))
neck += box3(NX - 7.0, NX + 7.0, TILT_Y0 - 3.3, TILT_Y0 - 0.3, TT_Z1 - 0.01, TZ)                # pivot cheek, round top
neck += mf.Manifold.cylinder(3.0, 7.0, 7.0, 48).rotate([-90, 0, 0]).translate([NX, TILT_Y0 - 3.3, TZ])
neck += mf.Manifold.cylinder(4.2, 2.45, 2.45, 32).rotate([90, 0, 0]).translate([NX, TILT_Y0 - 3.3 + 0.01, TZ])  # 4.9 mm pivot pin
neck -= cyl(NX, 0, 1.25, TT_Z0 - 1, TT_Z1 + 1, 16)                    # pan horn centre screw
for a in (0, 180):                                                    # horn arm screws
    neck -= cyl(NX + 8 * np.cos(np.radians(a)), 8 * np.sin(np.radians(a)), 0.8, TT_Z0 - 1, TT_Z1 + 1, 12)
neck -= xform(box3(SRV_BODY[0] - 0.3, SRV_BODY[1] + 0.3, SRV_BODY[2] - 0.4, SRV_BODY[3] + 0.4, SRV_BODY[4] - 0.3, SRV_BODY[5] + 0.3), trans([NX, TILT_Y0, TZ]) @ R_TILT)
neck_tm = to_tm(neck)

# ----------------------------------------------------------------------------- 5. head: CoreS3 shell (v1 face pocket) + two arms down to the tilt axis
face = box3(-T, D + T, -W / 2 - T, W / 2 + T, -3.0, 0.0)
face += box3(-T, D + T, -W / 2 - T, -W / 2, 0, CORE_H + 2 * CLR)
face += box3(-T, D + T, W / 2, W / 2 + T, 0, CORE_H + 2 * CLR)
face += box3(-T, D + T, -W / 2 - T, W / 2 + T, CORE_H + 2 * CLR, CORE_H + 2 * CLR + 2.0)
for sb in (-1, 1):
    face += box3(-T, 0, min(sb * (W / 2 - 4.6), sb * W / 2), max(sb * (W / 2 - 4.6), sb * W / 2), 0, CORE_H + 2 * CLR)
    face += box3(D - 1.0, D, min(sb * (W / 2 - 6), sb * W / 2), max(sb * (W / 2 - 6), sb * W / 2), 4, CORE_H - 4)
face += box3(-T, 0, -W / 2, W / 2, CORE_H + 2 * CLR - 3.0, CORE_H + 2 * CLR)
for sb in (-1, 1):
    face += box3(-T, 0, min(sb * (W / 2 - 8), sb * W / 2), max(sb * (W / 2 - 8), sb * W / 2), 0, 4.0)
face -= box3(3.0, D - 3.0, -13, 13, -20, 1)                          # cable slot (CoreS3 power + UART)
for sb in (-1, 1):
    face -= box3(D + T - 1.2, D + T + 1, sb * (W / 2 + T) - 1.5 * sb - 1.5, sb * (W / 2 + T) - 1.5 * sb + 1.5, 20, 34)
ax_lx, ax_lz = NX - HX, TZ - (HB + 3.0)                               # tilt axis in head-local coords
for (y0, y1) in [(TILT_Y0 + HORN_TOP + 0.25, TILT_Y0 + HORN_TOP + 3.25), (TILT_Y0 - 6.6, TILT_Y0 - 3.6)]:
    arm = box3(ax_lx - 6.0, ax_lx + 6.0, y0, y1, ax_lz, -3.0 + 0.01)
    arm += mf.Manifold.cylinder(y1 - y0, 6.0, 6.0, 48).rotate([-90, 0, 0]).translate([ax_lx, y0, ax_lz])
    face += arm
face -= mf.Manifold.cylinder(4, 2.6, 2.6, 32).rotate([-90, 0, 0]).translate([ax_lx, TILT_Y0 - 7, ax_lz])      # pin bore 5.2
face -= mf.Manifold.cylinder(5, 1.1, 1.1, 16).rotate([-90, 0, 0]).translate([ax_lx, TILT_Y0 + HORN_TOP - 1, ax_lz])  # horn screw
for dxz in (-7.0, 7.0):
    face -= mf.Manifold.cylinder(5, 0.8, 0.8, 12).rotate([-90, 0, 0]).translate([ax_lx, TILT_Y0 + HORN_TOP - 1, ax_lz + dxz])
face -= box3(0, D, -W / 2, W / 2, 0, CORE_H + 2 * CLR)
M_head0 = trans([HX, 0, HB + 3.0])
head_tm = to_tm(xform(face, M_head0))
core_tm = to_tm(xform(box3(CLR, D - CLR, -CORE_W / 2, CORE_W / 2, 0.01, CORE_H), M_head0))
screen_tm = to_tm(xform(box3(-0.2, 0.3, -20.3, 20.3, 17.5, 48.0), M_head0))
camdot_tm = to_tm(xform(mf.Manifold.cylinder(0.5, 2.2, 2.2, 24).rotate([0, -90, 0]).translate([0.3, 0, 9.0]), M_head0))
CAM0 = np.array([HX - 0.2, 0.0, HB + 3.0 + 9.0])

def head_M(pan, tilt):
    """head transform: tilt about the Y axis through (NX, TZ) (+ = nod down), then pan about Z through NX"""
    return rotz(pan, (NX, 0)) @ rot(-tilt, [0, 1, 0], [NX, 0, TZ])

def neck_M(pan):
    return rotz(pan, (NX, 0))

# ----------------------------------------------------------------------------- 6. robot assembly with moving neck + mandibles
cone_tm = v2['cone']; cov2_tm = v2['cover']
pan_srv = servo_at(M_pan)
pan_horn = servo_at(M_pan, horn=True)
STATIC = [('v3_body_cover', cover_tm, '#f5a54a'), ('jaw_housing', house_tm, '#f28c28'), ('jaw_servo', jaw_srv, '#2b2d33'),
          ('pan_servo', pan_srv, '#2b2d33'), ('belly_cone', cone_tm, '#3a3f4a')]
bot = Robot(d, replace={'stackchan_head': None, 'cores3': None, 'cores3_screen': None, 'cores3_camera': None, 'bottom_cover': cov2_tm},
            extra=STATIC)
NECK = [('neck', neck_tm, '#3a3f4a'), ('tilt_servo', tilt_srv, '#2b2d33'), ('pan_horn', pan_horn, '#e8e8e8')]
HEAD = [('tilt_horn', tilt_horn, '#e8e8e8'), ('head', head_tm, '#f5a54a'), ('cores3', core_tm, '#2b2d33'), ('cores3_screen', screen_tm, '#1f6feb'), ('cores3_camera', camdot_tm, '#111111')]

def moving(T=np.eye(4), pan=0.0, tilt=0.0, jaw=0.0):
    out = []
    for n, m, c in NECK:
        mm = m.copy(); mm.apply_transform(T @ neck_M(pan)); out.append((n, mm, c))
    for n, m, c in HEAD:
        mm = m.copy(); mm.apply_transform(T @ head_M(pan, tilt)); out.append((n, mm, c))
    for n, m, c, w in [('jaw_horn', jaw_horn, '#e8e8e8', 'A'), ('mandible_left', mandA_tm, '#e9e2d0', 'A'), ('mandible_right', mandB_tm, '#e9e2d0', 'B')]:
        M = rotz(jaw, (JX, JYA)) if n == 'jaw_horn' else mand_M(w, jaw)
        mm = m.copy(); mm.apply_transform(T @ M); out.append((n, mm, c))
    return out

def full(T=np.eye(4), hips=None, knees=None, pan=0.0, tilt=0.0, jaw=0.0):
    return bot.pose(T, hips, knees) + moving(T, pan, tilt, jaw)

MASS = {'servo': 13.4, 'scs0009': 13.5, 'cores3': 73.0, 's2mini': 3.0, 'battery': 45.0}
def part_mass(n, m):
    if n.startswith(('hip_servo', 'knee_servo', 'jaw_servo')): return MASS['servo']
    if n in ('pan_servo', 'tilt_servo'): return MASS['scs0009']
    if n == 'cores3': return MASS['cores3']
    if n in ('s2mini', 'battery'): return MASS[n]
    if n in ('cores3_screen', 'cores3_camera'): return 0.0
    return m.volume / 1000 * 1.24 * 0.45

def com(parts):
    ms = np.array([part_mass(n, m) for n, m, c in parts]); cs = np.array([m.center_mass if m.is_watertight else m.bounds.mean(0) for n, m, c in parts])
    return (ms[:, None] * cs).sum(0) / ms.sum(), ms.sum()

def main():
    global rep
    # ----------------------------------------------------------------------------- 7. checks
    rep = []
    def P(*s):
        line = ' '.join(str(x) for x in s); print(line); rep.append(line)
    P('== SesAnt v3 (StackChan neck + ant mandibles) verification ==')
    ZERO = {k: 0.0 for k in SG}
    STAND = bot.stand_on_floor()
    parts0 = full(STAND)
    c0, mtot = com(parts0)
    allv = np.vstack([m.vertices for n, m, c in parts0])
    P(f'standing: height {allv[:, 2].max():.0f} mm, footprint {np.ptp(allv[:, 0]):.0f} x {np.ptp(allv[:, 1]):.0f} mm, mass ~{mtot:.0f} g (est.), '
      f'CoM z {c0[2]:.0f} mm, x {c0[0]:+.1f}')

    # (a) static fit
    cm_body = CollisionManager()
    for n, m, c in bot.pose():
        if n in ('v3_body_cover', 'chassis', 'jaw_housing', 'bottom_cover', 'belly_cone') or n.startswith('hip_servo'): cm_body.add_object(n, m)
    fit = {}
    for n, m in [('pan_servo', pan_srv), ('jaw_servo', jaw_srv), ('s2mini', A['s2mini'])]:
        hit, names = cm_body.in_collision_single(m, return_names=True)
        fit[n] = sorted(names) if hit else 'clear'
    P('static fit (servos/electronics vs cover, housing, chassis):', fit)
    cmc = CollisionManager(); cmc.add_object('chassis', A['chassis'])
    cmv = CollisionManager(); cmv.add_object('cover', cover_tm)
    P('jaw housing vs chassis:', 'touching the plate tongue (it sits on it)' if cmc.in_collision_single(house_tm) else 'no contact (0 mm gap)',
      '; vs body cover:', 'OVERLAP' if cmv.in_collision_single(house_tm) else 'clear (lip bolted to the cover front wall, 2x M2)')

    # (b) neck sweep: head + neck vs body (and legs standing)
    cm_all = CollisionManager()
    for n, m, c in bot.pose():
        if n not in ('pan_servo',): cm_all.add_object(n, m)
    for n, m, c in moving(jaw=0.0):
        if n.startswith('mandible'): cm_all.add_object(n, m)
    bad = []
    tilt_ok = {t: True for t in range(-30, 71, 5)}
    for pan in range(-180, 180, 15):
        for t in tilt_ok:
            mv = moving(pan=pan, tilt=t)
            for n, m, c in mv:
                if n.startswith(('mandible', 'jaw')): continue
                if cm_all.in_collision_single(m): tilt_ok[t] = False; bad.append((pan, t, n)); break
            H = {n: m for n, m, c in mv}
            cmh = CollisionManager(); cmh.add_object('neck', H['neck']); cmh.add_object('tilt_servo', H['tilt_servo'])
            if cmh.in_collision_single(H['head']):
                hit, nm = cmh.in_collision_single(H['head'], return_names=True)
                tilt_ok[t] = False; bad.append((pan, t, 'head-' + '-'.join(sorted(nm))))
    ok_t = [t for t, v in tilt_ok.items() if v]
    lo = 0
    while lo - 5 in ok_t: lo -= 5
    hi = 0
    while hi + 5 in ok_t: hi += 5
    P(f'neck sweep: pan all round (15 deg steps) x tilt: head, CoreS3, neck and tilt servo collision-free for tilt {lo}..+{hi} deg '
      f'(+ = look down); firmware limits {TILT_RANGE}, pan {PAN_RANGE} (cable loop)')
    assert lo <= TILT_RANGE[0] and hi >= TILT_RANGE[1], ('neck range', lo, hi, bad[:4])
    worst = min(support_margin(com(full(STAND, pan=p, tilt=t))[0], [bot.tip_pts(k, 0, 0, STAND).mean(0) for k in SG])
                for p in (-120, -60, 0, 60, 120) for t in TILT_RANGE)
    P(f'  standing stability over the neck range: worst CoM margin {worst:.0f} mm inside the 4-foot support polygon')

    # (c) gears + mandibles
    gA0 = affinity.translate(gearA, JX, JYA); gB0 = affinity.translate(gearB, JX, JYB)
    ov = []; mind = []
    for a in np.linspace(0, JAW_OPEN, 41):
        ga = affinity.rotate(gA0, a, origin=(JX, JYA)); gb = affinity.rotate(gB0, -a, origin=(JX, JYB))
        ov.append(ga.intersection(gb).area); mind.append(ga.distance(gb))
    P(f'gear pair m{GEAR_M} z{GEAR_Z} (pivots {2 * RP:.1f} mm apart): max overlap {max(ov):.3f} mm2 over 0..{JAW_OPEN:.0f} deg '
      f'(must be ~0), closest approach {min(mind):.2f} mm -> {"meshes without binding" if max(ov) < 0.05 else "INTERFERENCE"}')
    def tips(a):
        pa = affinity.rotate(affinity.translate(arm2d(+1), JX, JYA), a, origin=(JX, JYA))
        pb = affinity.rotate(affinity.translate(arm2d(-1), JX, JYB), -a, origin=(JX, JYB))
        return pa, pb
    pa, pb = tips(0); gap0 = pa.distance(pb)
    pa, pb = tips(JAW_OPEN)
    tipA = np.array(pa.exterior.coords)[np.argmin(np.array(pa.exterior.coords)[:, 0])]
    tipB = np.array(pb.exterior.coords)[np.argmin(np.array(pb.exterior.coords)[:, 0])]
    inner = pa.distance(pb)
    P(f'mandibles: closed gap {gap0:.1f} mm (holds a 2 mm wire / card edge); open {JAW_OPEN:.0f} deg each -> tips '
      f'{np.hypot(*(tipA - tipB)):.0f} mm apart, narrowest inside gap {inner:.0f} mm; reach x = {min(tipA[0], tipB[0]):.0f} mm (body front -39.6)')
    cmj = CollisionManager()
    for n, m in [('jaw_housing', house_tm), ('jaw_servo', jaw_srv), ('chassis', A['chassis']), ('belly_cone', cone_tm), ('bottom_cover', cov2_tm)]: cmj.add_object(n, m)
    jbad = []
    for a in np.linspace(0, JAW_OPEN, 9):
        for n, m, c in moving(jaw=a):
            if not n.startswith('mandible'): continue
            hit, nm = cmj.in_collision_single(m, return_names=True)
            nm = {x for x in nm if not (x == 'jaw_servo' and n == 'mandible_left')}     # left mandible is screwed to the horn
            if nm: jbad.append((round(a), n, sorted(nm)))
    P('mandible sweep vs housing, servo, chassis, belly cone:', 'collision-free' if not jbad else jbad[:4])
    arm_len = 38.5
    P(f'grip: MG90S ~2.2 kg*cm on the left mandible, gears share it -> ~{2.2 * 9.81 / 2 / (arm_len / 10):.1f} N per tip at the tips; '
      f'friction mu 0.4 -> holds objects up to ~{2 * 0.4 * 2.2 * 9.81 / 2 / (arm_len / 10) / 9.81 * 1000:.0f} g')

    # (d) front hips vs the jaw (housing + open mandibles)
    cmf = CollisionManager()
    for n, m in [('jaw_housing', house_tm), ('jaw_servo', jaw_srv)]: cmf.add_object(n, m)
    for n, m, c in moving(jaw=JAW_OPEN):
        if n.startswith('mandible'): cmf.add_object(n, m)
    hip_rng = {}
    for k in ('FL', 'FR'):
        legm = [m for n, m, c, l, kd in bot.parts if l == k]
        ok = []
        for a in range(-90, 91, 3):
            if not any(cmf.in_collision_single(m, transform=bot.M_hip(k, a)) for m in legm): ok.append(a)
        lo = 0
        while lo - 3 in ok: lo -= 3
        hi = 0
        while hi + 3 in ok: hi += 3
        hip_rng[k] = (lo, hi)
    P('front hips with the jaw fitted (mandibles open): collision-free hip range', hip_rng,
      '(v1: -90..+90; the restricted side is the forward swing toward the snout)')

    # (e) grab pose: crouch nose-down until the open mandibles are 2 mm above the floor
    def mand_low(T):
        return min(m.bounds[0, 2] for n, m, c in moving(T, jaw=JAW_OPEN) if n.startswith('mandible'))
    best = None
    for pitch in np.arange(0, 16.1, 1.0):
        Tr = rot(-pitch, [0, 1, 0])
        z_m = mand_low(Tr)
        dz = 2.0 - z_m
        T = trans([0, 0, dz]) @ Tr
        kn = {k: bot.knee_for(k, 0.0, T) for k in SG}
        if any(v is None or v < -110 for v in kn.values()): continue
        body_low = min(m.bounds[0, 2] for n, m, c in bot.pose(T, knees=kn) if n in ('belly_cone', 'bottom_cover', 'chassis', 'jaw_housing') or n.startswith(('knee_servo',)))
        if body_low < 1.5: continue
        best = (pitch, T, kn, body_low); break
    assert best, 'no grab pose found'
    G_PITCH, G_T, G_KN, G_LOW = best
    # head tilt that centres the camera on the mandible tips
    mtips = np.vstack([np.array(m.vertices)[np.argmin(m.vertices[:, 0])] for n, m, c in moving(G_T, jaw=JAW_OPEN) if n.startswith('mandible')]).mean(0)
    def cam_err(t):
        Mh = G_T @ head_M(0, t); cam = trimesh.transform_points([CAM0], Mh)[0]
        fwd = (Mh[:3, :3] @ [-1, 0, 0]); v = mtips - cam
        return np.degrees(np.arctan2(np.cross(fwd, v)[1], np.dot(fwd, v))), cam
    ts = np.arange(TILT_RANGE[0], TILT_RANGE[1] + 0.1, 0.5)
    G_TILT = float(ts[np.argmin([abs(cam_err(t)[0]) for t in ts])])
    e_, camw = cam_err(G_TILT)
    grab_parts = full(G_T, knees=G_KN, tilt=G_TILT, jaw=JAW_OPEN)
    gm = support_margin(com(grab_parts)[0], [bot.tip_pts(k, 0, G_KN[k], G_T).mean(0) for k in SG])
    P(f'grab pose: body pitched {G_PITCH:.0f} deg nose-down, knees front {-G_KN["FL"]:.0f} / back {-G_KN["BL"]:.0f} deg outward '
      f'(RealAnt {knee_units(G_KN["FL"])} / {knee_units(G_KN["BL"])}); open mandibles 2 mm above the floor, '
      f'lowest other part {G_LOW:.1f} mm; CoM margin {gm:.0f} mm')
    P(f'  head tilt {G_TILT:.1f} deg down puts the mandible tips on the camera axis ({np.linalg.norm(mtips - camw):.0f} mm away; residual {e_:.1f} deg)')
    cmg = CollisionManager(); cmg.add_object('floor', trimesh.creation.box([600, 600, 2], transform=trans([0, 0, -1.0])))
    P('  grab pose floor contacts (0.3 mm lower):', sorted({n for n, m, c in full(trans([0, 0, -0.3]) @ G_T, knees=G_KN, tilt=G_TILT, jaw=JAW_OPEN) if cmg.in_collision_single(m)}),
      '(expected: the 4 blade feet only)')

    # (f) paw: stand on FR, BR, BL; lift FL forward
    PAW_KNEE = -95.0
    cands = []
    for hfr in range(0, 46, 5):
        for hbl in range(0, -46, -5):
            if not (hip_rng['FR'][0] <= hfr <= hip_rng['FR'][1]): continue
            hips = {'FL': -20.0, 'FR': float(hfr), 'BR': 0.0, 'BL': float(hbl)}
            knees = {'FL': PAW_KNEE, 'FR': 0.0, 'BR': 0.0, 'BL': 0.0}
            T = bot.stand_on_floor(hips={**hips, 'FL': 0.0}, knees={**knees, 'FL': 0.0})
            mg = support_margin(com(full(T, hips, knees, tilt=20))[0], [bot.tip_pts(k, hips[k], knees[k], T).mean(0) for k in ('FR', 'BR', 'BL')])
            cands.append((mg, abs(hfr) + abs(hbl), hips, knees, T))
    good = [c for c in cands if c[0] >= 15.0]
    c = min(good, key=lambda c: c[1]) if good else max(cands, key=lambda c: c[0])
    pbest = (c[0], c[2], c[3], c[4])
    PAW_M, PAW_H, PAW_K, PAW_T = pbest
    ftip = bot.tip_pts('FL', PAW_H['FL'], PAW_K['FL'], PAW_T).mean(0)
    cm_paw = CollisionManager()
    for n, m, c in full(PAW_T, PAW_H, PAW_K, tilt=20):
        if not (n.startswith(('blade_foot_L3', 'femur_L1', 'knee_servo_FL', 'hip_servo_FL'))): cm_paw.add_object(n, m)   # own hip servo carries the femur
    paw_hit = sorted({n for n, m, c in full(PAW_T, PAW_H, PAW_K, tilt=20) if n.startswith(('blade_foot_L3', 'femur_L1', 'knee_servo_FL')) and cm_paw.in_collision_single(m)})
    P(f'paw pose: support hips FR {PAW_H["FR"]:+.0f}, BL {PAW_H["BL"]:+.0f} deg; CoM {PAW_M:.0f} mm inside the 3-foot triangle; '
      f'FL hip {PAW_H["FL"]:+.0f}, knee {-PAW_K["FL"]:.0f} deg out lifts the FL tip to z {ftip[2]:.0f} mm at x {ftip[0]:.0f}; lifted leg',
      'clear of the body' if not paw_hit else paw_hit)

    # (g) still docks on the v2 puck
    pk = v2['puck']; fl = v2['flag']; SEAT_DZ = v2['SEAT_DZ']
    cmp_ = CollisionManager(); cmp_.add_object('puck', pk); cmp_.add_object('flag', fl)
    seat_kn = {k: bot.knee_for(k, 0.0, trans([0, 0, SEAT_DZ])) for k in SG}
    seat_parts = full(trans([0, 0, SEAT_DZ]), knees=seat_kn, tilt=0)
    P('on the v2 home puck: seated, pushed 0.3 mm down: contacts =',
      sorted({n for n, m, c in full(trans([0, 0, SEAT_DZ - 0.3]), knees=seat_kn) if cmp_.in_collision_single(m)}), '(expected: belly_cone only)')
    tagc = np.array([v2['params']['TAG_X'] + 4.4, 0, v2['params']['ZC']])
    def tilt_to(T, target):
        best_ = None
        for t in np.arange(TILT_RANGE[0], TILT_RANGE[1] + 0.1, 0.5):
            Mh = T @ head_M(0, t); cam = trimesh.transform_points([CAM0], Mh)[0]; fwd = Mh[:3, :3] @ [-1, 0, 0]; v = target - cam
            e = np.degrees(np.arctan2(np.cross(fwd, v)[1], np.dot(fwd, v)))
            if best_ is None or abs(e) < abs(best_[1]): best_ = (t, e, np.linalg.norm(v))
        return best_
    ts_, es_, ds_ = tilt_to(trans([0, 0, SEAT_DZ]), tagc); tw_, ew_, dw_ = tilt_to(STAND, tagc)
    P(f'  tag on the flag: head tilt {tw_:+.1f} deg standing ({dw_:.0f} mm), {ts_:+.1f} deg seated ({ds_:.0f} mm) centres it '
      f'(the neck replaces the v2 compromise tag height)')

    # ----------------------------------------------------------------------------- 8. export
    info = {}
    info['sesant_v3_body_cover'] = save_stl(cover_tm, OUT + '/stl/sesant_v3_body_cover.stl')
    info['sesant_v3_jaw_housing'] = save_stl(house_tm, OUT + '/stl/sesant_v3_jaw_housing.stl', rot(180, [1, 0, 0]))
    info['sesant_v3_neck'] = save_stl(neck_tm, OUT + '/stl/sesant_v3_neck.stl')
    info['sesant_v3_head'] = save_stl(head_tm, OUT + '/stl/sesant_v3_head.stl', rot(180, [1, 0, 0]))
    info['sesant_v3_mandible_left'] = save_stl(mandA_tm, OUT + '/stl/sesant_v3_mandible_left.stl')
    info['sesant_v3_mandible_right'] = save_stl(mandB_tm, OUT + '/stl/sesant_v3_mandible_right.stl')
    for n, m in info.items():
        e = m.extents; P(f'{n:26s} {e[0]:6.1f} x {e[1]:6.1f} x {e[2]:5.1f} mm  {m.volume / 1000:6.1f} cm3  ~{m.volume / 1000 * 1.24 * 0.45:4.0f} g')
    open(OUT + '/assembly/v3_verification_report.txt', 'w').write('\n'.join(rep) + '\n')

    sc = trimesh.Scene()
    for n, m, c in parts0: sc.add_geometry(color_mesh(m, c), node_name=n, geom_name=n)
    sc.export(OUT + '/assembly/sesant_v3.glb')
    json.dump({'neck': {'pan_axis_xy': [NX, 0], 'tilt_axis_xz': [NX, round(TZ, 2)], 'tilt_range_deg': TILT_RANGE, 'pan_range_deg': PAN_RANGE,
                        'scs_ids': {'pan': 1, 'tilt': 2}, 'deg_per_step': 0.3125},
               'jaw': {'pivots_xy': [[JX, JYA], [JX, JYB]], 'open_deg': JAW_OPEN, 'gear': f'm{GEAR_M} z{GEAR_Z} 20deg'},
               'grab_pose': {'pitch_deg': G_PITCH, 'knee_front_units': knee_units(G_KN['FL']), 'knee_back_units': knee_units(G_KN['BL']),
                             'head_tilt_deg': G_TILT},
               'paw_pose': {'hips_cad_deg': PAW_H, 'fl_knee_units': knee_units(PAW_K['FL'])},
               'mass_g_est': round(mtot)}, open(OUT + '/assembly/sesant_v3.json', 'w'), indent=1)
    pickle.dump({'stand': parts0, 'grab': grab_parts, 'paw': full(PAW_T, PAW_H, PAW_K, tilt=20), 'seat': seat_parts,
                 'puck_items': v2['puck_items'], 'look': full(STAND, pan=40, tilt=25),
                 'parts': {'cover': cover_tm, 'house': house_tm, 'neck': neck_tm, 'head': head_tm, 'mandA': mandA_tm, 'mandB': mandB_tm},
                 'jaw_open': moving(jaw=JAW_OPEN), 'jaw_closed': moving(jaw=0.0), 'static': [(n, m, c) for n, m, c in STATIC]},
                open(WORK + '/v3.pkl', 'wb'))


if __name__ == '__main__':
    main()
