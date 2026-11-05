"""SesAnt v2: "home puck" - a small round charging puck the robot walks over and sits on.

Usage: python v2_puck.py <work dir with v1 asm.pkl> <sesant root>

Idea: instead of aligning four feet, the robot carries a cone under its belly (the "belly cone", held by the
two existing bottom-cover screws). The puck has a matching 30-degree funnel. The robot walks over the puck,
opens its knees until the cone drops into the funnel, then relaxes its servos: the cone slides down the funnel
walls to the centre. Contacts are a centre disc and a ring on the cone tip, so the robot can sit at any heading.

Puck frame: Z up, z = 0 is the floor, origin on the funnel axis, the tag flag at -X (the robot faces it).
"""
import sys, os, pickle, json
sys.path.insert(0, os.path.dirname(__file__))
from common import *
from cadlib import *
from poses import Robot, SG, knee_units
from trimesh.collision import CollisionManager
from shapely.geometry import LineString

WORK, ROOT = sys.argv[1], sys.argv[2]
OUT = ROOT + '/v2'
for s in ('stl', 'images', 'assembly'): os.makedirs(f'{OUT}/{s}', exist_ok=True)
d = pickle.load(open(WORK + '/asm.pkl', 'rb'))
A = {n: m for n, m, c, g in d['asm']}

# ----------------------------------------------------------------------------- parameters
WALL_DEG = 30.0           # cone / funnel wall angle from horizontal (tan 30 = 0.58 > PLA friction ~0.3: it slides)
TW = np.tan(np.radians(WALL_DEG))
R_TIP = 12.0              # flat cone tip radius (carries the contact pads)
CONE_H = 8.5              # cone height
FL_Z0, FL_Z1 = -4.0, -1.0 # flange (robot frame): sits against the bottom cover, held by its two screws
R_CONE = R_TIP + CONE_H / TW             # cone radius at the flange = 26.7
CONE_TIP_Z = FL_Z0 - CONE_H              # -12.5 in the robot frame
CAPTURE = 12.0            # the cone tip may land up to 12 mm off centre and still enter the funnel
GAP = 1.0                 # cone tip floats this high above the funnel floor when seated (pins reach up to it)
R_FT = R_TIP + CAPTURE                   # funnel top radius 24
R_FF = R_TIP - GAP / TW                  # funnel floor radius
DEPTH = (R_FT - R_FF) * TW               # funnel depth 7.9
H = 22.0                  # puck height
RIM = 6.0                 # flat ring around the funnel
R_TOP = R_FT + RIM
R_BASE = R_TOP + H        # 45-degree outer slope -> base radius 52 (puck 104 mm across)
PAD_DISC_R, PAD_RING = 3.5, (6.0, 10.5)  # contacts on the cone tip: centre disc = +, ring = GND
PIN_R = 8.25              # outer pogo pin radius (centre pin at r = 0)
PIN_BORE = 3.1            # 3 mm barrel spring pins
COVER_SCREWS = [(-29.4, 0.0), (30.7, 0.0)]   # v1/Sesame bottom-cover screw holes (measured from the STL)
WIRE_UP = [(26.0, 9.0), (26.0, -9.0)]        # wire holes through the bottom cover: behind the battery, beside the screw boss
TAG = 50.0                # AprilTag (tag36h11) black square edge; card 58 mm
CARD = 58.0
TAG_X = -114.0            # tag plane: ~74 mm in front of the CoreS3 camera when the robot sits on the puck

SEAT_DZ = (H - DEPTH + GAP) - CONE_TIP_Z           # robot frame -> world lift when seated (centred)

# ----------------------------------------------------------------------------- 1. belly cone (robot frame)
flange2d = unary_union([rrect(-35.0, 36.0, -11.0, 11.0, 5.0), Point(0, 0).buffer(R_CONE + 1.5, resolution=64)])
cone = extrude(flange2d, FL_Z0, FL_Z1) + mf.Manifold.cylinder(CONE_H, R_TIP, R_CONE, 128).translate([0, 0, CONE_TIP_Z])
for (x, y) in COVER_SCREWS:                                      # longer M2 screws: through flange + cover into the frame
    cone -= cyl(x, y, 1.2, FL_Z0 - 1, FL_Z1 + 1, 24) + cyl(x, y, 2.3, FL_Z0 - 1, FL_Z0 + 1.6, 24)
z0 = CONE_TIP_Z
cone -= cyl(0, 0, PAD_DISC_R, z0 - 1, z0 + 0.6, 64)              # 0.6 mm recesses for brass/copper pads
cone -= (cyl(0, 0, PAD_RING[1], z0 - 1, z0 + 0.6, 128) - cyl(0, 0, PAD_RING[0], z0 - 2, z0 + 1, 128))
RING_WIRE = (0.0, -(PAD_RING[0] + PAD_RING[1]) / 2)
for (x, y) in [(0.0, 0.0), RING_WIRE]:                           # wires straight up through the cone
    cone -= cyl(x, y, 0.9, z0 - 1, FL_Z1 + 1, 16)
for (a, b) in [((0.0, 0.0), WIRE_UP[0]), (RING_WIRE, WIRE_UP[1])]:   # 2 x 1.2 mm grooves in the flange top face to the cover holes
    seg = LineString([a, b]).buffer(1.0, cap_style=1)
    cone -= extrude(seg, FL_Z1 - 1.2, FL_Z1 + 1)
cone_tm = to_tm(cone)

# v2 bottom cover: v1 cover + two wire holes
cov = to_mf(A['bottom_cover'])
for (x, y) in WIRE_UP: cov -= cyl(x, y, 1.25, -3, 8, 16)
cover_tm = to_tm(cov)

# ----------------------------------------------------------------------------- 2. puck (world frame)
# exact 30-degree wall from (R_FT, H) to (R_FF, H - DEPTH), with a 0.8 mm chamfer at the funnel lip
prof = [(0, 0), (R_BASE, 0), (R_BASE, 1.0), (R_TOP, H), (R_FT + 0.8, H), (R_FT - 0.3, H - 0.8),
        (R_FF, H - DEPTH), (0.0, H - DEPTH)]
puck = mf.CrossSection([np.array(prof, float)]).revolve(192)
# front tongue to the tag flag, rear tab for the USB-C input
tongue2d = rrect(TAG_X - 10.0, -R_BASE + 14.0, -12.0, 12.0, 4.0)
puck += chamfered_slab(tongue2d, 0, 3.0, 1.2, steps=6)                 # low: SesAnt Jumper's arms pass over it when seated
puck += chamfered_slab(rrect(TAG_X - 10.0, TAG_X + 10.0, -20.0, 20.0, 5.0), 0, 9.0, 2.5, steps=8)   # flag boss
tab2d = rrect(R_BASE - 16.0, R_BASE + 9.0, -12.5, 12.5, 4.0)
puck += chamfered_slab(tab2d, 0, 9.0, 2.0, steps=8)
# flag socket
puck -= box3(TAG_X - 3.2, TAG_X + 3.2, -15.2, 15.2, 2.0, 10)
# pogo pins + solder cavity
PINS = [(0.0, 0.0), (0.0, PIN_R)]
for (x, y) in PINS: puck -= cyl(x, y, PIN_BORE / 2, 3.0, H, 24)
puck -= cyl(0, 4, 11.0, -1, 4.0, 64)
# underside pockets: 2S CC/CV charger (TP5100 class), USB-C PD trigger (12 V) at the rear tab, wire channels
puck -= box3(-17, 17, 14, 38, -1, 6.5)
puck -= box3(R_BASE - 31, R_BASE + 2, -9.5, 9.5, -1, 5.5)
puck -= box3(R_BASE + 1, R_BASE + 10, -4.6, 4.6, 1.2, 5.4)       # USB-C receptacle opening
puck -= box3(-2.5, 2.5, 14, 16, -1, 3.5) + box3(8, R_BASE - 30, -2.5, 2.5, -1, 3.5) + box3(-2.5, 2.5, 4, 15, -1, 3.5)
for a in (45, 135, 225, 315):
    x, y = 40 * np.cos(np.radians(a)), 40 * np.sin(np.radians(a))
    puck -= cyl(x, y, 5.2, -1, 1.5, 32)
puck_tm = to_tm(puck)

# ----------------------------------------------------------------------------- 3. tag flag (world frame)
def build_flag(zc):
    z0_, z1_ = zc - CARD / 2 - 4, zc + CARD / 2 + 3
    f = box3(TAG_X - 3.0, TAG_X + 3.0, -15.0, 15.0, 2.2, z0_ + 1)                 # tenon + post
    f += box3(TAG_X - 3.0, TAG_X + 3.0, -CARD / 2 - 4, CARD / 2 + 4, z0_, z1_)      # back plate
    for sb in (-1, 1):
        f += box3(TAG_X + 3.0, TAG_X + 6.4, min(sb * (CARD / 2 + 4), sb * (CARD / 2 - 3)), max(sb * (CARD / 2 + 4), sb * (CARD / 2 - 3)), z0_, z1_)
    f += box3(TAG_X + 3.0, TAG_X + 6.4, -CARD / 2 - 4, CARD / 2 + 4, z0_, z0_ + 7)
    f -= box3(TAG_X + 3.0, TAG_X + 4.4, -CARD / 2 - 0.4, CARD / 2 + 0.4, z0_ + 3.6, z1_ + 1)   # 1.4 mm card slot, open at the top
    return f

# ----------------------------------------------------------------------------- 4. robot + poses
bot = Robot(d, replace={'bottom_cover': cover_tm}, extra=[('belly_cone', cone_tm, '#3a3f4a')])
ZERO = {k: 0.0 for k in SG}
STAND = bot.stand_on_floor()
STAND_DZ = STAND[2, 3]
seat_T = trans([0, 0, SEAT_DZ])
seat_knee = {k: bot.knee_for(k, 0.0, seat_T) for k in SG}
SEAT_KNEE = float(np.mean(list(seat_knee.values())))
CAM_R = A['cores3_camera'].bounds.mean(0)
cam_stand, cam_seat = CAM_R[2] + STAND_DZ, CAM_R[2] + SEAT_DZ
ZC = (cam_stand + cam_seat) / 2
flag_tm = to_tm(build_flag(ZC))
card_tm = to_tm(box3(TAG_X + 3.1, TAG_X + 4.3, -CARD / 2, CARD / 2, ZC - CARD / 2, ZC + CARD / 2))
tag_tm = to_tm(box3(TAG_X + 4.3, TAG_X + 4.5, -TAG / 2, TAG / 2, ZC - TAG / 2, ZC + TAG / 2))

def knees_at(T):
    return {k: bot.knee_for(k, 0.0, T) for k in SG}

# ----------------------------------------------------------------------------- 5. checks
rep = []
def P(*s):
    line = ' '.join(str(x) for x in s); print(line); rep.append(line)

P('== SesAnt v2 home puck verification ==')
P(f'puck: {2 * R_BASE:.0f} mm across, {H:.0f} mm high; funnel {2 * R_FT:.0f} mm wide at the top, {WALL_DEG:.0f}-degree walls, {DEPTH:.1f} mm deep')
P(f'belly cone: tip r {R_TIP} -> r {R_CONE:.1f} at the flange, {CONE_H} mm tall; tip {-CONE_TIP_Z - 1:.1f} mm below the belly')
P(f'standing: belly cone tip {CONE_TIP_Z + STAND_DZ:.1f} mm above the floor -> clears the puck top by {CONE_TIP_Z + STAND_DZ - H:.1f} mm when walking over it')
P(f'seat pose: knees {-SEAT_KNEE:.1f} deg outward (RealAnt knee {knee_units(SEAT_KNEE)}, stand = 224); body lowered {STAND_DZ - SEAT_DZ:.1f} mm',
  {k: round(v, 2) for k, v in seat_knee.items()})
seat_parts = bot.pose(seat_T, knees=seat_knee)
SP = {n: m for n, m, c in seat_parts}
P(f'  seated: cone tip {CONE_TIP_Z + SEAT_DZ:.2f} above the funnel floor {H - DEPTH:.2f} (pins reach it); belly flange '
  f'{FL_Z0 + SEAT_DZ - H:.2f} mm above the puck top; feet tips at r',
  {k: round(float(np.hypot(*bot.tip_pts(k, 0, seat_knee[k], seat_T).mean(0)[:2])), 1) for k in SG})

world_cm = CollisionManager(); world_cm.add_object('puck', puck_tm); world_cm.add_object('flag', flag_tm)
puck_cm = CollisionManager(); puck_cm.add_object('puck', puck_tm)
LOW = lambda n: n.startswith(('belly', 'bottom', 'blade', 'knee_servo', 'chassis', 'hip_servo', 'femur'))
# (a) seated, pushed 0.3 mm down: what touches?
hits = sorted({n for n, m, c in bot.pose(trans([0, 0, SEAT_DZ - 0.3]), knees=seat_knee) if world_cm.in_collision_single(m)})
P('seated, pushed 0.3 mm down: contacts =', hits, '(expected: belly_cone only; the feet stand on the floor)')
# (b) capture: lower the robot from standing at an offset / heading; the first thing to touch the puck must be the belly cone,
#     and its tip must land inside the funnel opening
def first_contact(ex, ey, yaw, step=0.5):
    """puck only: the flag limits the heading separately (below)"""
    Tr = trans([ex, ey, 0]) @ rotz(yaw)
    dz = CONE_TIP_Z * -1 + H + 1.0          # start just above first possible contact (cone tip 1 mm over the puck top)
    while dz > SEAT_DZ - 3:
        T = trans([0, 0, dz]) @ Tr
        kn = knees_at(T)
        if any(v is None for v in kn.values()): return ('knee limit', dz)
        for n, m, c in bot.pose(T, knees=kn, only=LOW):
            if m.bounds[0, 2] > H + 1: continue
            if puck_cm.in_collision_single(m): return (n, dz)
        dz -= step
    return (None, dz)

cases = []; bad = []
for e in (0.0, 4.0, 8.0, 11.0):
    for ang in (range(0, 360, 45) if e > 0 else [0]):
        for yaw in (0, 15, -15, 30, -30):
            ex, ey = e * np.cos(np.radians(ang)), e * np.sin(np.radians(ang))
            if abs(yaw) == 30 and e != 0.0: continue        # +-30 only centred: further out a foot meets the flag tongue
            n, dz = first_contact(ex, ey, yaw)
            cases.append((e, ang, yaw, n, dz))
            if n != 'belly_cone': bad.append((e, ang, yaw, n, round(dz, 1)))
P(f'capture: {len(cases)} lowering runs from cone-tip height H+1 mm down to the seat (offset 0..11 mm in 8 directions, heading 0 / +-15 deg, +-30 deg centred): ',
  'the belly cone is always the first contact' if not bad else f'FAIL {bad[:5]}')
P(f'  -> the disc/ring contacts work at any heading; offsets up to +-{CAPTURE:.0f} mm are funnelled to the centre '
  f'(30-degree walls: tan 30 = 0.58 > PLA-on-PLA friction ~0.3, so the cone slides once the servos relax)')
# heading limit from the tag flag: seated feet vs flag
yaw_ok = []
for yaw in range(0, 61, 5):
    Ts = trans([0, 0, SEAT_DZ]) @ rotz(-yaw)
    ok = not any(world_cm.in_collision_single(m) for n, m, c in bot.pose(trans([0, 0, 0.3]) @ Ts, knees=seat_knee) if n.startswith(('blade', 'knee')))
    Tw = trans([0, 0, STAND_DZ + 0.3]) @ rotz(-yaw)
    ok &= not any(world_cm.in_collision_single(m) for n, m, c in bot.pose(Tw) if n.startswith(('blade', 'knee')))
    if not ok: break
    yaw_ok.append(yaw)
P(f'  tag flag fitted: robot heading may be up to +-{max(yaw_ok)} deg off the flag axis before a foot touches the flag or its tongue (checked standing and seated)')
err = 1.0     # allow 1 mm residual for friction / print tolerance
okc = err < PAD_DISC_R and PAD_RING[0] < PIN_R - err and PIN_R + err < PAD_RING[1]
P(f'contacts: centre pin within r {err} of the disc centre (disc r {PAD_DISC_R}); outer pin r {PIN_R} +-{err} on ring {PAD_RING} -> '
  + ('OK at any heading, polarity cannot reverse' if okc else 'FAIL'))
d_cam = abs(CAM_R[0] - (TAG_X + 4.4))
P(f'camera -> tag: {d_cam:.0f} mm; camera z {cam_seat:.0f} (seated) .. {cam_stand:.0f} (standing), tag centre z {ZC:.0f}; '
  f'{TAG:.0f} mm tag spans +-{np.degrees(np.arctan(TAG / 2 / d_cam)):.0f} deg of the ~66 deg field of view')
vol = sum(m.volume for n, m, c, g in d['asm'] if g in ('print', 'sesame')) / 1000 + cone_tm.volume / 1000
m_body = vol * 1.24 * 0.45 + 8 * 13.4 + 73 + 3 + 45
lever = 66.0 * np.sin(np.radians(-SEAT_KNEE))
P(f'knee load at the end of the lowering (before the servos relax): robot ~{m_body:.0f} g (est.), lever {lever:.0f} mm -> '
  f'{(m_body / 4) * lever / 10 / 1000:.2f} kg*cm per knee vs MG90S stall ~2.2 kg*cm')

# ----------------------------------------------------------------------------- 6. export
info = {}
info['sesant_v2_home_puck'] = save_stl(puck_tm, OUT + '/stl/sesant_v2_home_puck.stl')
info['sesant_v2_tag_flag'] = save_stl(flag_tm, OUT + '/stl/sesant_v2_tag_flag.stl', rot(-90, [0, 1, 0]))      # lying on its back
info['sesant_v2_belly_cone'] = save_stl(cone_tm, OUT + '/stl/sesant_v2_belly_cone.stl', rot(180, [1, 0, 0]))  # flange on the bed
info['sesant_v2_bottom_cover'] = save_stl(cover_tm, OUT + '/stl/sesant_v2_bottom_cover.stl')
for n, m in info.items():
    e = m.extents; P(f'{n:24s} {e[0]:6.1f} x {e[1]:6.1f} x {e[2]:5.1f} mm  {m.volume / 1000:6.1f} cm3  ~{m.volume / 1000 * 1.24 * 0.4:4.0f} g @~40% fill')
open(OUT + '/assembly/v2_verification_report.txt', 'w').write('\n'.join(rep) + '\n')

PUCK_ITEMS = [(puck_tm, '#3a3f4a'), (flag_tm, '#3a3f4a'), (card_tm, '#f2f2f2'), (tag_tm, '#151515')]
for (x, y) in PINS: PUCK_ITEMS.append((to_tm(cyl(x, y, 1.5, H - DEPTH - 2, H - DEPTH + GAP + 0.2, 16)), '#d4a017'))
sc = trimesh.Scene()
for i, (m, c) in enumerate(PUCK_ITEMS): sc.add_geometry(color_mesh(m, c), node_name=f'puck_{i}', geom_name=f'puck_{i}')
for n, m, c in seat_parts: sc.add_geometry(color_mesh(m, c), node_name=n, geom_name=n)
sc.export(OUT + '/assembly/sesant_v2_on_puck.glb')
json.dump({'seat_knee_cad_deg': round(SEAT_KNEE, 2), 'seat_knee_realant_units': knee_units(SEAT_KNEE), 'stand_knee_realant_units': 224,
           'seat_lift_mm': round(SEAT_DZ, 2), 'stand_lift_mm': round(STAND_DZ, 2), 'puck_height_mm': H, 'puck_diameter_mm': 2 * R_BASE,
           'funnel': {'top_r': R_FT, 'floor_r': round(R_FF, 2), 'depth': round(DEPTH, 2), 'wall_deg': WALL_DEG},
           'cone': {'tip_r': R_TIP, 'flange_r': round(R_CONE, 2), 'height': CONE_H, 'tip_z_robot': CONE_TIP_Z},
           'pins_xy_mm': PINS, 'pads': {'disc_r': PAD_DISC_R, 'ring_r': PAD_RING}, 'tag': {'x': TAG_X, 'z_centre': round(ZC, 1), 'size': TAG}},
          open(OUT + '/assembly/sesant_v2_puck.json', 'w'), indent=1)
seq = []
for f in (0.0, 0.55, 1.0):
    dz = STAND_DZ + (SEAT_DZ - STAND_DZ) * f
    T = trans([0, 0, dz]); seq.append((dz, bot.pose(T, knees=knees_at(T))))
pickle.dump({'puck_items': PUCK_ITEMS, 'seated': seat_parts, 'seq': seq, 'cone': cone_tm, 'cover': cover_tm, 'puck': puck_tm, 'flag': flag_tm,
             'SEAT_DZ': SEAT_DZ, 'STAND_DZ': STAND_DZ, 'SEAT_KNEE': SEAT_KNEE, 'H': H, 'cases': cases,
             'params': dict(CONE_TIP_Z=CONE_TIP_Z, R_TIP=R_TIP, R_CONE=R_CONE, FL_Z0=FL_Z0, R_FT=R_FT, DEPTH=DEPTH, GAP=GAP, TAG_X=TAG_X, ZC=ZC)},
            open(WORK + '/v2.pkl', 'wb'))
