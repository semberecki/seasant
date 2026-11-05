"""SesAnt v4 (2/2): MuJoCo model of SesAnt (v3 neck + v4 Jumper arms with two-jaw claws) for Jumper's RL toolkit (mjlab / rsl_rl).

Usage: python v4_sim.py <work dir with v4.pkl> <sesant root>

Writes v4/sim/:
  sesant.xml       robot only, mjlab entity format (free base, IMU sensors, foot sites, no actuators: mjlab adds them)
  scene.xml        sesant.xml + floor + position actuators + keyframes, for plain MuJoCo
  meshes/          visual meshes (one per part) and convex collision hulls (one or more per body), in mm, scaled 0.001
and checks the model in MuJoCo: mass, standing, reaching the floor with both claws, neck + arm sweeps, a drop.
Robot frame as in the CAD (front at -X); MuJoCo units are metres.
"""
import sys, os, pickle, json
sys.path.insert(0, os.path.dirname(__file__))
import v3_neck_jaw as v3
from common import *
from cadlib import rot, trans, rotz
from poses import SG, knee_units, Robot
import mujoco

WORK, ROOT = sys.argv[1], sys.argv[2]
OUT = ROOT + '/v4/sim'
os.makedirs(OUT + '/meshes', exist_ok=True)
v4 = pickle.load(open(WORK + '/v4.pkl', 'rb'))
d = pickle.load(open(WORK + '/asm.pkl', 'rb'))
mm = 0.001
MAX_FACES = 2500

bot = Robot(d, replace={'stackchan_head': None, 'cores3': None, 'cores3_screen': None, 'cores3_camera': None, 'bottom_cover': v3.cov2_tm},
            extra=v4['STATIC'])
STAND_DZ = bot.stand_on_floor()[2, 3]
LEGS = ('FL', 'FR', 'BR', 'BL')

# ----------------------------------------------------------------------------- servo / joint data
MG90S = dict(effort=0.21, kp=3.0, kv=0.03)        # ~2.2 kg*cm at 5-6 V (est.); position-servo gain until it saturates
SCS0009 = dict(effort=0.22, kp=3.0, kv=0.03)      # ~2.3 kg*cm (est.)
ARMATURE, DAMPING, FRICTIONLOSS = 0.001, 0.01, 0.005   # reflected gearbox inertia of a ~1:300 micro servo (est.)
r = np.radians
RANGES = {'FL_hip': (r(-57), r(88)), 'FR_hip': (r(-88), r(48)), 'BR_hip': (r(-88), r(88)), 'BL_hip': (r(-88), r(88)),
          **{f'{k}_knee': (r(-118), r(48)) for k in LEGS},              # CAD sign: + = foot inward (firmware: 48 in / 118 out)
          'neck_pan': (r(-120), r(120)), 'neck_tilt': (r(v3.TILT_RANGE[0]), r(v3.TILT_RANGE[1]))}
AX, RG = v4['axes'], v4['ranges']
for s_ in 'lr':
    RANGES.update({f'arm_{s_}_yaw': tuple(r(np.array(RG['yaw']))), f'arm_{s_}_pitch': tuple(r(np.array(RG['pitch']))),
                   f'arm_{s_}_jaw': tuple(r(np.array(RG['jaw']))), f'arm_{s_}_jaw2': tuple(r(np.array(RG['jaw'])))})

# ----------------------------------------------------------------------------- bodies: (name, parent, joint, axis, anchor, parts)
def parts_of(pred):
    return [(n, m, c) for n, m, c, l, kd in bot.parts if pred(n, l, kd)]

base_parts = parts_of(lambda n, l, kd: kd == 'body')
BODIES = [('base', None, None, None, np.zeros(3), base_parts)]
for k in LEGS:
    BODIES.append((f'{k}_thigh', 'base', f'{k}_hip', [0, 0, 1], bot.HIP[k], parts_of(lambda n, l, kd, k=k: l == k and kd == 'femur')))
    BODIES.append((f'{k}_shin', f'{k}_thigh', f'{k}_knee', bot.TV[k], bot.KW[k], parts_of(lambda n, l, kd, k=k: l == k and kd == 'foot')))
BODIES.append(('neck', 'base', 'neck_pan', [0, 0, 1], np.array([v3.NX, 0, v3.TT_Z0]), list(v3.NECK)))
BODIES.append(('head', 'neck', 'neck_tilt', [0, -1, 0], np.array([v3.NX, 0, v3.TZ]), list(v3.HEAD)))   # + = look down
ARM_LINKS = {'yaw': ('shoulder', 'yaw_horn', 'pitch_servo'), 'pitch': ('pitch_horn', 'forearm', 'grip_servo', 'grip_horn', 'pushrod'),
             'jawB': ('jaw_finger',), 'jawA': ('jaw_palm',)}   # gripper horn + pushrod drawn at jaw 0 (the linkage itself is the jaw joint)
for s, sg in (('l', -1), ('r', 1)):
    P_ = v4['PARTS'][s]; C_ = v4['COL']
    plist = lambda link: [(f'arm_{s}_{n}', P_[n], C_[n]) for n in ARM_LINKS[link]]
    # yaw + = outward (right arm clockwise from above), pitch + = claw down, jaw/jaw2 + = open (finger swings out, palm in)
    BODIES.append((f'arm_{s}_shoulder', 'base', f'arm_{s}_yaw', [0, 0, -sg], np.array([AX['JX'], sg * AX['SY'], -7.0]), plist('yaw')))
    BODIES.append((f'arm_{s}_forearm', f'arm_{s}_shoulder', f'arm_{s}_pitch', [0, -1, 0], np.array([AX['XP'], sg * AX['SY'], AX['ZP']]), plist('pitch')))
    BODIES.append((f'arm_{s}_finger', f'arm_{s}_forearm', f'arm_{s}_jaw', [0, 0, -sg], np.array([AX['XG'], sg * AX['YG'], AX['ZB']]), plist('jawB')))
    BODIES.append((f'arm_{s}_palm', f'arm_{s}_forearm', f'arm_{s}_jaw2', [0, 0, sg], np.array([AX['XG'], sg * (AX['YG'] + AX['PA'][1]), AX['ZB']]), plist('jawA')))
COLL_GROUPS = {   # collision hulls per body (convex; MuJoCo collides meshes by their hulls anyway)
    'base': [('base_frame', ('chassis', 'v3_body_cover', 'bottom_cover') + tuple(f'hip_servo_{k}' for k in LEGS)), ('belly_cone', ('belly_cone',)),
             ('arm_base', ('arm_housing', 'yaw_servo_l', 'yaw_servo_r'))],
}

def hexrgba(c):
    c = c.lstrip('#'); return ' '.join(f'{int(c[i:i + 2], 16) / 255:.3f}' for i in (0, 2, 4)) + ' 1'

def fmt(v):
    return ' '.join(f'{x:.6g}' for x in v)

asset, mats, body_xml = [], {}, {}
mass_tot = 0.0
inertials = {}
for bname, parent, jname, axis, anchor, parts in BODIES:
    vis = []
    for n, m, c in parts:
        local = m.copy(); local.apply_translation(-anchor)
        if len(local.faces) > MAX_FACES:                           # keep the visual meshes light (MuJoCo, viewers, git)
            local = local.simplify_quadric_decimation(face_count=MAX_FACES)
        fn = f'{bname}__{n}.stl'; local.export(f'{OUT}/meshes/{fn}')
        asset.append(f'    <mesh name="{bname}__{n}" file="{fn}" scale="{mm} {mm} {mm}"/>')
        if c not in mats: mats[c] = f'm{len(mats)}'
        vis.append(f'<geom class="visual" mesh="{bname}__{n}" material="{mats[c]}"/>')
    groups = COLL_GROUPS.get(bname, [(bname, tuple(n for n, m, c in parts if n not in ('cores3_screen', 'cores3_camera', 'pan_horn', 'tilt_horn') and not n.endswith('_horn')))])
    col = []
    for gname, names in groups:
        ms = [m for n, m, c in parts if n in names]
        if not ms: continue
        hull = trimesh.util.concatenate(ms).convex_hull; hull.apply_translation(-anchor)
        fn = f'{gname}_hull.stl'; hull.export(f'{OUT}/meshes/{fn}')
        asset.append(f'    <mesh name="{gname}_hull" file="{fn}" scale="{mm} {mm} {mm}"/>')
        foot = bname.endswith('_shin')
        col.append(f'<geom name="{bname[:-5] + "_foot_collision" if foot else gname + "_collision"}" class="{"foot" if foot else "collision"}" mesh="{gname}_hull"/>')
    # inertia: estimated part masses (same model as the CAD checks), hull inertia scaled to that mass
    ms_ = np.array([(v3.MASS['servo'] if n.endswith(('yaw_servo', 'pitch_servo', 'grip_servo')) or n.startswith('yaw_servo') else v3.part_mass(n, m)) for n, m, c in parts]); ctr = np.array([(m.center_mass if m.is_watertight else m.bounds.mean(0)) for n, m, c in parts])
    mass = ms_.sum() / 1000.0; com = (ms_[:, None] * ctr).sum(0) / ms_.sum() - anchor
    hull = trimesh.util.concatenate([m for n, m, c in parts]).convex_hull
    I = hull.moment_inertia / hull.volume * mass * 1e-6          # kg m^2 (density-1 inertia in mm^5 -> per unit volume x mass)
    I = 0.5 * (I + I.T); mass_tot += mass
    inertials[bname] = (mass, com * mm, I)
    body_xml[bname] = (parent, jname, axis, anchor, vis, col)

def site_xml(bname):
    if not bname.endswith('_shin'): return ''
    k = bname[:-5]
    tip = bot.tip_pts(k, 0, 0).mean(0) - body_xml[bname][3]
    return f'<site name="{k}" pos="{fmt(tip * mm)}" type="sphere" size="0.004" group="5"/>'

def emit(bname, ind):
    parent, jname, axis, anchor, vis, col = body_xml[bname]
    ppos = anchor - (body_xml[parent][3] if parent else np.zeros(3))
    m, c, I = inertials[bname]
    pad = ' ' * ind
    pos = f'0 0 {STAND_DZ * mm + 0.002:.4f}' if parent is None else fmt(ppos * mm)
    s = [f'{pad}<body name="{bname}" pos="{pos}"' + (' childclass="sesant"' if parent is None else '') + '>']
    s.append(f'{pad}  <inertial pos="{fmt(c)}" mass="{m:.5f}" fullinertia="{fmt([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]])}"/>')
    if parent is None:
        s.append(f'{pad}  <freejoint name="floating_base_joint"/>')
        s.append(f'{pad}  <site name="imu" pos="0 0 {0.035:.3f}" group="5"/>')
    else:
        lo, hi = RANGES[jname]
        s.append(f'{pad}  <joint name="{jname}" axis="{fmt(axis)}" range="{lo:.4f} {hi:.4f}"/>')
    for g in vis + col: s.append(f'{pad}  {g}')
    st = site_xml(bname)
    if st: s.append(f'{pad}  {st}')
    if bname == 'head':
        s.append(f'{pad}  <camera name="cores3" pos="{fmt((v3.CAM0 - anchor) * mm)}" xyaxes="0 -1 0 0 0 1" fovy="50"/>')
    for child in [b for b, v in body_xml.items() if v[0] == bname]:
        s += emit(child, ind + 2)
    s.append(f'{pad}</body>')
    return s

robot_xml = f'''<mujoco model="sesant">
  <!-- SesAnt v4 (v1 body + v2 belly cone + v3 neck + v4 Jumper arms with two-jaw claws), generated by sesant/cad/scripts/v4_sim.py.
       Masses are estimates (prints at 45 % fill, MG90S 13.4 g, SCS0009 13.5 g, CoreS3 73 g, battery 45 g). -->
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>
  <default>
    <default class="sesant">
      <joint armature="{ARMATURE}" damping="{DAMPING}" frictionloss="{FRICTIONLOSS}"/>
      <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/></default>
      <default class="collision"><geom type="mesh" group="3" contype="1" conaffinity="0" density="0" condim="1"/></default>
      <default class="foot"><geom type="mesh" group="3" contype="1" conaffinity="0" density="0" condim="3" friction="0.8 0.005 0.0001" priority="1"/></default>
    </default>
  </default>
  <asset>
{chr(10).join(f'    <material name="{v}" rgba="{hexrgba(k)}"/>' for k, v in mats.items())}
{chr(10).join(asset)}
  </asset>
  <worldbody>
{chr(10).join(emit('base', 4))}
  </worldbody>
  <equality>
    <!-- the printed gear pair: the palm jaw turns with the finger jaw, the other way (both + = open) -->
    <joint joint1="arm_l_jaw2" joint2="arm_l_jaw" polycoef="0 1 0 0 0"/>
    <joint joint1="arm_r_jaw2" joint2="arm_r_jaw" polycoef="0 1 0 0 0"/>
  </equality>
  <sensor>
    <gyro name="imu_ang_vel" site="imu"/>
    <velocimeter name="imu_lin_vel" site="imu"/>
    <accelerometer name="imu_lin_acc" site="imu"/>
    <framezaxis name="imu_upvector" objtype="body" objname="world" reftype="site" refname="imu"/>
    <subtreeangmom name="root_angmom" body="base"/>
  </sensor>
</mujoco>
'''
open(OUT + '/sesant.xml', 'w').write(robot_xml)

# ----------------------------------------------------------------------------- scene for plain MuJoCo: floor, actuators, keyframes
JOINTS = [f'{k}_{j}' for k in LEGS for j in ('hip', 'knee')] + ['neck_pan', 'neck_tilt'] + [f'arm_{s_}_{j}' for s_ in 'lr' for j in ('yaw', 'pitch', 'jaw')]
def act(j):
    p = SCS0009 if j.startswith('neck') else MG90S
    lo, hi = RANGES[j]
    return f'    <position name="{j}" joint="{j}" kp="{p["kp"]}" kv="{p["kv"]}" forcerange="{-p["effort"]} {p["effort"]}" ctrlrange="{lo:.4f} {hi:.4f}"/>'
crouch_q = {'neck_tilt': np.radians(v4['REACH_TILT'])}       # 'reach': standing, both arms down, claws open at the floor
for s_ in 'lr':
    crouch_q.update({f'arm_{s_}_pitch': np.radians(v4['REACH']['pitch']), f'arm_{s_}_jaw': np.radians(v4['REACH']['jaw'])})
carry_q = {f'arm_{s_}_pitch': np.radians(v4['CARRY']['pitch']) for s_ in 'lr'}
QJ = [model_j for model_j in [f'{k}_{j}' for k in LEGS for j in ('hip', 'knee')] + ['neck_pan', 'neck_tilt'] +
      [f'arm_{s_}_{j}' for s_ in 'lr' for j in ('yaw', 'pitch', 'jaw', 'jaw2')]]          # qpos order = body order in sesant.xml
def key(name, z, q):
    q = dict(q); q.update({f'arm_{s_}_jaw2': q.get(f'arm_{s_}_jaw', 0.0) for s_ in 'lr'})
    qpos = [0, 0, z, 1, 0, 0, 0] + [q.get(j, 0.0) for j in QJ]
    return f'    <key name="{name}" qpos="{fmt(qpos)}" ctrl="{fmt([q.get(j, 0.0) for j in JOINTS])}"/>'
scene = f'''<mujoco model="sesant scene">
  <include file="sesant.xml"/>
  <option timestep="0.002" cone="elliptic" impratio="10"/>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="1 1 1" rgb2=".80 .85 .92" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1=".92 .92 .92" rgb2=".82 .82 .82" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0"/>
  </asset>
  <worldbody>
    <light pos="0.3 -0.6 1.2" dir="-0.2 0.5 -1" directional="true"/>
    <geom name="floor" type="plane" size="2 2 0.05" material="grid" contype="1" conaffinity="1" condim="3" friction="0.8 0.005 0.0001"/>
  </worldbody>
  <actuator>
{chr(10).join(act(j) for j in JOINTS)}
  </actuator>
  <keyframe>
{key('stand', STAND_DZ * mm + 0.001, {})}
{key('carry', STAND_DZ * mm + 0.001, carry_q)}
{key('reach', STAND_DZ * mm + 0.001, crouch_q)}
  </keyframe>
</mujoco>
'''
open(OUT + '/scene.xml', 'w').write(scene)

# ----------------------------------------------------------------------------- MuJoCo checks
rep = []
def P(*s):
    line = ' '.join(str(x) for x in s); print(line); rep.append(line)
model = mujoco.MjModel.from_xml_path(OUT + '/scene.xml')
data = mujoco.MjData(model)
P('== SesAnt v4 MuJoCo model checks (mujoco', mujoco.__version__ + ') ==')
P(f'model: {model.nbody - 1} bodies, {model.njnt - 1} hinge joints + free base, {model.nu} position actuators, {model.ngeom} geoms; '
  f'mass {sum(model.body_mass) * 1000:.0f} g (CAD estimate {mass_tot * 1000:.0f} g)')
jid = {j: model.joint(j).qposadr[0] for j in JOINTS}
aid = {j: model.actuator(j).id for j in JOINTS}
base = model.body('base').id
def euler(q):
    w, x, y, z = q
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return roll, pitch
def run(seconds, ctrl_fn=None):
    n = int(seconds / model.opt.timestep); fmax = 0.0
    for i in range(n):
        if ctrl_fn: ctrl_fn(data, i * model.opt.timestep)
        mujoco.mj_step(model, data)
        fmax = max(fmax, max(abs(data.actuator_force[aid[j]]) / (SCS0009 if j.startswith('neck') else MG90S)['effort'] for j in JOINTS))
    return fmax
def reset(keyname):
    mujoco.mj_resetDataKeyframe(model, data, model.key(keyname).id); mujoco.mj_forward(model, data)

reset('stand'); z0 = data.xpos[base][2]
f = run(3.0)
rl, pt = euler(data.xquat[base])
P(f'stand (3 s, holding the stand pose): base z {z0 * 1000:.1f} -> {data.xpos[base][2] * 1000:.1f} mm (CAD {STAND_DZ:.1f}); roll {rl:.2f}, pitch {pt:.2f} deg; '
  f'peak actuator load {f * 100:.0f} % of stall')
feet_geoms = [model.geom(f'{k}_foot_collision').id for k in LEGS]
touch = sorted({model.geom(c.geom1).name if model.geom(c.geom2).name == 'floor' else model.geom(c.geom2).name for c in data.contact[:data.ncon]})
P('  floor contacts while standing:', touch, '(expected: the 4 foot hulls)')

reset('stand')
q0 = np.array([data.ctrl[aid[j]] for j in JOINTS]); q1 = np.array([crouch_q.get(j, 0.0) for j in JOINTS])
def ramp(dt):
    def fn(dat, t):
        s = min(1.0, t / dt); s = 0.5 - 0.5 * np.cos(np.pi * s)
        for i, j in enumerate(JOINTS): dat.ctrl[aid[j]] = q0[i] + (q1[i] - q0[i]) * s
    return fn
f = run(2.5, ramp(1.5))
def geom_low(gid):
    mid = model.geom_dataid[gid]; vadr, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
    v = model.mesh_vert[vadr:vadr + vn]
    return (data.geom_xpos[gid][None, :] + v @ data.geom_xmat[gid].reshape(3, 3).T)[:, 2].min()
jaw_gids = [model.geom(f'arm_{s}_{b}_collision').id for s in 'lr' for b in ('finger', 'palm')]
rl, pt = euler(data.xquat[base])
cpl = max(abs(data.qpos[model.joint(f'arm_{s}_jaw2').qposadr[0]] - data.qpos[model.joint(f'arm_{s}_jaw').qposadr[0]]) for s in 'lr')
P(f'reach (standing, both arms {v4["REACH"]["pitch"]:.0f} deg down, claws open; ramp 1.5 s + hold 1 s): lowest jaw {min(geom_low(g) for g in jaw_gids) * 1000:.1f} mm '
  f'above the floor (CAD 2.0); body pitch {pt:.2f}, roll {rl:.2f} deg; palm jaw follows the finger jaw within {np.degrees(cpl):.2f} deg (gear coupling); peak load {f * 100:.0f} %')
touch = sorted({model.geom(c.geom1).name if model.geom(c.geom2).name == 'floor' else model.geom(c.geom2).name for c in data.contact[:data.ncon]})
P('  floor contacts in the reach pose:', touch)

reset('stand')
YW = np.radians(np.array(v4['ranges']['yaw'])); PT = np.radians(np.array(v4['ranges']['pitch']))
def sweep(dat, t):
    dat.ctrl[aid['neck_pan']] = np.radians(120) * np.sin(2 * np.pi * t / 2.0); dat.ctrl[aid['neck_tilt']] = np.radians(45)
    for s, ph in (('l', 0.0), ('r', np.pi)):
        u = 0.5 - 0.5 * np.cos(2 * np.pi * t / 1.5 + ph)
        dat.ctrl[aid[f'arm_{s}_yaw']] = YW[0] + (YW[1] - YW[0]) * u
        dat.ctrl[aid[f'arm_{s}_pitch']] = PT[0] + (np.radians(v4['REACH']['pitch'] - 2.0) - PT[0]) * (1 - u)   # down to just above the floor
        dat.ctrl[aid[f'arm_{s}_jaw']] = np.radians(v4['ranges']['jaw'][1]) * u
worst = 0.0
for i in range(int(4.5 / model.opt.timestep)):
    sweep(data, i * model.opt.timestep); mujoco.mj_step(model, data)
    rl, pt = euler(data.xquat[base]); worst = max(worst, abs(rl), abs(pt))
P(f'neck sweep (pan +-120 deg, head down 45) + both arms sweeping yaw, pitch (up to floor height) and jaws in antiphase, standing: worst body tilt {worst:.2f} deg; '
  f'neck tracking error {abs(np.degrees(data.qpos[jid["neck_pan"]] - data.ctrl[aid["neck_pan"]])):.1f} deg at the end')

reset('stand'); data.qpos[2] += 0.03; mujoco.mj_forward(model, data)
run(2.0); rl, pt = euler(data.xquat[base])
P(f'drop from 30 mm: settles at base z {data.xpos[base][2] * 1000:.1f} mm, roll {rl:.2f} pitch {pt:.2f} deg, '
  f'{"upright" if abs(rl) < 5 and abs(pt) < 5 else "FELL"}')
open(ROOT + '/v4/assembly/v4_sim_report.txt', 'w').write('\n'.join(rep) + '\n')
json.dump({'joints': JOINTS, 'stand_z_m': round(STAND_DZ * mm, 5), 'reach': {j: round(float(crouch_q.get(j, 0.0)), 4) for j in JOINTS},
           'carry': {j: round(float(carry_q.get(j, 0.0)), 4) for j in JOINTS},
           'ranges_rad': {j: [round(float(RANGES[j][0]), 4), round(float(RANGES[j][1]), 4)] for j in JOINTS},
           'mg90s': MG90S, 'scs0009': SCS0009, 'armature': ARMATURE},
          open(OUT + '/sesant_sim.json', 'w'), indent=1)

# a rendered frame from the simulation (needs an OpenGL context; skipped if there is none)
try:
    reset('reach'); run(0.5)
    rnd = mujoco.Renderer(model, 720, 960)
    cam = mujoco.MjvCamera(); cam.lookat[:] = data.xpos[base]; cam.distance = 0.55; cam.azimuth = 35; cam.elevation = -18
    rnd.update_scene(data, cam); from PIL import Image
    Image.fromarray(rnd.render()).save(ROOT + '/v4/images/sesant_mujoco_reach.png')
    reset('carry'); run(0.5); cam.lookat[:] = data.xpos[base]; rnd.update_scene(data, cam)
    Image.fromarray(rnd.render()).save(ROOT + '/v4/images/sesant_mujoco_stand.png')
    print('rendered MuJoCo frames')
except Exception as e:
    print('no MuJoCo render:', e)
