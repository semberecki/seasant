"""SesAnt Jumper animation from the MuJoCo model (sesant_jumper/sim/scene.xml): look around, reach down with both arms, grab, lift, wave,
stand up, lift the front-left leg and tap. Scripted joint targets on the simulated servos (not a learned policy).

Usage: python jumper_animate.py <work dir> <sesant root>   ->  sesant_jumper/images/sesant_jumper_animation.gif
"""
import sys, os, json
import numpy as np, mujoco
from PIL import Image

WORK, ROOT = sys.argv[1], sys.argv[2]
SIM = ROOT + '/sesant_jumper/sim'
model = mujoco.MjModel.from_xml_path(SIM + '/scene.xml')
data = mujoco.MjData(model)
meta = json.load(open(SIM + '/sesant_sim.json'))
JOINTS = meta['joints']
d = np.radians
W, H, FPS = 560, 400, 20

STAND = {j: 0.0 for j in JOINTS}
def pose(base=None, **kw):
    p = dict(base or STAND); p.update(kw); return p
CARRY = pose(**{k: v for k, v in meta['carry'].items() if abs(v) > 0})
REACH = pose(**{k: v for k, v in meta['reach'].items() if abs(v) > 0})
CLOSED = pose(REACH, arm_l_jaw=0.0, arm_r_jaw=0.0)
LIFT = pose(CARRY, arm_l_pitch=d(-12), arm_r_pitch=d(-12), neck_tilt=d(25))
WAVE_UP = pose(CARRY, arm_r_yaw=d(10), arm_r_pitch=d(-18), arm_r_jaw=d(35), neck_pan=d(-25))
WAVE_DN = pose(WAVE_UP, arm_r_pitch=d(20), arm_r_jaw=0.0)
paw_base = pose(CARRY, FR_hip=d(40), BL_hip=d(-45), neck_tilt=d(20))
paw_up = dict(paw_base, FL_hip=d(-20), FL_knee=d(-95))
paw_tap = dict(paw_up, FL_knee=d(-60))
KEYS = [(0.0, CARRY, 'SesAnt Jumper'), (0.6, CARRY, 'SesAnt Jumper'),
        (1.3, pose(CARRY, neck_pan=d(50), neck_tilt=d(-10)), 'StackChan neck: look around'),
        (2.1, pose(CARRY, neck_pan=d(-50), neck_tilt=d(-10)), 'StackChan neck: look around'),
        (2.7, CARRY, 'StackChan neck: look around'),
        (3.8, REACH, 'reach: both Jumper arms down, both jaws open'), (4.3, REACH, 'reach: both Jumper arms down, both jaws open'),
        (4.8, CLOSED, 'grab'), (5.1, CLOSED, 'grab'),
        (6.0, LIFT, 'lift'), (6.4, LIFT, 'lift'),
        (6.9, WAVE_UP, 'wave'), (7.3, WAVE_DN, 'wave'), (7.7, WAVE_UP, 'wave'), (8.1, WAVE_DN, 'wave'),
        (8.7, CARRY, 'carry (arms level for walking)'),
        (9.4, paw_base, 'paw: shift onto three feet'),
        (10.0, paw_up, 'paw: lift the front-left leg'),
        (10.3, paw_tap, 'paw: tap'), (10.6, paw_up, 'paw: tap'), (10.9, paw_tap, 'paw: tap'), (11.2, paw_up, 'paw: tap'),
        (11.9, paw_base, 'paw: tap'), (12.6, CARRY, 'SesAnt Jumper'), (13.2, CARRY, 'SesAnt Jumper')]

def target(t):
    for (t0, p0, c0), (t1, p1, c1) in zip(KEYS, KEYS[1:]):
        if t0 <= t <= t1:
            s = 0.5 - 0.5 * np.cos(np.pi * (t - t0) / max(t1 - t0, 1e-6))
            return {j: p0[j] + (p1[j] - p0[j]) * s for j in JOINTS}, c1 if s > 0.5 else c0
    return KEYS[-1][1], KEYS[-1][2]

mujoco.mj_resetDataKeyframe(model, data, model.key('carry').id)
for _ in range(250): mujoco.mj_step(model, data)           # settle on the floor
rnd = mujoco.Renderer(model, H, W)
cam = mujoco.MjvCamera(); cam.distance = 0.36; cam.elevation = -14
base = model.body('base').id
look = data.xpos[base].copy()
frames, tilt_max = [], 0.0
steps_per_frame = int(round(1.0 / FPS / model.opt.timestep))
T_END = KEYS[-1][0]
t = 0.0; k = 0
from PIL import ImageDraw, ImageFont
try:
    font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 17)
except Exception:
    font = ImageFont.load_default()
while t < T_END:
    tg, cap = target(t)
    for j in JOINTS: data.ctrl[model.actuator(j).id] = tg[j]
    for _ in range(steps_per_frame):
        mujoco.mj_step(model, data)
    t = data.time - 0.5
    w, x, y, z = data.xquat[base]
    tilt_max = max(tilt_max, abs(np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))),
                   abs(np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))))
    look = 0.9 * look + 0.1 * data.xpos[base]
    cam.lookat[:] = look + [-0.02, 0.0, 0.015]
    cam.azimuth = 25 + 30 * np.sin(2 * np.pi * t / T_END)  # slow orbit around the front-left
    rnd.update_scene(data, cam)
    im = Image.fromarray(rnd.render())
    dr = ImageDraw.Draw(im); dr.text((14, 12), cap, fill=(40, 40, 40), font=font)
    frames.append(im)
out = ROOT + '/sesant_jumper/images/sesant_jumper_animation.gif'
pal = [f.convert('P', palette=Image.ADAPTIVE, colors=128) for f in frames]
pal[0].save(out, save_all=True, append_images=pal[1:], duration=int(1000 / FPS), loop=0, optimize=True)
print(f'{len(frames)} frames, {T_END:.1f} s -> {out} ({os.path.getsize(out) / 1e6:.1f} MB); worst body tilt {tilt_max:.1f} deg '
      f'({"stayed upright" if tilt_max < 20 else "TIPPED"})')
sheet = Image.new('RGB', (W * 4, H * 2), 'white')
for i, idx in enumerate(np.linspace(0, len(frames) - 1, 8).astype(int)):
    sheet.paste(frames[idx], ((i % 4) * W, (i // 4) * H))
sheet.save(WORK + '/jumper_animation_sheet.png')
