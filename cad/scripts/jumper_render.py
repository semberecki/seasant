"""SesAnt Jumper images: hero with the two Jumper arms, the two-jaw claw (closed / open, next to Jumper's), reach pose, parts."""
import sys, os, pickle; sys.path.insert(0, os.path.dirname(__file__))
from common import *
from zrender import render, grid
from jumper_urdf import JumperURDF
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

WORK, ROOT = sys.argv[1], sys.argv[2]
v = pickle.load(open(WORK + '/jumper.pkl', 'rb')); IMG = ROOT + '/sesant_jumper/images/'
rob = lambda parts: [(m, c) for n, m, c in parts]

render(rob(v['stand']), view=(22, -135), size=(1600, 1250), ss=2).save(IMG + 'sesant_jumper_hero.png')
R = rob(v['reach'])
props = []
for s, col in (('l', '#2f7de1'), ('r', '#e8a33d')):            # a bottle cap in front of each open claw
    jaws = np.vstack([m.vertices for n, m, c in v['reach'] if n in (f'arm_{s}_jaw_palm', f'arm_{s}_jaw_finger')])
    cap = trimesh.creation.cylinder(radius=10.0, height=10.0, sections=48)
    cap.apply_translation([jaws[:, 0].min() + 14.0, jaws[:, 1].mean(), 5.0])
    props.append((cap, col))
render(R + props, view=(16, -145), size=(1500, 1050), ss=2).save(IMG + 'sesant_jumper_reach.png')

J = JumperURDF()
jc = [(m, '#c0c4cc' if 'palm' in n else '#d9a441') for n, m in J.meshes(frame='LF_palm_link', names=['LF_palm_link', 'LF_palm_pad_f_link', 'LF_palm_pad_b_link',
      'LF_palm_grip_insert_link', 'LF_finger_link', 'LF_finger_tip_link', 'LF_finger_grip_insert_link'])]
arm = lambda k: [(m, c) for n, m, c in v[k] if n.startswith('arm_r')]
ims = [render(jc, view=(0, 0), size=(700, 520)), render(arm('claw_closed'), view=(90, -90), size=(700, 520)), render(arm('claw_open'), view=(90, -90), size=(700, 520))]
grid(ims, 3, labels=['Jumper claw (its URDF meshes, 159 mm)', 'SesAnt Jumper right arm, claw closed (from above)', 'both jaws swing open sideways, 35 deg each: one MG90S + gear pair']).save(IMG + 'sesant_jumper_claw.png')

fig, ax = plt.subplots(figsize=(9, 3.4), dpi=150)
for g, c, lab in [(v['palm_j'], '#9aa0a8', 'palm'), (v['fing_j'], '#d9a441', 'finger')]:
    for p in (g.geoms if hasattr(g, 'geoms') else [g]):
        ax.fill(*p.exterior.xy, color=c, alpha=0.75, lw=0.6, ec='k', label=lab)
ax.plot(v['PIV'][1], v['PIV'][2], 'k+', ms=14, mew=2)
ax.set_aspect('equal'); ax.axis('off'); ax.legend(loc='lower right', fontsize=8, frameon=False)
ax.set_title('Jumper claw outline in the finger plane (palm frame, mm), the source of the SesAnt jaws', fontsize=9)
fig.tight_layout(); fig.savefig(IMG + 'jumper_claw_outline.png'); plt.close(fig)

P_ = v['PARTS']['r']
ims = [render([(v['house'], '#f28c28')], view=(20, -140), size=(600, 520)),
       render([(P_['shoulder'], '#f5a54a'), (P_['pitch_servo'], '#2b2d33')], view=(20, -140), size=(600, 520)),
       render([(P_['forearm'], '#f5a54a'), (P_['grip_servo'], '#2b2d33')], view=(20, -140), size=(600, 520)),
       render([(P_['jaw_palm'], '#c9ccd2'), (P_['jaw_finger'], '#d9a441')], view=(90, -90), size=(600, 520))]
grid(ims, 4, labels=['arm-base housing (2 yaw servos)', 'shoulder + pitch servo (x2)', 'forearm + gripper servo (x2)', 'geared jaws: palm + finger (x2)']).save(IMG + 'sesant_jumper_parts.png')
