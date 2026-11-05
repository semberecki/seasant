"""v3 images: hero, grab pose, paw pose, docked on the puck, mandible close-up, v1 vs v3, parts."""
import sys, os, pickle; sys.path.insert(0, os.path.dirname(__file__))
from common import *
from cadlib import to_tm, trans
import manifold3d as mf
from zrender import render, grid

WORK, ROOT = sys.argv[1], sys.argv[2]
v = pickle.load(open(WORK + '/v3.pkl', 'rb')); IMG = ROOT + '/v3/images/'
rob = lambda parts: [(m, c) for n, m, c in parts]

render(rob(v['look']), view=(22, -135), size=(1600, 1250), ss=2).save(IMG + 'sesant_v3_hero.png')
render(rob(v['stand']), view=(24, 45), size=(1200, 1000), ss=2).save(IMG + 'sesant_v3_back.png')

# grab: a 30 mm ball between the open mandibles
G = rob(v['grab'])
tips = np.vstack([m.vertices[np.argmin(m.vertices[:, 0])] for n, m, c in v['grab'] if n.startswith('mandible')]).mean(0)
ball = trimesh.creation.icosphere(subdivisions=3, radius=15.0); ball.apply_translation([tips[0] + 22, tips[1], 15.0])
render(G + [(ball, '#d23c3c')], view=(18, -150), size=(1500, 1050), ss=2).save(IMG + 'sesant_v3_grab.png')
render(rob(v['paw']), view=(16, -120), size=(1400, 1100), ss=2).save(IMG + 'sesant_v3_paw.png')
render(v['puck_items'] + rob(v['seat']), view=(22, -135), size=(1500, 1150), ss=2).save(IMG + 'sesant_v3_on_puck.png')

# mandibles from below: closed / open
def jaw_view(mov):
    items = [(m, c) for n, m, c in v['static'] if n in ('jaw_housing', 'jaw_servo')] + [(m, c) for n, m, c in mov if n.startswith(('mandible', 'jaw'))]
    return items
B = np.array([[-100, -35, -15], [-36, 35, 30]], float)
ims = [render(jaw_view(v['jaw_closed']), view=(-90, -90), size=(700, 640), bounds=B),
       render(jaw_view(v['jaw_open']), view=(-90, -90), size=(700, 640), bounds=B),
       render(jaw_view(v['jaw_open']), view=(20, -140), size=(700, 640))]
grid(ims, 3, labels=['mandibles closed (from below)', 'open 40 deg each: gear pair mirrors the servo', 'jaw housing, MG90S and mandibles']).save(IMG + 'sesant_v3_mandibles.png')

# v1 vs v3, same scale, side view
d1 = pickle.load(open(WORK + '/asm.pkl', 'rb'))
v1 = [(m.copy(), c) for n, m, c, g in d1['asm']]
g1 = min(m.bounds[0, 2] for m, c in v1)
for m, c in v1: m.apply_translation([0, 0, -g1])
xl = rob(v['stand'])
allv = np.vstack([m.vertices for m, c in xl + v1]); BB = np.array([allv.min(0) - 5, allv.max(0) + 5])
ims = [render(v1, view=(0, -90), size=(700, 620), bounds=BB), render(xl, view=(0, -90), size=(700, 620), bounds=BB)]
grid(ims, 2, labels=['SesAnt v1 - side', 'SesAnt v3 (neck + mandibles) - side, same scale']).save(IMG + 'v1_vs_v3.png')

p = v['parts']
ims = [render([(p['cover'], '#f5a54a')], view=(30, -130), size=(600, 520)), render([(p['house'], '#f28c28')], view=(20, -140), size=(600, 520)),
       render([(p['neck'], '#3a3f4a')], view=(30, -130), size=(600, 520)), render([(p['head'], '#f5a54a')], view=(20, -140), size=(600, 520)),
       render([(p['mandA'], '#e9e2d0')], view=(90, -90), size=(600, 520)), render([(p['mandB'], '#e9e2d0')], view=(90, -90), size=(600, 520))]
grid(ims, 3, labels=['body cover with neck collar', 'jaw housing (MG90S)', 'neck: turntable + tilt cradle', 'head: CoreS3 shell + tilt arms',
                     'left mandible (on the servo horn)', 'right mandible (M3 pivot)']).save(IMG + 'sesant_v3_parts.png')
