"""v2 home puck images: hero, sit sequence, cut-away of the cone in the funnel, parts."""
import sys, os, pickle; sys.path.insert(0, os.path.dirname(__file__))
from common import *
from zrender import render, grid
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly

WORK, ROOT = sys.argv[1], sys.argv[2]
v = pickle.load(open(WORK + '/v2.pkl', 'rb')); IMG = ROOT + '/v2/images/'
puck = v['puck_items']
rob = lambda parts: [(m, c) for n, m, c in parts]

render(puck + rob(v['seated']), view=(22, -135), size=(1600, 1150), ss=2).save(IMG + 'v2_puck_hero.png')
render(puck, view=(35, -130), size=(1100, 800), ss=2).save(IMG + 'v2_puck.png')

allv = np.vstack([m.vertices for m, c in puck[:2]] + [m.vertices for n, m, c in v['seq'][0][1]])
B = np.array([allv.min(0) - 5, allv.max(0) + 5])
labels = ['1. walk over the puck (cone tip clears it by 16 mm)', '2. knees open: the cone drops into the funnel',
          '3. servos relax: the cone slides to the centre, pins touch the pads']
ims = [render(puck + rob(parts), view=(0, -90), size=(1000, 460), bounds=B) for dz, parts in v['seq']]
grid(ims, 1, labels=labels).save(IMG + 'v2_sit_sequence.png')

# cut-away at y = 0 through the seated robot's centre
P_ = v['params']
fig, ax = plt.subplots(figsize=(10, 4.6), dpi=150)
items = [(v['puck'], '#3a3f4a', 'puck')]
S = {n: m for n, m, c in v['seated']}
for n, col in [('belly_cone', '#5a606b'), ('bottom_cover', '#3a3f4a'), ('chassis', '#f28c28'), ('battery', '#7bc96f')]:
    items.append((S[n], col, n))
for m, col, n in items:
    s = m.section(plane_origin=[0, 0, 0], plane_normal=[0, 1, 0])
    if s is None: continue
    M = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float)
    p2, Mb = s.to_2D(to_2D=M)
    for poly in p2.polygons_full:
        ext = np.array(poly.exterior.coords)
        ax.add_patch(MPoly(ext, closed=True, fc=col, ec='k', lw=0.6))
        for h in poly.interiors: ax.add_patch(MPoly(np.array(h.coords), closed=True, fc='white', ec='k', lw=0.4))
ax.add_patch(plt.Rectangle((-1.5, 4.0), 3.0, v['H'] - P_['DEPTH'] - 4.0 + P_['GAP'] + 0.2, fc='#d4a017', ec='k', lw=0.5))   # centre pin
ax.set_xlim(-75, 75); ax.set_ylim(-2, 50); ax.set_aspect('equal'); ax.axis('off')
ax.annotate('belly cone (contacts on its tip)', xy=(14, v['SEAT_DZ'] + P_['CONE_TIP_Z'] + 4), xytext=(30, 44), arrowprops=dict(arrowstyle='->'), fontsize=9)
ax.annotate('30° funnel: ±12 mm capture, any heading', xy=(-27, v['H'] - 1.5), xytext=(-74, 44), arrowprops=dict(arrowstyle='->'), fontsize=9)
ax.annotate('spring pin', xy=(0, v['H'] - P_['DEPTH'] - 2), xytext=(-60, 6), arrowprops=dict(arrowstyle='->'), fontsize=9)
ax.annotate('charger + USB-C PD pockets underneath', xy=(30, 3), xytext=(28, 14), arrowprops=dict(arrowstyle='->'), fontsize=9)
ax.set_title('Section through the seated robot and the home puck (y = 0)', fontsize=10)
fig.tight_layout(); fig.savefig(IMG + 'v2_puck_section.png'); plt.close(fig)

cone = v['cone']
ims = [render([(v['puck'], '#3a3f4a')], view=(90, -90), size=(800, 640)),
       render([(v['puck'], '#3a3f4a')], view=(-90, -90), size=(800, 640)),
       render([(cone, '#5a606b')], view=(-50, -60), size=(800, 640)),
       render([(v['flag'], '#3a3f4a')], view=(15, -60), size=(800, 640))]
grid(ims, 4, labels=['home puck - top (funnel, pin bores, flag socket)', 'home puck - underside (charger, PD trigger, feet)',
                     'belly cone - from below (disc + ring pads)', 'tag flag (50 mm AprilTag card)']).save(IMG + 'v2_parts.png')
