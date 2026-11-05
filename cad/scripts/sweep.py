import sys, os; sys.path.insert(0, os.path.dirname(__file__))
from common import *
from trimesh.collision import CollisionManager
HIP = {'FL':(-24.476,-25.384),'FR':(-24.477,25.416),'BR':(23.923,25.417),'BL':(23.924,-25.383)}
GRP = {'FL':['12','14','13'],'FR':['16','15','17'],'BR':['05','07','06'],'BL':['08','10','09']}
SG = {'FL':(-1,-1),'FR':(-1,1),'BR':(1,1),'BL':(1,-1)}
legs = {k: trimesh.util.concatenate([step(p) for p in v]) for k,v in GRP.items()}
def rotz(a, c):
    M = trimesh.transformations.rotation_matrix(np.radians(a),[0,0,1],[c[0],c[1],0]); return M
def placed(k, d, a):
    sx,sy = SG[k]; Tm = np.eye(4); Tm[:3,3]=[sx*d, sy*d, 0]
    h = np.array(HIP[k]) + [sx*d, sy*d]
    return rotz(a, h) @ Tm
def first_hit(k1, k2, d, sign1, sign2):
    # rotate both legs toward each other in 2-degree steps
    for a in range(0, 91, 2):
        cm = CollisionManager()
        cm.add_object('a', legs[k1], transform=placed(k1, d, sign1*a))
        hit = cm.in_collision_single(legs[k2], transform=placed(k2, d, sign2*a))
        if hit: return a
    return '>90'
# sign convention: positive angle = CCW. FL at (-,-): rotating toward BL (+x side) means CCW (+). BL toward FL: CW (-).
for d in [0, 6, 10, 14, 18, 22]:
    side = first_hit('FL','BL', d, +1, -1)
    front = first_hit('FL','FR', d, -1, +1)
    print(f"d={d:2d} hip radius={np.hypot(24.7+d,25.4+d):.1f}  same-side(FL<->BL) collide at ±{side}°   front(FL<->FR) collide at ±{front}°")
