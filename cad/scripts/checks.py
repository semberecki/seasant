import sys, os, pickle; sys.path.insert(0, os.path.dirname(__file__))
from common import *
from trimesh.collision import CollisionManager
out=sys.argv[1]; d=pickle.load(open(out+'/asm.pkl','rb')); A={n:m for n,m,c,g in d['asm']}
HIP=d['HIP']; KNEE=d['KNEE']; D=d['D']
SG={'FL':(-1,-1),'FR':(-1,1),'BR':(1,1),'BL':(1,-1)}
FEM={'FL':'L1','FR':'R1','BR':'R2','BL':'L2'}; FT={'FL':'L3','FR':'R3','BR':'R4','BL':'L4'}
body = CollisionManager()
for n in ['chassis','stackchan_head','bottom_cover','cores3']: body.add_object(n, A[n])
for k in HIP: body.add_object('hs'+k, A['hip_servo_'+k])
def R(a,c): return trimesh.transformations.rotation_matrix(np.radians(a),[0,0,1],[c[0],c[1],0])
print('== hip sweep: leg (femur+knee servo+blade foot) vs chassis/pods/StackChan head+CoreS3/bottom cover ==')
for k in HIP:
    leg = trimesh.util.concatenate([A['femur_'+FEM[k]], A['knee_servo_'+k], A['blade_foot_'+FT[k]]])
    bad=[]
    for a in range(-90,91,3):
        hit, names = body.in_collision_single(leg, transform=R(a,HIP[k]), return_names=True)
        names = {n for n in names if n!='hs'+k}   # own hip servo touches femur at the horn by design
        if names: bad.append((a, sorted(names)))
    print(k, 'collision-free over -90..+90' if not bad else f'hits at {bad[:3]}... ({len(bad)} angles)')
print('== knee sweep: blade foot vs femur & body (knee axis local) ==')
for k in HIP:
    K=np.array(KNEE[k])+[SG[k][0]*D, SG[k][1]*D, 0]; r=np.array([SG[k][0],SG[k][1],0])/np.sqrt(2); t=np.cross([0,0,1],r)
    cm=CollisionManager(); cm.add_object('femur', A['femur_'+FEM[k]])
    for n in ['chassis','stackchan_head','bottom_cover','cores3']: cm.add_object(n, A[n])
    ok=[]
    for a in range(-120,121,5):
        M=trimesh.transformations.rotation_matrix(np.radians(a), t, K)
        hit, names = cm.in_collision_single(A['blade_foot_'+FT[k]], transform=M, return_names=True)
        if not hit: ok.append(a)
    # contiguous range containing 0
    rng=[a for a in ok]; lo=0; hi=0
    while lo-5 in ok: lo-=5
    while hi+5 in ok: hi+=5
    print(k, f'foot free knee range {lo}..{hi} deg (- = foot swings outward/up, + = inward under body)' )
feet=[A['blade_foot_'+FT[k]] for k in HIP]
gz=min(f.bounds[0,2] for f in feet); bz=min(A[n].bounds[0,2] for n in ['chassis','bottom_cover'])
print('foot tip z %.1f, body bottom z %.1f -> ground clearance %.1f mm'%(gz,bz,bz-gz))
allm=trimesh.util.concatenate([m for n,m,c,g in d['asm']])
print('overall footprint', (allm.bounds[1]-allm.bounds[0]).round(1), 'standing height', round(allm.bounds[1,2]-gz,1))
print('== leg vs neighbour leg (both rotating toward each other, blade feet) ==')
LEGM={k: trimesh.util.concatenate([A['femur_'+FEM[k]], A['knee_servo_'+k], A['blade_foot_'+FT[k]]]) for k in HIP}
def first_hit(k1,k2,s1,s2):
    for a in range(0,91,3):
        cm=CollisionManager(); cm.add_object('a',LEGM[k1],transform=R(s1*a,HIP[k1]))
        if cm.in_collision_single(LEGM[k2],transform=R(s2*a,HIP[k2])): return a
    return None
for k1,k2,s1,s2 in [('FL','BL',1,-1),('FR','BR',-1,1),('FL','FR',-1,1),('BL','BR',1,-1)]:
    h=first_hit(k1,k2,s1,s2); print(k1,k2,'no contact up to 90 deg' if h is None else f'contact at {h} deg')
print('== StackChan head static fit ==')
cm=CollisionManager(); cm.add_object('head',A['stackchan_head']); cm.add_object('chassis',A['chassis'])
print('CoreS3 vs head/chassis overlap:', cm.in_collision_single(A['cores3']), '(must be False)')
cm2=CollisionManager(); cm2.add_object('head',A['stackchan_head'])
for n in ['s2mini','battery']: print(n,'inside head clear:', not cm2.in_collision_single(A[n]))
cm3=CollisionManager(); cm3.add_object('chassis',A['chassis'])
print('head vs chassis: touching seat only ->', 'contact' if cm3.in_collision_single(A['stackchan_head']) else 'no contact', '; head lowest z %.2f (spine top 21.97)'%A['stackchan_head'].bounds[0,2])
