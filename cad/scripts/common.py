import numpy as np, trimesh, glob, os
T=os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
SCR=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
Y = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]], float)
C = np.eye(4); C[:3,3] = [-26.0, 2.4, 0]
TF = C @ Y          # Sesame STEP/STL frame (Y-up) -> SesAnt frame (Z-up, spine centred)
P = T+'/sesame-robot/hardware/printing/stl/'
R = T+'/realant/stl/'
ST = SCR+'/sesame_step/'
def L(p, tf=TF):
    m = trimesh.load(p, force='mesh'); m = m.copy()
    if tf is not None: m.apply_transform(tf)
    return m
def step(prefix):
    return L(glob.glob(ST+prefix+'_*.stl')[0])
def hsec(m, z):
    """horizontal section at height z -> list of shapely polygons in XY"""
    s = m.section(plane_origin=[0,0,z], plane_normal=[0,0,1])
    if s is None: return []
    M = np.eye(4); M[2,3] = -z
    p2, _ = s.to_2D(to_2D=M)
    return list(p2.polygons_full)
def psec(m, origin, normal, to2d):
    s = m.section(plane_origin=origin, plane_normal=normal)
    if s is None: return []
    p2, _ = s.to_2D(to_2D=to2d)
    return list(p2.polygons_full)
