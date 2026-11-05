import sys, os, pickle, json, shutil; sys.path.insert(0, os.path.dirname(__file__))
from common import *
out=sys.argv[1]; dst=sys.argv[2]
d=pickle.load(open(out+'/asm.pkl','rb')); A={n:(m,c,g) for n,m,c,g in d['asm']}
os.makedirs(dst+'/stl', exist_ok=True); os.makedirs(dst+'/assembly', exist_ok=True)
def save(m, name, M=None):
    m=m.copy()
    if M is not None: m.apply_transform(M)
    m.apply_translation([-(m.bounds[0,0]+m.bounds[1,0])/2, -(m.bounds[0,1]+m.bounds[1,1])/2, -m.bounds[0,2]])
    assert m.is_watertight, name
    m.export(f'{dst}/stl/{name}.stl'); return m
flipX=np.diag([1,-1,-1,1.0])
info={}
info['sesant_chassis']=save(A['chassis'][0],'sesant_chassis',flipX)
info['sesant_bottom_cover']=save(A['bottom_cover'][0],'sesant_bottom_cover')
info['sesant_stackchan_head']=save(A['stackchan_head'][0],'sesant_stackchan_head')
YtoZ=np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1.0]])
for k,(m,L2W,fname) in d['feet_local'].items():
    info[f'sesant_blade_foot_{fname}']=save(m,f'sesant_blade_foot_{fname}',YtoZ)
for n,m in info.items():
    e=m.extents; print(f'{n:32s} {e[0]:6.1f} x {e[1]:6.1f} x {e[2]:5.1f} mm  {m.volume/1000:5.1f} cm3  ~{m.volume/1000*1.24*0.45:4.1f} g @~45% fill')
# assembly scene (Z-up, mm), with names + colours
sc=trimesh.Scene()
for n,(m,c,g) in A.items():
    mm=m.copy(); rgb=(np.array(trimesh.visual.color.hex_to_rgba(c))).astype(np.uint8); mm.visual.face_colors=rgb
    sc.add_geometry(mm, node_name=n, geom_name=n)
sc.export(dst+'/assembly/sesant_assembly.glb')
json.dump({'hip_axes_mm':{k:[round(float(x),2) for x in v] for k,v in d['HIP'].items()},
           'knee_axes_mm':{k:[round(float(x)+ (s*d['D'] if i<2 else 0),2) for i,(x,s) in enumerate(zip(v,{'FL':(-1,-1,0),'FR':(-1,1,0),'BR':(1,1,0),'BL':(1,-1,0)}[k]))] for k,v in d['KNEE'].items()},
           'femur_length_mm':36.8, 'hip_plane_z_mm':21.97, 'ground_z_standing_mm':-50.8},
          open(dst+'/assembly/sesant_geometry.json','w'), indent=1)
