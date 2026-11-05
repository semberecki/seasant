import sys,glob,os; sys.path.insert(0,os.path.dirname(__file__))
import trimesh
from zrender import render, grid
D=sys.argv[1]; fs=sorted(glob.glob(D+'/stl/*.stl'))
ims=[render([(trimesh.load(f),'#f28c28')],view=(35,-55),size=(500,400)) for f in fs]
grid(ims,4,labels=[os.path.basename(f)+' (as printed, bed = bottom)' for f in fs]).save(D+'/images/print_orientation.png')
