import sys, os, pickle, glob; sys.path.insert(0, os.path.dirname(__file__))
from common import *
from zrender import render, grid
from PIL import Image, ImageDraw
out, D = sys.argv[1], sys.argv[2]
d = pickle.load(open(out+'/asm.pkl','rb')); items=[(m,c) for n,m,c,g in d['asm']]
render(items, view=(22,-135), size=(1600,1150), ss=2).save(D+'/images/sesant_hero.png')
render(items, view=(24,45), size=(1200,860), ss=2).save(D+'/images/sesant_back.png')
# Sesame (stock) vs SesAnt at the same scale, top + front
ses=[]
for f in sorted(glob.glob(ST+'*.stl')):
    n=os.path.basename(f)
    if any(s in n for s in ['Screw','Phillips','KCD','Ovonic','57_']): continue
    c = '#2b2d33' if 'SG90' in n else '#f28c28' if ('femur' in n or 'foot' in n) else '#3a3f4a' if ('Internal' in n or 'Bottom' in n) else '#1f6feb' if 'OLED' in n or 'Display' in n else '#7bc96f'
    ses.append((L(f), c))
tc=L(P+'top-covers/Top-Cover-Enclosed-v117.stl'); ses.append((tc,'#f5a54a'))
allv=np.vstack([m.vertices for m,_ in items]); B=np.array([allv.min(0)-5, allv.max(0)+5])
ims=[render(ses,view=v,size=(700,560),bounds=B) for v in [(90,-90),(0,-90)]]+[render(items,view=v,size=(700,560),bounds=B) for v in [(90,-90),(0,-90)]]
g=grid(ims,2,labels=['Sesame (stock) - top','Sesame (stock) - side','SesAnt - top (same scale)','SesAnt - side (same scale)'])
g.save(D+'/images/sesame_vs_sesant.png')
