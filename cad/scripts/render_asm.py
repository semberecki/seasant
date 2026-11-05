import sys, os, pickle; sys.path.insert(0, os.path.dirname(__file__))
from common import *
from zrender import render, grid
out = sys.argv[1]; d = pickle.load(open(out+'/asm.pkl','rb'))
items = [(m, c) for n, m, c, g in d['asm']]
views = [(22,-55),(90,-90),(0,-90),(0,0),(-30,-55),(22,125)]
labs = ['iso','top','front (face left)','side','below','iso back']
ims = [render(items, view=v, size=(760,600)) for v in views]
grid(ims, 3, labels=labs).save(out+'/asm_views.png')
ch = [(m,c) for n,m,c,g in d['asm'] if n=='chassis']
grid([render(ch, view=v, size=(700,560)) for v in [(35,-55),(90,-90),(-40,-55)]], 3).save(out+'/chassis_views.png')
