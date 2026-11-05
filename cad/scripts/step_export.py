import sys, re, os
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
import cadquery as cq
doc = TDocStd_Document(TCollection_ExtendedString("doc"))
r = STEPCAFControl_Reader(); r.SetNameMode(True)
assert r.ReadFile(sys.argv[1]) == IFSelect_RetDone
r.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
def name(l):
    n = TDataStd_Name()
    return n.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), n) else '?'
out = sys.argv[2]
roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
root = roots.Value(1)
ch = TDF_LabelSequence(); st.GetComponents_s(root, ch)
for i in range(1, ch.Length()+1):
    l = ch.Value(i); nm = name(l)
    s = st.GetShape_s(l)
    fn = f"{i:02d}_" + re.sub(r'[^A-Za-z0-9]+', '_', nm)[:60] + '.stl'
    try:
        cq.exporters.export(cq.Shape.cast(s), os.path.join(out, fn), tolerance=0.05, angularTolerance=0.2)
        print('ok', fn)
    except Exception as e:
        print('fail', fn, e)
