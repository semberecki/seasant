"""Minimal reader for the Jumper URDF (../../jumper/assets/jumper/urdf/jumper): visual meshes + forward kinematics.

Jumper © 2026 KingKong Robotics, Apache-2.0. Only the claw outline is used (see v4_claw.py).
Units: the URDF is in metres; meshes() returns millimetres.
"""
import os, numpy as np, trimesh, xml.etree.ElementTree as ET
from common import T as WORKSPACE

URDF = os.path.join(WORKSPACE, 'jumper', 'assets', 'jumper', 'urdf', 'jumper', 'urdf', 'jumper.urdf')


def _tf(o):
    xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()] if o is not None else [0, 0, 0]
    rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()] if o is not None else [0, 0, 0]
    M = trimesh.transformations.euler_matrix(*rpy, 'sxyz'); M[:3, 3] = xyz
    return M


class JumperURDF:
    def __init__(self, path=URDF):
        root = ET.parse(path).getroot()
        self.links = {}
        for l in root.findall('link'):
            v = l.find('visual')
            if v is None:
                self.links[l.get('name')] = None; continue
            fn = os.path.join(os.path.dirname(path), '..', 'meshes', 'visual', os.path.basename(v.find('geometry/mesh').get('filename')))
            m = trimesh.load(fn, force='mesh')
            sc = v.find('geometry/mesh').get('scale')
            if sc: m.apply_scale([float(s) for s in sc.split()])
            m.apply_transform(_tf(v.find('origin')))
            self.links[l.get('name')] = m
        self.joints = {}
        for j in root.findall('joint'):
            ax = j.find('axis'); lim = j.find('limit')
            self.joints[j.get('name')] = dict(
                type=j.get('type'), parent=j.find('parent').get('link'), child=j.find('child').get('link'), M=_tf(j.find('origin')),
                axis=np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else None,
                lim=(float(lim.get('lower')), float(lim.get('upper'))) if lim is not None else None)

    def fk(self, q=None):
        q = q or {}
        W = {'base_link': np.eye(4)}
        changed = True
        while changed:
            changed = False
            for n, j in self.joints.items():
                if j['parent'] in W and j['child'] not in W:
                    M = W[j['parent']] @ j['M']
                    if j['type'] == 'revolute':
                        M = M @ trimesh.transformations.rotation_matrix(q.get(n, 0.0), j['axis'])
                    W[j['child']] = M; changed = True
        return W

    def meshes(self, q=None, frame=None, names=None):
        """[(link, mesh in mm)] in the base frame, or in the frame of link `frame`"""
        W = self.fk(q)
        F = np.linalg.inv(W[frame]) if frame else np.eye(4)
        out = []
        for n, m in self.links.items():
            if m is None or (names and not any(n.endswith(s) or n == s for s in names)): continue
            mm = m.copy(); mm.apply_transform(F @ W[n]); mm.apply_scale(1000.0); out.append((n, mm))
        return out
