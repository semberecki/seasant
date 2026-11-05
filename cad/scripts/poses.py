"""Leg kinematics and robot posing on top of the v1 assembly (work/asm.pkl), shared by the v2 and v3 generators.

Robot frame = v1 build frame: Z up, front (face) at -X, origin at the spine centre, z = -1 is the belly (bottom cover).
CAD joint signs: hip + = counter-clockwise seen from above; knee + = foot swings inward, - = outward.
RealAnt/firmware knee units: 224 = foot vertical, + = outward, 0.293 deg per unit.
"""
import numpy as np, trimesh
from cadlib import rot, trans, rotz

SG = {'FL': (-1, -1), 'FR': (-1, 1), 'BR': (1, 1), 'BL': (1, -1)}
FT = {'FL': 'L3', 'FR': 'R3', 'BR': 'R4', 'BL': 'L4'}           # blade foot names
FEM = {'FL': 'L1', 'FR': 'R1', 'BR': 'R2', 'BL': 'L2'}          # femur names
AX_DEG = 300.0 / 1023.0


class Robot:
    def __init__(self, d, replace=None, extra=None):
        """d: dict from asm.pkl. replace: {part name: mesh}. extra: list of (name, mesh, colour) rigid with the body."""
        self.D = d['D']
        self.HIP = {k: np.array(list(d['HIP'][k]) + [0.0]) for k in SG}
        self.KW = {k: np.array(d['KNEE'][k]) + [SG[k][0] * self.D, SG[k][1] * self.D, 0] for k in SG}
        self.RV = {k: np.array([SG[k][0], SG[k][1], 0.0]) / np.sqrt(2) for k in SG}
        self.TV = {k: np.cross([0, 0, 1.0], self.RV[k]) for k in SG}
        replace = replace or {}
        self.parts = []          # (name, mesh, colour, leg or None, kind)  kind: body | femur | foot
        for n, m, c, g in d['asm']:
            if n in replace:
                m = replace[n]
                if m is None: continue
            leg, kind = None, 'body'
            for k in SG:
                if n == 'blade_foot_' + FT[k]: leg, kind = k, 'foot'
                if n in ('femur_' + FEM[k], 'knee_servo_' + k): leg, kind = k, 'femur'
            self.parts.append((n, m, c, leg, kind))
        for n, m, c in (extra or []):
            self.parts.append((n, m, c, None, 'body'))
        self.foot = {k: next(m for n, m, c, l, kd in self.parts if kd == 'foot' and l == k) for k in SG}

    # ---- leg transforms in the robot frame
    def M_hip(self, k, hip):
        return rotz(hip, self.HIP[k][:2])

    def M_foot(self, k, hip, knee):
        return self.M_hip(k, hip) @ rot(knee, self.TV[k], self.KW[k])

    def tip_pts(self, k, hip, knee, T=np.eye(4), band=0.6):
        v = trimesh.transform_points(self.foot[k].vertices, T @ self.M_foot(k, hip, knee))
        return v[v[:, 2] < v[:, 2].min() + band]

    def tip_z(self, k, hip, knee, T=np.eye(4)):
        return trimesh.transform_points(self.foot[k].vertices, T @ self.M_foot(k, hip, knee))[:, 2].min()

    def knee_for(self, k, hip, T, z_floor=0.0, lo=-118.0, hi=48.0):
        """knee angle (CAD) that puts this foot's lowest point on z_floor with the body at world pose T.
        Searches the outward branch (tip height rises as the knee opens outward)."""
        f = lambda a: self.tip_z(k, hip, a, T) - z_floor
        if -1e-3 < f(0.0) < 1.0: return 0.0          # standing height: this foot is on (or <1 mm above) the floor
        if f(0.0) > 0: hi_, lo_ = 0.0, hi            # body low: need inward (rarely used)
        else: lo_, hi_ = lo, 0.0
        a, b = lo_, hi_
        if f(a) * f(b) > 0: return None
        for _ in range(48):
            m = (a + b) / 2
            if f(a) * f(m) <= 0: b = m
            else: a = m
        return (a + b) / 2

    def pose(self, T=np.eye(4), hips=None, knees=None, only=None):
        """list of (name, world mesh, colour); only: optional predicate on the part name"""
        hips = hips or {k: 0.0 for k in SG}; knees = knees or {k: 0.0 for k in SG}
        out = []
        for n, m, c, l, kd in self.parts:
            if only is not None and not only(n): continue
            M = T if kd == 'body' else (T @ self.M_hip(l, hips[l]) if kd == 'femur' else T @ self.M_foot(l, hips[l], knees[l]))
            mm = m.copy(); mm.apply_transform(M); out.append((n, mm, c))
        return out

    def stand_on_floor(self, T_rot=np.eye(4), hips=None, knees=None):
        """lift a posed robot so its lowest foot point is at z = 0; returns the full world transform"""
        hips = hips or {k: 0.0 for k in SG}; knees = knees or {k: 0.0 for k in SG}
        zmin = min(self.tip_z(k, hips[k], knees[k], T_rot) for k in SG)
        return trans([0, 0, -zmin]) @ T_rot


def knee_units(cad_deg):
    """CAD knee angle -> RealAnt/firmware knee units (224 = vertical, + = outward)"""
    return int(round(224 + (-cad_deg) / AX_DEG))


def support_margin(com_xy, feet_xy):
    """signed distance (mm) of the CoM from the edge of the support polygon (convex hull of the feet); > 0 = inside"""
    from shapely.geometry import MultiPoint, Point
    hull = MultiPoint([tuple(p) for p in feet_xy]).convex_hull
    p = Point(com_xy[0], com_xy[1])
    d = hull.exterior.distance(p)
    return d if hull.contains(p) else -d
