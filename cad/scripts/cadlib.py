"""Shared geometry helpers for the v2 (dock) and v3 (SesAnt-XL) generators.
v1's build.py keeps its own copies so the v1 parts stay byte-for-byte reproducible."""
import numpy as np, trimesh
import manifold3d as mf
from shapely.geometry import Polygon, MultiPolygon, box, Point
from shapely import affinity
from shapely.ops import unary_union


def to_mf(m):
    m = m.copy(); m.merge_vertices(); m.remove_unreferenced_vertices()
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))


def to_tm(x):
    mm = x.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mm.vert_properties)[:, :3], faces=np.asarray(mm.tri_verts), process=True)


def cs_from(poly):
    polys = list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]
    contours = []
    for p in polys:
        contours.append(np.asarray(p.exterior.coords)[:-1])
        for h in p.interiors: contours.append(np.asarray(h.coords)[:-1])
    return mf.CrossSection(contours, mf.FillRule.EvenOdd)


def extrude(poly, z0, z1):
    return cs_from(poly).extrude(z1 - z0).translate([0, 0, z0])


def xform(x, M):
    return x.transform(np.asarray(M[:3, :4], dtype=np.float64))


def trans(v):
    M = np.eye(4); M[:3, 3] = v; return M


def rotz(a_deg, c=(0, 0)):
    return trimesh.transformations.rotation_matrix(np.radians(a_deg), [0, 0, 1], [c[0], c[1], 0])


def rot(a_deg, axis, point=(0, 0, 0)):
    return trimesh.transformations.rotation_matrix(np.radians(a_deg), axis, point)


def box3(a0, a1, b0, b1, c0, c1):
    return mf.Manifold.cube([a1 - a0, b1 - b0, c1 - c0]).translate([a0, b0, c0])


def cyl(x, y, r, z0, z1, n=48):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])


def cone(x, y, r0, r1, z0, z1, n=48):
    return mf.Manifold.cylinder(z1 - z0, r0, r1, n).translate([x, y, z0])


def rrect(u0, u1, v0, v1, r):
    return box(u0 + r, v0 + r, u1 - r, v1 - r).buffer(r, resolution=16)


def chamfered_slab(poly, z0, z1, ch, steps=10):
    """extrude poly from z0 to z1 with a stepped 45-degree chamfer of size ch on the top edge (prints without supports)"""
    body = extrude(poly, z0, z1 - ch)
    for i in range(steps):
        a = z1 - ch + ch * i / steps; b = z1 - ch + ch * (i + 1) / steps
        p = poly.buffer(-ch * (i + 1) / steps, join_style=1)
        if p.is_empty: break
        body += extrude(p, a, b)
    return body


def prism_along(profile_uv, length, M):
    """extrude a 2D profile given in (u, v) along +w by `length`, then place with 4x4 M (local u,v,w -> world)"""
    return xform(cs_from(profile_uv).extrude(length), M)


def frame(origin, u, v, w):
    M = np.eye(4); M[:3, 0] = u; M[:3, 1] = v; M[:3, 2] = w; M[:3, 3] = origin; return M


def color_mesh(m, c):
    m = m.copy(); m.visual.face_colors = np.array(trimesh.visual.color.hex_to_rgba(c), np.uint8); return m


def save_stl(m, path, M=None):
    """place on the bed (min z = 0, centred in XY) and export; asserts watertight"""
    m = m.copy()
    if M is not None: m.apply_transform(M)
    m.apply_translation([-(m.bounds[0, 0] + m.bounds[1, 0]) / 2, -(m.bounds[0, 1] + m.bounds[1, 1]) / 2, -m.bounds[0, 2]])
    assert m.is_watertight, path
    m.export(path); return m
