"""Software z-buffer renderer for STL assemblies (numpy only).

render(items, view=(elev, azim), size=(W,H), ...) -> PIL.Image
items: list of (trimesh.Trimesh, color) ; color is hex or rgb tuple.
Coordinates: Z is up. Orthographic camera.
"""
import numpy as np
from PIL import Image, ImageFilter
import matplotlib.colors as mcolors


def _rot(elev, azim):
    # camera looks from direction given by azimuth (around Z) and elevation
    e, a = np.radians(elev), np.radians(azim)
    # view direction (from camera toward origin)
    d = -np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    up = np.array([0, 0, 1.0])
    if abs(np.dot(d, up)) > 0.999:
        up = np.array([0, 1.0, 0]) if elev < 0 else np.array([0, 1.0, 0])
    right = np.cross(d, up); right /= np.linalg.norm(right)
    upv = np.cross(right, d)
    # rows: x_screen, y_screen, depth (larger = farther)
    return np.vstack([right, upv, d])


def render(items, view=(25, -60), size=(1200, 900), ss=2, bg=(255, 255, 255),
           bounds=None, margin=0.06, outline=True, light=None, persp=None):
    W, H = size[0] * ss, size[1] * ss
    R = _rot(*view)
    allv = np.vstack([m.vertices for m, _ in items])
    cam_all = allv @ R.T
    if bounds is None:
        lo, hi = cam_all.min(0), cam_all.max(0)
    else:
        corners = np.array([[x, y, z] for x in bounds[:, 0] for y in bounds[:, 1] for z in bounds[:, 2]])
        cc = corners @ R.T
        lo, hi = cc.min(0), cc.max(0)
    span = max((hi[0] - lo[0]) / W, (hi[1] - lo[1]) / H) * (1 + 2 * margin)
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    zbuf = np.full((H, W), np.inf)
    cbuf = np.zeros((H, W, 3))
    nbuf = np.zeros((H, W, 3))
    idbuf = np.full((H, W), -1, dtype=np.int32)
    if light is None:
        light = np.array([-0.35, -0.55, 0.75])
    light = light / np.linalg.norm(light)
    light2 = np.array([0.6, 0.3, 0.2]); light2 /= np.linalg.norm(light2)
    viewdir = R[2]
    for mid, (m, col) in enumerate(items):
        base = np.array(mcolors.to_rgb(col)) if isinstance(col, str) else np.array(col, float)
        v = m.vertices @ R.T
        px = (v[:, 0] - cx) / span + W / 2
        py = H / 2 - (v[:, 1] - cy) / span
        pz = v[:, 2]
        f = m.faces
        fn = m.face_normals
        # two-sided lighting
        ndl = np.abs(fn @ light)
        ndl2 = np.clip(fn @ light2, 0, 1)
        facing = np.sign(-(fn @ viewdir))
        nview = fn * facing[:, None]
        h = light - viewdir; h /= np.linalg.norm(h)
        spec = np.clip(nview @ h, 0, 1) ** 40
        shade = 0.28 + 0.62 * ndl + 0.18 * ndl2
        fcol = np.clip(base[None, :] * shade[:, None] + 0.25 * spec[:, None], 0, 1)
        X = px[f]; Y = py[f]; Z = pz[f]
        xmin = np.floor(X.min(1)).astype(int); xmax = np.ceil(X.max(1)).astype(int)
        ymin = np.floor(Y.min(1)).astype(int); ymax = np.ceil(Y.max(1)).astype(int)
        ok = (xmax >= 0) & (xmin < W) & (ymax >= 0) & (ymin < H)
        bw = xmax - xmin + 1; bh = ymax - ymin + 1
        # group by bbox size for vectorization
        for cap in (4, 8, 16, 32, 64, 128, 256, 100000):
            sel = np.where(ok & (np.maximum(bw, bh) <= cap) & (np.maximum(bw, bh) > cap // 2 if cap > 4 else True))[0]
            if cap == 100000:
                sel = np.where(ok & (np.maximum(bw, bh) > 256))[0]
            if len(sel) == 0:
                continue
            if cap <= 64:
                # vectorized over triangles, chunked
                g = np.arange(cap)
                chunk = max(1, 400000 // (cap * cap))
                for s0 in range(0, len(sel), chunk):
                    idx = sel[s0:s0 + chunk]
                    gx = xmin[idx, None, None] + g[None, None, :]
                    gy = ymin[idx, None, None] + g[None, :, None]
                    gx = np.broadcast_to(gx, (len(idx), cap, cap)); gy = np.broadcast_to(gy, (len(idx), cap, cap))
                    _raster(idx, gx + 0.5, gy + 0.5, gx, gy, X, Y, Z, fcol, nview, mid, zbuf, cbuf, nbuf, idbuf, W, H)
            else:
                for t in sel:
                    xs = np.arange(max(xmin[t], 0), min(xmax[t], W - 1) + 1)
                    ys = np.arange(max(ymin[t], 0), min(ymax[t], H - 1) + 1)
                    if len(xs) == 0 or len(ys) == 0:
                        continue
                    gx, gy = np.meshgrid(xs, ys)
                    _raster(np.array([t]), gx[None] + 0.5, gy[None] + 0.5, gx[None], gy[None], X, Y, Z, fcol, nview, mid, zbuf, cbuf, nbuf, idbuf, W, H)
    img = np.empty((H, W, 3)); img[:] = np.array(bg) / 255.0
    hit = np.isfinite(zbuf)
    img[hit] = cbuf[hit]
    if outline:
        # edges from id and depth/normal discontinuities
        e = np.zeros((H, W), bool)
        zf = np.where(hit, zbuf, 1e9)
        dz_tol = span * 3.0
        for sh in ((0, 1), (1, 0)):
            a = np.s_[:-sh[0] or None, :-sh[1] or None]
            b = np.s_[sh[0]:, sh[1]:]
            d_id = idbuf[a] != idbuf[b]
            d_z = np.abs(zf[a] - zf[b]) > dz_tol
            d_n = np.sum(nbuf[a] * nbuf[b], axis=-1) < 0.6
            ed = d_id | d_z | (d_n & hit[a] & hit[b])
            e[a] |= ed
        e_img = Image.fromarray((e * 255).astype(np.uint8)).filter(ImageFilter.MaxFilter(3 if ss >= 2 else 1))
        e = np.array(e_img) > 0
        img[e] = img[e] * 0.25
    out = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
    if ss > 1:
        out = out.resize(size, Image.LANCZOS)
    return out


def _raster(idx, sx, sy, gx, gy, X, Y, Z, fcol, nview, mid, zbuf, cbuf, nbuf, idbuf, W, H):
    x0 = X[idx, 0][:, None, None]; x1 = X[idx, 1][:, None, None]; x2 = X[idx, 2][:, None, None]
    y0 = Y[idx, 0][:, None, None]; y1 = Y[idx, 1][:, None, None]; y2 = Y[idx, 2][:, None, None]
    den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
    den = np.where(np.abs(den) < 1e-12, 1e-12, den)
    w0 = ((y1 - y2) * (sx - x2) + (x2 - x1) * (sy - y2)) / den
    w1 = ((y2 - y0) * (sx - x2) + (x0 - x2) * (sy - y2)) / den
    w2 = 1 - w0 - w1
    inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6) & (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
    if not inside.any():
        return
    z = w0 * Z[idx, 0][:, None, None] + w1 * Z[idx, 1][:, None, None] + w2 * Z[idx, 2][:, None, None]
    ti, ii, jj = np.nonzero(inside)
    zz = z[ti, ii, jj]; xx = gx[ti, ii, jj]; yy = gy[ti, ii, jj]
    tri = idx[ti]
    # resolve multiple writes to same pixel: sort by depth descending so nearest written last
    order = np.argsort(-zz, kind='stable')
    xx, yy, zz, tri = xx[order], yy[order], zz[order], tri[order]
    cur = zbuf[yy, xx]
    m = zz < cur
    xx, yy, zz, tri = xx[m], yy[m], zz[m], tri[m]
    zbuf[yy, xx] = zz  # last (nearest) wins for duplicates because sorted far->near
    # write colors only where we are the final value
    win = zbuf[yy, xx] == zz
    xx, yy, tri = xx[win], yy[win], tri[win]
    cbuf[yy, xx] = fcol[tri]
    nbuf[yy, xx] = nview[tri]
    idbuf[yy, xx] = mid


def grid(images, cols, pad=10, bg=(255, 255, 255), labels=None):
    from PIL import ImageDraw
    w = max(i.width for i in images); h = max(i.height for i in images)
    rows = (len(images) + cols - 1) // cols
    out = Image.new('RGB', (cols * w + (cols + 1) * pad, rows * h + (rows + 1) * pad), bg)
    d = ImageDraw.Draw(out)
    for k, im in enumerate(images):
        r, c = divmod(k, cols)
        x, y = pad + c * (w + pad), pad + r * (h + pad)
        out.paste(im, (x, y))
        if labels:
            d.text((x + 8, y + 6), labels[k], fill=(60, 60, 60))
    return out
