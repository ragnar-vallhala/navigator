#!/usr/bin/env python3
"""Offline vegetation asset pipeline for the navigator simulator.

Regeneration tool only: the application build never runs this and never
downloads anything; the processed output is committed under
navigator/assets/vegetation/.

Usage (python3 -I build_vegetation.py RAW_DIR OUT_DIR [--only ID ...])

  RAW_DIR  directory holding one sub-directory per Poly Haven model id
           (<id>/<id>_1k.gltf + .bin + textures/ + info_api.json)
  OUT_DIR  receives <species>/*.vmesh, *.jpg, *.png and manifest.json

Dependencies: numpy and Pillow only.  The inputs are treated as untrusted
data: the glTF is parsed (never executed), buffer ranges and URIs are
validated, and no external tool (gltfpack, meshoptimizer, ...) is used.
The output is deterministic (fixed seeds, no timestamps).

Frames and colour spaces (also documented in the manifest)
  * Geometry stays in the glTF frame: right handed, +Y up, metres.  Each
    variant is translated so its base sits at y = 0 and its pivot is the
    trunk/base centre in x/z.  The renderer converts to the sim NED frame.
  * UVs are the glTF convention: (0,0) is the TOP-left of the image, v grows
    downwards.  Upload the decoded QImage unflipped and sample with these UVs.
    Materials with tiling textures keep UVs outside 0..1 (wrap = repeat).
  * *_diff.jpg  : sRGB albedo (decode as sRGB / apply GL_SRGB8).
  * *_nor.jpg   : LINEAR tangent-space normal map, OpenGL convention (+Y up,
                  green = up), x/y/z stored as 0.5 + 0.5 * n.  Not sRGB.
  * *_arm.jpg   : LINEAR data: R = ambient occlusion, G = roughness,
                  B = 0 (the source metalness is 0 everywhere).  Not sRGB.
  * *_alpha.png : LINEAR 8-bit coverage mask (see "Cut-outs" below).

Cut-outs: the Poly Haven glTFs declare alphaMode MASK (or none/BLEND) but ship
3-channel baseColor JPGs and no opacity map: the leaf/needle outline is real
geometry.  Decimating that geometry would lose the outline, so the pipeline
rasterises the UV footprint of ALL original triangles of a cut-out material
into an 8-bit mask (the *_alpha.png) and the reduced cards are alpha-tested
with it.  The colour textures get their background flooded with edge colour
so mip-mapping does not bleed black into the leaves.

Mesh container (.vmesh, little endian)
  off  size  field
    0     4  magic "VMSH"
    4     4  u32 version (=1)
    8     4  u32 vertexCount
   12     4  u32 indexCount (multiple of 3, CCW triangles)
   16     4  u32 submeshCount
   20     4  u32 indexBytes (2 = uint16, 4 = uint32)
   24    24  f32 bbox min xyz, bbox max xyz
   48  12*S  per submesh: u32 firstIndex, u32 indexCount, u32 materialIndex
    .  32*V  interleaved f32 position(3) normal(3) uv(2)
    .  I*b   indices (indexBytes each), index values are absolute vertex ids
materialIndex refers to the species' "materials" array in manifest.json.
"""
import argparse
import json
import os
import struct
import sys
import zlib

import numpy as np
from PIL import Image, ImageDraw

JPEG_QUALITY = 85
ALPHA_FULL_COVERAGE = 0.97   # above this a mask carries no information
LODS = 3

# ---------------------------------------------------------------------------
# Selection and budgets
# ---------------------------------------------------------------------------


def lod_budgets(total):
    return lambda orig, lod: total[lod]


def grass_budget(orig, lod):
    if lod == 0:
        return orig
    if lod == 1:
        return max(8, min(300, orig // 2))
    return max(4, min(60, orig // 4))


SMALL = (3000, 800, 150)
# Full-size trees (8-20 m): the crown is the whole point, so LOD 0 keeps enough
# cards to read as a dense crown from a drone at 20 m, and the far LODs stay
# cheap because the billboard impostor takes over beyond them.
BIG_CONIFER = (45000, 9000, 1400)
BIG_BROADLEAF = (45000, 8000, 1200)

SPECIES = [
    dict(id='fir_tree_01', gltf='fir_tree_01_1k.gltf', category='tree',
         title='Fir tree 01 (conifer)',
         # variant c is not shipped: its trunk mesh has no UV coordinates
         variants=['fir_tree_01_a_LOD0', 'fir_tree_01_b_LOD0'],
         budget=lod_budgets(BIG_CONIFER), pivot='base',
         mats={'fir_tree_01_bark': dict(weight=0.05, cutout=False, cull=False, size=512),
               'fir_tree_01_trunk_a': dict(weight=0.03, cutout=False, cull=False, size=512),
               'fir_tree_01_trunk_b': dict(weight=0.03, cutout=False, cull=False, size=512),
               'fir_tree_01_twig': dict(weight=0.86, cutout=True, cull=True, size=1024,
                                        scale_cap=(8.0, 11.0, 16.0), tris_per_comp=4)}),
    dict(id='pine_sapling_medium', gltf='pine_sapling_medium_1k.gltf', category='tree',
         title='Pine (medium, conifer)',
         variants=['pine_sapling_medium_a_LOD0', 'pine_sapling_medium_b_LOD0',
                   'pine_sapling_medium_c_LOD0'],
         budget=lod_budgets(BIG_CONIFER), pivot='base',
         mats={'pine_sapling_medium_bark': dict(weight=0.08, cutout=False, cull=False, size=512),
               'pine_sapling_medium_twig': dict(weight=0.92, cutout=True, cull=True, size=1024,
                                                scale_cap=(6.0, 8.0, 12.0), tris_per_comp=4)}),
    dict(id='jacaranda_tree', gltf='jacaranda_tree_1k.gltf', category='tree',
         title='Jacaranda tree (broadleaf)', variants=['jacaranda_tree_LOD0'], pivot='base',
         budget=lod_budgets(BIG_BROADLEAF),
         mats={'jacaranda_tree_trunk': dict(weight=0.08, cutout=False, cull=False, size=512),
               'jacaranda_tree_leaves': dict(weight=0.82, cutout=True, key_black=True, cull=True,
                                             size=1024, scale_cap=(3.5, 4.5, 6.0), tris_per_comp=2),
               'jacaranda_tree_branches': dict(weight=0.10, cutout=False, cull=True, size=512,
                                               scale_cap=(2.0, 2.5, 3.0), tris_per_comp=12)}),
    dict(id='island_tree_01', gltf='island_tree_01_1k.gltf', category='tree',
         title='Island tree 01 (broadleaf)', variants=['island_tree_01_LOD0'], pivot='base',
         budget=lod_budgets(BIG_BROADLEAF),
         mats={'island_tree_01': dict(weight=0.10, cutout=False, cull=False, size=512),
               'island_tree_01_leaves': dict(weight=0.78, cutout=True, key_black=True, cull=True,
                                             size=1024, scale_cap=(3.5, 4.5, 6.0), tris_per_comp=2),
               'island_tree_01_branches': dict(weight=0.12, cutout=False, cull=True, size=512,
                                               scale_cap=(2.0, 2.5, 3.0), tris_per_comp=12)}),
    dict(id='shrub_03', gltf='shrub_03_1k.gltf', category='shrub', title='Shrub 03',
         variants=['shrub_03_a', 'shrub_03_b', 'shrub_03_c', 'shrub_03_d'], pivot='bbox',
         budget=lod_budgets(SMALL),
         mats={'shrub_03': dict(weight=1.0, cutout=True, cull=True, size=1024,
                                scale_cap=(2.0, 3.0, 4.0), tris_per_comp=4)}),
    dict(id='shrub_04', gltf='shrub_04_1k.gltf', category='shrub', title='Shrub 04',
         variants=['shrub_04'], pivot='bbox', budget=lod_budgets(SMALL),
         mats={'shrub_04': dict(weight=1.0, cutout=True, cull=True, size=1024,
                                scale_cap=(2.0, 3.0, 4.0), tris_per_comp=4)}),
    dict(id='weed_plant_02', gltf='weed_plant_02_1k.gltf', category='ground_plant',
         title='Weed plant 02', variants=['weed_plant_02_a_LOD0', 'weed_plant_02_b_LOD0',
                                          'weed_plant_02_c_LOD0', 'weed_plant_02_d_LOD0',
                                          'weed_plant_02_e_LOD0'], pivot='bbox',
         budget=lod_budgets(SMALL),
         mats={'weed_plant_02': dict(weight=1.0, cutout=True, key_black=True, cull=True, size=512,
                                     scale_cap=(2.0, 3.0, 4.0), tris_per_comp=4)}),
    dict(id='grass_bermuda_01', gltf='grass_bermuda_01_1k.gltf', category='ground_plant',
         title='Bermuda grass clumps',
         variants=['grass_bermuda_01_medium_c', 'grass_bermuda_01_medium_e',
                   'grass_bermuda_01_medium_f', 'grass_bermuda_01_seedling_d',
                   'grass_bermuda_01_seedling_a', 'grass_bermuda_01_small_c'],
         pivot='bbox', budget=grass_budget,
         mats={'grass_bermuda_01': dict(weight=1.0, cutout=True, key_black=True, cull=True, size=512,
                                        scale_cap=(1.5, 2.0, 3.0), tris_per_comp=4)}),
    dict(id='rock_09', gltf='rock_09_1k.gltf', category='rock', title='Rock 09',
         variants=['rock_09_LOD0'], pivot='bbox', budget=lod_budgets(SMALL),
         mats={'rock_09': dict(weight=1.0, cutout=False, cull=False, size=1024)}),
    dict(id='rock_07', gltf='rock_07_1k.gltf', category='rock', title='Rock 07',
         variants=['rock_07'], pivot='bbox', budget=lod_budgets(SMALL),
         mats={'rock_07': dict(weight=1.0, cutout=False, cull=False, size=1024)}),
    dict(id='namaqualand_boulders_01', gltf='namaqualand_boulders_01_1k.gltf', category='rock',
         title='Namaqualand boulders 01',
         variants=['namaqualand_rocks_01_a', 'namaqualand_rocks_01_b'], pivot='bbox',
         budget=lod_budgets(SMALL),
         mats={'namaqualand_rocks_01': dict(weight=1.0, cutout=False, cull=False, size=1024)}),
]

# ---------------------------------------------------------------------------
# glTF loading (parse only)
# ---------------------------------------------------------------------------

_CT = {5126: '<f4', 5123: '<u2', 5125: '<u4'}
_NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3}


def safe_join(base, rel):
    p = os.path.realpath(os.path.join(base, rel))
    if not p.startswith(os.path.realpath(base) + os.sep):
        raise ValueError('path escapes model directory: ' + rel)
    return p


class Gltf:
    def __init__(self, directory, name):
        self.dir = directory
        with open(safe_join(directory, name), 'r', encoding='utf8') as f:
            self.j = json.load(f)
        j = self.j
        if len(j['buffers']) != 1:
            raise ValueError('expected a single buffer')
        with open(safe_join(directory, j['buffers'][0]['uri']), 'rb') as f:
            self.buf = f.read()
        if len(self.buf) < j['buffers'][0]['byteLength']:
            raise ValueError('buffer shorter than declared')

    def accessor(self, i):
        a = self.j['accessors'][i]
        if 'sparse' in a:
            raise ValueError('sparse accessors unsupported')
        bv = self.j['bufferViews'][a['bufferView']]
        if bv.get('byteStride'):
            raise ValueError('strided views unsupported')
        nc = _NC[a['type']]
        dt = np.dtype(_CT[a['componentType']])
        off = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        n = a['count'] * nc
        if off < 0 or off + n * dt.itemsize > len(self.buf):
            raise ValueError('accessor out of range')
        arr = np.frombuffer(self.buf, dtype=dt, count=n, offset=off)
        arr = arr.astype(np.float64 if dt.kind == 'f' else np.int64)
        return arr.reshape(a['count'], nc) if nc > 1 else arr

    def material_info(self, mi):
        m = self.j['materials'][mi]
        pbr = m['pbrMetallicRoughness']

        def uri(texinfo):
            t = self.j['textures'][texinfo['index']]
            return self.j['images'][t['source']]['uri']

        xf = None
        ext = pbr['baseColorTexture'].get('extensions', {}).get('KHR_texture_transform')
        if ext:
            if ext.get('rotation', 0) != 0:
                raise ValueError('texture rotation unsupported')
            xf = (np.array(ext.get('scale', [1, 1]), float), np.array(ext.get('offset', [0, 0]), float))
        return dict(name=m['name'], alphaMode=m.get('alphaMode', 'OPAQUE'),
                    base=uri(pbr['baseColorTexture']), normal=uri(m['normalTexture']),
                    arm=uri(pbr['metallicRoughnessTexture']), uvxf=xf)

    def node_prims(self, node_name):
        j = self.j
        node = [n for n in j['nodes'] if n.get('name') == node_name][0]
        for k in ('rotation', 'scale', 'matrix'):
            if k in node:
                raise ValueError('node ' + k + ' unsupported')
        t = np.array(node.get('translation', [0, 0, 0]), float)
        out = []
        for p in j['meshes'][node['mesh']]['primitives']:
            at = p['attributes']
            if 'TEXCOORD_0' not in at:
                continue   # no UVs (fir_tree_01 variant c trunk): cannot be textured
            P = self.accessor(at['POSITION']) + t
            N = self.accessor(at['NORMAL'])
            UV = self.accessor(at['TEXCOORD_0'])
            T = self.accessor(p['indices']).reshape(-1, 3)
            if T.max() >= len(P) or T.min() < 0:
                raise ValueError('index out of range')
            info = self.material_info(p['material'])
            if info['uvxf'] is not None:
                UV = UV * info['uvxf'][0] + info['uvxf'][1]
            out.append(dict(mat=info['name'], P=P, N=N, UV=UV, T=T))
        return out


# ---------------------------------------------------------------------------
# Decimation
# ---------------------------------------------------------------------------

def weld_ids(P):
    key = np.round(P * 1e5).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    return inv.ravel().astype(np.int64)


def components(W, T):
    """Connected-component label per welded vertex (min vertex id)."""
    nw = int(W.max()) + 1
    lab = np.arange(nw)
    Tw = W[T]
    while True:
        l = np.minimum(np.minimum(lab[Tw[:, 0]], lab[Tw[:, 1]]), lab[Tw[:, 2]])
        new = lab.copy()
        for k in range(3):
            np.minimum.at(new, Tw[:, k], l)
        while True:
            nn = new[new]
            if (nn == new).all():
                break
            new = nn
        if (new == lab).all():
            return lab
        lab = new


_IU = [(0, 0), (0, 1), (0, 2), (0, 3), (1, 1), (1, 2), (1, 3), (2, 2), (2, 3), (3, 3)]


def _add_quadric(Q, idx, planes, weight, nw):
    K = np.stack([planes[:, i] * planes[:, j] for i, j in _IU], 1) * weight[:, None]
    for col in range(10):
        Q[:, col] += np.bincount(idx, K[:, col], minlength=nw)


def _eval(Q, p):
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    return (Q[:, 0] * x * x + 2 * Q[:, 1] * x * y + 2 * Q[:, 2] * x * z + 2 * Q[:, 3] * x
            + Q[:, 4] * y * y + 2 * Q[:, 5] * y * z + 2 * Q[:, 6] * y
            + Q[:, 7] * z * z + 2 * Q[:, 8] * z + Q[:, 9])


def qem_decimate(P, UV, T, target, diag, boundary_weight=50.0, uv_weight=0.1):
    """Batched half-edge-collapse quadric simplification.

    Works on a position-welded graph (so UV seams cannot crack) while every
    surviving vertex keeps its original position/normal/UV: an edge collapse
    moves one endpoint onto the other, so texture coordinates are never
    interpolated.  Open boundaries (leaf outlines) get penalty planes.  Each
    round collapses a vertex-disjoint set of cheapest local-minimum edges
    (vectorised); candidates that would flip a face are dropped.
    Returns the reduced triangle array and the (possibly moved) positions.
    """
    P = P.copy()
    T = T.copy()
    W = weld_ids(P)
    nw = int(W.max()) + 1
    Pw = np.zeros((nw, 3))
    Pw[W] = P
    for _ in range(80):
        Tw = W[T]
        ok = (Tw[:, 0] != Tw[:, 1]) & (Tw[:, 1] != Tw[:, 2]) & (Tw[:, 0] != Tw[:, 2])
        T, Tw = T[ok], Tw[ok]
        m = len(T)
        if m <= target:
            break
        p0, p1, p2 = Pw[Tw[:, 0]], Pw[Tw[:, 1]], Pw[Tw[:, 2]]
        c = np.cross(p1 - p0, p2 - p0)
        a2 = np.linalg.norm(c, axis=1)
        nrm = c / np.maximum(a2, 1e-30)[:, None]
        pl = np.concatenate([nrm, -(nrm * p0).sum(1)[:, None]], 1)
        Q = np.zeros((nw, 10))
        for k in range(3):
            _add_quadric(Q, Tw[:, k], pl, a2 * 0.5, nw)
        # unique welded edges
        ea = np.concatenate([Tw[:, 0], Tw[:, 1], Tw[:, 2]])
        eb = np.concatenate([Tw[:, 1], Tw[:, 2], Tw[:, 0]])
        iv = np.concatenate([T[:, 0], T[:, 1], T[:, 2]])
        iw = np.concatenate([T[:, 1], T[:, 2], T[:, 0]])
        face = np.tile(np.arange(m), 3)
        lo, hi = np.minimum(ea, eb), np.maximum(ea, eb)
        uk, first, inv, cnt = np.unique(lo * nw + hi, return_index=True, return_inverse=True,
                                        return_counts=True)
        inv = inv.ravel()
        a, b = uk // nw, uk % nw
        E = len(uk)
        bnd = np.nonzero(cnt == 1)[0]
        if len(bnd):
            ev = Pw[b[bnd]] - Pw[a[bnd]]
            L = np.linalg.norm(ev, axis=1)
            pn = np.cross(ev, nrm[face[first[bnd]]])
            pn /= np.maximum(np.linalg.norm(pn, axis=1), 1e-30)[:, None]
            bp = np.concatenate([pn, -(pn * Pw[a[bnd]]).sum(1)[:, None]], 1)
            for ends in (a[bnd], b[bnd]):
                _add_quadric(Q, ends, bp, boundary_weight * L * L, nw)
        duv = np.zeros(E)
        np.maximum.at(duv, inv, np.linalg.norm(UV[iv] - UV[iw], axis=1))
        Qs = Q[a] + Q[b]
        c_ab = _eval(Qs, Pw[b])   # a collapses onto b
        c_ba = _eval(Qs, Pw[a])
        cost = np.minimum(c_ab, c_ba) + uv_weight * (duv * diag) ** 2
        a_is_src = c_ab <= c_ba
        # vertex-disjoint local minima; candidates that would flip a face are
        # banned and the selection is repeated on the remaining edges
        v2 = np.concatenate([a, b])
        i2 = np.concatenate([np.arange(E), np.arange(E)])
        banned = np.zeros(E, bool)
        accepted = False
        for _attempt in range(8):
            ecost = np.where(banned, np.inf, cost)
            c2 = np.concatenate([ecost, ecost])
            order = np.lexsort((i2, c2, v2))
            vs = v2[order]
            fst = np.r_[True, vs[1:] != vs[:-1]]
            best = np.full(nw, -1)
            best[vs[fst]] = i2[order][fst]
            sel = np.nonzero((best[a] == np.arange(E)) & (best[b] == np.arange(E)) & ~banned)[0]
            if not len(sel):
                break
            sel = sel[np.argsort(cost[sel], kind='stable')][:max(1, (m - target + 1) // 2)]
            src = np.where(a_is_src[sel], a[sel], b[sel])
            dst = np.where(a_is_src[sel], b[sel], a[sel])
            srcmap = np.full(nw, -1)
            srcmap[src] = dst
            eof = np.full(nw, -1)
            eof[src] = sel
            for _it in range(6):
                nc = np.where(srcmap[Tw] >= 0, srcmap[Tw], Tw)
                changed = (nc != Tw).any(1)
                deg = (nc[:, 0] == nc[:, 1]) | (nc[:, 1] == nc[:, 2]) | (nc[:, 0] == nc[:, 2])
                q0, q1, q2 = Pw[nc[:, 0]], Pw[nc[:, 1]], Pw[nc[:, 2]]
                nn = np.cross(q1 - q0, q2 - q0)
                bad = changed & ~deg & ((nn * c).sum(1) <= 0.1 * np.linalg.norm(nn, axis=1) * a2)
                if not bad.any():
                    break
                bw = Tw[bad].ravel()
                bw = bw[srcmap[bw] >= 0]
                banned[eof[bw]] = True
                srcmap[bw] = -1
            if (srcmap >= 0).any():
                accepted = True
                break
        if not accepted:
            break
        # merge indexed vertices that share a triangle edge with the target
        merge = np.arange(len(P))
        hit = srcmap[W[iv]] == W[iw]
        merge[iv[hit]] = iw[hit]
        sm = srcmap[W]
        mv = sm >= 0
        P[mv] = Pw[sm[mv]]
        W[mv] = sm[mv]
        T = merge[T]
    Tw = W[T]
    ok = (Tw[:, 0] != Tw[:, 1]) & (Tw[:, 1] != Tw[:, 2]) & (Tw[:, 0] != Tw[:, 2])
    return T[ok], P


def triangle_area(P, T):
    return 0.5 * np.linalg.norm(np.cross(P[T[:, 1]] - P[T[:, 0]], P[T[:, 2]] - P[T[:, 0]]), axis=1)


def cull_components(P, N, UV, T, target, tris_per_comp, scale_cap, rng):
    """Keep a spatially stratified subset of connected pieces (leaf cards,
    needle tufts) and scale the survivors about their centroids to keep the
    foliage mass.  Returns (P, T) with only kept triangles."""
    W = weld_ids(P)
    lab = components(W, T)
    tlab = lab[W[T[:, 0]]]
    ulab, tinv, tcount = np.unique(tlab, return_inverse=True, return_counts=True)
    tinv = tinv.ravel()
    nc = len(ulab)
    K = max(1, target // tris_per_comp)
    if nc <= K:
        return P, T
    cen = np.zeros((nc, 3))
    ar = triangle_area(P, T)
    for d in range(3):
        cen[:, d] = np.bincount(tinv, P[T[:, 0], d] * ar, minlength=nc)
    carea = np.bincount(tinv, ar, minlength=nc)
    cen /= np.maximum(carea, 1e-30)[:, None]
    lo, hi = cen.min(0), cen.max(0)
    vol = max(float(np.prod(np.maximum(hi - lo, 1e-3))), 1e-9)
    cell = (vol / K) ** (1.0 / 3.0)
    g = np.floor((cen - lo) / cell).astype(np.int64)
    cid = (g[:, 0] * 7919 + g[:, 1]) * 104729 + g[:, 2]
    r1 = rng.random(nc)
    order = np.lexsort((r1, cid))
    cs = cid[order]
    start = np.r_[0, np.nonzero(cs[1:] != cs[:-1])[0] + 1]
    rank = np.arange(nc) - np.repeat(start, np.diff(np.r_[start, nc]))
    rk = np.empty(nc, np.int64)
    rk[order] = rank
    # structural pieces (stems, trunks, big branches: much larger than the
    # typical card) are always kept; the rest is thinned.
    struct = np.zeros(nc, bool)
    big = np.argsort(-tcount, kind='stable')
    big = big[tcount[big] >= 6 * np.median(tcount)]
    budget_tris = 3 * target
    cum = np.cumsum(tcount[big])
    struct[big[cum <= budget_tris]] = True
    keep_ids = np.lexsort((rng.random(nc), rk))
    keep_ids = keep_ids[~struct[keep_ids]][:max(K - int(struct.sum()), 0)]
    keep = struct.copy()
    keep[keep_ids] = True
    kt = keep[tinv]
    nonst = ~struct
    s = float(np.sqrt(carea[nonst].sum() / max(carea[keep & nonst].sum(), 1e-30)))
    s = min(max(s, 1.0), scale_cap)
    P = P.copy()
    vt = np.zeros(len(P), np.int64) - 1
    for k in range(3):
        vt[T[kt, k]] = tinv[kt]
    used = vt >= 0
    sc = np.where(struct[vt[used]], 1.0, s)[:, None]
    P[used] = cen[vt[used]] + (P[used] - cen[vt[used]]) * sc
    return P, T[kt]


def compact(P, N, UV, T):
    used = np.unique(T)
    remap = np.full(len(P), -1, np.int64)
    remap[used] = np.arange(len(used))
    return P[used], N[used], UV[used], remap[T]


def reduce_primitive(prim, target, cfg, lod, diag, seed):
    P, N, UV, T = prim['P'], prim['N'], prim['UV'], prim['T']
    rng = np.random.default_rng(seed)
    if len(T) > target:
        if cfg['cull']:
            cap = cfg.get('scale_cap', (1, 1, 1))[lod]
            P, T = cull_components(P, N, UV, T, target, cfg.get('tris_per_comp', 4), cap, rng)
        if len(T) > target:
            T, P = qem_decimate(P, UV, T, target, diag)
    return compact(P, N, UV, T)


# ---------------------------------------------------------------------------
# Textures
# ---------------------------------------------------------------------------

def srgb_to_lin(x):
    return np.where(x <= 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)


def lin_to_srgb(x):
    x = np.clip(x, 0, 1)
    return np.where(x <= 0.0031308, x * 12.92, 1.055 * x ** (1 / 2.4) - 0.055)


def flood_background(img, mask):
    """Replace pixels outside `mask` by dilated edge colour (then the mean)."""
    img = img.copy()
    m = mask.copy()
    for _ in range(24):
        if m.all():
            break
        acc = np.zeros_like(img)
        cnt = np.zeros(m.shape, np.float32)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy or dx:
                    mm = np.roll(m, (dy, dx), (0, 1))
                    acc += np.roll(img, (dy, dx), (0, 1)) * mm[..., None]
                    cnt += mm
        new = (~m) & (cnt > 0)
        img[new] = acc[new] / cnt[new][:, None]
        m |= new
    if not m.all():
        img[~m] = img[m].mean(0)
    return img


def resize_box(arr, size):
    if arr.shape[0] == size:
        return arr
    ch = [np.asarray(Image.fromarray(arr[..., c].astype(np.float32), 'F').resize(
        (size, size), Image.BOX)) for c in range(arr.shape[2])]
    return np.stack(ch, 2)


def save_jpg(arr01, path, subsampling):
    Image.fromarray(np.round(np.clip(arr01, 0, 1) * 255).astype(np.uint8), 'RGB').save(
        path, 'JPEG', quality=JPEG_QUALITY, optimize=True, subsampling=subsampling)


def process_textures(raw_dir, info, out_prefix, size, mask):
    """mask: bool (1024x1024-ish at source res) or None.  Returns file names."""
    def load(rel):
        im = Image.open(safe_join(raw_dir, rel)).convert('RGB')
        return np.asarray(im, np.float32) / 255.0

    def fit_mask(shape):
        if mask is None:
            return None
        if mask.shape == shape[:2]:
            return mask
        return np.asarray(Image.fromarray(mask.astype(np.uint8) * 255).resize(
            (shape[1], shape[0]), Image.NEAREST)) > 127

    # albedo: sRGB; filter in linear light
    d = load(info['base'])
    mk = fit_mask(d.shape)
    if mk is not None:
        d = flood_background(d, mk)
    d = lin_to_srgb(resize_box(srgb_to_lin(d), size)).astype(np.float32)
    save_jpg(d, out_prefix + '_diff.jpg', 2)
    # normal: linear vector, renormalise after filtering
    n = load(info['normal'])
    if mk is not None:
        n = flood_background(n, mk)
    v = resize_box(n * 2 - 1, size)
    v /= np.maximum(np.linalg.norm(v, axis=2, keepdims=True), 1e-6)
    save_jpg(v * 0.5 + 0.5, out_prefix + '_nor.jpg', 0)
    # ARM: R = AO, G = roughness, B forced to 0 (metalness unused); linear
    r = load(info['arm'])
    if mk is not None:
        r = flood_background(r, mk)
    r = resize_box(r, size)
    r[..., 2] = 0
    save_jpg(r, out_prefix + '_arm.jpg', 0)
    return [os.path.basename(out_prefix + s) for s in ('_diff.jpg', '_nor.jpg', '_arm.jpg')]


def bake_coverage(uv_tris_list, res=1024):
    """Rasterise UV footprints (list of (n,3,2) arrays) into a coverage image."""
    ss = 2
    big = Image.new('L', (res * ss, res * ss), 0)
    dr = ImageDraw.Draw(big)
    for tri in uv_tris_list:
        q = np.round(np.mod(tri, 1.0) * 4096).astype(np.int32)   # dedupe
        q = np.unique(q.reshape(len(q), 6), axis=0)
        pts = q.reshape(-1, 3, 2).astype(np.float64) / 4096 * (res * ss)
        for t in pts:
            dr.polygon([tuple(t[0]), tuple(t[1]), tuple(t[2])], fill=255, outline=255)
    return np.asarray(big.reduce(ss), np.uint8)


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def write_vmesh(path, subs):
    """subs: list of (material_index, P, N, UV, T) with local indices."""
    verts, idx, table = [], [], []
    base = 0
    first = 0
    for mi, P, N, UV, T in subs:
        verts.append(np.concatenate([P, N, UV], 1).astype('<f4'))
        idx.append((T + base).astype(np.int64).ravel())
        table.append((first, T.size, mi))
        base += len(P)
        first += T.size
    V = np.concatenate(verts)
    I = np.concatenate(idx)
    ib = 2 if len(V) < 65536 else 4
    allp = V[:, :3]
    bb = np.concatenate([allp.min(0), allp.max(0)]).astype('<f4')
    with open(path, 'wb') as f:
        f.write(b'VMSH')
        f.write(struct.pack('<IIIII', 1, len(V), len(I), len(table), ib))
        f.write(bb.tobytes())
        for t in table:
            f.write(struct.pack('<III', *t))
        f.write(V.tobytes())
        f.write(I.astype('<u2' if ib == 2 else '<u4').tobytes())
    return len(V), len(I) // 3, [float(x) for x in bb]


def read_info(raw_dir, sid):
    with open(safe_join(raw_dir, sid + '/info_api.json'), 'r', encoding='utf8') as f:
        return json.load(f)


def process_species(sp, raw_dir, out_dir, log):
    sid = sp['id']
    mdir = safe_join(raw_dir, sid)
    g = Gltf(mdir, sp['gltf'])
    odir = os.path.join(out_dir, sid)
    os.makedirs(odir, exist_ok=True)
    seed0 = zlib.crc32(sid.encode())
    # materials in a stable order
    mat_names = list(sp['mats'].keys())
    minfo = {}
    for mi in range(len(g.j['materials'])):
        inf = g.material_info(mi)
        minfo[inf['name']] = inf
    src = {v: g.node_prims(v) for v in sp['variants']}
    # texture coverage masks for cut-out materials
    materials = []
    for name in mat_names:
        cfg = sp['mats'][name]
        info = minfo[name]
        mask = None
        alpha_file = None
        coverage = None
        if cfg['cutout']:
            tris = [pr['UV'][pr['T']] for v in src.values() for pr in v if pr['mat'] == name]
            # footprint of every variant of the source file that uses the material
            for nd in g.j['nodes']:
                if nd.get('name') not in src:
                    for pr in g.node_prims(nd['name']):
                        if pr['mat'] == name:
                            tris.append(pr['UV'][pr['T']])
            cov = bake_coverage(tris)
            if cfg.get('key_black'):
                # atlas has a pure-black background and the cards are coarser
                # than the leaves (or the cards are plain quads): key the
                # background out of the texture as well.
                base = np.asarray(Image.open(safe_join(mdir, info['base'])).convert('RGB').resize(
                    (1024, 1024), Image.BOX), np.float32).max(2)
                cov = np.round(cov * np.clip((base - 12.0) / 24.0, 0, 1)).astype(np.uint8)
            coverage = float((cov > 127).mean())
            mask = cov > 127
            if coverage < ALPHA_FULL_COVERAGE:
                a = Image.fromarray(cov, 'L')
                if cfg['size'] != 1024:
                    a = a.resize((cfg['size'], cfg['size']), Image.BOX)
                alpha_file = name + '_alpha.png'
                a.save(os.path.join(odir, alpha_file), optimize=True)
            else:
                mask = None
        files = process_textures(mdir, info, os.path.join(odir, name), cfg['size'], mask)
        materials.append(dict(
            name=name, diffuse=files[0], normal=files[1], arm=files[2], alpha=alpha_file,
            alphaMode='MASK' if alpha_file else 'OPAQUE', alphaCutoff=0.5 if alpha_file else None,
            sourceAlphaMode=info['alphaMode'], doubleSided=True, wrap='repeat',
            textureSize=cfg['size'], alphaCoverage=coverage))
        log('  material %s: tex %d, alpha=%s coverage=%s' % (name, cfg['size'], alpha_file, coverage))
    variants = []
    for vname in sp['variants']:
        prims = src[vname]
        allP = np.concatenate([p['P'] for p in prims])
        lo, hi = allP.min(0), allP.max(0)
        diag = float(np.linalg.norm(hi - lo))
        if sp['pivot'] == 'base':
            sel = allP[:, 1] < lo[1] + 0.04 * (hi[1] - lo[1])
            cx, cz = allP[sel, 0].mean(), allP[sel, 2].mean()
        else:
            cx, cz = (lo[0] + hi[0]) / 2, (lo[2] + hi[2]) / 2
        shift = np.array([cx, lo[1], cz])
        orig_tris = sum(len(p['T']) for p in prims)
        # Trunk radius: median distance from the (median) axis of the solid, non
        # cut-out, non thinned geometry between 0.5 and 1.5 m above the base.  A
        # multi-stem tree or low branches at the base make it larger, which is
        # what a drone would hit; the simulator's collider follows it.
        slab = []
        for pr in prims:
            cfg = sp['mats'].get(pr['mat'])
            if cfg is None or cfg['cutout'] or cfg['cull']:
                continue
            q = pr['P'][np.unique(pr['T'])] - shift
            slab.append(q[(q[:, 1] >= 0.5) & (q[:, 1] <= 1.5)])
        slab = np.concatenate(slab) if slab else np.zeros((0, 3))
        if len(slab):
            ax = np.median(slab[:, [0, 2]], axis=0)
            trunk_radius = round(float(np.median(np.hypot(slab[:, 0] - ax[0], slab[:, 2] - ax[1]))), 4)
        else:
            trunk_radius = None
        lod_rows = []
        for lod in range(LODS):
            total = sp['budget'](orig_tris, lod)
            subs = []
            for mi, name in enumerate(mat_names):
                cfg = sp['mats'][name]
                for pi, pr in enumerate([p for p in prims if p['mat'] == name]):
                    tgt = int(max(4, min(len(pr['T']), total * cfg['weight'])))
                    if lod == 0 and sp['budget'](orig_tris, 0) >= orig_tris:
                        tgt = len(pr['T'])
                    P, N, UV, T = reduce_primitive(
                        pr, tgt, cfg, lod, diag, seed0 + 97 * lod + 13 * pi + zlib.crc32(vname.encode()) % 1000)
                    nrm = N / np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
                    subs.append((mi, P - shift, nrm, UV, T))
            fn = '%s_lod%d.vmesh' % (vname, lod)
            nv, nt, bb = write_vmesh(os.path.join(odir, fn), subs)
            lod_rows.append(dict(file=fn, triangles=nt, vertices=nv, bboxMin=bb[:3], bboxMax=bb[3:]))
            log('  %s lod%d: %d tris %d verts' % (vname, lod, nt, nv))
        b0 = lod_rows[0]
        variants.append(dict(
            name=vname, sourceTriangles=orig_tris, lods=lod_rows,
            bboxMin=b0['bboxMin'], bboxMax=b0['bboxMax'],
            nominalHeight=round(b0['bboxMax'][1], 4), trunkRadius=trunk_radius,
            footprint=[round(b0['bboxMax'][0] - b0['bboxMin'][0], 4),
                       round(b0['bboxMax'][2] - b0['bboxMin'][2], 4)]))
    info = read_info(raw_dir, sid)
    return dict(id=sid, title=sp['title'], category=sp['category'], directory=sid,
                source='https://polyhaven.com/a/' + sid, license='CC0',
                authors=info.get('authors'), materials=materials, variants=variants)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('raw_dir')
    ap.add_argument('out_dir')
    ap.add_argument('--only', nargs='*', help='species ids to (re)build')
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    log = lambda s: print(s, flush=True)
    species = []
    for sp in SPECIES:
        if a.only and sp['id'] not in a.only:
            continue
        log(sp['id'])
        species.append(process_species(sp, a.raw_dir, a.out_dir, log))
    mpath = os.path.join(a.out_dir, 'manifest.json')
    if a.only and os.path.exists(mpath):
        old = {s['id']: s for s in json.load(open(mpath))['species']}
        old.update({s['id']: s for s in species})
        species = [old[s['id']] for s in SPECIES if s['id'] in old]
    manifest = dict(
        version=1, units='metres',
        frame='glTF: right-handed, +Y up; each variant has its base at y=0 and pivot at base centre (x,z)',
        uv='glTF convention: (0,0) = top-left of the image, v down; upload QImage unflipped; wrap=repeat',
        colorSpace=dict(diffuse='sRGB', normal='linear, tangent space, OpenGL (+Y up)',
                        arm='linear; R=ambient occlusion, G=roughness, B=0',
                        alpha='linear 8-bit coverage, alpha test at alphaCutoff'),
        meshFormat='vmesh v1 little endian, see tools/vegetation/build_vegetation.py',
        lodBudgetsConifers=list(BIG_CONIFER), lodBudgetsBroadleaf=list(BIG_BROADLEAF), lodBudgetsSmall=list(SMALL), species=species)
    with open(mpath, 'w', encoding='utf8') as f:
        json.dump(manifest, f, indent=1)
        f.write('\n')


if __name__ == '__main__':
    main()
