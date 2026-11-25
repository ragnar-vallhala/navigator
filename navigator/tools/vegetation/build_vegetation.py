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
# Broadleaf foliage: the scans are olive-yellow (hue about 70 deg); natural leaf
# green sits near 90-100. (hue rotation in degrees, chroma and value factors)
GRADE_LEAF = (-22.0, 0.95, 0.85)
# The island tree's crown reads darker than the jacaranda's next to it; its leaf
# scan is the darker one, so its value factor is 15% higher (0.85 -> 0.98).
GRADE_ISLAND_LEAF = (-22.0, 0.95, 0.98)
BIG_CONIFER = (32000, 7000, 1200)
BIG_BROADLEAF = (36000, 7000, 1200)
JACARANDA = (36000, 7000, 3000)

SPECIES = [
    # Conifers are built, not decimated: a tapered trunk plus tiers of needle
    # cards cut from the model's own twig texture (see card_conifer_mesh).
    dict(id='fir_tree_01', gltf='fir_tree_01_1k.gltf', category='tree',
         title='Fir (card conifer, Poly Haven fir_tree_01 textures)',
         cards=dict(bark='fir_tree_01_bark', twig='fir_tree_01_twig', key='hp', thick=2.6,
                    crop=(335, 385, 655, 785), grade=(0.55, 0.80, 0.90),
                    variants=[
                        dict(H=16.0, tiers=(10, 7, 5), cards=(16, 12, 8), y0=0.10, R=3.4, len=1.25,
                             wid=1.2, droop=(34, 20), r0=0.17, r1=0.03),
                        dict(H=16.0, tiers=(8, 6, 4), cards=(16, 12, 8), y0=0.14, R=3.0, len=1.3,
                             wid=1.25, droop=(38, 22), r0=0.17, r1=0.03)])),
    dict(id='pine_sapling_medium', gltf='pine_sapling_medium_1k.gltf', category='tree',
         title='Pine (card conifer, Poly Haven pine_sapling_medium textures)',
         # whorls of a few long, upward-curving branches with sky between them
         cards=dict(bark='pine_sapling_medium_bark', twig='pine_sapling_medium_twig', key='black',
                    crop=(30, 40, 235, 455), grade=(0.85, 1.20, 1.35), thick=1.6, de_yellow=0.8,
                    bark_tint=(0.40, 0.395, 0.41),
                    variants=[
                        dict(H=12.0, tiers=(7, 6, 6), cards=(7, 6, 4), y0=0.25, R=3.0, len=1.25,
                             wid=1.5, droop=(10, 5), curve=0.32, r0=0.34, r1=0.08, trunk_top=0.9, jit=0.015, sag=0.06, az=0.38,
                             lower=0.6, lowlen=0.15, bark_u=3.0, bark_v=1.2,
                             spacing_jit=0.35, alongs=(2, 2, 2), lowbr=(9, 9, 5), lowy=(0.06, 0.30), lowdroop=34),
                        dict(H=12.0, tiers=(7, 6, 6), cards=(7, 6, 4), y0=0.23, R=3.0, len=1.25,
                             wid=1.5, droop=(12, 6), curve=0.34, r0=0.34, r1=0.08, trunk_top=0.9, jit=0.015, sag=0.06, az=0.38,
                             lower=0.6, lowlen=0.15, bark_u=3.0, bark_v=1.2,
                             spacing_jit=0.35, alongs=(2, 2, 2), lowbr=(9, 9, 5), lowy=(0.06, 0.30), lowdroop=34),
                        dict(H=12.0, tiers=(7, 6, 6), cards=(7, 6, 4), y0=0.27, R=3.2, len=1.25,
                             wid=1.5, droop=(8, 4), curve=0.30, r0=0.34, r1=0.08, trunk_top=0.9, jit=0.015, sag=0.06, az=0.38,
                             lower=0.6, lowlen=0.15, bark_u=3.0, bark_v=1.2,
                             spacing_jit=0.35, alongs=(2, 2, 2), lowbr=(9, 9, 5), lowy=(0.06, 0.30), lowdroop=34)])),
    dict(id='jacaranda_tree', gltf='jacaranda_tree_1k.gltf', category='tree',
         title='Jacaranda tree (broadleaf)', pivot='base',
         variants=['jacaranda_tree_LOD0', 'jacaranda_tree_LOD0_wide'],
         # the second variant is the same model turned and reproportioned
         derive={'jacaranda_tree_LOD0_wide': dict(src='jacaranda_tree_LOD0', rot=2.4, sx=1.16, sy=0.86, sz=0.92)},
         budget=lod_budgets(JACARANDA),
         mats={'jacaranda_tree_trunk': dict(weight=0.08, cutout=False, cull=False, size=512),
               'jacaranda_tree_leaves': dict(weight=0.82, cutout=True, key_black=True, cull=True, grade=GRADE_LEAF,
                                             size=1024, scale_cap=(3.5, 4.5, 6.0), tris_per_comp=2),
               'jacaranda_tree_branches': dict(weight=0.10, cutout=False, cull=True, size=512,
                                               scale_cap=(2.0, 2.5, 3.0), tris_per_comp=12)}),
    dict(id='island_tree_01', gltf='island_tree_01_1k.gltf', category='tree',
         title='Island tree 01 (broadleaf)', pivot='base',
         variants=['island_tree_01_LOD0', 'island_tree_01_LOD0_slim'],
         derive={'island_tree_01_LOD0_slim': dict(src='island_tree_01_LOD0', rot=1.9, sx=0.88, sy=1.1, sz=1.08)},
         budget=lod_budgets(BIG_BROADLEAF),
         mats={'island_tree_01': dict(weight=0.10, cutout=False, cull=False, size=512),
               # impostor_cutoff: the island's sparse crown bakes denser than its mesh
               # shows it from 50 m out (the bake keeps leaf texels the mesh's alpha
               # test drops once the leaf texture is minified), so its impostor
               # holds 1.5 to 3 times the mesh's pixels; a higher cut-off in the bake
               # alone brings it down. The mesh LODs keep the 0.5.
               'island_tree_01_leaves': dict(weight=0.78, cutout=True, key_black=True, cull=True, grade=GRADE_ISLAND_LEAF,
                                             size=1024, scale_cap=(3.5, 4.5, 8.0), tris_per_comp=2,
                                             impostor_cutoff=0.7),
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


def cull_components(P, N, UV, T, target, tris_per_comp, scale_cap, rng, struct_budget=3.0, uv_shrink=1.0, fill=1.0):
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
        return P, T, UV
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
    # long twig stems are 91% of the triangles of the fir sources but little of
    # the visible foliage: conifers cap what they may take (struct_budget)
    budget_tris = int(struct_budget * target)
    cum = np.cumsum(tcount[big])
    struct[big[cum <= budget_tris]] = True
    keep_ids = np.lexsort((rng.random(nc), rk))
    keep_ids = keep_ids[~struct[keep_ids]][:max(K - int(struct.sum()), 0)]
    keep = struct.copy()
    keep[keep_ids] = True
    kt = keep[tinv]
    nonst = ~struct
    s = float(np.sqrt(carea[nonst].sum() / max(carea[keep & nonst].sum(), 1e-30)))
    s = min(max(s * fill, 1.0), scale_cap)  # fill > 1: denser than the source
    P = P.copy()
    vt = np.zeros(len(P), np.int64) - 1
    for k in range(3):
        vt[T[kt, k]] = tinv[kt]
    used = vt >= 0
    sc = np.where(struct[vt[used]], 1.0, s)[:, None]
    P[used] = cen[vt[used]] + (P[used] - cen[vt[used]]) * sc
    if uv_shrink < 1.0:
        # Needle blades map onto thin slivers of the twig texture (a dozen
        # texels wide): a mip level or two later the alpha test drops the
        # whole blade. Pull each kept blade's UVs to the middle of its sliver.
        UV = UV.copy()
        cuv = np.zeros((nc, 2))
        cnt = np.bincount(tinv[kt], minlength=nc).astype(float)
        for d in range(2):
            cuv[:, d] = np.bincount(tinv[kt], UV[T[kt, 0], d], minlength=nc)
        cuv /= np.maximum(cnt, 1.0)[:, None]
        sel = used & ~struct[np.maximum(vt, 0)]
        UV[sel] = cuv[vt[sel]] + (UV[sel] - cuv[vt[sel]]) * uv_shrink
    return P, T[kt], UV


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
            P, T, UV = cull_components(P, N, UV, T, target, cfg.get('tris_per_comp', 4), cap, rng,
                                       cfg.get('struct_budget', 3.0), cfg.get('uv_shrink', 1.0),
                                       cfg.get('fill', 1.0))
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


def grade_albedo(d, hue_deg, chroma, value):
    """Rotate the hue (YIQ plane), scale chroma and value of sRGB colour d."""
    M = np.array([[0.299, 0.587, 0.114], [0.596, -0.274, -0.322], [0.211, -0.523, 0.312]])
    q = d @ M.T
    c, s_ = np.cos(np.radians(hue_deg)), np.sin(np.radians(hue_deg))
    i, qq = q[..., 1] * c - q[..., 2] * s_, q[..., 1] * s_ + q[..., 2] * c
    out = np.stack([q[..., 0] * value, i * chroma * value, qq * chroma * value], -1)
    return np.clip(out @ np.linalg.inv(M).T, 0, 1).astype(np.float32)


def process_textures(raw_dir, info, out_prefix, size, mask, grade=None):
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
    if grade:
        d = grade_albedo(d, *grade)
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


# ---------------------------------------------------------------------------
# Card conifers: a trunk plus tiers of needle-branch cards
# ---------------------------------------------------------------------------

def box_blur(x, r):
    k = 2 * r + 1
    pd = np.pad(x, r, mode='edge')
    c = np.pad(np.cumsum(np.cumsum(pd, 0), 1), ((1, 0), (1, 0)))
    return (c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]) / (k * k)


def sprig_card_textures(raw_dir, info, cfg, out_prefix, size):
    """Cut one needle sprig out of a Poly Haven twig atlas into a square card
    texture (diffuse, normal, ARM, alpha) and grade it.  Returns file names,
    coverage and the mean linear luminance of the covered pixels."""
    def load(rel):
        return np.asarray(Image.open(safe_join(raw_dir, rel)).convert('RGB'), np.float32) / 255.0
    d, n, r = load(info['base']), load(info['normal']), load(info['arm'])
    f = d.shape[0] / 1024.0
    x0, y0, x1, y1 = [int(round(v * f)) for v in cfg['crop']]
    d, n, r = d[y0:y1, x0:x1], n[y0:y1, x0:x1], r[y0:y1, x0:x1]
    if cfg['key'] == 'black':
        a = np.clip((d.max(2) * 255.0 - 12.0) / 24.0, 0, 1)
    else:
        lum = d @ np.array([0.2126, 0.7152, 0.0722], np.float32)
        a = np.clip((box_blur(lum, int(round(12 * f))) - lum - cfg.get('hp', 0.015)) / 0.05, 0, 1)
    # thicken the needles a little: at card size they are about a pixel wide
    a = np.clip(box_blur(a, 1) * cfg.get('thick', 2.2), 0, 1)
    # keep only the sprig that holds the stem (the atlas has neighbours and
    # stretched fill around it): grow from the stem's foot through near pixels
    cand = a > 0.3
    rows = np.arange(a.shape[0] * 3 // 4, a.shape[0])
    ys, xs = np.nonzero(a[rows[0]:] > 0.5)
    k = np.argmin(np.abs(xs - a.shape[1] // 2) + 0.2 * np.abs(ys))
    m = np.zeros(a.shape, bool)
    m[rows[0] + ys[k], xs[k]] = True
    while True:
        g = m.copy()
        for dy in (-3, 0, 3):
            for dx in (-3, 0, 3):
                g |= np.roll(m, (dy, dx), (0, 1))
        g &= cand
        if (g == m).all():
            break
        m = g
    a = a * (box_blur(m.astype(np.float32), 3) > 0.02)
    side = max(a.shape)
    ph, pw = (side - a.shape[0]), (side - a.shape[1])
    pad = ((ph // 2, ph - ph // 2), (pw // 2, pw - pw // 2))
    a = np.pad(a, pad)
    d = flood_background(np.pad(d, pad + ((0, 0),)), a > 0.5)
    n = flood_background(np.pad(n, pad + ((0, 0),), constant_values=0.5), a > 0.5)
    r = flood_background(np.pad(r, pad + ((0, 0),), constant_values=0.5), a > 0.5)
    lin = srgb_to_lin(d)
    if cfg.get('de_yellow'):
        # dry, straw-coloured needles: pull red back toward green so they read as needles
        dry = lin[..., 0] > cfg['de_yellow'] * lin[..., 1]
        lin[..., 0] = np.where(dry, lin[..., 1] * cfg['de_yellow'] * 0.7, lin[..., 0])
        lin[..., 1] = np.where(dry, lin[..., 1] * 0.9, lin[..., 1])
    lin = lin * np.array(cfg['grade'], np.float32)
    lin = resize_box(lin.astype(np.float32), size)
    a8 = np.asarray(Image.fromarray(np.round(a * 255).astype(np.uint8), 'L').resize((size, size), Image.BOX))
    cov = a8 > 127
    lum = float((lin[cov] @ np.array([0.2126, 0.7152, 0.0722])).mean())
    save_jpg(lin_to_srgb(lin), out_prefix + '_diff.jpg', 2)
    v = resize_box(n * 2 - 1, size)
    v /= np.maximum(np.linalg.norm(v, axis=2, keepdims=True), 1e-6)
    save_jpg(v * 0.5 + 0.5, out_prefix + '_nor.jpg', 0)
    rr = resize_box(r, size)
    rr[..., 2] = 0
    save_jpg(rr, out_prefix + '_arm.jpg', 0)
    alpha_file = os.path.basename(out_prefix) + '_alpha.png'
    Image.fromarray(a8, 'L').save(out_prefix + '_alpha.png', optimize=True)
    return ([os.path.basename(out_prefix + s) for s in ('_diff.jpg', '_nor.jpg', '_arm.jpg')],
            alpha_file, float(cov.mean()), lum)


def furrowed_bark(prefix, tint, seed=5):
    """Overwrite a bark material (<prefix>_diff/_nor/_arm.jpg, 512 px, repeat) with
    a lighter, warm, vertically furrowed bark.  The colour is the CC0 source bark
    lifted and warmed; the furrows are seeded periodic noise stretched along the
    trunk (image y), so the texture tiles and every run is identical."""
    n = 512
    rng = np.random.default_rng(seed)
    fx = np.fft.fftfreq(n)[None, :] * n
    fy = np.fft.fftfreq(n)[:, None] * n

    def band(wx, wy, k):
        f = np.exp(-(fx / wx) ** 2 - (fy / wy) ** 2)
        h = np.fft.ifft2(np.fft.fft2(rng.standard_normal((n, n))) * f).real
        return (h - h.mean()) / h.std() * k
    h = band(60.0, 4.0, 1.0) + band(24.0, 2.5, 0.8) + band(120.0, 24.0, 0.3)
    rid = 1.0 / (1.0 + np.exp(-1.6 * h))                 # 0 in a furrow, 1 on a ridge
    src = np.asarray(Image.open(prefix + '_diff.jpg').convert('RGB'), np.float32) / 255.0
    lum = (src @ np.array([0.299, 0.587, 0.114], np.float32))[..., None]
    col = (lum / max(float(lum.mean()), 1e-3)) ** 0.5 * np.array(tint, np.float32)
    save_jpg(col * (0.72 + 0.38 * rid[..., None]), prefix + '_diff.jpg', 2)
    gx = (np.roll(rid, -1, 1) - np.roll(rid, 1, 1)) * 0.5 * n / 70.0
    gy = (np.roll(rid, -1, 0) - np.roll(rid, 1, 0)) * 0.5 * n / 70.0
    nv = np.stack([-gx, gy, np.ones_like(gx)], -1)
    nv /= np.linalg.norm(nv, axis=2, keepdims=True)
    save_jpg(nv * 0.5 + 0.5, prefix + '_nor.jpg', 0)
    arm = np.stack([0.55 + 0.45 * rid, 0.85 + 0 * rid, 0 * rid], -1)   # AO in furrows, rough
    save_jpg(arm, prefix + '_arm.jpg', 0)


def card_conifer_mesh(cv, lod, rng):
    """One card conifer (glTF frame, y up, base at the origin).  Returns
    ((trunk P, N, UV, T), (card P, N, UV, T), tier count)."""
    H = cv['H']
    ntier = cv['tiers'][lod]
    ncard = cv['cards'][lod]
    nalong = cv.get('alongs', (2, 1, 1))[lod]
    sides = (8, 6, 4)[lod]
    y_lo, y_hi = cv['y0'] * H, 0.93 * H
    r0, r1 = cv['r0'], cv['r1']

    def trunk_r(y):
        return r0 + (r1 - r0) * (y / H) ** 0.6

    # trunk: tapered prism, v tiles every 2 m
    rings = (5, 3, 2)[lod]
    TP, TN, TU, TT = [], [], [], []
    for i in range(rings + 1):
        y = H * cv.get('trunk_top', 0.72) * i / rings
        for k in range(sides + 1):
            a = 2 * np.pi * k / sides
            TP.append((trunk_r(y) * np.cos(a), y, trunk_r(y) * np.sin(a)))
            TN.append((np.cos(a), 0.0, np.sin(a)))
            TU.append((cv.get('bark_u', 2.0) * k / sides, 1.0 - y / cv.get('bark_v', 2.0)))
    for i in range(rings):
        for k in range(sides):
            a = i * (sides + 1) + k
            TT += [(a, a + 1, a + sides + 1), (a + 1, a + sides + 2, a + sides + 1)]
    trunk = (np.array(TP, np.float64), np.array(TN, np.float64), np.array(TU, np.float64),
             np.array(TT, np.int64))

    P, N, UV, T = [], [], [], []

    def add_card(root, out, up_n, L, W, droop, sag, nalong, curve=0.0):
        # root: 3-vector; out: unit horizontal direction; the card hangs along
        # `out`, drooping by `droop` (slope) and sagging `sag` toward its sides
        perp = np.array([-out[2], 0.0, out[0]])
        base = len(P)
        for i in range(nalong + 1):
            s = i / nalong
            c = root + out * (s * L) + np.array([0.0, (-droop * s + curve * s * s) * L, 0.0])
            for t in (-1.0, 0.0, 1.0):
                q = c + perp * (t * 0.5 * W) + np.array([0.0, -sag * L * abs(t) * s, 0.0])
                q[1] = max(q[1], 0.4)    # no card hangs into the ground
                P.append(q)
                N.append(up_n)
                UV.append((0.5 + 0.5 * t, 1.0 - s))
        for i in range(nalong):
            for k in range(2):
                a = base + i * 3 + k
                T.append((a, a + 1, a + 4))
                T.append((a, a + 4, a + 3))

    for j in range(ntier):
        t = j / max(ntier - 1, 1)
        if cv.get('spacing_jit', 0.0) > 0.0 and 0 < j < ntier - 1:
            # irregular whorl spacing: each whorl moves up to a fraction of the gap
            t += rng.uniform(-1.0, 1.0) * cv['spacing_jit'] / (ntier - 1)
        y = y_lo + (y_hi - y_lo) * t
        R = cv["R"] * (1.0 - t * 0.9) ** cv.get("taper", 1.0) + 0.55
        # coarser LODs have fewer, so larger, cards: the crown keeps its mass
        fill = min((cv['cards'][0] / ncard) ** 0.5 * (cv['tiers'][0] / ntier) ** 0.3, 1.8)
        L = R * cv['len'] * fill ** 0.5 * (1.0 + cv.get('lowlen', 0.0) * (1.0 - t))
        ncj = max(3, int(round(ncard * (1.0 + cv.get('lower', 0.0) * (1.0 - 2.0 * t)))))
        W = R * cv['len'] * cv['wid'] * fill * 1.1
        droop = np.tan(np.radians(cv['droop'][0] + (cv['droop'][1] - cv['droop'][0]) * t))
        off = rng.uniform(0, 2 * np.pi)
        for k in range(ncj):
            th = off + 2 * np.pi * (k + 0.5 * (j % 2) + rng.uniform(-1.0, 1.0) * cv.get('az', 0.18)) / ncj
            out = np.array([np.cos(th), 0.0, np.sin(th)])
            lj = L * rng.uniform(0.88, 1.12)
            yj = y + rng.uniform(-1.0, 1.0) * cv.get('jit', 0.04) * H
            root = out * (trunk_r(yj) * 0.6) + np.array([0.0, yj, 0.0])
            un = out * 0.45 + np.array([0.0, 0.9, 0.0])
            add_card(root, out, un / np.linalg.norm(un), lj, W * lj / L, droop, cv.get('sag', 0.14), nalong, cv.get('curve', 0.0))
    # a few long, low, drooping branches so the bottom third is not a bare pole
    for i in range(cv.get('lowbr', (0, 0, 0))[lod]):
        nlow = cv['lowbr'][lod]
        th = 2 * np.pi * (i + rng.uniform(0.0, 0.8)) / nlow + 0.9 * i
        out = np.array([np.cos(th), 0.0, np.sin(th)])
        yb = H * (cv['lowy'][0] + (cv['lowy'][1] - cv['lowy'][0]) * (i + rng.uniform(0.1, 0.9)) / nlow)
        fill = min((cv['cards'][0] / ncard) ** 0.5 * (cv['tiers'][0] / ntier) ** 0.3, 1.8)
        Lb = cv['R'] * cv['len'] * 1.0 * fill ** 0.5 * rng.uniform(0.85, 1.1)
        root = out * (trunk_r(yb) * 0.6) + np.array([0.0, yb, 0.0])
        un = out * 0.45 + np.array([0.0, 0.9, 0.0])
        add_card(root, out, un / np.linalg.norm(un), Lb, Lb * cv['wid'] * 0.9, np.tan(np.radians(cv['lowdroop'])),
                 cv.get('sag', 0.14), nalong, 0.12)
    # leader: two crossed upright cards
    for ph in (0.0, np.pi / 2):
        out = np.array([np.cos(ph), 0.0, np.sin(ph)])
        perp = np.array([-out[2], 0.0, out[0]])
        base = len(P)
        Lh = 0.11 * H * cv['len'] / 1.1
        for i, s in enumerate((0.0, 1.0)):
            for t in (-1.0, 1.0):
                P.append(np.array([0.0, y_hi - 0.02 * H + s * Lh, 0.0]) + perp * (t * 0.5 * Lh * 0.7))
                N.append(np.array([out[0] * 0.5, 0.8, out[2] * 0.5]))
                UV.append((0.5 + 0.5 * t, 1.0 - s))
        T += [(base, base + 1, base + 2), (base + 1, base + 3, base + 2)]
    cardm = (np.array(P, np.float64), np.array(N, np.float64) / np.linalg.norm(np.array(N), axis=1, keepdims=True),
             np.array(UV, np.float64), np.array(T, np.int64))
    return trunk, cardm, ntier


def process_card_species(sp, raw_dir, out_dir, log):
    sid = sp['id']
    mdir = safe_join(raw_dir, sid)
    g = Gltf(mdir, sp['gltf'])
    odir = os.path.join(out_dir, sid)
    os.makedirs(odir, exist_ok=True)
    minfo = {}
    for mi in range(len(g.j['materials'])):
        inf = g.material_info(mi)
        minfo[inf['name']] = inf
    cfg = sp['cards']
    # material 0: bark (the source model's, opaque); material 1: the needle card
    bark = cfg['bark']
    bfiles = process_textures(mdir, minfo[bark], os.path.join(odir, bark), 512, None)
    if cfg.get('bark_tint'):
        furrowed_bark(os.path.join(odir, bark), cfg['bark_tint'])
    nfiles, alpha_file, cov, lum = sprig_card_textures(
        mdir, minfo[cfg['twig']], cfg, os.path.join(odir, sid + '_needles'), 512)
    log('  needle card: coverage %.3f, mean luminance %.4f' % (cov, lum))
    materials = [
        dict(name=bark, diffuse=bfiles[0], normal=bfiles[1], arm=bfiles[2], alpha=None,
             alphaMode='OPAQUE', alphaCutoff=None, sourceAlphaMode='OPAQUE', doubleSided=True,
             wrap='repeat', textureSize=512, alphaCoverage=None),
        dict(name=sid + '_needles', diffuse=nfiles[0], normal=nfiles[1], arm=nfiles[2],
             alpha=alpha_file, alphaMode='MASK', alphaCutoff=0.5, sourceAlphaMode='BLEND',
             doubleSided=True, wrap='repeat', textureSize=512, alphaCoverage=cov,
             meanLuminance=round(lum, 4))]
    variants = []
    for vi, cv in enumerate(cfg['variants']):
        vname = '%s_card_%s' % (sid, 'abcdefgh'[vi])
        lod_rows = []
        for lod in range(LODS):
            rng = np.random.default_rng(zlib.crc32(vname.encode()))   # same layout on every LOD
            trunk, cardm, ntier = card_conifer_mesh(cv, lod, rng)
            subs = [(0,) + tuple(trunk), (1,) + tuple(cardm)]
            fn = '%s_lod%d.vmesh' % (vname, lod)
            nv, nt, bb = write_vmesh(os.path.join(odir, fn), subs)
            lod_rows.append(dict(file=fn, triangles=nt, vertices=nv, tiers=ntier,
                                 bboxMin=bb[:3], bboxMax=bb[3:]))
            log('  %s lod%d: %d tiers, %d tris' % (vname, lod, ntier, nt))
        b0 = lod_rows[0]
        variants.append(dict(
            name=vname, sourceTriangles=0, lods=lod_rows,
            bboxMin=b0['bboxMin'], bboxMax=b0['bboxMax'],
            nominalHeight=round(b0['bboxMax'][1], 4),
            trunkRadius=round(cv['r0'] + (cv['r1'] - cv['r0']) * (1.0 / cv['H']) ** 0.6, 4),
            footprint=[round(b0['bboxMax'][0] - b0['bboxMin'][0], 4),
                       round(b0['bboxMax'][2] - b0['bboxMin'][2], 4)]))
    info = read_info(raw_dir, sid)
    return dict(id=sid, title=sp['title'], category=sp['category'], directory=sid,
                source='https://polyhaven.com/a/' + sid, license='CC0',
                authors=info.get('authors'), materials=materials, variants=variants)


def derive_prims(prims, d):
    """A derived variant of a source mesh: yaw, anisotropic scale, same textures."""
    c, s = np.cos(d['rot']), np.sin(d['rot'])
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    S = np.array([d['sx'], d['sy'], d['sz']])
    out = []
    for pr in prims:
        q = dict(pr)
        q['P'] = (pr['P'] @ R.T) * S
        q['N'] = pr['N'] @ R.T
        out.append(q)
    return out


def read_info(raw_dir, sid):
    with open(safe_join(raw_dir, sid + '/info_api.json'), 'r', encoding='utf8') as f:
        return json.load(f)


def process_species(sp, raw_dir, out_dir, log):
    if 'cards' in sp:
        return process_card_species(sp, raw_dir, out_dir, log)
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
    derive = sp.get('derive', {})
    src = {v: g.node_prims(v) for v in sp['variants'] if v not in derive}
    for v, d in derive.items():
        src[v] = derive_prims(src[d['src']], d)
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
        files = process_textures(mdir, info, os.path.join(odir, name), cfg['size'], mask, cfg.get('grade'))
        materials.append(dict(
            name=name, diffuse=files[0], normal=files[1], arm=files[2], alpha=alpha_file,
            alphaMode='MASK' if alpha_file else 'OPAQUE', alphaCutoff=0.5 if alpha_file else None,
            sourceAlphaMode=info['alphaMode'], doubleSided=True, wrap='repeat',
            textureSize=cfg['size'], alphaCoverage=coverage))
        if alpha_file and cfg.get('impostor_cutoff'):
            materials[-1]['impostorCutoff'] = cfg['impostor_cutoff']
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
