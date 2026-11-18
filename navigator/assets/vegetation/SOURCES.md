# Vegetation asset sources

All models in this directory are derived from CC0 models published by Poly Haven (1K glTF downloads). Retrieved 2026-11-18.

## Licence

Read on 2026-11-18 at https://polyhaven.com/license (section "Asset License"). Verbatim:

> All assets (HDRIs, textures and 3D models) on this site are the original work of Poly Haven staff, or artists who willingly and directly donate/sell their work to Poly Haven.
> Our assets are all licensed as CC0, which is effectively Public Domain even in jurisdictions that do not support the Public Domain.
> You can use our assets for any purpose, including commercial work.
> You do not need to give credit or attribution when using them (although it is appreciated).
> You can redistribute them, share them around, include them when sharing your own work, or even in a product you sell.

(Each line is a separate sentence or bullet of that page, quoted unaltered; the page has the CC0 FAQ excerpt between the second and third.) Credit is not required; it is given here anyway.

## Models

Every entry: licence CC0, retrieved 2026-11-18, URL https://polyhaven.com/a/<id>. Authors are as given in the info record of the Poly Haven API (role in brackets).

### fir_sapling

- Name: Fir Sapling
- Author(s): Rob Tuytel (photography), Rico Cilliers (modeling)
- URL: https://polyhaven.com/a/fir_sapling
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 3 variants (a, b, c). Twig/needle cards thinned and scaled, bark decimated; alpha mask baked from source UV coverage.

### pine_sapling_small

- Name: Pine Sapling Small
- Author(s): Rob Tuytel (photography), Rico Cilliers (modeling)
- URL: https://polyhaven.com/a/pine_sapling_small
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 3 variants (a, b, c). Needle/twig cards thinned and scaled, bark decimated; alpha mask baked from source UV coverage.

### island_tree_02

- Name: Island Tree 02
- Author(s): Rob Tuytel (scanning, processing), Rico Cilliers (cleanup, processing)
- URL: https://polyhaven.com/a/island_tree_02
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 1 variant. Trunk and branch meshes decimated, leaf cards thinned and scaled; alpha mask = UV coverage x black-background key. The KHR_texture_transform on the branch/bark material was baked into the UVs (tiling, wrap = repeat).

### shrub_03

- Name: Shrub 03
- Author(s): Rico Cilliers (All)
- URL: https://polyhaven.com/a/shrub_03
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 4 variants (a-d). LOD0 is the source mesh unchanged.

### shrub_04

- Name: Shrub 04
- Author(s): Rico Cilliers (All)
- URL: https://polyhaven.com/a/shrub_04
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 1 variant, decimated to the shrub budget; alpha mask baked from source UV coverage.

### weed_plant_02

- Name: Weed Plant 02
- Author(s): Rob Tuytel (Photography), Rico Cilliers (Modeling)
- URL: https://polyhaven.com/a/weed_plant_02
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 5 variants (a-e). LOD0 is the source mesh unchanged; alpha mask = UV coverage x black-background key.

### grass_bermuda_01

- Name: Grass Bermuda 01
- Author(s): Rico Cilliers (All)
- URL: https://polyhaven.com/a/grass_bermuda_01
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 6 of the 21 pieces of the source sheet (medium_c, medium_e, medium_f, seedling_d, seedling_a, small_c). The source cards are plain polygons on a black background (no geometric cut-out), so alpha is keyed from the texture.

### rock_09

- Name: Rock 09
- Author(s): Jenelle van Heerden (All)
- URL: https://polyhaven.com/a/rock_09
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 1 variant, decimated.

### rock_07

- Name: Rock 07
- Author(s): Jenelle van Heerden (All)
- URL: https://polyhaven.com/a/rock_07
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 1 variant, decimated.

### namaqualand_boulders_01

- Name: Namaqualand Boulders 01
- Author(s): Greg Zaal (Photography), Jenelle van Heerden (modeling)
- URL: https://polyhaven.com/a/namaqualand_boulders_01
- Licence: CC0
- Retrieved: 2026-11-18
- Processing: 2 variants (rocks a, b), decimated.

## Processing common to all models

Done by tools/vegetation/build_vegetation.py (numpy + Pillow only):

- Geometry kept in the glTF frame (Y up, metres); node translations baked out; each variant lowered so its base is at y = 0 and centred on its base/bounding box in x/z.
- Three LODs per variant by the script's own decimation (half-edge quadric collapse on a position-welded graph with boundary and UV-seam protection; foliage cards thinned and scaled before that). Source meshes were up to 1.1 million triangles.
- Textures resized (box filter, linear light for albedo) to at most 1024 px and re-encoded as JPEG quality 85. Albedo is sRGB; normal maps (OpenGL, +Y up) and ARM-derived maps are linear data (ARM: R = ambient occlusion, G = roughness, B = 0).
- Cut-out materials carry a separate 8-bit linear alpha PNG (the sources have no opacity map); their colour background is flooded with edge colour.
- No source file is shipped, only the processed output. To regenerate: download the 1K glTF of each id from Poly Haven into <raw>/<id>/ and run `python3 -I tools/vegetation/build_vegetation.py <raw> assets/vegetation`.

Candidates fetched but not shipped: rock_face_01, quiver_tree_02, dead_quiver_trunk.
