/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// GroundCover.h — ground-cover variety layered on a terrain's base colour
// (Qt-free, deterministic from one seed).
//
// The base colour (TerrainField::color / meadowColor) only knows height and
// slope, so a world came out one flat green. GroundCover adds, per world
// position: a large-scale warm/cool tint, patches of different cover, tracks,
// scattered rock, parcels of fields (ponds are WaterField basins),
// parcels of fields (or paving zones for the yard) and mown stripes. Which of
// those a world gets, and every colour, is a data table per palette id (see
// GroundCover.cpp), so the look of a preset is tuned there.
//
// sample() is a pure function of (seed, palette, wx, wy, base, h01, flatness),
// so adjacent chunks agree on shared edges exactly like the base colour does.
#pragma once

#include "Noise.h"
#include "ProcgenTypes.h"

#include <cstdint>

namespace vsim::procgen {

// Palette ids (FieldParams::palette). 0..4 predate the cover layer.
enum GroundPalette {
  kPalEndless = 0,
  kPalDesert = 1,
  kPalForest = 2,
  kPalPasture = 3,
  kPalYard = 4,
  kPalMeadow = 5,
  kPalAlpine = 6,
  kPalHills = 7,
  kPalFlat = 8,
  kPalRacing = 9,
};

class GroundCover {
public:
  GroundCover(uint32_t seed, int palette);

  // Pond look and amount of a palette. The ponds themselves are WaterField
  // basins now (a real dip in the terrain); this is only their data.
  struct PondStyle {
    float amount = 0.0f; // 0 = no ponds
    PgVec3 water, shore;
    float fadeH = 9.0f;  // cover fades out above this elevation fraction
  };
  static PondStyle pondStyle(int palette);
  // Whether the palette paints the scattered rock blobs into the ground colour
  // (alpine and forest only; elsewhere the real rock meshes stand alone).
  static bool paintsRock(int palette);

  struct Sample {
    PgVec3 color;
    // 1 = grass may grow here, 0 = path / water / rock / bare (CPU grass
    // scatter multiplies its density by this).
    float grass = 1.0f;
    // The same colour without the parcel-border track (equal to `color` where
    // there is none). Valid from sample() and sampleSplit(), but the edge fields
    // below only from sampleSplit(). The High shader draws the border track per
    // pixel on top of `split` (see EdgeTrack.h); `color` keeps it baked in.
    PgVec3 split;
    // Only from sampleSplit(): edgeS / edgeA are the signed / unsigned distance to the
    // nearest tracked border (kEdgeFar when none), edgePc the track colour and
    // edgeK how much of it the ground takes (flat, below the alpine line).
    // TerrainField::colorAtSplit folds edgeK in: edgePc is then the ground's
    // colour on a track, and the shader mixes it in by the per-pixel weight.
    float edgeS = 8.0f, edgeA = 8.0f, edgeK = 0.0f;
    PgVec3 edgePc;
  };

  // `base` is the height/slope colour, h01 the elevation as a fraction of the
  // field's heightM, flatness 1 = level .. 0 = vertical.
  Sample sample(float wx, float wy, const PgVec3 &base, float h01,
                float flatness) const;
  // As sample(), but the parcel-border tracks are left out of `color` and
  // returned as data for the shader to draw per pixel. `grass` is unchanged.
  Sample sampleSplit(float wx, float wy, const PgVec3 &base, float h01,
                     float flatness) const;

  // The field-parcel colour at a point on its own (no base, patches, rocks or
  // tracks): the parcel grid's warp, the blend between parcels and the variation
  // inside one. Black for a palette without field parcels. Exposed so a test can
  // check that no parcel border is a hard colour step.
  PgVec3 parcelColour(float wx, float wy) const;

private:
  Sample sampleImpl(float wx, float wy, const PgVec3 &base, float h01,
                    float flatness, bool split) const;
  int pal_;
  float rot_; // field-grid heading [rad], from the seed
  Noise tint_, patch_, fine_, path_, rock_;
};

} // namespace vsim::procgen
