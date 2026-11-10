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
#include "DroneModel.h"

#include <algorithm>
#include <cmath>

namespace vsim {

namespace {

constexpr float kPi = 3.14159265358979f;

struct V3 {
  float x = 0, y = 0, z = 0;
};
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
V3 norm(V3 a) {
  const float l = std::sqrt(dot(a, a));
  return l > 1e-12f ? a * (1.0f / l) : V3{0, 0, -1};
}

using Col = std::array<float, 3>;

// ---- palette -------------------------------------------------------------
const Col kCarbon = {0.13f, 0.13f, 0.15f};
const Col kCarbonTop = {0.22f, 0.22f, 0.25f};
const Col kBattery = {0.10f, 0.12f, 0.19f};
const Col kBatteryTop = {0.24f, 0.30f, 0.50f};
const Col kStrap = {0.95f, 0.52f, 0.10f};
const Col kFront = {1.00f, 0.25f, 0.06f}; // front accent (red-orange)
const Col kRear = {0.90f, 0.92f, 0.96f};  // rear accent (white)
const Col kBell = {0.30f, 0.32f, 0.36f};
const Col kAlu = {0.70f, 0.71f, 0.75f};
const Col kBoardFc = {0.05f, 0.45f, 0.20f};
const Col kBoardEsc = {0.08f, 0.08f, 0.09f};
const Col kLens = {0.08f, 0.16f, 0.45f};
const Col kBlade = {0.40f, 0.43f, 0.50f};
const Col kHub = {0.62f, 0.64f, 0.68f};

// ---- mesh builder (body frame, NED: up is -z; heights h are "up") ---------
struct B {
  std::vector<float> &d;
  void vert(V3 p, V3 n, const Col &c) {
    d.insert(d.end(), {p.x, p.y, p.z, n.x, n.y, n.z, c[0], c[1], c[2]});
  }
  void tri(V3 a, V3 b, V3 c, V3 n, const Col &col) {
    vert(a, n, col);
    vert(b, n, col);
    vert(c, n, col);
  }
  // Quad with the normal facing away from `inside`.
  void quad(V3 a, V3 b, V3 c, V3 e, const Col &col, V3 inside) {
    V3 n = norm(cross(b - a, c - a));
    if (dot(n, (a + b + c + e) * 0.25f - inside) < 0)
      n = n * -1.0f;
    tri(a, b, c, n, col);
    tri(a, c, e, n, col);
  }
  // Two-sided quad (thin sheets such as propeller blades).
  void sheet(V3 a, V3 b, V3 c, V3 e, const Col &col) {
    const V3 n = norm(cross(b - a, c - a));
    tri(a, b, c, n, col);
    tri(a, c, e, n, col);
    tri(a, c, b, n * -1.0f, col);
    tri(a, e, c, n * -1.0f, col);
  }
  // Hexahedron: corners 0..3 bottom loop, 4..7 top loop above them.
  void hexa(const V3 (&p)[8], const Col &col, const Col &top) {
    V3 ctr;
    for (auto &v : p)
      ctr = ctr + v * 0.125f;
    quad(p[0], p[1], p[2], p[3], col, ctr);
    quad(p[4], p[5], p[6], p[7], top, ctr);
    quad(p[0], p[1], p[5], p[4], col, ctr);
    quad(p[1], p[2], p[6], p[5], col, ctr);
    quad(p[2], p[3], p[7], p[6], col, ctr);
    quad(p[3], p[0], p[4], p[7], col, ctr);
  }
  void box(float cx, float cy, float hc, float hx, float hy, float hz,
           const Col &col, const Col &top) {
    auto P = [&](float sx, float sy, float sh) {
      return V3{cx + sx * hx, cy + sy * hy, -(hc + sh * hz)};
    };
    const V3 p[8] = {P(-1, -1, -1), P(1, -1, -1), P(1, 1, -1), P(-1, 1, -1),
                     P(-1, -1, 1),  P(1, -1, 1),  P(1, 1, 1),  P(-1, 1, 1)};
    hexa(p, col, top);
  }
  // Tapered arm from a to b (xy), centred on height 0 with thickness t.
  void arm(float ax, float ay, float bx, float by, float wa, float wb, float t,
           const Col &col, const Col &top) {
    V3 dir = norm(V3{bx - ax, by - ay, 0});
    const V3 pp{-dir.y, dir.x, 0};
    auto P = [&](float x, float y, float w, float s, float h) {
      return V3{x + pp.x * w * 0.5f * s, y + pp.y * w * 0.5f * s, -h};
    };
    const V3 p[8] = {P(ax, ay, wa, -1, -t / 2), P(ax, ay, wa, 1, -t / 2),
                     P(bx, by, wb, 1, -t / 2), P(bx, by, wb, -1, -t / 2),
                     P(ax, ay, wa, -1, t / 2),  P(ax, ay, wa, 1, t / 2),
                     P(bx, by, wb, 1, t / 2),  P(bx, by, wb, -1, t / 2)};
    hexa(p, col, top);
  }
  // Convex polygon (xy) extruded between heights h0 < h1.
  void prism(const std::vector<std::array<float, 2>> &poly, float cx, float cy,
             float h0, float h1, const Col &col, const Col &top) {
    const int n = int(poly.size());
    const V3 inside{cx, cy, -(h0 + h1) * 0.5f};
    for (int i = 0; i < n; ++i) {
      const auto &a = poly[i];
      const auto &b = poly[(i + 1) % n];
      const V3 a0{a[0], a[1], -h0}, b0{b[0], b[1], -h0};
      const V3 a1{a[0], a[1], -h1}, b1{b[0], b[1], -h1};
      quad(a0, b0, b1, a1, col, inside);
      tri(V3{cx, cy, -h1}, a1, b1, V3{0, 0, -1}, top);
      tri(V3{cx, cy, -h0}, b0, a0, V3{0, 0, 1}, col);
    }
  }
  // Vertical cylinder (smooth sides) between heights h0 < h1; r1 = top radius.
  void cyl(float cx, float cy, float h0, float h1, float r0, float r1, int seg,
           const Col &col, const Col &top) {
    for (int i = 0; i < seg; ++i) {
      const float a0 = 2 * kPi * i / seg, a1 = 2 * kPi * (i + 1) / seg;
      const V3 u0{std::cos(a0), std::sin(a0), 0}, u1{std::cos(a1), std::sin(a1), 0};
      const V3 b0{cx + u0.x * r0, cy + u0.y * r0, -h0};
      const V3 b1{cx + u1.x * r0, cy + u1.y * r0, -h0};
      const V3 t0{cx + u0.x * r1, cy + u0.y * r1, -h1};
      const V3 t1{cx + u1.x * r1, cy + u1.y * r1, -h1};
      vert(b0, u0, col);
      vert(b1, u1, col);
      vert(t1, u1, col);
      vert(b0, u0, col);
      vert(t1, u1, col);
      vert(t0, u0, col);
      tri(V3{cx, cy, -h1}, t0, t1, V3{0, 0, -1}, top);
      tri(V3{cx, cy, -h0}, b1, b0, V3{0, 0, 1}, col);
    }
  }
  // Hollow ring (duct) between heights h0 < h1.
  void ring(float cx, float cy, float h0, float h1, float rin, float rout,
            int seg, const Col &col, const Col &top) {
    for (int i = 0; i < seg; ++i) {
      const float a0 = 2 * kPi * i / seg, a1 = 2 * kPi * (i + 1) / seg;
      const V3 u0{std::cos(a0), std::sin(a0), 0}, u1{std::cos(a1), std::sin(a1), 0};
      auto P = [&](const V3 &u, float r, float h) {
        return V3{cx + u.x * r, cy + u.y * r, -h};
      };
      // outer wall
      vert(P(u0, rout, h0), u0, col);
      vert(P(u1, rout, h0), u1, col);
      vert(P(u1, rout, h1), u1, col);
      vert(P(u0, rout, h0), u0, col);
      vert(P(u1, rout, h1), u1, col);
      vert(P(u0, rout, h1), u0, col);
      // inner wall (normals inward)
      vert(P(u0, rin, h0), u0 * -1.0f, kCarbon);
      vert(P(u1, rin, h1), u1 * -1.0f, kCarbon);
      vert(P(u1, rin, h0), u1 * -1.0f, kCarbon);
      vert(P(u0, rin, h0), u0 * -1.0f, kCarbon);
      vert(P(u0, rin, h1), u0 * -1.0f, kCarbon);
      vert(P(u1, rin, h1), u1 * -1.0f, kCarbon);
      // top / bottom annulus
      tri(P(u0, rin, h1), P(u0, rout, h1), P(u1, rout, h1), V3{0, 0, -1}, top);
      tri(P(u0, rin, h1), P(u1, rout, h1), P(u1, rin, h1), V3{0, 0, -1}, top);
      tri(P(u0, rin, h0), P(u1, rout, h0), P(u0, rout, h0), V3{0, 0, 1}, col);
      tri(P(u0, rin, h0), P(u1, rin, h0), P(u1, rout, h0), V3{0, 0, 1}, col);
    }
  }
};

struct Style {
  float rref;                // prop radius this table row is drawn for [m]
  float armW0, armW1, armT;  // arm width at root / motor end, thickness
  float plateL, plateW, plateT;
  float stand;               // gap between bottom and top plate
  float battL, battW, battH; // battery on top
  float motorR, motorH;
  int blades;
  float chord;
  bool duct;
  bool gps;
};

Style styleOf(DroneClass c) {
  switch (c) {
  case DroneClass::Light3:
    return {0.0381f, 0.013f, 0.009f, 0.0030f, 0.075f, 0.034f, 0.0016f, 0.016f,
            0.060f,  0.022f, 0.020f,  0.0095f, 0.011f, 3,      0.0095f, false,
            false};
  case DroneClass::LongRange7:
    return {0.0889f, 0.024f, 0.015f, 0.0060f, 0.150f, 0.058f, 0.0030f, 0.030f,
            0.135f,  0.045f, 0.042f, 0.0165f, 0.019f, 2,      0.0190f, false,
            true};
  case DroneClass::Heavy10:
    return {0.1270f, 0.030f, 0.019f, 0.0080f, 0.190f, 0.075f, 0.0040f, 0.036f,
            0.150f,  0.055f, 0.050f, 0.0225f, 0.026f, 2,      0.0260f, false,
            true};
  case DroneClass::Whoop65:
    return {0.0155f, 0.0050f, 0.0050f, 0.0016f, 0.036f, 0.024f, 0.0016f, 0.0f,
            0.028f,  0.010f,  0.008f,  0.0045f, 0.005f, 4,      0.0048f, true,
            false};
  case DroneClass::Freestyle5:
  default:
    return {0.0622f, 0.020f, 0.013f, 0.0045f, 0.115f, 0.052f, 0.0022f, 0.026f,
            0.105f,  0.034f, 0.030f, 0.0138f, 0.015f, 3,      0.0145f, false,
            false};
  }
}

bool finite3(const std::array<float, 3> &p) {
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

// Build one propeller (blade count n, radius r) with its hub at the origin.
ModelMesh makeProp(const Style &S, float k, float r, int spin, bool front) {
  ModelMesh m;
  B b{m.data};
  const int n = S.blades;
  const float rh = 0.14f * r;
  const float chordProfile[7] = {0.55f, 0.85f, 1.0f, 1.0f, 0.90f, 0.72f, 0.50f};
  const Col tip = propTipColor(spin);
  const Col band = {1.0f, 0.18f, 0.08f};
  for (int bi = 0; bi < n; ++bi) {
    const float psi0 = 2 * kPi * bi / n;
    V3 le[7], te[7];
    for (int j = 0; j < 7; ++j) {
      static const float kStation[7] = {0.0f, 0.2f, 0.4f, 0.6f, 0.78f, 0.9f, 1.0f};
      const float f = kStation[j];
      const float rj = rh + (r - rh) * f;
      const float psi = psi0 - float(spin) * 0.17f * f * f; // swept back
      const V3 pc{rj * std::cos(psi), -rj * std::sin(psi), 0};
      const V3 travel =
          norm(V3{-std::sin(psi), -std::cos(psi), 0}) * float(spin);
      const float c = S.chord * k * chordProfile[j];
      const float pitch = (28.0f - 16.0f * f) * kPi / 180.0f;
      const float dz = 0.5f * c * std::sin(pitch);
      le[j] = pc + travel * (0.40f * c) + V3{0, 0, +dz}; // leading edge low
      te[j] = pc - travel * (0.60f * c) + V3{0, 0, -dz};
    }
    for (int j = 0; j < 6; ++j) {
      Col c = kBlade;
      if (j >= 4)
        c = tip;
      else if (front && j == 2)
        c = band;
      b.sheet(le[j], te[j], te[j + 1], le[j + 1], c);
    }
  }
  b.cyl(0, 0, -0.0010f * k, 0.0075f * k, rh * 0.85f, rh * 0.55f, 10, kHub,
        kHub);
  return m;
}

} // namespace

std::array<float, 3> propTipColor(int spinSign) {
  return spinSign >= 0 ? std::array<float, 3>{1.0f, 0.85f, 0.10f}  // CCW yellow
                       : std::array<float, 3>{0.10f, 0.85f, 1.0f}; // CW cyan
}

DroneClass droneClassFor(const std::string &id, float r) {
  if (id == "5in")
    return DroneClass::Freestyle5;
  if (id == "3in")
    return DroneClass::Light3;
  if (id == "7in")
    return DroneClass::LongRange7;
  if (id == "10in")
    return DroneClass::Heavy10;
  if (id == "whoop65")
    return DroneClass::Whoop65;
  if (!(r > 0.0f))
    return DroneClass::Freestyle5;
  if (r < 0.025f)
    return DroneClass::Whoop65;
  if (r < 0.050f)
    return DroneClass::Light3;
  if (r < 0.075f)
    return DroneClass::Freestyle5;
  if (r < 0.108f)
    return DroneClass::LongRange7;
  return DroneClass::Heavy10;
}

DroneModel buildDroneModel(const DroneModelInput &in) {
  const Style S = styleOf(in.cls);
  float r = (std::isfinite(in.propRadius) && in.propRadius > 0.0f)
                ? in.propRadius
                : S.rref;
  r = std::clamp(r, 0.004f, 0.5f);

  // ---- sanitise the layout ----
  auto pos = in.motorPos;
  bool ok = true;
  for (auto &p : pos)
    ok = ok && finite3(p);
  float minPair = 1e30f;
  if (ok)
    for (int i = 0; i < 4; ++i)
      for (int j = i + 1; j < 4; ++j)
        minPair = std::min(minPair,
                           std::hypot(pos[i][0] - pos[j][0], pos[i][1] - pos[j][1]));
  if (!ok || minPair < 1e-3f) {
    const float p = 1.39f * r; // the 5-inch ratio
    pos = {{{p, p, 0}, {-p, p, 0}, {-p, -p, 0}, {p, -p, 0}}};
    minPair = 2 * p;
  }
  for (auto &p : pos)
    for (float &v : p)
      v = std::clamp(v, -2.0f, 2.0f);

  // Blades must not touch their neighbours (a duct needs extra margin).
  const float ductK = S.duct ? 1.22f : 1.0f;
  const float vr = std::min(r, 0.485f * minPair / ductK);
  const float k = vr / S.rref;

  float cx = 0, cy = 0;
  for (auto &p : pos) {
    cx += p[0] * 0.25f;
    cy += p[1] * 0.25f;
  }
  float dmin = 1e30f;
  for (auto &p : pos)
    dmin = std::min(dmin, std::hypot(p[0] - cx, p[1] - cy));
  dmin = std::max(dmin, 0.01f);

  DroneModel M;
  M.visualPropRadius = vr;
  M.bladeCount = S.blades;
  B b{M.body.data};

  const float T = S.armT * k;
  const float pT = std::max(S.plateT * k, 0.0006f);
  const float stand = S.stand * k;
  const float L = std::min(S.plateL * k, 1.7f * dmin);
  const float W = std::min(S.plateW * k, L * 0.55f);
  const float motorR = S.motorR * k, motorH = S.motorH * k;

  // ---- arms (root hidden under the plate), accent on the outer part ----
  for (int i = 0; i < 4; ++i) {
    const float mx = pos[i][0], my = pos[i][1];
    const bool front = mx > cx;
    const float ax = cx, ay = cy;
    const float w0 = S.armW0 * k, w1 = S.armW1 * k;
    const float tcut = 0.62f;
    const float wc = w0 + (w1 - w0) * tcut;
    const float bx = ax + (mx - ax) * tcut, by = ay + (my - ay) * tcut;
    b.arm(ax, ay, bx, by, w0, wc, T, kCarbon, kCarbonTop);
    b.arm(bx, by, mx, my, wc, w1, T, front ? kFront : kRear,
          front ? kFront : kRear);
  }

  // ---- centre plate (sits on the arms) ----
  const float plateLo = T / 2, plateHi = T / 2 + pT;
  auto octo = [&](float l, float w) {
    return std::vector<std::array<float, 2>>{
        {cx + l / 2, cy - 0.28f * w}, {cx + 0.28f * l, cy - 0.5f * w},
        {cx - 0.28f * l, cy - 0.5f * w}, {cx - l / 2, cy - 0.28f * w},
        {cx - l / 2, cy + 0.28f * w}, {cx - 0.28f * l, cy + 0.5f * w},
        {cx + 0.28f * l, cy + 0.5f * w}, {cx + l / 2, cy + 0.28f * w}};
  };
  b.prism(octo(L, W), cx, cy, plateLo, plateHi, kCarbon, kCarbonTop);

  float topHi = plateHi; // top of whatever the battery stands on
  if (!S.duct) {
    const float topLo = plateHi + stand;
    const float topH = topLo + pT;
    // stack: ESC + FC boards and four standoffs
    const float bs = std::min(0.036f * k, W * 0.9f);
    b.box(cx, cy, plateHi + 0.28f * stand, bs / 2, bs / 2, 0.0009f * k, kBoardEsc,
          kBoardEsc);
    b.box(cx, cy, plateHi + 0.66f * stand, bs / 2, bs / 2, 0.0009f * k, kBoardFc,
          kBoardFc);
    const float so = std::min(0.0153f * k, W * 0.40f);
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2)
        b.cyl(cx + sx * so, cy + sy * so, plateHi, topLo, 0.0018f * k,
              0.0018f * k, 8, kAlu, kAlu);
    b.prism(octo(L * 0.80f, W * 0.84f), cx, cy, topLo, topH, kCarbon,
            kCarbonTop);
    topHi = topH;

    // battery, strapped, slightly aft
    const float bl = std::min(S.battL * k, L * 1.05f);
    const float bw = std::min(S.battW * k, W * 0.9f);
    const float bh = S.battH * k;
    const float bcx = cx - 0.04f * L;
    b.box(bcx, cy, topHi + bh / 2, bl / 2, bw / 2, bh / 2, kBattery, kBatteryTop);
    for (int s = -1; s <= 1; s += 2) {
      const float sxp = bcx + s * 0.28f * bl;
      b.box(sxp, cy, topLo + (bh + (topHi - topLo)) / 2 + 0.0004f * k,
            0.0042f * k, bw / 2 + 0.0007f * k,
            (bh + (topHi - topLo)) / 2 + 0.0007f * k, kStrap, kStrap);
    }
    // camera: orange TPU mount, black body, blue lens, at the front
    const float camH = stand + 2 * pT;
    const float camX = cx + L / 2 - 0.004f * k;
    const float camHalfY = std::min(0.0155f * k, W * 0.5f);
    b.box(camX, cy, plateLo + camH / 2, 0.0085f * k, camHalfY, camH / 2, kFront,
          kFront);
    b.box(camX + 0.0095f * k, cy, plateLo + camH / 2, 0.0045f * k,
          camHalfY * 0.62f, camH * 0.36f, kBoardEsc, kBoardEsc);
    b.box(camX + 0.0142f * k, cy, plateLo + camH / 2, 0.0006f * k,
          camHalfY * 0.36f, camH * 0.2f, kLens, kLens);
    // GPS puck on a mast at the rear (long range / heavy)
    if (S.gps) {
      const float gx = cx - L / 2 + 0.012f * k;
      b.cyl(gx, cy, topHi, topHi + 0.030f * k, 0.0018f * k, 0.0018f * k, 8,
            kAlu, kAlu);
      b.cyl(gx, cy, topHi + 0.030f * k, topHi + 0.035f * k, 0.014f * k,
            0.0125f * k, 14, kRear, kRear);
    }
  } else {
    // whoop: flat body, tiny battery on top, camera in front
    const float bl = std::min(S.battL * k, L), bw = std::min(S.battW * k, W * 0.8f);
    const float bh = S.battH * k;
    b.box(cx, cy, plateHi + 0.0009f * k + bh / 2, bl / 2, bw / 2, bh / 2, kBattery,
          kBatteryTop);
    b.box(cx, cy, plateHi + 0.0006f * k, 0.011f * k, 0.008f * k, 0.0006f * k,
          kBoardFc, kBoardFc);
    b.box(cx + L / 2 - 0.001f * k, cy, plateHi + 0.0055f * k, 0.0045f * k,
          0.0058f * k, 0.0045f * k, kFront, kFront);
    b.box(cx + L / 2 + 0.0036f * k, cy, plateHi + 0.0055f * k, 0.0008f * k,
          0.0035f * k, 0.0028f * k, kLens, kLens);
  }

  // ---- motors, ducts, hubs, props ----
  for (int i = 0; i < 4; ++i) {
    const float mx = pos[i][0], my = pos[i][1];
    const bool front = mx > cx;
    const Col accent = front ? kFront : kRear;
    b.cyl(mx, my, -T / 2 - 0.004f * k, -T / 2, motorR * 0.8f, motorR * 0.8f, 14,
          kAlu, kAlu);
    b.cyl(mx, my, T / 2, T / 2 + motorH - 0.0022f * k, motorR, motorR, 18, kBell,
          kBell);
    b.cyl(mx, my, T / 2 + motorH - 0.0022f * k, T / 2 + motorH, motorR, motorR,
          18, accent, accent);
    if (S.duct) {
      const float dh = 0.55f * vr;
      b.ring(mx, my, -0.45f * dh, 0.55f * dh, 1.08f * vr, 1.20f * vr, 28,
             kCarbonTop, accent);
    }
    const float hubUp = T / 2 + motorH + 0.0015f * k;
    M.hub[i] = {mx, my, pos[i][2] - hubUp};
    M.prop[i] = makeProp(S, k, vr, in.spin[i] >= 0 ? +1 : -1, front);
  }

  // ---- bounds (body + props placed at hubs) ----
  M.bmin = {1e30f, 1e30f, 1e30f};
  M.bmax = {-1e30f, -1e30f, -1e30f};
  auto grow = [&](float x, float y, float z) {
    M.bmin = {std::min(M.bmin[0], x), std::min(M.bmin[1], y),
              std::min(M.bmin[2], z)};
    M.bmax = {std::max(M.bmax[0], x), std::max(M.bmax[1], y),
              std::max(M.bmax[2], z)};
  };
  for (size_t v = 0; v < M.body.vertexCount(); ++v)
    grow(M.body.data[v * 9], M.body.data[v * 9 + 1], M.body.data[v * 9 + 2]);
  for (int i = 0; i < 4; ++i)
    for (size_t v = 0; v < M.prop[i].vertexCount(); ++v)
      grow(M.hub[i][0] + M.prop[i].data[v * 9], M.hub[i][1] + M.prop[i].data[v * 9 + 1],
           M.hub[i][2] + M.prop[i].data[v * 9 + 2]);
  return M;
}

float advanceRotorAngle(float angle, int spinSign, float omega, int blades,
                        float dt) {
  const int nb = std::max(1, blades);
  const float dtc = std::clamp(dt, 1.0f / 240.0f, 0.05f);
  const float limitRps = 0.4f / (float(nb) * dtc); // revolutions / s
  const float rps = std::fabs(omega) / (2.0f * float(M_PI));
  const float vis = limitRps * std::tanh(rps / limitRps);
  const float dir = spinSign >= 0 ? 1.0f : -1.0f;
  return std::fmod(angle + dir * vis * 2.0f * float(M_PI) * dt,
                   2.0f * float(M_PI));
}

} // namespace vsim
