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
// Tree colliders against the REAL engine module: the ENV frames SimWorker sends
// (BEGIN, ADD x N, COMMIT / CLEAR) must make a trunk solid, leave a line beside
// it free, and stop being solid when cleared.
//
// The drone is driven ballistically, which needs no RC and no firmware flight:
// gravity off, no ground plane, the airframe reset at (0, y, -5) with 8 m/s
// along +x. The firmware stays disarmed (zero duty), so the only forces are the
// engine's drag and any contact. Each flight is stepped 2500 times (2.5 s of
// engine time, wall-clock paced) and the pose is read after every step.
//
// Geometry, so the offsets are not magic numbers. The default airframe's contact
// footprint is four points at 0.0866 m east/north of the centre pushed out to
// the prop tip: sqrt(2) * 0.0866 = 0.1225 m hub radius plus (prop radius 0.0635
// - contact radius 0.04) = 0.1460 m, and each point is a sphere of 0.04 m, so
// whatever its yaw the airframe reaches at most 0.186 m from its centre. The
// trunk is 0.3 m in radius, so a line passes it free when it keeps more than
// 0.3 + 0.186 = 0.486 m from the axis. 1.0 m is that plus a 0.51 m margin: it
// cannot touch by design, and is still close enough that a collider the wrong
// size or place (a radius taken as a diameter, an axis off by half a metre)
// would show.
//
// Needs the engine module (VAYU_SITL_MODULE) and an SDK with the ENV messages;
// without either the test skips and says why (an SDK older than v0.3.0 has no
// ENV messages; the pinned v0.3.0 has them, so CI runs the flights).

#include "SimWorker.h"
#include "SitlModule.h"
#include "VfsTestEnv.h"
#include "procgen/TreeColliderWindow.h"

#include <QSignalSpy>
#include <QtTest>

#include <cmath>
#include <vector>

using namespace vsim::procgen;

namespace {

constexpr float kTrunkR = 0.3f;
constexpr float kTrunkH = 10.0f;
constexpr float kTrunkX = 3.0f;
constexpr float kLaneZ = -5.0f; // mid-height of the trunk (ground at z = 0)
constexpr float kV0 = 8.0f;
constexpr int kSteps = 2500;
constexpr float kReach = 0.186f; // see the file header

void noSink(void *, const uint8_t *, size_t) {}

bool readPose(const vayu_sitl_api_t *a, vsim_pose_frame_t &p) {
#if VAYU_SITL_ABI_VERSION >= 7
  return a->get_pose(&p, sizeof p) > 0;
#else
  a->get_pose(&p);
  return true;
#endif
}

struct Flight {
  float maxX = -1e9f, finalX = 0, finalVx = 0, minVx = 1e9f, maxVx = -1e9f;
  float closest = 1e9f; // smallest distance of the centre to the trunk axis
  float xAtMinVx = 0;
  float contactX = 0; // x where vx first dropped 20% under the approach speed
  float maxAbsY = 0;
  float maxAbsDz = 0; // vertical drift from the lane
  int steps = 0;
};

// A trunk standing on the ground at (x, y) as the window builds it from the
// scatter output, and the wire ops SimWorker queues for it.
std::vector<std::vector<uint8_t>> trunkAdds(float x, float y) {
  const ColliderWindow w = treeColliderWindow(
      {VegCollider{PgVec3{x, y, 0.0f}, kTrunkR, kTrunkH, false}}, PgVec3{0, 0, 0}, 0.0f);
  return encodeEnvAdds(w.cylinders);
}

Flight fly(const vayu_sitl_api_t *api, float laneY, float trunkY, float laneZ = kLaneZ) {
  vsim_ctl_reset_t r{};
  r.pos_w[0] = 0.0f;
  r.pos_w[1] = laneY;
  r.pos_w[2] = laneZ;
  r.quat_wxyz[0] = 1.0f;
  r.vel_w[0] = kV0;
  api->reset_to(&r);
  Flight f;
  bool contact = false;
  float prevVx = kV0;
  for (int i = 0; i < kSteps; ++i) {
    api->run_step();
    vsim_pose_frame_t p{};
    if (!readPose(api, p))
      break;
    ++f.steps;
    const float x = p.pos_w[0], y = p.pos_w[1], vx = p.vel_w[0];
    f.maxX = std::max(f.maxX, x);
    f.maxVx = std::max(f.maxVx, vx);
    if (vx < f.minVx) {
      f.minVx = vx;
      f.xAtMinVx = x;
    }
    f.maxAbsY = std::max(f.maxAbsY, std::fabs(y - laneY));
    f.maxAbsDz = std::max(f.maxAbsDz, std::fabs(p.pos_w[2] - laneZ));
    f.closest = std::min(f.closest, std::hypot(x - kTrunkX, y - trunkY));
    if (!contact && i > 0 && vx < prevVx - 0.2f) { // drag takes ~0.001 per step
      contact = true;
      f.contactX = x;
    }
    prevVx = vx;
    f.finalX = x;
    f.finalVx = vx;
  }
  return f;
}

void report(const char *what, const Flight &f) {
  qInfo("%s: steps %d, max x %.3f, closest to axis %.3f, first contact x %.3f, "
        "vx max %.2f min %.2f (at x %.2f) final %.2f, final x %.3f, max |dy| %.3f",
        what, f.steps, f.maxX, f.closest, f.contactX, f.maxVx, f.minVx, f.xAtMinVx,
        f.finalVx, f.finalX, f.maxAbsY);
}

} // namespace

class TstTreeCollidersEngine : public QObject {
  Q_OBJECT
private slots:
  void skipsWithoutEnvSupport() {
    // Documents the gate: on an SDK without the ENV messages the sender is off.
    if (vsim::SimWorker::envCollidersAvailable())
      QSKIP("this SDK has the ENV messages; the engine flights below run");
    vsim::SimWorker w;
    QSignalSpy log(&w, &vsim::SimWorker::logLine);
    ColliderWindow win;
    win.cylinders.push_back(EngineCylinder{});
    w.sendTreeColliders(win);
    w.sendTreeColliders(win);
    QCOMPARE(log.count(), 1); // said once, never silently dropped
    QVERIFY(log.first().first().toString().contains("disabled"));
    QVERIFY(!vsim::SimWorker::applyEnvClear(nullptr));
  }

  void capSendsExactly4096NearestAndLogsTheDrop() {
    if (!vsim::SimWorker::envCollidersAvailable())
      QSKIP("SDK has no ENV messages: tree colliders are off");
    // 5000 distinct trunks 1.0 .. 148.5 m from the drone, all inside the window.
    std::vector<VegCollider> in;
    for (int i = 0; i < 5000; ++i)
      in.push_back(VegCollider{PgVec3{1.0f + 0.0295f * float(i), 0.0f, 0.0f}, 0.3f, 10.0f, false});
    const ColliderWindow w = treeColliderWindow(in, PgVec3{0, 0, 0}, 0.0f);
    QCOMPARE(int(w.cylinders.size()), 4096);
    QCOMPARE(int(w.dropped), 5000 - 4096);
    // The nearest are the ones kept: the farthest kept is trunk #4095.
    QCOMPARE(w.cylinders.back().center.x, in[4095].pos.x);
    const auto adds = encodeEnvAdds(w.cylinders);
    size_t total = 0;
    for (const auto &m : adds) {
      uint32_t c;
      memcpy(&c, m.data(), 4);
      total += c;
    }
    QCOMPARE(int(total), 4096);
    QCOMPARE(int(adds.size()), 342);

    vsim::SimWorker sim;
    QSignalSpy log(&sim, &vsim::SimWorker::logLine);
    sim.sendTreeColliders(w);
    QCOMPARE(log.count(), 1);
    const QString line = log.first().first().toString();
    QVERIFY2(line.contains("904") && line.contains("4096"), qPrintable(line));
    // A window that fits logs nothing.
    log.clear();
    ColliderOptions o;
    o.cap = 5000;
    sim.sendTreeColliders(treeColliderWindow(in, PgVec3{0, 0, 0}, 0.0f, o));
    QCOMPARE(log.count(), 0);
  }

  void trunkIsSolidBesideIsFreeAndClearedIsGone() {
    testenv::isolateVfs();
    if (!vsim::SimWorker::envCollidersAvailable())
      QSKIP("SDK has no ENV messages: tree colliders are off");
    if (!SitlModule::instance().available())
      QSKIP(qPrintable("no SITL engine module: " + SitlModule::instance().error()));
    const vayu_sitl_api_t *api = SitlModule::instance().api();
    qInfo("engine build: %s, ABI %u", api->build_id ? api->build_id : "?", unsigned(api->abi_version));

    QCOMPARE(api->boot(&noSink, nullptr), 0);
    vsim_ctl_world_t world{};
    world.gravity = 0.0f;
    world.ground_z = INFINITY; // no ground plane
    world.restitution = 0.3f;
    world.linear_drag = 0.10f;
    world.angular_drag = 0.00125f;
    api->set_world(&world);
    vsim_ctl_testrig_t rig{}; // enable 0: free flight
    api->set_testrig(&rig);
    api->clear_obstacles();
    api->run_begin();

    // Control: nothing in the way.
    vsim::SimWorker::applyEnvClear(api);
    const Flight control = fly(api, 0.0f, 0.0f);
    report("control (no trunk)", control);
    QCOMPARE(control.steps, kSteps);
    QVERIFY(control.finalX > kTrunkX + 5.0f); // coasts well past where the trunk will be
    QVERIFY(control.maxAbsY < 0.05f);

    // (i) Head-on at the axis: the trunk must stop it.
    QVERIFY(vsim::SimWorker::applyEnvSet(api, trunkAdds(kTrunkX, 0.0f), encodeEnvCommit()));
    const Flight head = fly(api, 0.0f, 0.0f);
    report("(i) head-on", head);
    QVERIFY2(head.maxX < kTrunkX, "the drone's centre reached the trunk axis");
    QVERIFY2(head.maxX < kTrunkX - kTrunkR, "the centre went into the trunk's body");
    // The engine's response is a stop, not a rebound: each contact point takes a
    // quarter of the impulse, so vx falls to zero and the drone rests against the
    // trunk. The test pins "stopped or bounced back", not which.
    QVERIFY2(head.finalVx < 0.5f * kV0, "speed along the line did not drop");
    QVERIFY2(head.finalVx > -kV0, "the drone left faster than it came");
    QVERIFY2(head.contactX > 0.0f && head.contactX < kTrunkX - kTrunkR, "no contact before the trunk");
    QVERIFY(head.finalX < control.finalX - 5.0f);

    // (ii) A line 1.0 m beside the axis: untouched, same as the control.
    QVERIFY(vsim::SimWorker::applyEnvSet(api, trunkAdds(kTrunkX, 1.0f), encodeEnvCommit()));
    const Flight beside = fly(api, 0.0f, 1.0f);
    report("(ii) 1.0 m beside", beside);
    QVERIFY2(beside.closest > kTrunkR + kReach, "the line is not clear of the trunk by design");
    QVERIFY(beside.closest > 0.95f);
    QVERIFY2(std::fabs(beside.finalX - control.finalX) < 0.05f,
             "flying past the trunk changed the trajectory");
    QVERIFY(beside.minVx > 0.0f);
    QVERIFY(beside.maxAbsY < 0.05f);

    // (iii) Trees off: the trunk is ON the line, then cleared.
    QVERIFY(vsim::SimWorker::applyEnvSet(api, trunkAdds(kTrunkX, 0.0f), encodeEnvCommit()));
    QVERIFY(vsim::SimWorker::applyEnvClear(api));
    const Flight off = fly(api, 0.0f, 0.0f);
    report("(iii) trunk on the line, then ENV_CLEAR", off);
    QVERIFY2(off.closest < 0.1f, "the line did not go through the trunk's axis");
    QVERIFY2(std::fabs(off.finalX - control.finalX) < 0.05f, "a cleared trunk still blocked");
    QVERIFY(off.minVx > 0.0f);

    // Replacing a set replaces it: a second COMMIT drops the first set's trunk.
    QVERIFY(vsim::SimWorker::applyEnvSet(api, trunkAdds(kTrunkX, 0.0f), encodeEnvCommit()));
    QVERIFY(vsim::SimWorker::applyEnvSet(api, trunkAdds(kTrunkX, 20.0f), encodeEnvCommit()));
    const Flight swapped = fly(api, 0.0f, 0.0f);
    report("(iv) set replaced by one far away", swapped);
    QVERIFY(std::fabs(swapped.finalX - control.finalX) < 0.05f);

    // The reach used to pick the 1.0 m offset is real: 0.40 m (inside r + reach
    // = 0.486) is blocked, 0.58 m (outside) is free.
    QVERIFY(vsim::SimWorker::applyEnvSet(api, trunkAdds(kTrunkX, 0.40f), encodeEnvCommit()));
    const Flight graze = fly(api, 0.0f, 0.40f);
    report("(v) 0.40 m beside (inside r + reach)", graze);
    QVERIFY2(graze.finalX < control.finalX - 1.0f, "a line inside r + reach was not blocked");
    QVERIFY(vsim::SimWorker::applyEnvSet(api, trunkAdds(kTrunkX, 0.58f), encodeEnvCommit()));
    const Flight clear = fly(api, 0.0f, 0.58f);
    report("(v) 0.58 m beside (outside r + reach)", clear);
    QVERIFY2(std::fabs(clear.finalX - control.finalX) < 0.05f, "a line outside r + reach was blocked");

    // Over the top. The trunk stands on the ground (base z = 0 NED, the same
    // reference as above), 10 m tall: its top is at z = -10. Gravity is off, as
    // in every flight here, so the drone holds its altitude. The engine's top
    // rule (resolveEnvObstacles): a footprint point touches when its horizontal
    // distance to the axis is under r + 0.04 AND |z - centre z| is under
    // h/2 + 0.04, i.e. its z is within 0.04 m of the top. The footprint points
    // sit at the body origin's height when the airframe is level, so the
    // airframe's 0.186 m reach is HORIZONTAL only: vertically it reaches 0.04 m.
    // Expectation: 0.2 m above the top is NOT blocked, nor anything above 0.04 m.
    QVERIFY(vsim::SimWorker::applyEnvSet(api, trunkAdds(kTrunkX, 0.0f), encodeEnvCommit()));
    const float top = -kTrunkH; // NED z of the top
    QCOMPARE(top, -10.0f);
    const Flight over = fly(api, 0.0f, 0.0f, top - 1.0f);
    report("(vi) 1.0 m above the top (z -11.0)", over);
    QVERIFY2(std::fabs(over.finalX - control.finalX) < 0.05f, "flying 1.0 m over the top was blocked");
    QVERIFY(over.minVx > 0.0f);
    // Sweep the clearance above the top to find the real threshold.
    float firstClear = -1.0f;
    float lastBlocked = -1.0f;
    for (float d : {0.0f, 0.02f, 0.03f, 0.04f, 0.05f, 0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f}) {
      const Flight fl = fly(api, 0.0f, 0.0f, top - d);
      const bool blocked = std::fabs(fl.finalX - control.finalX) > 0.05f;
      qInfo("(vii) %.2f m above the top (z %.2f): %s, final x %.3f, max x %.3f, vx min %.2f, vertical drift %.3f",
            d, top - d, blocked ? "BLOCKED" : "clear", fl.finalX, fl.maxX, fl.minVx, fl.maxAbsDz);
      if (blocked)
        lastBlocked = d;
      else if (firstClear < 0.0f)
        firstClear = d;
    }
    qInfo("(vii) threshold: last blocked at %.2f m, first clear at %.2f m above the top", lastBlocked, firstClear);
    // Pin what the engine does: blocked inside its 0.04 m vertical contact
    // radius, free beyond it, and 0.2 m (the airframe's horizontal reach) is free.
    QVERIFY2(lastBlocked >= 0.0f, "nothing was blocked even touching the top");
    QVERIFY2(lastBlocked <= 0.04f, "blocked above the engine's 0.04 m vertical contact radius");
    QVERIFY2(firstClear > lastBlocked && firstClear <= 0.05f, "the clear region does not start at the contact radius");

    vsim::SimWorker::applyEnvClear(api);
    api->set_telemetry_sink(nullptr, nullptr);
  }
};

QTEST_MAIN(TstTreeCollidersEngine)
#include "tst_treecolliders_engine.moc"
