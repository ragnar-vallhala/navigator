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
#include "VehicleJson.h"

#include <QJsonArray>

namespace vsim {

namespace {

QJsonObject vec3ToJson(const QVector3D &v) {
  return QJsonObject{{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
}
QVector3D vec3FromJson(const QJsonValue &v, const QVector3D &def = {}) {
  if (!v.isObject())
    return def;
  const QJsonObject o = v.toObject();
  return QVector3D(o.value("x").toDouble(def.x()),
                   o.value("y").toDouble(def.y()),
                   o.value("z").toDouble(def.z()));
}

} // namespace

QJsonObject geometryToJson(const GeometryConfig &c) {
  QJsonObject root;
  root["format"] = "vayu-vehicle";
  root["version"] = 1;
  root["meshPath"] = c.meshPath;
  root["scale"] = c.scale;
  root["mass"] = c.mass;
  root["prop_radius"] = c.propRadius;
  root["translate"] = vec3ToJson(c.translate);
  root["rotate"] = vec3ToJson(c.rotate);
  root["com"] = vec3ToJson(c.com);
  QJsonArray inertia;
  for (float v : c.inertia)
    inertia.append(v);
  root["inertia"] = inertia;
  QJsonArray motors;
  for (const auto &m : c.motors) {
    motors.append(QJsonObject{{"pos", vec3ToJson(m.pos)},
                              {"axis", vec3ToJson(m.axis)},
                              {"spin", m.spin},
                              {"k_thrust", m.k_thrust},
                              {"k_moment", m.k_moment},
                              {"max_omega", m.max_omega},
                              {"tau", m.tau}});
  }
  root["motors"] = motors;
  return root;
}

GeometryConfig geometryFromJson(const QJsonObject &root) {
  GeometryConfig c; // defaults fill anything missing
  c.meshPath = root.value("meshPath").toString(c.meshPath);
  c.scale = root.value("scale").toDouble(c.scale);
  c.mass = root.value("mass").toDouble(c.mass);
  c.propRadius = root.value("prop_radius").toDouble(c.propRadius);
  c.translate = vec3FromJson(root.value("translate"), c.translate);
  c.rotate = vec3FromJson(root.value("rotate"), c.rotate);
  c.com = vec3FromJson(root.value("com"), c.com);
  const QJsonArray inertia = root.value("inertia").toArray();
  for (int i = 0; i < 9 && i < inertia.size(); ++i)
    c.inertia[i] = inertia[i].toDouble(c.inertia[i]);
  const QJsonArray motors = root.value("motors").toArray();
  for (int i = 0; i < 4 && i < motors.size(); ++i) {
    const QJsonObject m = motors[i].toObject();
    auto &mc = c.motors[i];
    mc.pos = vec3FromJson(m.value("pos"), mc.pos);
    mc.axis = vec3FromJson(m.value("axis"), mc.axis);
    mc.spin = m.value("spin").toInt(mc.spin);
    mc.k_thrust = m.value("k_thrust").toDouble(mc.k_thrust);
    mc.k_moment = m.value("k_moment").toDouble(mc.k_moment);
    mc.max_omega = m.value("max_omega").toDouble(mc.max_omega);
    mc.tau = m.value("tau").toDouble(mc.tau);
  }
  return c;
}

} // namespace vsim
