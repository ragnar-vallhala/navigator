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
#include "AutotuneEngine.h"

#include "Cost.h" // kBig
#include "Optimizer.h"

using namespace autotune;

namespace {
QVector<double> toQv(const Vec &v) {
  QVector<double> q;
  q.reserve(int(v.size()));
  for (double x : v)
    q.push_back(x);
  return q;
}
} // namespace

AutotuneEngine::AutotuneEngine(bool tuneYaw, QString optimizer, int budget,
                               quint64 seed, Rollout rollout, bool fastRtos,
                               QObject *parent)
    : QObject(parent), m_space(tuneYaw, fastRtos),
      m_optimizer(std::move(optimizer)), m_budget(budget), m_seed(seed),
      m_rollout(std::move(rollout)) {
  // Registered so evaluated()/finished() can cross a thread boundary (the UI
  // runs the engine in a worker thread).
  qRegisterMetaType<QVector<double>>("QVector<double>");
}

QStringList AutotuneEngine::paramNames() const {
  QStringList n;
  for (const std::string &s : m_space.names())
    n << QString::fromStdString(s);
  return n;
}

void AutotuneEngine::run() {
  const QStringList names = paramNames();

  Evaluator ev(
      [this](const Vec &x) -> double {
        if (m_cancel)
          throw BudgetExhausted{};
        // nullopt (un-scorable rollout) -> divergence penalty, search moves on.
        return m_rollout(toQv(x)).value_or(kBig);
      },
      m_budget);

  ev.onEval = [this](const Vec &cur, const Vec &best, double cost,
                     double bestCost, int n) {
    emit evaluated(toQv(cur), toQv(best), cost, bestCost, n);
  };

  Rng rng(m_seed);
  // Qualify: an unqualified run() would recurse into this method.
  autotune::run(m_optimizer.toStdString(), ev, m_space.seed(), m_space.bounds(),
                rng);

  emit finished(ev.hasBest() ? toQv(ev.bestX()) : QVector<double>(), names,
                ev.hasBest() ? ev.best() : kBig);
}
