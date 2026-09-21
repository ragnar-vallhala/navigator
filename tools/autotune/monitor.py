# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Thread-safe shared state between the autotuner worker and the live plotter.

The optimization runs in a worker thread and pushes progress here; the
matplotlib dashboard (main thread) reads it plus live telemetry straight off
the SITL stack's sample deque.
"""

import math
import threading


class Monitor:
    def __init__(self, stack, names):
        self.stack = stack            # for live telemetry via stack.snapshot()
        self.names = names
        self._lock = threading.Lock()
        self.stop = threading.Event()  # set by the plotter on window close

        self.baseline = None
        self.optimizer = "-"
        self.current_x = None
        self.evals = []               # (global_index, cost, optimizer)
        self.best_cost = math.inf
        self.best_x = None
        self.best_opt = None
        self.done = False

    def set_baseline(self, c):
        with self._lock:
            self.baseline = c

    def set_optimizer(self, name):
        with self._lock:
            self.optimizer = name

    def set_current(self, x):
        with self._lock:
            self.current_x = list(x)

    def add_eval(self, cost):
        with self._lock:
            n = len(self.evals) + 1
            self.evals.append((n, cost, self.optimizer))
            if cost < self.best_cost:
                self.best_cost = cost
                self.best_x = list(self.current_x) if self.current_x else None
                self.best_opt = self.optimizer

    def finish(self):
        with self._lock:
            self.done = True

    def state(self):
        """Atomic snapshot of the scalar/aggregate fields for the plotter."""
        with self._lock:
            return {
                "baseline": self.baseline,
                "optimizer": self.optimizer,
                "current_x": list(self.current_x) if self.current_x else None,
                "evals": list(self.evals),
                "best_cost": self.best_cost,
                "best_x": list(self.best_x) if self.best_x else None,
                "best_opt": self.best_opt,
                "done": self.done,
                "names": self.names,
            }
