#!/usr/bin/env python3
"""Aggregate the harness's <out>.stacks file (NAVIGATOR_PROFILE_SAMPLE) by function.

usage: stacks.py FILE.stacks [--thread main|worker|pool|all] [--from MS] [--to MS]
                             [--top N] [--demangle]

Each sample is one CPU-time tick of one thread, so shares are shares of that
thread's CPU time. Frames of the Navigator binary are symbolized with addr2line
(needs the -g build the run used: pass --bin); library frames use the dynamic
symbol the harness recorded. Prints, per thread group, the top functions by
self time (leaf frame) and by inclusive time (anywhere on the stack).
"""
import argparse, collections, os, subprocess, sys


def parse(path):
    for line in open(path, errors="replace"):
        f = line.split()
        if len(f) < 4 or f[0] != "S":
            continue
        yield int(f[1]), int(f[2]), f[3], f[4:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--bin", default=None, help="Navigator binary (for addr2line)")
    ap.add_argument("--thread", default="all")
    ap.add_argument("--from", dest="t0", type=float, default=0)
    ap.add_argument("--to", dest="t1", type=float, default=1e18)
    ap.add_argument("--top", type=int, default=25)
    a = ap.parse_args()

    samples = [s for s in parse(a.file) if a.t0 <= s[1] <= a.t1]
    # thread groups: main = tid with the lowest id (the process), worker =
    # comm SimWorker, pool = other Navigator-named threads.
    tids = sorted({s[0] for s in samples})
    main_tid = tids[0] if tids else 0

    def group(s):
        if s[0] == main_tid:
            return "main"
        return "worker" if s[2].startswith("SimWorker") else "pool"

    # collect frames needing addr2line per module
    need = collections.defaultdict(set)
    for s in samples:
        for fr in s[3]:
            mod, off, name = (fr.split(":", 2) + ["", ""])[:3]
            if not name and mod != "?":
                need[mod].add(off)
    resolved = {}
    for mod, offs in need.items():
        if os.path.basename(mod) != "Navigator" and not a.bin:
            continue
        exe = a.bin if os.path.basename(mod) == "Navigator" and a.bin else mod
        if not os.path.exists(exe):
            continue
        offs = sorted(offs)
        try:
            out = subprocess.run(["addr2line", "-f", "-C", "-e", exe] + ["0x" + o for o in offs],
                                 capture_output=True, text=True, timeout=300).stdout.splitlines()
        except Exception:
            continue
        for i, o in enumerate(offs):
            if 2 * i < len(out):
                resolved[(mod, o)] = out[2 * i]

    def is_app(fr):
        return os.path.basename(fr.split(":", 1)[0]) == "Navigator"

    def sym(fr):
        mod, off, name = (fr.split(":", 2) + ["", ""])[:3]
        if is_app(fr):
            r = resolved.get((mod, off))
            return r if r and r != "??" else "Navigator+0x" + off
        # Library frames: only the nearest EXPORTED symbol is known (no debug
        # info installed), so the name can be a neighbour of the real function.
        return "%s!~%s" % (os.path.basename(mod).split(".so")[0], name or "0x" + off)

    def dso(fr):
        return os.path.basename(fr.split(":", 1)[0]).split(".so")[0]

    by = collections.defaultdict(list)
    for s in samples:
        by[group(s)].append(s)
    for g in ("main", "worker", "pool"):
        if a.thread not in ("all", g) or not by[g]:
            continue
        ss = by[g]
        selfc, incl, appincl, dsoc = (collections.Counter() for _ in range(4))
        for s in ss:
            syms = [sym(f) for f in s[3]]
            if not syms:
                continue
            selfc[syms[0]] += 1
            dsoc[dso(s[3][0])] += 1
            for x in set(syms):
                incl[x] += 1
            for x in {sym(f) for f in s[3] if is_app(f)}:
                appincl[x] += 1
        n = len(ss)
        print("== %s: %d samples (%.1f s of CPU at the sampling rate) ==" % (g, n, n / 100.0))
        print("-- self --")
        for k, v in selfc.most_common(a.top):
            print("%6.1f%%  %s" % (100.0 * v / n, k[:140]))
        print("-- self time by shared object (leaf frame) --")
        for k, v in dsoc.most_common(10):
            print("%6.1f%%  %s" % (100.0 * v / n, k))
        print("-- inclusive, Navigator's own frames only --")
        for k, v in appincl.most_common(a.top):
            print("%6.1f%%  %s" % (100.0 * v / n, k[:140]))


if __name__ == "__main__":
    main()
