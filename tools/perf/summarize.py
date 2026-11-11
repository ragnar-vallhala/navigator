#!/usr/bin/env python3
"""Summarize harness JSON-lines runs (NAVIGATOR_PROFILE_OUT) as markdown.

usage: summarize.py RUNS_DIR MODE LABEL [--phases a,b] [--series s1,s2,...]

Picks RUNS_DIR/MODE_LABEL_r*.jsonl and, per phase, prints each requested stat as
"mean over repetitions [min-max]". Stats of a series are the harness's own
(mean, p50, p95, p99, max, >33 ms and >50 ms counts) per run; the bracket is the
spread between repetitions, i.e. the run-to-run noise.
"""
import glob, json, os, statistics, sys

DEFAULT = [
    ("main.interval_ms", ["rate_hz", "p50", "p95", "p99", "max", "gt33", "gt50"]),
    ("down.interval_ms", ["rate_hz", "p99", "max", "gt33"]),
    ("drone3d.interval_ms", ["rate_hz", "p99", "max"]),
    ("evloop.lag_ms", ["p95", "p99", "max", "gt33"]),
    ("main.paint_cpu_ms", ["mean", "p99", "max"]),
    ("main.gpu_scene_ms", ["mean", "p99", "max"]),
    ("main.gpu_hud_ms", ["mean", "p99"]),
    ("main.hud_cpu_ms", ["mean", "max"]),
    ("main.paint_to_swap_ms", ["mean", "p99", "max"]),
    ("down.paint_cpu_ms", ["mean"]),
    ("down.gpu_scene_ms", ["mean"]),
]


def load(path):
    out = {"phases": {}, "meta": None, "hitches": [], "err": None}
    for l in open(path):
        if not l.strip():
            continue
        r = json.loads(l)
        if r["type"] == "phase":
            out["phases"][r["phase"]] = r
        elif r["type"] == "meta":
            out["meta"] = r
        elif r["type"] == "hitch":
            out["hitches"].append(r)
        elif r["type"] == "error":
            out["err"] = r
    return out


def fmt(vals, nd=2):
    if not vals:
        return "-"
    m = statistics.mean(vals)
    if len(vals) == 1:
        return f"{m:.{nd}f}"
    return f"{m:.{nd}f} [{min(vals):.{nd}f}-{max(vals):.{nd}f}]"


def thread_pct(ph, comm):
    return sum(t["pct"] for t in ph["threads"] if t["comm"] == comm)


def main():
    a = sys.argv[1:]
    d, mode, label = a[0], a[1], a[2]
    files = sorted(glob.glob(os.path.join(d, f"{mode}_{label}_r*.jsonl")))
    runs = [load(f) for f in files]
    runs = [r for r in runs if r["phases"]]
    if not runs:
        print("no runs", file=sys.stderr)
        return
    phases = []
    for r in runs:
        for p in r["phases"]:
            if p not in phases:
                phases.append(p)
    want = None
    if "--phases" in a:
        want = a[a.index("--phases") + 1].split(",")
    spec = DEFAULT
    if "--series" in a:
        spec = [(s, ["mean", "p99", "max"]) for s in a[a.index("--series") + 1].split(",")]
    gl = runs[0]["meta"]["gl"].get("renderer", "?") if runs[0]["meta"] else "?"
    print(f"### {mode} {label}  ({len(runs)} runs; {gl})\n")
    for ph in phases:
        if want and ph not in want:
            continue
        rs = [r["phases"][ph] for r in runs if ph in r["phases"]]
        print(f"**phase {ph}** ({fmt([x['dur_s'] for x in rs], 1)} s)\n")
        print("| metric | stat | value (mean [min-max] over runs) |\n|---|---|---|")
        for name, stats in spec:
            have = [x["series"][name] for x in rs if name in x["series"]]
            if not have:
                continue
            for st in stats:
                print(f"| {name} | {st} | {fmt([h[st] for h in have])} |")
        print(f"| main thread CPU % | | {fmt([x['main_cpu_pct'] for x in rs], 1)} |")
        print(f"| SimWorker threads CPU % | | {fmt([thread_pct(x, 'SimWorker') for x in rs], 1)} |")
        print(f"| process CPU % | | {fmt([x['proc_cpu_pct'] for x in rs], 1)} |")
        print(f"| RSS MB | | {fmt([x['rss_mb'] for x in rs], 0)} |")
        print()


if __name__ == "__main__":
    main()
