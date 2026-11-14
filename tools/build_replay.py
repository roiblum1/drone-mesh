#!/usr/bin/env python3
"""Build the replay viewer page from simulation traces (sim/pack_sim --trace).

Usage:
  python3 -I tools/build_replay.py OUT.html TRACE.json [TRACE.json ...] [--fragment]

Each trace becomes a scenario the page can switch to. By default OUT.html is a
standalone document; --fragment leaves out the doctype and head so the page can
be published where a host supplies them.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

CLASS = {0: "unknown source", 1: "RF emitter", 2: "anomaly", 3: "thermal hazard", 4: "distress beacon"}


def describe(tr):
    n = len(tr["drones"])
    active = [f"D{d['id']}" for d in tr["drones"] if d["active"]]
    flown = (" and ".join(active) + (" is" if len(active) == 1 else " are") + " flown by the operator; the rest") if active else "None is flown by the operator; all"
    real = [e for e in tr["emitters"] if e["alertable"]]
    kinds = ", ".join(sorted({CLASS.get(e["cls"], "source") for e in real}))
    faults = sum(1 for _, line in tr["log"] if line.startswith("FAULT"))
    lede = (
        f"{n} drones for {tr['duration']:.0f} s (seed {tr['seed']}). {flown} "
        f"fly the route and work as one sensor. {len(real)} real sources to find ({kinds}), "
        f"{len(tr['emitters']) - len(real)} that must stay quiet, {faults} injected faults. "
        "Bearings are drawn as the drones report them; the discs are the pack's own picture."
    )
    return lede


def compact(tr):
    # Round what the page draws: metres to the metre, degrees to 0.1.
    for f in tr["frames"]:
        f["d"] = [[int(d[0]), round(d[1]), round(d[2]), round(d[3]), int(d[4]), int(d[5])] for d in f["d"]]
        f["ev"] = [[int(e[0]), round(e[1]), round(e[2]), round(e[3]), int(e[4]), int(e[5]), int(e[6]), int(e[7]),
                    int(e[8]), int(e[9]), int(e[10])] for e in f["ev"]]
        f["b"] = [[int(b[0]), round(b[1]), round(b[2]), round(b[3], 1), int(b[4])] for b in f["b"]]
    return tr


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    fragment = "--fragment" in argv
    if len(args) < 2:
        print(__doc__)
        return 2
    out_path, trace_paths = args[0], args[1:]
    traces = []
    for p in trace_paths:
        with open(p) as fh:
            tr = compact(json.load(fh))
        n = len(tr["drones"])
        tr["title"] = f"{n}-drone pack" if n >= 10 else f"{n}-drone storyline"
        tr["short"] = tr["title"]
        tr["lede"] = describe(tr)
        traces.append(tr)
    with open(os.path.join(HERE, "replay_viewer_template.html")) as fh:
        page = fh.read()
    blob = json.dumps(traces, separators=(",", ":"))
    marker = "/*TRACES*/"
    assert marker in page
    page = page.replace(marker, "window.TRACES = " + blob.replace("</", "<\\/") + ";")
    if not fragment:
        page = ('<!doctype html>\n<html lang="en">\n<meta charset="utf-8">\n'
                '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n' + page + "\n</html>\n")
    with open(out_path, "w") as fh:
        fh.write(page)
    print("wrote", out_path, len(page), "bytes,", len(traces), "scenarios")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
