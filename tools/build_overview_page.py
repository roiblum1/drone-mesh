#!/usr/bin/env python3
"""Build docs/pack-mesh-overview.html from the repo's own files (diagrams, prompts, code) so nothing drifts.
Usage: python3 -I tools/build_overview_page.py docs/pack-mesh-overview.html [--local-mermaid]   (--local-mermaid adds a CDN script for local preview only)"""
import html
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(os.path.dirname(HERE), "")
out_path = sys.argv[1]
local_mermaid = "--local-mermaid" in sys.argv


def read(rel):
    return open(ROOT + rel).read()


def esc(s):
    return html.escape(s, quote=False)


# ------------------------------------------------------------------ sources
doc3 = read("docs/03_hardware_block_diagram.md")
mermaid_src = re.search(r"```mermaid\n(.*?)```", doc3, re.S).group(1).strip()
ascii_src = re.search(r"## 2\. ASCII hardware block diagram\n\n```\n(.*?)\n```", doc3, re.S).group(1)

doc1 = read("docs/01_marketing_storyboard.md")
master = re.search(r"> \*\*Prompt\*\*\n>\n> (.*?)\n\n", doc1, re.S).group(1).strip()
negative = re.search(r"> \*\*Negative prompt\*\*\n>\n> (.*?)\n\n", doc1, re.S).group(1).strip()
mj = re.search(r"## Midjourney variant\n\n```\n(.*?)\n```", doc1, re.S).group(1).strip()
short = re.search(r"## Short variant[^\n]*\n\n> (.*?)\n\n", doc1, re.S).group(1).strip()

mgr = read("src/pack_mesh_manager.cpp")
eval_fn = re.search(r"(void PackMeshManager::evaluateAndCoordinateAlerts.*?\n}\n)", mgr, re.S).group(1)
state_h = read("include/amc8270/mesh/drone_state.hpp")
drone_state = re.search(r"(// What every peer needs.*?\n};)", state_h, re.S).group(1)
pkt_h = read("include/amc8270/mesh/mesh_packet.hpp")
mesh_packet = re.search(r"(// Variant index == MsgType value.*?\n};)", pkt_h, re.S).group(1)
ev_h = read("include/amc8270/mesh/detected_event.hpp")
detected = re.search(r"(// Fused picture of one source.*?\n};)", ev_h, re.S).group(1)

# ------------------------------------------------- scenario map (to scale, km)
SC = 9.6


def X(x):
    return 44 + (x + 4) * SC


def Y(y):
    return 650 - (y + 30) * SC


def circle_r(km):
    return km * SC


# Drawn from the seed-1 storyline (make sim) at the ADVISORY, t = 31.6 s: drone
# positions, the bearings as reported (D2's was in a multipath reflection), the
# pack's estimate and CEP from the alert, and the verifiers' standoff points.
drones = [("D1", -25), ("D2", -14.7), ("D3", -5), ("D4", 4.9), ("D5", 15), ("D6", 25)]
dx = 1.2
beacon = (16, 8)
estimate = (14.40, 8.92)
cep_km = 0.604
# drone: (reported bearing in degrees, reflected)
bearings = {"D2": (22.6, True), "D3": (46.1, False), "D4": (74.1, False), "D5": (116.0, False), "D6": (136.6, False)}
svg = []
a = svg.append
a('<svg class="map" viewBox="0 0 520 700" role="img" aria-labelledby="map-t map-d">')
a('<title id="map-t">Scenario map at the ADVISORY, t = 31.7 s</title>')
a('<desc id="map-d">Six drones in a line abreast 10 km apart. Five of them report bearings on a beacon 16 km east and 8 km north of the pack centre; one of those, from D2, is in a multipath reflection and 12 degrees off, which pulls the pack estimate 1.8 km west of the source. The controlled drone D1 is out of range. Two verifiers head for standoff points 6 km from the source.</desc>')
# mesh spine
for (n1, y1), (n2, y2) in zip(drones, drones[1:]):
    a(f'<line class="mesh" x1="{X(dx):.1f}" y1="{Y(y1):.1f}" x2="{X(dx):.1f}" y2="{Y(y2):.1f}"/>')
# route arrow
a(f'<line class="route" x1="{X(-3.2):.1f}" y1="{Y(-30):.1f}" x2="{X(-3.2):.1f}" y2="{Y(-27):.1f}"/>')
# standoff ring
a(f'<circle class="ring" cx="{X(beacon[0]):.1f}" cy="{Y(beacon[1]):.1f}" r="{circle_r(6):.1f}"/>')
# bearings as reported, each drawn 4 km past the source's range
for name, y in drones[1:]:
    brg, refl = bearings[name]
    rng = math.hypot(beacon[0] - dx, beacon[1] - y) + 4.0
    ex = dx + rng * math.sin(math.radians(brg))
    ey = y + rng * math.cos(math.radians(brg))
    cls = "bearing refl" if refl else "bearing"
    a(f'<line class="{cls}" x1="{X(dx):.1f}" y1="{Y(y):.1f}" x2="{X(ex):.1f}" y2="{Y(ey):.1f}"/>')
a(f'<text class="note" x="{X(13.6):.1f}" y="{Y(15.4):.1f}">D2: reflected, 12&#176; off</text>')
# D1 out of range
a(f'<line class="oor" x1="{X(dx):.1f}" y1="{Y(-25):.1f}" x2="{X(beacon[0]):.1f}" y2="{Y(beacon[1]):.1f}"/>')
# verifier paths: D2 primary (EO, from the east), D4 secondary (IR, from 150 deg)
so_d2 = (21.74, 7.73)
so_d4 = (18.74, 2.54)
a(f'<line class="path" x1="{X(dx):.1f}" y1="{Y(-14.7):.1f}" x2="{X(so_d2[0]):.1f}" y2="{Y(so_d2[1]):.1f}"/>')
a(f'<line class="path" x1="{X(dx):.1f}" y1="{Y(4.9):.1f}" x2="{X(so_d4[0]):.1f}" y2="{Y(so_d4[1]):.1f}"/>')
# loss marker on D2's path (t = 90 s, from the trace)
fx, fy = 2.95, -12.75
a(f'<g class="lost"><line x1="{X(fx)-6:.1f}" y1="{Y(fy)-6:.1f}" x2="{X(fx)+6:.1f}" y2="{Y(fy)+6:.1f}"/><line x1="{X(fx)-6:.1f}" y1="{Y(fy)+6:.1f}" x2="{X(fx)+6:.1f}" y2="{Y(fy)-6:.1f}"/></g>')
a(f'<text class="note" x="{X(fx)+10:.1f}" y="{Y(fy)+18:.1f}">D2 lost, t = 90 s</text>')
for p in (so_d2, so_d4):
    a(f'<circle class="stand" cx="{X(p[0]):.1f}" cy="{Y(p[1]):.1f}" r="5"/>')
a(f'<text class="note" x="{X(so_d2[0])+9:.1f}" y="{Y(so_d2[1])+4:.1f}">D2 standoff, east</text>')
a(f'<text class="note" x="{X(so_d4[0])+9:.1f}" y="{Y(so_d4[1])+12:.1f}">D4 standoff</text>')
# pack estimate + CEP, and the true source
a(f'<circle class="cep" cx="{X(estimate[0]):.1f}" cy="{Y(estimate[1]):.1f}" r="{max(circle_r(cep_km), 5):.1f}"/>')
a(f'<circle class="src" cx="{X(beacon[0]):.1f}" cy="{Y(beacon[1]):.1f}" r="3.2"/>')
a(f'<text class="lbl" x="{X(23.0):.1f}" y="{Y(12.4):.1f}">406.025 MHz beacon</text>')
a(f'<text class="note" x="{X(23.0):.1f}" y="{Y(12.4)+16:.1f}">estimate: CEP 604 m (disc),</text>')
a(f'<text class="note" x="{X(23.0):.1f}" y="{Y(12.4)+31:.1f}">1.8 km from the source (dot)</text>')
# other emitters
for (ex, ey, lab) in ((30, -12, "ISM 433.92 MHz, novel"), (22, 30, "weather radio, pre-briefed benign")):
    a(f'<rect class="emit" x="{X(ex)-4:.1f}" y="{Y(ey)-4:.1f}" width="8" height="8" transform="rotate(45 {X(ex):.1f} {Y(ey):.1f})"/>')
    a(f'<text class="note" x="{X(ex)-8:.1f}" y="{Y(ey)-10:.1f}" text-anchor="{"end" if ex > 25 else "start"}">{lab}</text>')
# drones
for name, y in drones:
    cls = "uav op" if name == "D1" else ("uav low" if name == "D5" else "uav")
    a(f'<g class="{cls}"><circle cx="{X(dx):.1f}" cy="{Y(y):.1f}" r="7"/><text x="{X(dx)-14:.1f}" y="{Y(y)+4:.1f}" text-anchor="end" class="lbl">{name}</text></g>')
a(f'<text class="note" x="{X(dx)+13:.1f}" y="{Y(-25)-3:.1f}">out of DF range</text>')
a(f'<text class="note" x="{X(dx)+13:.1f}" y="{Y(-25)+12:.1f}">37 km from source</text>')
a(f'<text class="note" x="{X(-3.2)+8:.1f}" y="{Y(-28.4):.1f}">route</text>')
# scale bar and north
sx, sy = 44, 686
a(f'<line class="scale" x1="{sx}" y1="{sy}" x2="{sx + 10*SC:.1f}" y2="{sy}"/><line class="scale" x1="{sx}" y1="{sy-4}" x2="{sx}" y2="{sy+4}"/><line class="scale" x1="{sx+10*SC:.1f}" y1="{sy-4}" x2="{sx+10*SC:.1f}" y2="{sy+4}"/>')
a(f'<text class="note" x="{sx + 10*SC + 8:.1f}" y="{sy+4}">10 km</text>')
a('<text class="note" x="490" y="36" text-anchor="middle">N</text><line class="scale" x1="490" y1="42" x2="490" y2="14"/><polyline class="scale" points="485,20 490,12 495,20"/>')
a("</svg>")
map_svg = "\n".join(svg)


def copy_block(id_, label, text):
    return (f'<div class="copyblock"><div class="cb-head"><span>{label}</span>'
            f'<button type="button" class="copy" data-target="{id_}">Copy</button></div>'
            f'<pre id="{id_}" class="prompt">{esc(text)}</pre></div>')


page = open(os.path.join(HERE, "overview_page_template.html")).read()
repl = {
    "{{MAP}}": map_svg,
    "{{MERMAID}}": esc(mermaid_src),
    "{{ASCII}}": esc(ascii_src),
    "{{PROMPT_MASTER}}": copy_block("p-master", "Master prompt (hero image)", master),
    "{{PROMPT_NEG}}": copy_block("p-neg", "Negative prompt", negative),
    "{{PROMPT_MJ}}": copy_block("p-mj", "Midjourney variant", mj),
    "{{PROMPT_SHORT}}": copy_block("p-short", "Short variant (DALL-E / Imagen style)", short),
    "{{CODE_STATE}}": esc(drone_state),
    "{{CODE_EVENT}}": esc(detected),
    "{{CODE_PACKET}}": esc(mesh_packet),
    "{{CODE_EVAL}}": esc(eval_fn),
    "{{LOCAL_MERMAID}}": (
        '<script type="module">import mermaid from "https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.esm.min.mjs";'
        'mermaid.initialize({startOnLoad:true,theme:"default"});</script>' if local_mermaid else ""),
}
for k, v in repl.items():
    assert k in page, k
    page = page.replace(k, v)
open(out_path, "w").write(page)
print("wrote", out_path, len(page), "bytes")
