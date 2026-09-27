"""Independent design-rule check on dist/circuit.json (shapely geometry).

  python drc.py [dist/circuit.json]

Checks, per copper layer:
  * clearance between copper of different nets  >= CLEAR
  * copper to board edge                         >= EDGE
  * every net is one connected piece of copper (layers joined by vias/THT)
  * via drill / annular ring vs. JLCPCB limits
"""
import json, sys
from collections import defaultdict
from shapely.geometry import Point, Polygon, LineString, box
from shapely.affinity import rotate
from shapely.ops import unary_union
from shapely.strtree import STRtree

CLEAR, EDGE, VIA_DRILL, VIA_RING = 0.20, 0.3, 0.3, 0.15  # mm: 2x JLCPCB's 0.1 mm spacing, and its via/edge minimums
LAYERS = ("top", "bottom")

cj = json.load(open(sys.argv[1] if len(sys.argv) > 1 else "dist/circuit.json"))
by = defaultdict(list)
for e in cj:
    by[e["type"]].append(e)
idx = {k: {e[k + "_id"]: e for e in by[k]} for k in ("source_port", "pcb_port", "source_trace", "source_net", "source_component", "pcb_component")}

def net_of_port(pcb_port_id):
    pp = idx["pcb_port"].get(pcb_port_id)
    sp = pp and idx["source_port"].get(pp.get("source_port_id"))
    if sp and not sp.get("subcircuit_connectivity_map_key") and "_internal_" in sp["name"]:
        # extra pads of one pin (e.g. ESP32 thermal pad) share the main pin's net
        base = sp["name"].split("_internal_")[0]
        sp = next((q for q in by["source_port"] if q["source_component_id"] == sp["source_component_id"] and q["name"] == base), sp)
    return sp and sp.get("subcircuit_connectivity_map_key")

def label_of_port(pcb_port_id):
    pp = idx["pcb_port"][pcb_port_id]
    sp = idx["source_port"][pp["source_port_id"]]
    return f'{idx["source_component"][sp["source_component_id"]]["name"]}.{sp["name"]}'

net_names = {}
for n in by["source_net"]:
    net_names[n["subcircuit_connectivity_map_key"]] = n["name"]

def pad_shape(p):
    x, y, s = p["x"], p["y"], p["shape"]
    if s in ("rect", "rotated_rect"):
        g = box(x - p["width"] / 2, y - p["height"] / 2, x + p["width"] / 2, y + p["height"] / 2)
        return rotate(g, p.get("ccw_rotation", 0) or 0, origin=(x, y))
    if s == "circle":
        return Point(x, y).buffer(p["radius"])
    if s in ("pill", "rotated_pill"):
        w, h = p["width"], p["height"]
        r = min(w, h) / 2
        seg = LineString([(x - (w / 2 - r), y), (x + (w / 2 - r), y)]) if w >= h else LineString([(x, y - (h / 2 - r)), (x, y + (h / 2 - r))])
        return rotate(seg.buffer(r), p.get("ccw_rotation", 0) or 0, origin=(x, y))
    if s == "polygon":
        return Polygon([(q["x"], q["y"]) for q in p["points"]])
    raise ValueError(f"pad shape {s}")

def hole_shape(h):
    x, y, s = h["x"], h["y"], h["shape"]
    if s == "circle":
        return Point(x, y).buffer(h["outer_diameter"] / 2)
    w, hh = h.get("outer_width", 0), h.get("outer_height", 0)
    r = min(w, hh) / 2
    seg = LineString([(x - (w / 2 - r), y), (x + (w / 2 - r), y)]) if w >= hh else LineString([(x, y - (hh / 2 - r)), (x, y + (hh / 2 - r))])
    g = seg.buffer(r) if s in ("pill", "oval") else box(x - w / 2, y - hh / 2, x + w / 2, y + hh / 2)
    return rotate(g, h.get("ccw_rotation", 0) or 0, origin=(x, y))

# ---- collect copper: (layer, net, geometry, label)
copper = []
for p in by["pcb_smtpad"]:
    copper.append((p["layer"], net_of_port(p.get("pcb_port_id")), pad_shape(p), label_of_port(p["pcb_port_id"]) if p.get("pcb_port_id") else "pad"))
for h in by["pcb_plated_hole"]:
    g = hole_shape(h)
    for L in LAYERS:
        copper.append((L, net_of_port(h.get("pcb_port_id")), g, label_of_port(h["pcb_port_id"]) if h.get("pcb_port_id") else "hole"))
problems = []
for v in by["pcb_via"]:
    g = Point(v["x"], v["y"]).buffer(v["outer_diameter"] / 2)
    for L in LAYERS:
        net = v.get("subcircuit_connectivity_map_key") or (v.get("source_net_id") and idx["source_net"][v["source_net_id"]]["subcircuit_connectivity_map_key"])
        copper.append((L, net, g, f'via@{v["x"]:.2f},{v["y"]:.2f}'))
    if v["hole_diameter"] < VIA_DRILL - 1e-6 or (v["outer_diameter"] - v["hole_diameter"]) / 2 < VIA_RING - 1e-6:
        problems.append(f'via at ({v["x"]:.2f},{v["y"]:.2f}) {v["hole_diameter"]}/{v["outer_diameter"]} below JLC limits')
for t in by["pcb_trace"]:
    st = idx["source_trace"].get(t.get("source_trace_id"))
    net = st and st.get("subcircuit_connectivity_map_key")
    r = t["route"]
    for a, b in zip(r, r[1:]):
        if a.get("route_type") == "wire" and b.get("route_type") == "wire" and a["layer"] == b["layer"]:
            seg = LineString([(a["x"], a["y"]), (b["x"], b["y"])]) if (a["x"], a["y"]) != (b["x"], b["y"]) else Point(a["x"], a["y"])
            copper.append((a["layer"], net, seg.buffer(a["width"] / 2, cap_style="round"), f"trace {t['pcb_trace_id']}"))
    for a in r:
        if a.get("route_type") == "via":
            g = Point(a["x"], a["y"]).buffer((a.get("outer_diameter") or 0.6) / 2)
            for L in LAYERS:
                copper.append((L, net, g, f"trace-via {t['pcb_trace_id']}"))
for p in by["pcb_copper_pour"]:
    b = p["brep_shape"]
    ring = lambda r: [(v["x"], v["y"]) for v in r["vertices"]]
    g = Polygon(ring(b["outer_ring"]), [ring(r) for r in b["inner_rings"]]).buffer(0)
    net = idx["source_net"][p["source_net_id"]]["subcircuit_connectivity_map_key"]
    copper.append((p["layer"], net, g, f"pour {p['pcb_copper_pour_id']}"))

# ---- clearance between different nets
for L in LAYERS:
    items = [c for c in copper if c[0] == L]
    tree = STRtree([c[2] for c in items])
    for i, (_, net, g, lab) in enumerate(items):
        for j in tree.query(g.buffer(CLEAR)):
            if j <= i:
                continue
            _, net2, g2, lab2 = items[j]
            if net and net2 and net == net2:
                continue
            d = g.distance(g2)
            if d < CLEAR - 5e-3:
                problems.append(f"{L}: {lab} [{net_names.get(net, net)}] <-> {lab2} [{net_names.get(net2, net2)}] clearance {d:.3f} mm")

# ---- board edge clearance
board = by["pcb_board"][0]
outline = Polygon([(q["x"], q["y"]) for q in board["outline"]])
inner = outline.buffer(-EDGE)
for L, net, g, lab in copper:
    if not inner.contains(g) and not lab.startswith("pour"):
        problems.append(f"{L}: {lab} within {EDGE} mm of board edge")
    if lab.startswith("pour") and not outline.buffer(-EDGE + 1e-3).contains(g):
        problems.append(f"{L}: {lab} too close to board edge")

# ---- connectivity: each net must be one connected blob (vias / THT join layers)
unrouted = []
nets = defaultdict(list)
for c in copper:
    if c[1]:
        nets[c[1]].append(c)
for net, items in nets.items():
    parent = list(range(len(items)))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    geoms = [c[2] for c in items]
    tree = STRtree(geoms)
    for i, (L, _, g, lab) in enumerate(items):
        for j in tree.query(g):
            if j != i and (items[j][0] == L or lab == items[j][3]) and g.intersects(geoms[j]):
                parent[find(i)] = find(j)
    # pads that are one pin inside the part (e.g. ESP32 thermal pad) are one conductor
    base = lambda lab: lab.split("_internal_")[0]
    first = {}
    for i, c in enumerate(items):
        first.setdefault(base(c[3]), i)
    for i, c in enumerate(items):
        if "_internal_" in c[3] and base(c[3]) in first:
            parent[find(i)] = find(first[base(c[3])])
    # same via / THT object on both layers is one conductor
    groups = defaultdict(set)
    for i, c in enumerate(items):
        groups[find(i)].add(c[3])
    if len(groups) > 1:
        pads = [sorted(l for l in g if not l.startswith(("trace", "via", "pour"))) for g in groups.values()]
        unrouted.append(f"net {net_names.get(net, net)} is split into {len(groups)} pieces: " + " | ".join(",".join(p[:6]) or "(copper only)" for p in pads))

for p in problems + unrouted:
    print("  ✗", p)
print(f"drc.py: {len(copper)} copper items, {len(problems)} clearance/edge problems, {len(unrouted)} split nets")
sys.exit(1 if problems or unrouted else 0)
