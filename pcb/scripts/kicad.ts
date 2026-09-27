// Touch-ups for tscircuit's KiCad export (kicad/quad.kicad_pcb, .kicad_sch, .kicad_pro):
//  * pads of one pin that the exporter leaves without a net (the second pair
//    of USB-C shell slots) get the net of their twin pad,
//  * the ESP32's and the TPS63001's exposed pads get reduced paste like the
//    real stencil (for U3 one 90 % x 90 % opening stands in for TI's two),
//  * board texts get the size and stroke they have in the gerbers (KiCad's
//    text size is the full glyph height, tscircuit's is 1/0.7 of it),
//  * the project carries JLCPCB's limits, so KiCad's DRC checks against them.
//  * the ground pour's fill arrives as ~1000 triangles (visible seams, and
//    KiCad sees them as islands): it is dropped, press B in KiCad to refill.
import { readFileSync, writeFileSync } from "node:fs"
import { randomUUID } from "node:crypto"

const file = "kicad/quad.kicad_pcb"
const footprints = readFileSync(file, "utf8").split(/(?=\n  \(footprint\b)/)
const PAD = /\n    \(pad "([^"]*)"[^]*?\n    \)/g // one pad block
const fixed = footprints.map((fp) => {
  const netOf = new Map<string, string>()
  for (const [pad, num] of fp.matchAll(PAD)) {
    const net = pad.match(/\(net (\d+ "[^"]*")\)/)?.[1]
    if (net && !netOf.has(num)) netOf.set(num, net)
  }
  const isU1 = fp.includes('(property "Reference" "U1"'), isU3 = fp.includes('(property "Reference" "U3"')
  return fp.replace(PAD, (pad, num) => {
    if (!pad.includes("(net ") && netOf.has(num)) pad = pad.replace(/\n      \(uuid/, `\n      (net ${netOf.get(num)})$&`)
    if (isU1 && num === "41") pad = pad.replace(/\n      \(uuid/, "\n      (solder_paste_margin -0.1)$&")
    if (isU3 && num === "11") pad = pad.replace(/\n      \(uuid/, "\n      (solder_paste_margin_ratio -0.1)$&")
    return pad
  })
})
const board = fixed
  .join("")
  .replace(/\n    \(filled_polygon[^]*?\n    \)/g, "")
  .replace(/(\(connect_pads yes\s+\(clearance )0\.15\)/, (_, head) => `${head}0.25)`) // as in the gerbers
  .replace(/\(gr_text[^]*?\n  \)/g, (t) =>
    t.replace(/\(size ([\d.]+) ([\d.]+)\)(\s+)\(thickness [\d.]+\)/, (_, w, h, sp) =>
      `(size ${+(0.7 * w).toFixed(3)} ${+(0.7 * h).toFixed(3)})${sp}(thickness ${Math.max(0.09 * h, 0.16).toFixed(3)})`))
writeFileSync(file, board)

// JLCPCB 2-layer limits (clearances as used on this board: 0.2 mm copper)
writeFileSync("kicad/quad.kicad_pro", JSON.stringify({
  meta: { filename: "quad.kicad_pro", version: 1 },
  board: {
    design_settings: {
      rules: {
        min_clearance: 0.15, min_copper_edge_clearance: 0.3, min_hole_clearance: 0.15, min_hole_to_hole: 0.25,
        min_through_hole_diameter: 0.3, min_track_width: 0.127, min_via_annular_width: 0.13, min_via_diameter: 0.45,
        min_silk_clearance: 0, min_text_height: 0.8, min_text_thickness: 0.15,
      },
    },
  },
  net_settings: { classes: [{ name: "Default", clearance: 0.2, track_width: 0.25, via_diameter: 0.6, via_drill: 0.3 }] },
  // ERC noise of any imported schematic: tscircuit's grid is not KiCad's, its
  // symbols are embedded rather than from a library, and all pins are passive
  // (nothing "drives" a power net)
  erc: { rule_severities: { endpoint_off_grid: "ignore", lib_symbol_issues: "ignore", power_pin_not_driven: "ignore" } },
}, null, 2) + "\n")
console.log("kicad/: nets on shell pads, EPAD paste, JLCPCB rules")

// ---- schematic. tscircuit's export draws the right picture, but KiCad reads
// a few things in it differently; fix those, so that KiCad's netlist of the
// drawing is the design's netlist (checked by comparing the two):
//  * GND / V3V3 / VBAT / VBUS are ordinary symbols (a separate net each, and in
//    the BOM, their name hidden): make them KiCad power symbols, which KiCad
//    joins by name, and show the name,
//  * the library holds a copy of a symbol per instance under one name, and the
//    diodes' and some connectors' pins are numbered in drawing order instead of
//    the part's: keep one copy per name, and give an instance whose pins are
//    numbered differently (per tscircuit's own port data) a copy of its own,
//  * KiCad doesn't join a pin or label to a wire whose middle carries a
//    T-junction: split the wires at their junctions,
//  * pins the design leaves unconnected get a no-connect flag.
const schFile = "kicad/quad.kicad_sch"
let sch = readFileSync(schFile, "utf8")
const circuit: any[] = JSON.parse(readFileSync("dist/sch.circuit.json", "utf8"))
const byId = (t: string) => new Map<string, any>(circuit.filter((e) => e.type === t).map((e) => [e[`${t}_id`], e]))
const srcComp = byId("source_component"), srcPort = byId("source_port")

// power symbols
sch = sch
  .replace(/(\(symbol "Custom:rail_(?:up|down)"\n)/g, "$1      (power)\n")
  .replace(/(\(symbol "rail_(?:up|down)_1_1"\s+\(pin )passive/g, "$1power_in")
let pwr = 0
sch = sch.replace(/\n  \(symbol\n    \(lib_id "Custom:rail_(?:up|down)"\)[^]*?\n  \)(?=\n)/g, (inst) => {
  const ref = `#PWR${String(++pwr).padStart(2, "0")}`
  return inst
    .replace("(in_bom yes)", "(in_bom no)").replace("(on_board yes)", "(on_board no)")
    .replace(/\(property "Reference" "[^"]*"([^]*?\(size [^)]*\)\n\s*\))/, (_, body) => `(property "Reference" "${ref}"${body}\n        (hide yes)`)
    .replace(/\(reference "[^"]*"\)/, `(reference "${ref}")`)
    // show the net name, just past the symbol's bar (1.35 mm from its origin)
    .replace(/(\(property "Value" "[^"]*"\s+\(id 1\)\s+)\(at [^)]*\)(\s+\(effects\s+\(font\s+\(size [^)]*\)\s+\))\s+\(hide yes\)/, (_, head, fx) => {
      const [, dir, x, y, r] = inst.match(/\(lib_id "Custom:rail_(up|down)"\)\s+\(at ([-\d.e]+) ([-\d.e]+) ([-\d.]+)\)/)!
      return `${head}(at ${x} ${+y + (dir === "up" ? -2.9 : 2.9)} ${r})${fx}`
    })
})

// library: the first definition of each symbol only
const lib = new Map<string, string>()
sch = sch.replace(/\n    \(symbol "([^"]+)"\n[^]*?\n    \)(?=\n)/g, (def, id) => (lib.has(id) ? "" : (lib.set(id, def), def)))
const pinsOf = (def: string) =>
  [...def.matchAll(/\(pin \w+ \w+\s+\(at ([-\d.e]+) ([-\d.e]+) [-\d.]+\)[^]*?\(number "([^"]+)"/g)].map((m) => ({ num: m[3], x: +m[1], y: +m[2] }))

// where tscircuit put each port, in KiCad coordinates (scale and offset from
// the placed symbols: an instance sits at its schematic component's centre)
const INST = /\n  \(symbol\n    \(lib_id "([^"]+)"\)\n    \(at ([-\d.e]+) ([-\d.e]+) ([-\d.]+)\)([^]*?)\n  \)(?=\n)/g
const insts = [...sch.matchAll(INST)].map((m) => ({ text: m[0], lib: m[1], x: +m[2], y: +m[3], rot: +m[4], ref: m[5].match(/\(property "Reference" "([^"]+)"/)![1] }))
const centre = new Map([...byId("schematic_component").values()].map((c) => [srcComp.get(c.source_component_id).name, c.center]))
const placed = insts.filter((i) => centre.has(i.ref)).sort((a, b) => a.x - b.x)
const [p0, p1] = [placed[0], placed[placed.length - 1]]
const k = (p1.x - p0.x) / (centre.get(p1.ref).x - centre.get(p0.ref).x)
const ports = circuit.filter((e) => e.type === "schematic_port").map((sp) => {
  const s = srcPort.get(sp.source_port_id)
  const c0 = centre.get(p0.ref)
  return { ref: srcComp.get(s.source_component_id).name, num: String(s.pin_number), open: !s.subcircuit_connectivity_map_key,
    x: p0.x + k * (sp.center.x - c0.x), y: p0.y - k * (sp.center.y - c0.y) }
})

const variants = new Map<string, string>() // lib id + renumbering -> own lib id
const flags: string[] = []
for (const inst of insts) {
  const a = (inst.rot * Math.PI) / 180, renumber = new Map<string, string>()
  for (const pin of pinsOf(lib.get(inst.lib) ?? "")) {
    const x = inst.x + pin.x * Math.cos(a) - pin.y * Math.sin(a), y = inst.y - (pin.x * Math.sin(a) + pin.y * Math.cos(a))
    const port = ports.find((q) => q.ref === inst.ref && Math.hypot(q.x - x, q.y - y) < 0.01)
    if (!port) continue
    if (port.num !== pin.num) renumber.set(pin.num, port.num)
    if (port.open) flags.push(`  (no_connect\n    (at ${+x.toFixed(4)} ${+y.toFixed(4)})\n    (uuid ${randomUUID()})\n  )`)
  }
  if (!renumber.size) continue
  const key = `${inst.lib} ${[...renumber].join()}`
  if (!variants.has(key)) {
    const id = `${inst.lib}_${variants.size + 1}`, [base, nbase] = [inst.lib, id].map((s) => s.split(":")[1])
    variants.set(key, id)
    const def = lib.get(inst.lib)!
      .replace(`(symbol "${inst.lib}"`, `(symbol "${id}"`)
      .replaceAll(`(symbol "${base}_`, `(symbol "${nbase}_`)
      .replace(/(\(pin \w+ \w+[^]*?\(number ")([^"]+)"/g, (_, head, num) => `${head}${renumber.get(num) ?? num}"`)
    sch = sch.replace(lib.get(inst.lib)!, lib.get(inst.lib)! + def)
  }
  sch = sch.replace(inst.text, inst.text.replace(`(lib_id "${inst.lib}")`, `(lib_id "${variants.get(key)}")`))
}

// split wires at the junctions along them
const round = (v: string) => +(+v).toFixed(4)
const junctions = [...sch.matchAll(/\(junction\s+\(at ([-\d.e]+) ([-\d.e]+)\)/g)].map((m) => [round(m[1]), round(m[2])])
let split = 0
sch = sch.replace(/\n  \(wire\s+\(pts\s+\(xy ([-\d.e]+) ([-\d.e]+)\)\s+\(xy ([-\d.e]+) ([-\d.e]+)\)\s+\)([^]*?)\n  \)/g, (w, ax, ay, bx, by, rest) => {
  const [a, b] = [[round(ax), round(ay)], [round(bx), round(by)]]
  const on = junctions.filter(([x, y]) => !(x === a[0] && y === a[1]) && !(x === b[0] && y === b[1]) &&
    Math.min(a[0], b[0]) <= x && x <= Math.max(a[0], b[0]) && Math.min(a[1], b[1]) <= y && y <= Math.max(a[1], b[1]) &&
    Math.abs((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0])) < 1e-6)
  if (!on.length) return w
  split++
  const pts = [a, ...on.sort((p, q) => Math.hypot(p[0] - a[0], p[1] - a[1]) - Math.hypot(q[0] - a[0], q[1] - a[1])), b]
  return pts.slice(1).map((q, i) => `\n  (wire\n    (pts\n      (xy ${pts[i][0]} ${pts[i][1]})\n      (xy ${q[0]} ${q[1]})\n    )${rest.replace(/\(uuid [^)]+\)/, `(uuid ${randomUUID()})`)}\n  )`).join("")
})

// free texts arrive centred and all 1.27 mm: give them tscircuit's anchor and
// size; the block frames (schematic rects) don't arrive at all: draw them
const at = (p: { x: number; y: number }) => [p0.x + k * (p.x - centre.get(p0.ref).x), p0.y - k * (p.y - centre.get(p0.ref).y)]
const texts = circuit.filter((e) => e.type === "schematic_text" && !e.schematic_component_id)
sch = sch.replace(/\n  \(text "((?:[^"\\]|\\.)*)"\n    \(at ([-\d.e]+) ([-\d.e]+) ([-\d.]+)\)\n    \(effects\n      \(font\n        \(size [\d.]+ [\d.]+\)([^]*?)\n      \)\n    \)/g, (t, s, x, y, r, font) => {
  const d = (e: any) => Math.hypot(at(e.position)[0] - +x, at(e.position)[1] - +y)
  const near = texts.filter((e) => e.text === s).sort((a, b) => d(a) - d(b))[0]
  if (!near) return t
  const size = +(near.font_size * k * 0.6).toFixed(2), justify = /left|right/.exec(near.anchor)?.[0]
  return `\n  (text "${s}"\n    (at ${x} ${y} ${r})\n    (effects\n      (font\n        (size ${size} ${size})${font}\n      )${justify ? `\n      (justify ${justify})` : ""}\n    )`
})
const frames = circuit.filter((e) => e.type === "schematic_rect").map((r) => {
  const [x1, y1] = at({ x: r.center.x - r.width / 2, y: r.center.y + r.height / 2 }), [x2, y2] = at({ x: r.center.x + r.width / 2, y: r.center.y - r.height / 2 })
  return `  (rectangle\n    (start ${+x1.toFixed(3)} ${+y1.toFixed(3)})\n    (end ${+x2.toFixed(3)} ${+y2.toFixed(3)})\n    (stroke\n      (width 0.2)\n      (type dash)\n      (color 154 154 154 1)\n    )\n    (fill\n      (type none)\n    )\n    (uuid ${randomUUID()})\n  )`
})
// a value that only repeats the reference (the buttons) is shown once
sch = sch.replace(/\n    \(property "Reference" "([^"]+)"((?:(?!\n  \(symbol)[^])*?)\(property "Value" "\1"([^]*?\(size [^)]*\)\n\s*\))/g, (_, ref, mid, val) => `\n    (property "Reference" "${ref}"${mid}(property "Value" "${ref}"${val}\n        (hide yes)`)

sch = sch.replace(/\n\)\s*$/, `\n${[...flags, ...frames].join("\n")}\n)\n`)
writeFileSync(schFile, sch)
console.log(`kicad/quad.kicad_sch: ${pwr} power symbols, ${variants.size} renumbered symbols, ${split} wires split, ${flags.length} no-connect flags`)
