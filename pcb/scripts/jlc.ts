// JLCPCB assembly files from dist/circuit.json -> fab/bom.csv, fab/cpl.csv
//
// tscircuit's own pick-and-place file uses the centre of each part's pads,
// but JLCPCB places a part at the origin of its (EasyEDA) footprint. The two
// differ for these footprints; offsets in footprint coordinates, measured by
// matching EasyEDA's pad data against footprints/*.json:
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs"

const ORIGIN: Record<string, [number, number]> = {
  C2913202: [0, 0.15], // ESP32-S3-WROOM-1
  C2765186: [0, -0.25], // USB-C
  C122410: [0, 0.35], // battery connector
  C2845379: [0, 0.075], // motor connectors
}

const cj: any[] = JSON.parse(readFileSync("dist/circuit.json", "utf8"))
const of = (type: string) => cj.filter((e) => e.type === type)
const src = new Map(of("source_component").map((e) => [e.source_component_id, e]))
const cad = new Map(of("cad_component").map((e) => [e.pcb_component_id, e]))
const rot = (deg: number, [x, y]: number[]) => {
  const a = (deg * Math.PI) / 180
  return [x * Math.cos(a) - y * Math.sin(a), x * Math.sin(a) + y * Math.cos(a)]
}
const pin = (e: any) => e.port_hints?.find((h: string) => /^pin\d+$/.test(h))
const num = (x: number) => String(Math.round(x * 1000) / 1000)
const value = (s: any) =>
  s.manufacturer_part_number ??
  (s.resistance !== undefined ? (s.resistance >= 1000 ? `${num(s.resistance / 1000)}k` : `${num(s.resistance)}R`) : undefined) ??
  (s.capacitance >= 1e-6 ? `${num(s.capacitance * 1e6)}uF` : `${num(s.capacitance * 1e9)}nF`)

const parts = of("pcb_component").flatMap((pc) => {
  const s = src.get(pc.source_component_id)
  const lcsc = s?.supplier_part_numbers?.jlcpcb?.[0]
  if (!lcsc) return [] // test pads: bare copper, nothing to place
  const deg = (((pc.rotation ?? 0) % 360) + 360) % 360
  // footprint origin: from one pad, placed vs. its position in the footprint file
  let [x, y] = [pc.center.x, pc.center.y]
  const file = `footprints/${lcsc}.json`
  if (existsSync(file)) {
    const pad = of("pcb_smtpad").find((p) => p.pcb_component_id === pc.pcb_component_id && pin(p))
    const ref = JSON.parse(readFileSync(file, "utf8")).footprintCircuitJson.find((e: any) => e.type === "pcb_smtpad" && pin(e) === pin(pad))
    const [dx, dy] = rot(deg, [ref.x, ref.y])
    ;[x, y] = [pad.x - dx, pad.y - dy]
  }
  const [ox, oy] = rot(deg, ORIGIN[lcsc] ?? [0, 0])
  const footprint = cad.get(pc.pcb_component_id)?.footprinter_string?.replace(/^(res|cap|led)/, "") ?? lcsc
  return [{ ref: s.name, lcsc, comment: value(s), footprint, x: x + ox, y: y + oy, deg, layer: pc.layer === "bottom" ? "Bottom" : "Top" }]
})

const natural = (a: string, b: string) => a.localeCompare(b, "en", { numeric: true })
const csv = (rows: (string | number)[][]) => rows.map((r) => r.map((c) => `"${c}"`).join(",")).join("\n") + "\n"
const groups = Map.groupBy(parts, (p) => p.lcsc)
const bom = [...groups.values()]
  .map((g) => g.sort((a, b) => natural(a.ref, b.ref)))
  .sort((a, b) => natural(a[0].ref, b[0].ref))
  .map((g) => [g[0].comment, g.map((p) => p.ref).join(","), g[0].footprint, g[0].lcsc])
const cpl = parts
  .sort((a, b) => natural(a.ref, b.ref))
  .map((p) => [p.ref, `${p.x.toFixed(3)}mm`, `${p.y.toFixed(3)}mm`, p.layer, p.deg.toFixed(0)])

mkdirSync("fab", { recursive: true })
writeFileSync("fab/bom.csv", csv([["Comment", "Designator", "Footprint", "JLCPCB Part #"], ...bom]))
writeFileSync("fab/cpl.csv", csv([["Designator", "Mid X", "Mid Y", "Layer", "Rotation"], ...cpl]))
console.log(`fab/bom.csv: ${bom.length} lines, ${parts.length} parts; fab/cpl.csv: ${cpl.length} placements`)
