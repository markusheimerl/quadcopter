// Build = render index.tsx twice:
//   dist/circuit.json      the board: placement, hand-drawn copper, pour
//   dist/sch.circuit.json  the schematic (see `render` below for why apart)
// then run the design-rule checks and print what they find.
// We don't use `tsci build`: its built-in DRC step can hang.
//   --sch    schematic only (seconds)
//   --place  placement only, no copper
//   x.tsx    build another design file (writes dist/x.circuit.json ...)
import * as React from "react"
import { RootCircuit } from "tscircuit"
import { getPlatformConfig } from "@tscircuit/eval/platform-config"
import * as checks from "@tscircuit/checks"
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs"
import { createHash } from "node:crypto"
import { cleanSilkscreen } from "./scripts/silk"
;(globalThis as any).React = React

const t0 = performance.now()
const log = (...a: any[]) => console.log(`${((performance.now() - t0) / 1000).toFixed(0).padStart(4)}s`, ...a)
const placeOnly = process.argv.includes("--place")
const schOnly = process.argv.includes("--sch")
const file = process.argv.find((a) => a.endsWith(".tsx") && !a.endsWith("build.tsx")) ?? "index.tsx"
const stem = file === "index.tsx" ? "" : file.replace(/\.tsx$/, ".")
// JLCPCB footprints are fetched once and kept in footprints/ (checked in), so
// builds are reproducible and work offline. Part lookups use a disk cache too.
const base = getPlatformConfig()
const fetchJlc = base.footprintLibraryMap!.jlcpcb as (partNumber: string) => Promise<unknown>
const cacheFile = (key: string) => `.tscircuit/cache/${createHash("md5").update(key).digest("hex")}.json`
const fetchPart = base.partsEngine!.fetchPartCircuitJson!.bind(base.partsEngine)
const platform = {
  ...base,
  partsEngine: {
    ...base.partsEngine,
    // supplier-footprint lookups (used for the footprint/polarity cross-check);
    // failures are remembered too, delete .tscircuit/cache to retry them
    fetchPartCircuitJson: async (args: any) => {
      const file = cacheFile(`part:${args.supplierPartNumber ?? JSON.stringify(args)}`)
      if (existsSync(file)) { const hit = JSON.parse(readFileSync(file, "utf8")); if (hit === null) throw new Error("cached miss"); return hit }
      mkdirSync(".tscircuit/cache", { recursive: true })
      try { const r = await fetchPart(args); writeFileSync(file, JSON.stringify(r ?? null)); return r }
      catch (e) { writeFileSync(file, "null"); throw e }
    },
  },
  localCacheEngine: {
    getItem: (key: string) => (existsSync(cacheFile(key)) ? readFileSync(cacheFile(key), "utf8") : null),
    setItem: (key: string, value: string) => (mkdirSync(".tscircuit/cache", { recursive: true }), writeFileSync(cacheFile(key), value)),
  },
  footprintLibraryMap: {
    ...base.footprintLibraryMap,
    jlcpcb: async (partNumber: string) => {
      const file = `footprints/${partNumber}.json`
      if (existsSync(file)) return JSON.parse(readFileSync(file, "utf8"))
      const footprint = await fetchJlc(partNumber)
      mkdirSync("footprints", { recursive: true })
      writeFileSync(file, JSON.stringify(footprint))
      return footprint
    },
  },
}
const Board = (await import(`./${file}`)).default
// The hand-drawn tracks are <trace>s between ports, which tscircuit would also
// draw into the schematic (as wires and labels). So the schematic comes from a
// second render without them: dist/sch.circuit.json.
const render = async ({ pcb, copper }: { pcb: boolean; copper: boolean }) => {
  const circuit = new RootCircuit({ platform: { ...platform, drcChecksDisabled: true } as any })
  ;(circuit as any).pcbDisabled = !pcb
  circuit.add(React.createElement(Board, { copper }))
  circuit.render()
  while (!circuit.isDoneRendering()) {
    await new Promise((r) => setTimeout(r, 200))
    circuit.render()
  }
  const cj = (await circuit.getCircuitJson()) as any[]
  // tscircuit writes net names in tiny type along wires; the signals that
  // leave a block carry proper <netlabel>s (index.tsx), so drop these
  const isWireName = (e: any) => e.type === "schematic_text" && e.source_trace_id
  return cj.filter(
    (e) =>
      !isWireName(e) &&
      // tscircuit draws its own section dividers, which misplace once a row
      // holds two blocks; index.tsx draws the frames itself
      !(e.type === "schematic_line" && e.is_dashed && e.color === "#000000") &&
      // assembly is top side only: no bottom stencil
      !(e.type === "pcb_solder_paste" && e.layer === "bottom"),
  )
}
// design-rule checks by family; returns how many distinct problems were found
const runChecks = async (cj: any[], families: string[]) => {
  const board = cj.find((e) => e.type === "pcb_board")
  const drc: any[] = []
  for (const [name, run] of [
    ["routing", checks.runAllRoutingChecks],
    ["placement", checks.runAllPlacementChecks],
    ["netlist", checks.runAllNetlistChecks],
    ["jlcpcb", (c: any) => platform.fabricatorEngine!.runDrcChecks({ circuitJson: c, fabricatorPreset: "jlcpcb_economy", pcbBoardId: board.pcb_board_id } as any)],
  ] as const) {
    if (!families.includes(name)) continue
    drc.push(...(await (run as any)(structuredClone(cj))))
    log(`${name} checks done`)
  }
  // build-time errors (placement, routing) plus the check results
  const seen = new Set<string>()
  for (const p of [...cj, ...drc].filter((e) => /_error$/.test(e.type) || e.error_type)) {
    const msg = `${p.type}: ${p.message}`
    if (!seen.has(msg)) { seen.add(msg); console.log("  ✗", msg) }
  }
  return seen.size
}

mkdirSync("dist", { recursive: true })
log("drawing schematic...")
const sch = await render({ pcb: false, copper: false })
writeFileSync(`dist/${stem}sch.circuit.json`, JSON.stringify(sch))
log(`wrote dist/${stem}sch.circuit.json`)
let problems = await runChecks(sch, ["netlist"])

if (!schOnly) {
  log(placeOnly ? "placing..." : "placing + drawing copper...")
  let cj = await render({ pcb: true, copper: !placeOnly })
  // tscircuit shrinks every paste opening to 70 % x 70 % (half the solder).
  // Use 1:1 openings, the usual stencil rule (and Bosch's for the BMI323)...
  const padById = new Map(cj.filter((e) => e.type === "pcb_smtpad").map((p) => [p.pcb_smtpad_id, p]))
  for (const paste of cj.filter((e) => e.type === "pcb_solder_paste" && e.pcb_smtpad_id)) {
    const pad = padById.get(paste.pcb_smtpad_id)
    for (const k of ["width", "height", "radius"]) if (k in pad) paste[k] = pad[k]
  }
  // ...except under the ESP32's exposed pad: 0.7 mm squares on its 0.9 mm tiles
  // (~60 % cover) so the module sits flat on its 40 pins instead of floating
  const u1 = cj.find((e) => e.type === "source_component" && e.name === "U1").source_component_id
  const epadPorts = new Set(cj.filter((e) => e.type === "source_port" && e.source_component_id === u1 && /^pin41(_internal_\d+)?$/.test(e.name)).map((e) => e.source_port_id))
  const epad = new Set(cj.filter((e) => e.type === "pcb_port" && epadPorts.has(e.source_port_id)).map((e) => e.pcb_port_id))
  for (const paste of cj.filter((e) => e.type === "pcb_solder_paste" && epad.has(padById.get(e.pcb_smtpad_id)?.pcb_port_id))) paste.width = paste.height = 0.7
  // USB1's outline hangs off the board edge. The motor connectors' pin-1 dot
  // would sit on + for two of them and on - for the others; index.tsx marks +.
  if (!placeOnly) cj = cleanSilkscreen(cj, (ref, e) => ref === "USB1" || (/^CN[1-4]$/.test(ref) && e.type === "pcb_silkscreen_circle"))
  const out = `dist/${stem}${placeOnly ? "place." : ""}circuit.json`
  writeFileSync(out, JSON.stringify(cj))
  log(`wrote ${out}`)
  problems += await runChecks(cj, placeOnly ? ["placement"] : ["routing", "placement", "netlist", "jlcpcb"])
}
log(problems ? `${problems} problem(s)` : "no DRC errors")
process.exit(problems ? 1 : 0)
