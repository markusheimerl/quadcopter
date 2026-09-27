// img/schematic.svg: tscircuit draws the sheet on a fixed 1200 x 600 canvas
// with a wide empty margin; crop the canvas to the block frames and the title
import { readFileSync, writeFileSync } from "node:fs"

const cj: any[] = JSON.parse(readFileSync("dist/sch.circuit.json", "utf8"))
const file = "img/schematic.svg"
let svg = readFileSync(file, "utf8")
const [a, , , d, e, f] = svg.match(/data-real-to-screen-transform="matrix\(([^)]+)\)"/)![1].split(",").map(Number)
const pts = cj.flatMap((el) =>
  el.type === "schematic_rect"
    ? [[el.center.x - el.width / 2, el.center.y - el.height / 2], [el.center.x + el.width / 2, el.center.y + el.height / 2]]
    : el.type === "schematic_text" && !el.schematic_component_id ? [[el.position.x, el.position.y + el.font_size]] : [],
)
const m = 0.3 // sheet units
const [x1, x2] = [Math.min(...pts.map((p) => p[0])) - m, Math.max(...pts.map((p) => p[0])) + m]
const [y1, y2] = [Math.min(...pts.map((p) => p[1])) - m, Math.max(...pts.map((p) => p[1])) + m]
const [sx, sy, w, h] = [a * x1 + e, d * y2 + f, a * (x2 - x1), -d * (y2 - y1)].map((v) => Math.round(v))
svg = svg.replace(/<svg([^>]*?) width="\d+" height="\d+"( viewBox="[^"]*")?/, `<svg$1 width="${w}" height="${h}" viewBox="${sx} ${sy} ${w} ${h}"`)
writeFileSync(file, svg)
console.log(`${file}: cropped to ${w} x ${h}`)
