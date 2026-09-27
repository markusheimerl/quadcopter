// Silkscreen clean-up on the finished circuit json, so the files show exactly
// what JLCPCB will print:
//  * footprint reference designators (0.4 mm, too small to print) and the
//    hairline outlines of 0402/0603 passives are dropped; index.tsx adds the
//    labels that matter,
//  * every other line gets a printable width and is cut back from pads, holes,
//    vias and the board edge (JLCPCB would clip it there anyway).
type P = { x: number; y: number }
const MIN_WIDTH = 0.15, PAD_GAP = 0.15, EDGE_GAP = 0.25, STEP = 0.02

const segDist = (p: P, a: P, b: P) => {
  const [dx, dy] = [b.x - a.x, b.y - a.y]
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy || 1)))
  return Math.hypot(p.x - a.x - t * dx, p.y - a.y - t * dy)
}
const inside = (p: P, poly: P[]) => {
  let r = false
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [a, b] = [poly[i], poly[j]]
    if (a.y > p.y !== b.y > p.y && p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x) r = !r
  }
  return r
}
const ring = (poly: P[]) => poly.map((a, i) => [a, poly[(i + 1) % poly.length]] as const)
// distance from p to a pad / hole / via outline (0 inside)
const rectDist = (p: P, c: P, w: number, h: number, deg = 0) => {
  const a = (-deg * Math.PI) / 180, [x, y] = [p.x - c.x, p.y - c.y]
  const [lx, ly] = [x * Math.cos(a) - y * Math.sin(a), x * Math.sin(a) + y * Math.cos(a)]
  return Math.hypot(Math.max(Math.abs(lx) - w / 2, 0), Math.max(Math.abs(ly) - h / 2, 0))
}
const pillDist = (p: P, c: P, w: number, h: number, deg = 0) => {
  const r = Math.min(w, h) / 2, a = (deg * Math.PI) / 180, l = Math.abs(w - h) / 2
  const [ux, uy] = w >= h ? [Math.cos(a), Math.sin(a)] : [-Math.sin(a), Math.cos(a)]
  return Math.max(0, segDist(p, { x: c.x - ux * l, y: c.y - uy * l }, { x: c.x + ux * l, y: c.y + uy * l }) - r)
}
const shapeDist = (e: any): ((p: P) => number) => {
  const c = { x: e.x, y: e.y }, w = e.width ?? e.outer_width ?? e.outer_diameter ?? e.hole_width ?? e.hole_diameter
  const h = e.height ?? e.outer_height ?? e.outer_diameter ?? e.hole_height ?? e.hole_diameter
  const deg = e.ccw_rotation ?? e.rotation ?? 0, shape = e.shape ?? e.hole_shape
  if (shape === "polygon") return (p) => (inside(p, e.points) ? 0 : Math.min(...ring(e.points).map(([a, b]) => segDist(p, a, b))))
  if (shape === "circle" || e.type === "pcb_via") return (p) => Math.max(0, Math.hypot(p.x - c.x, p.y - c.y) - (e.radius ?? w / 2))
  if (/pill|oval/.test(shape)) return (p) => pillDist(p, c, w, h, deg)
  return (p) => rectDist(p, c, w, h, deg)
}

// `drop(ref, element)` removes more footprint silk by hand
export function cleanSilkscreen(cj: any[], drop: (ref: string, e: any) => boolean = () => false): any[] {
  const src = new Map(cj.filter((e) => e.type === "source_component").map((e) => [e.source_component_id, e]))
  const comp = new Map(cj.filter((e) => e.type === "pcb_component").map((e) => [e.pcb_component_id, src.get(e.source_component_id)]))
  const passive = (e: any) => ["simple_resistor", "simple_capacitor"].includes(comp.get(e.pcb_component_id)?.ftype)
  const outline: P[] = cj.find((e) => e.type === "pcb_board").outline
  const edges = ring(outline)
  const obstacles: Record<string, ((p: P) => number)[]> = { top: [], bottom: [] }
  for (const e of cj) {
    if (e.type === "pcb_smtpad") obstacles[e.layer].push(shapeDist(e))
    if (["pcb_plated_hole", "pcb_hole", "pcb_via"].includes(e.type)) for (const l of ["top", "bottom"]) obstacles[l].push(shapeDist(e))
  }
  const ok = (p: P, layer: string, w: number) =>
    inside(p, outline) && edges.every(([a, b]) => segDist(p, a, b) >= EDGE_GAP + w / 2) && obstacles[layer].every((d) => d(p) >= PAD_GAP + w / 2)

  // cut one polyline into the runs that keep their distance
  const clip = (route: P[], layer: string, w: number): P[][] => {
    const runs: P[][] = []
    let run: P[] = []
    // (stubs under 0.4 mm left over from clipping are dropped too)
    const len = (r: P[]) => r.slice(1).reduce((l, p, i) => l + Math.hypot(p.x - r[i].x, p.y - r[i].y), 0)
    const end = () => { if (run.length > 1 && len(run) >= 0.4) runs.push(run); run = [] }
    route.forEach((b, i) => {
      const a = route[i - 1] ?? b, n = i === 0 ? 1 : Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / STEP))
      for (let k = i === 0 ? n : 1; k <= n; k++) {
        const p = { x: a.x + ((b.x - a.x) * k) / n, y: a.y + ((b.y - a.y) * k) / n }
        if (!ok(p, layer, w)) { end(); continue }
        // along a straight piece only its two ends are kept
        if (run.length > 1 && k > 1) run[run.length - 1] = p
        else run.push(p)
      }
    })
    end()
    return runs
  }

  const out: any[] = []
  for (const e of cj) {
    if (e.type === "pcb_silkscreen_text" && e.pcb_component_id && e.text === comp.get(e.pcb_component_id)?.name) continue
    if (e.type !== "pcb_silkscreen_path" && e.type !== "pcb_silkscreen_circle") { out.push(e); continue }
    if (passive(e) || drop(comp.get(e.pcb_component_id)?.name, e)) continue
    // circles become 48-gons so they can be clipped like any other line
    const route: P[] = e.route ?? Array.from({ length: 49 }, (_, k) => ({ x: e.center.x + e.radius * Math.cos((k * Math.PI) / 24), y: e.center.y + e.radius * Math.sin((k * Math.PI) / 24) }))
    const w = Math.max(e.stroke_width ?? 0, MIN_WIDTH), id = e.pcb_silkscreen_path_id ?? e.pcb_silkscreen_circle_id
    clip(route, e.layer, w).forEach((r, k) => out.push({ type: "pcb_silkscreen_path", pcb_silkscreen_path_id: `${id}_${k}`, pcb_component_id: e.pcb_component_id, layer: e.layer, route: r, stroke_width: w }))
  }
  return out
}
