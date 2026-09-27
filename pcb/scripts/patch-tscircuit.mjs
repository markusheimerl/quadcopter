// Small fixes for tscircuit bugs that affect this board, applied after
// `npm install`. Each patch is idempotent and fails loudly if the code it
// patches ever changes.
import { readFileSync, writeFileSync } from "node:fs"

const patch = (path, mark, pairs, what) => {
  const file = new URL(`../node_modules/${path}`, import.meta.url)
  let src = readFileSync(file, "utf8")
  if (src.includes(mark)) return
  for (const [a, b] of pairs) {
    if (src.split(a).length !== 2) throw new Error(`patch-tscircuit: pattern not found exactly once in ${path}:\n${a}`)
    src = src.replace(a, b)
  }
  writeFileSync(file, src)
  console.log(`patched ${path}: ${what}`)
}

// core 0.0.1971: when a <chip> has a schPinArrangement, pins with several
// separate pads (ESP32 exposed pad, USB-C shell) lose ALL their pads, which
// then float on the PCB. Without an arrangement tscircuit creates extra
// "internal" pins for such pads; make it do the same with an arrangement.
const MARK1 = "/* quad-patch: internal ports with arrangement */"
patch("@tscircuit/core/dist/index.js", MARK1, [
  [`    if (!this._getSchematicPortArrangement()) {
      const hasReactSymbol = isValidElement(this.props.symbol);`,
   `    ${MARK1}
    const hasSchArrangement = Boolean(this._getSchematicPortArrangement());
    {
      const hasReactSymbol = isValidElement(this.props.symbol);`],
  [`        for (const port of portsFromFootprint) {
          if (!port._isPrimaryPort) {
            portsToCreate.push(port);
            continue;
          }
          const matchingPort`,
   `        for (const port of portsFromFootprint) {
          if (!port._isPrimaryPort) {
            portsToCreate.push(port);
            continue;
          }
          if (hasSchArrangement) continue;
          const matchingPort`],
], "internal ports for multi-pad pins with schPinArrangement")

// cli gerber export: paste for rotated pads (the 45° motor connectors) is a
// flash rotated with %LR, which not every CAM tool honours. Draw it as a
// plain polygon region instead, like the copper pad itself.
const MARK2 = "/* quad-patch: rotated paste as region */"
patch("@tscircuit/cli/dist/cli/main.js", MARK2, [
  [`        if (elm.shape === "oval") {
          addConfigIfNew(REGION_APERTURE_CONFIG);
          continue;
        }
        addConfigIfNew(getApertureConfigFromPcbSolderPaste(elm));`,
   `        if (elm.shape === "oval" || elm.shape === "rotated_rect") {
          addConfigIfNew(REGION_APERTURE_CONFIG);
          continue;
        }
        addConfigIfNew(getApertureConfigFromPcbSolderPaste(elm));`],
  [`          const glayer = glayers[getGerberLayerName(layer, "paste")];
          let rotation = 0;`,
   `          const glayer = glayers[getGerberLayerName(layer, "paste")];
          if (element.shape === "rotated_rect") { ${MARK2}
            const a = (element.ccw_rotation ?? 0) * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), w = element.width / 2, h = element.height / 2;
            addClosedRegionFromPoints({ target: glayer, apertureSource: glayer,
              points: [[-w, -h], [w, -h], [w, h], [-w, h]].map(([x, y]) => ({ x: element.x + x * c - y * s, y: element.y + x * s + y * c })) });
            continue;
          }
          let rotation = 0;`],
], "rotated solder paste exported as a region")

// cli gerber export: silkscreen text is stroked at 9 % of its height, so
// 1 mm text would be 0.09 mm thin - below what JLCPCB can print (0.153 mm).
const MARK3 = "/* quad-patch: printable silkscreen text */"
patch("@tscircuit/cli/dist/cli/main.js", MARK3, [
  [`}, getApertureConfigFromPcbSilkscreenText = (elm) => {
  if ("font_size" in elm) {
    return {
      standard_template_code: "C",
      diameter: elm.font_size * strokeWidthRatio
    };`,
   `}, getApertureConfigFromPcbSilkscreenText = (elm) => {
  if ("font_size" in elm) {
    return { ${MARK3}
      standard_template_code: "C",
      diameter: Math.max(elm.font_size * strokeWidthRatio, 0.16)
    };`],
], "silkscreen text stroke >= 0.16 mm")

// schematic: a pin the tidy first pass left unwired gets joined to the
// nearest pin of its net by a wire of any length (e.g. across the whole MCU).
// Keep the distance limit (<board schMaxTraceDistance>) for those too, so the
// pin gets its power symbol / net label instead.
const MARK4 = "/* quad-patch: no long recovery wires */"
patch("@tscircuit/schematic-trace-solver/dist/index.js", MARK4, [
  [`          if (isNamedTwoPinConnection && manhattanDistance(sourcePin, targetPin) > this.maxMspPairDistance) {`,
   `          if (manhattanDistance(sourcePin, targetPin) > this.maxMspPairDistance) { ${MARK4}`],
], "schematic long-distance pairs respect schMaxTraceDistance")
