/*
 * Micro quadcopter: flight controller and frame on one PCB.
 *
 * Board axes: +y = nose, +x = right (top view). The four arms are the frame;
 * motors sit on the arm tips, wired back to CN1..CN4 (1.25 mm, 2 pin).
 *
 * Pin map (matches firmware/main/main.c):
 *   IO1 IO2 IO16 IO4 motors M0 FR / M1 BR / M2 BL / M3 FL (low side N-FET, 20 kHz PWM)
 *   IO11/IO12 I2C SDA/SCL -> BMI323 (addr 0x68)
 *   IO21      user LED, active low
 *   IO19/IO20 native USB (D-/D+), programming + console
 *   IO10      battery voltage / 2 (ADC1)
 *
 * Every part is a JLCPCB stock part (C-number given), passives are "basic".
 * All copper is drawn by hand below (no autorouter): see "hand-drawn copper".
 */

import { Fragment } from "react"
import type { SchematicPortArrangement as Pins } from "@tscircuit/props"

const JLC = (n: string) => ({ jlcpcb: [n] })
// net label: [net, port, x, y, side the wire enters from]
type LabelAt = [string, string, number, number, "left" | "right" | "top" | "bottom"]
const Label = ({ at: [net, port, x, y, side] }: { at: LabelAt }) => (
  <netlabel net={net} connectsTo={port} schX={x} schY={y} anchorSide={side} />
)
// schematic position of a part: its row in SCH (below) or explicit [x, y, rotation]
const sch = (at: string | number[]) => {
  const [schX, schY, schRotation = 0] = typeof at === "string" ? SCH[at] : at
  return { schX, schY, schRotation }
}

// ---- pin names (module pin order) and schematic pin layouts
// (inputs left, outputs right, ground at the bottom)
const pinNames = (names: string[]) => Object.fromEntries(names.map((n, i) => [`pin${i + 1}`, n]))
const ESP_LABELS = pinNames(["GND1", "3V3", "EN", "IO4", "IO5", "IO6", "IO7", "IO15", "IO16", "IO17", "IO18", "IO8", "IO19", "IO20", "IO3", "IO46", "IO9", "IO10", "IO11", "IO12", "IO13", "IO14", "IO21", "IO47", "IO48", "IO45", "IO0", "IO35", "IO36", "IO37", "IO38", "IO39", "IO40", "IO41", "IO42", "RXD0", "TXD0", "IO2", "IO1", "GND2"]) // pin41 = exposed GND pad, named by its footprint
const ESP_NC_LEFT = ["IO3", "IO5", "IO6", "IO7", "IO8", "IO9", "IO13", "IO14", "IO15", "IO17", "IO18", "IO35", "IO36", "IO37", "IO38"]
const ESP_NC_RIGHT = ["IO39", "IO40", "IO41", "IO42", "IO45", "IO46", "IO47", "IO48"]
const ESP_PINS: Pins = {
  leftSide: { direction: "top-to-bottom", pins: ["3V3", "EN", "IO0", ...ESP_NC_LEFT] },
  rightSide: { direction: "top-to-bottom", pins: ["IO1", "IO2", "IO16", "IO4", "IO11", "IO12", "IO19", "IO20", "TXD0", "RXD0", "IO21", "IO10", ...ESP_NC_RIGHT] },
  bottomSide: { direction: "left-to-right", pins: ["GND1", "GND2", 41] },
}
const ESP_GAPS = { EN: { topMargin: 0.4 }, IO0: { topMargin: 2.4 }, IO3: { topMargin: 0.2 }, IO11: { topMargin: 0.2 }, IO19: { topMargin: 0.2 }, TXD0: { topMargin: 0.2 }, RXD0: { topMargin: 0.2 }, IO21: { topMargin: 0.4 }, IO10: { topMargin: 1.4 } }
const NMOS_PINS: Pins = { leftSide: { direction: "top-to-bottom", pins: ["G"] }, topSide: { direction: "left-to-right", pins: ["D"] }, bottomSide: { direction: "left-to-right", pins: ["S"] } }
const CONN_PINS: Pins = { leftSide: { direction: "top-to-bottom", pins: ["pin1", "pin2"] }, rightSide: { direction: "top-to-bottom", pins: ["pin3", "pin4"] } }

// schematic block frame with its title in the top-left corner
const Frame = ({ x1, x2, y1, y2, title }: { x1: number; x2: number; y1: number; y2: number; title: string }) => (
  <>
    <schematicrect schX={(x1 + x2) / 2} schY={(y1 + y2) / 2} width={x2 - x1} height={y2 - y1} strokeWidth={0.02} color="#9a9a9a" isDashed />
    <schematictext text={title} schX={x1 + 0.2} schY={y2 - 0.25} fontSize={0.22} anchor="left" color="#333" />
  </>
)

// pin map and conventions, for whoever writes the firmware
const NOTES = [
  ["IO1 IO2 IO16 IO4", "PWM0-3 → motors M0 FR · M1 BR · M2 BL · M3 FL (20 kHz, high = on)"],
  ["IO11 IO12", "I2C SDA · SCL → BMI323 at 0x68 (SDO low; CSB high = I2C)"],
  ["IO10", "battery voltage / 2 (ADC1_CH9)"],
  ["IO21", "LED1, active low"],
  ["IO19 IO20", "native USB D- · D+ (flashing + console)"],
  ["IO0 · EN", "BOOT · RESET buttons"],
  ["TXD0 RXD0", "UART0 pads TP1 · TP2 on the back (TP3 = GND)"],
  ["IMU axes", "+X nose, +Y left, +Z up"],
]
const Notes = ({ x, y }: { x: number; y: number }) => (
  <>
    {NOTES.map(([k, v], i) => (
      <Fragment key={k}>
        <schematictext text={k} schX={x} schY={y - 0.45 * i} fontSize={0.16} anchor="left" color="#222" />
        <schematictext text={v} schX={x + 2.3} schY={y - 0.45 * i} fontSize={0.16} anchor="left" color="#444" />
      </Fragment>
    ))}
  </>
)

// ---- board outline: 44.8 mm square core with concave sides + 4 diagonal arms
const ARM = [[23.44, 21.32], [37.56, 35.44], [35.44, 37.56], [21.32, 23.44]]
const rot = ([x, y]: number[], k: number): number[] =>
  [[x, y], [-y, x], [-x, -y], [y, -x]][k % 4]
const arc = (p: number[], q: number[], sag = 3.6, n = 12) => {
  const [mx, my] = [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2]
  const c = Math.hypot(q[0] - p[0], q[1] - p[1])
  const [ex, ey] = [(q[0] - p[0]) / c, (q[1] - p[1]) / c]
  const [ux, uy] = [-mx / Math.hypot(mx, my), -my / Math.hypot(mx, my)]
  const R = (c * c / 4 + sag * sag) / (2 * sag)
  return Array.from({ length: n - 1 }, (_, i) => {
    const x = ((i + 1) / n - 0.5) * c
    const b = Math.sqrt(R * R - x * x) - (R - sag)
    return [mx + ex * x + ux * b, my + ey * x + uy * b]
  })
}
const outline = [0, 1, 2, 3]
  .flatMap((k) => [...ARM.map((p) => rot(p, k)), ...arc(rot(ARM[3], k), rot(ARM[0], k + 1))])
  .map(([x, y]) => ({ x, y }))

// ---- PCB placement: [x, y, rotation] of every part, in one table so the
// hand-drawn tracks below can refer to the same numbers.
type Place = [number, number, number?]
const AT: Record<string, Place> = {
  U1: [0, 10], // module; antenna hangs over the nose edge
  C1: [-10.85, 18.6], R9: [-10.22, 17.14, 270], // at module pins 3V3 / EN
  C3: [-10.85, 0.2, 270], SW1: [-16.6, 0], // RESET on the left edge
  SW2: [16.6, 0], R10: [8.5, -1.2], // BOOT on the right edge
  LED1: [3.9, -1.2], R11: [6.6, -1.2],
  U2: [-1.405, -4.2], C4: [-2.6, -6.7, 270], C5: [1.5, -5.3, 90], R12: [-3.3, -2.1], R13: [0.4, -2.0, 180], // IMU
  U3: [-9.0, -6.0, 270], C6: [-10.3, -8.5, 180], C7: [-6.4, -5.9, 270], C2: [-4.9, -5.9, 270], // 3.3 V LDO
  R19: [-7.9, -1.8], R20: [-6.3, -1.8, 270], C10: [-5.3, -1.8, 270], // battery sense
  USB1: [-4.0, -15.6], R14: [-6.1, -10.9, 90], R15: [-2.25, -10.7, 90],
  CN5: [5.4, -14.9], C8: [5.4, -10.9], // battery
  U4: [7.8, -7.6], R16: [6.85, -4.6, 90], R18: [8.75, -4.1, 90], C9: [10.0, -3.7, 270], LED2: [3.75, -8.7, 180], R17: [3.0, -6.8, 90], // charger
  C11: [11.2, -6.15, 90], C12: [-10.6, -10.9, 90], // VBAT at the motor drivers
}
// motor channel k sits in corner (sx, sy); its layout is mirrored into each
// corner. SOT-23 pinouts can't be mirrored, so the gate is on the inner side
// in FR/BL and on the outer side in FL/BR.
const MOTORS = [
  { i: 0, sx: 1, sy: 1, name: "front right", spin: "CCW", io: 1 },
  { i: 1, sx: 1, sy: -1, name: "back right", spin: "CW", io: 2 },
  { i: 2, sx: -1, sy: -1, name: "back left", spin: "CCW", io: 16 },
  { i: 3, sx: -1, sy: 1, name: "front left", spin: "CW", io: 4 },
]
const RAIL = 12.425 // VBAT rails run straight down at x = ±RAIL (top layer)
for (const { i, sx, sy } of MOTORS) {
  const gateInner = sx * sy > 0
  AT[`CN${i + 1}`] = [15.6 * sx, 15.6 * sy, (Math.atan2(sy, sx) * 180) / Math.PI + 90]
  AT[`D${i + 1}`] = [(RAIL + 1.2) * sx, 11.95 * sy, (Math.atan2(-sy, sx) * 180) / Math.PI] // cathode on the rail
  AT[`Q${i + 1}`] = [14.825 * sx, 7.5 * sy, sy > 0 ? 270 : 90] // drain right under the anode
  AT[`R${i + 1}`] = [(gateInner ? 14.025 : 15.775) * sx, 4.2 * sy, sy > 0 ? 90 : 270] // 47R, PWM side toward y = 0
  AT[`R${i + 5}`] = [(gateInner ? 15.525 : 17.275) * sx, 4.2 * sy, sy > 0 ? 270 : 90] // 10k, GND side toward y = 0
}
const pcb = (n: string) => ({ pcbX: AT[n][0], pcbY: AT[n][1], pcbRotation: AT[n][2] ?? 0 })

// ---- schematic placement: [x, y, rotation] of every part but the motor
// channels (those are drawn side by side by <Motor>)
const SCH: Record<string, Place> = {
  // USB-C + charger
  USB1: [-6.5, 8.8], R14: [-7.6, 6.2, 270], R15: [-6.6, 6.2, 270],
  R18: [-3.0, 9.6, 270], C9: [-3.0, 8.7, 270],
  U4: [-0.8, 10.0], R16: [-0.5, 8.5, 270], LED2: [1.7, 10.0, 180], R17: [2.8, 10.3, 90],
  // battery + 3.3 V
  CN5: [5.6, 8.5], C8: [7.1, 8.9, 270], C11: [8.2, 8.6, 270], C12: [9.2, 8.6, 270],
  C6: [10.2, 8.6, 270], U3: [12.4, 8.8], C7: [14.4, 8.6, 270],
  // MCU: ESP32 with its supply caps, reset/boot, LED, VBAT/2 divider, UART pads
  U1: [-1.4, 0], C2: [-7.4, 3.9, 270], C1: [-6.3, 3.9, 270],
  C3: [-4.1, 2.0, 270], R9: [-4.8, 3.2, 270], SW1: [-5.6, 1.8, 90],
  R10: [-3.6, 0.3, 270], SW2: [-4.5, -1.1, 270],
  TP1: [0.9, 1.0], TP2: [0.9, 0.6], TP3: [-0.3, -4.1, 90],
  LED1: [2.0, 0.0, 180], R11: [3.1, 0.3, 90],
  R19: [1.3, -1.3, 270], R20: [1.3, -2.2, 270], C10: [2.4, -2.2, 270],
  // IMU
  U2: [10.4, 1.8], C4: [12.5, 3.2, 270], C5: [13.6, 3.2, 270], R12: [8.6, 2.5, 270], R13: [7.3, 2.5, 270],
}
const MOTOR_SCH = (i: number) => [-5.0 + i * 5.9, -7.5] // motor channels side by side
// signals between blocks meet at net labels (plus one supply symbol the
// schematic solver would otherwise leave out)
const LABELS: LabelAt[] = [
  ["USB_DN", "USB1.Dn1", -4.4, 9.4, "left"], ["USB_DP", "USB1.Dp1", -4.4, 8.8, "left"],
  ["PWM0", "U1.IO1", 0.6, 3.2, "left"], ["PWM1", "U1.IO2", 0.6, 3.0, "left"],
  ["PWM2", "U1.IO16", 0.6, 2.8, "left"], ["PWM3", "U1.IO4", 0.6, 2.6, "left"],
  ["SDA", "U1.IO11", 0.6, 2.2, "left"], ["SCL", "U1.IO12", 0.6, 2.0, "left"],
  ["SDA", "U2.SDX", 8.0, 1.9, "right"], ["SCL", "U2.SCX", 6.8, 1.7, "right"],
  ["USB_DN", "U1.IO19", 0.6, 1.6, "left"], ["USB_DP", "U1.IO20", 0.6, 1.4, "left"],
  ["V3V3", "R11.pin2", 3.1, 0.9, "bottom"],
  ...[0, 1, 2, 3].map((i): LabelAt => [`PWM${i}`, `R${i + 1}.pin1`, MOTOR_SCH(i)[0] - 2.2, MOTOR_SCH(i)[1], "right"]),
]

// ---- hand-drawn copper. A track runs from one port to another through
// global board points (mm, or a port name); vias carry a net and let a track
// change layers. Everything is drawn here, nothing is autorouted.
type XY = [number, number]
const local = (ref: string, [x, y]: XY) => {
  const [cx, cy, r = 0] = AT[ref]
  const a = (-r * Math.PI) / 180
  return { x: (x - cx) * Math.cos(a) - (y - cy) * Math.sin(a), y: (x - cx) * Math.sin(a) + (y - cy) * Math.cos(a) }
}
const VIAS: [string, string, number, number][] = [] // name, net, x, y
const TRACKS: [string, string, number, (XY | string)[]][] = [] // from, to, width, points
const via = (name: string, net: string, x: number, y: number) => { VIAS.push([name, net, x, y]); AT[name] = [x, y] }
const track = (from: string, to: string, w: number, ...pts: (XY | string)[]) => { TRACKS.push([from, to, w, pts]) }
const Copper = () => (
  <>
    {VIAS.map(([name, net, x, y]) => (
      <Fragment key={name}><via name={name} pcbX={x} pcbY={y} connectsTo={`net.${net}`} holeDiameter="0.3mm" outerDiameter="0.6mm" /></Fragment>
    ))}
    {TRACKS.map(([from, to, w, pts], k) => (
      <Fragment key={k}><trace from={from} to={to} thickness={`${w}mm`} pcbPath={pts.map((p) => (typeof p === "string" ? p : local(from.split(".")[0], p)))} /></Fragment>
    ))}
  </>
)

// motor channels: each corner mirrors the front-right one. (u, v) are
// coordinates along the arm and across it (v > 0 toward the board centre line).
for (const { i, sx, sy } of MOTORS) {
  const k = i + 1, gateInner = sx * sy > 0
  const m = (x: number, y: number): XY => [x * sx, y * sy]
  const uv = (u: number, v: number): XY => m((u - v) / Math.SQRT2, (u + v) / Math.SQRT2)
  const [plus, minus] = i % 2 === 0 ? ["pin2", "pin1"] : ["pin1", "pin2"]
  const rs = gateInner ? 14.025 : 15.775, rp = rs + 1.5 // gate resistor columns
  track(`CN${k}.${plus}`, `D${k}.cathode`, 0.8) // VBAT
  track(`CN${k}.${minus}`, `D${k}.anode`, 0.6, uv(19.3, -0.9), uv(18.12, -0.9)) // motor -, under the diode body
  track(`D${k}.anode`, `Q${k}.D`, 0.8)
  track(`Q${k}.G`, `R${k}.pin2`, 0.3)
  track(`R${k}.pin2`, `R${k + 4}.pin1`, 0.3)
  // source to ground: two vias beside the FET
  const s = gateInner ? [16.9, 16.9] : [13.875, 13.875], sv = gateInner ? [6.9, 6.1] : [5.4, 4.6]
  for (const j of [0, 1]) { via(`GQ${k}${j}`, "GND", ...m(s[j], sv[j])); track(`Q${k}.S`, `GQ${k}${j}.top`, 0.6) }
  via(`GR${k}`, "GND", ...m(rp, 2.4)); track(`R${k + 4}.pin2`, `GR${k}.top`, 0.4)
}
track("D1.cathode", "D2.cathode", 1.0) // right VBAT rail
track("D4.cathode", "D3.cathode", 1.0) // left VBAT rail

// module ground: four vias between the nine squares of the exposed pad, each
// tied to its four neighbours; GND2 gets its own via, GND1 is shared with C1
const EP = [["pin41_internal_2", "pin41_internal_1", "pin41"], ["pin41_internal_7", "pin41_internal_8", "pin41_internal_6"], ["pin41_internal_3", "pin41_internal_4", "pin41_internal_5"]]
for (const [r, c] of [[0, 0], [0, 1], [1, 0], [1, 1]]) {
  via(`GE${r}${c}`, "GND", -2.2 + 1.4 * c, 10.625 + 1.4 * r)
  for (const [dr, dc] of [[0, 0], [0, 1], [1, 0], [1, 1]]) track(`U1.${EP[r + dr][c + dc]}`, `GE${r}${c}.top`, 0.3)
}
via("GU2", "GND", 10.0, 19.5); track("U1.GND2", "GU2.top", 0.4)
track("C1.pin2", "U1.GND1", 0.3, [-9.8, 18.6]); via("GU1", "GND", -10.3, 19.55); track("C1.pin2", "GU1.top", 0.3)
// 3V3 into module pin 2 (decoupled by C1 right there); R9 pulls EN up
track("U1.3V3", "C1.pin1", 0.3, [-11.2, 17.775], [-11.2, 18.6])
track("R9.pin1", "U1.3V3", 0.3); track("R9.pin2", "U1.EN", 0.25)
// left of the module: 3V3, EN, PWM3, PWM2 run down side by side
// (they sit further in while passing the FL diode, then step out in turn)
const L3 = -11.55, LEN = -11.075, LP3 = -10.45, LP2 = -9.825
track("U1.EN", "ENT.top", 0.25, [-10.725, 16.505], [-10.725, 11.96], [LEN, 11.61], [LEN, 2.0], [-10.9, 1.825])
track("U1.IO4", "PT3.top", 0.25, [-10.15, 15.235], [-10.15, 10.75], [LP3, 10.45])
track("U1.IO16", "PT2.top", 0.25, [LP2, 8.885])
// PWM3 / PWM2 / EN dive under the rail to their motor channel and the RESET button
via("PT3", "PWM3", LP3, 2.4); via("PB3", "PWM3", -15.775, 2.4)
track("PT3.bottom", "PB3.bottom", 0.25); track("PB3.top", "R4.pin1", 0.25)
via("PT2", "PWM2", LP2, -2.7); via("PB2", "PWM2", -14.025, -2.4)
track("PT2.bottom", "PB2.bottom", 0.25); track("PB2.top", "R3.pin1", 0.25)
via("ENT", "EN", -10.9, 1.5); via("ENB", "EN", -13.5, 1.5)
track("ENT.top", "C3.pin1", 0.25); track("ENT.bottom", "ENB.bottom", 0.25); track("ENB.top", "SW1.pin2", 0.25)
via("GC3", "GND", -10.85, -1.0); track("C3.pin2", "GC3.top", 0.3)
via("GSW1", "GND", -18.8, 1.5); track("SW1.pin1", "GSW1.top", 0.4)
// 3V3 down to the LDO
track("U1.3V3", "U3.OUT", 0.3, [-11.2, 17.775], [-11.2, 13.11], [L3, 12.76], [L3, -3.4], [-7.8, -3.4])
// USB data leave the module pins inward and run on the bottom layer
via("DNT", "USB_DN", -7.3, 3.805); via("DPT", "USB_DP", -7.3, 2.535)
track("U1.IO19", "DNT.top", 0.25); track("U1.IO20", "DPT.top", 0.25)

// right of the module: PWM0, PWM1 down to their channels, IO0 to BOOT; the
// UART0 pins drop to test pads on the back
track("U1.IO1", "PT0.top", 0.25, [11.2, 17.775])
via("PT0", "PWM0", 11.2, 2.4); via("PB0", "PWM0", 14.025, 2.4)
track("PT0.bottom", "PB0.bottom", 0.25); track("PB0.top", "R1.pin1", 0.25)
track("U1.IO2", "PT1.top", 0.25, [10.725, 16.505], [10.725, 13.2], [10.275, 12.75])
via("PT1", "PWM1", 10.275, -2.4); via("PB1", "PWM1", 15.775, -2.4)
track("PT1.bottom", "PB1.bottom", 0.25); track("PB1.top", "R2.pin1", 0.25)
track("U1.IO0", "R10.pin2", 0.25, [9.825, 2.535], [9.825, -1.2])
via("IOT", "IO0", 9.6, 0.3); via("IOB", "IO0", 13.5, 1.5)
track("IOT.bottom", "IOB.bottom", 0.25); track("IOB.top", "SW2.pin1", 0.25)
via("GSW2", "GND", 18.8, 1.5); track("SW2.pin2", "GSW2.top", 0.4)
// local VBAT caps next to the rails
track("C11.pin1", "D2.cathode", 0.5, [RAIL, -6.975])
via("TXT", "TXD0", 10.1, 15.235); track("U1.TXD0", "TXT.top", 0.25); track("TXT.bottom", "TP1.pin1", 0.25)
via("RXT", "RXD0", 10.1, 13.965); track("U1.RXD0", "RXT.top", 0.25); track("RXT.bottom", "TP2.pin1", 0.25)

// status LED on IO21
track("U1.IO21", "LED1.cathode", 0.25)
track("LED1.anode", "R11.pin1", 0.25)
track("R11.pin2", "R10.pin1", 0.3)
// IMU: I2C straight down from the module, pull-ups beside it
track("U1.IO11", "U2.SDX", 0.25)
track("R12.pin2", "U1.IO11", 0.25, [-1.905, -2.1])
track("U1.IO12", "U2.SCX", 0.25, [-0.635, -2.3], [-1.405, -2.3])
track("R13.pin2", "U1.IO12", 0.25, [-0.635, -2.0])
via("GSDO", "GND", -3.4, -3.45); track("U2.SDO", "GSDO.top", 0.25)
via("GIMU", "GND", -1.155, -6.3); track("U2.GNDIO", "GIMU.top", 0.25, [-1.405, -5.8]); track("U2.GND", "GIMU.top", 0.25, [-0.905, -5.8])
via("GC4", "GND", -1.7, -7.2); track("C4.pin2", "GC4.top", 0.3)
via("GC5", "GND", 2.0, -3.9); track("C5.pin2", "GC5.top", 0.3)
// 3V3 from the LDO: east past its output caps, up the IMU's west side to
// R12, under the IMU and up its east side to VDD, CSB, R13 and on to R11/R10
track("U3.OUT", "C7.pin1", 0.4); track("C7.pin1", "C2.pin1", 0.4)
track("C2.pin1", "R12.pin1", 0.3, [-4.0, -5.05], [-4.0, -4.2], [-4.2, -4.0], [-4.2, -2.49])
track("C2.pin1", "U2.VDDIO", 0.3, [-4.0, -5.05], [-4.0, -6.19], [-2.6, -6.19], [-1.905, -5.7])
track("U2.VDDIO", "C4.pin1", 0.25, [-1.905, -5.7])
track("U2.VDDIO", "R13.pin1", 0.3, [-1.905, -5.7], [-2.6, -6.19], [-4.0, -6.19], [-4.0, -7.9], [0.7, -7.9], [0.7, -2.0])
track("U2.VDD", "C5.pin1", 0.25, [0.7, -4.95], [0.7, -5.81])
track("U2.CSB", "R13.pin1", 0.2, [-0.905, -2.75], [0.7, -2.75])
track("R13.pin1", "R11.pin2", 0.3, [1.2, -2.0], [7.11, -2.0])
via("GC7", "GND", -6.4, -7.6); track("C7.pin2", "GC7.top", 0.4)
via("GC2", "GND", -4.9, -7.6); track("C2.pin2", "GC2.top", 0.4)
// LDO: VBAT in from the left rail, EN tied to IN under the body
track("U3.IN", "U3.EN", 0.4, [-9.15, -5.05], [-9.15, -6.95])
track("C12.pin1", "D3.cathode", 0.5, [-RAIL, -11.725]); track("U3.IN", "D4.cathode", 0.5, [-RAIL, -5.05])
track("U3.EN", "C6.pin1", 0.4)
via("GU3", "GND", -11.35, -6.0); track("U3.GND", "GU3.top", 0.4)
via("GC6", "GND", -11.125, -9.4); track("C6.pin2", "GC6.top", 0.4)
via("GC12", "GND", -9.6, -10.1); track("C12.pin2", "GC12.top", 0.4)
// battery sense: VBAT / 2 on IO10; VBAT comes under the left corridor
track("U1.IO10", "R19.pin2", 0.25, [-3.175, -1.29], [-6.3, -1.29])
track("C10.pin1", "R20.pin1", 0.25)
via("VST", "VBAT", RAIL * -1, -1.8); via("VSB", "VBAT", -9.2, -1.8)
track("VST.bottom", "VSB.bottom", 0.3); track("VSB.top", "R19.pin1", 0.3)
via("GR20", "GND", -6.3, -3.2); track("R20.pin2", "GR20.top", 0.3)
via("GC10", "GND", -5.3, -3.2); track("C10.pin2", "GC10.top", 0.3)

// charger: VBUS arrives on the bottom layer (from the USB-C connector) at two
// vias; BAT goes straight to the right rail, GND straight into the bulk cap
via("VBV", "VBUS", 9.9, -5.6); track("VBV.top", "U4.VCC", 0.4); track("VBV.top", "R18.pin1", 0.4)
track("R18.pin2", "C9.pin1", 0.3)
via("GC9", "GND", 10.9, -4.3); track("C9.pin2", "GC9.top", 0.3); track("C11.pin2", "GC9.top", 0.4)
track("U4.PROG", "R16.pin1", 0.25)
via("GR16", "GND", 6.85, -3.2); track("R16.pin2", "GR16.top", 0.3)
track("U4.CHRG", "LED2.cathode", 0.25)
track("LED2.anode", "R17.pin1", 0.25)
via("VBR", "VBUS", 3.0, -5.4); track("R17.pin2", "VBR.top", 0.3)
track("U4.BAT", "D2.cathode", 0.5, [RAIL, -8.758])
track("U4.GND", "C8.pin2", 0.5, [7.8, -9.6])
via("GU4P", "GND", 7.8, -9.75); track("U4.GND", "GU4P.top", 0.5) // heat path: the TP4054 sheds its heat through GND
via("GU4", "GND", 8.0, -10.9); track("C8.pin2", "GU4.top", 0.5)
track("C8.pin2", "CN5.pin2", 0.8, [6.86, -11.8], [6.03, -12.4])
track("C8.pin1", "CN5.pin1", 0.8, [3.94, -11.8], [4.77, -12.4])
// the whole motor return current leaves the ground pour here: six vias
via("GCN5", "GND", 7.2, -13.1); track("CN5.pin2", "GCN5.top", 0.8)
via("GCN5B", "GND", 8.05, -12.55); via("GCN5C", "GND", 8.05, -13.65); via("GCN5D", "GND", 8.9, -13.1)
track("GCN5.top", "GCN5B.top", 0.6); track("GCN5.top", "GCN5C.top", 0.6); track("GCN5B.top", "GCN5D.top", 0.6)
track("GCN5C.top", "GCN5D.top", 0.6); track("GCN5B.top", "GU4.top", 0.6)
// battery -> bottom-layer VBAT bus under both connectors -> both rails
const BUS: XY[] = [[3.0, -12.6], [3.0, -13.4], [3.75, -12.3], [3.75, -13.1]]
BUS.forEach(([x, y], k) => { via(`VB${k}`, "VBAT", x, y); track(`VB${k}.top`, "CN5.pin1", 0.6) })
track("VB2.bottom", "VB0.bottom", 0.6) // (VB0, VB1, VB3 already sit in the bus)
for (const sx of [-1, 1]) {
  const tag = sx < 0 ? "L" : "R", d = sx < 0 ? "D3" : "D2"
  const cl: XY[] = [[11.3 * sx, -14.5], [10.6 * sx, -15.0], [11.5 * sx, -15.3]]
  cl.forEach(([x, y], k) => { via(`V${tag}${k}`, "VBAT", x, y); track(`V${tag}${k}.top`, `${d}.cathode`, 0.8) })
  track(`V${tag}1.bottom`, "VB1.bottom", 1.5, [10.6 * sx, -16.2], [2.2, -16.2], [2.2, -14.2], [3.0, -13.4])
}
// USB-C (pads 0.5 mm apart): exits fan out, the two D+ pads join on top around
// Dn1, the two D- pads join on the bottom. CC1/CC2 get their pull-downs right
// at the connector. VBUS2 feeds the charger; VBUS1 joins it on the bottom
const XU = AT.USB1[0], ux = (dx: number) => XU + dx
track("USB1.GND1", "USB1.pin13", 0.4); track("USB1.GND2", "USB1.pin14", 0.4)
track("USB1.CC1", "R14.pin1", 0.15, [ux(-1.25), -12.75], [ux(-2.1), -11.9])
track("USB1.CC2", "R15.pin1", 0.15, [ux(1.75), -11.6])
via("GR14", "GND", ux(-2.9), -10.4); track("R14.pin2", "GR14.top", 0.3)
via("GR15", "GND", ux(1.75), -9.35); track("R15.pin2", "GR15.top", 0.3)
track("USB1.Dp1", "USB1.Dp2", 0.15, [ux(-0.25), -12.75], [ux(-0.55), -12.45], [ux(-0.55), -11.2], [ux(1.05), -11.2], [ux(1.05), -12.45], [ux(0.75), -12.75])
via("DPU", "USB_DP", ux(0.25), -10.6); track("DPU.top", "USB1.Dp1", 0.15, [ux(0.25), -11.2], [ux(-0.55), -11.2], [ux(-0.55), -12.45], [ux(-0.25), -12.75])
via("DN1", "USB_DN", ux(0.25), -12.0); track("USB1.Dn1", "DN1.top", 0.15)
via("DN2", "USB_DN", ux(-1.25), -11.6); track("USB1.Dn2", "DN2.top", 0.15, [ux(-0.75), -12.75], [ux(-1.25), -12.25])
track("DN1.bottom", "DN2.bottom", 0.2, [ux(-0.2), -12.0], [ux(-0.8), -11.6])
via("VBU", "VBUS", ux(2.4), -12.3); track("USB1.VBUS2", "VBU.top", 0.4)
via("VBU1", "VBUS", ux(-2.4), -12.5); track("USB1.VBUS1", "VBU1.top", 0.3)
track("VBU1.bottom", "VBU.bottom", 0.4, [ux(-2.4), -13.4], [ux(2.4), -13.4])
track("VBU.bottom", "VBR.bottom", 0.4, [-1.0, -11.8], [-1.0, -8.6], [3.0, -8.6]); track("VBR.bottom", "VBV.bottom", 0.4, [9.7, -5.4])
// USB data on the bottom layer, from under the module down to the connector
track("DNT.bottom", "DN2.bottom", 0.25, [-8.1, 3.455], [-8.1, -8.2], [ux(-1.25), -10.95])
track("DPT.bottom", "DPU.bottom", 0.25, [-7.3, -8.0], [-4.9, -10.4], [ux(0.05), -10.4])

// ---- silkscreen (scripts/silk.ts drops the unprintable footprint texts):
// motor number + spin on each arm, + on the motor and battery connectors,
// the buttons, the charge LED; title, FRONT and the UART pads on the back
const Text = ({ t, x, y, size = 1.2, r = 0, back = false }: { t: string; x: number; y: number; size?: number; r?: number; back?: boolean }) => (
  <silkscreentext text={t} pcbX={x} pcbY={y} fontSize={size} pcbRotation={r} layer={back ? "bottom" : "top"} anchorAlignment="center" />
)
const Silkscreen = () => (
  <>
    {MOTORS.map(({ i, sx, sy, spin }) => (
      <Fragment key={i}>
        <Text t={`M${i} ${spin}`} x={28.5 * sx} y={28.5 * sy} size={1.4} r={45 * sx * sy} />
        <Text t="+" x={13.14 * sx} y={15.64 * sy} />
      </Fragment>
    ))}
    <Text t="+" x={4.77} y={-18.35} />
    <Text t="-" x={6.03} y={-18.35} />
    <Text t="RST" x={-17.9} y={-2.55} />
    <Text t="BOOT" x={17.85} y={2.6} />
    <Text t="CHG" x={0.7} y={-8.7} />
    <silkscreencircle pcbX={-3.35} pcbY={-2.8} radius={0.1} strokeWidth={0.15} /> {/* IMU pin 1 (its own dot is under a via) */}
    <silkscreenpath layer="bottom" strokeWidth={0.25} route={[{ x: -1.6, y: 16.0 }, { x: 0, y: 17.6 }, { x: 1.6, y: 16.0 }]} />
    <Text t="FRONT" x={0} y={14.6} back />
    <Text t="QUAD" x={0} y={6.4} size={3} back />
    <Text t="rev A  2026-09" x={0} y={3.4} back />
    <Text t="JLCJLCJLCJLC" x={0} y={1.2} back /> {/* JLCPCB prints its order number here ("Specify a location") */}
    <Text t="TX" x={8.3} y={16.9} back />
    <Text t="RX" x={8.3} y={12.3} back />
    <Text t="GND" x={7.8} y={10.4} back />
  </>
)

const Motor = ({ i, name, spin, io }: (typeof MOTORS)[number]) => {
  const [ox, oy] = MOTOR_SCH(i)
  // connector polarity as on the May 2026 board: pin 2 = + on FR/BL, pin 1 = + on BR/FL
  const plus = i % 2 === 0 ? "pin2" : "pin1"
  const minus = i % 2 === 0 ? "pin1" : "pin2"
  return (
    <>
      <net name={`M${i}`} />
      <schematictext text={`M${i} · ${name} · ${spin} · IO${io}`} schX={ox + 0.35} schY={oy + 2.75} fontSize={0.16} anchor="left" color="#555" />
      <connector name={`CN${i + 1}`} schSectionName="motors" manufacturerPartNumber="HC-1.25-2PWT" footprint="jlcpcb:C2845379" supplierPartNumbers={JLC("C2845379")}
        schPinArrangement={{ ...CONN_PINS, leftSide: { direction: "top-to-bottom", pins: [plus, minus] } }} noConnect={["pin3", "pin4"]} {...sch([ox + 1.2, oy + 1.7])} {...pcb(`CN${i + 1}`)}
        connections={{ [plus]: "net.VBAT", [minus]: `net.M${i}` }} />
      <diode name={`D${i + 1}`} schSectionName="motors" manufacturerPartNumber="B5819W" footprint="jlcpcb:C8598" supplierPartNumbers={JLC("C8598")} {...sch([ox, oy + 1.7, 90])}
        pinLabels={{ pin1: "cathode", pin2: "anode" }} {...pcb(`D${i + 1}`)}
        connections={{ anode: `net.M${i}`, cathode: "net.VBAT" }} />
      <chip name={`Q${i + 1}`} schSectionName="motors" manufacturerPartNumber="AO3400A" footprint="jlcpcb:C20917" supplierPartNumbers={JLC("C20917")}
        pinLabels={{ pin1: "G", pin2: "S", pin3: "D" }} schPinArrangement={NMOS_PINS} schWidth={0.8} {...sch([ox, oy])} {...pcb(`Q${i + 1}`)}
        connections={{ G: `net.G${i}`, S: "net.GND", D: `net.M${i}` }} />
      <resistor name={`R${i + 1}`} schSectionName="motors" resistance="47" footprint="0603" supplierPartNumbers={JLC("C23182")} {...sch([ox - 1.5, oy])}
        {...pcb(`R${i + 1}`)} connections={{ pin1: `net.PWM${i}`, pin2: `net.G${i}` }} />
      <resistor name={`R${i + 5}`} schSectionName="motors" resistance="10k" footprint="0603" supplierPartNumbers={JLC("C25804")} {...sch([ox - 1.05, oy - 0.85, 270])}
        {...pcb(`R${i + 5}`)} connections={{ pin1: `net.G${i}`, pin2: "net.GND" }} />
    </>
  )
}

// copper={false}: the parts only, for drawing the schematic (build.tsx)
export default ({ copper = true }: { copper?: boolean }) => (
  <board title="quad" schMaxTraceDistance={2.4} outline={outline} thickness="1.6mm" fabricatorPreset="jlcpcb_economy" routingDisabled
    minViaHoleDiameter="0.3mm" minViaPadDiameter="0.6mm">
    <net name="GND" isGroundNet />
    <net name="VBAT" isPowerNet />
    <net name="V3V3" isPowerNet />
    <net name="VBUS" isPowerNet />
    <schematictext text="QUAD · micro quadcopter flight controller · rev A" schX={-8.3} schY={12.6} fontSize={0.32} anchor="left" />
    <schematictext text="USB-C 5 V → TP4054 charges the 1S LiPo at 260 mA (R16 = 3.3 k, datasheet formula 1) · TLV757 makes 3.3 V from VBAT · ESP32-S3 drives four brushed motors through AO3400A low-side switches" schX={-8.3} schY={12.2} fontSize={0.16} anchor="left" color="#555" />
    {/* sections decide wire vs. label (links between blocks become labels);
        the frames are drawn explicitly below */}
    {["power", "battery", "mcu", "imu", "motors"].map((name) => <Fragment key={name}><schematicsection name={name} /></Fragment>)}
    <Frame x1={-8.3} x2={4.35} y1={5.1} y2={11.9} title="USB-C + charger" />
    <Frame x1={4.35} x2={15.2} y1={5.1} y2={11.9} title="Battery + 3.3 V" />
    <Frame x1={-8.3} x2={4.35} y1={-4.6} y2={5.1} title="MCU" />
    <Frame x1={4.35} x2={15.2} y1={0.1} y2={5.1} title="IMU" />
    <Frame x1={4.35} x2={15.2} y1={-4.6} y2={0.1} title="Notes" />
    <Notes x={4.6} y={-0.55} />
    <schematictext text="1S LiPo" schX={4.65} schY={7.65} fontSize={0.16} anchor="left" color="#555" />
    <schematictext text="bulk at the connector · C11, C12 at the motor rails · C6 at the LDO" schX={5.1} schY={6.8} fontSize={0.14} anchor="left" color="#555" />
    <schematictext text="EN tied to IN: on whenever a battery is in" schX={11.2} schY={7.2} fontSize={0.14} anchor="left" color="#555" />
    <schematictext text="R18 + C9: damped VBUS bypass against hot-plug spikes · CHRG low (LED2 on) while charging" schX={-5.9} schY={5.4} fontSize={0.14} anchor="left" color="#555" />
    <schematictext text="low-side switch per motor: R1-R4 limit the gate current, R5-R8 hold the gates low while the ESP32 boots, D1-D4 clamp the motor's flyback" schX={-7.9} schY={-9.45} fontSize={0.14} anchor="left" color="#555" />
    <Frame x1={-8.3} x2={15.2} y1={-9.7} y2={-4.6} title="Motor drivers" />
    <copperpour layer="bottom" connectsTo="net.GND" clearance="0.25mm" boardEdgeMargin="0.3mm" />

    {/* ---- MCU module */}
    <chip name="U1" schSectionName="mcu" manufacturerPartNumber="ESP32-S3-WROOM-1-N16R8" footprint="jlcpcb:C2913202" supplierPartNumbers={JLC("C2913202")}
      pinLabels={ESP_LABELS} schPinArrangement={ESP_PINS} schPinStyle={ESP_GAPS} schWidth={2.4} noConnect={[...ESP_NC_LEFT, ...ESP_NC_RIGHT]} {...sch("U1")}
      {...pcb("U1")}
      connections={{
        GND1: "net.GND", GND2: "net.GND", pin41: "net.GND", "3V3": "net.V3V3", EN: "net.EN", IO0: "net.IO0",
        IO1: "net.PWM0", IO2: "net.PWM1", IO16: "net.PWM2", IO4: "net.PWM3",
        IO11: "net.SDA", IO12: "net.SCL", IO19: "net.USB_DN", IO20: "net.USB_DP", IO21: "net.LED", IO10: "net.VSENSE",
        TXD0: "net.TXD0", RXD0: "net.RXD0",
      }} />
    <capacitor name="C1" maxDecouplingTraceLength={4} schSectionName="mcu" capacitance="100nF" footprint="0402" supplierPartNumbers={JLC("C1525")} {...sch("C1")}
      {...pcb("C1")} connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />
    <capacitor name="C2" maxDecouplingTraceLength={6} schSectionName="mcu" capacitance="22uF" footprint="0603" supplierPartNumbers={JLC("C59461")} {...sch("C2")}
      {...pcb("C2")} connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />
    {/* EN: 10k / 1uF power-on delay, RESET button */}
    <resistor name="R9" schSectionName="mcu" resistance="10k" footprint="0402" supplierPartNumbers={JLC("C25744")} {...sch("R9")}
      {...pcb("R9")} connections={{ pin1: "net.V3V3", pin2: "net.EN" }} />
    <capacitor name="C3" schSectionName="mcu" capacitance="1uF" footprint="0402" supplierPartNumbers={JLC("C52923")} {...sch("C3")}
      {...pcb("C3")} connections={{ pin1: "net.EN", pin2: "net.GND" }} />
    <pushbutton name="SW1" schSectionName="mcu" manufacturerPartNumber="TS-1088-AR02016" footprint="jlcpcb:C720477" supplierPartNumbers={JLC("C720477")} {...sch("SW1")}
      {...pcb("SW1")} connections={{ pin1: "net.GND", pin2: "net.EN" }} />
    {/* IO0: 10k pull-up, BOOT button */}
    <resistor name="R10" schSectionName="mcu" resistance="10k" footprint="0402" supplierPartNumbers={JLC("C25744")} {...sch("R10")}
      {...pcb("R10")} connections={{ pin1: "net.V3V3", pin2: "net.IO0" }} />
    <pushbutton name="SW2" schSectionName="mcu" manufacturerPartNumber="TS-1088-AR02016" footprint="jlcpcb:C720477" supplierPartNumbers={JLC("C720477")} {...sch("SW2")}
      {...pcb("SW2")} connections={{ pin1: "net.IO0", pin2: "net.GND" }} />
    {/* user LED, active low on IO21 */}
    <led name="LED1" schSectionName="mcu" color="red" manufacturerPartNumber="KT-0603R" footprint="jlcpcb:C2286" supplierPartNumbers={JLC("C2286")} pinLabels={{ pin1: "anode", pin2: "cathode" }} {...sch("LED1")}
      {...pcb("LED1")} connections={{ anode: "net.LED_A", cathode: "net.LED" }} />
    <resistor name="R11" schSectionName="mcu" resistance="1k" footprint="0402" supplierPartNumbers={JLC("C11702")} {...sch("R11")}
      {...pcb("R11")} connections={{ pin1: "net.LED_A", pin2: "net.V3V3" }} />
    {/* UART0 pads on the back, for flashing if USB ever fails (hold BOOT, press RST) */}
    <testpoint name="TP1" schSectionName="mcu" {...sch("TP1")} footprintVariant="pad" padShape="circle" padDiameter="1.2mm" layer="bottom" pcbX={10.1} pcbY={16.9} connections={{ pin1: "net.TXD0" }} />
    <testpoint name="TP2" schSectionName="mcu" {...sch("TP2")} footprintVariant="pad" padShape="circle" padDiameter="1.2mm" layer="bottom" pcbX={10.1} pcbY={12.3} connections={{ pin1: "net.RXD0" }} />
    <testpoint name="TP3" schSectionName="mcu" {...sch("TP3")} footprintVariant="pad" padShape="circle" padDiameter="1.2mm" layer="bottom" pcbX={10.1} pcbY={10.4} connections={{ pin1: "net.GND" }} />

    {/* ---- IMU at the centre. Pin 1 top-left => +X nose, +Y left, +Z up */}
    <chip name="U2" schSectionName="imu" manufacturerPartNumber="BMI323" footprint="jlcpcb:C5368700" supplierPartNumbers={JLC("C5368700")}
      pinLabels={pinNames(["SDO", "NC4", "NC3", "INT1", "VDDIO", "GNDIO", "GND", "VDD", "INT2", "NC2", "NC1", "CSB", "SCX", "SDX"])}
      schPinArrangement={{ leftSide: { direction: "top-to-bottom", pins: ["SDX", "SCX"] }, topSide: { direction: "left-to-right", pins: ["CSB", "VDDIO", "VDD"] },
        bottomSide: { direction: "left-to-right", pins: ["SDO", "GNDIO", "GND"] }, rightSide: { direction: "top-to-bottom", pins: ["INT1", "INT2", "NC1", "NC2", "NC3", "NC4"] } }}
      schWidth={1.6} noConnect={["INT1", "INT2", "NC1", "NC2", "NC3", "NC4"]} {...sch("U2")} {...pcb("U2")}
      connections={{
        VDD: "net.V3V3", VDDIO: "net.V3V3", GND: "net.GND", GNDIO: "net.GND",
        CSB: "net.V3V3", SDO: "net.GND", SDX: "net.SDA", SCX: "net.SCL",
      }} />
    <capacitor name="C4" maxDecouplingTraceLength={2.5} schSectionName="imu" capacitance="100nF" footprint="0402" supplierPartNumbers={JLC("C1525")} {...sch("C4")}
      {...pcb("C4")} connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />
    <capacitor name="C5" maxDecouplingTraceLength={3} schSectionName="imu" capacitance="100nF" footprint="0402" supplierPartNumbers={JLC("C1525")} {...sch("C5")}
      {...pcb("C5")} connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />
    <resistor name="R12" schSectionName="imu" resistance="4.7k" footprint="0402" supplierPartNumbers={JLC("C25900")} {...sch("R12")}
      {...pcb("R12")} connections={{ pin1: "net.V3V3", pin2: "net.SDA" }} />
    <resistor name="R13" schSectionName="imu" resistance="4.7k" footprint="0402" supplierPartNumbers={JLC("C25900")} {...sch("R13")}
      {...pcb("R13")} connections={{ pin1: "net.V3V3", pin2: "net.SCL" }} />

    {/* ---- power: 1S LiPo -> 3.3 V LDO; USB-C -> TP4054 charger -> battery */}
    <connector name="CN5" schSectionName="battery" manufacturerPartNumber="53398-0271" footprint="jlcpcb:C122410" supplierPartNumbers={JLC("C122410")}
      schPinArrangement={{ topSide: { direction: "left-to-right", pins: ["pin1"] }, bottomSide: { direction: "left-to-right", pins: ["pin2"] }, leftSide: { direction: "top-to-bottom", pins: ["pin3", "pin4"] } }}
      noConnect={["pin3", "pin4"]} {...sch("CN5")} {...pcb("CN5")} connections={{ pin1: "net.VBAT", pin2: "net.GND" }} />
    <capacitor name="C8" maxDecouplingTraceLength={5} schSectionName="battery" capacitance="100uF" footprint="1206" supplierPartNumbers={JLC("C15008")} {...sch("C8")}
      {...pcb("C8")} connections={{ pin1: "net.VBAT", pin2: "net.GND" }} />
    <capacitor name="C11" maxDecouplingTraceLength={10} schSectionName="battery" capacitance="10uF" footprint="0603" supplierPartNumbers={JLC("C19702")} {...sch("C11")}
      {...pcb("C11")} connections={{ pin1: "net.VBAT", pin2: "net.GND" }} />
    <capacitor name="C12" maxDecouplingTraceLength={5} schSectionName="battery" capacitance="10uF" footprint="0603" supplierPartNumbers={JLC("C19702")} {...sch("C12")}
      {...pcb("C12")} connections={{ pin1: "net.VBAT", pin2: "net.GND" }} />
    <chip name="U3" schSectionName="battery" manufacturerPartNumber="TLV75733PDBVR" footprint="jlcpcb:C485517" supplierPartNumbers={JLC("C485517")}
      pinLabels={pinNames(["IN", "GND", "EN", "NC", "OUT"])}
      schPinArrangement={{ leftSide: { direction: "top-to-bottom", pins: ["IN"] }, topSide: { direction: "left-to-right", pins: ["EN"] }, rightSide: { direction: "top-to-bottom", pins: ["OUT", "NC"] }, bottomSide: { direction: "left-to-right", pins: ["GND"] } }}
      schWidth={1.4} noConnect={["NC"]} {...sch("U3")} {...pcb("U3")}
      connections={{ IN: "net.VBAT", EN: "net.VBAT", GND: "net.GND", OUT: "net.V3V3" }} />
    <capacitor name="C6" maxDecouplingTraceLength={5} schSectionName="battery" capacitance="10uF" footprint="0603" supplierPartNumbers={JLC("C19702")} {...sch("C6")}
      {...pcb("C6")} connections={{ pin1: "net.VBAT", pin2: "net.GND" }} />
    <capacitor name="C7" maxDecouplingTraceLength={5} schSectionName="battery" capacitance="10uF" footprint="0603" supplierPartNumbers={JLC("C19702")} {...sch("C7")}
      {...pcb("C7")} connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />
    {/* battery voltage / 2 -> IO10 (ADC1_CH9) */}
    <resistor name="R19" schSectionName="mcu" resistance="100k" footprint="0402" supplierPartNumbers={JLC("C25741")} {...sch("R19")}
      {...pcb("R19")} connections={{ pin1: "net.VBAT", pin2: "net.VSENSE" }} />
    <resistor name="R20" schSectionName="mcu" resistance="100k" footprint="0402" supplierPartNumbers={JLC("C25741")} {...sch("R20")}
      {...pcb("R20")} connections={{ pin1: "net.VSENSE", pin2: "net.GND" }} />
    <capacitor name="C10" schSectionName="mcu" capacitance="100nF" footprint="0402" supplierPartNumbers={JLC("C1525")} {...sch("C10")}
      {...pcb("C10")} connections={{ pin1: "net.VSENSE", pin2: "net.GND" }} />

    <connector name="USB1" schSectionName="power" manufacturerPartNumber="TYPE-C 16PIN 2MD(073)" footprint="jlcpcb:C2765186" supplierPartNumbers={JLC("C2765186")}
      pinLabels={{ pin15: "GND1", pin16: "VBUS1", pin17: "SBU2", pin18: "CC1", pin19: "Dn2",
        pin20: "Dp1", pin21: "Dn1", pin22: "Dp2", pin23: "SBU1", pin24: "CC2", pin25: "VBUS2", pin26: "GND2" }}
      schPinArrangement={{ rightSide: { direction: "top-to-bottom", pins: ["VBUS2", "VBUS1", "Dn1", "Dn2", "Dp1", "Dp2", "SBU1", "SBU2"] },
        bottomSide: { direction: "left-to-right", pins: ["CC1", "CC2", "GND1", "GND2", 13, 14] } }}
      schPinStyle={{ Dn1: { topMargin: 0.2 }, Dp1: { topMargin: 0.2 }, SBU1: { topMargin: 0.2 }, SBU2: { bottomMargin: 0.4 }, CC2: { leftMargin: 0.8 }, GND1: { leftMargin: 0.4 } }}
      schWidth={2.4} noConnect={["SBU1", "SBU2"]} {...sch("USB1")} {...pcb("USB1")}
      connections={{
        VBUS1: "net.VBUS", VBUS2: "net.VBUS", GND1: "net.GND", GND2: "net.GND", pin13: "net.GND", pin14: "net.GND",
        CC1: "net.CC1", CC2: "net.CC2",
        Dp1: "net.USB_DP", Dp2: "net.USB_DP", Dn1: "net.USB_DN", Dn2: "net.USB_DN",
      }} />
    <resistor name="R14" schSectionName="power" resistance="5.1k" footprint="0402" supplierPartNumbers={JLC("C25905")} {...sch("R14")}
      {...pcb("R14")} connections={{ pin1: "net.CC1", pin2: "net.GND" }} />
    <resistor name="R15" schSectionName="power" resistance="5.1k" footprint="0402" supplierPartNumbers={JLC("C25905")} {...sch("R15")}
      {...pcb("R15")} connections={{ pin1: "net.CC2", pin2: "net.GND" }} />
    <chip name="U4" schSectionName="power" manufacturerPartNumber="TP4054-42-SOT25R" footprint="jlcpcb:C32574" supplierPartNumbers={JLC("C32574")}
      pinLabels={pinNames(["CHRG", "GND", "BAT", "VCC", "PROG"])}
      schPinArrangement={{ leftSide: { direction: "top-to-bottom", pins: ["VCC"] }, topSide: { direction: "left-to-right", pins: ["BAT"] }, rightSide: { direction: "top-to-bottom", pins: ["CHRG"] }, bottomSide: { direction: "left-to-right", pins: ["GND", "PROG"] } }}
      schPinStyle={{ PROG: { leftMargin: 0.4 } }} schWidth={1.6} schHeight={1.2} {...sch("U4")} {...pcb("U4")}
      connections={{ VCC: "net.VBUS", BAT: "net.VBAT", GND: "net.GND", PROG: "net.PROG", CHRG: "net.CHRG" }} />
    {/* VBUS bypass with a damping resistor against USB hot-plug overshoot (TP4054 datasheet) */}
    <capacitor name="C9" schSectionName="power" capacitance="1uF" footprint="0402" supplierPartNumbers={JLC("C52923")} {...sch("C9")}
      {...pcb("C9")} connections={{ pin1: "net.VBUS_C", pin2: "net.GND" }} />
    <resistor name="R18" schSectionName="power" resistance="2.2" footprint="0603" supplierPartNumbers={JLC("C22939")} {...sch("R18")}
      {...pcb("R18")} connections={{ pin1: "net.VBUS", pin2: "net.VBUS_C" }} />
    <resistor name="R16" schSectionName="power" resistance="3.3k" footprint="0402" supplierPartNumbers={JLC("C25890")} {...sch("R16")}
      {...pcb("R16")} connections={{ pin1: "net.PROG", pin2: "net.GND" }} />
    <led name="LED2" schSectionName="power" color="red" manufacturerPartNumber="KT-0603R" footprint="jlcpcb:C2286" supplierPartNumbers={JLC("C2286")} pinLabels={{ pin1: "anode", pin2: "cathode" }} {...sch("LED2")}
      {...pcb("LED2")} connections={{ anode: "net.CHG_A", cathode: "net.CHRG" }} />
    <resistor name="R17" schSectionName="power" resistance="1k" footprint="0402" supplierPartNumbers={JLC("C11702")} {...sch("R17")}
      {...pcb("R17")} connections={{ pin1: "net.CHG_A", pin2: "net.VBUS" }} />

    {MOTORS.map((m) => <Motor key={m.i} {...m} />)}
    {LABELS.map((l) => <Label key={l.join()} at={l} />)}
    <schematictext text="RESET" schX={-5.95} schY={1.8} fontSize={0.16} anchor="right" color="#555" />
    <schematictext text="BOOT" schX={-4.85} schY={-1.1} fontSize={0.16} anchor="right" color="#555" />
    {copper && <Copper />}
    <Silkscreen />
  </board>
)
