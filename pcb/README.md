# quad pcb

Flight controller and frame on a single 2‑layer PCB, written in
[tscircuit](https://tscircuit.com). `index.tsx` is the whole design: parts,
schematic, placement and every copper track (drawn by hand, no autorouter).
The four arms are the frame; the electronics sit in the 44.8 mm core. Each
motor connector sits in its corner with the cable opening facing down the arm.

![board](img/board-3d.png)

Schematic: [img/schematic.svg](img/schematic.svg) · KiCad files (to look
around, measure, run DRC/ERC): [kicad/](kicad/).

## Order it (no install needed)

The ready‑made files are in [fab/](fab/). On jlcpcb.com:

1. *Order now* → upload `fab/gerbers.zip`. 2 layers, 1.6 mm, FR‑4, any
   colour. Via covering: *tented* (the default). Surface finish: lead‑free
   HASL works; ENIG gives flatter pads for the IMU's 0.5 mm LGA and is worth
   it if the budget allows. *Remove order number*: choose *Specify a
   location* – the back has a `JLCJLCJLCJLC` spot for it.
2. Tick *PCB Assembly*, top side. Upload `fab/bom.csv` and `fab/cpl.csv`.
   Every row has its JLCPCB part number; all parts are in stock, the
   passives are *basic* parts.
3. The ESP32 module overhangs the nose edge (its antenna) and the USB‑C
   socket the tail edge. If JLCPCB asks for edge rails, let them add them on
   the left/right (arm) sides, not across the nose or tail.
4. In the placement preview check each part once: antenna toward the nose,
   USB‑C opening at the tail edge, diode bands toward the VBAT rails (outer
   side of D1–D4), LEDs, the IMU's pin‑1 dot (top‑left corner), U3/U4 pin 1.
   `cpl.csv` already places every part at JLCPCB's own footprint origin.

## What is on it

| Block | Part | JLCPCB # |
|---|---|---|
| MCU | ESP32‑S3‑WROOM‑1‑N16R8, antenna over the nose edge | C2913202 |
| IMU | BMI323, I2C at 0x68, +X = nose, +Y = left, +Z = up | C5368700 |
| Motor drive ×4 | AO3400A low‑side, 47 Ω gate, 10 k pull‑down, B5819W flyback | C20917, C8598 |
| Motor connectors ×4 | HC‑1.25‑2PWT (1.25 mm, 2 pin), "+" marked on the silkscreen | C2845379 |
| Battery | Molex PicoBlade 53398‑0271, "+"/"−" marked, 100 µF bulk | C122410, C15008 |
| 3.3 V | TLV75733 LDO (1 A) from the battery, always on | C485517 |
| Charger | TP4054, 260 mA (R16 = 3.3 k, datasheet formula 1), CHG LED | C32574 |
| USB | USB‑C, native S3 USB (flashing + console), 5.1 k CC pull‑downs | C2765186 |
| Buttons | RST (EN, left edge) and BOOT (IO0, right edge), top‑actuated | C720477 |
| Battery sense | VBAT / 2 on IO10 (100 k / 100 k, 100 nF) | |
| UART pads | TX, RX, GND on the back (TP1–TP3) | |

Pin map (also on the schematic, and in `firmware/main/main.c`):

| GPIO | Use |
|---|---|
| IO1, IO2, IO16, IO4 | PWM0–3 → M0 front right, M1 back right, M2 back left, M3 front left (high = on) |
| IO11, IO12 | I2C SDA, SCL (BMI323) |
| IO10 | battery voltage / 2 (ADC1 channel 9, use 12 dB attenuation) |
| IO21 | LED1, active low |
| IO19, IO20 | USB D−, D+ |
| IO0 / EN | BOOT / RESET buttons |
| TXD0, RXD0 | UART0 pads TP1, TP2 |

Motors: FR and BL spin CCW, BR and FL spin CW (printed on each arm). The
connector pin marked "+" is VBAT; the other pin is switched to ground. The
pinout matches the May 2026 board, so its motor leads plug in the same way.
A brushed motor's direction follows its polarity – if one spins the wrong
way, swap its two wires.

Copper: bottom layer is a ground pour; VBAT runs 1 mm wide on top plus a
1.5 mm bus on the bottom from the battery connector; motor nets 0.6–0.8 mm;
vias 0.3/0.6 mm.

## Change the design

Needs Node ≥ 20; `npm install` brings everything else (including `bun`) and
applies the small tscircuit fixes in `scripts/patch-tscircuit.mjs`.

```bash
cd pcb
npm install
npm run build   # ~10 s: dist/circuit.json (board) + dist/sch.circuit.json (schematic), then DRC
npm run fab     # fab/gerbers.zip, fab/bom.csv, fab/cpl.csv
npm run kicad   # kicad/quad.kicad_pcb, .kicad_sch, .kicad_pro
npm run img     # img/pcb.png, img/schematic.svg
npm run place   # placement only (no copper) -> img/place.png
npm run dev     # live PCB / schematic / 3D view at http://localhost:3020
```

* `build.tsx` replaces `tsci build` (whose built‑in checks can hang). It
  renders the board twice: once with the copper for the PCB, once without it
  for the schematic, since tscircuit would otherwise draw the hand‑made PCB
  tracks into the schematic.
* `scripts/silk.ts` drops unprintable footprint texts and clips silkscreen
  lines 0.15 mm off pads, holes and the board edge (what JLCPCB would clip).
* `scripts/jlc.ts` writes the BOM and a CPL at JLCPCB's footprint origins
  (tscircuit's own pick‑and‑place uses pad centres, which is off for the
  module, USB‑C and connectors).
* `scripts/kicad.ts` touches up the KiCad export so KiCad reads it right:
  real power symbols, correct diode/connector pin numbers, net ties at
  junctions, no‑connect flags, JLCPCB design rules, text sizes as printed.
  The ground pour comes without fill: press **B** in KiCad to fill it.
* `footprints/` caches the JLCPCB footprints (some edited: USB‑C slots,
  LDO pad toes, a connector courtyard), so builds work offline.
* `drc.py` is an independent check (`pip install shapely`): 0.2 mm copper
  clearance, 0.3 mm to the edge, via sizes, and that every net is one piece
  of copper: `python3 drc.py`.

## Checked before release

* tscircuit DRC (routing, placement, netlist, JLCPCB rules) and `drc.py`: clean.
* KiCad 9 DRC on the exported board with JLCPCB limits, zones refilled: clean;
  KiCad ERC on the schematic: clean.
* The netlist KiCad computes from the drawn schematic, and the connectivity
  extracted from the gerber copper + drill files, both equal the design
  netlist pin for pin (no opens, no shorts).
* Silkscreen in the gerbers: ≥ 0.15 mm from every pad and hole, text ≥ 0.8 mm
  high with ≥ 0.16 mm strokes.
* Every part checked against its datasheet (pinout vs footprint vs netlist,
  application circuit, ratings), each finding re‑checked independently.

## Limits worth knowing

* **Low battery:** the LDO has no headroom on 1S, so 3.3 V follows VBAT
  below about 3.4 V. Land (firmware cut‑off on IO10) at about 3.3 V loaded.
* **Battery connector (the one real compromise):** PicoBlade contacts are
  rated 1 A, but CN5 carries all four motors – 2–6 A in flight, more at
  spin‑up. It was kept so your existing packs and the May 2026 board's leads
  fit. Expect some extra sag and a warm connector; for hard flying, solder a
  22–24 AWG pigtail with a BT2.0/XT30 plug instead. The 1.25 mm motor
  connectors (also 1 A) are within rating up to about 1 A per motor.
* No reverse‑polarity protection on the battery and no ESD protection on
  USB: check the "+" before plugging a battery in.
* Keep a battery plugged in while working over USB: the charger alone is not
  a stable supply. While the ESP32 runs, the charger never terminates (it
  holds the pack at 4.2 V) and the CHG LED stays on.
* Firmware to‑dos the review turned up: a low‑battery cut‑off from IO10
  (ADC1 channel 9, 12 dB, VBAT = 2 × reading), arming only at zero throttle
  plus a stall/crash cut‑off, staggered PWM phases (LEDC `hpoint` 0/64/128/192)
  to cut the peak battery current, and a 16 MB flash size in `sdkconfig`
  (it builds for 2 MB today, which also works). Note that `app_main` spins
  every motor briefly at boot: take the props off or remove `motors_sweep`.
* The buttons are pressed from the top. The antenna sits at the nose: keep
  metal and the battery away from it and test BLE range on the first board.
