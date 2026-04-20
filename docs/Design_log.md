# Design Log

Reverse-chronological record of design decisions, measured physical
constants, calibration outcomes, and any values that future work will
need to reference.  Each entry should include **what** was recorded,
**why**, and **how it was derived** so a later reader can re-verify.

This file is for *design facts* (numbers, positions, chosen parameters,
rejected alternatives).  For project state / bug history / next-steps,
use `Project_log.md`; for step-by-step roadmap, use `Development_plan.md`.

---

## 2026-04-19 — Radial gradient map centres (A3 sheets)

Extracted the centre coordinates of every radial-gradient test map in
`maps/`.  Coordinates are in **millimetres** on an **A3 portrait page
(297.04 × 420.16 mm)**, reported from two origins so the robot's
starting corner does not matter:

- `X_BL / Y_BL` — distance from the **bottom-left** corner.
- `X_TR / Y_TR` — distance from the **top-right** corner
  (`= page_width − X_BL`, `page_height − Y_BL`).

The centre-finder FSM's end-of-run pose report can be cross-checked
against these values to evaluate position accuracy.

| Map file | X_BL | Y_BL | X_TR | Y_TR | Radius |
|---|---:|---:|---:|---:|---:|
| `correctedandlinear-radial-a3-200rad.pdf` — upper radial | 148.68 | 317.05 | 148.36 | 103.11 | 100.03 |
| `correctedandlinear-radial-a3-200rad.pdf` — lower radial | 149.74 | 108.73 | 147.30 | 311.43 | 100.03 |
| `correctedgrad-radial-a3-400rad.pdf` | 68.56 | 209.81 | 228.48 | 210.35 | 200.05 |
| `correctedgrad-radial-a3-600rad.pdf` | 146.96 | 354.80 | 150.08 | 65.36 | 300.08 |
| `linear-radial-a3-400rad.pdf` | 79.71 | 207.60 | 217.33 | 212.56 | 200.05 |
| `lineargrad-radial-a3-600rad.pdf` | 156.46 | 334.10 | 140.58 | 86.06 | 300.08 |

All values in mm.

**Filename convention:** the `NNNrad` suffix is the gradient **diameter**
in mm (e.g. `600rad` → 300 mm radius).  `corrected*` maps use the
gamma-corrected transfer to produce a perceptually-linear darkness
profile; `linear*` maps use a raw linear gradient.

**Derivation:** for each PDF, read the `/Coords` array of the Type-3
(radial) Shading object in its xref, then compose the local `cm`
transform at the `sh` paint site with the page-level Y-flip
`1 0 0 -1 0 1191 cm`.  The result is in PDF default user space, which
is already bottom-left-origin in points; convert to mm with ×25.4/72.
Cross-checked by rendering each page at 1 px/pt and confirming the
darkest pixel lands within ~1 mm of the predicted centre (the residual
is sampling noise, not a systematic error).

**How to apply:** when logging an end-of-run `captured_x / captured_y`,
subtract these centres from the robot's *starting pose* (if the
starting position relative to the page was recorded) to get a signed
position error.  For the two-radial sheet, both centres sit on the
sheet's vertical midline (X ≈ 149 mm), separated by 208 mm in Y —
useful for back-to-back runs without swapping paper.
