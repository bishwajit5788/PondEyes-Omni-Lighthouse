# PondEyes Omni-Lighthouse — Sensor Body

## Intended model dimensions

Source: Blender dimension screenshot (`../sensor_tot_dimensions_blender.png`)

- X: **62.70 mm**
- Y: **62.11 mm**
- Z: **125.00 mm**
- Intended printer: **Bambu Lab A1**
- Nozzle: **0.4 mm**

The committed `sensor_tot.stl` has been corrected so that its actual geometric bounding box matches these values. Do not apply an extra scale in the slicer.

## Sensor hardware

- 6 × Hi-Link HLK-LD2450 24 GHz FMCW radar modules
- Nominal module envelope: **44 × 15 mm** per module
- Sensor directions: **0°, 60°, 120°, 180°, 240°, 300°**
- Opposing pairs: **S1/S4, S2/S5, S3/S6**
- Design allowance previously specified: 0.2 mm per side (44.4 × 15.4 mm fit envelope)

## Internal electronics to accommodate

- 2 × Teensy 4.1 — approx. **61 × 17.8 mm** each
- 1 × Seeed Studio XIAO ESP32-S3 — **21 × 17.8 mm**

These are board dimensions only. Account for USB connectors, pin headers, cable bend radius, battery pack and service access when checking fit.

## Print checklist

1. Import `sensor_tot.stl` into Bambu Studio (or equivalent).
2. Confirm the object reports approximately 62.7 × 62.1 × 125 mm.
3. Orient for best surface quality on the radar faces and minimal support on critical interfaces.
4. Verify wall thickness and radar cradle openings against a physical HLK-LD2450 module.
5. Print a short test section (one cradle + adjacent wall) before the full part.
6. After printing, dry-fit the six radar modules and check internal board clearance.

## Files

- Print file: `../sensor_tot.stl`
- Dimension evidence: `../sensor_tot_dimensions_blender.png`
- Related full enclosure/hardware spec: `../PondEyes_Enclosure_Spec.md`
