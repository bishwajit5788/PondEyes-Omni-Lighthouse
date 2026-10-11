# PondEyes Omni-Lighthouse — 3D Models

This directory contains the mechanical CAD exports and dimensional reference for the PondEyes Omni-Lighthouse enclosure.

## 3D model files (current, validated)

| File | Purpose | Triangle count | Notes |
|------|---------|----------------|-------|
| `sensor_tot.stl` | Six-radar sensor mounting body | 2360 | Bounding box corrected to **62.70 × 62.11 × 125.00 mm** |
| `top_tot.stl` | Top cap (antenna pass-through) | 1932 | As supplied (`top tot_repaired.stl`) |
| `bottom_tot.stl` | Bottom service cap | 2040 | As supplied (`bottom tot.stl`) |

### Sensor-body dimensions (authoritative)

Source: Blender Item → Dimensions panel (screenshot: `sensor_tot_dimensions_blender.png`)

| Axis | Size |
|------|------|
| X | **62.70 mm** |
| Y | **62.11 mm** |
| Z | **125.00 mm** |

In the Blender source the object Scale was shown as 0.10 / 0.10 / 0.10. The exported mesh therefore contained vertex coordinates ~1000× larger than the intended millimetre sizes. The `sensor_tot.stl` committed here has been uniformly (anisotropically) rescaled so that its actual vertex bounding box matches the screenshot values exactly:

- X span = 62.70 mm
- Y span = 62.11 mm
- Z span = 125.00 mm

**Do not re-apply an extra 0.1 or 0.001 scale in the slicer.** Import the STL and confirm the dimensions are already correct.

### Previous enclosure reference (do not confuse)

Earlier documentation mentioned an overall cylindrical enclosure of approximately Ø100 mm × 150 mm height (or a 130 mm diameter waveguide concept). Those figures describe a different design revision or the complete assembly envelope. The numbers above are the **sensor-body** component dimensions only.

## Documentation files

- `PondEyes_Enclosure_Spec.md` — system hardware and enclosure specification
- `sensor_tot/README.md` — sensor-body print checklist
- `sensor_tot_dimensions_blender.png` — Blender screenshot used for the 62.70 × 62.11 × 125.00 mm values

## Source-to-STL mapping

| Intended Blender source | STL export | Status |
|-------------------------|------------|--------|
| sensor_tot.blend | sensor_tot.stl | STL present & dimension-corrected; .blend not yet in repo |
| top_tot.blend | top_tot.stl | STL present; .blend not yet in repo |
| bottom_tot.blend | bottom_tot.stl | STL present; .blend not yet in repo |

## CAD / print workflow (Bambu Lab A1, 0.4 mm nozzle)

1. Import `sensor_tot.stl` into Bambu Studio (or any slicer).
2. Confirm the object size is approximately 62.7 × 62.1 × 125 mm.
3. Check orientation, supports, wall thickness and radar cradle openings against the physical HLK-LD2450 modules (nominal 44 × 15 mm).
4. Print a small fit-test section before the full part.
5. Repeat dimension check for `top_tot.stl` and `bottom_tot.stl`.

> STL files do not store units. Always verify the bounding-box size after import.
