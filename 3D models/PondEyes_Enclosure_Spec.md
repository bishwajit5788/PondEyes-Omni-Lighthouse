# PondEyes Omni-Lighthouse — 3D Model & Hardware Specification

## Sensor-body model dimensions (authoritative)

Source: Blender dimension screenshot (`sensor_tot_dimensions_blender.png`)

| Axis | Dimension |
|------|----------:|
| X    | 62.70 mm  |
| Y    | 62.11 mm  |
| Z    | 125.00 mm |

These are the dimensions shown in the Blender Item panel. The committed `sensor_tot.stl` has been rescaled so that its vertex bounding box matches these values exactly.

### STL scale history

- Original exported mesh had raw spans ≈ 63 627 × 62 500 × 120 000 coordinate units (typical when Blender object Scale = 0.10 is left un-applied).
- Corrected `sensor_tot.stl` now measures 62.70 × 62.11 × 125.00 mm.
- Do **not** apply an additional scale factor in the slicer.

## Enclosure reference (previous specification — reconcile before final assembly)

- Main body: cylindrical handheld enclosure
- Previously documented outer diameter: 100 mm (or 130 mm waveguide concept in README)
- Previously documented nominal wall thickness: 3 mm
- Previously documented body height: 150 mm
- Printer target: Bambu Lab A1
- Nozzle: 0.4 mm

The sensor-body dimensions above describe the six-radar mounting component. They are not automatically identical to the overall enclosure envelope.

## Radar sensors

### Hi-Link HLK-LD2450
- Quantity: 6
- Module dimensions: 44 mm × 15 mm
- Mounting: external / low-profile cradles on the sensor body
- Sensor angles: 0°, 60°, 120°, 180°, 240°, 300°
- Opposing pairs: S1/S4, S2/S5, S3/S6
- Nominal fit envelope with 0.2 mm clearance per side: 44.4 mm × 15.4 mm

## Development boards (internal)

### Teensy 4.1
- Quantity: 2
- Approximate board dimensions: 61 mm × 17.8 mm
- Preserve access to USB, pins and wiring.

### Seeed Studio XIAO ESP32-S3
- Quantity: 1
- Board dimensions: 21 mm × 17.8 mm

## Caps

### Top cap (`top_tot.stl`)
- Removable
- Hollow internal volume
- Centered antenna pass-through / bulkhead interface
- Antenna hole target: verify against actual connector (previously discussed ≈ Ø6.5 mm)

### Bottom service cap (`bottom_tot.stl`)
- Removable
- Closed bottom surface
- Intended to mate with any threaded interface on the body
- Thread fit (if present) must be validated with a physical test print

## Manufacturing checklist

- [x] Confirm model units and applied object scale in Blender (screenshot archived).
- [x] Export / correct STL so bounding box = 62.70 × 62.11 × 125.00 mm.
- [ ] Re-import the committed STL into a clean slicer scene and re-verify dimensions.
- [ ] Inspect mesh integrity (non-manifold edges, inverted normals, thin walls).
- [ ] Check sensor cradle dimensions against physical HLK-LD2450 modules.
- [ ] Check internal volume against two Teensy 4.1 + XIAO + wiring + battery.
- [ ] Use Bambu Lab A1 / 0.4 mm nozzle profile; decide orientation and supports.
- [ ] Print a small fit-test section before the full sensor body.
- [ ] RF coverage and radar-to-radar interference are **not** validated by CAD alone.
