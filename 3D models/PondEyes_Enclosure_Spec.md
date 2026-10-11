# PondEyes Omni-Lighthouse — 3D Model & Hardware Specification

## Sensor-body model dimensions (latest supplied Blender screenshot)

| Axis | Dimension |
|---|---:|
| X | 62.70 mm |
| Y | 62.11 mm |
| Z | 125.00 mm |

These are the dimensions shown in the supplied Blender screenshot. They should be treated as the intended sensor-body bounding dimensions, not as proof that the uploaded STL is correctly scaled.

### STL scale validation warning

The supplied binary STL contains 2,360 triangles. Reading its vertex coordinates gives raw bounds of approximately 63,626.59 × 62,500 × 120,000 coordinate units. STL itself does not define units, and these bounds do not match the screenshot dimensions when interpreted as millimetres. Re-export the mesh from Blender with the correct scale/transforms and confirm the resulting dimensions in a slicer before printing. Do not assume the current raw STL is print-ready.

## Enclosure reference (previous specification; reconcile before assembly)

- Main body: cylindrical handheld enclosure
- Previously documented outer diameter: 100 mm
- Previously documented nominal wall thickness: 3 mm
- Previously documented inner diameter: 94 mm
- Previously documented body height: 150 mm
- Printer target: Bambu Lab A1
- Nozzle: 0.4 mm

The sensor-body screenshot dimensions differ from the previous overall enclosure target. Do not silently substitute one for the other; verify which dimensions describe the sensor component versus the complete enclosure.

## Radar sensors

### Hi-Link HLK-LD2450
- Quantity: 6
- Module dimensions: 44 mm × 15 mm
- Mounting: external, low-profile cradles
- Sensor angles: 0°, 60°, 120°, 180°, 240°, 300°
- Opposing pairs: S1/S4, S2/S5, S3/S6
- Nominal fit envelope with 0.2 mm clearance per side: 44.4 mm × 15.4 mm

## Development boards

### Teensy 4.1
- Quantity: 2
- Approximate board dimensions: 61 mm × 17.8 mm
- Mounted internally
- Preserve access to connectors and provide wiring clearance.

### Seeed Studio XIAO ESP32-S3
- Quantity: 1
- Board dimensions: 21 mm × 17.8 mm
- Mounted internally
- Associated with the top/antenna assembly.

## Caps

### Top cap
- Removable
- Hollow internal volume
- Centered antenna pass-through/support
- Antenna hole target: Ø6.5 mm (verify against the actual bulkhead fitting)

### Bottom service cap
- Removable
- Closed bottom surface
- Hollow interior
- Intended to mate with the enclosure's threaded interface
- Thread fit must be validated physically for the selected FDM material and print orientation.

## Manufacturing checklist

- Confirm model units and applied object scale in Blender.
- Export the sensor body as STL using the correct unit scale.
- Re-import the exported STL and verify 62.70 × 62.11 × 125.00 mm if those remain the approved target dimensions.
- Inspect for non-manifold edges, inverted normals, disconnected shells, and thin walls.
- Check sensor cradle dimensions and board clearances against the physical hardware.
- Use the Bambu Lab A1 / 0.4 mm nozzle profile and verify orientation/supports in the slicer.
- Print a small fit-test section before committing to the full print.
- RF coverage and radar-to-radar interference are not validated by mechanical CAD alone.
