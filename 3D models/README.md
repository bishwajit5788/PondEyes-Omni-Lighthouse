# PondEyes Omni-Lighthouse — 3D Models

This directory contains the mechanical CAD exports and dimensional reference for the PondEyes Omni-Lighthouse enclosure.

## 3D model files

### Blender source files
The .blend files are the editable Blender source models used to create corresponding STL exports.

- bottom_tot.blend — bottom service-cap assembly
- sensor_tot.blend — six-sensor mounting body
- top_tot.blend — top cap assembly

### STL exports
- top_tot.stl — top cap export
- sensor_tot.stl — six-radar sensor mounting body
- bottom_tot.stl — bottom service cap

**Sensor model update:** the newly supplied STL and Blender dimension screenshot have been inspected. The screenshot reports **62.70 × 62.11 × 125.00 mm (X × Y × Z)**, but the uploaded STL's raw vertex bounds are approximately **63,626.59 × 62,500 × 120,000 coordinate units**. STL files do not store a unit, and those raw bounds do not directly match the screenshot. The mesh must be exported with the correct applied scale and checked in a slicer before it is treated as a print-ready replacement. Do not print the raw uploaded STL at its current apparent scale.

## Documentation
- PondEyes_Enclosure_Spec.md — system hardware and enclosure specification
- sensor_tot/README.md — sensor-body model dimensions, hardware fit, and print checklist

## Source-to-STL mapping

| Blender source | STL export | Purpose |
|---|---|---|
| top_tot.blend | top_tot.stl | Top cap |
| sensor_tot.blend | sensor_tot.stl | Six-radar sensor mounting body |
| bottom_tot.blend | bottom_tot.stl | Bottom service cap |

## Current enclosure reference

The previously documented enclosure target was **100 mm outer diameter × 150 mm body height × 3 mm nominal wall**. The newer sensor-body screenshot reports a different bounding size (**62.70 × 62.11 × 125.00 mm**). Treat these as different references until the complete assembly and master dimensions are reconciled.

## CAD and print workflow

1. Open the authoritative Blender source and apply/verify object scale.
2. Confirm dimensions in millimetres in Blender's Item/Dimensions panel.
3. Export STL with the intended unit scale and object transforms.
4. Re-import the STL into a fresh Blender scene or slicer and verify the dimensions.
5. Check mesh integrity, clearances, orientation, supports, and fit with a small test print before printing the full assembly.

> Blender .blend files are editable design sources. STL files are mesh exports and do not encode a unit system. Always verify scale after export.
