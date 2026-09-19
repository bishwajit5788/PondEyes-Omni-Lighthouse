# PondEyes Omni-Lighthouse — 3D Models

This directory contains the mechanical CAD exports and dimensional reference for the PondEyes Omni-Lighthouse enclosure.

## 3D model files

### Blender source files

The `.blend` files are the editable Blender source models used to create the corresponding STL exports.

- `bottom_tot.blend` — editable Blender source for the bottom service-cap assembly
- `sensor_tot.blend` — editable Blender source for the six-sensor external mounting assembly
- `top_tot.blend` — editable Blender source for the top cap assembly

### STL exports

The STL files are the 3D-printable mesh exports corresponding to the Blender source files.

- `top_tot.stl` — top cap export
- `sensor_tot.stl` — six-sensor external mounting export
- `bottom_tot.stl` — bottom service-cap export

### Documentation

- `PondEyes_Enclosure_Spec.md` — hardware and enclosure dimensions

## Source-to-STL mapping

| Blender source | STL export | Purpose |
|---|---|---|
| `top_tot.blend` | `top_tot.stl` | Top cap |
| `sensor_tot.blend` | `sensor_tot.stl` | Six-radar sensor mounting body |
| `bottom_tot.blend` | `bottom_tot.stl` | Bottom service cap |

## Mechanical target

**100 mm outer diameter × 150 mm body height × 3 mm nominal wall**

Six HLK-LD2450 radar modules are arranged externally at 60° intervals. Two Teensy 4.1 boards and one XIAO ESP32-S3 are intended to fit inside the enclosure.

## CAD workflow

The recommended workflow is:

1. Edit the corresponding `.blend` source in Blender.
2. Verify dimensions, clearances, manifold geometry, and assembly interfaces.
3. Export the updated component as STL for slicing/3D printing.
4. Keep the Blender source and STL export together so the printable mesh remains traceable to its editable CAD source.

> **Note:** The Blender files are the editable design sources; the STL files are mesh exports intended for fabrication and should not be treated as the primary editable CAD source.
