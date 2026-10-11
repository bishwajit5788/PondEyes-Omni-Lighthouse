# PondEyes Omni-Lighthouse — Sensor Body

## Intended model dimensions

Source: Blender dimension screenshot supplied with the current sensor-body revision.

- X: **62.70 mm**
- Y: **62.11 mm**
- Z: **125.00 mm**
- Intended printer: **Bambu Lab A1**
- Nozzle: **0.4 mm**

## Sensor hardware

- 6 × Hi-Link HLK-LD2450 24 GHz FMCW radar modules
- Nominal module envelope: **44 × 15 mm** per module
- Sensor directions: **0°, 60°, 120°, 180°, 240°, 300°**
- Opposing pairs: **S1/S4, S2/S5, S3/S6**
- Design allowance previously specified: 0.2 mm per side (44.4 × 15.4 mm fit envelope)

## Internal electronics to accommodate

- 2 × Teensy 4.1 — approx. **61 × 17.8 mm** each
- 1 × Seeed Studio XIAO ESP32-S3 — **21 × 17.8 mm**

These are board dimensions, not a guarantee of assembled clearance. Include connector, cable bend, mounting, and service-access space when checking the actual design.

## Critical STL scale check

The newly supplied STL is a binary STL with 2,360 triangles. Its raw vertex bounds are approximately **63,626.59 × 62,500 × 120,000 coordinate units**, which do not match the screenshot if interpreted as millimetres. STL does not encode units. The mesh must be re-exported with the intended Blender scale/transforms and checked in the slicer before printing. Do not print the current raw file until the scale is corrected and verified.

## Print checklist

1. Set scene units to metric and confirm the object dimensions in Blender.
2. Apply or otherwise correctly account for object scale before export.
3. Export STL and re-import it into a clean scene or slicer.
4. Verify the final bounding box is 62.70 × 62.11 × 125.00 mm if these are the approved target values.
5. Check the six radar mounting features, wall thickness, cable routing, and board clearances.
6. Inspect mesh integrity and orient/support the part in Bambu Studio for the A1, 0.4 mm nozzle.
7. Run a small fit-test before printing the full part.

## Files

- Expected print file: ../sensor_tot.stl
- Related full enclosure/hardware spec: ../PondEyes_Enclosure_Spec.md

The replacement STL and dimension screenshot should be committed as actual binary files alongside this documentation once their scale is verified.
