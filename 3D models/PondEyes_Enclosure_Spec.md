# PondEyes Omni-Lighthouse — 3D Model & Hardware Specification

## Current enclosure target

- Main body: cylindrical handheld enclosure
- Main body outer diameter: 100 mm
- Outer radius: 50 mm
- Nominal wall thickness: 3 mm
- Inner diameter: 94 mm
- Inner radius: 47 mm
- Main body height: 150 mm
- Printer target: Bambu Lab A1
- Nozzle: 0.4 mm

## Radar sensors

### Hi-Link HLK-LD2450

- Quantity: 6
- Module dimensions: 44 mm × 15 mm
- Mounting: external, low-profile cradles
- Sensor angles: 0°, 60°, 120°, 180°, 240°, 300°
- Opposing pairs: S1/S4, S2/S5, S3/S6
- Nominal sensor fit envelope with 0.2 mm per-side clearance: 44.4 mm × 15.4 mm

## Development boards

### Teensy 4.1

- Quantity: 2
- Approximate board dimensions: 61 mm × 17.8 mm
- Mounted internally
- Internal holder/rail system should preserve service access and wiring clearance

### Seeed Studio XIAO ESP32-S3

- Quantity: 1
- Board dimensions: 21 mm × 17.8 mm
- Mounted internally
- Associated with the top/antenna assembly

## Caps

### Top cap

- Removable
- Hollow internal volume
- Centered antenna pass-through/support
- Antenna hole target: Ø6.5 mm

### Bottom service cap

- Removable
- Closed bottom surface
- Hollow interior
- Female helical thread on the inside of the cap
- Mates with the enclosure's external male helical thread
- Intended to lock/unlock by rotation
- Thread fit must be validated physically for the selected FDM material and print orientation

## CAD / manufacturing notes

The STL exports are the current supplied CAD revision. Before fabrication, verify the STL dimensions against the master dimensions above and perform a test print of the threaded interface. RF performance and radar-to-radar interference are not proven by mechanical CAD alone.
