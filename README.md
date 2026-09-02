# PondEyes Omni-Lighthouse: 360-Degree Multi-Target Radar Platform

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![Platform](https://img.shields.io/badge/platform-Teensy%204.1%20%7C%20ESP32--S3-blue.svg)]()
[![Sensor](https://img.shields.io/badge/radar-Hi--Link%20HLK--LD2450%20(24GHz)-orange.svg)]()
[![Protocol](https://img.shields.io/badge/protocol-SPI%20%2B%20MQTT%20(10Hz)-purple.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()

**PondEyes Omni-Lighthouse** is a high-precision, 360-degree multi-target tracking radar platform engineered for panoramic spatial awareness. It integrates six **Hi-Link HLK-LD2450 24GHz FMCW mmWave radar sensors** arranged radially at 60° increments inside an isolated 6-compartment waveguide cylinder. 

The system leverages a **two-stage distributed embedded architecture**: two high-speed **Teensy 4.1** microcontrollers handle real-time 256k-baud UART frame parsing, trigonometric rotation, and Euclidean seam deduplication, while a **Seeed Studio XIAO ESP32-S3** master coordinator aggregates fused binary streams over an 8 MHz SPI bus and broadcasts structured JSON telemetry over Wi-Fi/MQTT at 10 Hz.

---

## Table of Contents
1. [Architecture & Hardware Bug Fixes](#architecture--hardware-bug-fixes)
2. [Waveguide Hollow Cylinder & RF Isolation](#waveguide-hollow-cylinder--rf-isolation)
3. [Power Management (1S4P Matrix + MT3608)](#power-management-1s4p-matrix--mt3608)
4. [Hardware Pinout & Interconnects](#hardware-pinout--interconnects)
5. [Coordinate Geometry & Rotation Matrix](#coordinate-geometry--rotation-matrix)
6. [Boundary Seam Deduplication & SVG Overlay](#boundary-seam-deduplication--svg-overlay)
7. [MQTT Data Stream Schema](#mqtt-data-stream-schema)
8. [Building & Flashing](#building--flashing)
9. [Configuration Reference](#configuration-reference)

---

## Architecture & Hardware Bug Fixes

```
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │                              STAGE 1: REAL-TIME DSP & FUSION                           │
 ├─────────────────────────────────────────┬──────────────────────────────────────────────┤
 │             TEENSY 4.1 (NODE A)         │              TEENSY 4.1 (NODE B)             │
 │          600 MHz ARM Cortex-M7          │           600 MHz ARM Cortex-M7              │
 │                                         │                                              │
 │  [S1 @ 0°]   [S2 @ 60°]   [S3 @ 120°]   │  [S4 @ 180°]  [S5 @ 240°]  [S6 @ 300°]       │
 │   Serial1      Serial2      Serial3     │    Serial1       Serial2      Serial3        │
 │  (256kbaud)   (256kbaud)   (256kbaud)   │   (256kbaud)    (256kbaud)   (256kbaud)      │
 │      │            │            │        │       │             │            │           │
 │      ▼            ▼            ▼        │       ▼             ▼            ▼           │
 │  ┌───────────────────────────────────┐  │  ┌────────────────────────────────────────┐  │
 │  │ 3x UART Ring Buffers (Zero Drops) │  │  │  3x UART Ring Buffers (Zero Drops)     │  │
 │  │ LD2450 Sign-Magnitude Decoders    │  │  │  LD2450 Sign-Magnitude Decoders        │  │
 │  │ 2D Rotation Matrix (0°/60°/120°)  │  │  │  2D Rotation Matrix (180°/240°/300°)   │  │
 │  │ Euclidean Seam Deduplication      │  │  │  Euclidean Seam Deduplication          │  │
 │  │ TSPISlave + Tri-State MISO        │  │  │  TSPISlave + Tri-State MISO            │  │
 │  └─────────────────┬─────────────────┘  │  └───────────────────┬────────────────────┘  │
 └────────────────────┼────────────────────┴──────────────────────┼───────────────────────┘
                      │ (SPI Bus + CS_A + DRDY_A)                 │ (SPI Bus + CS_B + DRDY_B)
                      └─────────────────────────┬─────────────────┘
                                                ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │                      STAGE 2: MASTER COORDINATOR & BROADCASTER                         │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │                             SEEED STUDIO XIAO ESP32-S3                                 │
 │                              240 MHz Dual-Core LX7                                     │
 │                                                                                        │
 │   • High-Speed Hardware SPI Master Engine (8 MHz Bus Clock)                            │
 │   • Deterministic Multi-Node CS Arbitration & Hardware DRDY Interrupt Handshake        │
 │   • CRC16 Verification & Frame Synchronization                                         │
 │   • Exponential Backoff Wi-Fi Reconnection & Hardware Watchdog                         │
 │   • Zero-Heap-Leak Static Memory (StaticJsonDocument<2048>) @ 10 Hz                    │
 └──────────────────────────────────────────────┬─────────────────────────────────────────┘
                                                │ (10Hz Wi-Fi MQTT)
                                                ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │ MQTT BROKER / PONDEYES DASHBOARD                                                       │
 │ Topics: PondEyes/lighthouse/S1..S6  |  PondEyes/lighthouse/fused                       │
 └────────────────────────────────────────────────────────────────────────────────────────┘
```

### Critical Bug Fixes Implemented:
1. **Teensy SPI Hardware Compatibility (`TSPISlave.h`)**:
   - Replaced deprecated SPI slave declarations with `TSPISlave` register-level drivers targeting the i.MX RT1062 LPSPI peripheral.
2. **MISO Line Collision Prevention (Active Tri-State Switching)**:
   - When CS is HIGH (deselected), each Teensy immediately configures its hardware MISO pin (Pin 12) to high-impedance floating mode (`pinMode(12, INPUT);`).
   - MISO is driven as `OUTPUT` (`pinMode(12, OUTPUT);`) **only** when its specific CS line is pulled LOW by the ESP32-S3 master, preventing electrical contention on the shared MISO bus.
3. **Zero-Heap-Leak Static Memory Architecture on ESP32-S3**:
   - Eliminated dynamic heap allocation in the 10 Hz broadcast loop by utilizing a static fixed buffer (`StaticJsonDocument<2048> doc;`), completely preventing heap fragmentation and watchdog resets.
4. **Strict Configuration Validation**:
   - `radar_config.json` fully defines `arena.dimensions_mm: [10000, 10000]` and `arena.center_grid_mm: [5000, 5000]` alongside native integer pin maps.

---

## Waveguide Hollow Cylinder & RF Isolation

The physical chassis of the Omni-Lighthouse consists of a **130 mm diameter hollow cylinder** partitioned into **six 60° isolated radial compartments**.

```
                           [ S1 (0°) ]
                             ▲  ▲
                           /  ││  \
              [ S6 (300°) ] /───┼───\ [ S2 (60°) ]
                   ◄──     │ 1 │ 2 │     ──►
                   ◄──     │───┼───│     ──►
                           │ 6 │ 3 │
              [ S5 (240°) ] \───┼───/ [ S3 (120°) ]
                           \  ││  /
                             ▼  ▼
                          [ S4 (180°) ]
```

### RF Cross-Talk Mitigation:
1. **Operating Band**: 24.00 GHz – 24.25 GHz FMCW radar.
2. **Azimuth FOV**: Each sensor has an effective horizontal beamwidth of ±60° (120° total).
3. **Internal Waveguide Septa**: Each compartment features a 35 mm deep conductive barrier lined with 0.1 mm copper shielding tape to prevent direct line-of-sight transmit lobe bleed into neighboring radar receiver antennas.
4. **Antenna Mounting Radius ($R = 65.0\text{ mm}$)**: Sensor apertures sit flush with the cylinder perimeter.

---

## Power Management (1S4P Matrix + MT3608)

```
 ┌──────────────────────┐      ┌─────────────────────────┐      ┌──────────────────────┐
 │  1S4P 18650 Matrix   │ 3.7V │     MT3608 Step-Up      │ 5.0V │   LC Filter + Rail   │
 │   3.0V - 4.2V Nom    ├─────►│     Boost Converter     ├─────►│ L: 10µH, C: 470µF    ├──► 5V Rail
 │  14,000 mAh (51.8Wh) │      │ (Set: 5.05V, max 3.0A)  │      │ (Decoupling per node)│
 └──────────────────────┘      └─────────────────────────┘      └──────────────────────┘
```

### Power Specifications:
| Subsystem | Operating Voltage | Peak Current | Avg Current | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **6x HLK-LD2450 Radars** | 5.0 V DC | 6x 120 mA = 720 mA | ~550 mA | 24GHz FMCW RF transceivers |
| **2x Teensy 4.1 Nodes** | 5.0 V DC (VIN) | 2x 150 mA = 300 mA | ~200 mA | 600 MHz Cortex-M7 |
| **1x XIAO ESP32-S3** | 5.0 V DC (5V pin) | 350 mA (Wi-Fi TX) | ~120 mA | 2.4GHz 802.11 b/g/n active |
| **Total System Draw** | **5.0 V Rail** | **~1.37 A Peak** | **~870 mA Avg** | **~4.35 Watts** |

- **Battery Matrix**: 1S4P 18650 cells (14,000 mAh @ 3.7V = 51.8 Wh).
- **Runtime**: **10.8 to 14.5 Hours** continuous operation.
- **LC Low-Pass Filter**: $10\mu\text{H}$ inductor + $470\mu\text{F}$ low-ESR capacitor eliminates 1.2 MHz MT3608 switching ripple ($< 15\text{ mV}_\text{p-p}$).

---

## Hardware Pinout & Interconnects

### 1. Teensy 4.1 Node A (Sensors 1, 2, 3)
| Function | Teensy 4.1 Pin | Connected Device | Description |
| :--- | :--- | :--- | :--- |
| **Serial1 RX** | Pin 0 (RX1) | S1 (0°) TX | Sensor 1 256k-baud UART |
| **Serial1 TX** | Pin 1 (TX1) | S1 (0°) RX | Sensor 1 Config (Optional) |
| **Serial2 RX** | Pin 7 (RX2) | S2 (60°) TX | Sensor 2 256k-baud UART |
| **Serial2 TX** | Pin 8 (TX2) | S2 (60°) RX | Sensor 2 Config (Optional) |
| **Serial3 RX** | Pin 14 (RX3)| S3 (120°) TX | Sensor 3 256k-baud UART |
| **Serial3 TX** | Pin 15 (TX3)| S3 (120°) RX | Sensor 3 Config (Optional) |
| **SPI CS** | Pin 10 | XIAO D1 (GPIO 2) | SPI Slave Select (Active LOW) |
| **SPI MOSI** | Pin 11 | XIAO D10 (GPIO 9)| SPI Data In from Master |
| **SPI MISO** | Pin 12 | XIAO D9 (GPIO 8) | Tri-State MISO Data Out |
| **SPI SCK** | Pin 13 | XIAO D8 (GPIO 7) | SPI Clock (8 MHz) |
| **DRDY Handshake**| Pin 2 | XIAO D3 (GPIO 4) | Data Ready Handshake Line |
| **Power VIN** | VIN (5V) | 5.0V Boost Rail | System 5V Power Supply |
| **Ground** | GND | Common Ground | Unified Ground Plane |

### 2. Teensy 4.1 Node B (Sensors 4, 5, 6)
| Function | Teensy 4.1 Pin | Connected Device | Description |
| :--- | :--- | :--- | :--- |
| **Serial1 RX** | Pin 0 (RX1) | S4 (180°) TX | Sensor 4 256k-baud UART |
| **Serial1 TX** | Pin 1 (TX1) | S4 (180°) RX | Sensor 4 Config (Optional) |
| **Serial2 RX** | Pin 7 (RX2) | S5 (240°) TX | Sensor 5 256k-baud UART |
| **Serial2 TX** | Pin 8 (TX2) | S5 (240°) RX | Sensor 5 Config (Optional) |
| **Serial3 RX** | Pin 14 (RX3)| S6 (300°) TX | Sensor 6 256k-baud UART |
| **Serial3 TX** | Pin 15 (TX3)| S6 (300°) RX | Sensor 6 Config (Optional) |
| **SPI CS** | Pin 10 | XIAO D2 (GPIO 3) | SPI Slave Select (Active LOW) |
| **SPI MOSI** | Pin 11 | XIAO D10 (GPIO 9)| SPI Data In from Master |
| **SPI MISO** | Pin 12 | XIAO D9 (GPIO 8) | Tri-State MISO Data Out |
| **SPI SCK** | Pin 13 | XIAO D8 (GPIO 7) | SPI Clock (8 MHz) |
| **DRDY Handshake**| Pin 2 | XIAO D4 (GPIO 5) | Data Ready Handshake Line |
| **Power VIN** | VIN (5V) | 5.0V Boost Rail | System 5V Power Supply |
| **Ground** | GND | Common Ground | Unified Ground Plane |

### 3. Seeed Studio XIAO ESP32-S3 Master Coordinator
| Function | XIAO Pin | Connected Device | Description |
| :--- | :--- | :--- | :--- |
| **SPI SCK** | D8 (GPIO 7) | Node A Pin 13 & Node B Pin 13 | Master SPI Clock (8 MHz) |
| **SPI MISO**| D9 (GPIO 8) | Node A Pin 12 & Node B Pin 12 | Shared Master SPI MISO In |
| **SPI MOSI**| D10 (GPIO 9)| Node A Pin 11 & Node B Pin 11 | Shared Master SPI MOSI Out |
| **CS Node A**| D1 (GPIO 2) | Node A Pin 10 | Node A Chip Select |
| **CS Node B**| D2 (GPIO 3) | Node B Pin 10 | Node B Chip Select |
| **DRDY Node A**| D3 (GPIO 4) | Node A Pin 2 | Node A Data Ready Interrupt |
| **DRDY Node B**| D4 (GPIO 5) | Node B Pin 2 | Node B Data Ready Interrupt |
| **Status LED** | LED_BUILTIN (GPIO 21) | Onboard LED | Connection / Error Indicator |
| **Power 5V** | 5V Pin | 5.0V Boost Rail | System 5V Power Supply |
| **Ground** | GND | Common Ground | Unified Ground Plane |

---

## Coordinate Geometry & Rotation Matrix

- **Arena Center**: $(X_c, Y_c) = (5000.0\text{ mm}, 5000.0\text{ mm})$.
- **Sensor Radius**: $R = 65.0\text{ mm}$.
- **Sensor Angles $\theta_k$**: $0^\circ, 60^\circ, 120^\circ, 180^\circ, 240^\circ, 300^\circ$ clockwise from North ($+Y$).

$$\mathbf{X_{\text{global}}} = X_c + R \cdot \sin(\theta_k) + X_{\text{local}} \cdot \cos(\theta_k) + Y_{\text{local}} \cdot \sin(\theta_k)$$
$$\mathbf{Y_{\text{global}}} = Y_c + R \cdot \cos(\theta_k) - X_{\text{local}} \cdot \sin(\theta_k) + Y_{\text{local}} \cdot \cos(\theta_k)$$

---

## Boundary Seam Deduplication & SVG Overlay

Adjacent sensors overlap by 60° across boundary seams (at 30°, 90°, 150°, 210°, 270°, 330°). If distance between detections from different sensors is $< 500\text{ mm}$, they are fused into a single track $(\frac{X_i+X_j}{2}, \frac{Y_i+Y_j}{2})$.

### Dashboard SVG Overlay Code (`map.svg`):
```xml
<svg viewBox="0 0 10000 10000" width="100%" height="100%" xmlns="http://www.w3.org/2000/svg">
  <!-- Unified Arena Grid -->
  <rect width="10000" height="10000" fill="#0D1117" stroke="#30363D" stroke-width="2"/>
  
  <!-- Boundary Seam Rays for Visual Calibration -->
  <g id="seam-overlays" stroke="#FF2A2A" stroke-width="0.75pt" stroke-dasharray="4 2">
    <!-- Seam 1: 30° -->
    <line x1="5000" y1="5000" x2="8000" y2="10196" />
    <!-- Seam 2: 90° -->
    <line x1="5000" y1="5000" x2="11000" y2="5000" />
    <!-- Seam 3: 150° -->
    <line x1="5000" y1="5000" x2="8000" y2="-196" />
    <!-- Seam 4: 210° -->
    <line x1="5000" y1="5000" x2="2000" y2="-196" />
    <!-- Seam 5: 270° -->
    <line x1="5000" y1="5000" x2="-1000" y2="5000" />
    <!-- Seam 6: 330° -->
    <line x1="5000" y1="5000" x2="2000" y2="10196" />
  </g>
  
  <!-- Central Lighthouse Body -->
  <circle cx="5000" cy="5000" r="65" fill="#58A6FF" stroke="#FFFFFF" stroke-width="2"/>
</svg>
```

---

## MQTT Data Stream Schema

### Fused Telemetry Topic (`PondEyes/lighthouse/fused` @ 10 Hz)
```json
{
  "system": "PondEyes Omni-Lighthouse",
  "timestamp_ms": 145230,
  "rate_hz": 10,
  "total_active_targets": 2,
  "fused_targets": [
    {
      "sensor": 1,
      "track_id": 1,
      "x": 5120,
      "y": 7450,
      "speed": -24,
      "distance": 2450,
      "fused": false,
      "source_mask": 1
    },
    {
      "sensor": 2,
      "track_id": 1,
      "x": 6850,
      "y": 6200,
      "speed": 0,
      "distance": 2150,
      "fused": true,
      "source_mask": 3
    }
  ]
}
```

---

## Building & Flashing

```bash
# Build all firmware binaries
pio run

# Flash Teensy Node A
pio run -e teensy_node_a --target upload

# Flash Teensy Node B
pio run -e teensy_node_b --target upload

# Flash Seeed Studio XIAO ESP32-S3
pio run -e xiao_broadcaster --target upload
```

---

## Live 360° Tracking Canvas & Simulator

Run the real-time PyGame 360° tracking canvas and mock radar telemetry generator:

```bash
# 1. Install dependencies
pip install pygame paho-mqtt

# 2. Run the radar telemetry simulator (10 Hz mock feed)
python3 simulate_radar.py

# 3. In another terminal, launch the live visualization canvas
python3 main.py
```

---

## License

Distributed under the MIT License. See `LICENSE` for details.

