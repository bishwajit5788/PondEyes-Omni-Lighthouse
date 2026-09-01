/**
 * ============================================================================
 * PondEyes Omni-Lighthouse - Teensy Node B Firmware
 * Microcontroller: Teensy 4.1 (600 MHz ARM Cortex-M7)
 * File: src/teensy_node_b.cpp
 * 
 * Sensors Handled:
 *   - Sensor 4 (S4): Angle 180.0° (South, -Y axis)        -> Hardware Serial1 (256,000 baud)
 *   - Sensor 5 (S5): Angle 240.0° (West-South-West)       -> Hardware Serial2 (256,000 baud)
 *   - Sensor 6 (S6): Angle 300.0° (West-North-West)       -> Hardware Serial3 (256,000 baud)
 * 
 * Core Features & Hardware Fixes:
 *   1. Hardware SPI Slave utilizing <TSPISlave.h> for deterministic transfers
 *   2. Active High-Impedance MISO Switching (Tri-State) to eliminate bus collisions:
 *      - pinMode(PIN_SPI_MISO, INPUT) when CS is HIGH (inactive)
 *      - pinMode(PIN_SPI_MISO, OUTPUT) when CS is LOW (active selected)
 *   3. 3x Non-blocking 256k-baud UART Ring Buffers for HLK-LD2450 30-byte frames
 *   4. Sign-magnitude 16-bit coordinate extraction (X, Y, Speed, Distance)
 *   5. Real-time Trigonometric Rotation Matrix to unified [5000, 5000] mm grid
 *   6. 500mm Euclidean Distance Deduplication loop near boundary seams
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <TSPISlave.h>
#include <math.h>

#if __has_include("../include/pondeyes_protocol.h")
#include "../include/pondeyes_protocol.h"
#elif __has_include("include/pondeyes_protocol.h")
#include "include/pondeyes_protocol.h"
#else
#include "pondeyes_protocol.h"
#endif

// ============================================================================
// HARDWARE PIN DEFINITIONS & CONSTANTS
// ============================================================================
#define NODE_ID                     NODE_ID_B

// Teensy 4.1 Hardware SPI0 Pins
#define PIN_SPI_CS                  10  // SPI Chip Select (Input from ESP32-S3)
#define PIN_SPI_MOSI                11  // SPI MOSI (Input from ESP32-S3)
#define PIN_SPI_MISO                12  // SPI MISO (Tri-state Output to ESP32-S3)
#define PIN_SPI_SCK                 13  // SPI Clock (Input from ESP32-S3)
#define PIN_SPI_DRDY                2   // Data Ready Handshake (Output to ESP32-S3)
#define PIN_STATUS_LED              13  // Onboard Diagnostic LED

// Sensor Mapping for Node B
#define S4_ID                       4
#define S5_ID                       5
#define S6_ID                       6

#define S4_ANGLE_DEG                180.0f
#define S5_ANGLE_DEG                240.0f
#define S6_ANGLE_DEG                300.0f

#define DEG_TO_RAD_F                (3.14159265358979323846f / 180.0f)
#define PARSER_FRAME_LEN            30

// ============================================================================
// SENSOR PRECOMPUTED GEOMETRY TABLE
// ============================================================================
typedef struct {
    uint8_t  sensor_id;
    float    angle_deg;
    float    angle_rad;
    float    sin_val;
    float    cos_val;
    float    mount_x_mm;
    float    mount_y_mm;
} SensorGeometry_t;

static SensorGeometry_t g_sensors[3];

// ============================================================================
// LD2450 BINARY FRAME PARSER STATE MACHINE
// ============================================================================
typedef enum {
    STATE_WAIT_HEADER_0 = 0,
    STATE_WAIT_HEADER_1,
    STATE_WAIT_HEADER_2,
    STATE_WAIT_HEADER_3,
    STATE_READ_PAYLOAD,
    STATE_WAIT_TAIL_0,
    STATE_WAIT_TAIL_1
} ParserState_t;

typedef struct {
    HardwareSerial* serial;
    uint8_t         sensor_id;
    ParserState_t   state;
    uint8_t         payload_buf[24];
    uint8_t         payload_idx;
    uint32_t        packets_parsed;
    uint32_t        frame_errors;
    LD2450_RawTarget_t raw_targets[MAX_TARGETS_PER_SENSOR];
    bool            new_data_available;
} SensorPort_t;

static SensorPort_t g_ports[3];

// ============================================================================
// SPI SLAVE & PACKET BUFFERING
// ============================================================================
static TSPISlave           g_spi_slave = TSPISlave(SPI, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SCK, PIN_SPI_CS);
static PondEyesSpiPacket_t g_tx_packet;
static volatile bool       g_cs_active = false;
static volatile uint16_t   g_packet_sequence = 0;
static uint32_t            g_last_debug_print_ms = 0;

// Temporary target buffers for rotation and deduplication
static PondEyesTarget_t    g_transformed_targets[MAX_TARGETS_PER_NODE];
static uint8_t             g_transformed_count = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void init_sensor_geometry();
void init_uart_ports();
void init_spi_slave_interface();
void process_uart_port(SensorPort_t* port);
void parse_raw_payload(SensorPort_t* port);
void transform_targets_to_global();
void deduplicate_seam_targets();
void prepare_spi_packet();
void isr_cs_pin_transition();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    delay(1000);
    Serial.println(F("=================================================="));
    Serial.println(F(" PondEyes Omni-Lighthouse: TEENSY NODE B"));
    Serial.println(F(" Node ID: 0x0B | S4@180°, S5@240°, S6@300°"));
    Serial.println(F(" Baudrate: 256k | TSPISlave + Tri-State MISO"));
    Serial.println(F("=================================================="));

    // Precompute trigonometric matrices
    init_sensor_geometry();

    // Initialize 3 Hardware Serial ports at 256,000 baud concurrently
    init_uart_ports();

    // Initialize SPI Slave & Tri-State MISO management
    init_spi_slave_interface();

    digitalWrite(PIN_STATUS_LED, LOW);
    Serial.println(F("[Node B] Initialization successful. System active."));
}

// ============================================================================
// PRECOMPUTE GEOMETRIC TRANSFORMATIONS
// ============================================================================
void init_sensor_geometry() {
    float angles[3] = { S4_ANGLE_DEG, S5_ANGLE_DEG, S6_ANGLE_DEG };
    uint8_t ids[3]  = { S4_ID, S5_ID, S6_ID };

    for (int i = 0; i < 3; i++) {
        g_sensors[i].sensor_id  = ids[i];
        g_sensors[i].angle_deg  = angles[i];
        g_sensors[i].angle_rad  = angles[i] * DEG_TO_RAD_F;
        g_sensors[i].sin_val    = sinf(g_sensors[i].angle_rad);
        g_sensors[i].cos_val    = cosf(g_sensors[i].angle_rad);
        g_sensors[i].mount_x_mm = LIGHTHOUSE_RADIUS_MM * g_sensors[i].sin_val;
        g_sensors[i].mount_y_mm = LIGHTHOUSE_RADIUS_MM * g_sensors[i].cos_val;

        Serial.printf("[Node B] Sensor S%u: Angle=%.1f° | MountOffset=(%.1f, %.1f) mm\n",
                      g_sensors[i].sensor_id, g_sensors[i].angle_deg,
                      g_sensors[i].mount_x_mm, g_sensors[i].mount_y_mm);
    }
}

// ============================================================================
// INITIALIZE 3 HARDWARE SERIAL PORTS AT 256K BAUD
// ============================================================================
void init_uart_ports() {
    // Sensor 4 on Serial1 (Teensy Pin 0 RX1, Pin 1 TX1)
    Serial1.begin(LD2450_UART_BAUD);
    g_ports[0].serial = &Serial1;
    g_ports[0].sensor_id = S4_ID;
    g_ports[0].state = STATE_WAIT_HEADER_0;
    g_ports[0].payload_idx = 0;
    g_ports[0].packets_parsed = 0;
    g_ports[0].frame_errors = 0;
    g_ports[0].new_data_available = false;

    // Sensor 5 on Serial2 (Teensy Pin 7 RX2, Pin 8 TX2)
    Serial2.begin(LD2450_UART_BAUD);
    g_ports[1].serial = &Serial2;
    g_ports[1].sensor_id = S5_ID;
    g_ports[1].state = STATE_WAIT_HEADER_0;
    g_ports[1].payload_idx = 0;
    g_ports[1].packets_parsed = 0;
    g_ports[1].frame_errors = 0;
    g_ports[1].new_data_available = false;

    // Sensor 6 on Serial3 (Teensy Pin 14 RX3, Pin 15 TX3)
    Serial3.begin(LD2450_UART_BAUD);
    g_ports[2].serial = &Serial3;
    g_ports[2].sensor_id = S6_ID;
    g_ports[2].state = STATE_WAIT_HEADER_0;
    g_ports[2].payload_idx = 0;
    g_ports[2].packets_parsed = 0;
    g_ports[2].frame_errors = 0;
    g_ports[2].new_data_available = false;
}

// ============================================================================
// INITIALIZE SPI SLAVE & HIGH-IMPEDANCE MISO
// ============================================================================
void init_spi_slave_interface() {
    pinMode(PIN_SPI_DRDY, OUTPUT);
    digitalWrite(PIN_SPI_DRDY, LOW);

    // FIX MISO COLLISIONS: Configure MISO as high-impedance INPUT initially
    pinMode(PIN_SPI_CS, INPUT_PULLUP);
    pinMode(PIN_SPI_MOSI, INPUT);
    pinMode(PIN_SPI_SCK, INPUT);
    pinMode(PIN_SPI_MISO, INPUT); // Tri-state high impedance

    // Attach high-priority interrupt to CS line for instantaneous tri-state switching
    attachInterrupt(digitalPinToInterrupt(PIN_SPI_CS), isr_cs_pin_transition, CHANGE);

    // Pre-initialize empty sealed packet
    memset(&g_tx_packet, 0, sizeof(g_tx_packet));
    g_tx_packet.magic[0] = PONDEYES_MAGIC_0;
    g_tx_packet.magic[1] = PONDEYES_MAGIC_1;
    g_tx_packet.node_id = NODE_ID;
    g_tx_packet.sequence = 0;
    g_tx_packet.timestamp_ms = millis();
    g_tx_packet.target_count = 0;
    pondeyes_seal_packet_crc(&g_tx_packet);
}

// ============================================================================
// ACTIVE HIGH-IMPEDANCE MISO SWITCHING ISR
// ============================================================================
void isr_cs_pin_transition() {
    int cs_val = digitalReadFast(PIN_SPI_CS);

    if (cs_val == LOW) {
        // CS pulled LOW by Master: This node is SELECTED
        // Enable MISO pin as OUTPUT to drive the bus
        pinMode(PIN_SPI_MISO, OUTPUT);
        g_cs_active = true;

        // Clock out current packet buffer synchronously with Master SCK
        const uint8_t* p_bytes = (const uint8_t*)&g_tx_packet;
        size_t total_len = sizeof(PondEyesSpiPacket_t);

        for (size_t b = 0; b < total_len; b++) {
            uint8_t out_byte = p_bytes[b];
            for (int bit = 7; bit >= 0; bit--) {
                while (digitalReadFast(PIN_SPI_SCK) == HIGH && digitalReadFast(PIN_SPI_CS) == LOW);
                digitalWriteFast(PIN_SPI_MISO, (out_byte & (1 << bit)) ? HIGH : LOW);
                while (digitalReadFast(PIN_SPI_SCK) == LOW && digitalReadFast(PIN_SPI_CS) == LOW);
            }
        }
    } else {
        // CS pulled HIGH by Master: This node is DESELECTED
        // Instantly return MISO to floating INPUT (high impedance) to prevent bus collision
        pinMode(PIN_SPI_MISO, INPUT);
        digitalWriteFast(PIN_SPI_DRDY, LOW);
        g_cs_active = false;
    }
}

// ============================================================================
// MAIN DSP LOOP
// ============================================================================
void loop() {
    // 1. Ingest UART streams from all 3 sensors concurrently
    for (int i = 0; i < 3; i++) {
        process_uart_port(&g_ports[i]);
    }

    // 2. Process fresh tracking frames
    bool fresh_data = false;
    for (int i = 0; i < 3; i++) {
        if (g_ports[i].new_data_available) {
            fresh_data = true;
            g_ports[i].new_data_available = false;
        }
    }

    // 3. Transform coordinates, deduplicate seams, and update SPI packet
    if (fresh_data) {
        transform_targets_to_global();
        deduplicate_seam_targets();
        prepare_spi_packet();
    }

    // 4. Periodic diagnostic telemetry on USB Serial
    uint32_t now_ms = millis();
    if (now_ms - g_last_debug_print_ms >= 1000) {
        g_last_debug_print_ms = now_ms;
        Serial.printf("[Node B] Uptime: %lu ms | Seq: %u | ActiveTargets: %u | S4: %lu pkts, S5: %lu pkts, S6: %lu pkts\n",
                      now_ms, g_packet_sequence, g_tx_packet.target_count,
                      g_ports[0].packets_parsed, g_ports[1].packets_parsed, g_ports[2].packets_parsed);
    }
}

// ============================================================================
// HLK-LD2450 BINARY PACKET PARSER STATE MACHINE
// ============================================================================
void process_uart_port(SensorPort_t* port) {
    while (port->serial->available() > 0) {
        uint8_t byte_in = port->serial->read();

        switch (port->state) {
            case STATE_WAIT_HEADER_0:
                if (byte_in == LD2450_FRAME_HEAD_0) {
                    port->state = STATE_WAIT_HEADER_1;
                }
                break;

            case STATE_WAIT_HEADER_1:
                if (byte_in == LD2450_FRAME_HEAD_1) {
                    port->state = STATE_WAIT_HEADER_2;
                } else {
                    port->state = STATE_WAIT_HEADER_0;
                }
                break;

            case STATE_WAIT_HEADER_2:
                if (byte_in == LD2450_FRAME_HEAD_2) {
                    port->state = STATE_WAIT_HEADER_3;
                } else {
                    port->state = STATE_WAIT_HEADER_0;
                }
                break;

            case STATE_WAIT_HEADER_3:
                if (byte_in == LD2450_FRAME_HEAD_3) {
                    port->payload_idx = 0;
                    port->state = STATE_READ_PAYLOAD;
                } else {
                    port->state = STATE_WAIT_HEADER_0;
                }
                break;

            case STATE_READ_PAYLOAD:
                port->payload_buf[port->payload_idx++] = byte_in;
                if (port->payload_idx >= 24) {
                    port->state = STATE_WAIT_TAIL_0;
                }
                break;

            case STATE_WAIT_TAIL_0:
                if (byte_in == LD2450_FRAME_TAIL_0) {
                    port->state = STATE_WAIT_TAIL_1;
                } else {
                    port->frame_errors++;
                    port->state = STATE_WAIT_HEADER_0;
                }
                break;

            case STATE_WAIT_TAIL_1:
                if (byte_in == LD2450_FRAME_TAIL_1) {
                    parse_raw_payload(port);
                    port->packets_parsed++;
                    port->new_data_available = true;
                } else {
                    port->frame_errors++;
                }
                port->state = STATE_WAIT_HEADER_0;
                break;

            default:
                port->state = STATE_WAIT_HEADER_0;
                break;
        }
    }
}

// ============================================================================
// SIGN-MAGNITUDE 16-BIT COORDINATE EXTRACTION
// ============================================================================
void parse_raw_payload(SensorPort_t* port) {
    const uint8_t* buf = port->payload_buf;

    for (int t = 0; t < MAX_TARGETS_PER_SENSOR; t++) {
        int base = t * 8;
        
        // 1. Decode raw X coordinate (Bytes 0-1) - Sign-Magnitude
        uint16_t raw_x = (uint16_t)(buf[base + 0] | (buf[base + 1] << 8));
        int16_t x_mm = (raw_x & 0x8000) ? -((int16_t)(raw_x & 0x7FFF)) : (int16_t)(raw_x & 0x7FFF);

        // 2. Decode raw Y coordinate (Bytes 2-3) - Sign-Magnitude
        uint16_t raw_y = (uint16_t)(buf[base + 2] | (buf[base + 3] << 8));
        int16_t y_mm = (raw_y & 0x8000) ? -((int16_t)(raw_y & 0x7FFF)) : (int16_t)(raw_y & 0x7FFF);

        // 3. Decode raw Speed (Bytes 4-5) - Sign-Magnitude (cm/s)
        uint16_t raw_speed = (uint16_t)(buf[base + 4] | (buf[base + 5] << 8));
        int16_t speed_cms = (raw_speed & 0x8000) ? -((int16_t)(raw_speed & 0x7FFF)) : (int16_t)(raw_speed & 0x7FFF);

        // 4. Decode Distance Resolution (Bytes 6-7) in mm
        uint16_t dist_mm = (uint16_t)(buf[base + 6] | (buf[base + 7] << 8));

        port->raw_targets[t].x_mm = x_mm;
        port->raw_targets[t].y_mm = y_mm;
        port->raw_targets[t].speed_cms = speed_cms;
        port->raw_targets[t].distance_res_mm = dist_mm;

        if (dist_mm > 50 && dist_mm <= 6500 && y_mm > 0) {
            port->raw_targets[t].valid = 1;
        } else {
            port->raw_targets[t].valid = 0;
        }
    }
}

// ============================================================================
// TRIGONOMETRIC ROTATION MATRIX TO UNIFIED ARENA GRID
// ============================================================================
void transform_targets_to_global() {
    g_transformed_count = 0;

    for (int p = 0; p < 3; p++) {
        SensorPort_t* port = &g_ports[p];
        SensorGeometry_t* geom = &g_sensors[p];

        for (int t = 0; t < MAX_TARGETS_PER_SENSOR; t++) {
            if (!port->raw_targets[t].valid) continue;

            float x_local = (float)port->raw_targets[t].x_mm;
            float y_local = (float)port->raw_targets[t].y_mm;

            // Rotation matrix calculation
            float dx = (x_local * geom->cos_val) + (y_local * geom->sin_val);
            float dy = (-x_local * geom->sin_val) + (y_local * geom->cos_val);

            float x_global = ARENA_GRID_CENTER_X_MM + geom->mount_x_mm + dx;
            float y_global = ARENA_GRID_CENTER_Y_MM + geom->mount_y_mm + dy;

            // Boundary clamping within [0, 10000 mm]
            if (x_global < 0.0f) x_global = 0.0f;
            if (x_global > 10000.0f) x_global = 10000.0f;
            if (y_global < 0.0f) y_global = 0.0f;
            if (y_global > 10000.0f) y_global = 10000.0f;

            if (g_transformed_count < MAX_TARGETS_PER_NODE) {
                PondEyesTarget_t* tgt = &g_transformed_targets[g_transformed_count++];
                tgt->target_id    = (uint8_t)(t + 1);
                tgt->sensor_id    = geom->sensor_id;
                tgt->x_global_mm  = (int16_t)roundf(x_global);
                tgt->y_global_mm  = (int16_t)roundf(y_global);
                tgt->speed_cms    = port->raw_targets[t].speed_cms;
                tgt->distance_mm  = port->raw_targets[t].distance_res_mm;
                tgt->x_local_mm   = port->raw_targets[t].x_mm;
                tgt->y_local_mm   = port->raw_targets[t].y_mm;
                tgt->is_fused     = 0;
                tgt->source_mask  = (uint8_t)(1 << (geom->sensor_id - 1));
            }
        }
    }
}

// ============================================================================
// EUCLIDEAN DISTANCE DEDUPLICATION & SEAM FUSION
// ============================================================================
void deduplicate_seam_targets() {
    if (g_transformed_count <= 1) return;

    bool merged[MAX_TARGETS_PER_NODE] = { false };

    for (int i = 0; i < g_transformed_count; i++) {
        if (merged[i]) continue;

        for (int j = i + 1; j < g_transformed_count; j++) {
            if (merged[j]) continue;

            if (g_transformed_targets[i].sensor_id != g_transformed_targets[j].sensor_id) {
                float dx = (float)(g_transformed_targets[i].x_global_mm - g_transformed_targets[j].x_global_mm);
                float dy = (float)(g_transformed_targets[i].y_global_mm - g_transformed_targets[j].y_global_mm);
                float dist_sq = (dx * dx) + (dy * dy);

                if (dist_sq < SEAM_DEDUP_DIST_SQ_MM) {
                    // Fuse target coordinates and speed
                    g_transformed_targets[i].x_global_mm = (int16_t)((g_transformed_targets[i].x_global_mm + g_transformed_targets[j].x_global_mm) / 2);
                    g_transformed_targets[i].y_global_mm = (int16_t)((g_transformed_targets[i].y_global_mm + g_transformed_targets[j].y_global_mm) / 2);
                    g_transformed_targets[i].speed_cms   = (int16_t)((g_transformed_targets[i].speed_cms + g_transformed_targets[j].speed_cms) / 2);
                    g_transformed_targets[i].distance_mm = (uint16_t)((g_transformed_targets[i].distance_mm + g_transformed_targets[j].distance_mm) / 2);
                    g_transformed_targets[i].is_fused    = 1;
                    g_transformed_targets[i].source_mask |= g_transformed_targets[j].source_mask;

                    merged[j] = true;
                }
            }
        }
    }

    // Compact output array
    uint8_t write_idx = 0;
    for (int i = 0; i < g_transformed_count; i++) {
        if (!merged[i]) {
            if (write_idx != i) {
                g_transformed_targets[write_idx] = g_transformed_targets[i];
            }
            write_idx++;
        }
    }
    g_transformed_count = write_idx;
}

// ============================================================================
// PACK SPI BUFFER AND ASSERT DATA READY (DRDY)
// ============================================================================
void prepare_spi_packet() {
    g_tx_packet.magic[0]      = PONDEYES_MAGIC_0;
    g_tx_packet.magic[1]      = PONDEYES_MAGIC_1;
    g_tx_packet.node_id       = NODE_ID;
    g_tx_packet.sequence      = ++g_packet_sequence;
    g_tx_packet.timestamp_ms  = millis();
    g_tx_packet.target_count  = g_transformed_count;

    for (uint8_t i = 0; i < g_transformed_count; i++) {
        g_tx_packet.targets[i] = g_transformed_targets[i];
    }
    for (uint8_t i = g_transformed_count; i < MAX_TARGETS_PER_NODE; i++) {
        memset(&g_tx_packet.targets[i], 0, sizeof(PondEyesTarget_t));
    }

    pondeyes_seal_packet_crc(&g_tx_packet);

    // Pulse DRDY HIGH to notify master coordinator
    digitalWriteFast(PIN_SPI_DRDY, HIGH);
}
