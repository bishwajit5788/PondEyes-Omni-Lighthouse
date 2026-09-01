/**
 * ============================================================================
 * PondEyes Omni-Lighthouse - Teensy Node A Firmware
 * Microcontroller: Teensy 4.1 (600 MHz ARM Cortex-M7)
 * Sensors Handled:
 *   - Sensor 1 (S1): Angle 0.0°   (North, +Y axis)        -> Hardware Serial1 (256,000 baud)
 *   - Sensor 2 (S2): Angle 60.0°  (East-North-East)       -> Hardware Serial2 (256,000 baud)
 *   - Sensor 3 (S3): Angle 120.0° (East-South-East)       -> Hardware Serial3 (256,000 baud)
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "pondeyes_protocol.h"

// ============================================================================
// HARDWARE PIN DEFINITIONS & CONSTANTS
// ============================================================================
#define NODE_ID                     NODE_ID_A

// Teensy 4.1 Hardware SPI0 Pins
#define PIN_SPI_CS                  10  // SPI Chip Select (Input from ESP32-S3)
#define PIN_SPI_MOSI                11  // SPI MOSI (Input from ESP32-S3)
#define PIN_SPI_MISO                12  // SPI MISO (Output to ESP32-S3)
#define PIN_SPI_SCK                 13  // SPI Clock (Input from ESP32-S3)
#define PIN_SPI_DRDY                2   // Data Ready Handshake (Output to ESP32-S3)
#define PIN_STATUS_LED              13  // Onboard LED for status indication

// Sensor Mapping for Node A
#define S1_ID                       1
#define S2_ID                       2
#define S3_ID                       3

#define S1_ANGLE_DEG                0.0f
#define S2_ANGLE_DEG                60.0f
#define S3_ANGLE_DEG                120.0f

#define DEG_TO_RAD_F                (3.14159265358979323846f / 180.0f)

// Ring buffer sizing for high-baud UART
#define UART_RX_BUFFER_SIZE         512
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
    float    mount_x_mm;  // Offset from lighthouse center
    float    mount_y_mm;  // Offset from lighthouse center
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
static PondEyesSpiPacket_t g_tx_packet;
static volatile bool       g_spi_transfer_in_progress = false;
static volatile uint16_t   g_packet_sequence = 0;
static uint32_t            g_last_loop_time_us = 0;
static uint32_t            g_last_debug_print_ms = 0;

// Temporary target buffers for rotation and deduplication
static PondEyesTarget_t    g_transformed_targets[MAX_TARGETS_PER_NODE];
static uint8_t             g_transformed_count = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void init_sensor_geometry();
void init_uart_ports();
void init_spi_interface();
void process_uart_port(SensorPort_t* port);
void parse_raw_payload(SensorPort_t* port);
void transform_targets_to_global();
void deduplicate_seam_targets();
void prepare_spi_packet();
void isr_spi_cs_changed();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    // Initialize USB Serial for diagnostic telemetry
    Serial.begin(115200);
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    delay(1000);
    Serial.println(F("=================================================="));
    Serial.println(F(" PondEyes Omni-Lighthouse: TEENSY NODE A STARTING"));
    Serial.println(F(" Node ID: 0x0A (Sensors: S1@0°, S2@60°, S3@120°)"));
    Serial.println(F(" UART Baudrate: 256,000 bps | 600 MHz Cortex-M7"));
    Serial.println(F("=================================================="));

    // Precompute trigonometric matrices for 0°, 60°, and 120°
    init_sensor_geometry();

    // Initialize 3 Hardware Serial ports at 256,000 baud concurrently
    init_uart_ports();

    // Initialize SPI Slave / Handshake interface
    init_spi_interface();

    digitalWrite(PIN_STATUS_LED, LOW);
    Serial.println(F("[Node A] Initialization complete. Entering high-speed DSP loop."));
}

// ============================================================================
// PRECOMPUTE GEOMETRIC TRANSFORMATIONS
// ============================================================================
void init_sensor_geometry() {
    float angles[3] = { S1_ANGLE_DEG, S2_ANGLE_DEG, S3_ANGLE_DEG };
    uint8_t ids[3]  = { S1_ID, S2_ID, S3_ID };

    for (int i = 0; i < 3; i++) {
        g_sensors[i].sensor_id  = ids[i];
        g_sensors[i].angle_deg  = angles[i];
        g_sensors[i].angle_rad  = angles[i] * DEG_TO_RAD_F;
        g_sensors[i].sin_val    = sinf(g_sensors[i].angle_rad);
        g_sensors[i].cos_val    = cosf(g_sensors[i].angle_rad);
        
        // Sensor aperture mounting coordinate offset relative to lighthouse center
        // X = R * sin(theta), Y = R * cos(theta) (where 0 deg = North / +Y axis)
        g_sensors[i].mount_x_mm = LIGHTHOUSE_RADIUS_MM * g_sensors[i].sin_val;
        g_sensors[i].mount_y_mm = LIGHTHOUSE_RADIUS_MM * g_sensors[i].cos_val;

        Serial.printf("[Node A] Sensor S%u: Angle=%.1f deg | Sin=%.4f, Cos=%.4f | MountOffset=(%.1f, %.1f) mm\n",
                      g_sensors[i].sensor_id, g_sensors[i].angle_deg,
                      g_sensors[i].sin_val, g_sensors[i].cos_val,
                      g_sensors[i].mount_x_mm, g_sensors[i].mount_y_mm);
    }
}

// ============================================================================
// INITIALIZE 3 HARDWARE SERIAL PORTS AT 256K BAUD
// ============================================================================
void init_uart_ports() {
    // Sensor 1 on Serial1 (Teensy Pin 0 RX1, Pin 1 TX1)
    Serial1.begin(LD2450_UART_BAUD);
    g_ports[0].serial = &Serial1;
    g_ports[0].sensor_id = S1_ID;
    g_ports[0].state = STATE_WAIT_HEADER_0;
    g_ports[0].payload_idx = 0;
    g_ports[0].packets_parsed = 0;
    g_ports[0].frame_errors = 0;
    g_ports[0].new_data_available = false;

    // Sensor 2 on Serial2 (Teensy Pin 7 RX2, Pin 8 TX2)
    Serial2.begin(LD2450_UART_BAUD);
    g_ports[1].serial = &Serial2;
    g_ports[1].sensor_id = S2_ID;
    g_ports[1].state = STATE_WAIT_HEADER_0;
    g_ports[1].payload_idx = 0;
    g_ports[1].packets_parsed = 0;
    g_ports[1].frame_errors = 0;
    g_ports[1].new_data_available = false;

    // Sensor 3 on Serial3 (Teensy Pin 14 RX3, Pin 15 TX3)
    Serial3.begin(LD2450_UART_BAUD);
    g_ports[2].serial = &Serial3;
    g_ports[2].sensor_id = S3_ID;
    g_ports[2].state = STATE_WAIT_HEADER_0;
    g_ports[2].payload_idx = 0;
    g_ports[2].packets_parsed = 0;
    g_ports[2].frame_errors = 0;
    g_ports[2].new_data_available = false;
}

// ============================================================================
// INITIALIZE SPI INTERFACE
// ============================================================================
void init_spi_interface() {
    pinMode(PIN_SPI_DRDY, OUTPUT);
    digitalWrite(PIN_SPI_DRDY, LOW);

    pinMode(PIN_SPI_CS, INPUT_PULLUP);
    pinMode(PIN_SPI_MOSI, INPUT);
    pinMode(PIN_SPI_MISO, OUTPUT);
    pinMode(PIN_SPI_SCK, INPUT);

    // Set up CS interrupt to synchronize SPI transfers with ESP32-S3 Master
    attachInterrupt(digitalPinToInterrupt(PIN_SPI_CS), isr_spi_cs_changed, CHANGE);

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
// MAIN REAL-TIME LOOP
// ============================================================================
void loop() {
    // 1. Ingest incoming UART byte streams from all 3 radar sensors concurrently
    for (int i = 0; i < 3; i++) {
        process_uart_port(&g_ports[i]);
    }

    // 2. Check if any sensor has fresh parsed frames
    bool fresh_data = false;
    for (int i = 0; i < 3; i++) {
        if (g_ports[i].new_data_available) {
            fresh_data = true;
            g_ports[i].new_data_available = false;
        }
    }

    // 3. Perform Coordinate Transformation, Boundary Seam Deduplication, and SPI buffering
    if (fresh_data) {
        transform_targets_to_global();
        deduplicate_seam_targets();
        prepare_spi_packet();
    }

    // 4. Periodic diagnostic telemetry on USB Serial (every 1000ms)
    uint32_t now_ms = millis();
    if (now_ms - g_last_debug_print_ms >= 1000) {
        g_last_debug_print_ms = now_ms;
        Serial.printf("[Node A Status] Uptime: %lu ms | Seq: %u | ValidTargets: %u | S1_pkts: %lu, S2_pkts: %lu, S3_pkts: %lu\n",
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
                    // Frame completed and validated successfully
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
        
        // 1. Decode raw X coordinate (Bytes 0-1) - Sign-Magnitude format
        // Bit 15: 1 = Negative (Left), 0 = Positive (Right)
        uint16_t raw_x = (uint16_t)(buf[base + 0] | (buf[base + 1] << 8));
        int16_t x_mm;
        if (raw_x & 0x8000) {
            x_mm = -((int16_t)(raw_x & 0x7FFF));
        } else {
            x_mm = (int16_t)(raw_x & 0x7FFF);
        }

        // 2. Decode raw Y coordinate (Bytes 2-3) - Sign-Magnitude format
        // Bit 15: 1 = Negative, 0 = Positive (Bore-sight distance in front of radar)
        uint16_t raw_y = (uint16_t)(buf[base + 2] | (buf[base + 3] << 8));
        int16_t y_mm;
        if (raw_y & 0x8000) {
            y_mm = -((int16_t)(raw_y & 0x7FFF));
        } else {
            y_mm = (int16_t)(raw_y & 0x7FFF);
        }

        // 3. Decode raw Speed (Bytes 4-5) - Sign-Magnitude format
        // Unit: cm/s. Bit 15: 1 = Negative (Approaching), 0 = Positive (Moving away)
        uint16_t raw_speed = (uint16_t)(buf[base + 4] | (buf[base + 5] << 8));
        int16_t speed_cms;
        if (raw_speed & 0x8000) {
            speed_cms = -((int16_t)(raw_speed & 0x7FFF));
        } else {
            speed_cms = (int16_t)(raw_speed & 0x7FFF);
        }

        // 4. Decode Distance Resolution / Distance (Bytes 6-7)
        // Unit: mm
        uint16_t dist_mm = (uint16_t)(buf[base + 6] | (buf[base + 7] << 8));

        // Store into raw targets buffer
        port->raw_targets[t].x_mm = x_mm;
        port->raw_targets[t].y_mm = y_mm;
        port->raw_targets[t].speed_cms = speed_cms;
        port->raw_targets[t].distance_res_mm = dist_mm;

        // Validity heuristic: Target is active if measured distance > 0 or coordinates non-zero
        // Also ensure target is within standard detection boundary (< 6500 mm)
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

            // Apply rotation matrix
            float dx = (x_local * geom->cos_val) + (y_local * geom->sin_val);
            float dy = (-x_local * geom->sin_val) + (y_local * geom->cos_val);

            float x_global = ARENA_GRID_CENTER_X_MM + geom->mount_x_mm + dx;
            float y_global = ARENA_GRID_CENTER_Y_MM + geom->mount_y_mm + dy;

            // Bounds clamping within arena grid [0, 10000 mm]
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

            // Only deduplicate targets reported by different sensors
            if (g_transformed_targets[i].sensor_id != g_transformed_targets[j].sensor_id) {
                float dx = (float)(g_transformed_targets[i].x_global_mm - g_transformed_targets[j].x_global_mm);
                float dy = (float)(g_transformed_targets[i].y_global_mm - g_transformed_targets[j].y_global_mm);
                float dist_sq = (dx * dx) + (dy * dy);

                if (dist_sq < SEAM_DEDUP_DIST_SQ_MM) {
                    // Fuse target j into target i
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

    // Compact transformed array to eliminate merged duplicates
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
    // Populate binary packet header
    g_tx_packet.magic[0]      = PONDEYES_MAGIC_0;
    g_tx_packet.magic[1]      = PONDEYES_MAGIC_1;
    g_tx_packet.node_id       = NODE_ID;
    g_tx_packet.sequence      = ++g_packet_sequence;
    g_tx_packet.timestamp_ms  = millis();
    g_tx_packet.target_count  = g_transformed_count;

    // Copy cleaned target array
    for (uint8_t i = 0; i < g_transformed_count; i++) {
        g_tx_packet.targets[i] = g_transformed_targets[i];
    }
    // Clear unused target slots
    for (uint8_t i = g_transformed_count; i < MAX_TARGETS_PER_NODE; i++) {
        memset(&g_tx_packet.targets[i], 0, sizeof(PondEyesTarget_t));
    }

    // Calculate CRC16-CCITT and seal packet
    pondeyes_seal_packet_crc(&g_tx_packet);

    // Pulse Data-Ready line HIGH to signal ESP32-S3 Master that fresh data is ready
    digitalWrite(PIN_SPI_DRDY, HIGH);
}

// ============================================================================
// SPI CS INTERRUPT SERVICE ROUTINE (HIGH-SPEED HARDWARE SPI SLAVE EMULATOR)
// ============================================================================
void isr_spi_cs_changed() {
    int cs_val = digitalReadFast(PIN_SPI_CS);

    if (cs_val == LOW) {
        // Master asserted CS LOW: Begin SPI frame transmission
        g_spi_transfer_in_progress = true;
        const uint8_t* p_bytes = (const uint8_t*)&g_tx_packet;
        size_t total_len = sizeof(PondEyesSpiPacket_t);

        // Hardware bit-bang / high-speed synchronous transfer clocked by Master SCK
        for (size_t b = 0; b < total_len; b++) {
            uint8_t out_byte = p_bytes[b];
            for (int bit = 7; bit >= 0; bit--) {
                // Wait for SCK LOW
                while (digitalReadFast(PIN_SPI_SCK) == HIGH && digitalReadFast(PIN_SPI_CS) == LOW);
                // Set MISO
                digitalWriteFast(PIN_SPI_MISO, (out_byte & (1 << bit)) ? HIGH : LOW);
                // Wait for SCK HIGH
                while (digitalReadFast(PIN_SPI_SCK) == LOW && digitalReadFast(PIN_SPI_CS) == LOW);
            }
        }
    } else {
        // Master released CS HIGH: Transfer complete
        g_spi_transfer_in_progress = false;
        digitalWriteFast(PIN_SPI_DRDY, LOW);
        digitalWriteFast(PIN_SPI_MISO, LOW);
    }
}
