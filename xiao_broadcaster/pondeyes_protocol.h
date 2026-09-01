/**
 * ============================================================================
 * PondEyes Omni-Lighthouse - Unified System Protocol & Shared Header
 * Project: 360-Degree Multi-Target Radar Lighthouse
 * Target Platforms: Teensy 4.1 (ARM Cortex-M7) & Seeed Studio XIAO ESP32-S3
 * ============================================================================
 * 
 * Defines high-efficiency binary data structures, endianness-safe packing,
 * sensor physical orientation constants, and CRC16 checksum validation for
 * low-latency SPI bus communications and 10Hz MQTT streaming.
 */

#ifndef PONDEYES_PROTOCOL_H
#define PONDEYES_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// PROTOCOL CONSTANTS & MAGIC HEADERS
// ============================================================================
#define PONDEYES_MAGIC_0            0x50  // 'P'
#define PONDEYES_MAGIC_1            0x45  // 'E'
#define PONDEYES_PROTOCOL_VERSION   0x01

#define NODE_ID_A                   0x0A  // Teensy Node A (Sensors 1, 2, 3)
#define NODE_ID_B                   0x0B  // Teensy Node B (Sensors 4, 5, 6)
#define NODE_ID_COORDINATOR         0x0C  // XIAO ESP32-S3 Master Coordinator

#define MAX_TARGETS_PER_SENSOR      3
#define SENSORS_PER_TEENSY          3
#define MAX_TARGETS_PER_NODE        (MAX_TARGETS_PER_SENSOR * SENSORS_PER_TEENSY) // 9 max
#define MAX_TOTAL_SYSTEM_TARGETS    18    // 6 sensors * 3 targets

// HLK-LD2450 Radar Constants
#define LD2450_UART_BAUD            256000
#define LD2450_FRAME_HEAD_0         0xAA
#define LD2450_FRAME_HEAD_1         0xFF
#define LD2450_FRAME_HEAD_2         0x03
#define LD2450_FRAME_HEAD_3         0x00
#define LD2450_FRAME_TAIL_0         0x55
#define LD2450_FRAME_TAIL_1         0xCC
#define LD2450_FRAME_LEN            30    // 4 header + 24 (3*8) payload + 2 tail

// Geometric & Arena Defaults (in millimeters)
#define ARENA_GRID_CENTER_X_MM      5000.0f
#define ARENA_GRID_CENTER_Y_MM      5000.0f
#define LIGHTHOUSE_RADIUS_MM        65.0f   // Distance from center to sensor antenna face
#define SEAM_DEDUP_DISTANCE_MM      500.0f  // Euclidean distance threshold for fusing twin ghost targets
#define SEAM_DEDUP_DIST_SQ_MM       (SEAM_DEDUP_DISTANCE_MM * SEAM_DEDUP_DISTANCE_MM) // 250,000 mm^2

// Sensor Angular Orientations (Degrees)
#define SENSOR_1_ANGLE_DEG          0.0f    // Node A - S1: North
#define SENSOR_2_ANGLE_DEG          60.0f   // Node A - S2: East-North-East
#define SENSOR_3_ANGLE_DEG          120.0f  // Node A - S3: East-South-East
#define SENSOR_4_ANGLE_DEG          180.0f  // Node B - S4: South
#define SENSOR_5_ANGLE_DEG          240.0f  // Node B - S5: West-South-West
#define SENSOR_6_ANGLE_DEG          300.0f  // Node B - S6: West-North-West

// SPI Bus Configuration
#define PONDEYES_SPI_CLOCK_HZ       8000000 // 8 MHz hardware SPI transfer
#define PONDEYES_SPI_BIT_ORDER      MSBFIRST
#define PONDEYES_SPI_DATA_MODE      SPI_MODE0

// ============================================================================
// PACKED DATA STRUCTURES (Zero-padding, Byte-aligned across ARM & Xtensa)
// ============================================================================
#pragma pack(push, 1)

/**
 * @brief Raw target data extracted directly from HLK-LD2450 UART frame.
 */
typedef struct {
    int16_t x_mm;             // Relative X coordinate (-6000 to +6000 mm, sign-magnitude)
    int16_t y_mm;             // Relative Y coordinate (0 to +6000 mm, bore-sight distance)
    int16_t speed_cms;        // Radial velocity (-500 to +500 cm/s, + = moving away)
    uint16_t distance_res_mm; // Target distance resolution / measured distance in mm
    uint8_t valid;            // 1 if target is active and tracked, 0 if noise/empty
} LD2450_RawTarget_t;

/**
 * @brief Processed target coordinate with global grid mapping and fusion flags.
 */
typedef struct {
    uint8_t  target_id;       // Target track slot (1, 2, 3)
    uint8_t  sensor_id;       // Sensor origin (1..6)
    int16_t  x_global_mm;     // Unified arena grid X [0..10000 mm]
    int16_t  y_global_mm;     // Unified arena grid Y [0..10000 mm]
    int16_t  speed_cms;       // Target radial speed in cm/s
    uint16_t distance_mm;    // Distance from sensor face in mm
    int16_t  x_local_mm;      // Local relative X in mm
    int16_t  y_local_mm;      // Local relative Y in mm
    uint8_t  is_fused;        // 1 if target is a fused multi-sensor coordinate, 0 otherwise
    uint8_t  source_mask;     // Bitmask of contributing sensors (e.g. 0b00000011 for S1+S2)
} PondEyesTarget_t;

/**
 * @brief High-speed binary transmission packet exchanged over SPI.
 * Total size: 2 (magic) + 1 (node_id) + 2 (sequence) + 4 (timestamp) + 1 (count) 
 *             + 9 * 16 (targets) + 2 (crc16) = 156 bytes.
 */
typedef struct {
    uint8_t          magic[2];       // PONDEYES_MAGIC_0 (0x50), PONDEYES_MAGIC_1 (0x45)
    uint8_t          node_id;        // NODE_ID_A or NODE_ID_B
    uint16_t         sequence;       // Monotonically increasing packet counter
    uint32_t         timestamp_ms;   // Node uptime in milliseconds
    uint8_t          target_count;   // Count of valid targets contained in targets array (0..9)
    PondEyesTarget_t targets[MAX_TARGETS_PER_NODE]; // Array of active targets
    uint16_t         crc16;          // CRC16-CCITT checksum over all previous bytes
} PondEyesSpiPacket_t;

#pragma pack(pop)

// ============================================================================
// CRC16-CCITT CHECKSUM IMPLEMENTATION (Polynomial: 0x1021, Init: 0xFFFF)
// ============================================================================

/**
 * @brief Calculate CRC16-CCITT for packet integrity verification.
 * @param data Pointer to byte buffer.
 * @param length Number of bytes to compute.
 * @return Computed 16-bit CRC checksum.
 */
static inline uint16_t pondeyes_calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Validate packet CRC16 checksum.
 * @param packet Pointer to received packet.
 * @return True if valid, false if corrupted.
 */
static inline bool pondeyes_verify_packet_crc(const PondEyesSpiPacket_t *packet) {
    if (packet->magic[0] != PONDEYES_MAGIC_0 || packet->magic[1] != PONDEYES_MAGIC_1) {
        return false;
    }
    size_t payload_len = offsetof(PondEyesSpiPacket_t, crc16);
    uint16_t calculated = pondeyes_calculate_crc16((const uint8_t *)packet, payload_len);
    return (calculated == packet->crc16);
}

/**
 * @brief Sign and seal packet with calculated CRC16.
 * @param packet Pointer to packet to be finalized.
 */
static inline void pondeyes_seal_packet_crc(PondEyesSpiPacket_t *packet) {
    packet->magic[0] = PONDEYES_MAGIC_0;
    packet->magic[1] = PONDEYES_MAGIC_1;
    size_t payload_len = offsetof(PondEyesSpiPacket_t, crc16);
    packet->crc16 = pondeyes_calculate_crc16((const uint8_t *)packet, payload_len);
}

#ifdef __cplusplus
}
#endif

#endif // PONDEYES_PROTOCOL_H
