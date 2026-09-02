/**
 * ============================================================================
 * PondEyes Omni-Lighthouse - Master Coordinator & Broadcaster Firmware
 * Microcontroller: Seeed Studio XIAO ESP32-S3 (240 MHz Dual-Core Xtensa LX7)
 * File: src/xiao_broadcaster.cpp
 * 
 * Hardware Architecture & Fixes:
 *   1. High-Speed SPI Master Engine (8 MHz) polling Node A and Node B
 *   2. Dual DRDY hardware interrupt handlers with volatile flags
 *   3. Zero-Heap-Leak Static Memory Architecture:
 *      - Replaces dynamic heap allocation with static buffer StaticJsonDocument<2048>
 *      - Optimized for up to 18 live targets without heap fragmentation
 *   4. Exponential Backoff Wi-Fi Reconnection State Machine
 *   5. Hardware Watchdog Timer Protection (esp_task_wdt)
 *   6. 10 Hz Dynamic JSON Serializer streaming to "PondEyes/lighthouse/fused"
 * ============================================================================
 */

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif
#if __has_include(<HWCDC.h>)
#include <HWCDC.h>
#endif
#if __has_include(<HardwareSerial.h>)
#include <HardwareSerial.h>
#endif
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#if __has_include(<esp_attr.h>)
#include <esp_attr.h>
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// Arduino Core Timing & Utility Prototypes (C-linkage for IDE / Clangd / Toolchains)
#ifdef __cplusplus
extern "C" {
#endif
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
unsigned long millis(void);
unsigned long micros(void);
void yield(void);
#ifdef __cplusplus
}
#endif

#if __has_include("../include/pondeyes_protocol.h")
#include "../include/pondeyes_protocol.h"
#elif __has_include("include/pondeyes_protocol.h")
#include "include/pondeyes_protocol.h"
#else
#include "pondeyes_protocol.h"
#endif

// ============================================================================
// HARDWARE PIN DEFINITIONS (Seeed Studio XIAO ESP32-S3)
// ============================================================================
#define PIN_SPI_SCK                 7   // D8 (GPIO 7) - SPI Clock Out (8 MHz)
#define PIN_SPI_MISO                8   // D9 (GPIO 8) - SPI MISO In (from Teensy nodes)
#define PIN_SPI_MOSI                9   // D10 (GPIO 9) - SPI MOSI Out (to Teensy nodes)

#define PIN_CS_NODE_A               2   // D1 (GPIO 2) - Chip Select for Teensy Node A
#define PIN_CS_NODE_B               3   // D2 (GPIO 3) - Chip Select for Teensy Node B

#define PIN_DRDY_NODE_A             4   // D3 (GPIO 4) - Data Ready Interrupt from Node A
#define PIN_DRDY_NODE_B             5   // D4 (GPIO 5) - Data Ready Interrupt from Node B

#define PIN_STATUS_LED              21  // Yellow User LED on XIAO ESP32-S3
#define WDT_TIMEOUT_SECONDS         8   // Hardware Watchdog Period

// ============================================================================
// NETWORK & MQTT CONFIGURATION
// ============================================================================
const char* WIFI_SSID               = "PondEyes_Field_AP";
const char* WIFI_PASSWORD           = "OmniLighthouse2026";

const char* MQTT_BROKER_HOST        = "192.168.1.100";
const uint16_t MQTT_BROKER_PORT     = 1883;
const char* MQTT_CLIENT_ID          = "PondEyes_Lighthouse_Coord";
const char* MQTT_USER               = "";
const char* MQTT_PASS               = "";

// MQTT Topics
const char* TOPIC_LIGHTHOUSE_BASE   = "PondEyes/lighthouse/";
const char* TOPIC_FUSED_SUMMARY     = "PondEyes/lighthouse/fused";
const char* TOPIC_HEARTBEAT         = "PondEyes/lighthouse/telemetry";

#define BROADCAST_INTERVAL_MS       100 // 10 Hz rate

// ============================================================================
// STATIC MEMORY ALLOCATION (ZERO HEAP FRAGMENTATION)
// ============================================================================
static WiFiClient           g_wifi_client;
static PubSubClient         g_mqtt_client(g_wifi_client);

// Static JSON document buffer block - preallocated 2048 bytes
static StaticJsonDocument<2048> g_static_json_doc;
static char                 g_mqtt_payload_buffer[2048];

// SPI Raw Packets
static PondEyesSpiPacket_t  g_packet_node_a;
static PondEyesSpiPacket_t  g_packet_node_b;

// Cached Targets from last successful read
static PondEyesTarget_t     g_latest_targets_a[MAX_TARGETS_PER_NODE];
static uint8_t              g_latest_count_a = 0;
static uint32_t             g_last_seen_node_a_ms = 0;

static PondEyesTarget_t     g_latest_targets_b[MAX_TARGETS_PER_NODE];
static uint8_t              g_latest_count_b = 0;
static uint32_t             g_last_seen_node_b_ms = 0;

// Interrupt Flags
static volatile bool        g_drdy_flag_a = false;
static volatile bool        g_drdy_flag_b = false;

// Diagnostics & Timers
static uint32_t             g_spi_success_a = 0;
static uint32_t             g_spi_crc_err_a = 0;
static uint32_t             g_spi_success_b = 0;
static uint32_t             g_spi_crc_err_b = 0;
static uint32_t             g_mqtt_pub_count = 0;
static uint32_t             g_last_broadcast_ms = 0;
static uint32_t             g_last_heartbeat_ms = 0;

// Wi-Fi Exponential Backoff State
static uint32_t             g_wifi_backoff_ms = 1000;
static uint32_t             g_last_wifi_attempt_ms = 0;
static const uint32_t       WIFI_MAX_BACKOFF_MS = 30000;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void IRAM_ATTR isr_drdy_node_a();
void IRAM_ATTR isr_drdy_node_b();
void init_gpio_and_spi();
void init_watchdog();
void maintain_wifi_exponential_backoff();
void maintain_mqtt_connection();
bool read_node_spi(uint8_t cs_pin, PondEyesSpiPacket_t* dest_packet);
void publish_sensor_payloads_10hz();
void publish_fused_summary_json();
void publish_system_heartbeat();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n========================================================"));
    Serial.println(F(" PondEyes Omni-Lighthouse: MASTER COORDINATOR (ESP32-S3)"));
    Serial.println(F(" Dual SPI Master -> 10Hz Static-Memory MQTT Broadcaster"));
    Serial.println(F("========================================================"));

    init_gpio_and_spi();
    init_watchdog();

    g_mqtt_client.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    g_mqtt_client.setBufferSize(2048);
    g_mqtt_client.setKeepAlive(15);

    maintain_wifi_exponential_backoff();

    Serial.println(F("[Broadcaster] Initialization complete. Entering high-reliability loop."));
}

// ============================================================================
// GPIO & SPI INITIALIZATION
// ============================================================================
void init_gpio_and_spi() {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    pinMode(PIN_CS_NODE_A, OUTPUT);
    pinMode(PIN_CS_NODE_B, OUTPUT);
    digitalWrite(PIN_CS_NODE_A, HIGH); // Inactive High
    digitalWrite(PIN_CS_NODE_B, HIGH); // Inactive High

    pinMode(PIN_DRDY_NODE_A, INPUT_PULLDOWN);
    pinMode(PIN_DRDY_NODE_B, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY_NODE_A), isr_drdy_node_a, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY_NODE_B), isr_drdy_node_b, RISING);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    digitalWrite(PIN_STATUS_LED, LOW);
}

// ============================================================================
// WATCHDOG TIMER INITIALIZATION
// ============================================================================
void init_watchdog() {
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();
}

// ============================================================================
// DATA READY ISRs
// ============================================================================
void IRAM_ATTR isr_drdy_node_a() {
    g_drdy_flag_a = true;
}

void IRAM_ATTR isr_drdy_node_b() {
    g_drdy_flag_b = true;
}

// ============================================================================
// MAIN COORDINATOR LOOP
// ============================================================================
void loop() {
    esp_task_wdt_reset();

    // 1. Maintain Network & MQTT
    maintain_wifi_exponential_backoff();
    if (WiFi.status() == WL_CONNECTED) {
        maintain_mqtt_connection();
        g_mqtt_client.loop();
    }

    // 2. Poll Node A over SPI when signaled
    if (g_drdy_flag_a || digitalRead(PIN_DRDY_NODE_A) == HIGH) {
        g_drdy_flag_a = false;
        if (read_node_spi(PIN_CS_NODE_A, &g_packet_node_a)) {
            g_latest_count_a = g_packet_node_a.target_count;
            for (uint8_t i = 0; i < g_latest_count_a; i++) {
                g_latest_targets_a[i] = g_packet_node_a.targets[i];
            }
            g_last_seen_node_a_ms = millis();
            g_spi_success_a++;
        } else {
            g_spi_crc_err_a++;
        }
    }

    // 3. Poll Node B over SPI when signaled
    if (g_drdy_flag_b || digitalRead(PIN_DRDY_NODE_B) == HIGH) {
        g_drdy_flag_b = false;
        if (read_node_spi(PIN_CS_NODE_B, &g_packet_node_b)) {
            g_latest_count_b = g_packet_node_b.target_count;
            for (uint8_t i = 0; i < g_latest_count_b; i++) {
                g_latest_targets_b[i] = g_packet_node_b.targets[i];
            }
            g_last_seen_node_b_ms = millis();
            g_spi_success_b++;
        } else {
            g_spi_crc_err_b++;
        }
    }

    // 4. 10 Hz Periodic MQTT Broadcaster using Static Memory
    uint32_t now_ms = millis();
    if (now_ms - g_last_broadcast_ms >= BROADCAST_INTERVAL_MS) {
        g_last_broadcast_ms = now_ms;

        if (g_mqtt_client.connected()) {
            publish_sensor_payloads_10hz();
            publish_fused_summary_json();
            g_mqtt_pub_count++;
        }
    }

    // 5. System Health Telemetry Heartbeat (every 5000ms)
    if (now_ms - g_last_heartbeat_ms >= 5000) {
        g_last_heartbeat_ms = now_ms;
        publish_system_heartbeat();
    }
}

// ============================================================================
// HARDWARE SPI TRANSACTION (8 MHz with CRC16 Verification)
// ============================================================================
bool read_node_spi(uint8_t cs_pin, PondEyesSpiPacket_t* dest_packet) {
    size_t packet_size = sizeof(PondEyesSpiPacket_t);
    uint8_t* rx_ptr = (uint8_t*)dest_packet;

    SPI.beginTransaction(SPISettings(PONDEYES_SPI_CLOCK_HZ, PONDEYES_SPI_BIT_ORDER, PONDEYES_SPI_DATA_MODE));
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(2);

    for (size_t i = 0; i < packet_size; i++) {
        rx_ptr[i] = SPI.transfer(0x00);
    }

    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    return pondeyes_verify_packet_crc(dest_packet);
}

// ============================================================================
// EXPONENTIAL BACKOFF WI-FI MANAGER
// ============================================================================
void maintain_wifi_exponential_backoff() {
    if (WiFi.status() == WL_CONNECTED) {
        g_wifi_backoff_ms = 1000; // Reset backoff upon successful connection
        return;
    }

    uint32_t now = millis();
    if (now - g_last_wifi_attempt_ms < g_wifi_backoff_ms) {
        return;
    }
    g_last_wifi_attempt_ms = now;

    Serial.printf("[Wi-Fi] Connecting to %s (Backoff: %lu ms)...\n", WIFI_SSID, g_wifi_backoff_ms);
    digitalWrite(PIN_STATUS_LED, HIGH);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[Wi-Fi] Connected! IP: %s | RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        digitalWrite(PIN_STATUS_LED, LOW);
        g_wifi_backoff_ms = 1000;
    } else {
        // Double backoff duration up to maximum
        g_wifi_backoff_ms = min(g_wifi_backoff_ms * 2, WIFI_MAX_BACKOFF_MS);
    }
}

// ============================================================================
// ROBUST MQTT CONNECTION HANDLER
// ============================================================================
void maintain_mqtt_connection() {
    if (g_mqtt_client.connected()) {
        return;
    }

    static uint32_t last_mqtt_attempt = 0;
    uint32_t now = millis();
    if (now - last_mqtt_attempt < 3000) {
        return;
    }
    last_mqtt_attempt = now;

    Serial.printf("[MQTT] Connecting to Broker %s:%d ...\n", MQTT_BROKER_HOST, MQTT_BROKER_PORT);

    bool connected = false;
    if (strlen(MQTT_USER) > 0) {
        connected = g_mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
    } else {
        connected = g_mqtt_client.connect(MQTT_CLIENT_ID);
    }

    if (connected) {
        Serial.println(F("[MQTT] Broker Connected!"));
        digitalWrite(PIN_STATUS_LED, LOW);
    } else {
        Serial.printf("[MQTT] Connect failed, rc=%d\n", g_mqtt_client.state());
        digitalWrite(PIN_STATUS_LED, HIGH);
    }
}

// ============================================================================
// PUBLISH INDIVIDUAL SENSORS (STATIC JSON BUFFER - ZERO HEAP LEAKS)
// ============================================================================
void publish_sensor_payloads_10hz() {
    char topic_buffer[64];
    float sensor_angles[6] = { 0.0f, 60.0f, 120.0f, 180.0f, 240.0f, 300.0f };

    // Combine targets
    PondEyesTarget_t all_targets[MAX_TARGETS_PER_NODE * 2];
    uint8_t total_count = 0;

    for (uint8_t i = 0; i < g_latest_count_a && total_count < MAX_TOTAL_SYSTEM_TARGETS; i++) {
        all_targets[total_count++] = g_latest_targets_a[i];
    }
    for (uint8_t i = 0; i < g_latest_count_b && total_count < MAX_TOTAL_SYSTEM_TARGETS; i++) {
        all_targets[total_count++] = g_latest_targets_b[i];
    }

    for (uint8_t s = 1; s <= 6; s++) {
        g_static_json_doc.clear(); // Re-use static memory block
        g_static_json_doc["node_id"]      = (s <= 3) ? "NODE_A" : "NODE_B";
        g_static_json_doc["sensor_id"]    = s;
        g_static_json_doc["angle_deg"]    = sensor_angles[s - 1];
        g_static_json_doc["timestamp_ms"] = millis();

        JsonArray targets_arr = g_static_json_doc.createNestedArray("targets");
        uint8_t sensor_target_count = 0;

        for (uint8_t i = 0; i < total_count; i++) {
            if (all_targets[i].sensor_id == s) {
                sensor_target_count++;
                JsonObject t_obj = targets_arr.createNestedObject();
                t_obj["id"]          = all_targets[i].target_id;
                t_obj["x_global"]    = all_targets[i].x_global_mm;
                t_obj["y_global"]    = all_targets[i].y_global_mm;
                t_obj["x_local"]     = all_targets[i].x_local_mm;
                t_obj["y_local"]     = all_targets[i].y_local_mm;
                t_obj["speed_cms"]   = all_targets[i].speed_cms;
                t_obj["distance_mm"] = all_targets[i].distance_mm;
                t_obj["is_fused"]    = (bool)all_targets[i].is_fused;
                t_obj["source_mask"] = all_targets[i].source_mask;
            }
        }

        g_static_json_doc["target_count"] = sensor_target_count;

        snprintf(topic_buffer, sizeof(topic_buffer), "%sS%u", TOPIC_LIGHTHOUSE_BASE, s);
        size_t len = serializeJson(g_static_json_doc, g_mqtt_payload_buffer, sizeof(g_mqtt_payload_buffer));
        g_mqtt_client.publish(topic_buffer, (const uint8_t*)g_mqtt_payload_buffer, len, false);
    }
}

// ============================================================================
// PUBLISH AGGREGATED 360° FUSED TOPIC ("PondEyes/lighthouse/fused")
// ============================================================================
void publish_fused_summary_json() {
    g_static_json_doc.clear(); // Re-use static memory block

    g_static_json_doc["system"]       = "PondEyes Omni-Lighthouse";
    g_static_json_doc["timestamp_ms"] = millis();
    g_static_json_doc["rate_hz"]      = 10;

    JsonArray targets_arr = g_static_json_doc.createNestedArray("fused_targets");
    uint8_t count = 0;

    for (uint8_t i = 0; i < g_latest_count_a; i++) {
        JsonObject t = targets_arr.createNestedObject();
        t["sensor"]      = g_latest_targets_a[i].sensor_id;
        t["track_id"]    = g_latest_targets_a[i].target_id;
        t["x"]           = g_latest_targets_a[i].x_global_mm;
        t["y"]           = g_latest_targets_a[i].y_global_mm;
        t["speed"]       = g_latest_targets_a[i].speed_cms;
        t["distance"]    = g_latest_targets_a[i].distance_mm;
        t["fused"]       = (bool)g_latest_targets_a[i].is_fused;
        t["source_mask"] = g_latest_targets_a[i].source_mask;
        count++;
    }

    for (uint8_t i = 0; i < g_latest_count_b; i++) {
        JsonObject t = targets_arr.createNestedObject();
        t["sensor"]      = g_latest_targets_b[i].sensor_id;
        t["track_id"]    = g_latest_targets_b[i].target_id;
        t["x"]           = g_latest_targets_b[i].x_global_mm;
        t["y"]           = g_latest_targets_b[i].y_global_mm;
        t["speed"]       = g_latest_targets_b[i].speed_cms;
        t["distance"]    = g_latest_targets_b[i].distance_mm;
        t["fused"]       = (bool)g_latest_targets_b[i].is_fused;
        t["source_mask"] = g_latest_targets_b[i].source_mask;
        count++;
    }

    g_static_json_doc["total_active_targets"] = count;

    size_t len = serializeJson(g_static_json_doc, g_mqtt_payload_buffer, sizeof(g_mqtt_payload_buffer));
    g_mqtt_client.publish(TOPIC_FUSED_SUMMARY, (const uint8_t*)g_mqtt_payload_buffer, len, false);
}

// ============================================================================
// PUBLISH SYSTEM HEALTH TELEMETRY HEARTBEAT
// ============================================================================
void publish_system_heartbeat() {
    g_static_json_doc.clear();

    g_static_json_doc["device"]          = "XIAO_ESP32S3_Coordinator";
    g_static_json_doc["uptime_ms"]       = millis();
    g_static_json_doc["wifi_rssi_dbm"]   = WiFi.RSSI();
    g_static_json_doc["wifi_ip"]         = WiFi.localIP().toString();
    g_static_json_doc["free_heap_bytes"] = ESP.getFreeHeap();

    JsonObject stats_a = g_static_json_doc.createNestedObject("node_a");
    stats_a["spi_ok"]      = g_spi_success_a;
    stats_a["crc_err"]     = g_spi_crc_err_a;
    stats_a["last_seen_ms"]= millis() - g_last_seen_node_a_ms;

    JsonObject stats_b = g_static_json_doc.createNestedObject("node_b");
    stats_b["spi_ok"]      = g_spi_success_b;
    stats_b["crc_err"]     = g_spi_crc_err_b;
    stats_b["last_seen_ms"]= millis() - g_last_seen_node_b_ms;

    g_static_json_doc["total_mqtt_pub"]  = g_mqtt_pub_count;

    size_t len = serializeJson(g_static_json_doc, g_mqtt_payload_buffer, sizeof(g_mqtt_payload_buffer));
    g_mqtt_client.publish(TOPIC_HEARTBEAT, (const uint8_t*)g_mqtt_payload_buffer, len, false);
}
