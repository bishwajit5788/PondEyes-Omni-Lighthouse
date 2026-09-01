/**
 * ============================================================================
 * PondEyes Omni-Lighthouse - Master Coordinator & Broadcaster Firmware
 * Microcontroller: Seeed Studio XIAO ESP32-S3 (240 MHz Dual-Core Xtensa LX7)
 * ============================================================================
 */

#include <Arduino.h>
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
#include "pondeyes_protocol.h"

// ============================================================================
// HARDWARE PIN DEFINITIONS (Seeed Studio XIAO ESP32-S3)
// ============================================================================
#define PIN_SPI_SCK                 7   // D8 (GPIO 7) - SPI Clock Out
#define PIN_SPI_MISO                8   // D9 (GPIO 8) - SPI MISO In (from Teensy)
#define PIN_SPI_MOSI                9   // D10 (GPIO 9) - SPI MOSI Out (to Teensy)

#define PIN_CS_NODE_A               2   // D1 (GPIO 2) - Chip Select for Teensy Node A
#define PIN_CS_NODE_B               3   // D2 (GPIO 3) - Chip Select for Teensy Node B

#define PIN_DRDY_NODE_A             4   // D3 (GPIO 4) - Data Ready Interrupt from Node A
#define PIN_DRDY_NODE_B             5   // D4 (GPIO 5) - Data Ready Interrupt from Node B

#define PIN_STATUS_LED              21  // Yellow User LED on XIAO ESP32-S3

// Watchdog Timeout (Seconds)
#define WDT_TIMEOUT_SECONDS         8

// ============================================================================
// NETWORK & MQTT CONFIGURATION
// ============================================================================
const char* WIFI_SSID               = "PondEyes_Field_AP";
const char* WIFI_PASSWORD           = "OmniLighthouse2026";

// MQTT Broker Configuration
const char* MQTT_BROKER_HOST        = "192.168.1.100";
const uint16_t MQTT_BROKER_PORT     = 1883;
const char* MQTT_CLIENT_ID          = "PondEyes_Lighthouse_Coord";
const char* MQTT_USER               = "";
const char* MQTT_PASS               = "";

// MQTT Topic Definitions
const char* TOPIC_LIGHTHOUSE_BASE   = "PondEyes/lighthouse/";
const char* TOPIC_FUSED_SUMMARY     = "PondEyes/lighthouse/fused";
const char* TOPIC_HEARTBEAT         = "PondEyes/lighthouse/telemetry";

// Broadcast Timing (10 Hz = 100 ms period)
#define BROADCAST_INTERVAL_MS       100

// ============================================================================
// GLOBAL STATE & SYSTEM BUFFERS
// ============================================================================
static WiFiClient         g_wifi_client;
static PubSubClient       g_mqtt_client(g_wifi_client);

// SPI Raw Receive Buffers
static PondEyesSpiPacket_t g_packet_node_a;
static PondEyesSpiPacket_t g_packet_node_b;

// Cached Targets from last successful SPI read
static PondEyesTarget_t    g_latest_targets_a[MAX_TARGETS_PER_NODE];
static uint8_t             g_latest_count_a = 0;
static uint32_t            g_last_seen_node_a_ms = 0;

static PondEyesTarget_t    g_latest_targets_b[MAX_TARGETS_PER_NODE];
static uint8_t             g_latest_count_b = 0;
static uint32_t            g_last_seen_node_b_ms = 0;

// Interrupt Flags for Data-Ready signals
static volatile bool       g_drdy_flag_a = false;
static volatile bool       g_drdy_flag_b = false;

// Statistics & Diagnostics
static uint32_t            g_spi_success_a = 0;
static uint32_t            g_spi_crc_err_a = 0;
static uint32_t            g_spi_success_b = 0;
static uint32_t            g_spi_crc_err_b = 0;
static uint32_t            g_mqtt_pub_count = 0;
static uint32_t            g_last_broadcast_ms = 0;
static uint32_t            g_last_heartbeat_ms = 0;
static uint32_t            g_wifi_reconnect_timer = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void IRAM_ATTR isr_drdy_node_a();
void IRAM_ATTR isr_drdy_node_b();
void init_gpio_and_spi();
void init_watchdog();
void maintain_wifi_connection();
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
    Serial.println(F(" PondEyes Omni-Lighthouse: MASTER BROADCASTER (ESP32-S3)"));
    Serial.println(F(" Dual-Node SPI Master -> 10Hz MQTT Stream Engine"));
    Serial.println(F("========================================================"));

    // Initialize Hardware Pins, SPI Bus, and Interrupts
    init_gpio_and_spi();

    // Initialize Hardware Watchdog
    init_watchdog();

    // Configure MQTT Client Buffer Size for large JSON frames
    g_mqtt_client.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    g_mqtt_client.setBufferSize(2048);
    g_mqtt_client.setKeepAlive(15);

    // Initial Wi-Fi connection attempt
    maintain_wifi_connection();

    Serial.println(F("[Broadcaster] Initialization complete. Entering main coordinator loop."));
}

// ============================================================================
// INITIALIZE GPIO & HARDWARE SPI BUS
// ============================================================================
void init_gpio_and_spi() {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    // Configure Chip Select Lines
    pinMode(PIN_CS_NODE_A, OUTPUT);
    pinMode(PIN_CS_NODE_B, OUTPUT);
    digitalWrite(PIN_CS_NODE_A, HIGH); // Inactive High
    digitalWrite(PIN_CS_NODE_B, HIGH); // Inactive High

    // Configure Data-Ready Interrupt Inputs
    pinMode(PIN_DRDY_NODE_A, INPUT_PULLDOWN);
    pinMode(PIN_DRDY_NODE_B, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY_NODE_A), isr_drdy_node_a, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY_NODE_B), isr_drdy_node_b, RISING);

    // Initialize Hardware SPI Master Bus at 8 MHz
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    digitalWrite(PIN_STATUS_LED, LOW);
}

// ============================================================================
// INITIALIZE HARDWARE WATCHDOG TIMER
// ============================================================================
void init_watchdog() {
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();
}

// ============================================================================
// INTERRUPT SERVICE ROUTINES FOR DATA READY HANDSHAKES
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
    // 1. Kick hardware watchdog timer
    esp_task_wdt_reset();

    // 2. Maintain Network and Broker Connections
    maintain_wifi_connection();
    if (WiFi.status() == WL_CONNECTED) {
        maintain_mqtt_connection();
        g_mqtt_client.loop();
    }

    // 3. Service SPI Transactions from Teensy Node A when signaled or polled
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

    // 4. Service SPI Transactions from Teensy Node B when signaled or polled
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

    // 5. High-Precision 10 Hz (100ms) Periodic MQTT Publisher
    uint32_t now_ms = millis();
    if (now_ms - g_last_broadcast_ms >= BROADCAST_INTERVAL_MS) {
        g_last_broadcast_ms = now_ms;

        if (g_mqtt_client.connected()) {
            publish_sensor_payloads_10hz();
            publish_fused_summary_json();
            g_mqtt_pub_count++;
        }
    }

    // 6. Periodic System Telemetry Heartbeat (every 5000ms)
    if (now_ms - g_last_heartbeat_ms >= 5000) {
        g_last_heartbeat_ms = now_ms;
        publish_system_heartbeat();
    }
}

// ============================================================================
// HARDWARE SPI TRANSACTION HANDLER WITH CRC16 VERIFICATION
// ============================================================================
bool read_node_spi(uint8_t cs_pin, PondEyesSpiPacket_t* dest_packet) {
    size_t packet_size = sizeof(PondEyesSpiPacket_t);
    uint8_t* rx_ptr = (uint8_t*)dest_packet;

    // Begin SPI Transaction (8 MHz, MSB First, Mode 0)
    SPI.beginTransaction(SPISettings(PONDEYES_SPI_CLOCK_HZ, PONDEYES_SPI_BIT_ORDER, PONDEYES_SPI_DATA_MODE));
    
    // Assert CS LOW to initiate transfer
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(2); // Setup time

    // Transfer bytes over SPI
    for (size_t i = 0; i < packet_size; i++) {
        rx_ptr[i] = SPI.transfer(0x00);
    }

    // De-assert CS HIGH to complete transfer
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    // Verify Packet Magic and CRC16 Checksum
    return pondeyes_verify_packet_crc(dest_packet);
}

// ============================================================================
// ROBUST WI-FI CONNECTION HANDLER
// ============================================================================
void maintain_wifi_connection() {
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }

    uint32_t now = millis();
    if (now - g_wifi_reconnect_timer < 5000) {
        return; // Retry every 5s with non-blocking check
    }
    g_wifi_reconnect_timer = now;

    Serial.printf("[Wi-Fi] Connecting to SSID: %s ...\n", WIFI_SSID);
    digitalWrite(PIN_STATUS_LED, HIGH);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // Non-blocking quick check
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[Wi-Fi] Connected! IP: %s | RSSI: %d dBm\n", 
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        digitalWrite(PIN_STATUS_LED, LOW);
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
        return; // Non-blocking retry every 3s
    }
    last_mqtt_attempt = now;

    Serial.printf("[MQTT] Connecting to Broker: %s:%d ...\n", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    
    bool connected = false;
    if (strlen(MQTT_USER) > 0) {
        connected = g_mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
    } else {
        connected = g_mqtt_client.connect(MQTT_CLIENT_ID);
    }

    if (connected) {
        Serial.println(F("[MQTT] Broker Connected Successfully!"));
        digitalWrite(PIN_STATUS_LED, LOW);
    } else {
        Serial.printf("[MQTT] Connection Failed, rc=%d\n", g_mqtt_client.state());
        digitalWrite(PIN_STATUS_LED, HIGH);
    }
}

// ============================================================================
// PUBLISH INDIVIDUAL SENSOR TOPICS ("PondEyes/lighthouse/S1"..."S6") AT 10 HZ
// ============================================================================
void publish_sensor_payloads_10hz() {
    StaticJsonDocument<1024> doc;
    char topic_buffer[64];
    char json_buffer[1024];

    // Combine targets from both nodes
    PondEyesTarget_t all_targets[MAX_TARGETS_PER_NODE * 2];
    uint8_t total_count = 0;

    for (uint8_t i = 0; i < g_latest_count_a && total_count < 18; i++) {
        all_targets[total_count++] = g_latest_targets_a[i];
    }
    for (uint8_t i = 0; i < g_latest_count_b && total_count < 18; i++) {
        all_targets[total_count++] = g_latest_targets_b[i];
    }

    // Publish individually for each sensor 1 through 6
    float sensor_angles[6] = { 0.0f, 60.0f, 120.0f, 180.0f, 240.0f, 300.0f };

    for (uint8_t s = 1; s <= 6; s++) {
        doc.clear();
        doc["node_id"]      = (s <= 3) ? "NODE_A" : "NODE_B";
        doc["sensor_id"]    = s;
        doc["angle_deg"]    = sensor_angles[s - 1];
        doc["timestamp_ms"] = millis();

        JsonArray targets_arr = doc.createNestedArray("targets");
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

        doc["target_count"] = sensor_target_count;

        // Serialize and publish
        snprintf(topic_buffer, sizeof(topic_buffer), "%sS%u", TOPIC_LIGHTHOUSE_BASE, s);
        size_t len = serializeJson(doc, json_buffer, sizeof(json_buffer));
        g_mqtt_client.publish(topic_buffer, (const uint8_t*)json_buffer, len, false);
    }
}

// ============================================================================
// PUBLISH AGGREGATED 360-DEGREE FUSED TARGET OVERVIEW TOPIC
// ============================================================================
void publish_fused_summary_json() {
    StaticJsonDocument<2048> doc;
    char json_buffer[2048];

    doc["system"]       = "PondEyes Omni-Lighthouse";
    doc["timestamp_ms"] = millis();
    doc["rate_hz"]      = 10;

    JsonArray targets_arr = doc.createNestedArray("fused_targets");
    uint8_t count = 0;

    // Node A Targets
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

    // Node B Targets
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

    doc["total_active_targets"] = count;

    size_t len = serializeJson(doc, json_buffer, sizeof(json_buffer));
    g_mqtt_client.publish(TOPIC_FUSED_SUMMARY, (const uint8_t*)json_buffer, len, false);
}

// ============================================================================
// PUBLISH SYSTEM HEALTH & TELEMETRY HEARTBEAT
// ============================================================================
void publish_system_heartbeat() {
    StaticJsonDocument<512> doc;
    char json_buffer[512];

    doc["device"]          = "XIAO_ESP32S3_Coordinator";
    doc["uptime_ms"]       = millis();
    doc["wifi_rssi_dbm"]   = WiFi.RSSI();
    doc["wifi_ip"]         = WiFi.localIP().toString();
    doc["free_heap_bytes"] = ESP.getFreeHeap();
    
    JsonObject stats_a = doc.createNestedObject("node_a");
    stats_a["spi_ok"]      = g_spi_success_a;
    stats_a["crc_err"]     = g_spi_crc_err_a;
    stats_a["last_seen_ms"]= millis() - g_last_seen_node_a_ms;

    JsonObject stats_b = doc.createNestedObject("node_b");
    stats_b["spi_ok"]      = g_spi_success_b;
    stats_b["crc_err"]     = g_spi_crc_err_b;
    stats_b["last_seen_ms"]= millis() - g_last_seen_node_b_ms;

    doc["total_mqtt_pub"]  = g_mqtt_pub_count;

    size_t len = serializeJson(doc, json_buffer, sizeof(json_buffer));
    g_mqtt_client.publish(TOPIC_HEARTBEAT, (const uint8_t*)json_buffer, len, false);

    Serial.printf("[Heartbeat] Uptime: %lu ms | Heap: %u bytes | RSSI: %d dBm | NodeA SPI: %lu (err: %lu) | NodeB SPI: %lu (err: %lu) | MQTT Pubs: %lu\n",
                  millis(), ESP.getFreeHeap(), WiFi.RSSI(), 
                  g_spi_success_a, g_spi_crc_err_a,
                  g_spi_success_b, g_spi_crc_err_b,
                  g_mqtt_pub_count);
}
