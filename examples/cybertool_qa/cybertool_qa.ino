/*
 * Cybertool QA Test Firmware for LILYGO T-2Can
 *
 * Transmits CAN frames every 250ms and checks for a valid Cybertool response.
 * Prints PASS/FAIL status to serial every second.
 *
 * CAN bus used: ESP32-S3 internal TWAI (CAN-B) at 500 kbps.
 *
 * Transmitted frames (cycled every ~50ms within each 250ms window):
 *   0x405 - 10 00 00 00 00 30 30 30
 *   0x405 - 11 58 30 30 30 35 30 50
 *   0x405 - 12 30 30 30 30 30 30 30
 *   0x3F8 - 83 28 80 00 4A 99 05 10
 *   0x3FD - 01 00 0A 00 00 00 05 80
 *
 * We send 0x3FD with byte[2] bit 3 = 1 (0x0A).
 * A working Cybertool echoes 0x3FD back with byte[2] bit 3 = 0 (0x02).
 *
 * Status LED on GPIO 38: solid ON = PASS, OFF = no response.
 * Solder LED anode (+) to IO38 pad, cathode (-) to GND pad on the
 * 26-pin expansion header.
 */

#include <Arduino.h>
#include "driver/twai.h"
#include "pin_config.h"

// External status LED (active high)
#define LED_PIN 38

// How long after last valid response before we consider Cybertool disconnected
#define RESPONSE_TIMEOUT_MS 500

// Transmission cycle
#define CYCLE_PERIOD_MS  250
#define FRAME_GAP_MS      50

// Status print interval
#define STATUS_INTERVAL_MS 1000

// ── CAN frame definitions ───────────────────────────────────────────────────

typedef struct {
    uint32_t id;
    uint8_t data[8];
} tx_frame_def_t;

static const tx_frame_def_t TX_FRAMES[] = {
    { 0x405, { 0x10, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x30 } },
    { 0x405, { 0x11, 0x58, 0x30, 0x30, 0x30, 0x35, 0x30, 0x50 } },
    { 0x405, { 0x12, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30 } },
    { 0x3F8, { 0x83, 0x28, 0x80, 0x00, 0x4A, 0x99, 0x05, 0x10 } },
    { 0x3FD, { 0x01, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x05, 0x80 } },
};
static const int TX_FRAME_COUNT = sizeof(TX_FRAMES) / sizeof(TX_FRAMES[0]);

// ── State ───────────────────────────────────────────────────────────────────

static unsigned long last_valid_response_ms = 0;
static unsigned long last_status_print_ms = 0;
static bool was_connected = false;

static unsigned long next_cycle_ms = 0;
static int current_frame_idx = 0;
static unsigned long next_frame_ms = 0;
static bool cycle_in_progress = false;

// ── TWAI init ───────────────────────────────────────────────────────────────

static void twai_init() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("TWAI: Failed to install driver");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("TWAI: Failed to start driver");
        return;
    }

    uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_BUS_ERROR |
                      TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF;
    twai_reconfigure_alerts(alerts, NULL);

    Serial.println("TWAI: CAN-B initialized at 500 kbps");
}

// ── Send one CAN frame ─────────────────────────────────────────────────────

static void send_frame(int idx) {
    twai_message_t msg = {};
    msg.identifier = TX_FRAMES[idx].id;
    msg.data_length_code = 8;
    memcpy(msg.data, TX_FRAMES[idx].data, 8);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        Serial.printf("TX 0x%03X failed (%d)\n", TX_FRAMES[idx].id, err);
    }
}

// ── Check for valid Cybertool response ──────────────────────────────────────

static void check_rx() {
    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) {
        if (rx.identifier == 0x3FD && rx.data_length_code >= 3) {
            // Check byte[2] bit 3 is 0
            if ((rx.data[2] & 0x08) == 0) {
                last_valid_response_ms = millis();
            }
        }
    }
}

// ── Bus recovery ────────────────────────────────────────────────────────────

static void check_bus_health() {
    uint32_t alerts;
    twai_read_alerts(&alerts, 0);

    if (alerts & TWAI_ALERT_BUS_OFF) {
        Serial.println("TWAI: Bus off - initiating recovery");
        twai_initiate_recovery();
    }
}

// ── Arduino entry points ────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Cybertool QA Test Board ===");

    // LED init
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    twai_init();

    next_cycle_ms = millis();
    Serial.println("Ready. Waiting for Cybertool...");
}

void loop() {
    unsigned long now = millis();

    // ── TX: send frames in a staggered cycle ──
    if (!cycle_in_progress && now >= next_cycle_ms) {
        cycle_in_progress = true;
        current_frame_idx = 0;
        next_frame_ms = now;
    }

    if (cycle_in_progress && now >= next_frame_ms) {
        send_frame(current_frame_idx);
        current_frame_idx++;
        if (current_frame_idx >= TX_FRAME_COUNT) {
            cycle_in_progress = false;
            next_cycle_ms = now + CYCLE_PERIOD_MS;
        } else {
            next_frame_ms = now + FRAME_GAP_MS;
        }
    }

    // ── RX: check for Cybertool responses ──
    check_rx();
    check_bus_health();

    // ── Status: print PASS/FAIL to serial ──
    bool cybertool_connected = (now - last_valid_response_ms) < RESPONSE_TIMEOUT_MS
                               && last_valid_response_ms != 0;

    // LED: solid on when connected, off when not
    digitalWrite(LED_PIN, cybertool_connected ? HIGH : LOW);

    // Print immediately on state change
    if (cybertool_connected && !was_connected) {
        Serial.println(">>> PASS - Cybertool detected and responding <<<");
        last_status_print_ms = now;
    } else if (!cybertool_connected && was_connected) {
        Serial.println(">>> FAIL - Cybertool disconnected <<<");
        last_status_print_ms = now;
    }

    // Print periodic status every second
    if (now - last_status_print_ms >= STATUS_INTERVAL_MS) {
        if (cybertool_connected) {
            Serial.println("PASS");
        } else {
            Serial.println("FAIL - No Cybertool response");
        }
        last_status_print_ms = now;
    }

    was_connected = cybertool_connected;
}
