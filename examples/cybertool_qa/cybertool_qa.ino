/*
 * Cybertool QA Test Firmware for LILYGO T-2Can
 *
 * Transmits CAN frames every 250ms and checks for a valid Cybertool response.
 * When a working Cybertool is connected, the APA102 LED blinks ~4 times/sec.
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
 * Each valid response triggers one LED blink cycle.
 */

#include <Arduino.h>
#include "driver/twai.h"
#include "pin_config.h"

// APA102 LED pins
#define APA102_DATA  8
#define APA102_CLOCK 3

// How long after last valid response before we consider Cybertool disconnected
#define RESPONSE_TIMEOUT_MS 500

// Transmission cycle
#define CYCLE_PERIOD_MS  250
#define FRAME_GAP_MS      50

// LED blink: on for half the cycle, off for half
#define LED_ON_MS  125

// ── APA102 helpers ──────────────────────────────────────────────────────────

static void apa102_write_byte(uint8_t b) {
    for (int i = 7; i >= 0; i--) {
        digitalWrite(APA102_DATA, (b >> i) & 1);
        digitalWrite(APA102_CLOCK, HIGH);
        digitalWrite(APA102_CLOCK, LOW);
    }
}

static void apa102_set(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    // Start frame: 32 bits of 0
    for (int i = 0; i < 4; i++) apa102_write_byte(0x00);
    // LED frame: 111 + 5-bit brightness, then B, G, R
    apa102_write_byte(0xE0 | (brightness & 0x1F));
    apa102_write_byte(b);
    apa102_write_byte(g);
    apa102_write_byte(r);
    // End frame: 32 bits of 1
    for (int i = 0; i < 4; i++) apa102_write_byte(0xFF);
}

static void led_off() {
    apa102_set(0, 0, 0, 0);
}

static void led_on() {
    apa102_set(0, 255, 0, 10);  // green, moderate brightness
}

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
static bool led_is_on = false;
static unsigned long led_on_since = 0;

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
                Serial.println("RX: Valid Cybertool response on 0x3FD");
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
    pinMode(APA102_DATA, OUTPUT);
    pinMode(APA102_CLOCK, OUTPUT);
    led_off();

    // CAN init
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

    // ── LED: blink while Cybertool is connected ──
    bool cybertool_connected = (now - last_valid_response_ms) < RESPONSE_TIMEOUT_MS
                               && last_valid_response_ms != 0;

    if (cybertool_connected) {
        // Each valid response triggers led_on; it stays on for LED_ON_MS then off
        if (led_is_on && (now - led_on_since >= LED_ON_MS)) {
            led_off();
            led_is_on = false;
        }
        // Turn on when we get a new response (driven by last_valid_response_ms changing)
        if (!led_is_on && (now - last_valid_response_ms < 10)) {
            led_on();
            led_is_on = true;
            led_on_since = now;
        }
    } else {
        if (led_is_on) {
            led_off();
            led_is_on = false;
        }
    }
}
