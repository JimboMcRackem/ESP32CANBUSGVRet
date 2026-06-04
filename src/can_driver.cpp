#include "can_driver.h"
#include "driver/twai.h"
#include "esp_timer.h"
#include "esp_log.h"

namespace {
const char* kTag = "can_driver";
uint32_t g_bitrate = 500000;
bool g_enabled = false;
bool g_listenOnly = false;
bool g_recovering = false;

// Map a bitrate to a TWAI timing config. Falls back to 500k for unknown rates.
bool timingFor(uint32_t bitrate, twai_timing_config_t& out) {
    switch (bitrate) {
        case 1000000: out = TWAI_TIMING_CONFIG_1MBITS();   return true;
        case 800000:  out = TWAI_TIMING_CONFIG_800KBITS(); return true;
        case 500000:  out = TWAI_TIMING_CONFIG_500KBITS(); return true;
        case 250000:  out = TWAI_TIMING_CONFIG_250KBITS(); return true;
        case 125000:  out = TWAI_TIMING_CONFIG_125KBITS(); return true;
        case 100000:  out = TWAI_TIMING_CONFIG_100KBITS(); return true;
        case 50000:   out = TWAI_TIMING_CONFIG_50KBITS();  return true;
        case 25000:   out = TWAI_TIMING_CONFIG_25KBITS();  return true;
        default:      out = TWAI_TIMING_CONFIG_500KBITS(); return false;
    }
}
}  // namespace

// Defined in main.cpp; the CAN RX/TX pins to the transceiver.
extern int g_rxPin;
extern int g_txPin;

namespace can_driver {

bool start(uint32_t bitrate, int rxPin, int txPin, bool listenOnly) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)txPin, (gpio_num_t)rxPin,
        listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
    g.tx_queue_len = 32;
    g.rx_queue_len = 64;
    g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                       TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL;

    twai_timing_config_t t;
    timingFor(bitrate, t);
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        ESP_LOGE(kTag, "driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(kTag, "driver start failed");
        twai_driver_uninstall();
        return false;
    }
    g_bitrate = bitrate;
    g_enabled = true;
    g_listenOnly = listenOnly;
    g_recovering = false;
    ESP_LOGI(kTag, "TWAI started @ %u bps listenOnly=%d", bitrate, listenOnly);
    return true;
}

void stop() {
    twai_stop();
    twai_driver_uninstall();
    g_enabled = false;
}

bool restart(uint32_t bitrate, bool listenOnly) {
    if (g_enabled) stop();
    return start(bitrate, g_rxPin, g_txPin, listenOnly);
}

bool receive(CanFrame& out, uint32_t timeoutMs) {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) return false;
    out = CanFrame{};
    out.id = msg.identifier;
    out.extended = msg.extd != 0;
    out.rtr = msg.rtr != 0;
    out.dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
    for (int i = 0; i < out.dlc; ++i) out.data[i] = msg.data[i];
    out.timestamp_us = (uint32_t)esp_timer_get_time();
    out.bus = 0;
    return true;
}

bool transmit(const CanFrame& f, uint32_t timeoutMs) {
    if (!g_enabled || g_listenOnly) return false;
    twai_message_t msg = {};
    msg.identifier = f.id;
    msg.extd = f.extended ? 1 : 0;
    msg.rtr = f.rtr ? 1 : 0;
    msg.data_length_code = f.dlc > 8 ? 8 : f.dlc;
    for (int i = 0; i < msg.data_length_code; ++i) msg.data[i] = f.data[i];
    return twai_transmit(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

void serviceAlerts(bool& errorActive) {
    uint32_t alerts = 0;
    // Non-blocking poll.
    if (twai_read_alerts(&alerts, 0) != ESP_OK) {
        errorActive = g_recovering;
        return;
    }
    if (alerts & TWAI_ALERT_BUS_OFF) {
        ESP_LOGW(kTag, "bus-off detected, initiating recovery");
        twai_initiate_recovery();
        g_recovering = true;
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        ESP_LOGI(kTag, "bus recovered, restarting");
        twai_start();
        g_recovering = false;
    }
    errorActive = g_recovering || (alerts & TWAI_ALERT_ERR_PASS);
}

uint32_t bitrate() { return g_bitrate; }
bool enabled() { return g_enabled; }
bool listenOnly() { return g_listenOnly; }

}  // namespace can_driver
