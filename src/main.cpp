#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

#include "can_frame.h"
#include "gvret.h"
#include "settings.h"
#include "config_store.h"
#include "can_driver.h"
#include "usb_serial.h"
#include "indicators.h"

// CAN RX/TX pins (to the transceiver). "D4"/"D5" on the DevKit.
int g_rxPin = 4;
int g_txPin = 5;

namespace {
const char* kTag = "main";

// Indicator pins.
constexpr int kCanLedPin = 26;
constexpr int kConnLedPin = 27;
constexpr int kButtonPin = 25;

constexpr uint32_t kUsbBaud = 1000000;
constexpr uint32_t kConnTimeoutMs = 4000;  // no host commands => disconnected
constexpr int kRxQueueDepth = 256;

QueueHandle_t g_rxQueue = nullptr;        // CanFrame from CAN task -> comms task
volatile uint32_t g_lastCmdMs = 0;        // last host command time
volatile uint32_t g_dropped = 0;          // overflow counter

uint32_t millis32() { return (uint32_t)(esp_timer_get_time() / 1000); }

// Handler the GVRET parser calls. Runs in the comms task context.
class BridgeHandler : public gvret::Handler {
public:
    void onTransmitFrame(const CanFrame& f) override {
        can_driver::transmit(f, 10);
        indicators::noteCanActivity();
    }
    void onSetBitrate(uint8_t bus, uint32_t bitrate, bool enabled,
                      bool listenOnly) override {
        if (bus != 0) return;
        uint32_t br = settings::sanitizeBitrate(bitrate);
        if (!enabled) { can_driver::stop(); return; }
        can_driver::restart(br, listenOnly);
        config_store::saveBitrate(br);
    }
    uint32_t currentBitrate(uint8_t) override { return can_driver::bitrate(); }
    bool busEnabled(uint8_t) override { return can_driver::enabled(); }
    uint32_t nowMicros() override { return (uint32_t)esp_timer_get_time(); }
    void writeBytes(const uint8_t* data, size_t len) override {
        usb_serial::write(data, len);
    }
    void onHostCommand() override { g_lastCmdMs = millis32(); }
};

// CAN task: block on receive, push frames to the queue (drop-oldest on full).
void canTask(void*) {
    CanFrame f;
    for (;;) {
        if (can_driver::receive(f, 50)) {
            if (xQueueSend(g_rxQueue, &f, 0) != pdTRUE) {
                CanFrame discard;
                xQueueReceive(g_rxQueue, &discard, 0);  // drop oldest
                xQueueSend(g_rxQueue, &f, 0);
                g_dropped++;
            }
        }
    }
}

// Comms task: drain queue -> encode -> USB; read USB -> parse; manage state.
void commsTask(void*) {
    BridgeHandler handler;
    gvret::Parser parser(handler);

    esp_task_wdt_add(nullptr);

    uint8_t rxBytes[512];
    uint8_t encoded[gvret::kMaxEncodedFrame];
    CanFrame frame;
    bool wasConnected = false;

    for (;;) {
        esp_task_wdt_reset();

        // 1) Outbound: drain CAN frames to the host.
        int drained = 0;
        while (drained < 64 && xQueueReceive(g_rxQueue, &frame, 0) == pdTRUE) {
            size_t n = gvret::encodeFrame(frame, encoded, sizeof(encoded));
            if (n) usb_serial::write(encoded, n);
            indicators::noteCanActivity();
            drained++;
        }

        // 2) Inbound: feed host bytes to the parser.
        size_t got = usb_serial::read(rxBytes, sizeof(rxBytes));
        for (size_t i = 0; i < got; ++i) parser.feed(rxBytes[i]);

        // 3) Connection state.
        bool connected =
            (millis32() - g_lastCmdMs) < kConnTimeoutMs && g_lastCmdMs != 0;
        if (connected != wasConnected) {
            indicators::setConnected(connected);
            wasConnected = connected;
        }

        // 4) CAN error/recovery state.
        bool errorActive = false;
        can_driver::serviceAlerts(errorActive);
        indicators::setError(errorActive);

        if (drained == 0 && got == 0) vTaskDelay(pdMS_TO_TICKS(2));
    }
}

}  // namespace

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);  // keep UART0 clean for GVRET data

    config_store::begin();
    uint32_t bitrate = config_store::loadBitrate();

    usb_serial::begin(kUsbBaud);
    indicators::begin(kCanLedPin, kConnLedPin, kButtonPin);
    can_driver::start(bitrate, g_rxPin, g_txPin, /*listenOnly=*/false);

    g_rxQueue = xQueueCreate(kRxQueueDepth, sizeof(CanFrame));

    xTaskCreatePinnedToCore(canTask, "can", 4096, nullptr, 10, nullptr, 1);
    xTaskCreatePinnedToCore(commsTask, "comms", 8192, nullptr, 9, nullptr, 0);

    // Main loop: indicator updates + button handling.
    for (;;) {
        uint32_t now = millis32();
        indicators::update(now);
        switch (indicators::pollButton(now)) {
            case indicators::Button::ShortPress:
                ESP_LOGI(kTag, "short press: reset connectivity");
                can_driver::restart(can_driver::bitrate(),
                                    can_driver::listenOnly());
                g_lastCmdMs = 0;  // force disconnected until host re-handshakes
                break;
            case indicators::Button::LongPress:
                ESP_LOGI(kTag, "long press: reboot");
                esp_restart();
                break;
            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
