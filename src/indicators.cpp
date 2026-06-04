#include "indicators.h"
#include "driver/gpio.h"

namespace {
int g_canLed = 26, g_connLed = 27, g_button = 25;

// Activity LED
bool g_activityPending = false;
uint32_t g_activityUntilMs = 0;
bool g_error = false;
bool g_connected = false;

// Button debounce / press timing
constexpr uint32_t kDebounceMs = 30;
constexpr uint32_t kLongPressMs = 3000;
bool g_btnStableDown = false;
bool g_btnRaw = false;
uint32_t g_btnChangeMs = 0;
uint32_t g_btnDownMs = 0;
bool g_longFired = false;

void writeLed(int pin, bool on) { gpio_set_level((gpio_num_t)pin, on ? 1 : 0); }
}  // namespace

namespace indicators {

void begin(int canLedPin, int connLedPin, int buttonPin) {
    g_canLed = canLedPin; g_connLed = connLedPin; g_button = buttonPin;

    gpio_config_t out = {};
    out.mode = GPIO_MODE_OUTPUT;
    out.pin_bit_mask = (1ULL << g_canLed) | (1ULL << g_connLed);
    gpio_config(&out);

    gpio_config_t in = {};
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_ENABLE;   // button to GND, active-low
    in.pin_bit_mask = (1ULL << g_button);
    gpio_config(&in);

    writeLed(g_canLed, false);
    writeLed(g_connLed, false);
}

void noteCanActivity() { g_activityPending = true; }
void setConnected(bool connected) { g_connected = connected; }
void setError(bool error) { g_error = error; }

void update(uint32_t nowMs) {
    // Connection LED: solid when connected.
    writeLed(g_connLed, g_connected);

    // Activity LED: error => fast blink; else flicker ~40ms per frame burst.
    if (g_error) {
        writeLed(g_canLed, (nowMs / 100) % 2 == 0);
        return;
    }
    if (g_activityPending) {
        g_activityPending = false;
        g_activityUntilMs = nowMs + 40;
    }
    writeLed(g_canLed, nowMs < g_activityUntilMs);
}

Button pollButton(uint32_t nowMs) {
    bool raw = gpio_get_level((gpio_num_t)g_button) == 0;  // active-low
    if (raw != g_btnRaw) {
        g_btnRaw = raw;
        g_btnChangeMs = nowMs;
        return Button::None;
    }
    if ((nowMs - g_btnChangeMs) < kDebounceMs) return Button::None;

    // raw is now the debounced stable level
    if (raw && !g_btnStableDown) {       // press edge
        g_btnStableDown = true;
        g_btnDownMs = nowMs;
        g_longFired = false;
        return Button::None;
    }
    if (raw && g_btnStableDown && !g_longFired &&
        (nowMs - g_btnDownMs) >= kLongPressMs) {
        g_longFired = true;
        return Button::LongPress;        // fire while still held
    }
    if (!raw && g_btnStableDown) {       // release edge
        g_btnStableDown = false;
        if (!g_longFired) return Button::ShortPress;
    }
    return Button::None;
}

}  // namespace indicators
