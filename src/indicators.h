#pragma once
#include <cstdint>

namespace indicators {

enum class Button { None, ShortPress, LongPress };

// Configure GPIO26 (CAN activity), GPIO27 (connected), GPIO25 (button input).
void begin(int canLedPin, int connLedPin, int buttonPin);

// Record CAN traffic; triggers a brief activity flicker.
void noteCanActivity();

// SavvyCAN client connection state (drives GPIO27).
void setConnected(bool connected);

// CAN error/bus-off state (fast-blinks the activity LED).
void setError(bool error);

// Drive LED outputs based on current state. Call frequently with a ms clock.
void update(uint32_t nowMs);

// Debounced button poll; returns a press event once per gesture.
Button pollButton(uint32_t nowMs);

}  // namespace indicators
