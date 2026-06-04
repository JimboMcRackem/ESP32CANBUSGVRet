#include "usb_serial.h"
#include "driver/uart.h"

namespace {
constexpr uart_port_t kPort = UART_NUM_0;
constexpr int kRxBuf = 2048;
constexpr int kTxBuf = 4096;
}  // namespace

namespace usb_serial {

void begin(uint32_t baud) {
    uart_config_t cfg = {};
    cfg.baud_rate = (int)baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    uart_driver_install(kPort, kRxBuf, kTxBuf, 0, nullptr, 0);
    uart_param_config(kPort, &cfg);
    // UART0 default pins (TX0/RX0) route to the USB-UART bridge; no pin remap.
}

size_t read(uint8_t* buf, size_t maxLen) {
    int n = uart_read_bytes(kPort, buf, maxLen, 0);
    return n < 0 ? 0 : (size_t)n;
}

void write(const uint8_t* buf, size_t len) {
    uart_write_bytes(kPort, (const char*)buf, len);
}

}  // namespace usb_serial
