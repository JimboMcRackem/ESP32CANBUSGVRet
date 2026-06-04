#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void) {
    printf("esp32 canbus gvret bridge: boot\n");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
