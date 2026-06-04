#include "config_store.h"
#include "settings.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

namespace {
const char* kTag = "config_store";
const char* kNamespace = "canbridge";
const char* kKeyBitrate = "bitrate";
}  // namespace

namespace config_store {

bool begin() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

uint32_t loadBitrate() {
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return settings::kDefaultBitrate;
    }
    uint32_t value = settings::kDefaultBitrate;
    esp_err_t err = nvs_get_u32(handle, kKeyBitrate, &value);
    nvs_close(handle);
    if (err != ESP_OK) return settings::kDefaultBitrate;
    return settings::sanitizeBitrate(value);
}

bool saveBitrate(uint32_t bitrate) {
    bitrate = settings::sanitizeBitrate(bitrate);
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    bool ok = (nvs_set_u32(handle, kKeyBitrate, bitrate) == ESP_OK) &&
              (nvs_commit(handle) == ESP_OK);
    nvs_close(handle);
    return ok;
}

}  // namespace config_store
