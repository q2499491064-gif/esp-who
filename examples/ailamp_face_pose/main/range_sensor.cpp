#include "range_sensor.hpp"

#include <cinttypes>
#include <new>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "vl53l1x_driver.hpp"

namespace {

constexpr const char *TAG = "range";
constexpr i2c_port_num_t kI2cPort = I2C_NUM_0;
constexpr gpio_num_t kSdaGpio = GPIO_NUM_8;
constexpr gpio_num_t kSclGpio = GPIO_NUM_9;
constexpr gpio_num_t kXshutGpio = GPIO_NUM_10;
constexpr gpio_num_t kIntGpio = GPIO_NUM_21;
constexpr uint32_t kI2cFreqHz = 100000;
constexpr uint32_t kReadPeriodMs = 500;
constexpr uint32_t kResetAfterConsecutiveFails = 10;

SemaphoreHandle_t s_state_mutex = nullptr;
SemaphoreHandle_t s_driver_mutex = nullptr;
range_state_t s_state = {
    .initialized = false,
    .valid = false,
    .distance_mm = 0,
    .status = 255,
    .timestamp_ms = 0,
    .read_count = 0,
    .error_count = 0,
    .consecutive_ok_count = 0,
    .consecutive_fail_count = 0,
    .read_ms = 0,
    .status_text = "not_initialized",
};
i2c_master_bus_handle_t s_bus = nullptr;
Vl53l1xDriver *s_sensor = nullptr;
TaskHandle_t s_task = nullptr;
bool s_i2c_bus_created = false;

uint32_t now_ms()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void publish_state(const range_state_t &state)
{
    if (s_state_mutex == nullptr) {
        return;
    }
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_state = state;
    xSemaphoreGive(s_state_mutex);
}

range_state_t snapshot_state()
{
    range_state_t state = {};
    if (s_state_mutex != nullptr) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        state = s_state;
        xSemaphoreGive(s_state_mutex);
    }
    return state;
}

esp_err_t configure_xshut()
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << kXshutGpio;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&io_conf);
}

void reset_sensor_with_xshut()
{
    gpio_set_level(kXshutGpio, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(kXshutGpio, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

esp_err_t create_i2c_bus()
{
    if (s_i2c_bus_created) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = kI2cPort;
    bus_config.sda_io_num = kSdaGpio;
    bus_config.scl_io_num = kSclGpio;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus);
    if (err == ESP_OK) {
        s_i2c_bus_created = true;
    }
    return err;
}

bool scan_for_vl53l1x()
{
    ESP_LOGI(TAG, "I2C scan start on I2C_NUM_0 SDA=GPIO8 SCL=GPIO9 freq=%luHz INT=GPIO%u unused",
             static_cast<unsigned long>(kI2cFreqHz),
             static_cast<unsigned>(kIntGpio));
    esp_err_t err = i2c_master_probe(s_bus, Vl53l1xDriver::kDefaultAddress, 50);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "VL53L1X found at 0x%02x", Vl53l1xDriver::kDefaultAddress);
        return true;
    }
    ESP_LOGE(TAG, "VL53L1X not found at 0x%02x: %s", Vl53l1xDriver::kDefaultAddress, esp_err_to_name(err));
    return false;
}

void mark_init_failed(const char *reason)
{
    range_state_t state = snapshot_state();
    state.initialized = false;
    state.valid = false;
    state.distance_mm = 0;
    state.status = 255;
    state.timestamp_ms = now_ms();
    state.status_text = reason;
    publish_state(state);
}

bool initialize_sensor_device()
{
    if (s_sensor != nullptr) {
        delete s_sensor;
        s_sensor = nullptr;
    }

    s_sensor = new (std::nothrow) Vl53l1xDriver(s_bus);
    if (s_sensor == nullptr) {
        ESP_LOGE(TAG, "VL53L1X driver alloc failed");
        mark_init_failed("alloc_failed");
        return false;
    }

    uint16_t sensor_id = 0;
    esp_err_t err = s_sensor->read_sensor_id(&sensor_id);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "VL53L1X sensor_id=0x%04x", sensor_id);
    } else {
        ESP_LOGW(TAG, "read sensor id failed: %s", s_sensor->last_error());
    }

    err = s_sensor->init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X init failed: %s", s_sensor->last_error());
        mark_init_failed("init_failed");
        return false;
    }

    err = s_sensor->start_ranging();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X start ranging failed: %s", s_sensor->last_error());
        mark_init_failed("start_failed");
        return false;
    }

    range_state_t state = snapshot_state();
    state.initialized = true;
    state.valid = false;
    state.distance_mm = 0;
    state.status = 255;
    state.timestamp_ms = now_ms();
    state.status_text = "waiting";
    publish_state(state);
    return true;
}

void range_sensor_task(void *)
{
    int64_t last_log_us = 0;

    while (true) {
        uint16_t distance_mm = 0;
        uint8_t range_status = 255;
        const uint32_t read_start_ms = now_ms();

        xSemaphoreTake(s_driver_mutex, portMAX_DELAY);
        esp_err_t err = s_sensor != nullptr ? s_sensor->read_distance(&distance_mm, &range_status) : ESP_ERR_INVALID_STATE;
        const char *status_text =
            s_sensor != nullptr ? s_sensor->range_status_to_string(range_status) : "not_initialized";
        const char *error_text = s_sensor != nullptr ? s_sensor->last_error() : "not_initialized";
        xSemaphoreGive(s_driver_mutex);

        const uint32_t read_ms = now_ms() - read_start_ms;
        range_state_t state = snapshot_state();
        state.initialized = true;
        state.read_ms = read_ms;
        state.timestamp_ms = now_ms();
        state.read_count++;

        if (err == ESP_OK && range_status == 0) {
            state.valid = true;
            state.distance_mm = distance_mm;
            state.status = range_status;
            state.status_text = "ok";
            state.consecutive_ok_count++;
            state.consecutive_fail_count = 0;
        } else {
            state.valid = false;
            state.distance_mm = err == ESP_OK ? distance_mm : 0;
            state.status = range_status;
            state.status_text = err == ESP_OK ? status_text : error_text;
            state.error_count++;
            state.consecutive_fail_count++;
            state.consecutive_ok_count = 0;
        }
        publish_state(state);

        const int64_t now_us_value = esp_timer_get_time();
        if (now_us_value - last_log_us >= 1000000) {
            last_log_us = now_us_value;
            ESP_LOGI(TAG,
                     "[range] valid=%d distance=%umm status=%s read_ms=%" PRIu32 " err=%" PRIu32,
                     state.valid ? 1 : 0,
                     static_cast<unsigned>(state.distance_mm),
                     state.status_text != nullptr ? state.status_text : "unknown",
                     state.read_ms,
                     state.error_count);
        }

        if (state.consecutive_fail_count >= kResetAfterConsecutiveFails) {
            range_sensor_reset();
        }

        vTaskDelay(pdMS_TO_TICKS(kReadPeriodMs));
    }
}

} // namespace

bool range_sensor_init(void)
{
    if (s_state_mutex == nullptr) {
        s_state_mutex = xSemaphoreCreateMutex();
        if (s_state_mutex == nullptr) {
            ESP_LOGE(TAG, "range state mutex create failed");
            return false;
        }
    }
    if (s_driver_mutex == nullptr) {
        s_driver_mutex = xSemaphoreCreateMutex();
        if (s_driver_mutex == nullptr) {
            ESP_LOGE(TAG, "range driver mutex create failed");
            return false;
        }
    }

    esp_err_t err = configure_xshut();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "XSHUT GPIO config failed: %s", esp_err_to_name(err));
        mark_init_failed("xshut_config_failed");
        return false;
    }

    reset_sensor_with_xshut();

    err = create_i2c_bus();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C_NUM_0 init failed: %s", esp_err_to_name(err));
        mark_init_failed("i2c_init_failed");
        return false;
    }

    if (!scan_for_vl53l1x()) {
        mark_init_failed("not_found");
        return false;
    }

    xSemaphoreTake(s_driver_mutex, portMAX_DELAY);
    const bool ok = initialize_sensor_device();
    xSemaphoreGive(s_driver_mutex);
    return ok;
}

void range_sensor_start_task(void)
{
    if (s_task != nullptr) {
        return;
    }
    if (s_state_mutex == nullptr || s_driver_mutex == nullptr || s_sensor == nullptr) {
        ESP_LOGW(TAG, "range task not started: sensor not initialized");
        return;
    }

    BaseType_t ok = xTaskCreate(range_sensor_task, "range_sensor_task", 6144, nullptr, 3, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "range task create failed");
        s_task = nullptr;
    }
}

bool range_sensor_get_snapshot(range_state_t *out)
{
    if (out == nullptr || s_state_mutex == nullptr) {
        return false;
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    *out = s_state;
    xSemaphoreGive(s_state_mutex);
    return true;
}

void range_sensor_reset(void)
{
    if (s_driver_mutex == nullptr) {
        return;
    }

    ESP_LOGW(TAG, "[range] sensor reset by xshut");
    xSemaphoreTake(s_driver_mutex, portMAX_DELAY);
    reset_sensor_with_xshut();
    const bool ok = initialize_sensor_device();
    xSemaphoreGive(s_driver_mutex);

    range_state_t state = snapshot_state();
    state.valid = false;
    state.distance_mm = 0;
    state.status = 255;
    state.timestamp_ms = now_ms();
    state.consecutive_fail_count = 0;
    state.consecutive_ok_count = 0;
    state.status_text = ok ? "reset_ok" : "reset_failed";
    publish_state(state);
}
