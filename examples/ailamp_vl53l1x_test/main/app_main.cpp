#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "vl53l1x_driver.hpp"

namespace {

constexpr const char *TAG = "ailamp_vl53l1x_test";
constexpr i2c_port_num_t kI2cPort = I2C_NUM_1;
constexpr gpio_num_t kSdaGpio = GPIO_NUM_8;
constexpr gpio_num_t kSclGpio = GPIO_NUM_9;
constexpr gpio_num_t kXshutGpio = GPIO_NUM_10;
constexpr uint32_t kI2cFreqHz = 100000;

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

esp_err_t create_i2c_bus(i2c_master_bus_handle_t *bus)
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = kI2cPort;
    bus_config.sda_io_num = kSdaGpio;
    bus_config.scl_io_num = kSclGpio;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    return i2c_new_master_bus(&bus_config, bus);
}

bool scan_for_address(i2c_master_bus_handle_t bus, uint8_t target_addr)
{
    bool found = false;

    ESP_LOGI(TAG, "I2C scan start on I2C_NUM_1 SDA=GPIO8 SCL=GPIO9 freq=%luHz",
             static_cast<unsigned long>(kI2cFreqHz));
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        esp_err_t err = i2c_master_probe(bus, addr, 50);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02x", addr);
            if (addr == target_addr) {
                found = true;
            }
        }
    }

    if (found) {
        ESP_LOGI(TAG, "VL53L1X found at 0x29");
    } else {
        ESP_LOGW(TAG, "VL53L1X not found");
    }

    return found;
}

} // namespace

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(configure_xshut());
    reset_sensor_with_xshut();

    i2c_master_bus_handle_t bus = nullptr;
    ESP_ERROR_CHECK(create_i2c_bus(&bus));

    if (!scan_for_address(bus, Vl53l1xDriver::kDefaultAddress)) {
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    Vl53l1xDriver sensor(bus);

    uint16_t sensor_id = 0;
    esp_err_t err = sensor.read_sensor_id(&sensor_id);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "VL53L1X sensor_id=0x%04x", sensor_id);
    } else {
        ESP_LOGE(TAG, "read sensor id failed: %s", sensor.last_error());
    }

    err = sensor.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X init failed: %s", sensor.last_error());
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    err = sensor.start_ranging();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start ranging failed: %s", sensor.last_error());
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    while (true) {
        uint16_t distance_mm = 0;
        uint8_t range_status = 255;
        err = sensor.read_distance(&distance_mm, &range_status);
        if (err == ESP_OK && range_status == 0) {
            ESP_LOGI(TAG, "distance_mm=%u status=ok", distance_mm);
        } else if (err == ESP_OK) {
            ESP_LOGW(TAG, "distance_mm=%u status=%s(%u)", distance_mm,
                     sensor.range_status_to_string(range_status), range_status);
        } else {
            ESP_LOGE(TAG, "read distance failed: %s", sensor.last_error());
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
