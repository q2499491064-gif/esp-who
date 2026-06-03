#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

class Vl53l1xDriver {
public:
    static constexpr uint8_t kDefaultAddress = 0x29;

    explicit Vl53l1xDriver(i2c_master_bus_handle_t bus);
    ~Vl53l1xDriver();

    esp_err_t init();
    esp_err_t start_ranging();
    esp_err_t stop_ranging();
    esp_err_t read_sensor_id(uint16_t *sensor_id);
    esp_err_t read_distance(uint16_t *distance_mm, uint8_t *range_status);

    const char *last_error() const;
    const char *range_status_to_string(uint8_t range_status) const;

private:
    esp_err_t add_device();
    esp_err_t write_u8(uint16_t reg, uint8_t value);
    esp_err_t write_u16(uint16_t reg, uint16_t value);
    esp_err_t write_u32(uint16_t reg, uint32_t value);
    esp_err_t read_u8(uint16_t reg, uint8_t *value);
    esp_err_t read_u16(uint16_t reg, uint16_t *value);
    esp_err_t read_multi(uint16_t reg, uint8_t *data, size_t len);
    esp_err_t wait_booted(uint32_t timeout_ms);
    esp_err_t wait_data_ready(uint32_t timeout_ms);
    esp_err_t clear_interrupt();
    esp_err_t sensor_init_sequence();
    esp_err_t set_distance_mode_long();
    esp_err_t set_timing_budget_100ms();
    esp_err_t set_intermeasurement_ms(uint32_t intermeasurement_ms);
    bool data_ready();
    uint8_t read_range_status();
    void set_error(const char *message);

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;
    const char *last_error_ = "ok";
};
