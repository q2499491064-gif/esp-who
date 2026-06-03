#include "vl53l1x_driver.hpp"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr uint16_t SOFT_RESET = 0x0000;
constexpr uint16_t VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
constexpr uint16_t GPIO_HV_MUX__CTRL = 0x0030;
constexpr uint16_t GPIO__TIO_HV_STATUS = 0x0031;
constexpr uint16_t PHASECAL_CONFIG__TIMEOUT_MACROP = 0x004B;
constexpr uint16_t RANGE_CONFIG__TIMEOUT_MACROP_A_HI = 0x005E;
constexpr uint16_t RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060;
constexpr uint16_t RANGE_CONFIG__TIMEOUT_MACROP_B_HI = 0x0061;
constexpr uint16_t RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063;
constexpr uint16_t RANGE_CONFIG__VALID_PHASE_HIGH = 0x0069;
constexpr uint16_t VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C;
constexpr uint16_t SD_CONFIG__WOI_SD0 = 0x0078;
constexpr uint16_t SD_CONFIG__INITIAL_PHASE_SD0 = 0x007A;
constexpr uint16_t SYSTEM__INTERRUPT_CLEAR = 0x0086;
constexpr uint16_t SYSTEM__MODE_START = 0x0087;
constexpr uint16_t VL53L1_RESULT__RANGE_STATUS = 0x0089;
constexpr uint16_t VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096;
constexpr uint16_t VL53L1_RESULT__OSC_CALIBRATE_VAL = 0x00DE;
constexpr uint16_t VL53L1_FIRMWARE__SYSTEM_STATUS = 0x00E5;
constexpr uint16_t VL53L1_IDENTIFICATION__MODEL_ID = 0x010F;

constexpr uint8_t kDefaultConfig[] = {
    0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
    0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9b, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00,
};
static_assert(sizeof(kDefaultConfig) == (0x0087 - 0x002D + 1), "VL53L1X default config size mismatch");

constexpr uint8_t kStatusMap[] = {
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255,
    9,   13,  255, 255, 255, 255, 10, 6, 255, 255, 11, 12,
};

uint64_t now_ms()
{
    return static_cast<uint64_t>(esp_timer_get_time() / 1000);
}

} // namespace

Vl53l1xDriver::Vl53l1xDriver(i2c_master_bus_handle_t bus) : bus_(bus) {}

Vl53l1xDriver::~Vl53l1xDriver()
{
    if (dev_ != nullptr) {
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
}

esp_err_t Vl53l1xDriver::init()
{
    esp_err_t err = add_device();
    if (err != ESP_OK) {
        return err;
    }

    err = wait_booted(1000);
    if (err != ESP_OK) {
        return err;
    }

    err = sensor_init_sequence();
    if (err != ESP_OK) {
        return err;
    }

    err = set_distance_mode_long();
    if (err != ESP_OK) {
        return err;
    }

    err = set_timing_budget_100ms();
    if (err != ESP_OK) {
        return err;
    }

    err = set_intermeasurement_ms(100);
    if (err != ESP_OK) {
        return err;
    }

    set_error("ok");
    return ESP_OK;
}

esp_err_t Vl53l1xDriver::start_ranging()
{
    esp_err_t err = write_u8(SYSTEM__MODE_START, 0x40);
    if (err != ESP_OK) {
        set_error("failed to start ranging");
    }
    return err;
}

esp_err_t Vl53l1xDriver::stop_ranging()
{
    esp_err_t err = write_u8(SYSTEM__MODE_START, 0x00);
    if (err != ESP_OK) {
        set_error("failed to stop ranging");
    }
    return err;
}

esp_err_t Vl53l1xDriver::read_sensor_id(uint16_t *sensor_id)
{
    esp_err_t err = add_device();
    if (err != ESP_OK) {
        return err;
    }

    err = read_u16(VL53L1_IDENTIFICATION__MODEL_ID, sensor_id);
    if (err != ESP_OK) {
        set_error("failed to read sensor id register 0x010f");
    } else {
        set_error("ok");
    }
    return err;
}

esp_err_t Vl53l1xDriver::read_distance(uint16_t *distance_mm, uint8_t *range_status)
{
    if (distance_mm == nullptr || range_status == nullptr) {
        set_error("null output pointer");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = wait_data_ready(200);
    if (err != ESP_OK) {
        return err;
    }

    *range_status = read_range_status();
    err = read_u16(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distance_mm);
    if (err != ESP_OK) {
        set_error("failed to read distance register 0x0096");
        return err;
    }

    err = clear_interrupt();
    if (err != ESP_OK) {
        set_error("failed to clear interrupt after distance read");
        return err;
    }

    set_error("ok");
    return ESP_OK;
}

const char *Vl53l1xDriver::last_error() const
{
    return last_error_;
}

const char *Vl53l1xDriver::range_status_to_string(uint8_t range_status) const
{
    switch (range_status) {
    case 0:
        return "ok";
    case 1:
        return "sigma_fail";
    case 2:
        return "signal_fail";
    case 3:
        return "range_valid_min_range_clipped";
    case 4:
        return "out_of_bounds";
    case 5:
        return "hardware_fail";
    case 6:
        return "no_wrap_check";
    case 7:
        return "wrapped_target_fail";
    case 9:
        return "xtalk_signal_fail";
    case 10:
        return "synchronization_int";
    case 11:
        return "range_valid_no_wrap_check";
    case 12:
        return "range_valid";
    case 13:
        return "range_valid_merged_pulse";
    default:
        return "unknown";
    }
}

esp_err_t Vl53l1xDriver::add_device()
{
    if (dev_ != nullptr) {
        return ESP_OK;
    }

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = kDefaultAddress;
    dev_config.scl_speed_hz = 100000;

    esp_err_t err = i2c_master_bus_add_device(bus_, &dev_config, &dev_);
    if (err != ESP_OK) {
        set_error("failed to add VL53L1X I2C device at 0x29");
    }
    return err;
}

esp_err_t Vl53l1xDriver::write_u8(uint16_t reg, uint8_t value)
{
    uint8_t buffer[] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xff),
        value,
    };
    return i2c_master_transmit(dev_, buffer, sizeof(buffer), 100);
}

esp_err_t Vl53l1xDriver::write_u16(uint16_t reg, uint16_t value)
{
    uint8_t buffer[] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xff),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xff),
    };
    return i2c_master_transmit(dev_, buffer, sizeof(buffer), 100);
}

esp_err_t Vl53l1xDriver::write_u32(uint16_t reg, uint32_t value)
{
    uint8_t buffer[] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xff),
        static_cast<uint8_t>((value >> 24) & 0xff),
        static_cast<uint8_t>((value >> 16) & 0xff),
        static_cast<uint8_t>((value >> 8) & 0xff),
        static_cast<uint8_t>(value & 0xff),
    };
    return i2c_master_transmit(dev_, buffer, sizeof(buffer), 100);
}

esp_err_t Vl53l1xDriver::read_u8(uint16_t reg, uint8_t *value)
{
    return read_multi(reg, value, 1);
}

esp_err_t Vl53l1xDriver::read_u16(uint16_t reg, uint16_t *value)
{
    uint8_t data[2] = {};
    esp_err_t err = read_multi(reg, data, sizeof(data));
    if (err == ESP_OK) {
        *value = static_cast<uint16_t>((data[0] << 8) | data[1]);
    }
    return err;
}

esp_err_t Vl53l1xDriver::read_multi(uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t reg_buffer[] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xff),
    };
    return i2c_master_transmit_receive(dev_, reg_buffer, sizeof(reg_buffer), data, len, 100);
}

esp_err_t Vl53l1xDriver::wait_booted(uint32_t timeout_ms)
{
    const uint64_t deadline = now_ms() + timeout_ms;
    uint8_t state = 0;
    while (now_ms() < deadline) {
        esp_err_t err = read_u8(VL53L1_FIRMWARE__SYSTEM_STATUS, &state);
        if (err != ESP_OK) {
            set_error("failed to read boot state register 0x00e5");
            return err;
        }
        if ((state & 0x01) != 0) {
            set_error("ok");
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    set_error("timeout waiting for VL53L1X boot state");
    return ESP_ERR_TIMEOUT;
}

esp_err_t Vl53l1xDriver::wait_data_ready(uint32_t timeout_ms)
{
    const uint64_t deadline = now_ms() + timeout_ms;
    while (now_ms() < deadline) {
        if (data_ready()) {
            set_error("ok");
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    set_error("timeout waiting for data ready");
    return ESP_ERR_TIMEOUT;
}

esp_err_t Vl53l1xDriver::clear_interrupt()
{
    return write_u8(SYSTEM__INTERRUPT_CLEAR, 0x01);
}

esp_err_t Vl53l1xDriver::sensor_init_sequence()
{
    for (uint16_t reg = 0x002D; reg <= 0x0087; ++reg) {
        esp_err_t err = write_u8(reg, kDefaultConfig[reg - 0x002D]);
        if (err != ESP_OK) {
            set_error("failed to write VL53L1X default configuration table");
            return err;
        }
    }

    esp_err_t err = start_ranging();
    if (err != ESP_OK) {
        return err;
    }

    err = wait_data_ready(1000);
    if (err != ESP_OK) {
        return err;
    }

    err = clear_interrupt();
    if (err != ESP_OK) {
        set_error("failed to clear interrupt during init");
        return err;
    }

    err = stop_ranging();
    if (err != ESP_OK) {
        return err;
    }

    err = write_u8(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    if (err != ESP_OK) {
        set_error("failed to configure VHV loop bound");
        return err;
    }

    err = write_u8(SOFT_RESET + 0x000B, 0x00);
    if (err != ESP_OK) {
        set_error("failed to configure VHV temperature restart");
        return err;
    }

    set_error("ok");
    return ESP_OK;
}

esp_err_t Vl53l1xDriver::set_distance_mode_long()
{
    esp_err_t err = write_u8(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
    if (err == ESP_OK) {
        err = write_u8(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
    }
    if (err == ESP_OK) {
        err = write_u8(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
    }
    if (err == ESP_OK) {
        err = write_u8(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
    }
    if (err == ESP_OK) {
        err = write_u8(SD_CONFIG__WOI_SD0, 0x0F);
    }
    if (err == ESP_OK) {
        err = write_u8(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E);
    }

    if (err != ESP_OK) {
        set_error("failed to set long distance mode");
    }
    return err;
}

esp_err_t Vl53l1xDriver::set_timing_budget_100ms()
{
    esp_err_t err = write_u16(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x04F3);
    if (err == ESP_OK) {
        err = write_u16(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0624);
    }

    if (err != ESP_OK) {
        set_error("failed to set 100ms timing budget");
    }
    return err;
}

esp_err_t Vl53l1xDriver::set_intermeasurement_ms(uint32_t intermeasurement_ms)
{
    uint16_t clock_pll = 0;
    esp_err_t err = read_u16(VL53L1_RESULT__OSC_CALIBRATE_VAL, &clock_pll);
    if (err != ESP_OK) {
        set_error("failed to read oscillator calibration");
        return err;
    }

    clock_pll &= 0x03ff;
    if (clock_pll == 0) {
        set_error("invalid oscillator calibration value");
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint32_t period = static_cast<uint32_t>(clock_pll * intermeasurement_ms * 1075 / 1000);
    err = write_u32(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, period);
    if (err != ESP_OK) {
        set_error("failed to set intermeasurement period");
    }
    return err;
}

bool Vl53l1xDriver::data_ready()
{
    uint8_t gpio_hv_mux = 0;
    uint8_t gpio_status = 0;

    if (read_u8(GPIO_HV_MUX__CTRL, &gpio_hv_mux) != ESP_OK) {
        set_error("failed to read interrupt polarity");
        return false;
    }
    if (read_u8(GPIO__TIO_HV_STATUS, &gpio_status) != ESP_OK) {
        set_error("failed to read data-ready status");
        return false;
    }

    const uint8_t interrupt_polarity = ((gpio_hv_mux & 0x10) == 0) ? 1 : 0;
    return (gpio_status & 0x01) == interrupt_polarity;
}

uint8_t Vl53l1xDriver::read_range_status()
{
    uint8_t raw_status = 0;
    if (read_u8(VL53L1_RESULT__RANGE_STATUS, &raw_status) != ESP_OK) {
        set_error("failed to read range status");
        return 255;
    }

    raw_status &= 0x1f;
    if (raw_status < sizeof(kStatusMap)) {
        return kStatusMap[raw_status];
    }
    return 255;
}

void Vl53l1xDriver::set_error(const char *message)
{
    last_error_ = message;
}
