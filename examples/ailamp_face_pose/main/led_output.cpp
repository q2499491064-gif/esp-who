#include "led_output.hpp"

#include <algorithm>
#include <cmath>
#include <cinttypes>
#include <cstring>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

constexpr const char *TAG = "led_output";

constexpr ledc_mode_t kLedcSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_1;
constexpr ledc_channel_t kWarmChannel = LEDC_CHANNEL_0;
constexpr ledc_channel_t kCoolChannel = LEDC_CHANNEL_1;
constexpr uint32_t kPwmFreqHz = AILAMP_LED_PWM_FREQ_HZ;
constexpr uint8_t kDutyResolutionBits = AILAMP_LED_PWM_RES_BITS;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_13_BIT;
constexpr uint32_t kMaxDuty = AILAMP_LED_PWM_MAX_DUTY;
constexpr uint32_t kTaskPeriodMs = AILAMP_LED_TASK_PERIOD_MS;
constexpr float kSmoothStep = AILAMP_LED_SMOOTH_STEP;

#ifndef AILAMP_LED_OUTPUT_DEFAULT_ENABLE
#define AILAMP_LED_OUTPUT_DEFAULT_ENABLE 1
#endif

static_assert(AILAMP_LED_PWM_RES_BITS == 13, "AILAMP_LED_PWM_RES_BITS must match LEDC_TIMER_13_BIT");

SemaphoreHandle_t s_mutex = nullptr;
TaskHandle_t s_task = nullptr;
LedOutputTestMode s_test_mode = LedOutputTestMode::NORMAL;
float s_normal_warm_target = 0.0f;
float s_normal_cool_target = 0.0f;
const char *s_normal_source_mode = "none";
const char *s_normal_source_reason = "not_initialized";

LedOutputSnapshot s_snapshot = {
    .initialized = false,
    .enabled = false,
    .fault = true,
    .common_anode_cob = AILAMP_LED_COMMON_ANODE_COB != 0,
    .active_high = AILAMP_LED_PWM_ACTIVE_HIGH != 0,
    .swap_warm_cool_channels = AILAMP_LED_SWAP_WARM_COOL_CHANNELS != 0,
    .safe_test_limit_enable = AILAMP_LED_SAFE_TEST_LIMIT_ENABLE != 0,
    .warm_gpio = static_cast<int>(AILAMP_LED_WARM_PWM_GPIO),
    .cool_gpio = static_cast<int>(AILAMP_LED_COOL_PWM_GPIO),
    .pwm_freq_hz = kPwmFreqHz,
    .duty_resolution_bits = kDutyResolutionBits,
    .max_duty = kMaxDuty,
    .warm_target = 0.0f,
    .cool_target = 0.0f,
    .warm_current = 0.0f,
    .cool_current = 0.0f,
    .warm_duty_logical = 0,
    .cool_duty_logical = 0,
    .warm_duty_output = 0,
    .cool_duty_output = 0,
    .source_mode = "none",
    .source_reason = "not_initialized",
    .test_mode = "normal",
    .test_mode_active = false,
    .update_count = 0,
    .last_update_ms = 0,
    .fault_reason = "not_initialized",
};

const char *s_last_logged_mode = nullptr;
const char *s_last_logged_reason = nullptr;
int64_t s_last_pwm_log_us = 0;

uint32_t now_ms()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

float sanitize_pwm(float value)
{
    if (std::isnan(value) || !std::isfinite(value)) {
        return 0.0f;
    }
    return std::min(std::max(value, 0.0f), 1.0f);
}

float approach(float current, float target)
{
    if (current < target) {
        return std::min(current + kSmoothStep, target);
    }
    if (current > target) {
        return std::max(current - kSmoothStep, target);
    }
    return current;
}

uint32_t pwm_to_logical_duty(float value)
{
    const float sanitized = sanitize_pwm(value);
    const uint32_t duty = static_cast<uint32_t>(std::lround(sanitized * static_cast<float>(kMaxDuty)));
    return std::min(duty, kMaxDuty);
}

uint32_t logical_to_output_duty(uint32_t logical_duty)
{
    const uint32_t safe_duty = std::min(logical_duty, kMaxDuty);
#if AILAMP_LED_PWM_ACTIVE_HIGH
    return safe_duty;
#else
    return kMaxDuty - safe_duty;
#endif
}

esp_err_t write_channel_duty(ledc_channel_t channel, uint32_t duty)
{
    const uint32_t safe_duty = std::min(duty, kMaxDuty);
    esp_err_t err = ledc_set_duty(kLedcSpeedMode, channel, safe_duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(kLedcSpeedMode, channel);
}

void set_duty_snapshot_locked(uint32_t warm_logical, uint32_t cool_logical)
{
    s_snapshot.warm_duty_logical = std::min(warm_logical, kMaxDuty);
    s_snapshot.cool_duty_logical = std::min(cool_logical, kMaxDuty);
    s_snapshot.warm_duty_output = logical_to_output_duty(s_snapshot.warm_duty_logical);
    s_snapshot.cool_duty_output = logical_to_output_duty(s_snapshot.cool_duty_logical);
}

esp_err_t apply_duty_locked(uint32_t warm_logical, uint32_t cool_logical, const char *failure_reason)
{
    const uint32_t warm_output = logical_to_output_duty(warm_logical);
    const uint32_t cool_output = logical_to_output_duty(cool_logical);

#if AILAMP_LED_SWAP_WARM_COOL_CHANNELS
    const uint32_t channel0_output = cool_output;
    const uint32_t channel1_output = warm_output;
#else
    const uint32_t channel0_output = warm_output;
    const uint32_t channel1_output = cool_output;
#endif

    esp_err_t err = write_channel_duty(kWarmChannel, channel0_output);
    if (err != ESP_OK) {
        s_snapshot.fault = true;
        s_snapshot.enabled = false;
        s_snapshot.fault_reason = failure_reason != nullptr ? failure_reason : "warm_duty_failed";
        write_channel_duty(kWarmChannel, logical_to_output_duty(0));
        write_channel_duty(kCoolChannel, logical_to_output_duty(0));
        s_snapshot.warm_current = 0.0f;
        s_snapshot.cool_current = 0.0f;
        set_duty_snapshot_locked(0, 0);
        return err;
    }

    err = write_channel_duty(kCoolChannel, channel1_output);
    if (err != ESP_OK) {
        s_snapshot.fault = true;
        s_snapshot.enabled = false;
        s_snapshot.fault_reason = failure_reason != nullptr ? failure_reason : "cool_duty_failed";
        write_channel_duty(kWarmChannel, logical_to_output_duty(0));
        write_channel_duty(kCoolChannel, logical_to_output_duty(0));
        s_snapshot.warm_current = 0.0f;
        s_snapshot.cool_current = 0.0f;
        set_duty_snapshot_locked(0, 0);
        return err;
    }

    set_duty_snapshot_locked(warm_logical, cool_logical);
    return ESP_OK;
}

void set_static_snapshot_fields_locked()
{
    s_snapshot.common_anode_cob = AILAMP_LED_COMMON_ANODE_COB != 0;
    s_snapshot.active_high = AILAMP_LED_PWM_ACTIVE_HIGH != 0;
    s_snapshot.swap_warm_cool_channels = AILAMP_LED_SWAP_WARM_COOL_CHANNELS != 0;
    s_snapshot.safe_test_limit_enable = AILAMP_LED_SAFE_TEST_LIMIT_ENABLE != 0;
    s_snapshot.warm_gpio = static_cast<int>(AILAMP_LED_WARM_PWM_GPIO);
    s_snapshot.cool_gpio = static_cast<int>(AILAMP_LED_COOL_PWM_GPIO);
    s_snapshot.pwm_freq_hz = kPwmFreqHz;
    s_snapshot.duty_resolution_bits = kDutyResolutionBits;
    s_snapshot.max_duty = kMaxDuty;
    s_snapshot.test_mode = led_output_test_mode_to_text(s_test_mode);
    s_snapshot.test_mode_active = s_test_mode != LedOutputTestMode::NORMAL;
}

void set_effective_target_locked(float warm, float cool, const char *mode, const char *reason)
{
    s_snapshot.warm_target = sanitize_pwm(warm);
    s_snapshot.cool_target = sanitize_pwm(cool);
    s_snapshot.source_mode = mode != nullptr ? mode : "unknown";
    s_snapshot.source_reason = reason != nullptr ? reason : "unknown";
}

void refresh_effective_target_locked()
{
    set_static_snapshot_fields_locked();
    switch (s_test_mode) {
    case LedOutputTestMode::OFF:
        set_effective_target_locked(0.0f, 0.0f, "LED_TEST", "test_off");
        break;
    case LedOutputTestMode::WARM:
        set_effective_target_locked(0.15f, 0.0f, "LED_TEST", "test_warm");
        break;
    case LedOutputTestMode::COOL:
        set_effective_target_locked(0.0f, 0.15f, "LED_TEST", "test_cool");
        break;
    case LedOutputTestMode::BOTH:
        set_effective_target_locked(0.10f, 0.10f, "LED_TEST", "test_both");
        break;
    case LedOutputTestMode::NORMAL:
    default:
        set_effective_target_locked(s_normal_warm_target,
                                    s_normal_cool_target,
                                    s_normal_source_mode,
                                    s_normal_source_reason);
        break;
    }
}

void set_init_fault_locked(const char *reason, esp_err_t err)
{
    s_snapshot.initialized = false;
    s_snapshot.enabled = false;
    s_snapshot.fault = true;
    s_snapshot.fault_reason = reason != nullptr ? reason : "init_failed";
    s_snapshot.warm_target = 0.0f;
    s_snapshot.cool_target = 0.0f;
    s_snapshot.warm_current = 0.0f;
    s_snapshot.cool_current = 0.0f;
    set_duty_snapshot_locked(0, 0);
    write_channel_duty(kWarmChannel, logical_to_output_duty(0));
    write_channel_duty(kCoolChannel, logical_to_output_duty(0));
    ESP_LOGE(TAG,
             "[init] failed err=%s reason=%s",
             esp_err_to_name(err),
             s_snapshot.fault_reason != nullptr ? s_snapshot.fault_reason : "unknown");
}

void maybe_log_source_locked()
{
    const char *mode = s_snapshot.source_mode != nullptr ? s_snapshot.source_mode : "unknown";
    const char *reason = s_snapshot.source_reason != nullptr ? s_snapshot.source_reason : "unknown";
    const bool source_changed =
        s_last_logged_mode == nullptr || s_last_logged_reason == nullptr ||
        std::strcmp(mode, s_last_logged_mode) != 0 || std::strcmp(reason, s_last_logged_reason) != 0;

    if (!source_changed) {
        return;
    }

    s_last_logged_mode = mode;
    s_last_logged_reason = reason;
    ESP_LOGI(TAG,
             "[source] mode=%s reason=%s test=%s warm=%.2f cool=%.2f",
             mode,
             reason,
             s_snapshot.test_mode != nullptr ? s_snapshot.test_mode : "normal",
             static_cast<double>(s_snapshot.warm_target),
             static_cast<double>(s_snapshot.cool_target));
}

void maybe_log_pwm_locked()
{
    const int64_t now_us_value = esp_timer_get_time();
    if (now_us_value - s_last_pwm_log_us < 2000000) {
        return;
    }
    s_last_pwm_log_us = now_us_value;

    ESP_LOGI(TAG,
             "[pwm] en=%d mode=%s warm_target=%.2f cool_target=%.2f warm_current=%.2f cool_current=%.2f warm_logical=%" PRIu32 " cool_logical=%" PRIu32 " warm_out=%" PRIu32 " cool_out=%" PRIu32,
             s_snapshot.enabled ? 1 : 0,
             s_snapshot.source_mode != nullptr ? s_snapshot.source_mode : "unknown",
             static_cast<double>(s_snapshot.warm_target),
             static_cast<double>(s_snapshot.cool_target),
             static_cast<double>(s_snapshot.warm_current),
             static_cast<double>(s_snapshot.cool_current),
             s_snapshot.warm_duty_logical,
             s_snapshot.cool_duty_logical,
             s_snapshot.warm_duty_output,
             s_snapshot.cool_duty_output);
}

void led_output_task(void *)
{
    while (true) {
        led_output_update_once();
        vTaskDelay(pdMS_TO_TICKS(kTaskPeriodMs));
    }
}

} // namespace

const char *led_output_test_mode_to_text(LedOutputTestMode mode)
{
    switch (mode) {
    case LedOutputTestMode::OFF:
        return "off";
    case LedOutputTestMode::WARM:
        return "warm";
    case LedOutputTestMode::COOL:
        return "cool";
    case LedOutputTestMode::BOTH:
        return "both";
    case LedOutputTestMode::NORMAL:
    default:
        return "normal";
    }
}

esp_err_t led_output_init()
{
    if (s_mutex == nullptr) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    set_static_snapshot_fields_locked();
    s_snapshot.warm_target = 0.0f;
    s_snapshot.cool_target = 0.0f;
    s_snapshot.warm_current = 0.0f;
    s_snapshot.cool_current = 0.0f;
    set_duty_snapshot_locked(0, 0);
    s_snapshot.source_mode = "none";
    s_snapshot.source_reason = "init";
    s_snapshot.fault_reason = "none";
    s_normal_warm_target = 0.0f;
    s_normal_cool_target = 0.0f;
    s_normal_source_mode = "none";
    s_normal_source_reason = "init";
    xSemaphoreGive(s_mutex);

    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = kLedcSpeedMode;
    timer_conf.timer_num = kLedcTimer;
    timer_conf.duty_resolution = kDutyResolution;
    timer_conf.freq_hz = kPwmFreqHz;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        set_init_fault_locked("timer_config_failed", err);
        xSemaphoreGive(s_mutex);
        return err;
    }

    ledc_channel_config_t warm_conf = {};
    warm_conf.gpio_num = static_cast<int>(AILAMP_LED_WARM_PWM_GPIO); // GPIO47 -> LED driver PWM1 -> Warm channel W- by default
    warm_conf.speed_mode = kLedcSpeedMode;
    warm_conf.channel = kWarmChannel;
    warm_conf.intr_type = LEDC_INTR_DISABLE;
    warm_conf.timer_sel = kLedcTimer;
    warm_conf.duty = logical_to_output_duty(0);
    warm_conf.hpoint = 0;

    err = ledc_channel_config(&warm_conf);
    if (err != ESP_OK) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        set_init_fault_locked("warm_channel_config_failed", err);
        xSemaphoreGive(s_mutex);
        return err;
    }

    ledc_channel_config_t cool_conf = {};
    cool_conf.gpio_num = static_cast<int>(AILAMP_LED_COOL_PWM_GPIO); // GPIO48 -> LED driver PWM2 -> Cool channel C- by default
    cool_conf.speed_mode = kLedcSpeedMode;
    cool_conf.channel = kCoolChannel;
    cool_conf.intr_type = LEDC_INTR_DISABLE;
    cool_conf.timer_sel = kLedcTimer;
    cool_conf.duty = logical_to_output_duty(0);
    cool_conf.hpoint = 0;

    err = ledc_channel_config(&cool_conf);
    if (err != ESP_OK) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        set_init_fault_locked("cool_channel_config_failed", err);
        xSemaphoreGive(s_mutex);
        return err;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.initialized = true;
    s_snapshot.enabled = AILAMP_LED_OUTPUT_DEFAULT_ENABLE != 0;
    s_snapshot.fault = false;
    s_snapshot.fault_reason = "none";
    s_snapshot.last_update_ms = now_ms();
    refresh_effective_target_locked();
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG,
             "[init] ok warm_gpio=%d cool_gpio=%d freq=%" PRIu32 "Hz res=%u max_duty=%" PRIu32 " active_high=%d swap=%d safe_limit=%d",
             static_cast<int>(AILAMP_LED_WARM_PWM_GPIO),
             static_cast<int>(AILAMP_LED_COOL_PWM_GPIO),
             kPwmFreqHz,
             static_cast<unsigned>(kDutyResolutionBits),
             kMaxDuty,
             AILAMP_LED_PWM_ACTIVE_HIGH != 0 ? 1 : 0,
             AILAMP_LED_SWAP_WARM_COOL_CHANNELS != 0 ? 1 : 0,
             AILAMP_LED_SAFE_TEST_LIMIT_ENABLE != 0 ? 1 : 0);
    return ESP_OK;
}

bool led_output_is_initialized()
{
    if (s_mutex == nullptr) {
        return false;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const bool initialized = s_snapshot.initialized;
    xSemaphoreGive(s_mutex);
    return initialized;
}

bool led_output_is_enabled()
{
    if (s_mutex == nullptr) {
        return false;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const bool enabled = s_snapshot.enabled;
    xSemaphoreGive(s_mutex);
    return enabled;
}

void led_output_set_enabled(bool enabled)
{
    if (s_mutex == nullptr) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.enabled = enabled && s_snapshot.initialized && !s_snapshot.fault;
    if (!s_snapshot.enabled) {
        s_normal_warm_target = 0.0f;
        s_normal_cool_target = 0.0f;
        refresh_effective_target_locked();
        s_snapshot.warm_current = 0.0f;
        s_snapshot.cool_current = 0.0f;
        apply_duty_locked(0, 0, "set_enabled_off");
    }
    s_snapshot.last_update_ms = now_ms();
    xSemaphoreGive(s_mutex);
}

esp_err_t led_output_set_target(float warm_pwm_virtual,
                                float cool_pwm_virtual,
                                const char *source_mode,
                                const char *source_reason)
{
    if (s_mutex == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!s_snapshot.initialized) {
        s_snapshot.fault = true;
        s_snapshot.enabled = false;
        s_snapshot.fault_reason = "set_target_before_init";
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_snapshot.fault) {
        xSemaphoreGive(s_mutex);
        return ESP_FAIL;
    }

    s_normal_warm_target = sanitize_pwm(warm_pwm_virtual);
    s_normal_cool_target = sanitize_pwm(cool_pwm_virtual);
    s_normal_source_mode = source_mode != nullptr ? source_mode : "unknown";
    s_normal_source_reason = source_reason != nullptr ? source_reason : "unknown";
    refresh_effective_target_locked();
    s_snapshot.last_update_ms = now_ms();
    maybe_log_source_locked();
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t led_output_force_off(const char *reason)
{
    if (s_mutex == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.enabled = false;
    s_test_mode = LedOutputTestMode::OFF;
    s_normal_warm_target = 0.0f;
    s_normal_cool_target = 0.0f;
    refresh_effective_target_locked();
    s_snapshot.warm_current = 0.0f;
    s_snapshot.cool_current = 0.0f;
    s_snapshot.source_reason = reason != nullptr ? reason : "force_off";
    s_snapshot.fault_reason = reason != nullptr ? reason : "force_off";
    s_snapshot.last_update_ms = now_ms();
    const esp_err_t err = apply_duty_locked(0, 0, "force_off_failed");
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t led_output_set_test_mode(LedOutputTestMode mode)
{
    if (s_mutex == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!s_snapshot.initialized || s_snapshot.fault) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_test_mode = mode;
    refresh_effective_target_locked();
    s_snapshot.last_update_ms = now_ms();
    maybe_log_source_locked();
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

bool led_output_get_snapshot(LedOutputSnapshot *out)
{
    if (out == nullptr) {
        return false;
    }
    if (s_mutex == nullptr) {
        *out = s_snapshot;
        return true;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    refresh_effective_target_locked();
    *out = s_snapshot;
    xSemaphoreGive(s_mutex);
    return true;
}

void led_output_update_once()
{
    if (s_mutex == nullptr) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    refresh_effective_target_locked();
    if (!s_snapshot.initialized || s_snapshot.fault || !s_snapshot.enabled) {
        s_snapshot.warm_current = 0.0f;
        s_snapshot.cool_current = 0.0f;
        apply_duty_locked(0, 0, "safe_off_failed");
        s_snapshot.update_count++;
        s_snapshot.last_update_ms = now_ms();
        xSemaphoreGive(s_mutex);
        return;
    }

    s_snapshot.warm_current = approach(s_snapshot.warm_current, s_snapshot.warm_target);
    s_snapshot.cool_current = approach(s_snapshot.cool_current, s_snapshot.cool_target);
    const uint32_t warm_logical = pwm_to_logical_duty(s_snapshot.warm_current);
    const uint32_t cool_logical = pwm_to_logical_duty(s_snapshot.cool_current);
    apply_duty_locked(warm_logical, cool_logical, "duty_update_failed");
    s_snapshot.update_count++;
    s_snapshot.last_update_ms = now_ms();
    maybe_log_pwm_locked();
    xSemaphoreGive(s_mutex);
}

void led_output_task_start()
{
    if (s_task != nullptr) {
        return;
    }

    BaseType_t ok = xTaskCreate(led_output_task, "led_output_task", 3072, nullptr, 2, &s_task);
    if (ok != pdPASS) {
        s_task = nullptr;
        if (s_mutex != nullptr) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_snapshot.fault = true;
            s_snapshot.enabled = false;
            s_snapshot.fault_reason = "task_create_failed";
            apply_duty_locked(0, 0, "task_create_failed");
            xSemaphoreGive(s_mutex);
        }
        ESP_LOGE(TAG, "led output task create failed");
    }
}
