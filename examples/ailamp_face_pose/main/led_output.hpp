#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

// GPIO47 -> LED driver PWM1 -> Warm channel W- by default
// GPIO48 -> LED driver PWM2 -> Cool channel C- by default
#ifndef AILAMP_LED_WARM_PWM_GPIO
#define AILAMP_LED_WARM_PWM_GPIO GPIO_NUM_47
#endif

#ifndef AILAMP_LED_COOL_PWM_GPIO
#define AILAMP_LED_COOL_PWM_GPIO GPIO_NUM_48
#endif

#ifndef AILAMP_LED_PWM_FREQ_HZ
#define AILAMP_LED_PWM_FREQ_HZ 5000
#endif

#ifndef AILAMP_LED_PWM_RES_BITS
#define AILAMP_LED_PWM_RES_BITS 13
#endif

#ifndef AILAMP_LED_PWM_MAX_DUTY
#define AILAMP_LED_PWM_MAX_DUTY ((1 << AILAMP_LED_PWM_RES_BITS) - 1)
#endif

#ifndef AILAMP_LED_COMMON_ANODE_COB
#define AILAMP_LED_COMMON_ANODE_COB 1
#endif

// Default: driver PWM input is active high. Set to 0 if higher duty makes the lamp darker.
#ifndef AILAMP_LED_PWM_ACTIVE_HIGH
#define AILAMP_LED_PWM_ACTIVE_HIGH 1
#endif

// Set to 1 if GPIO47/PWM1 controls cool white and GPIO48/PWM2 controls warm white.
#ifndef AILAMP_LED_SWAP_WARM_COOL_CHANNELS
#define AILAMP_LED_SWAP_WARM_COOL_CHANNELS 0
#endif

#ifndef AILAMP_LED_SAFE_TEST_LIMIT_ENABLE
#define AILAMP_LED_SAFE_TEST_LIMIT_ENABLE 1
#endif

#ifndef AILAMP_LED_SAFE_TEST_PWM_LIMIT
#define AILAMP_LED_SAFE_TEST_PWM_LIMIT 0.30f
#endif

#ifndef AILAMP_LED_TOTAL_PWM_LIMIT
#define AILAMP_LED_TOTAL_PWM_LIMIT 0.90f
#endif

#ifndef AILAMP_LED_SMOOTH_STEP
#define AILAMP_LED_SMOOTH_STEP 0.02f
#endif

#ifndef AILAMP_LED_TASK_PERIOD_MS
#define AILAMP_LED_TASK_PERIOD_MS 20
#endif

enum class LedOutputTestMode {
    NORMAL = 0,
    OFF,
    WARM,
    COOL,
    BOTH,
};

struct LedOutputSnapshot {
    bool initialized;
    bool enabled;
    bool fault;

    bool common_anode_cob;
    bool active_high;
    bool swap_warm_cool_channels;
    bool safe_test_limit_enable;

    int warm_gpio;
    int cool_gpio;

    uint32_t pwm_freq_hz;
    uint8_t duty_resolution_bits;
    uint32_t max_duty;

    float warm_target;
    float cool_target;
    float warm_current;
    float cool_current;

    uint32_t warm_duty_logical;
    uint32_t cool_duty_logical;
    uint32_t warm_duty_output;
    uint32_t cool_duty_output;

    const char *source_mode;
    const char *source_reason;

    const char *test_mode;
    bool test_mode_active;

    uint32_t update_count;
    uint32_t last_update_ms;
    const char *fault_reason;
};

esp_err_t led_output_init();
bool led_output_is_initialized();
bool led_output_is_enabled();
void led_output_set_enabled(bool enabled);
esp_err_t led_output_set_target(float warm_pwm_virtual,
                                float cool_pwm_virtual,
                                const char *source_mode,
                                const char *source_reason);
esp_err_t led_output_force_off(const char *reason);
esp_err_t led_output_set_test_mode(LedOutputTestMode mode);
const char *led_output_test_mode_to_text(LedOutputTestMode mode);
bool led_output_get_snapshot(LedOutputSnapshot *out);
void led_output_task_start();
void led_output_update_once();
