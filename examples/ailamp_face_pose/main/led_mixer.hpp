#pragma once

#include <stdbool.h>

#define AILAMP_COB_WARM_CCT_K 2700
#define AILAMP_COB_COOL_CCT_K 6500

#define AILAMP_USE_MIN_CCT_K 4200
#define AILAMP_USE_NORMAL_CCT_K 5600
#define AILAMP_USE_ALERT_CCT_K 5200
#define AILAMP_USE_MAX_CCT_K 6000

#ifndef AILAMP_LED_TOTAL_PWM_LIMIT
#define AILAMP_LED_TOTAL_PWM_LIMIT 0.90f
#endif

#ifndef AILAMP_LED_SAFE_TEST_LIMIT_ENABLE
#define AILAMP_LED_SAFE_TEST_LIMIT_ENABLE 1
#endif

#ifndef AILAMP_LED_SAFE_TEST_PWM_LIMIT
#define AILAMP_LED_SAFE_TEST_PWM_LIMIT 0.30f
#endif

struct LedMixInput {
    float brightness;
    int cct_k;
};

struct LedMixOutput {
    bool valid;
    float brightness_limited;
    int cct_limited;

    float warm_ratio;
    float cool_ratio;

    float warm_pwm_raw;
    float cool_pwm_raw;

    float warm_pwm;
    float cool_pwm;

    bool clipped;
    bool safe_test_limited;
};

bool led_mixer_calculate(const LedMixInput *input, LedMixOutput *output);
