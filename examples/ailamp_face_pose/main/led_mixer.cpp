#include "led_mixer.hpp"

#include <algorithm>
#include <cmath>

namespace {

float sanitize_float(float value)
{
    if (std::isnan(value) || !std::isfinite(value)) {
        return 0.0f;
    }
    return value;
}

float clampf(float value, float low, float high)
{
    value = sanitize_float(value);
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

} // namespace

bool led_mixer_calculate(const LedMixInput *input, LedMixOutput *output)
{
    if (input == nullptr || output == nullptr) {
        return false;
    }

    LedMixOutput next = {};
    next.valid = true;
    next.brightness_limited = clampf(input->brightness, 0.0f, 1.0f);
    next.cct_limited = static_cast<int>(std::lround(clampf(static_cast<float>(input->cct_k),
                                                           static_cast<float>(AILAMP_COB_WARM_CCT_K),
                                                           static_cast<float>(AILAMP_COB_COOL_CCT_K))));

    const float cct_span = static_cast<float>(AILAMP_COB_COOL_CCT_K - AILAMP_COB_WARM_CCT_K);
    next.cool_ratio = clampf((static_cast<float>(next.cct_limited - AILAMP_COB_WARM_CCT_K)) / cct_span,
                             0.0f,
                             1.0f);
    next.warm_ratio = clampf(1.0f - next.cool_ratio, 0.0f, 1.0f);

    next.warm_pwm_raw = clampf(next.brightness_limited * next.warm_ratio, 0.0f, 1.0f);
    next.cool_pwm_raw = clampf(next.brightness_limited * next.cool_ratio, 0.0f, 1.0f);

    const float pwm_sum = next.warm_pwm_raw + next.cool_pwm_raw;
    if (pwm_sum > AILAMP_LED_TOTAL_PWM_LIMIT && pwm_sum > 0.0f) {
        const float scale = AILAMP_LED_TOTAL_PWM_LIMIT / pwm_sum;
        next.warm_pwm_raw = clampf(next.warm_pwm_raw * scale, 0.0f, 1.0f);
        next.cool_pwm_raw = clampf(next.cool_pwm_raw * scale, 0.0f, 1.0f);
        next.clipped = true;
    }

#if AILAMP_LED_SAFE_TEST_LIMIT_ENABLE
    next.warm_pwm = std::min(next.warm_pwm_raw, AILAMP_LED_SAFE_TEST_PWM_LIMIT);
    next.cool_pwm = std::min(next.cool_pwm_raw, AILAMP_LED_SAFE_TEST_PWM_LIMIT);
    next.safe_test_limited = true;
#else
    next.warm_pwm = next.warm_pwm_raw;
    next.cool_pwm = next.cool_pwm_raw;
    next.safe_test_limited = false;
#endif

    next.warm_pwm = clampf(next.warm_pwm, 0.0f, 1.0f);
    next.cool_pwm = clampf(next.cool_pwm, 0.0f, 1.0f);
    *output = next;
    return true;
}
