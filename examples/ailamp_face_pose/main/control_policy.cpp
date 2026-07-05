#include "control_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cinttypes>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "fusion_state.hpp"
#include "led_mixer.hpp"
#include "led_output.hpp"

namespace {

constexpr const char *TAG = "control";

constexpr uint32_t kControlTaskPeriodMs = 200;
constexpr uint32_t kControlFusionStaleMs = 1500;
constexpr uint32_t kControlFaceHoldMs = 800;
constexpr uint32_t kControlRangeHoldMs = 1500;
constexpr uint32_t kNoFaceTimeoutMs = 3000;
constexpr uint32_t kNoFaceRecoveryHoldMs = 2000;
constexpr float kBaselinePitch = 5.0f;
constexpr float kNormalBrightness = 0.90f;
constexpr float kNormalCctK = static_cast<float>(AILAMP_USE_NORMAL_CCT_K);
constexpr float kAlertBrightness = 0.90f;
constexpr float kAlertCctK = static_cast<float>(AILAMP_USE_ALERT_CCT_K);
constexpr float kSensorInvalidBrightness = 0.85f;
constexpr float kSensorInvalidCctK = 5000.0f;
constexpr uint32_t kPostureHoldMs = 1000;
constexpr uint32_t kFuzzyNoImproveMs = 8000;

SemaphoreHandle_t s_mutex = nullptr;
TaskHandle_t s_task = nullptr;
control_state_t s_state = {
    .valid = false,
    .mode = control_mode_t::SENSOR_INVALID,
    .timestamp_ms = 0,
    .update_count = 0,
    .posture_score = 0.0f,
    .pitch_score = 0.0f,
    .distance_score = 0.0f,
    .yaw_score = 0.0f,
    .roll_score = 0.0f,
    .center_score = 0.0f,
    .face_lost_score = 0.0f,
    .duration_score = 0.0f,
    .fuzzy_level = 0.0f,
    .target_brightness = kNormalBrightness,
    .target_cct_k = kNormalCctK,
    .warm_pwm_virtual = 0.0f,
    .cool_pwm_virtual = 0.0f,
    .warm_ratio = 0.0f,
    .cool_ratio = 0.0f,
    .warm_pwm_raw = 0.0f,
    .cool_pwm_raw = 0.0f,
    .led_mix_clipped = false,
    .led_mix_safe_test_limited = false,
    .voice_alert_required = false,
    .app_alert_required = false,
    .normal_ms = 0,
    .fuzzy_ms = 0,
    .alert_ms = 0,
    .bad_posture_ms = 0,
    .severe_posture_ms = 0,
    .fusion_valid = false,
    .fusion_target_valid = false,
    .fusion_face_valid = false,
    .fusion_range_valid = false,
    .suspected_head_down_no_face = false,
    .target_present_by_range = false,
    .recent_face_valid = false,
    .fusion_age_ms = UINT32_MAX,
    .fusion_frame_id = 0,
    .face_lost_ms = 0,
    .fusion_reason = "not_initialized",
    .mode_text = "SENSOR_INVALID",
    .reason = "not_initialized",
};

control_mode_t s_mode = control_mode_t::SENSOR_INVALID;
uint32_t s_mode_enter_ms = 0;
uint32_t s_last_update_ms = 0;
uint32_t s_bad_start_ms = 0;
uint32_t s_good_start_ms = 0;
uint32_t s_severe_start_ms = 0;
uint32_t s_too_near_start_ms = 0;
uint32_t s_target_invalid_start_ms = 0;
uint32_t s_alert_recovery_start_ms = 0;
uint32_t s_alert_enter_ms = 0;
uint32_t s_last_voice_alert_ms = 0;
uint32_t s_fuzzy_enter_score = 0;
uint32_t s_both_invalid_start_ms = 0;
uint32_t s_no_face_start_ms = 0;
uint32_t s_range_invalid_start_ms = 0;
fusion_state_t s_last_good_fusion = {};
uint32_t s_last_good_fusion_ms = 0;
bool s_has_last_good_fusion = false;
int64_t s_last_log_us = 0;

uint32_t now_ms()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

float clampf(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

float linear_range(float value, float low, float high, float out_low, float out_high)
{
    if (value <= low) {
        return out_low;
    }
    if (value >= high) {
        return out_high;
    }
    const float ratio = (value - low) / (high - low);
    return out_low + (out_high - out_low) * ratio;
}

float rate_limit(float current, float target, float max_step)
{
    if (target > current + max_step) {
        return current + max_step;
    }
    if (target < current - max_step) {
        return current - max_step;
    }
    return target;
}

uint32_t elapsed_or_zero(uint32_t now, uint32_t start)
{
    return start == 0 ? 0 : now - start;
}

uint32_t snapshot_age_ms(uint32_t now, uint32_t timestamp)
{
    return timestamp == 0 ? UINT32_MAX : now - timestamp;
}

void set_timer(bool active, uint32_t now, uint32_t *start_ms)
{
    if (active) {
        if (*start_ms == 0) {
            *start_ms = now;
        }
    } else {
        *start_ms = 0;
    }
}

struct posture_score_breakdown_t {
    float total;
    float pitch;
    float distance;
    float yaw;
    float roll;
    float center;
    float face_lost;
    float duration;
};

posture_score_breakdown_t compute_posture_score(const fusion_state_t &fusion, uint32_t bad_posture_ms)
{
    posture_score_breakdown_t score = {};

    const float distance = static_cast<float>(fusion.filtered_distance_mm);
    if (fusion.range_valid && fusion.filtered_distance_mm > 0) {
        if (distance < 300.0f) {
            score.distance = 35.0f;
        } else if (distance < 400.0f) {
            score.distance = linear_range(distance, 300.0f, 400.0f, 35.0f, 0.0f);
        } else if (distance <= 650.0f) {
            score.distance = 0.0f;
        } else if (distance <= 900.0f) {
            score.distance = linear_range(distance, 650.0f, 900.0f, 0.0f, 10.0f);
        }
    }

    const float abs_pitch = std::fabs(fusion.filtered_pitch - kBaselinePitch);
    if (abs_pitch > 30.0f) {
        score.pitch = 35.0f;
    } else {
        score.pitch = linear_range(abs_pitch, 12.0f, 30.0f, 0.0f, 35.0f);
    }

    const float abs_yaw = std::fabs(fusion.filtered_yaw);
    score.yaw = abs_yaw > 35.0f ? 10.0f : linear_range(abs_yaw, 15.0f, 35.0f, 0.0f, 10.0f);

    const float abs_roll = std::fabs(fusion.filtered_roll);
    score.roll = abs_roll > 25.0f ? 10.0f : linear_range(abs_roll, 10.0f, 25.0f, 0.0f, 10.0f);

    const float center_x = clampf(fusion.face_center_x_norm, 0.0f, 1.0f);
    const float center_y = clampf(fusion.face_center_y_norm, 0.0f, 1.0f);
    const float center_offset = std::sqrt((center_x - 0.5f) * (center_x - 0.5f) +
                                          (center_y - 0.5f) * (center_y - 0.5f));
    score.center = center_offset > 0.60f ? 10.0f : linear_range(center_offset, 0.25f, 0.60f, 0.0f, 10.0f);

    if (!fusion.face_valid && fusion.target_present_by_range) {
        if (fusion.face_lost_ms >= 1500) {
            score.face_lost = 40.0f;
        } else if (fusion.face_lost_ms >= 800) {
            score.face_lost = linear_range(static_cast<float>(fusion.face_lost_ms), 800.0f, 1500.0f, 10.0f, 25.0f);
        }
    }

    score.duration = bad_posture_ms >= kPostureHoldMs ? 10.0f : 0.0f;
    score.total = clampf(score.pitch + score.distance + score.yaw + score.roll +
                             score.center + score.face_lost + score.duration,
                         0.0f,
                         100.0f);
    return score;
}

void compute_led_mix(control_state_t *state)
{
    LedMixInput input = {
        .brightness = state->target_brightness,
        .cct_k = static_cast<int>(std::lround(state->target_cct_k)),
    };
    LedMixOutput output = {};
    if (!led_mixer_calculate(&input, &output)) {
        output = {};
    }

    state->target_brightness = output.brightness_limited;
    state->target_cct_k = static_cast<float>(output.cct_limited);
    state->warm_ratio = output.warm_ratio;
    state->cool_ratio = output.cool_ratio;
    state->warm_pwm_raw = output.warm_pwm_raw;
    state->cool_pwm_raw = output.cool_pwm_raw;
    state->warm_pwm_virtual = output.warm_pwm;
    state->cool_pwm_virtual = output.cool_pwm;
    state->led_mix_clipped = output.clipped;
    state->led_mix_safe_test_limited = output.safe_test_limited;
}

void enter_mode(control_mode_t next_mode, uint32_t now, const char **reason)
{
    if (s_mode == next_mode) {
        return;
    }

    s_mode = next_mode;
    s_mode_enter_ms = now;
    s_good_start_ms = 0;
    s_alert_recovery_start_ms = 0;

    if (next_mode == control_mode_t::FUZZY_ADJUST) {
        *reason = "bad_posture_held";
    } else if (next_mode == control_mode_t::ALERT) {
        s_alert_enter_ms = now;
        if (*reason == nullptr || std::strcmp(*reason, "normal") == 0 || std::strcmp(*reason, "bad_posture_held") == 0) {
            *reason = "alert_condition_held";
        }
    } else if (next_mode == control_mode_t::NORMAL) {
        *reason = "posture_recovered";
    } else {
        if (*reason == nullptr || std::strcmp(*reason, "normal") == 0) {
            *reason = "fusion_invalid";
        }
    }
}

void maybe_log_control(const control_state_t &state)
{
    const int64_t now_us_value = esp_timer_get_time();
    if (now_us_value - s_last_log_us < 1000000) {
        return;
    }
    s_last_log_us = now_us_value;

    ESP_LOGI(TAG,
             "[control] mode=%s score=%.1f fuzzy=%.2f bright=%.2f cct=%.0f warm=%.2f cool=%.2f voice=%d app=%d reason=%s face_lost_ms=%" PRIu32 " target_by_range=%d recent_face=%d suspected_head_down=%d fusion_valid=%d target=%d face=%d range=%d age=%" PRIu32 " frame=%" PRIu32 " fusion_reason=%s",
             state.mode_text != nullptr ? state.mode_text : "UNKNOWN",
             static_cast<double>(state.posture_score),
             static_cast<double>(state.fuzzy_level),
             static_cast<double>(state.target_brightness),
             static_cast<double>(state.target_cct_k),
             static_cast<double>(state.warm_pwm_virtual),
             static_cast<double>(state.cool_pwm_virtual),
             state.voice_alert_required ? 1 : 0,
             state.app_alert_required ? 1 : 0,
             state.reason != nullptr ? state.reason : "unknown",
             state.face_lost_ms,
             state.target_present_by_range ? 1 : 0,
             state.recent_face_valid ? 1 : 0,
             state.suspected_head_down_no_face ? 1 : 0,
             state.fusion_valid ? 1 : 0,
             state.fusion_target_valid ? 1 : 0,
             state.fusion_face_valid ? 1 : 0,
             state.fusion_range_valid ? 1 : 0,
             state.fusion_age_ms,
             state.fusion_frame_id,
             state.fusion_reason != nullptr ? state.fusion_reason : "unknown");
}

void control_task(void *)
{
    while (true) {
        control_policy_update_from_fusion();
        vTaskDelay(pdMS_TO_TICKS(kControlTaskPeriodMs));
    }
}

} // namespace

const char *control_mode_to_text(control_mode_t mode)
{
    switch (mode) {
    case control_mode_t::NORMAL:
        return "NORMAL";
    case control_mode_t::FUZZY_ADJUST:
        return "FUZZY_ADJUST";
    case control_mode_t::ALERT:
        return "ALERT";
    case control_mode_t::SENSOR_INVALID:
    default:
        return "SENSOR_INVALID";
    }
}

bool control_policy_init(void)
{
    if (s_mutex == nullptr) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == nullptr) {
            ESP_LOGE(TAG, "control state mutex create failed");
            return false;
        }
    }

    if (s_task == nullptr) {
        BaseType_t ok = xTaskCreate(control_task, "control_task", 4096, nullptr, 2, &s_task);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "control task create failed");
            s_task = nullptr;
            return false;
        }
    }
    return true;
}

bool control_policy_update_from_fusion(void)
{
    if (s_mutex == nullptr) {
        return false;
    }

    const uint32_t now = now_ms();
    const float dt_s = s_last_update_ms == 0 ? 0.0f : static_cast<float>(now - s_last_update_ms) / 1000.0f;
    s_last_update_ms = now;

    fusion_state_t raw_fusion = {};
    const bool has_fusion = fusion_state_get_snapshot(&raw_fusion);
    const uint32_t raw_fusion_age_ms = has_fusion ? snapshot_age_ms(now, raw_fusion.timestamp_ms) : UINT32_MAX;
    const bool raw_fusion_fresh = has_fusion && raw_fusion.valid && raw_fusion.timestamp_ms != 0 &&
                                  raw_fusion_age_ms <= kControlFusionStaleMs;

    control_state_t next = s_state;
    next.timestamp_ms = now;
    next.update_count = s_state.update_count + 1;
    next.voice_alert_required = false;
    next.app_alert_required = false;
    next.fusion_valid = raw_fusion_fresh;
    next.fusion_target_valid = has_fusion && raw_fusion.target_valid;
    next.fusion_face_valid = has_fusion && raw_fusion.face_valid;
    next.fusion_range_valid = has_fusion && raw_fusion.range_valid;
    next.suspected_head_down_no_face = has_fusion && raw_fusion.suspected_head_down_no_face;
    next.target_present_by_range = has_fusion && raw_fusion.target_present_by_range;
    next.recent_face_valid = has_fusion && raw_fusion.recent_face_valid;
    next.fusion_age_ms = raw_fusion_age_ms;
    next.fusion_frame_id = has_fusion ? raw_fusion.frame_id : 0;
    next.face_lost_ms = has_fusion ? raw_fusion.face_lost_ms : 0;
    next.fusion_reason = has_fusion && raw_fusion.invalid_reason != nullptr ? raw_fusion.invalid_reason : "snapshot_failed";

    const char *reason = "normal";
    fusion_state_t control_fusion = raw_fusion;
    bool control_fusion_available = false;
    bool using_last_good_fusion = false;

    if (!has_fusion) {
        reason = "snapshot_failed";
    } else if (raw_fusion.timestamp_ms == 0) {
        reason = "fusion_timestamp_zero";
    } else if (!raw_fusion.valid) {
        reason = s_has_last_good_fusion ? "fusion_invalid_hold" : "fusion_invalid";
    } else if (!raw_fusion_fresh) {
        reason = "fusion_stale";
    }

    if (raw_fusion_fresh) {
        set_timer(!raw_fusion.face_valid, now, &s_no_face_start_ms);
        set_timer(!raw_fusion.range_valid, now, &s_range_invalid_start_ms);
        set_timer(!raw_fusion.face_valid && !raw_fusion.range_valid, now, &s_both_invalid_start_ms);

        if (raw_fusion.target_valid) {
            control_fusion = raw_fusion;
            control_fusion_available = true;
            s_last_good_fusion = raw_fusion;
            s_last_good_fusion_ms = now;
            s_has_last_good_fusion = true;
            reason = "normal";
        } else if (!raw_fusion.face_valid && raw_fusion.range_valid && s_has_last_good_fusion &&
                   elapsed_or_zero(now, s_last_good_fusion_ms) <= kControlFaceHoldMs) {
            control_fusion = s_last_good_fusion;
            control_fusion.distance_mm = raw_fusion.distance_mm;
            control_fusion.filtered_distance_mm = raw_fusion.filtered_distance_mm;
            control_fusion.range_valid = raw_fusion.range_valid;
            control_fusion.distance_in_range = raw_fusion.distance_in_range;
            control_fusion.too_near = raw_fusion.too_near;
            control_fusion.too_far = raw_fusion.too_far;
            control_fusion.timestamp_ms = raw_fusion.timestamp_ms;
            control_fusion.frame_id = raw_fusion.frame_id;
            control_fusion_available = true;
            using_last_good_fusion = true;
            reason = "face_hold";
        } else if (!raw_fusion.face_valid && !raw_fusion.range_valid && s_has_last_good_fusion &&
                   elapsed_or_zero(now, s_last_good_fusion_ms) <= kControlRangeHoldMs &&
                   elapsed_or_zero(now, s_both_invalid_start_ms) < kControlRangeHoldMs) {
            control_fusion = s_last_good_fusion;
            control_fusion.timestamp_ms = raw_fusion.timestamp_ms;
            control_fusion.frame_id = raw_fusion.frame_id;
            control_fusion_available = true;
            using_last_good_fusion = true;
            reason = "fusion_hold";
        } else if (raw_fusion.face_valid || raw_fusion.range_valid) {
            control_fusion = raw_fusion;
            control_fusion_available = true;
            reason = raw_fusion.invalid_reason != nullptr ? raw_fusion.invalid_reason : "target_not_valid";
        }

        if (raw_fusion.suspected_head_down_no_face) {
            control_fusion = raw_fusion;
            control_fusion_available = true;
            reason = "suspected_head_down_no_face";
        } else if (elapsed_or_zero(now, s_no_face_start_ms) >= kNoFaceTimeoutMs &&
                   (!raw_fusion.target_present_by_range || !raw_fusion.recent_face_valid)) {
            control_fusion_available = false;
            reason = "no_face_timeout";
        } else if (!raw_fusion.face_valid && !raw_fusion.range_valid &&
                   elapsed_or_zero(now, s_both_invalid_start_ms) >= kControlRangeHoldMs) {
            control_fusion_available = false;
            reason = "face_range_invalid_timeout";
        }
    } else if (s_has_last_good_fusion && elapsed_or_zero(now, s_last_good_fusion_ms) <= kControlFaceHoldMs) {
        control_fusion = s_last_good_fusion;
        control_fusion_available = true;
        using_last_good_fusion = true;
        if (std::strcmp(reason, "normal") == 0) {
            reason = "fusion_hold";
        }
    }

    if (!control_fusion_available) {
        next.valid = false;
        next.posture_score = 0.0f;
        next.pitch_score = 0.0f;
        next.distance_score = 0.0f;
        next.yaw_score = 0.0f;
        next.roll_score = 0.0f;
        next.center_score = 0.0f;
        next.face_lost_score = 0.0f;
        next.duration_score = 0.0f;
        next.fuzzy_level = 0.0f;
        if (has_fusion && raw_fusion.too_far) {
            reason = "target_absent";
        } else if (has_fusion && !raw_fusion.target_present_by_range && !raw_fusion.recent_face_valid) {
            reason = "target_absent";
        } else if (reason == nullptr || std::strcmp(reason, "normal") == 0) {
            reason = "sensor_invalid";
        }
        enter_mode(control_mode_t::SENSOR_INVALID, now, &reason);
    } else {
        next.valid = true;
        posture_score_breakdown_t base_score = compute_posture_score(control_fusion, 0);
        const bool bad_posture = base_score.total >= 30.0f;
        const bool too_near = control_fusion.filtered_distance_mm > 0 &&
                              control_fusion.filtered_distance_mm < 300;
        const bool target_absent = control_fusion.too_far ||
                                   (!control_fusion.target_present_by_range && !control_fusion.recent_face_valid);
        const bool sensor_invalid = (!control_fusion.face_valid && !control_fusion.range_valid) || target_absent;

        set_timer(bad_posture, now, &s_bad_start_ms);
        const uint32_t held_bad_ms = elapsed_or_zero(now, s_bad_start_ms);
        posture_score_breakdown_t score = compute_posture_score(control_fusion, held_bad_ms);
        next.posture_score = score.total;
        next.pitch_score = score.pitch;
        next.distance_score = score.distance;
        next.yaw_score = score.yaw;
        next.roll_score = score.roll;
        next.center_score = score.center;
        next.face_lost_score = score.face_lost;
        next.duration_score = score.duration;

        const bool good_for_normal = next.posture_score < 20.0f;
        const bool severe_posture = next.posture_score >= 70.0f;
        const bool suspected_head_down = control_fusion.suspected_head_down_no_face;

        set_timer(good_for_normal, now, &s_good_start_ms);
        set_timer(severe_posture, now, &s_severe_start_ms);
        set_timer(too_near, now, &s_too_near_start_ms);
        set_timer(sensor_invalid && !using_last_good_fusion, now, &s_target_invalid_start_ms);

        if (sensor_invalid) {
            reason = target_absent ? "target_absent" : "sensor_invalid";
            enter_mode(control_mode_t::SENSOR_INVALID, now, &reason);
        } else if (s_mode == control_mode_t::SENSOR_INVALID) {
            enter_mode(control_mode_t::NORMAL, now, &reason);
        }

        if (sensor_invalid) {
            next.valid = false;
        } else if (too_near) {
            reason = "distance_too_near";
            enter_mode(control_mode_t::ALERT, now, &reason);
        } else if (suspected_head_down) {
            reason = "suspected_head_down_no_face";
            enter_mode(control_mode_t::ALERT, now, &reason);
        } else if (s_mode == control_mode_t::NORMAL) {
            if (severe_posture && elapsed_or_zero(now, s_severe_start_ms) >= kPostureHoldMs) {
                reason = "severe_posture_degradation";
                enter_mode(control_mode_t::ALERT, now, &reason);
            } else if (bad_posture && held_bad_ms >= kPostureHoldMs) {
                s_fuzzy_enter_score = static_cast<uint32_t>(std::lround(next.posture_score));
                enter_mode(control_mode_t::FUZZY_ADJUST, now, &reason);
            } else {
                reason = using_last_good_fusion ? reason : (bad_posture ? "bad_posture_timer" : "posture_ok");
            }
        } else if (s_mode == control_mode_t::FUZZY_ADJUST) {
            const bool fuzzy_too_long =
                elapsed_or_zero(now, s_mode_enter_ms) >= kFuzzyNoImproveMs &&
                next.posture_score > static_cast<float>(s_fuzzy_enter_score) - 10.0f;

            if (elapsed_or_zero(now, s_good_start_ms) >= 3000) {
                enter_mode(control_mode_t::NORMAL, now, &reason);
            } else if (severe_posture && elapsed_or_zero(now, s_severe_start_ms) >= kPostureHoldMs) {
                reason = "severe_posture_degradation";
                enter_mode(control_mode_t::ALERT, now, &reason);
            } else if (fuzzy_too_long) {
                reason = "fuzzy_no_improve";
                enter_mode(control_mode_t::ALERT, now, &reason);
            } else {
                reason = using_last_good_fusion ? reason : "mild_posture_degradation";
            }
        } else if (s_mode == control_mode_t::ALERT) {
            reason = too_near ? "distance_too_near" : "alert_active";
            if (!too_near && control_fusion.face_valid && control_fusion.range_valid && next.posture_score < 25.0f) {
                if (s_alert_recovery_start_ms == 0) {
                    s_alert_recovery_start_ms = now;
                }
                reason = "alert_recovery_timer";
            } else {
                s_alert_recovery_start_ms = 0;
            }

            if (elapsed_or_zero(now, s_alert_recovery_start_ms) >= kNoFaceRecoveryHoldMs) {
                enter_mode(control_mode_t::NORMAL, now, &reason);
            }
        }
    }

    next.mode = s_mode;
    next.mode_text = control_mode_to_text(s_mode);
    next.reason = reason != nullptr ? reason : "unknown";
    next.bad_posture_ms = elapsed_or_zero(now, s_bad_start_ms);
    next.severe_posture_ms = elapsed_or_zero(now, s_severe_start_ms);
    next.normal_ms = s_mode == control_mode_t::NORMAL ? elapsed_or_zero(now, s_mode_enter_ms) : 0;
    next.fuzzy_ms = s_mode == control_mode_t::FUZZY_ADJUST ? elapsed_or_zero(now, s_mode_enter_ms) : 0;
    next.alert_ms = s_mode == control_mode_t::ALERT ? elapsed_or_zero(now, s_mode_enter_ms) : 0;

    float desired_brightness = kNormalBrightness;
    float desired_cct = kNormalCctK;
    if (s_mode == control_mode_t::FUZZY_ADJUST) {
        next.fuzzy_level = clampf((next.posture_score - 30.0f) / 40.0f, 0.0f, 1.0f);
        desired_brightness = kNormalBrightness - 0.15f * next.fuzzy_level;
        desired_cct = kNormalCctK - 1400.0f * next.fuzzy_level;
        next.target_brightness = rate_limit(s_state.target_brightness, desired_brightness, 0.08f * dt_s);
        next.target_cct_k = rate_limit(s_state.target_cct_k, desired_cct, 80.0f * dt_s);
    } else if (s_mode == control_mode_t::ALERT) {
        next.fuzzy_level = 0.0f;
        desired_brightness = kAlertBrightness;
        desired_cct = kAlertCctK;
        next.target_brightness = rate_limit(s_state.target_brightness, desired_brightness, 0.10f * dt_s);
        next.target_cct_k = rate_limit(s_state.target_cct_k, desired_cct, 800.0f * dt_s);
        next.app_alert_required = true;
        if (s_last_voice_alert_ms == 0 || now - s_last_voice_alert_ms >= 60000 || now - s_alert_enter_ms < kControlTaskPeriodMs + 20) {
            next.voice_alert_required = true;
            s_last_voice_alert_ms = now;
        }
    } else if (s_mode == control_mode_t::SENSOR_INVALID) {
        next.fuzzy_level = 0.0f;
        desired_brightness = kSensorInvalidBrightness;
        desired_cct = kSensorInvalidCctK;
        next.target_brightness = rate_limit(s_state.target_brightness, desired_brightness, 0.10f * dt_s);
        next.target_cct_k = rate_limit(s_state.target_cct_k, desired_cct, 500.0f * dt_s);
    } else {
        next.fuzzy_level = 0.0f;
        next.target_brightness = rate_limit(s_state.target_brightness, kNormalBrightness, 0.10f * dt_s);
        next.target_cct_k = rate_limit(s_state.target_cct_k, kNormalCctK, 500.0f * dt_s);
    }

    if (dt_s <= 0.0f) {
        next.target_brightness = desired_brightness;
        next.target_cct_k = desired_cct;
    }

    compute_led_mix(&next);

    led_output_set_target(next.warm_pwm_virtual,
                          next.cool_pwm_virtual,
                          next.mode_text,
                          next.reason);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state = next;
    xSemaphoreGive(s_mutex);

    maybe_log_control(next);
    return true;
}

bool control_policy_get_snapshot(control_state_t *out)
{
    if (out == nullptr || s_mutex == nullptr) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_state;
    xSemaphoreGive(s_mutex);
    return true;
}
