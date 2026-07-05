#include "fusion_state.hpp"

#include <algorithm>
#include <cmath>
#include <cinttypes>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

constexpr const char *TAG = "fusion";
constexpr uint32_t kVisionStaleMs = 500;
constexpr uint32_t kRangeStaleMs = 1000;
constexpr float kDistanceAlpha = 0.3f;
constexpr uint16_t kDistanceJumpRejectMm = 300;
constexpr uint32_t kFusionTaskPeriodMs = 200;
constexpr uint32_t kNoFaceAlertGraceMs = 1500;
constexpr uint32_t kRecentFaceWindowMs = 5000;
constexpr uint32_t kFastNoFaceAlertGraceMs = 800;
constexpr float kBaselinePitch = 5.0f;
constexpr float kHeadDownPitchDeltaDeg = 24.0f;

SemaphoreHandle_t s_mutex = nullptr;
TaskHandle_t s_task = nullptr;
fusion_state_t s_state = {
    .valid = false,
    .face_valid = false,
    .range_valid = false,
    .target_valid = false,
    .timestamp_ms = 0,
    .frame_id = 0,
    .face_count = 0,
    .displayed_face_count = 0,
    .distance_mm = 0,
    .filtered_distance_mm = 0,
    .distance_in_range = false,
    .too_near = false,
    .too_far = false,
    .target_present_by_range = false,
    .lost_with_range_valid = false,
    .recent_face_valid = false,
    .suspected_head_down_no_face = false,
    .face_lost_ms = 0,
    .last_face_valid_ms = 0,
    .last_pose_valid_ms = 0,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .roll = 0.0f,
    .filtered_pitch = 0.0f,
    .filtered_yaw = 0.0f,
    .filtered_roll = 0.0f,
    .last_valid_filtered_pitch = 0.0f,
    .face_score = 0.0f,
    .face_center_x_norm = 0.5f,
    .face_center_y_norm = 0.5f,
    .range_confidence = 0.0f,
    .fusion_confidence = 0.0f,
    .vision_age_ms = 0,
    .range_age_ms = 0,
    .update_count = 0,
    .invalid_count = 0,
    .state_text = "not_initialized",
    .invalid_reason = "not_initialized",
};
bool s_distance_filter_initialized = false;
float s_filtered_distance = 0.0f;
int64_t s_last_log_us = 0;
uint32_t s_face_lost_start_ms = 0;
uint32_t s_last_face_valid_ms = 0;
uint32_t s_last_pose_valid_ms = 0;
float s_last_valid_filtered_pitch = 0.0f;
bool s_has_last_valid_filtered_pitch = false;
bool s_suspected_head_down_latched = false;

uint32_t now_ms()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

uint32_t age_ms(uint32_t now, uint32_t timestamp)
{
    return timestamp == 0 ? UINT32_MAX : now - timestamp;
}

uint16_t update_filtered_distance(uint16_t distance_mm, bool usable)
{
    if (!usable || distance_mm == 0) {
        return s_distance_filter_initialized ? static_cast<uint16_t>(std::lround(s_filtered_distance)) : 0;
    }

    if (!s_distance_filter_initialized) {
        s_filtered_distance = static_cast<float>(distance_mm);
        s_distance_filter_initialized = true;
        return distance_mm;
    }

    const float delta = std::fabs(static_cast<float>(distance_mm) - s_filtered_distance);
    const float alpha = delta > kDistanceJumpRejectMm ? kDistanceAlpha * 0.25f : kDistanceAlpha;
    s_filtered_distance = alpha * static_cast<float>(distance_mm) + (1.0f - alpha) * s_filtered_distance;
    return static_cast<uint16_t>(std::lround(s_filtered_distance));
}

float clamp01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

bool tracking_state_is_live(const char *state)
{
    return state != nullptr && std::strcmp(state, "tracking") == 0;
}

void maybe_log_fusion(const fusion_state_t &state)
{
    const int64_t now_us_value = esp_timer_get_time();
    if (now_us_value - s_last_log_us < 1000000) {
        return;
    }
    s_last_log_us = now_us_value;
    ESP_LOGI(TAG,
             "[fusion] target=%d face=%d range=%d dist=%u filt=%u state=%s reason=%s suspected_head_down=%d face_lost_ms=%" PRIu32 " target_by_range=%d recent_face=%d conf=%.2f",
             state.target_valid ? 1 : 0,
             state.face_valid ? 1 : 0,
             state.range_valid ? 1 : 0,
             static_cast<unsigned>(state.distance_mm),
             static_cast<unsigned>(state.filtered_distance_mm),
             state.state_text != nullptr ? state.state_text : "unknown",
             state.invalid_reason != nullptr ? state.invalid_reason : "unknown",
             state.suspected_head_down_no_face ? 1 : 0,
             state.face_lost_ms,
             state.target_present_by_range ? 1 : 0,
             state.recent_face_valid ? 1 : 0,
             static_cast<double>(state.fusion_confidence));
}

void fusion_task(void *)
{
    while (true) {
        fusion_state_update_from_latest();
        vTaskDelay(pdMS_TO_TICKS(kFusionTaskPeriodMs));
    }
}

} // namespace

bool fusion_state_init(void)
{
    if (s_mutex == nullptr) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == nullptr) {
            ESP_LOGE(TAG, "fusion state mutex create failed");
            return false;
        }
    }

    if (s_task == nullptr) {
        BaseType_t ok = xTaskCreate(fusion_task, "fusion_task", 4096, nullptr, 2, &s_task);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "fusion task create failed");
            s_task = nullptr;
            return false;
        }
    }
    return true;
}

bool fusion_state_update(const face_http_snapshot_t *vision, const range_state_t *range)
{
    if (vision == nullptr || range == nullptr || s_mutex == nullptr) {
        return false;
    }

    const uint32_t now = now_ms();
    fusion_state_t next = {};

    next.valid = true;
    next.timestamp_ms = now;
    next.frame_id = vision->frame_id;
    next.face_count = vision->actual_face_count;
    next.displayed_face_count = vision->display_face_count;
    next.distance_mm = range->distance_mm;
    next.pitch = vision->pose_valid ? vision->pitch : 0.0f;
    next.yaw = vision->pose_valid ? vision->yaw : 0.0f;
    next.roll = vision->pose_valid ? vision->roll : 0.0f;
    next.filtered_pitch = vision->filtered_pose_valid ? vision->filtered_pitch : next.pitch;
    next.filtered_yaw = vision->filtered_pose_valid ? vision->filtered_yaw : next.yaw;
    next.filtered_roll = vision->filtered_pose_valid ? vision->filtered_roll : next.roll;
    next.face_score = vision->face_score;
    next.face_center_x_norm = vision->face_center_x_norm;
    next.face_center_y_norm = vision->face_center_y_norm;
    next.vision_age_ms = age_ms(now, vision->timestamp_ms);
    next.range_age_ms = age_ms(now, range->timestamp_ms);
    next.too_near = range->valid && range->distance_mm > 0 && range->distance_mm < AILAMP_DISTANCE_MIN_MM;
    next.too_far = range->valid && range->distance_mm > AILAMP_DISTANCE_MAX_MM;
    next.distance_in_range = range->valid && range->distance_mm >= AILAMP_DISTANCE_MIN_MM &&
                             range->distance_mm <= AILAMP_DISTANCE_MAX_MM;

    const bool vision_fresh = vision->valid && next.vision_age_ms <= kVisionStaleMs;
    const bool range_fresh = range->valid && next.range_age_ms <= kRangeStaleMs;
    next.face_valid = vision_fresh && vision->actual_face_count > 0 && tracking_state_is_live(vision->tracking_state);
    next.range_valid = range_fresh && range->distance_mm > 0;

    next.range_confidence = next.range_valid ? 0.90f : 0.0f;
    if (next.range_valid && !next.distance_in_range) {
        next.range_confidence = 0.45f;
    }
    next.fusion_confidence = clamp01((next.face_valid ? next.face_score : 0.0f) * 0.65f +
                                     next.range_confidence * 0.35f);

    if (next.face_valid) {
        s_last_face_valid_ms = now;
        s_face_lost_start_ms = 0;
        next.face_lost_ms = 0;
    } else {
        if (s_face_lost_start_ms == 0) {
            s_face_lost_start_ms = now;
        }
        next.face_lost_ms = now - s_face_lost_start_ms;
    }

    if (vision->pose_valid || vision->filtered_pose_valid) {
        s_last_pose_valid_ms = now;
        s_last_valid_filtered_pitch = next.filtered_pitch;
        s_has_last_valid_filtered_pitch = true;
    }

    next.last_face_valid_ms = s_last_face_valid_ms;
    next.last_pose_valid_ms = s_last_pose_valid_ms;
    next.last_valid_filtered_pitch = s_has_last_valid_filtered_pitch ? s_last_valid_filtered_pitch : 0.0f;
    next.target_present_by_range = next.range_valid && !next.too_far && range->distance_mm > 0 &&
                                   range->distance_mm <= AILAMP_DISTANCE_MAX_MM;
    next.lost_with_range_valid = !next.face_valid && next.range_valid;
    next.recent_face_valid = s_last_face_valid_ms != 0 && now - s_last_face_valid_ms <= kRecentFaceWindowMs;

    const bool last_pitch_was_head_down =
        s_has_last_valid_filtered_pitch &&
        std::fabs(s_last_valid_filtered_pitch - kBaselinePitch) >= kHeadDownPitchDeltaDeg;
    const uint32_t no_face_alert_grace =
        (last_pitch_was_head_down || next.too_near) ? kFastNoFaceAlertGraceMs : kNoFaceAlertGraceMs;
    const bool suspected_now = !next.face_valid && next.target_present_by_range &&
                               next.recent_face_valid && next.face_lost_ms >= no_face_alert_grace;
    if (next.face_valid || !next.target_present_by_range) {
        s_suspected_head_down_latched = false;
    } else if (suspected_now) {
        s_suspected_head_down_latched = true;
    }
    next.suspected_head_down_no_face = suspected_now || s_suspected_head_down_latched;

    if (!vision_fresh) {
        next.state_text = "invalid";
        next.invalid_reason = "vision_stale";
    } else if (!range_fresh) {
        next.state_text = "invalid";
        next.invalid_reason = "range_stale";
    } else if (!next.face_valid) {
        next.state_text = "face_invalid";
        next.invalid_reason = next.suspected_head_down_no_face
                                  ? "suspected_head_down_no_face"
                                  : (next.target_present_by_range ? "no_face_with_target_present"
                                                                  : (vision->actual_face_count <= 0 ? "no_face"
                                                                                                    : "face_invalid"));
    } else if (!next.range_valid) {
        next.state_text = "range_invalid";
        next.invalid_reason = "range_invalid";
    } else if (next.too_near) {
        next.state_text = "too_near";
        next.invalid_reason = "too_near";
    } else if (next.too_far) {
        next.state_text = "too_far";
        next.invalid_reason = "too_far";
    } else if (next.distance_in_range) {
        next.target_valid = true;
        next.state_text = "target_valid";
        next.invalid_reason = "none";
    } else {
        next.state_text = "invalid";
        next.invalid_reason = "distance_invalid";
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    next.filtered_distance_mm = update_filtered_distance(range->distance_mm, next.range_valid);
    next.update_count = s_state.update_count + 1;
    next.invalid_count = s_state.invalid_count + (next.target_valid ? 0 : 1);
    s_state = next;
    xSemaphoreGive(s_mutex);

    maybe_log_fusion(next);
    return true;
}

bool fusion_state_update_from_latest(void)
{
    face_http_snapshot_t vision = {};
    range_state_t range = {};
    const bool has_vision = face_http_result_get_snapshot(&vision);
    const bool has_range = range_sensor_get_snapshot(&range);

    if (!has_vision) {
        vision.valid = false;
        vision.timestamp_ms = 0;
        vision.tracking_state = "lost";
    }
    if (!has_range) {
        range.valid = false;
        range.timestamp_ms = 0;
        range.status_text = "not_initialized";
    }

    return fusion_state_update(&vision, &range);
}

bool fusion_state_get_snapshot(fusion_state_t *out)
{
    if (out == nullptr || s_mutex == nullptr) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_state;
    xSemaphoreGive(s_mutex);
    return true;
}
