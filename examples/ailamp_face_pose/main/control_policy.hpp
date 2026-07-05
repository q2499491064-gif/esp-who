#pragma once

#include <cstdint>

enum class control_mode_t {
    NORMAL,
    FUZZY_ADJUST,
    ALERT,
    SENSOR_INVALID,
};

struct control_state_t {
    bool valid;
    control_mode_t mode;

    uint32_t timestamp_ms;
    uint32_t update_count;

    float posture_score;
    float pitch_score;
    float distance_score;
    float yaw_score;
    float roll_score;
    float center_score;
    float face_lost_score;
    float duration_score;
    float fuzzy_level;

    float target_brightness;
    float target_cct_k;

    float warm_pwm_virtual;
    float cool_pwm_virtual;
    float warm_ratio;
    float cool_ratio;
    float warm_pwm_raw;
    float cool_pwm_raw;
    bool led_mix_clipped;
    bool led_mix_safe_test_limited;

    bool voice_alert_required;
    bool app_alert_required;

    uint32_t normal_ms;
    uint32_t fuzzy_ms;
    uint32_t alert_ms;

    uint32_t bad_posture_ms;
    uint32_t severe_posture_ms;

    bool fusion_valid;
    bool fusion_target_valid;
    bool fusion_face_valid;
    bool fusion_range_valid;
    bool suspected_head_down_no_face;
    bool target_present_by_range;
    bool recent_face_valid;
    uint32_t fusion_age_ms;
    uint32_t fusion_frame_id;
    uint32_t face_lost_ms;
    const char *fusion_reason;

    const char *mode_text;
    const char *reason;
};

bool control_policy_init(void);
bool control_policy_update_from_fusion(void);
bool control_policy_get_snapshot(control_state_t *out);
const char *control_mode_to_text(control_mode_t mode);
