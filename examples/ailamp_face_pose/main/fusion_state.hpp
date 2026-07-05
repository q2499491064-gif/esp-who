#pragma once

#include <cstdint>

#include "face_http_result.hpp"
#include "range_sensor.hpp"

#define AILAMP_DISTANCE_MIN_MM 250
#define AILAMP_DISTANCE_MAX_MM 900
#define AILAMP_DISTANCE_IDEAL_MIN_MM 350
#define AILAMP_DISTANCE_IDEAL_MAX_MM 650

struct fusion_state_t {
    bool valid;
    bool face_valid;
    bool range_valid;
    bool target_valid;

    uint32_t timestamp_ms;
    uint32_t frame_id;

    int face_count;
    int displayed_face_count;

    uint16_t distance_mm;
    uint16_t filtered_distance_mm;

    bool distance_in_range;
    bool too_near;
    bool too_far;
    bool target_present_by_range;
    bool lost_with_range_valid;
    bool recent_face_valid;
    bool suspected_head_down_no_face;

    uint32_t face_lost_ms;
    uint32_t last_face_valid_ms;
    uint32_t last_pose_valid_ms;

    float pitch;
    float yaw;
    float roll;

    float filtered_pitch;
    float filtered_yaw;
    float filtered_roll;
    float last_valid_filtered_pitch;

    float face_score;
    float face_center_x_norm;
    float face_center_y_norm;
    float range_confidence;
    float fusion_confidence;

    uint32_t vision_age_ms;
    uint32_t range_age_ms;

    uint32_t update_count;
    uint32_t invalid_count;

    const char *state_text;
    const char *invalid_reason;
};

bool fusion_state_init(void);
bool fusion_state_update(const face_http_snapshot_t *vision, const range_state_t *range);
bool fusion_state_update_from_latest(void);
bool fusion_state_get_snapshot(fusion_state_t *out);
