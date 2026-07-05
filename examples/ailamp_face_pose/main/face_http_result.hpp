#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

static constexpr int FACE_HTTP_SNAPSHOT_MAX_FACES = 4;

struct face_http_point_snapshot_t {
    float x;
    float y;
};

struct face_http_landmarks_snapshot_t {
    face_http_point_snapshot_t left_eye;
    face_http_point_snapshot_t right_eye;
    face_http_point_snapshot_t nose;
    face_http_point_snapshot_t left_mouth;
    face_http_point_snapshot_t right_mouth;
};

struct face_http_face_snapshot_t {
    int x1;
    int y1;
    int x2;
    int y2;
    int w;
    int h;
    float score;
    bool landmarks_valid;
    face_http_landmarks_snapshot_t landmarks;
};

struct face_http_snapshot_t {
    bool valid;
    uint32_t timestamp_ms;
    uint32_t frame_id;
    int frame_width;
    int frame_height;
    int face_count;
    int actual_face_count;
    int display_face_count;
    const char *tracking_state;
    float pitch;
    float yaw;
    float roll;
    float filtered_pitch;
    float filtered_yaw;
    float filtered_roll;
    float face_score;
    float face_center_x_norm;
    float face_center_y_norm;
    int inference_ms;
    float hit_rate;
    bool pose_valid;
    bool filtered_pose_valid;
    int overlay_face_count;
    face_http_face_snapshot_t faces[FACE_HTTP_SNAPSHOT_MAX_FACES];
};

esp_err_t face_http_result_start();
int face_http_result_json(char *buf, size_t buf_size);
void face_http_result_summary(uint32_t *frame_id, int *face_count, int *inference_ms);
void face_http_result_set_stream_stats(int stream_fps);
bool face_http_result_get_snapshot(face_http_snapshot_t *out);
