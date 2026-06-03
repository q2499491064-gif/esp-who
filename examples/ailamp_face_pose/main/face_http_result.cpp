#include "face_http_result.hpp"

#include <algorithm>
#include <cmath>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <list>
#include <new>

#include "camera_capture_lock.hpp"
#include "dl_image_jpeg.hpp"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "face_pose.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "human_face_detect.hpp"
#include "pose_filter.hpp"

static const char *TAG = "stage2c";
static constexpr int FACE_MAX_COUNT = 4;
static constexpr uint32_t LOW_HEAP_WARN_BYTES = 60 * 1024;

#ifndef CONFIG_AILAMP_FACE_DETECT_INTERVAL_MS
#define CONFIG_AILAMP_FACE_DETECT_INTERVAL_MS 200
#endif

#ifndef CONFIG_AILAMP_RESULT_HOLD_MS
#define CONFIG_AILAMP_RESULT_HOLD_MS 800
#endif

#ifndef CONFIG_AILAMP_RESULT_LOST_MAX
#define CONFIG_AILAMP_RESULT_LOST_MAX 5
#endif

#ifndef CONFIG_AILAMP_LOG_PERIOD_MS
#define CONFIG_AILAMP_LOG_PERIOD_MS 1000
#endif

#ifndef CONFIG_AILAMP_MIN_FACE_SCORE
#define CONFIG_AILAMP_MIN_FACE_SCORE 55
#endif

static constexpr int FACE_DETECT_INTERVAL_MS = CONFIG_AILAMP_FACE_DETECT_INTERVAL_MS;
static constexpr int RESULT_HOLD_MS = CONFIG_AILAMP_RESULT_HOLD_MS;
static constexpr int RESULT_LOST_MAX = CONFIG_AILAMP_RESULT_LOST_MAX;
static constexpr int LOG_PERIOD_MS = CONFIG_AILAMP_LOG_PERIOD_MS;
static constexpr float MIN_FACE_SCORE = static_cast<float>(CONFIG_AILAMP_MIN_FACE_SCORE) / 100.0f;

#ifndef CONFIG_AILAMP_STAGE3_FACE_POSE
#define CONFIG_AILAMP_STAGE3_FACE_POSE 1
#endif

#ifndef CONFIG_AILAMP_STAGE4_POSE_FILTER
#define CONFIG_AILAMP_STAGE4_POSE_FILTER 1
#endif

struct FaceBox {
    int x;
    int y;
    int w;
    int h;
    float score;
    FaceLandmarks5 landmarks;
    HeadPose pose;
    PoseRaw raw_pose;
    PoseFiltered filtered_pose;
    PoseRaw calibrated_pose;
    PoseFilterDebug pose_filter;
    HeadPose rough_pose;
    bool rough_pose_valid;
    const char *reason;
};

struct LatestFaceResult {
    bool valid;
    int frame_width;
    int frame_height;
    int face_count;
    int actual_face_count;
    int display_face_count;
    const char *tracking_state;
    int lost_count;
    int hold_ms;
    int face_hit_rate;
    int capture_ms;
    int decode_ms;
    int detect_ms;
    int landmark_ms;
    int pose_ms;
    int total_ms;
    int stream_fps;
    int detect_hz;
    int inference_ms;
    uint32_t frame_id;
    uint32_t free_heap;
    PoseRaw raw_pose;
    PoseFiltered filtered_pose;
    PoseRaw calibrated_pose;
    PoseFilterDebug pose_filter;
    FaceBox faces[FACE_MAX_COUNT];
};

static SemaphoreHandle_t s_result_mutex = nullptr;
static LatestFaceResult s_latest_result = {};
static LatestFaceResult s_last_valid_result = {};
static int s_result_lost_count = 0;
static int64_t s_last_valid_us = 0;
static int s_stream_fps = 0;
static uint8_t *s_jpeg_copy = nullptr;
static size_t s_jpeg_copy_capacity = 0;
#if CONFIG_AILAMP_STAGE4_POSE_FILTER
static PoseEmaFilter s_pose_filter;
#endif

static bool ensure_jpeg_copy_capacity(size_t required)
{
    if (required <= s_jpeg_copy_capacity) {
        return true;
    }

    uint8_t *new_buf = static_cast<uint8_t *>(heap_caps_malloc(required, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (new_buf == nullptr) {
        new_buf = static_cast<uint8_t *>(heap_caps_malloc(required, MALLOC_CAP_8BIT));
    }
    if (new_buf == nullptr) {
        ESP_LOGE(TAG, "[stage2c] jpeg copy alloc failed: %u bytes", static_cast<unsigned>(required));
        return false;
    }

    if (s_jpeg_copy != nullptr) {
        heap_caps_free(s_jpeg_copy);
    }
    s_jpeg_copy = new_buf;
    s_jpeg_copy_capacity = required;
    return true;
}

static void publish_result(const LatestFaceResult &result)
{
    xSemaphoreTake(s_result_mutex, portMAX_DELAY);
    s_latest_result = result;
    xSemaphoreGive(s_result_mutex);
}

void face_http_result_set_stream_stats(int stream_fps)
{
    if (s_result_mutex == nullptr) {
        return;
    }
    xSemaphoreTake(s_result_mutex, portMAX_DELAY);
    s_stream_fps = stream_fps;
    s_latest_result.stream_fps = stream_fps;
    xSemaphoreGive(s_result_mutex);
}

static void publish_display_result(const LatestFaceResult &raw_result)
{
    LatestFaceResult display = raw_result;
    display.actual_face_count = raw_result.face_count;
    display.display_face_count = raw_result.face_count;
    display.stream_fps = s_stream_fps;

    const int64_t now_us = esp_timer_get_time();
    if (raw_result.face_count > 0) {
        s_result_lost_count = 0;
        s_last_valid_us = now_us;
        s_last_valid_result = raw_result;
        s_last_valid_result.actual_face_count = raw_result.face_count;
        s_last_valid_result.display_face_count = raw_result.face_count;
        s_last_valid_result.tracking_state = "tracking";

        display.tracking_state = "tracking";
        display.lost_count = 0;
        display.hold_ms = 0;
    } else {
        s_result_lost_count++;
        const int hold_ms = s_last_valid_us > 0 ? static_cast<int>((now_us - s_last_valid_us) / 1000) : 0;
        const bool can_hold = s_last_valid_result.valid && (s_result_lost_count <= RESULT_LOST_MAX || hold_ms < RESULT_HOLD_MS);
        if (can_hold) {
            display = s_last_valid_result;
            display.frame_id = raw_result.frame_id;
            display.inference_ms = raw_result.inference_ms;
            display.capture_ms = raw_result.capture_ms;
            display.decode_ms = raw_result.decode_ms;
            display.detect_ms = raw_result.detect_ms;
            display.landmark_ms = raw_result.landmark_ms;
            display.pose_ms = raw_result.pose_ms;
            display.total_ms = raw_result.total_ms;
            display.detect_hz = raw_result.detect_hz;
            display.stream_fps = s_stream_fps;
            display.free_heap = raw_result.free_heap;
            display.actual_face_count = 0;
            display.display_face_count = s_last_valid_result.face_count;
            display.face_count = s_last_valid_result.face_count;
            display.tracking_state = "hold";
            display.lost_count = s_result_lost_count;
            display.hold_ms = hold_ms;
        } else {
            display.face_count = 0;
            display.actual_face_count = 0;
            display.display_face_count = 0;
            display.tracking_state = "lost";
            display.lost_count = s_result_lost_count;
            display.hold_ms = hold_ms;
            display.raw_pose = {};
            display.filtered_pose = raw_result.filtered_pose;
            display.calibrated_pose = raw_result.calibrated_pose;
            display.pose_filter = raw_result.pose_filter;
        }
    }

    publish_result(display);
}

static void publish_empty_result(uint32_t frame_id, int width, int height, int inference_ms)
{
    LatestFaceResult result = {};
    result.valid = true;
    result.frame_id = frame_id;
    result.frame_width = width;
    result.frame_height = height;
    result.actual_face_count = 0;
    result.display_face_count = 0;
    result.tracking_state = "lost";
    result.inference_ms = inference_ms;
    result.total_ms = inference_ms;
    result.free_heap = esp_get_free_heap_size();
#if CONFIG_AILAMP_STAGE4_POSE_FILTER
    result.filtered_pose = s_pose_filter.update({}, false);
    result.calibrated_pose = pose_calibration_apply(result.filtered_pose);
    result.pose_filter = s_pose_filter.debug();
#endif
    publish_display_result(result);
}

static bool nose_inside_face_box(const FaceLandmarks5 &lm, const FaceBox &face)
{
    return lm.nose_x >= static_cast<float>(face.x) && lm.nose_y >= static_cast<float>(face.y) &&
           lm.nose_x <= static_cast<float>(face.x + face.w) &&
           lm.nose_y <= static_cast<float>(face.y + face.h);
}

static HeadPose estimate_rough_pose_from_box(const FaceBox &face, int frame_width, int frame_height)
{
    HeadPose pose = {};
    if (frame_width <= 0 || frame_height <= 0 || face.w <= 0 || face.h <= 0) {
        return pose;
    }

    const float face_cx = static_cast<float>(face.x) + static_cast<float>(face.w) * 0.5f;
    const float face_cy = static_cast<float>(face.y) + static_cast<float>(face.h) * 0.5f;
    const float frame_cx = static_cast<float>(frame_width) * 0.5f;
    const float frame_cy = static_cast<float>(frame_height) * 0.5f;

    pose.valid = true;
    pose.pitch_deg = ((face_cy - frame_cy) / frame_cy) * 30.0f;
    pose.yaw_deg = ((face_cx - frame_cx) / frame_cx) * 35.0f;
    pose.roll_deg = 0.0f;
    return pose;
}

static FaceLandmarks5 landmarks_from_detection(const dl::detect::result_t &det)
{
    FaceLandmarks5 lm = {};
    if (det.keypoint.size() != 10) {
        return lm;
    }

    // ESP-WHO print_detect_results labels keypoint pairs in this order:
    // left_eye, left_mouth, nose, right_eye, right_mouth.
    lm.valid = true;
    lm.left_eye_x = det.keypoint[0];
    lm.left_eye_y = det.keypoint[1];
    lm.left_mouth_x = det.keypoint[2];
    lm.left_mouth_y = det.keypoint[3];
    lm.nose_x = det.keypoint[4];
    lm.nose_y = det.keypoint[5];
    lm.right_eye_x = det.keypoint[6];
    lm.right_eye_y = det.keypoint[7];
    lm.right_mouth_x = det.keypoint[8];
    lm.right_mouth_y = det.keypoint[9];
    return lm;
}

static void face_detect_task(void *arg)
{
    auto *model = static_cast<HumanFaceDetect *>(arg);
    uint32_t frame_id = 0;
    int64_t last_log_us = 0;
    int64_t detect_window_us = esp_timer_get_time();
    int detect_window_count = 0;
    int hit_window_count = 0;
    bool was_tracking = false;

    ESP_LOGI(TAG, "[stage2c] face_detect_task started");

    while (true) {
        const int64_t start_us = esp_timer_get_time();
        int capture_ms = 0;
        int decode_ms = 0;
        int detect_ms = 0;
        int landmark_ms = 0;
        int pose_ms = 0;
        int detect_hz = 0;
        int face_hit_rate = 0;
        int width = 0;
        int height = 0;
        size_t jpeg_len = 0;

        const int64_t capture_start_us = esp_timer_get_time();
        camera_fb_t *fb = camera_capture_get();
        if (fb == nullptr) {
            ESP_LOGW(TAG, "[stage2c] face detect frame capture failed");
            vTaskDelay(pdMS_TO_TICKS(FACE_DETECT_INTERVAL_MS));
            continue;
        }

        if (fb->format == PIXFORMAT_JPEG && ensure_jpeg_copy_capacity(fb->len)) {
            std::memcpy(s_jpeg_copy, fb->buf, fb->len);
            jpeg_len = fb->len;
            width = fb->width;
            height = fb->height;
        } else {
            ESP_LOGW(TAG, "[stage2c] unsupported frame format for detect: %d", fb->format);
        }
        camera_capture_return(fb);
        capture_ms = static_cast<int>((esp_timer_get_time() - capture_start_us) / 1000);

        frame_id++;
        if (jpeg_len == 0) {
            publish_empty_result(frame_id, width, height, capture_ms);
            vTaskDelay(pdMS_TO_TICKS(FACE_DETECT_INTERVAL_MS));
            continue;
        }

        const int64_t decode_start_us = esp_timer_get_time();
#if CONFIG_IDF_TARGET_ESP32P4
        uint32_t caps = 0;
#else
        uint32_t caps = dl::image::DL_IMAGE_CAP_RGB565_BIG_ENDIAN;
#endif
#if CONFIG_SOC_JPEG_CODEC_SUPPORTED
        dl::image::img_t img =
            dl::image::hw_decode_jpeg({s_jpeg_copy, jpeg_len}, dl::image::DL_IMAGE_PIX_TYPE_RGB565, caps);
#else
        dl::image::img_t img =
            dl::image::sw_decode_jpeg({s_jpeg_copy, jpeg_len}, dl::image::DL_IMAGE_PIX_TYPE_RGB565, caps);
#endif
        decode_ms = static_cast<int>((esp_timer_get_time() - decode_start_us) / 1000);
        if (img.data == nullptr) {
            ESP_LOGW(TAG, "[stage2c] jpeg decode failed");
            publish_empty_result(frame_id, width, height, capture_ms + decode_ms);
            vTaskDelay(pdMS_TO_TICKS(FACE_DETECT_INTERVAL_MS));
            continue;
        }

        const int64_t detect_start_us = esp_timer_get_time();
        auto &detections = model->run(img);
        detect_ms = static_cast<int>((esp_timer_get_time() - detect_start_us) / 1000);
        LatestFaceResult result = {};
        result.valid = true;
        result.frame_id = frame_id;
        result.frame_width = img.width;
        result.frame_height = img.height;
        result.free_heap = esp_get_free_heap_size();

        bool landmark_available = false;
        const char *pose_invalid_reason = nullptr;
        const int64_t landmark_start_us = esp_timer_get_time();
        for (const auto &det : detections) {
            if (result.face_count >= FACE_MAX_COUNT || det.box.size() < 4) {
                continue;
            }
            const int x1 = std::max(0, det.box[0]);
            const int y1 = std::max(0, det.box[1]);
            const int x2 = std::min(static_cast<int>(img.width) - 1, det.box[2]);
            const int y2 = std::min(static_cast<int>(img.height) - 1, det.box[3]);
            FaceBox face = {
                .x = x1,
                .y = y1,
                .w = std::max(0, x2 - x1),
                .h = std::max(0, y2 - y1),
                .score = det.score,
                .landmarks = {},
                .pose = {},
                .raw_pose = {},
                .filtered_pose = {},
                .calibrated_pose = {},
                .pose_filter = {},
                .rough_pose = {},
                .rough_pose_valid = false,
                .reason = nullptr,
            };
#if CONFIG_AILAMP_STAGE3_FACE_POSE
            face.landmarks = landmarks_from_detection(det);
            landmark_available = landmark_available || face.landmarks.valid;
            if (det.score < MIN_FACE_SCORE) {
                face.reason = "low_score";
                pose_invalid_reason = face.reason;
            } else if (face.landmarks.valid && !nose_inside_face_box(face.landmarks, face)) {
                face.reason = "bad_landmarks";
                pose_invalid_reason = face.reason;
            } else if (face.landmarks.valid) {
                const int64_t pose_start_us = esp_timer_get_time();
                face.pose = estimate_head_pose_from_5pts(face.landmarks, img.width, img.height);
                pose_ms += static_cast<int>((esp_timer_get_time() - pose_start_us) / 1000);
                if (!face.pose.valid) {
                    face.reason = "bad_landmarks";
                    pose_invalid_reason = face.reason;
                } else {
                    face.raw_pose = {true, face.pose.pitch_deg, face.pose.yaw_deg, face.pose.roll_deg};
                    if (!result.raw_pose.valid) {
                        result.raw_pose = face.raw_pose;
                    }
                }
            } else {
                face.reason = "no landmarks in current detector result";
                pose_invalid_reason = face.reason;
                face.rough_pose = estimate_rough_pose_from_box(face, img.width, img.height);
                face.rough_pose_valid = face.rough_pose.valid;
            }
#endif
            result.faces[result.face_count++] = face;
        }
        landmark_ms = static_cast<int>((esp_timer_get_time() - landmark_start_us) / 1000) - pose_ms;
        if (landmark_ms < 0) {
            landmark_ms = 0;
        }
        detect_window_count++;
        if (result.face_count > 0) {
            hit_window_count++;
        }
        const int64_t window_elapsed_us = esp_timer_get_time() - detect_window_us;
        if (window_elapsed_us >= 1000000) {
            detect_hz = static_cast<int>((detect_window_count * 1000000LL) / window_elapsed_us);
            face_hit_rate = detect_window_count > 0 ? (hit_window_count * 100) / detect_window_count : 0;
            detect_window_us = esp_timer_get_time();
            detect_window_count = 0;
            hit_window_count = 0;
        } else {
            detect_hz = detect_window_count;
            face_hit_rate = detect_window_count > 0 ? (hit_window_count * 100) / detect_window_count : 0;
        }

        result.actual_face_count = result.face_count;
        result.display_face_count = result.face_count;
        result.tracking_state = result.face_count > 0 ? "tracking" : "lost";
        result.face_hit_rate = face_hit_rate;
        result.capture_ms = capture_ms;
        result.decode_ms = decode_ms;
        result.detect_ms = detect_ms;
        result.landmark_ms = landmark_ms;
        result.pose_ms = pose_ms;
        result.total_ms = static_cast<int>((esp_timer_get_time() - start_us) / 1000);
        result.inference_ms = result.detect_ms;
        result.detect_hz = detect_hz;
        result.stream_fps = s_stream_fps;

#if CONFIG_AILAMP_STAGE4_POSE_FILTER
        const bool update_pose_filter = result.raw_pose.valid || s_result_lost_count >= RESULT_LOST_MAX;
        if (update_pose_filter) {
            result.filtered_pose = s_pose_filter.update(result.raw_pose, result.face_count > 0);
        } else {
            result.filtered_pose = s_pose_filter.get();
        }
        result.calibrated_pose = pose_calibration_apply(result.filtered_pose);
        result.pose_filter = s_pose_filter.debug();
        if (result.face_count > 0) {
            result.faces[0].pose_filter = result.pose_filter;
            for (int i = 0; i < result.face_count; i++) {
                if (result.faces[i].raw_pose.valid) {
                    result.faces[i].filtered_pose = result.filtered_pose;
                    result.faces[i].calibrated_pose = result.calibrated_pose;
                    result.faces[i].pose_filter = result.pose_filter;
                    break;
                }
            }
        }
#endif
        publish_display_result(result);
        heap_caps_free(img.data);

        if (result.face_count > 0 && !was_tracking) {
            ESP_LOGI(TAG, "[face] state=tracking face_count=%d score=%.2f", result.face_count, result.faces[0].score);
        } else if (result.face_count == 0 && was_tracking) {
            ESP_LOGI(TAG, "[face] state=lost lost_count=%d keep_last=1", s_result_lost_count + 1);
        }
        was_tracking = result.face_count > 0;

        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_log_us >= LOG_PERIOD_MS * 1000LL) {
            last_log_us = now_us;
            ESP_LOGI(TAG,
                     "[perf] stream_fps=%d detect_hz=%d face_hit_rate=%d%% capture_ms=%d decode_ms=%d detect_ms=%d landmark_ms=%d pose_ms=%d total_ms=%d free_heap=%" PRIu32,
                     s_stream_fps,
                     result.detect_hz,
                     result.face_hit_rate,
                     result.capture_ms,
                     result.decode_ms,
                     result.detect_ms,
                     result.landmark_ms,
                     result.pose_ms,
                     result.total_ms,
                     result.free_heap);
            ESP_LOGI(TAG,
                     "[face] state=%s face_count=%d score=%.2f lost_count=%d landmarks=%s pose=%s reason=%s",
                     result.face_count > 0 ? "tracking" : "lost",
                     result.face_count,
                     result.face_count > 0 ? result.faces[0].score : 0.0f,
                     s_result_lost_count,
                     landmark_available ? "true" : "false",
                     result.raw_pose.valid ? "true" : "false",
                     pose_invalid_reason != nullptr ? pose_invalid_reason : "none");
            uint32_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
            if (free_heap < LOW_HEAP_WARN_BYTES) {
                ESP_LOGW(TAG, "[stage2c] low heap warning: free_heap=%" PRIu32, free_heap);
            }
        }

        const int elapsed_ms = static_cast<int>((esp_timer_get_time() - start_us) / 1000);
        if (elapsed_ms < FACE_DETECT_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(FACE_DETECT_INTERVAL_MS - elapsed_ms));
        } else {
            taskYIELD();
        }
    }
}

esp_err_t face_http_result_start()
{
    if (s_result_mutex == nullptr) {
        s_result_mutex = xSemaphoreCreateMutex();
        if (s_result_mutex == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_LOGI(TAG, "[stage2c] face detect model init start");
    auto *model = new (std::nothrow) HumanFaceDetect();
    if (model == nullptr) {
        ESP_LOGE(TAG, "[stage2c] face detect model alloc failed");
        return ESP_ERR_NO_MEM;
    }
    model->set_score_thr(MIN_FACE_SCORE, 0);
    ESP_LOGI(TAG, "[stage2c] face detect model init success");

    BaseType_t ok = xTaskCreatePinnedToCore(face_detect_task, "face_detect_task", 12 * 1024, model, 5, nullptr, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void face_http_result_summary(uint32_t *frame_id, int *face_count, int *inference_ms)
{
    LatestFaceResult result = {};
    if (s_result_mutex != nullptr) {
        xSemaphoreTake(s_result_mutex, portMAX_DELAY);
        result = s_latest_result;
        xSemaphoreGive(s_result_mutex);
    }

    if (frame_id != nullptr) {
        *frame_id = result.frame_id;
    }
    if (face_count != nullptr) {
        *face_count = result.face_count;
    }
    if (inference_ms != nullptr) {
        *inference_ms = result.inference_ms;
    }
}

int face_http_result_json(char *buf, size_t buf_size)
{
    LatestFaceResult result = {};
    if (s_result_mutex != nullptr) {
        xSemaphoreTake(s_result_mutex, portMAX_DELAY);
        result = s_latest_result;
        xSemaphoreGive(s_result_mutex);
    }
    if (result.free_heap == 0) {
        result.free_heap = esp_get_free_heap_size();
    }
    int offset = std::snprintf(buf,
                               buf_size,
                               "{\n"
                               "  \"valid\": %s,\n"
                               "  \"frame_id\": %" PRIu32 ",\n"
                               "  \"frame_width\": %d,\n"
                               "  \"frame_height\": %d,\n"
                               "  \"actual_face_count\": %d,\n"
                               "  \"display_face_count\": %d,\n"
                               "  \"face_count\": %d,\n"
                               "  \"tracking_state\": \"%s\",\n"
                               "  \"lost_count\": %d,\n"
                               "  \"hold_ms\": %d,\n"
                               "  \"face_hit_rate\": %d,\n"
                               "  \"inference_ms\": %d,\n"
                               "  \"capture_ms\": %d,\n"
                               "  \"decode_ms\": %d,\n"
                               "  \"detect_ms\": %d,\n"
                               "  \"landmark_ms\": %d,\n"
                               "  \"pose_ms\": %d,\n"
                               "  \"total_ms\": %d,\n"
                               "  \"stream_fps\": %d,\n"
                               "  \"detect_hz\": %d,\n"
                               "  \"free_heap\": %" PRIu32 ",\n"
                               "  \"raw_pose_valid\": %s,\n"
                               "  \"filtered_pose_valid\": %s,\n"
                               "  \"calibrated_pose_valid\": %s",
                               result.valid ? "true" : "false",
                               result.frame_id,
                               result.frame_width,
                               result.frame_height,
                               result.actual_face_count,
                               result.display_face_count,
                               result.face_count,
                               result.tracking_state != nullptr ? result.tracking_state : "lost",
                               result.lost_count,
                               result.hold_ms,
                               result.face_hit_rate,
                               result.inference_ms,
                               result.capture_ms,
                               result.decode_ms,
                               result.detect_ms,
                               result.landmark_ms,
                               result.pose_ms,
                               result.total_ms,
                               result.stream_fps,
                               result.detect_hz,
                               result.free_heap,
                               result.raw_pose.valid ? "true" : "false",
                               result.filtered_pose.valid ? "true" : "false",
                               result.calibrated_pose.valid ? "true" : "false");

    if (result.raw_pose.valid && offset > 0 && static_cast<size_t>(offset) < buf_size) {
        offset += std::snprintf(buf + offset,
                                buf_size - offset,
                                ",\n"
                                "  \"raw_pose\": {\"pitch\": %.2f, \"yaw\": %.2f, \"roll\": %.2f}",
                                result.raw_pose.pitch,
                                result.raw_pose.yaw,
                                result.raw_pose.roll);
    }
    if (result.filtered_pose.valid && offset > 0 && static_cast<size_t>(offset) < buf_size) {
        offset += std::snprintf(buf + offset,
                                buf_size - offset,
                                ",\n"
                                "  \"filtered_pose\": {\"pitch\": %.2f, \"yaw\": %.2f, \"roll\": %.2f}",
                                result.filtered_pose.pitch,
                                result.filtered_pose.yaw,
                                result.filtered_pose.roll);
    }
    if (result.calibrated_pose.valid && offset > 0 && static_cast<size_t>(offset) < buf_size) {
        offset += std::snprintf(buf + offset,
                                buf_size - offset,
                                ",\n"
                                "  \"calibrated_pose\": {\"pitch\": %.2f, \"yaw\": %.2f, \"roll\": %.2f}",
                                result.calibrated_pose.pitch,
                                result.calibrated_pose.yaw,
                                result.calibrated_pose.roll);
    }
    if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
        offset += std::snprintf(buf + offset,
                                buf_size - offset,
                                ",\n"
                                "  \"pose_filter\": {\"initialized\": %s, \"lost_count\": %d, \"reset_count\": %d, "
                                "\"last_reset_reason\": \"%s\", \"alpha\": %.2f, \"max_delta\": %.2f}",
                                result.pose_filter.initialized ? "true" : "false",
                                result.pose_filter.lost_count,
                                result.pose_filter.reset_count,
                                result.pose_filter.last_reset_reason != nullptr ? result.pose_filter.last_reset_reason
                                                                                : "none",
                                result.pose_filter.alpha,
                                result.pose_filter.max_delta);
    }
    if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
        offset += std::snprintf(buf + offset, buf_size - offset, ",\n  \"faces\": [");
    }

    for (int i = 0; i < result.face_count && offset > 0 && static_cast<size_t>(offset) < buf_size; i++) {
        const auto &face = result.faces[i];
        const int x2 = face.x + face.w;
        const int y2 = face.y + face.h;
        // Stage 4A currently serializes up to FACE_MAX_COUNT detections, but the UI highlights the first face pose.
        offset += std::snprintf(buf + offset,
                                buf_size - offset,
                                "%s\n"
                                "    {\"x1\": %d, \"y1\": %d, \"x2\": %d, \"y2\": %d, \"w\": %d, \"h\": %d, \"score\": %.3f",
                                i == 0 ? "" : ",",
                                face.x,
                                face.y,
                                x2,
                                y2,
                                face.w,
                                face.h,
                                face.score);
#if CONFIG_AILAMP_STAGE3_FACE_POSE
        if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"landmarks_valid\": %s",
                                    face.landmarks.valid ? "true" : "false");
        }
        if (face.landmarks.valid && offset > 0 && static_cast<size_t>(offset) < buf_size) {
            const auto &lm = face.landmarks;
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"landmarks\": {"
                                    "\"left_eye\": {\"x\": %.1f, \"y\": %.1f}, "
                                    "\"right_eye\": {\"x\": %.1f, \"y\": %.1f}, "
                                    "\"nose\": {\"x\": %.1f, \"y\": %.1f}, "
                                    "\"left_mouth\": {\"x\": %.1f, \"y\": %.1f}, "
                                    "\"right_mouth\": {\"x\": %.1f, \"y\": %.1f}}",
                                    lm.left_eye_x,
                                    lm.left_eye_y,
                                    lm.right_eye_x,
                                    lm.right_eye_y,
                                    lm.nose_x,
                                    lm.nose_y,
                                    lm.left_mouth_x,
                                    lm.left_mouth_y,
                                    lm.right_mouth_x,
                                    lm.right_mouth_y);
        }
        if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"pose_valid\": %s, \"raw_pose_valid\": %s",
                                    face.pose.valid ? "true" : "false",
                                    face.pose.valid ? "true" : "false");
        }
        if (face.pose.valid && offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"pose\": {\"pitch\": %.2f, \"yaw\": %.2f, \"roll\": %.2f}"
                                    ", \"raw_pose\": {\"pitch\": %.2f, \"yaw\": %.2f, \"roll\": %.2f}",
                                    face.pose.pitch_deg,
                                    face.pose.yaw_deg,
                                    face.pose.roll_deg,
                                    face.raw_pose.pitch,
                                    face.raw_pose.yaw,
                                    face.raw_pose.roll);
        } else if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"reason\": \"%s\"",
                                    face.reason != nullptr ? face.reason : "pose unavailable");
        }
#if CONFIG_AILAMP_STAGE4_POSE_FILTER
        if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"filtered_pose_valid\": %s",
                                    face.filtered_pose.valid ? "true" : "false");
        }
        if (face.filtered_pose.valid && offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"filtered_pose\": {\"pitch\": %.2f, \"yaw\": %.2f, \"roll\": %.2f}",
                                    face.filtered_pose.pitch,
                                    face.filtered_pose.yaw,
                                    face.filtered_pose.roll);
        }
        if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"calibrated_pose_valid\": %s",
                                    face.calibrated_pose.valid ? "true" : "false");
        }
        if (face.calibrated_pose.valid && offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"calibrated_pose\": {\"pitch\": %.2f, \"yaw\": %.2f, \"roll\": %.2f}",
                                    face.calibrated_pose.pitch,
                                    face.calibrated_pose.yaw,
                                    face.calibrated_pose.roll);
        }
        if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"pose_filter\": {\"initialized\": %s, \"lost_count\": %d, "
                                    "\"reset_count\": %d, \"last_reset_reason\": \"%s\", \"alpha\": %.2f, "
                                    "\"max_delta\": %.2f}",
                                    face.pose_filter.initialized ? "true" : "false",
                                    face.pose_filter.lost_count,
                                    face.pose_filter.reset_count,
                                    face.pose_filter.last_reset_reason != nullptr ? face.pose_filter.last_reset_reason
                                                                                  : "none",
                                    face.pose_filter.alpha,
                                    face.pose_filter.max_delta);
        }
#endif
        if (face.rough_pose_valid && offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset,
                                    buf_size - offset,
                                    ", \"rough_pose\": {\"pitch\": %.2f, \"yaw\": %.2f, \"roll\": %.2f}",
                                    face.rough_pose.pitch_deg,
                                    face.rough_pose.yaw_deg,
                                    face.rough_pose.roll_deg);
        }
#endif
        if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
            offset += std::snprintf(buf + offset, buf_size - offset, "}");
        }
    }

    if (offset > 0 && static_cast<size_t>(offset) < buf_size) {
        offset += std::snprintf(buf + offset, buf_size - offset, "\n  ]\n}\n");
    }
    return offset;
}
