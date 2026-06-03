#include "http_stream_server.hpp"

#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "camera_capture_lock.hpp"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "face_http_result.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "range_sensor.hpp"

static const char *TAG = "preview_http";
static constexpr const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static constexpr const char *STREAM_BOUNDARY = "--frame\r\n";
static constexpr const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

#ifndef CONFIG_AILAMP_STREAM_TARGET_FPS
#define CONFIG_AILAMP_STREAM_TARGET_FPS 8
#endif

#ifndef CONFIG_AILAMP_LOG_PERIOD_MS
#define CONFIG_AILAMP_LOG_PERIOD_MS 1000
#endif

static constexpr int STREAM_TARGET_FPS = CONFIG_AILAMP_STREAM_TARGET_FPS;
static constexpr int STREAM_FRAME_PERIOD_MS = 1000 / STREAM_TARGET_FPS;
static constexpr int STREAM_LOG_PERIOD_MS = 2000;

static uint32_t s_stream_count = 0;
static size_t s_last_frame_len = 0;
static bool s_camera_ok = false;

static esp_err_t index_handler(httpd_req_t *req)
{
    static constexpr const char *html =
        "<!doctype html>\n"
        "<html>\n"
        "  <head>\n"
        "    <meta charset=\"utf-8\">\n"
        "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
        "    <title>ESP32-S3 Face Preview</title>\n"
        "    <style>\n"
        "      body{font-family:Arial,sans-serif;margin:16px;background:#101214;color:#eee;}\n"
        "      #videoWrap{position:relative;display:inline-block;max-width:100%;}\n"
        "      #stream{display:block;width:640px;max-width:100%;height:auto;background:#000;}\n"
        "      #overlay{position:absolute;left:0;top:0;pointer-events:none;}\n"
        "      .status{margin-top:10px;color:#b8f7d4;font-size:14px;line-height:1.8;}\n"
        "      .pose{margin-top:6px;color:#ffd27d;font-size:15px;line-height:1.8;}\n"
        "      .row span{display:inline-block;min-width:72px;}\n"
        "      #fetchStatus{min-width:160px;color:#ffcf70;}\n"
        "      a{color:#8ecbff;}\n"
        "    </style>\n"
        "  </head>\n"
        "  <body>\n"
        "    <h2>ESP32-S3 OV2640 Face Preview</h2>\n"
        "    <div id=\"videoWrap\" style=\"position:relative;display:inline-block;\">\n"
        "      <img id=\"stream\" src=\"\" style=\"display:block;width:640px;max-width:100%;\">\n"
        "      <canvas id=\"overlay\" style=\"position:absolute;left:0;top:0;pointer-events:none;\"></canvas>\n"
        "    </div>\n"
        "    <div class=\"status\">\n"
        "      <div class=\"row\">Face Count: <span id=\"faceCount\">--</span> Inference ms: <span id=\"inferenceMs\">--</span></div>\n"
        "      <div class=\"row\">Actual: <span id=\"actualFaceCount\">--</span> Display: <span id=\"displayFaceCount\">--</span> State: <span id=\"trackingState\">--</span></div>\n"
        "      <div class=\"row\">Capture: <span id=\"captureMs\">--</span> Decode: <span id=\"decodeMs\">--</span> Detect: <span id=\"detectMs\">--</span> Landmark: <span id=\"landmarkMs\">--</span> Pose: <span id=\"poseMs\">--</span> Total: <span id=\"totalMs\">--</span></div>\n"
        "      <div class=\"row\">Stream FPS: <span id=\"streamFps\">--</span> Detect Hz: <span id=\"detectHz\">--</span> Hit Rate: <span id=\"faceHitRate\">--</span> Hold ms: <span id=\"holdMs\">--</span></div>\n"
        "      <div class=\"row\">Frame ID: <span id=\"frameId\">--</span> Free Heap: <span id=\"freeHeap\">--</span></div>\n"
        "      <div class=\"row\">Distance: <span id=\"rangeDistance\">--</span> mm Range Status: <span id=\"rangeStatus\">--</span> Range Valid: <span id=\"rangeValid\">--</span></div>\n"
        "      <div class=\"row\">Status: <span id=\"fetchStatus\">starting</span></div>\n"
        "    </div>\n"
        "    <div class=\"pose\">\n"
        "      <div class=\"row\">Raw Pitch: <span id=\"rawPitch\">--</span> Raw Yaw: <span id=\"rawYaw\">--</span> Raw Roll: <span id=\"rawRoll\">--</span></div>\n"
        "      <div class=\"row\">Filtered Pitch: <span id=\"filteredPitch\">--</span> Filtered Yaw: <span id=\"filteredYaw\">--</span> Filtered Roll: <span id=\"filteredRoll\">--</span></div>\n"
        "      <div class=\"row\">Calibrated Pitch: <span id=\"calPitch\">--</span> Calibrated Yaw: <span id=\"calYaw\">--</span> Calibrated Roll: <span id=\"calRoll\">--</span></div>\n"
        "      <div class=\"row\">Filter Alpha: <span id=\"filterAlpha\">--</span> Max Delta: <span id=\"maxDelta\">--</span> Lost Count: <span id=\"lostCount\">--</span></div>\n"
        "      <div class=\"row\">Reset Count: <span id=\"resetCount\">--</span> Reset Reason: <span id=\"resetReason\">--</span></div>\n"
        "    </div>\n"
        "    <p><a href=\"/capture\">Capture one JPEG</a></p>\n"
        "    <p><a href=\"/status\">Status JSON</a></p>\n"
        "    <p><a href=\"/face_result\">Face result JSON</a></p>\n"
        "    <script>\n"
        "      const img=document.getElementById('stream');\n"
        "      const canvas=document.getElementById('overlay');\n"
        "      const ctx=canvas.getContext('2d');\n"
        "      const streamUrl='http://'+window.location.hostname+':81/stream';\n"
        "      img.src=streamUrl;\n"
        "      function el(id){return document.getElementById(id);}\n"
        "      function valueOr(v,d){return v===undefined||v===null?d:v;}\n"
        "      function fmt(v){if(v===undefined||v===null||Number.isNaN(Number(v)))return '--';return Number(v).toFixed(2);}\n"
        "      function resizeCanvas(){const w=img.clientWidth||img.naturalWidth||640;const h=img.clientHeight||img.naturalHeight||480;if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}}\n"
        "      function clearOverlay(){resizeCanvas();ctx.clearRect(0,0,canvas.width,canvas.height);}\n"
        "      function drawPoint(x,y,sx,sy,label){const px=x*sx;const py=y*sy;ctx.beginPath();ctx.arc(px,py,4,0,Math.PI*2);ctx.fill();ctx.fillText(label,px+6,py-6);}\n"
        "      function clearPose(){el('rawPitch').textContent='--';el('rawYaw').textContent='--';el('rawRoll').textContent='--';el('filteredPitch').textContent='--';el('filteredYaw').textContent='--';el('filteredRoll').textContent='--';el('calPitch').textContent='--';el('calYaw').textContent='--';el('calRoll').textContent='--';el('filterAlpha').textContent='--';el('maxDelta').textContent='--';el('lostCount').textContent='--';el('resetCount').textContent='--';el('resetReason').textContent='--';}\n"
        "      function showFilter(f){if(!f)return;el('filterAlpha').textContent=fmt(f.alpha);el('maxDelta').textContent=fmt(f.max_delta);el('lostCount').textContent=valueOr(f.lost_count,'--');el('resetCount').textContent=valueOr(f.reset_count,'--');el('resetReason').textContent=valueOr(f.last_reset_reason,'--');}\n"
        "      function drawResult(data){\n"
        "        resizeCanvas();ctx.clearRect(0,0,canvas.width,canvas.height);\n"
        "        const fw=data.frame_width||320;const fh=data.frame_height||240;const sx=canvas.width/fw;const sy=canvas.height/fh;\n"
        "        const state=valueOr(data.tracking_state,'lost');\n"
        "        el('faceCount').textContent=valueOr(data.face_count,0);el('actualFaceCount').textContent=valueOr(data.actual_face_count,0);el('displayFaceCount').textContent=valueOr(data.display_face_count,valueOr(data.face_count,0));el('trackingState').textContent=state;el('inferenceMs').textContent=valueOr(data.inference_ms,0);el('captureMs').textContent=valueOr(data.capture_ms,0);el('decodeMs').textContent=valueOr(data.decode_ms,0);el('detectMs').textContent=valueOr(data.detect_ms,0);el('landmarkMs').textContent=valueOr(data.landmark_ms,0);el('poseMs').textContent=valueOr(data.pose_ms,0);el('totalMs').textContent=valueOr(data.total_ms,0);el('streamFps').textContent=valueOr(data.stream_fps,0);el('detectHz').textContent=valueOr(data.detect_hz,0);el('faceHitRate').textContent=valueOr(data.face_hit_rate,0)+'%';el('holdMs').textContent=valueOr(data.hold_ms,0);el('frameId').textContent=valueOr(data.frame_id,0);el('freeHeap').textContent=valueOr(data.free_heap,0);\n"
        "        if(state==='lost'||!data.faces||data.faces.length===0){el('fetchStatus').textContent='no face';clearPose();showFilter(data.pose_filter);return;}\n"
        "        el('fetchStatus').textContent=state==='hold'?'hold':'face detected';clearPose();ctx.font='14px Arial';\n"
        "        for(const face of data.faces){\n"
        "          const x1=valueOr(face.x1,0);const y1=valueOr(face.y1,0);const w=valueOr(face.w,valueOr(face.x2,0)-x1);const h=valueOr(face.h,valueOr(face.y2,0)-y1);\n"
        "          const color=state==='hold'?'#ffd84d':'#00ff00';ctx.lineWidth=2;ctx.strokeStyle=color;ctx.fillStyle=color;ctx.strokeRect(x1*sx,y1*sy,w*sx,h*sy);if(state==='hold'){ctx.fillText('hold',x1*sx+4,y1*sy+16);}\n"
        "          if(face.landmarks_valid&&face.landmarks){drawPoint(face.landmarks.left_eye.x,face.landmarks.left_eye.y,sx,sy,'LE');drawPoint(face.landmarks.right_eye.x,face.landmarks.right_eye.y,sx,sy,'RE');drawPoint(face.landmarks.nose.x,face.landmarks.nose.y,sx,sy,'N');drawPoint(face.landmarks.left_mouth.x,face.landmarks.left_mouth.y,sx,sy,'LM');drawPoint(face.landmarks.right_mouth.x,face.landmarks.right_mouth.y,sx,sy,'RM');}\n"
        "          if(face.raw_pose_valid&&face.raw_pose){el('rawPitch').textContent=fmt(face.raw_pose.pitch);el('rawYaw').textContent=fmt(face.raw_pose.yaw);el('rawRoll').textContent=fmt(face.raw_pose.roll);}\n"
        "          if(face.filtered_pose_valid&&face.filtered_pose){el('filteredPitch').textContent=fmt(face.filtered_pose.pitch);el('filteredYaw').textContent=fmt(face.filtered_pose.yaw);el('filteredRoll').textContent=fmt(face.filtered_pose.roll);}\n"
        "          if(face.calibrated_pose_valid&&face.calibrated_pose){el('calPitch').textContent=fmt(face.calibrated_pose.pitch);el('calYaw').textContent=fmt(face.calibrated_pose.yaw);el('calRoll').textContent=fmt(face.calibrated_pose.roll);}\n"
        "          showFilter(face.pose_filter||data.pose_filter);\n"
        "        }\n"
        "      }\n"
        "      async function pollResult(){\n"
        "        let resp;\n"
        "        try{resp=await fetch('/face_result?t='+Date.now(),{cache:'no-store'});}catch(e){el('fetchStatus').textContent='fetch failed: '+e.message;console.error(e);clearOverlay();return;}\n"
        "        if(!resp.ok){el('fetchStatus').textContent='fetch failed: HTTP '+resp.status;return;}\n"
        "        let data;\n"
        "        try{data=await resp.json();}catch(e){el('fetchStatus').textContent='fetch failed: '+e.message;console.error(e);clearOverlay();return;}\n"
        "        drawResult(data);\n"
        "      }\n"
        "      async function pollRange(){\n"
        "        try{const resp=await fetch('/range_result?t='+Date.now(),{cache:'no-store'});if(!resp.ok)return;const data=await resp.json();el('rangeDistance').textContent=valueOr(data.distance_mm,'--');el('rangeStatus').textContent=valueOr(data.status_text,'--');el('rangeValid').textContent=valueOr(data.valid,false);}\n"
        "        catch(e){el('rangeStatus').textContent='fetch failed';}\n"
        "      }\n"
        "      img.addEventListener('load',resizeCanvas);window.addEventListener('resize',resizeCanvas);setInterval(pollResult,200);setInterval(pollRange,500);pollResult();pollRange();\n"
        "    </script>\n"
        "  </body>\n"
        "</html>\n";

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static bool ensure_jpeg_buffer(uint8_t **buf, size_t *capacity, size_t required)
{
    if (*buf != nullptr && *capacity >= required) {
        return true;
    }

    uint8_t *new_buf = static_cast<uint8_t *>(heap_caps_malloc(required, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (new_buf == nullptr) {
        new_buf = static_cast<uint8_t *>(heap_caps_malloc(required, MALLOC_CAP_8BIT));
    }
    if (new_buf == nullptr) {
        return false;
    }

    if (*buf != nullptr) {
        heap_caps_free(*buf);
    }
    *buf = new_buf;
    *capacity = required;
    return true;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = camera_capture_get();
    if (fb == nullptr) {
        ESP_LOGE(TAG, "[preview] capture failed");
        return httpd_resp_send_500(req);
    }

    if (fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "[preview] capture frame is not JPEG");
        camera_capture_return(fb);
        return httpd_resp_send_500(req);
    }

    uint8_t *jpeg = nullptr;
    size_t jpeg_capacity = 0;
    if (!ensure_jpeg_buffer(&jpeg, &jpeg_capacity, fb->len)) {
        ESP_LOGE(TAG, "[preview] capture copy alloc failed");
        camera_capture_return(fb);
        return httpd_resp_send_500(req);
    }
    std::memcpy(jpeg, fb->buf, fb->len);
    const size_t jpeg_len = fb->len;
    s_last_frame_len = jpeg_len;
    camera_capture_return(fb);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    esp_err_t res = httpd_resp_send(req, reinterpret_cast<const char *>(jpeg), jpeg_len);

    ESP_LOGI(TAG, "[preview] capture success: len=%u", static_cast<unsigned>(jpeg_len));
    heap_caps_free(jpeg);
    return res;
}

static esp_err_t result_handler(httpd_req_t *req)
{
    if (req == nullptr) {
        return ESP_FAIL;
    }

    static int64_t s_last_result_log_us = 0;
    const int64_t now_us = esp_timer_get_time();
    const bool should_log = now_us - s_last_result_log_us >= 1000000;
    if (should_log) {
        s_last_result_log_us = now_us;
        ESP_LOGI(TAG, "result_handler enter uri=%s", req->uri != nullptr ? req->uri : "(null)");
    }

    esp_err_t err = httpd_resp_set_type(req, "application/json; charset=utf-8");
    if (err != ESP_OK) {
        return err;
    }
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    static constexpr size_t JSON_BUF_SIZE = 8192;
    char *json = static_cast<char *>(heap_caps_malloc(JSON_BUF_SIZE, MALLOC_CAP_8BIT));
    if (json == nullptr) {
        static constexpr const char *fallback_json =
            "{\"valid\":true,\"frame_id\":0,\"frame_width\":320,\"frame_height\":240,"
            "\"inference_ms\":0,\"free_heap\":0,\"faces\":[]}";
        if (should_log) {
            ESP_LOGW(TAG, "result_handler using fallback json: alloc failed");
            ESP_LOGI(TAG, "result_handler send json len=%u", static_cast<unsigned>(std::strlen(fallback_json)));
        }
        return httpd_resp_send(req, fallback_json, HTTPD_RESP_USE_STRLEN);
    }

    int len = face_http_result_json(json, JSON_BUF_SIZE);
    if (len <= 0 || static_cast<size_t>(len) >= JSON_BUF_SIZE) {
        len = std::snprintf(json,
                            JSON_BUF_SIZE,
                            "{\"valid\":true,\"frame_id\":0,\"frame_width\":320,\"frame_height\":240,"
                            "\"inference_ms\":0,\"free_heap\":%u,\"faces\":[]}",
                            static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)));
    }
    if (len <= 0 || static_cast<size_t>(len) >= JSON_BUF_SIZE) {
        heap_caps_free(json);
        return httpd_resp_send_500(req);
    }
    json[JSON_BUF_SIZE - 1] = '\0';

    if (should_log) {
        ESP_LOGI(TAG, "result_handler send json len=%d", static_cast<int>(std::strlen(json)));
    }
    err = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    heap_caps_free(json);
    return err;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char json[160];
    std::snprintf(json,
                  sizeof(json),
                  "{\n"
                  "  \"camera_ok\": %s,\n"
                  "  \"stream_count\": %" PRIu32 ",\n"
                  "  \"last_frame_len\": %u,\n"
                  "  \"free_heap\": %u\n"
                  "}\n",
                  s_camera_ok ? "true" : "false",
                  s_stream_count,
                  static_cast<unsigned>(s_last_frame_len),
                  static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)));

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t range_result_handler(httpd_req_t *req)
{
    range_state_t state = {};
    const bool has_state = range_sensor_get_snapshot(&state);
    if (!has_state) {
        state.status_text = "not_initialized";
        state.status = 255;
    }

    char json[512];
    const int len = std::snprintf(json,
                                  sizeof(json),
                                  "{\n"
                                  "  \"initialized\": %s,\n"
                                  "  \"valid\": %s,\n"
                                  "  \"distance_mm\": %u,\n"
                                  "  \"status\": %u,\n"
                                  "  \"status_text\": \"%s\",\n"
                                  "  \"timestamp_ms\": %" PRIu32 ",\n"
                                  "  \"read_count\": %" PRIu32 ",\n"
                                  "  \"error_count\": %" PRIu32 ",\n"
                                  "  \"consecutive_ok_count\": %" PRIu32 ",\n"
                                  "  \"consecutive_fail_count\": %" PRIu32 ",\n"
                                  "  \"read_ms\": %" PRIu32 "\n"
                                  "}\n",
                                  state.initialized ? "true" : "false",
                                  state.valid ? "true" : "false",
                                  static_cast<unsigned>(state.distance_mm),
                                  static_cast<unsigned>(state.status),
                                  state.status_text != nullptr ? state.status_text : "unknown",
                                  state.timestamp_ms,
                                  state.read_count,
                                  state.error_count,
                                  state.consecutive_ok_count,
                                  state.consecutive_fail_count,
                                  state.read_ms);
    if (len <= 0 || static_cast<size_t>(len) >= sizeof(json)) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, len);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "[preview] stream client connected");
    int64_t last_stream_log_us = esp_timer_get_time();
    uint32_t frames_in_window = 0;
    uint32_t jpeg_len_sum = 0;
    uint32_t dropped = 0;
    uint8_t *jpeg = nullptr;
    size_t jpeg_capacity = 0;

    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    while (true) {
        const int64_t frame_start_us = esp_timer_get_time();
        camera_fb_t *fb = camera_capture_get();
        if (fb == nullptr) {
            ESP_LOGE(TAG, "[preview] stream frame capture failed");
            res = ESP_FAIL;
            break;
        }

        if (fb->format != PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "[preview] stream frame is not JPEG");
            camera_capture_return(fb);
            res = ESP_FAIL;
            break;
        }

        if (!ensure_jpeg_buffer(&jpeg, &jpeg_capacity, fb->len)) {
            ESP_LOGE(TAG, "[preview] stream copy alloc failed");
            camera_capture_return(fb);
            res = ESP_ERR_NO_MEM;
            break;
        }
        std::memcpy(jpeg, fb->buf, fb->len);
        const size_t jpeg_len = fb->len;
        camera_capture_return(fb);

        char part_buf[64];
        const int header_len =
            std::snprintf(part_buf, sizeof(part_buf), STREAM_PART, static_cast<unsigned>(jpeg_len));

        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, std::strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, part_buf, header_len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, reinterpret_cast<const char *>(jpeg), jpeg_len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, "\r\n", 2);
        }

        s_stream_count++;
        s_last_frame_len = jpeg_len;
        frames_in_window++;
        jpeg_len_sum += static_cast<uint32_t>(jpeg_len);

        if (res != ESP_OK) {
            break;
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_stream_log_us >= STREAM_LOG_PERIOD_MS * 1000LL) {
            const int window_ms = static_cast<int>((now_us - last_stream_log_us) / 1000);
            const int fps = window_ms > 0 ? static_cast<int>((frames_in_window * 1000U) / static_cast<uint32_t>(window_ms)) : 0;
            const uint32_t avg_len = frames_in_window > 0 ? jpeg_len_sum / frames_in_window : 0;
            ESP_LOGI(TAG, "[preview] fps=%d avg_jpeg_len=%" PRIu32 " dropped=%" PRIu32, fps, avg_len, dropped);
            face_http_result_set_stream_stats(fps);
            frames_in_window = 0;
            jpeg_len_sum = 0;
            dropped = 0;
            last_stream_log_us = now_us;
        }

        const int elapsed_ms = static_cast<int>((esp_timer_get_time() - frame_start_us) / 1000);
        if (elapsed_ms < STREAM_FRAME_PERIOD_MS) {
            vTaskDelay(pdMS_TO_TICKS(STREAM_FRAME_PERIOD_MS - elapsed_ms));
        } else {
            dropped++;
            taskYIELD();
        }
    }

    if (jpeg != nullptr) {
        heap_caps_free(jpeg);
    }
    ESP_LOGI(TAG, "[preview] stream client disconnected");
    return res;
}

esp_err_t http_stream_server_start()
{
    s_camera_ok = esp_camera_sensor_get() != nullptr;
    if (!camera_capture_lock_init()) {
        return ESP_ERR_NO_MEM;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 16;
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    config.stack_size = 8192;

    httpd_handle_t server = nullptr;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[preview] control http server start failed: %s", esp_err_to_name(err));
        return err;
    }

    const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = nullptr,
    };
    const httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = nullptr,
    };
    const httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = nullptr,
    };
    const httpd_uri_t face_result_uri = {
        .uri = "/face_result",
        .method = HTTP_GET,
        .handler = result_handler,
        .user_ctx = nullptr,
    };
    const httpd_uri_t result_uri = {
        .uri = "/result",
        .method = HTTP_GET,
        .handler = result_handler,
        .user_ctx = nullptr,
    };
    const httpd_uri_t range_result_uri = {
        .uri = "/range_result",
        .method = HTTP_GET,
        .handler = range_result_handler,
        .user_ctx = nullptr,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &capture_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &face_result_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &result_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &range_result_uri));

    httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
    stream_config.server_port = 81;
    stream_config.ctrl_port = 32769;
    stream_config.max_uri_handlers = 4;
    stream_config.max_open_sockets = 4;
    stream_config.lru_purge_enable = true;
    stream_config.recv_wait_timeout = 5;
    stream_config.send_wait_timeout = 5;
    stream_config.stack_size = 8192;

    httpd_handle_t stream_server = nullptr;
    err = httpd_start(&stream_server, &stream_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[preview] stream http server start failed: %s", esp_err_to_name(err));
        httpd_stop(server);
        return err;
    }

    const httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = nullptr,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(stream_server, &stream_uri));

    ESP_LOGI(TAG, "[preview] control http server started on port 80");
    ESP_LOGI(TAG, "[preview] stream http server started on port 81");
    return ESP_OK;
}
