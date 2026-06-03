#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

esp_err_t face_http_result_start();
int face_http_result_json(char *buf, size_t buf_size);
void face_http_result_summary(uint32_t *frame_id, int *face_count, int *inference_ms);
void face_http_result_set_stream_stats(int stream_fps);
