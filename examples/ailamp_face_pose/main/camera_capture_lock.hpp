#pragma once

#include "esp_camera.h"

bool camera_capture_lock_init();
camera_fb_t *camera_capture_get();
void camera_capture_return(camera_fb_t *fb);
