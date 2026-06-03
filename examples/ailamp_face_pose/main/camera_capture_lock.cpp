#include "camera_capture_lock.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_camera_fb_mutex = nullptr;

bool camera_capture_lock_init()
{
    if (s_camera_fb_mutex == nullptr) {
        s_camera_fb_mutex = xSemaphoreCreateMutex();
    }
    return s_camera_fb_mutex != nullptr;
}

camera_fb_t *camera_capture_get()
{
    if (!camera_capture_lock_init()) {
        return nullptr;
    }
    xSemaphoreTake(s_camera_fb_mutex, portMAX_DELAY);
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr) {
        xSemaphoreGive(s_camera_fb_mutex);
    }
    return fb;
}

void camera_capture_return(camera_fb_t *fb)
{
    if (fb != nullptr) {
        esp_camera_fb_return(fb);
    }
    if (s_camera_fb_mutex != nullptr) {
        xSemaphoreGive(s_camera_fb_mutex);
    }
}
