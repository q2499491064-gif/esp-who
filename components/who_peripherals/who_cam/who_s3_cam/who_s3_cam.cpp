#include "who_s3_cam.hpp"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "WhoS3Cam";

namespace who {
namespace cam {

WhoS3Cam::WhoS3Cam(const pixformat_t pixel_format,
                   const framesize_t frame_size,
                   const uint8_t fb_count,
                   bool vertical_flip,
                   bool horizontal_flip) :
    WhoCam(fb_count, resolution[frame_size].width, resolution[frame_size].height), m_format(pixel_format)
{
    camera_config_t camera_config = {
        .pin_pwdn = -1,  // PWDN is tied to GND.
        .pin_reset = -1, // RESET/RST is tied to 3V3.
        .pin_xclk = -1,  // XCLK is not routed on the current OV2640 module.
        .pin_sccb_sda = 4,
        .pin_sccb_scl = 5,
        .pin_d7 = 16,
        .pin_d6 = 17,
        .pin_d5 = 18,
        .pin_d4 = 12,
        .pin_d3 = 13,
        .pin_d2 = 14,
        .pin_d1 = 15,
        .pin_d0 = 2,
        .pin_vsync = 6,
        .pin_href = 7,
        .pin_pclk = 11,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = pixel_format,
        .frame_size = frame_size,
        .jpeg_quality = 12,
        .fb_count = fb_count,
#if CONFIG_SPIRAM
        .fb_location = CAMERA_FB_IN_PSRAM,
#else
        .fb_location = CAMERA_FB_IN_DRAM,
#endif
        .grab_mode = CAMERA_GRAB_LATEST,
        .sccb_i2c_port = 0,
    };

    ESP_ERROR_CHECK(esp_camera_init(&camera_config));
#if CONFIG_SPIRAM
    ESP_ERROR_CHECK(esp_camera_set_psram_mode(true));
#endif
    ESP_ERROR_CHECK(set_flip(!vertical_flip, !horizontal_flip));
}

WhoS3Cam::~WhoS3Cam()
{
    ESP_ERROR_CHECK(esp_camera_deinit());
}

cam_fb_t *WhoS3Cam::cam_fb_get()
{
    camera_fb_t *fb = esp_camera_fb_get();
    int i = get_cam_fb_index();
    m_cam_fbs[i] = cam_fb_t(*fb);
    return &m_cam_fbs[i];
}

void WhoS3Cam::cam_fb_return(cam_fb_t *fb)
{
    esp_camera_fb_return((camera_fb_t *)fb->ret);
}

esp_err_t WhoS3Cam::set_flip(bool vertical_flip, bool horizontal_flip)
{
    if (!vertical_flip & !horizontal_flip) {
        return ESP_OK;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (vertical_flip) {
        if (s->set_vflip(s, 1) != 0) {
            ESP_LOGE(TAG, "Failed to mirror the frame vertically.");
            return ESP_FAIL;
        }
    }
    if (horizontal_flip) {
        if (s->set_hmirror(s, 1) != 0) {
            ESP_LOGE(TAG, "Failed to mirror the frame horizontally.");
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

int WhoS3Cam::get_cam_fb_index()
{
    static int i = 0;
    int index = i;
    i = (i + 1) % m_fb_count;
    return index;
}

} // namespace cam
} // namespace who
