#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "control_policy.hpp"
#include "face_http_result.hpp"
#include "frame_cap_pipeline.hpp"
#include "fusion_state.hpp"
#include "http_stream_server.hpp"
#include "led_output.hpp"
#include "nvs_flash.h"
#include "range_sensor.hpp"
#include "wifi_app.hpp"
#include "who_recognition_app_lcd.hpp"
#include "who_recognition_app_term.hpp"
#include "who_spiflash_fatfs.hpp"

using namespace who::frame_cap;
using namespace who::app;

#ifndef CONFIG_AILAMP_STAGE2B_HTTP_PREVIEW
#define CONFIG_AILAMP_STAGE2B_HTTP_PREVIEW 1
#endif

#ifndef CONFIG_AILAMP_STAGE2C_FACE_HTTP_RESULT
#define CONFIG_AILAMP_STAGE2C_FACE_HTTP_RESULT 1
#endif

#ifndef CONFIG_AILAMP_STAGE3_FACE_POSE
#define CONFIG_AILAMP_STAGE3_FACE_POSE 1
#endif

static const char *TAG = "ailamp_main";
static constexpr int CAMERA_INIT_MAX_RETRY = 3;
static constexpr int CAMERA_INIT_RETRY_DELAY_MS = 200;

static void preview_hold_forever()
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t preview_nvs_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

static esp_err_t preview_camera_init()
{
    ESP_LOGI(TAG, "[preview] camera init start");

    camera_config_t camera_config = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = -1,
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
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 2,
#if CONFIG_SPIRAM
        .fb_location = CAMERA_FB_IN_PSRAM,
#else
        .fb_location = CAMERA_FB_IN_DRAM,
#endif
        .grab_mode = CAMERA_GRAB_LATEST,
        .sccb_i2c_port = 0,
    };

    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= CAMERA_INIT_MAX_RETRY; ++attempt) {
        err = esp_camera_init(&camera_config);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG,
                 "[preview] camera init attempt %d/%d failed: %s",
                 attempt,
                 CAMERA_INIT_MAX_RETRY,
                 esp_err_to_name(err));
        esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(CAMERA_INIT_RETRY_DELAY_MS));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[preview] camera init failed: %s", esp_err_to_name(err));
        return err;
    }
#if CONFIG_SPIRAM
    err = esp_camera_set_psram_mode(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[preview] camera psram mode failed: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return err;
    }
#endif

    ESP_LOGI(TAG, "[preview] camera init success");
    return ESP_OK;
}

extern "C" void app_main(void)
{
    vTaskPrioritySet(xTaskGetCurrentTaskHandle(), 5);
#if CONFIG_AILAMP_STAGE2C_FACE_HTTP_RESULT || CONFIG_AILAMP_STAGE2B_HTTP_PREVIEW
    printf("\n[preview] %s entry\n",
#if CONFIG_AILAMP_STAGE2C_FACE_HTTP_RESULT
           "STAGE2C_FACE_HTTP_RESULT"
#else
           "STAGE2B_HTTP_PREVIEW"
#endif
    );

    esp_err_t err = preview_nvs_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[preview] nvs init failed: %s", esp_err_to_name(err));
        preview_hold_forever();
    }

    err = preview_camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[preview] stop: camera init failed");
        preview_hold_forever();
    }

    err = led_output_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED PWM output init failed, preview continues: %s", esp_err_to_name(err));
    }
    led_output_task_start();

    err = wifi_app_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[preview] stop: wifi init/connect failed");
        ESP_LOGE(TAG, "[preview] run idf.py menuconfig -> AILamp Preview -> set Wi-Fi SSID/password");
        preview_hold_forever();
    }

    if (!fusion_state_init()) {
        ESP_LOGW(TAG, "fusion state init failed, preview continues without fusion state");
    }

    if (!control_policy_init()) {
        ESP_LOGW(TAG, "control policy init failed, preview continues without control policy");
    }

    err = http_stream_server_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[preview] stop: http server start failed");
        preview_hold_forever();
    }

#if CONFIG_AILAMP_STAGE2C_FACE_HTTP_RESULT
    err = face_http_result_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[stage2c] stop: face detect task start failed: %s", esp_err_to_name(err));
        preview_hold_forever();
    }
#endif

    if (range_sensor_init()) {
        ESP_LOGI(TAG, "VL53L1X range sensor init ok");
        range_sensor_start_task();
    } else {
        ESP_LOGW(TAG, "VL53L1X range sensor init failed, vision system continues");
    }

    preview_hold_forever();
#else
#if CONFIG_DB_FATFS_FLASH
    ESP_ERROR_CHECK(fatfs_flash_mount());
#elif CONFIG_DB_SPIFFS
    ESP_ERROR_CHECK(bsp_spiffs_mount());
#endif
#if CONFIG_DB_FATFS_SDCARD || CONFIG_HUMAN_FACE_DETECT_MODEL_IN_SDCARD || CONFIG_HUMAN_FACE_FEAT_MODEL_IN_SDCARD
    ESP_ERROR_CHECK(bsp_sdcard_mount());
#endif

// close led
#ifdef BSP_BOARD_ESP32_S3_EYE
    ESP_ERROR_CHECK(bsp_leds_init());
    ESP_ERROR_CHECK(bsp_led_set(BSP_LED_GREEN, false));
#endif

#if CONFIG_IDF_TARGET_ESP32S3
    auto frame_cap = get_dvp_frame_cap_pipeline();
#elif CONFIG_IDF_TARGET_ESP32P4
    auto frame_cap = get_mipi_csi_frame_cap_pipeline();
    // auto frame_cap = get_uvc_frame_cap_pipeline();
#endif
    auto recognition_app = new WhoRecognitionAppLCD(frame_cap);
    // try this if you don't have a lcd.
    // auto recognition_app = new WhoRecognitionAppTerm(frame_cap);
    recognition_app->run();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}
