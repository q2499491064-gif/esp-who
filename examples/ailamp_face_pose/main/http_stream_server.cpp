#include "http_stream_server.hpp"

#include <cstdarg>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "camera_capture_lock.hpp"
#include "control_policy.hpp"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "face_http_result.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fusion_state.hpp"
#include "led_mixer.hpp"
#include "led_output.hpp"
#include "range_sensor.hpp"

static const char *TAG = "preview_http";
static constexpr const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static constexpr const char *JSON_CONTENT_TYPE = "application/json; charset=utf-8";
static constexpr const char *STREAM_BOUNDARY = "--frame\r\n";
static constexpr const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

#ifndef CONFIG_AILAMP_STREAM_TARGET_FPS
#define CONFIG_AILAMP_STREAM_TARGET_FPS 8
#endif

#ifndef CONFIG_AILAMP_LOG_PERIOD_MS
#define CONFIG_AILAMP_LOG_PERIOD_MS 1000
#endif

#ifndef AILAMP_LED_TEST_HTTP_ENABLE
#define AILAMP_LED_TEST_HTTP_ENABLE 0
#endif

static constexpr int STREAM_TARGET_FPS = CONFIG_AILAMP_STREAM_TARGET_FPS;
static constexpr int STREAM_FRAME_PERIOD_MS = 1000 / STREAM_TARGET_FPS;
static constexpr int STREAM_LOG_PERIOD_MS = 2000;

static uint32_t s_stream_count = 0;
static size_t s_last_frame_len = 0;
static bool s_camera_ok = false;
static uint32_t s_http_stream_count = 0;
static uint32_t s_http_system_result_count = 0;
static uint32_t s_http_status_count = 0;
static uint32_t s_http_face_result_count = 0;
static uint32_t s_http_range_result_count = 0;
static uint32_t s_http_fusion_result_count = 0;
static uint32_t s_http_control_result_count = 0;

static bool json_append(char *buf, size_t buf_size, size_t *offset, const char *fmt, ...)
{
    if (buf == nullptr || offset == nullptr || *offset >= buf_size) {
        return false;
    }

    va_list args;
    va_start(args, fmt);
    const int written = std::vsnprintf(buf + *offset, buf_size - *offset, fmt, args);
    va_end(args);

    if (written < 0 || static_cast<size_t>(written) >= buf_size - *offset) {
        if (buf_size > 0) {
            buf[buf_size - 1] = '\0';
        }
        return false;
    }

    *offset += static_cast<size_t>(written);
    return true;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    static constexpr const char *html =
        "<!doctype html>\n"
        "<html lang=\"zh-CN\" translate=\"no\">\n"
        "  <head>\n"
        "    <meta charset=\"utf-8\">\n"
        "    <meta name=\"google\" content=\"notranslate\">\n"
        "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
        "    <title>智能学习台灯调试面板</title>\n"
        "    <title>&#x667a;&#x80fd;&#x5b66;&#x4e60;&#x53f0;&#x706f;&#x8c03;&#x8bd5;&#x9762;&#x677f;</title>\n"
        "    <style>\n"
        "      body{font-family:Arial,sans-serif;margin:16px;background:#101214;color:#eee;}\n"
        "      h2{display:none;}\n"
        "      #videoWrap{position:relative;display:inline-block;max-width:100%;}\n"
        "      #stream{display:block;width:640px;max-width:100%;height:auto;background:#000;}\n"
        "      #overlay{position:absolute;left:0;top:0;pointer-events:none;}\n"
        "      .status{margin-top:10px;color:#b8f7d4;font-size:14px;line-height:1.8;}\n"
        "      .pose{margin-top:6px;color:#ffd27d;font-size:15px;line-height:1.8;}\n"
        "      .row span{display:inline-block;min-width:72px;}\n"
        "      #fetchStatus{min-width:160px;color:#ffcf70;}\n"
        "      .notranslate{unicode-bidi:isolate;}\n"
        "      a{color:#8ecbff;}\n"
        "    </style>\n"
        "  </head>\n"
        "  <body>\n"
        "    <h1>&#x667a;&#x80fd;&#x5b66;&#x4e60;&#x53f0;&#x706f;&#x72b6;&#x6001;</h1>\n"
        "    <h2>智能学习台灯状态</h2>\n"
        "    <div id=\"videoWrap\" style=\"position:relative;display:inline-block;\">\n"
        "      <img id=\"stream\" src=\"\" style=\"display:block;width:640px;max-width:100%;\">\n"
        "      <canvas id=\"overlay\" style=\"position:absolute;left:0;top:0;pointer-events:none;\"></canvas>\n"
        "    </div>\n"
        "    <div class=\"status notranslate\">\n"
        "      <div class=\"row\">&#x4eba;&#x8138;&#x6570;&#x91cf;&#xff1a;<span id=\"faceCount\">--</span> &#x63a8;&#x7406;&#x65f6;&#x95f4;&#xff08;&#x6beb;&#x79d2;&#xff09;&#xff1a;<span id=\"inferenceMs\">--</span></div>\n"
        "      <div class=\"row\">&#x5b9e;&#x9645;&#x6570;&#x91cf;&#xff1a;<span id=\"actualFaceCount\">--</span> &#x663e;&#x793a;&#x6570;&#x91cf;&#xff1a;<span id=\"displayFaceCount\">--</span> &#x8ddf;&#x8e2a;&#x72b6;&#x6001;&#xff1a;<span id=\"trackingState\">--</span></div>\n"
        "      <div class=\"row\">&#x91c7;&#x96c6;&#xff08;&#x6beb;&#x79d2;&#xff09;&#xff1a;<span id=\"captureMs\">--</span> &#x89e3;&#x7801;&#xff08;&#x6beb;&#x79d2;&#xff09;&#xff1a;<span id=\"decodeMs\">--</span> &#x68c0;&#x6d4b;&#xff08;&#x6beb;&#x79d2;&#xff09;&#xff1a;<span id=\"detectMs\">--</span> &#x5173;&#x952e;&#x70b9;&#xff08;&#x6beb;&#x79d2;&#xff09;&#xff1a;<span id=\"landmarkMs\">--</span> &#x59ff;&#x6001;&#xff08;&#x6beb;&#x79d2;&#xff09;&#xff1a;<span id=\"poseMs\">--</span> &#x603b;&#x8017;&#x65f6;&#xff08;&#x6beb;&#x79d2;&#xff09;&#xff1a;<span id=\"totalMs\">--</span></div>\n"
        "      <div class=\"row\">&#x89c6;&#x9891;&#x5e27;&#x7387;&#xff1a;<span id=\"streamFps\">--</span> &#x68c0;&#x6d4b;&#x9891;&#x7387;&#xff1a;<span id=\"detectHz\">--</span> &#x547d;&#x4e2d;&#x7387;&#xff1a;<span id=\"faceHitRate\">--</span> &#x4fdd;&#x6301;&#x65f6;&#x95f4;&#xff08;&#x6beb;&#x79d2;&#xff09;&#xff1a;<span id=\"holdMs\">--</span></div>\n"
        "      <div class=\"row\">&#x5e27;&#x7f16;&#x53f7;&#xff1a;<span id=\"frameId\">--</span> &#x53ef;&#x7528;&#x5185;&#x5b58;&#xff1a;<span id=\"freeHeap\">--</span></div>\n"
        "      <div class=\"row\">&#x8ddd;&#x79bb;&#xff1a;<span id=\"rangeDistance\">--</span> &#x6beb;&#x7c73; &#x8ddd;&#x79bb;&#x72b6;&#x6001;&#xff1a;<span id=\"rangeStatus\">--</span> &#x8ddd;&#x79bb;&#x6709;&#x6548;&#x6027;&#xff1a;<span id=\"rangeValid\">--</span></div>\n"
        "      <div class=\"row\"><b>&#x878d;&#x5408;&#x72b6;&#x6001;</b> &#x76ee;&#x6807;&#xff1a;<span id=\"fusionTarget\">--</span> &#x72b6;&#x6001;&#xff1a;<span id=\"fusionState\">--</span></div>\n"
        "      <div class=\"row\">&#x8ddd;&#x79bb;&#xff1a;<span id=\"fusionDistance\">--</span> &#x6beb;&#x7c73; &#x6ee4;&#x6ce2;&#x8ddd;&#x79bb;&#xff1a;<span id=\"fusionFilteredDistance\">--</span> &#x6beb;&#x7c73; &#x8ddd;&#x79bb;&#x8303;&#x56f4;&#xff1a;<span id=\"fusionRange\">--</span></div>\n"
        "      <div class=\"row\">&#x878d;&#x5408;&#x7f6e;&#x4fe1;&#x5ea6;&#xff1a;<span id=\"fusionConfidence\">--</span> &#x539f;&#x56e0;&#xff1a;<span id=\"fusionReason\">--</span> &#x4e22;&#x8138;&#x65f6;&#x95f4;&#xff1a;<span id=\"faceLostMs\">--</span></div>\n"
        "      <div class=\"row\"><b>&#x706f;&#x5149;&#x63a7;&#x5236;&#x7b56;&#x7565;</b> &#x6a21;&#x5f0f;&#xff1a;<span id=\"controlMode\">--</span> &#x5750;&#x59ff;&#x8bc4;&#x5206;&#xff1a;<span id=\"controlScore\">--</span> &#x5fae;&#x8c03;&#x7b49;&#x7ea7;&#xff1a;<span id=\"controlFuzzy\">--</span></div>\n"
        "      <div class=\"row\">&#x4eae;&#x5ea6;&#xff1a;<span id=\"controlBrightness\">--</span> &#x8272;&#x6e29;&#xff1a;<span id=\"controlCct\">--</span> &#x5f00;&#x5c14;&#x6587; &#x6696;&#x5149;&#x865a;&#x62df;&#x8f93;&#x51fa;&#xff1a;<span id=\"controlWarm\">--</span> &#x51b7;&#x5149;&#x865a;&#x62df;&#x8f93;&#x51fa;&#xff1a;<span id=\"controlCool\">--</span></div>\n"
        "      <div class=\"row\">&#x8bed;&#x97f3;&#x63d0;&#x9192;&#xff1a;<span id=\"controlVoice\">--</span> &#x5e94;&#x7528;&#x63d0;&#x9192;&#xff1a;<span id=\"controlApp\">--</span> &#x539f;&#x56e0;&#xff1a;<span id=\"controlReason\">--</span></div>\n"
        "      <div class=\"row\">&#x63a7;&#x5236;&#x878d;&#x5408;&#xff1a;<span id=\"controlFusionValid\">--</span> &#x76ee;&#x6807;&#xff1a;<span id=\"controlFusionTarget\">--</span> &#x4eba;&#x8138;&#xff1a;<span id=\"controlFusionFace\">--</span> &#x8ddd;&#x79bb;&#xff1a;<span id=\"controlFusionRange\">--</span> &#x65f6;&#x5ef6;&#xff1a;<span id=\"controlFusionAge\">--</span></div>\n"
        "      <div class=\"row\"><b>LED &#x786c;&#x4ef6;&#x4e0e; PWM &#x8f93;&#x51fa;</b> &#x7535;&#x6c60;&#x6807;&#x79f0;&#x7535;&#x538b;&#xff1a;<span id=\"batteryNominal\">--</span> &#x7535;&#x6c60;&#x6ee1;&#x5145;&#x7535;&#x538b;&#xff1a;<span id=\"batteryFull\">--</span> ESP32 &#x4f9b;&#x7535;&#xff1a;<span id=\"espSupply\">--</span></div>\n"
        "      <div class=\"row\">LED &#x706f;&#x73e0;&#xff1a;<span id=\"cobType\">--</span> &#x8272;&#x6e29;&#x8303;&#x56f4;&#xff1a;<span id=\"cobCctRange\">--</span> &#x5de5;&#x4f5c;&#x89c4;&#x683c;&#xff1a;<span id=\"cobSpec\">--</span></div>\n"
        "      <div class=\"row\">LEDC &#x521d;&#x59cb;&#x5316;&#xff1a;<span id=\"ledInitialized\">--</span> PWM &#x8f93;&#x51fa;&#x4f7f;&#x80fd;&#xff1a;<span id=\"ledEnabled\">--</span> &#x6545;&#x969c;&#x72b6;&#x6001;&#xff1a;<span id=\"ledFault\">--</span></div>\n"
        "      <div class=\"row\">&#x6696;&#x767d; PWM GPIO&#xff1a;<span id=\"ledWarmGpio\">--</span> &#x51b7;&#x767d; PWM GPIO&#xff1a;<span id=\"ledCoolGpio\">--</span> PWM &#x9891;&#x7387;&#xff1a;<span id=\"ledFreq\">--</span> &#x5360;&#x7a7a;&#x6bd4;&#x5206;&#x8fa8;&#x7387;&#xff1a;<span id=\"ledResolution\">--</span></div>\n"
        "      <div class=\"row\">PWM &#x6781;&#x6027;&#xff1a;<span id=\"ledPolarity\">--</span> &#x901a;&#x9053;&#x4ea4;&#x6362;&#xff1a;<span id=\"ledSwap\">--</span> &#x5b89;&#x5168;&#x6d4b;&#x8bd5;&#x9650;&#x5e45;&#xff1a;<span id=\"ledSafeLimit\">--</span> &#x6d4b;&#x8bd5;&#x6a21;&#x5f0f;&#xff1a;<span id=\"ledTestMode\">--</span></div>\n"
        "      <div class=\"row\">&#x6696;&#x767d;&#x76ee;&#x6807; PWM&#xff1a;<span id=\"ledWarmTarget\">--</span> &#x51b7;&#x767d;&#x76ee;&#x6807; PWM&#xff1a;<span id=\"ledCoolTarget\">--</span> &#x6696;&#x767d;&#x5f53;&#x524d; PWM&#xff1a;<span id=\"ledWarmCurrent\">--</span> &#x51b7;&#x767d;&#x5f53;&#x524d; PWM&#xff1a;<span id=\"ledCoolCurrent\">--</span></div>\n"
        "      <div class=\"row\">&#x6696;&#x767d; logical duty&#xff1a;<span id=\"ledWarmDutyLogical\">--</span> &#x51b7;&#x767d; logical duty&#xff1a;<span id=\"ledCoolDutyLogical\">--</span> &#x6696;&#x767d;&#x8f93;&#x51fa; duty&#xff1a;<span id=\"ledWarmDutyOutput\">--</span> &#x51b7;&#x767d;&#x8f93;&#x51fa; duty&#xff1a;<span id=\"ledCoolDutyOutput\">--</span></div>\n"
        "      <div class=\"row\">&#x6696;&#x767d;&#x6bd4;&#x4f8b;&#xff1a;<span id=\"ledWarmRatio\">--</span> &#x51b7;&#x767d;&#x6bd4;&#x4f8b;&#xff1a;<span id=\"ledCoolRatio\">--</span> &#x603b;&#x529f;&#x7387;&#x9650;&#x5e45;&#xff1a;<span id=\"ledMixLimit\">--</span></div>\n"
        "      <div class=\"row\">&#x5f53;&#x524d;&#x63a7;&#x5236;&#x6a21;&#x5f0f;&#xff1a;<span id=\"ledSourceMode\">--</span> &#x5f53;&#x524d;&#x539f;&#x56e0;&#xff1a;<span id=\"ledSourceReason\">--</span> &#x98ce;&#x9669;&#x8bc4;&#x5206;&#xff1a;<span id=\"ledRiskScore\">--</span></div>\n"
        "      <div class=\"row\">&#x662f;&#x5426;&#x9700;&#x8981;&#x8bed;&#x97f3;&#x63d0;&#x9192;&#xff1a;<span id=\"ledVoiceAlert\">--</span> &#x662f;&#x5426;&#x9700;&#x8981; App &#x63d0;&#x9192;&#xff1a;<span id=\"ledAppAlert\">--</span></div>\n"
        "      <div class=\"row\">&#x6545;&#x969c;&#x539f;&#x56e0;&#xff1a;<span id=\"ledFaultReason\">--</span></div>\n"
        "      <div class=\"row\">&#x7cfb;&#x7edf;&#x72b6;&#x6001;&#xff1a;<span id=\"fetchStatus\">&#x542f;&#x52a8;&#x4e2d;</span></div>\n"
        "    </div>\n"
        "    <div class=\"pose notranslate\">\n"
        "      <div class=\"row\">&#x539f;&#x59cb;&#x4fef;&#x4ef0;&#xff1a;<span id=\"rawPitch\">--</span> &#x539f;&#x59cb;&#x504f;&#x822a;&#xff1a;<span id=\"rawYaw\">--</span> &#x539f;&#x59cb;&#x7ffb;&#x6eda;&#xff1a;<span id=\"rawRoll\">--</span></div>\n"
        "      <div class=\"row\">&#x6ee4;&#x6ce2;&#x4fef;&#x4ef0;&#xff1a;<span id=\"filteredPitch\">--</span> &#x6ee4;&#x6ce2;&#x504f;&#x822a;&#xff1a;<span id=\"filteredYaw\">--</span> &#x6ee4;&#x6ce2;&#x7ffb;&#x6eda;&#xff1a;<span id=\"filteredRoll\">--</span></div>\n"
        "      <div class=\"row\">&#x6821;&#x51c6;&#x4fef;&#x4ef0;&#xff1a;<span id=\"calPitch\">--</span> &#x6821;&#x51c6;&#x504f;&#x822a;&#xff1a;<span id=\"calYaw\">--</span> &#x6821;&#x51c6;&#x7ffb;&#x6eda;&#xff1a;<span id=\"calRoll\">--</span></div>\n"
        "      <div class=\"row\">&#x6ee4;&#x6ce2;&#x7cfb;&#x6570;&#xff1a;<span id=\"filterAlpha\">--</span> &#x6700;&#x5927;&#x53d8;&#x5316;&#xff1a;<span id=\"maxDelta\">--</span> &#x4e22;&#x5931;&#x8ba1;&#x6570;&#xff1a;<span id=\"lostCount\">--</span></div>\n"
        "      <div class=\"row\">&#x91cd;&#x7f6e;&#x8ba1;&#x6570;&#xff1a;<span id=\"resetCount\">--</span> &#x91cd;&#x7f6e;&#x539f;&#x56e0;&#xff1a;<span id=\"resetReason\">--</span></div>\n"
        "    </div>\n"
        "    <p class=\"notranslate\"><a href=\"/capture\">&#x624b;&#x52a8;&#x6293;&#x62cd;</a></p>\n"
        "    <p class=\"notranslate\"><a href=\"/system_result\">&#x7cfb;&#x7edf;&#x7ed3;&#x679c;&#x8c03;&#x8bd5;</a></p>\n"
        "    <p class=\"notranslate\"><a href=\"/status\">&#x72b6;&#x6001;&#x8c03;&#x8bd5;</a> <a href=\"/face_result\">&#x4eba;&#x8138;&#x8c03;&#x8bd5;</a> <a href=\"/range_result\">&#x8ddd;&#x79bb;&#x8c03;&#x8bd5;</a> <a href=\"/fusion_result\">&#x878d;&#x5408;&#x8c03;&#x8bd5;</a> <a href=\"/control_result\">&#x63a7;&#x5236;&#x8c03;&#x8bd5;</a></p>\n"
        "    <div style=\"display:none\">\n"
        "    <div class=\"status notranslate\">\n"
        "      <div class=\"row\">人脸数量：<span id=\"faceCount\">--</span> 推理时间（毫秒）：<span id=\"inferenceMs\">--</span></div>\n"
        "      <div class=\"row\">实际数量：<span id=\"actualFaceCount\">--</span> 显示数量：<span id=\"displayFaceCount\">--</span> 跟踪状态：<span id=\"trackingState\">--</span></div>\n"
        "      <div class=\"row\">采集（毫秒）：<span id=\"captureMs\">--</span> 解码（毫秒）：<span id=\"decodeMs\">--</span> 检测（毫秒）：<span id=\"detectMs\">--</span> 关键点（毫秒）：<span id=\"landmarkMs\">--</span> 姿态（毫秒）：<span id=\"poseMs\">--</span> 总耗时（毫秒）：<span id=\"totalMs\">--</span></div>\n"
        "      <div class=\"row\">视频帧率：<span id=\"streamFps\">--</span> 检测频率：<span id=\"detectHz\">--</span> 命中率：<span id=\"faceHitRate\">--</span> 保持时间（毫秒）：<span id=\"holdMs\">--</span></div>\n"
        "      <div class=\"row\">帧编号：<span id=\"frameId\">--</span> 可用内存：<span id=\"freeHeap\">--</span></div>\n"
        "      <div class=\"row\">距离：<span id=\"rangeDistance\">--</span> 毫米 距离状态：<span id=\"rangeStatus\">--</span> 距离有效性：<span id=\"rangeValid\">--</span></div>\n"
        "      <div class=\"row\"><b>融合状态</b> 目标：<span id=\"fusionTarget\">--</span> 状态：<span id=\"fusionState\">--</span></div>\n"
        "      <div class=\"row\">距离：<span id=\"fusionDistance\">--</span> 毫米 滤波距离：<span id=\"fusionFilteredDistance\">--</span> 毫米 距离范围：<span id=\"fusionRange\">--</span></div>\n"
        "      <div class=\"row\">融合置信度：<span id=\"fusionConfidence\">--</span> 原因：<span id=\"fusionReason\">--</span> 丢脸时间：<span id=\"faceLostMs\">--</span></div>\n"
        "      <div class=\"row\"><b>灯光控制策略</b> 模式：<span id=\"controlMode\">--</span> 坐姿评分：<span id=\"controlScore\">--</span> 微调等级：<span id=\"controlFuzzy\">--</span></div>\n"
        "      <div class=\"row\">亮度：<span id=\"controlBrightness\">--</span> 色温：<span id=\"controlCct\">--</span> 开尔文 暖光虚拟输出：<span id=\"controlWarm\">--</span> 冷光虚拟输出：<span id=\"controlCool\">--</span></div>\n"
        "      <div class=\"row\">语音提醒：<span id=\"controlVoice\">--</span> 应用提醒：<span id=\"controlApp\">--</span> 原因：<span id=\"controlReason\">--</span></div>\n"
        "      <div class=\"row\">控制融合：<span id=\"controlFusionValid\">--</span> 目标：<span id=\"controlFusionTarget\">--</span> 人脸：<span id=\"controlFusionFace\">--</span> 距离：<span id=\"controlFusionRange\">--</span> 年龄：<span id=\"controlFusionAge\">--</span></div>\n"
        "      <div class=\"row\">系统状态：<span id=\"fetchStatus\">启动中</span></div>\n"
        "    </div>\n"
        "    <div class=\"pose notranslate\">\n"
        "      <div class=\"row\">原始俯仰：<span id=\"rawPitch\">--</span> 原始偏航：<span id=\"rawYaw\">--</span> 原始翻滚：<span id=\"rawRoll\">--</span></div>\n"
        "      <div class=\"row\">滤波俯仰：<span id=\"filteredPitch\">--</span> 滤波偏航：<span id=\"filteredYaw\">--</span> 滤波翻滚：<span id=\"filteredRoll\">--</span></div>\n"
        "      <div class=\"row\">校准俯仰：<span id=\"calPitch\">--</span> 校准偏航：<span id=\"calYaw\">--</span> 校准翻滚：<span id=\"calRoll\">--</span></div>\n"
        "      <div class=\"row\">滤波系数：<span id=\"filterAlpha\">--</span> 最大变化：<span id=\"maxDelta\">--</span> 丢失计数：<span id=\"lostCount\">--</span></div>\n"
        "      <div class=\"row\">重置计数：<span id=\"resetCount\">--</span> 重置原因：<span id=\"resetReason\">--</span></div>\n"
        "    </div>\n"
        "    <p class=\"notranslate\"><a href=\"/capture\">手动抓拍</a></p>\n"
        "    <p class=\"notranslate\"><a href=\"/system_result\">系统结果调试</a></p>\n"
        "    <p class=\"notranslate\"><a href=\"/status\">状态调试</a> <a href=\"/face_result\">人脸调试</a> <a href=\"/range_result\">距离调试</a> <a href=\"/fusion_result\">融合调试</a> <a href=\"/control_result\">控制调试</a></p>\n"
        "    </div>\n"
        "    <script>\n"
        "      const img=document.getElementById('stream');\n"
        "      const canvas=document.getElementById('overlay');\n"
        "      const ctx=canvas.getContext('2d');\n"
        "      const streamUrl='http://'+window.location.hostname+':81/stream';\n"
        "      img.src=streamUrl;\n"
        "      function el(id){return document.getElementById(id);}\n"
        "      function valueOr(v,d){return v===undefined||v===null?d:v;}\n"
        "      function fmt(v){if(v===undefined||v===null||Number.isNaN(Number(v)))return '--';return Number(v).toFixed(2);}\n"
        "      function yesNo(v){return v?'是':'否';}\n"
        "      function validText(v){return v?'有效':'无效';}\n"
        "      function voiceText(v){return v?'需要提醒':'不提醒';}\n"
        "      function appText(v){return v?'需要推送':'不推送';}\n"
        "      function rangeStateText(v){return ({ok:'正常',too_near:'过近',too_far:'过远',invalid:'无效'})[v]||valueOr(v,'--');}\n"
        "      function controlModeText(v){return ({NORMAL:'常规专注',FUZZY_ADJUST:'无感微调',ALERT:'警报提醒',SENSOR_INVALID:'传感器无效'})[v]||valueOr(v,'--');}\n"
        "      function fusionTargetText(v){return v?'目标有效':'目标无效';}\n"
        "      function fusionStateText(v){return ({target_valid:'目标有效',face_lost:'人脸丢失',face_invalid:'人脸无效',range_invalid:'距离无效',sensor_invalid:'传感器无效',invalid:'传感器无效',too_near:'过近',too_far:'过远'})[v]||valueOr(v,'--');}\n"
        "      function yesNo(v){return v?'\\u662f':'\\u5426';}\n"
        "      function validText(v){return v?'\\u6709\\u6548':'\\u65e0\\u6548';}\n"
        "      function voiceText(v){return v?'\\u9700\\u8981\\u63d0\\u9192':'\\u4e0d\\u63d0\\u9192';}\n"
        "      function appText(v){return v?'\\u9700\\u8981\\u63d0\\u9192':'\\u4e0d\\u63d0\\u9192';}\n"
        "      function trackingStateText(v){return ({tracking:'\\u8ddf\\u8e2a\\u4e2d',hold:'\\u77ed\\u6682\\u4fdd\\u6301',lost:'\\u5df2\\u4e22\\u5931'})[v]||'--';}\n"
        "      function rangeStateText(v){return ({ok:'\\u6b63\\u5e38',too_near:'\\u8fc7\\u8fd1',too_far:'\\u8fc7\\u8fdc',invalid:'\\u65e0\\u6548'})[v]||'--';}\n"
        "      function controlModeText(v){return ({NORMAL:'\\u5e38\\u89c4\\u4e13\\u6ce8',FUZZY_ADJUST:'\\u65e0\\u611f\\u5fae\\u8c03',ALERT:'\\u8b66\\u62a5\\u63d0\\u9192',SENSOR_INVALID:'\\u4f20\\u611f\\u5668\\u65e0\\u6548'})[v]||'--';}\n"
        "      function fusionTargetText(v){return v?'\\u76ee\\u6807\\u6709\\u6548':'\\u76ee\\u6807\\u65e0\\u6548';}\n"
        "      function fusionStateText(v){return ({target_valid:'\\u76ee\\u6807\\u6709\\u6548',face_lost:'\\u4eba\\u8138\\u4e22\\u5931',face_invalid:'\\u4eba\\u8138\\u65e0\\u6548',range_invalid:'\\u8ddd\\u79bb\\u65e0\\u6548',sensor_invalid:'\\u4f20\\u611f\\u5668\\u65e0\\u6548',target_absent:'\\u76ee\\u6807\\u79bb\\u5f00',invalid:'\\u4f20\\u611f\\u5668\\u65e0\\u6548',too_near:'\\u8fc7\\u8fd1',too_far:'\\u8fc7\\u8fdc'})[v]||'--';}\n"
        "      function reasonText(v){return ({none:'\\u65e0',normal:'\\u6b63\\u5e38',posture_ok:'\\u6b63\\u5e38',test_off:'\\u6d4b\\u8bd5\\u5173\\u95ed',test_warm:'\\u6696\\u767d\\u6d4b\\u8bd5',test_cool:'\\u51b7\\u767d\\u6d4b\\u8bd5',test_both:'\\u53cc\\u901a\\u9053\\u6d4b\\u8bd5',mild_posture_degradation:'\\u8f7b\\u5fae\\u59ff\\u6001\\u9000\\u5316',bad_posture_timer:'\\u59ff\\u6001\\u504f\\u5dee\\u8ba1\\u65f6\\u4e2d',bad_posture_held:'\\u59ff\\u6001\\u504f\\u5dee\\u6301\\u7eed',distance_too_near:'\\u8ddd\\u79bb\\u8fc7\\u8fd1',too_near:'\\u8ddd\\u79bb\\u8fc7\\u8fd1',no_face:'\\u672a\\u68c0\\u6d4b\\u5230\\u4eba\\u8138',no_face_timeout:'\\u957f\\u65f6\\u95f4\\u672a\\u68c0\\u6d4b\\u5230\\u4eba\\u8138',suspected_head_down_no_face:'\\u7591\\u4f3c\\u4e25\\u91cd\\u4f4e\\u5934\\u6216\\u8138\\u90e8\\u906e\\u6321',no_face_with_target_present:'\\u6709\\u76ee\\u6807\\u4f46\\u4eba\\u8138\\u4e0d\\u53ef\\u89c1',sensor_invalid:'\\u4f20\\u611f\\u5668\\u65e0\\u6548',target_absent:'\\u76ee\\u6807\\u79bb\\u5f00',fusion_invalid:'\\u4f20\\u611f\\u5668\\u65e0\\u6548',fusion_invalid_hold:'\\u878d\\u5408\\u77ed\\u6682\\u4fdd\\u6301',face_hold:'\\u4eba\\u8138\\u77ed\\u6682\\u4fdd\\u6301',fusion_hold:'\\u878d\\u5408\\u77ed\\u6682\\u4fdd\\u6301',alert_active:'\\u8b66\\u62a5\\u63d0\\u9192\\u4e2d',alert_recovery_timer:'\\u6062\\u590d\\u89c2\\u5bdf\\u4e2d',severe_posture:'\\u4e25\\u91cd\\u59ff\\u6001\\u5f02\\u5e38',severe_posture_degradation:'\\u4e25\\u91cd\\u59ff\\u6001\\u5f02\\u5e38',fuzzy_no_improvement:'\\u5fae\\u8c03\\u540e\\u4ecd\\u672a\\u6539\\u5584',fuzzy_no_improve:'\\u5fae\\u8c03\\u540e\\u4ecd\\u672a\\u6539\\u5584'})[v]||'--';}\n"
        "      function millisText(v){return valueOr(v,'--')==='--'?'--':valueOr(v,0)+' \\u6beb\\u79d2';}\n"
        "      function percentText(v){if(v===undefined||v===null||Number.isNaN(Number(v)))return '--';return Math.round(Number(v)*100)+'%';}\n"
        "      function ledFaultText(v){return v?'\\u6545\\u969c':'\\u6b63\\u5e38';}\n"
        "      function enabledText(v){return v?'\\u542f\\u7528':'\\u5173\\u95ed';}\n"
        "      function polarityText(v){return v?'\\u9ad8\\u7535\\u5e73\\u6709\\u6548':'\\u4f4e\\u7535\\u5e73\\u6709\\u6548';}\n"
        "      function testModeText(v){return ({normal:'\\u6b63\\u5e38\\u63a7\\u5236',off:'\\u5173\\u95ed',warm:'\\u6696\\u767d\\u5355\\u901a\\u9053',cool:'\\u51b7\\u767d\\u5355\\u901a\\u9053',both:'\\u53cc\\u901a\\u9053\\u4f4e\\u529f\\u7387'})[v]||'--';}\n"
        "      function faultReasonText(v){const m={none:'\\u65e0',init:'\\u521d\\u59cb\\u5316\\u4e2d',not_initialized:'\\u672a\\u521d\\u59cb\\u5316',snapshot_failed:'\\u5feb\\u7167\\u8bfb\\u53d6\\u5931\\u8d25',timer_config_failed:'LEDC \\u5b9a\\u65f6\\u5668\\u914d\\u7f6e\\u5931\\u8d25',warm_channel_config_failed:'\\u6696\\u767d\\u901a\\u9053\\u914d\\u7f6e\\u5931\\u8d25',cool_channel_config_failed:'\\u51b7\\u767d\\u901a\\u9053\\u914d\\u7f6e\\u5931\\u8d25',set_target_before_init:'\\u672a\\u521d\\u59cb\\u5316\\u5c31\\u5199\\u5165\\u76ee\\u6807',duty_update_failed:'PWM duty \\u66f4\\u65b0\\u5931\\u8d25',safe_off_failed:'\\u5b89\\u5168\\u5173\\u65ad\\u5931\\u8d25',force_off:'\\u5f3a\\u5236\\u5173\\u95ed',force_off_failed:'\\u5f3a\\u5236\\u5173\\u95ed\\u5931\\u8d25',task_create_failed:'PWM \\u4efb\\u52a1\\u521b\\u5efa\\u5931\\u8d25'};return m[v]||reasonText(v)||'--';}\n"
        "      function resizeCanvas(){const w=img.clientWidth||img.naturalWidth||640;const h=img.clientHeight||img.naturalHeight||480;if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}}\n"
        "      function clearOverlay(){resizeCanvas();ctx.clearRect(0,0,canvas.width,canvas.height);}\n"
        "      function drawPoint(x,y,sx,sy,label){const px=x*sx;const py=y*sy;ctx.beginPath();ctx.arc(px,py,4,0,Math.PI*2);ctx.fill();ctx.fillText(label,px+6,py-6);}\n"
        "      function clearPose(){el('rawPitch').textContent='--';el('rawYaw').textContent='--';el('rawRoll').textContent='--';el('filteredPitch').textContent='--';el('filteredYaw').textContent='--';el('filteredRoll').textContent='--';el('calPitch').textContent='--';el('calYaw').textContent='--';el('calRoll').textContent='--';el('filterAlpha').textContent='--';el('maxDelta').textContent='--';el('lostCount').textContent='--';el('resetCount').textContent='--';el('resetReason').textContent='--';}\n"
        "      function showFilter(f){if(!f)return;el('filterAlpha').textContent=fmt(f.alpha);el('maxDelta').textContent=fmt(f.max_delta);el('lostCount').textContent=valueOr(f.lost_count,'--');el('resetCount').textContent=valueOr(f.reset_count,'--');el('resetReason').textContent=valueOr(f.last_reset_reason,'--');}\n"
        "      function drawSystemOverlay(data){\n"
        "        const f=data.face||{};resizeCanvas();ctx.clearRect(0,0,canvas.width,canvas.height);\n"
        "        const faces=f.faces||[];const state=valueOr(f.state,'lost');\n"
        "        if(state==='lost'||faces.length===0){return;}\n"
        "        const fw=f.frame_width||320;const fh=f.frame_height||240;const sx=canvas.width/fw;const sy=canvas.height/fh;ctx.font='14px Arial';\n"
        "        for(const face of faces){\n"
        "          const x1=valueOr(face.x1,0);const y1=valueOr(face.y1,0);const w=valueOr(face.w,valueOr(face.x2,0)-x1);const h=valueOr(face.h,valueOr(face.y2,0)-y1);\n"
        "          const color=state==='hold'?'#ffd84d':'#00ff00';ctx.lineWidth=2;ctx.strokeStyle=color;ctx.fillStyle=color;ctx.strokeRect(x1*sx,y1*sy,w*sx,h*sy);if(state==='hold'){ctx.fillText('\\u4fdd\\u6301',x1*sx+4,y1*sy+16);}\n"
        "          if(face.landmarks_valid&&face.landmarks){drawPoint(face.landmarks.left_eye.x,face.landmarks.left_eye.y,sx,sy,'\\u5de6\\u773c');drawPoint(face.landmarks.right_eye.x,face.landmarks.right_eye.y,sx,sy,'\\u53f3\\u773c');drawPoint(face.landmarks.nose.x,face.landmarks.nose.y,sx,sy,'\\u9f3b');drawPoint(face.landmarks.left_mouth.x,face.landmarks.left_mouth.y,sx,sy,'\\u5de6\\u5634');drawPoint(face.landmarks.right_mouth.x,face.landmarks.right_mouth.y,sx,sy,'\\u53f3\\u5634');}\n"
        "        }\n"
        "      }\n"
        "      function applySystem(data){\n"
        "        const f=data.face||{};const p=data.pose||{};const r=data.range||{};const fu=data.fusion||{};const c=data.control||{};const hp=data.hardware_power||{};const mix=data.led_mix||{};const led=data.led_output||{};\n"
        "        el('faceCount').textContent=valueOr(f.count,0);el('actualFaceCount').textContent=valueOr(f.count,0);el('displayFaceCount').textContent=valueOr(f.display_count,0);el('trackingState').textContent=trackingStateText(f.state);el('inferenceMs').textContent=millisText(f.inference_ms);el('faceHitRate').textContent=Math.round(Number(valueOr(f.hit_rate,0))*100)+'%';el('frameId').textContent=valueOr(f.frame_id,0);\n"
        "        el('rawPitch').textContent=fmt(p.pitch);el('rawYaw').textContent=fmt(p.yaw);el('rawRoll').textContent=fmt(p.roll);el('filteredPitch').textContent=fmt(p.filtered_pitch);el('filteredYaw').textContent=fmt(p.filtered_yaw);el('filteredRoll').textContent=fmt(p.filtered_roll);\n"
        "        el('rangeDistance').textContent=valueOr(r.distance_mm,'--');el('rangeStatus').textContent=rangeStateText(r.state);el('rangeValid').textContent=validText(!!r.valid);\n"
        "        el('fusionTarget').textContent=fusionTargetText(!!fu.target_valid);el('fusionState').textContent=fusionStateText(fu.state);el('fusionDistance').textContent=valueOr(r.distance_mm,'--');el('fusionFilteredDistance').textContent=valueOr(r.filtered_distance_mm,'--');el('fusionRange').textContent=rangeStateText(r.state);el('fusionConfidence').textContent=Math.round(Number(valueOr(fu.confidence,0))*100)+'%';el('fusionReason').textContent=reasonText(fu.reason);el('faceLostMs').textContent=millisText(fu.face_lost_ms);\n"
        "        el('controlMode').textContent=controlModeText(c.mode);el('controlScore').textContent=fmt(c.posture_score);el('controlFuzzy').textContent=fmt(c.fuzzy_level);el('controlBrightness').textContent=Math.round(Number(valueOr(c.brightness,0))*100)+'%';el('controlCct').textContent=Math.round(Number(valueOr(c.cct_k,0)));el('controlWarm').textContent=fmt(c.warm_pwm_virtual);el('controlCool').textContent=fmt(c.cool_pwm_virtual);el('controlVoice').textContent=voiceText(!!c.voice_alert_required);el('controlApp').textContent=appText(!!c.app_alert_required);el('controlReason').textContent=reasonText(c.reason);el('controlFusionValid').textContent=validText(!!fu.valid);el('controlFusionTarget').textContent=fusionTargetText(!!fu.target_valid);el('controlFusionFace').textContent=validText(!!fu.face_valid);el('controlFusionRange').textContent=validText(!!fu.range_valid);el('controlFusionAge').textContent='--';\n"
        "        el('batteryNominal').textContent=valueOr(hp.battery_nominal_v,'--')+'V';el('batteryFull').textContent=valueOr(hp.battery_full_v,'--')+'V';el('espSupply').textContent=hp.esp32_supply_from_lm2596?'LM2596 \\u964d\\u538b 5V':'--';el('cobType').textContent=led.common_anode_cob?'\\u5171\\u9633\\u53cc\\u8272 COB':'--';el('cobCctRange').textContent=valueOr(mix.warm_cct_k,'--')+'K~'+valueOr(mix.cool_cct_k,'--')+'K';el('cobSpec').textContent=valueOr(hp.cob_spec,'--');\n"
        "        el('ledInitialized').textContent=yesNo(!!led.initialized);el('ledEnabled').textContent=enabledText(!!led.enabled);el('ledFault').textContent=ledFaultText(!!led.fault);el('ledWarmGpio').textContent=led.warm_gpio===undefined||led.warm_gpio===null?'--':'GPIO'+led.warm_gpio;el('ledCoolGpio').textContent=led.cool_gpio===undefined||led.cool_gpio===null?'--':'GPIO'+led.cool_gpio;el('ledFreq').textContent=valueOr(led.pwm_freq_hz,'--')+' \\u8d6b\\u5179';el('ledResolution').textContent=valueOr(led.duty_resolution_bits,'--')+' \\u4f4d';\n"
        "        el('ledPolarity').textContent=polarityText(!!led.active_high);el('ledSwap').textContent=yesNo(!!led.swap_warm_cool_channels);el('ledSafeLimit').textContent=enabledText(!!led.safe_test_limit_enable)+' '+percentText(mix.safe_test_pwm_limit);el('ledTestMode').textContent=testModeText(led.test_mode);\n"
        "        el('ledWarmTarget').textContent=percentText(led.warm_target);el('ledCoolTarget').textContent=percentText(led.cool_target);el('ledWarmCurrent').textContent=percentText(led.warm_current);el('ledCoolCurrent').textContent=percentText(led.cool_current);el('ledWarmDutyLogical').textContent=valueOr(led.warm_duty_logical,'--');el('ledCoolDutyLogical').textContent=valueOr(led.cool_duty_logical,'--');el('ledWarmDutyOutput').textContent=valueOr(led.warm_duty_output,'--');el('ledCoolDutyOutput').textContent=valueOr(led.cool_duty_output,'--');\n"
        "        el('ledWarmRatio').textContent=percentText(mix.warm_ratio);el('ledCoolRatio').textContent=percentText(mix.cool_ratio);el('ledMixLimit').textContent=enabledText(!!mix.clipped);el('ledSourceMode').textContent=controlModeText(led.source_mode);el('ledSourceReason').textContent=faultReasonText(led.source_reason);el('ledRiskScore').textContent=fmt(c.posture_score);el('ledVoiceAlert').textContent=voiceText(!!c.voice_alert_required);el('ledAppAlert').textContent=appText(!!c.app_alert_required);el('ledFaultReason').textContent=faultReasonText(led.fault_reason);\n"
        "        drawSystemOverlay(data);\n"
        "        el('fetchStatus').textContent='\\u7cfb\\u7edf\\u6b63\\u5e38';\n"
        "      }\n"
        "      async function pollSystem(){\n"
        "        try{const resp=await fetch('/system_result?t='+Date.now(),{cache:'no-store'});if(!resp.ok){el('fetchStatus').textContent='\\u7cfb\\u7edf\\u8bf7\\u6c42\\u5931\\u8d25 '+resp.status;return;}const data=await resp.json();applySystem(data);}\n"
        "        catch(e){el('fetchStatus').textContent='\\u7cfb\\u7edf\\u8bf7\\u6c42\\u5931\\u8d25';console.error(e);}\n"
        "      }\n"
        "      const SYSTEM_RESULT_INTERVAL_MS=500;\n"
        "      img.addEventListener('load',resizeCanvas);window.addEventListener('resize',resizeCanvas);setInterval(pollSystem,SYSTEM_RESULT_INTERVAL_MS);pollSystem();\n"
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
    if (std::strcmp(req->uri, "/face_result") == 0) {
        s_http_face_result_count++;
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

    static constexpr size_t JSON_BUF_SIZE = 12288;
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
    s_http_status_count++;
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
    s_http_range_result_count++;
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

static esp_err_t fusion_result_handler(httpd_req_t *req)
{
    s_http_fusion_result_count++;
    fusion_state_update_from_latest();

    fusion_state_t state = {};
    if (!fusion_state_get_snapshot(&state)) {
        state.valid = false;
        state.state_text = "not_initialized";
        state.invalid_reason = "not_initialized";
    }

    char json[2048];
    const int len = std::snprintf(json,
                                  sizeof(json),
                                  "{\n"
                                  "  \"valid\": %s,\n"
                                  "  \"target_valid\": %s,\n"
                                  "  \"face_valid\": %s,\n"
                                  "  \"range_valid\": %s,\n"
                                  "  \"timestamp_ms\": %" PRIu32 ",\n"
                                  "  \"frame_id\": %" PRIu32 ",\n"
                                  "  \"face_count\": %d,\n"
                                  "  \"displayed_face_count\": %d,\n"
                                  "  \"distance_mm\": %u,\n"
                                  "  \"filtered_distance_mm\": %u,\n"
                                  "  \"distance_in_range\": %s,\n"
                                  "  \"too_near\": %s,\n"
                                  "  \"too_far\": %s,\n"
                                  "  \"target_present_by_range\": %s,\n"
                                  "  \"lost_with_range_valid\": %s,\n"
                                  "  \"recent_face_valid\": %s,\n"
                                  "  \"suspected_head_down_no_face\": %s,\n"
                                  "  \"face_lost_ms\": %" PRIu32 ",\n"
                                  "  \"last_face_valid_ms\": %" PRIu32 ",\n"
                                  "  \"last_pose_valid_ms\": %" PRIu32 ",\n"
                                  "  \"pitch\": %.2f,\n"
                                  "  \"yaw\": %.2f,\n"
                                  "  \"roll\": %.2f,\n"
                                  "  \"filtered_pitch\": %.2f,\n"
                                  "  \"filtered_yaw\": %.2f,\n"
                                  "  \"filtered_roll\": %.2f,\n"
                                  "  \"face_score\": %.2f,\n"
                                  "  \"range_confidence\": %.2f,\n"
                                  "  \"fusion_confidence\": %.2f,\n"
                                  "  \"state_text\": \"%s\",\n"
                                  "  \"invalid_reason\": \"%s\",\n"
                                  "  \"vision_age_ms\": %" PRIu32 ",\n"
                                  "  \"range_age_ms\": %" PRIu32 ",\n"
                                  "  \"update_count\": %" PRIu32 ",\n"
                                  "  \"invalid_count\": %" PRIu32 "\n"
                                  "}\n",
                                  state.valid ? "true" : "false",
                                  state.target_valid ? "true" : "false",
                                  state.face_valid ? "true" : "false",
                                  state.range_valid ? "true" : "false",
                                  state.timestamp_ms,
                                  state.frame_id,
                                  state.face_count,
                                  state.displayed_face_count,
                                  static_cast<unsigned>(state.distance_mm),
                                  static_cast<unsigned>(state.filtered_distance_mm),
                                  state.distance_in_range ? "true" : "false",
                                  state.too_near ? "true" : "false",
                                  state.too_far ? "true" : "false",
                                  state.target_present_by_range ? "true" : "false",
                                  state.lost_with_range_valid ? "true" : "false",
                                  state.recent_face_valid ? "true" : "false",
                                  state.suspected_head_down_no_face ? "true" : "false",
                                  state.face_lost_ms,
                                  state.last_face_valid_ms,
                                  state.last_pose_valid_ms,
                                  static_cast<double>(state.pitch),
                                  static_cast<double>(state.yaw),
                                  static_cast<double>(state.roll),
                                  static_cast<double>(state.filtered_pitch),
                                  static_cast<double>(state.filtered_yaw),
                                  static_cast<double>(state.filtered_roll),
                                  static_cast<double>(state.face_score),
                                  static_cast<double>(state.range_confidence),
                                  static_cast<double>(state.fusion_confidence),
                                  state.state_text != nullptr ? state.state_text : "unknown",
                                  state.invalid_reason != nullptr ? state.invalid_reason : "unknown",
                                  state.vision_age_ms,
                                  state.range_age_ms,
                                  state.update_count,
                                  state.invalid_count);
    if (len <= 0 || static_cast<size_t>(len) >= sizeof(json)) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, len);
}

static esp_err_t control_result_handler(httpd_req_t *req)
{
    s_http_control_result_count++;
    control_policy_update_from_fusion();

    control_state_t state = {};
    if (!control_policy_get_snapshot(&state)) {
        state.valid = false;
        state.mode = control_mode_t::SENSOR_INVALID;
        state.mode_text = "SENSOR_INVALID";
        state.reason = "not_initialized";
        state.target_brightness = 0.0f;
        state.target_cct_k = 0.0f;
    }

    char json[2048];
    const int len = std::snprintf(json,
                                  sizeof(json),
                                  "{\n"
                                  "  \"valid\": %s,\n"
                                  "  \"mode\": \"%s\",\n"
                                  "  \"timestamp_ms\": %" PRIu32 ",\n"
                                  "  \"update_count\": %" PRIu32 ",\n"
                                  "  \"posture_score\": %.2f,\n"
                                  "  \"pitch_score\": %.2f,\n"
                                  "  \"distance_score\": %.2f,\n"
                                  "  \"yaw_score\": %.2f,\n"
                                  "  \"roll_score\": %.2f,\n"
                                  "  \"center_score\": %.2f,\n"
                                  "  \"face_lost_score\": %.2f,\n"
                                  "  \"duration_score\": %.2f,\n"
                                  "  \"fuzzy_level\": %.2f,\n"
                                  "  \"target_brightness\": %.3f,\n"
                                  "  \"target_cct_k\": %.0f,\n"
                                  "  \"warm_pwm_virtual\": %.3f,\n"
                                  "  \"cool_pwm_virtual\": %.3f,\n"
                                  "  \"warm_ratio\": %.3f,\n"
                                  "  \"cool_ratio\": %.3f,\n"
                                  "  \"warm_pwm_raw\": %.3f,\n"
                                  "  \"cool_pwm_raw\": %.3f,\n"
                                  "  \"led_mix_clipped\": %s,\n"
                                  "  \"led_mix_safe_test_limited\": %s,\n"
                                  "  \"voice_alert_required\": %s,\n"
                                  "  \"app_alert_required\": %s,\n"
                                  "  \"normal_ms\": %" PRIu32 ",\n"
                                  "  \"fuzzy_ms\": %" PRIu32 ",\n"
                                  "  \"alert_ms\": %" PRIu32 ",\n"
                                  "  \"bad_posture_ms\": %" PRIu32 ",\n"
                                  "  \"severe_posture_ms\": %" PRIu32 ",\n"
                                  "  \"fusion_valid\": %s,\n"
                                  "  \"fusion_target_valid\": %s,\n"
                                  "  \"fusion_face_valid\": %s,\n"
                                  "  \"fusion_range_valid\": %s,\n"
                                  "  \"suspected_head_down_no_face\": %s,\n"
                                  "  \"target_present_by_range\": %s,\n"
                                  "  \"recent_face_valid\": %s,\n"
                                  "  \"face_lost_ms\": %" PRIu32 ",\n"
                                  "  \"fusion_age_ms\": %" PRIu32 ",\n"
                                  "  \"fusion_frame_id\": %" PRIu32 ",\n"
                                  "  \"fusion_reason\": \"%s\",\n"
                                  "  \"reason\": \"%s\"\n"
                                  "}\n",
                                  state.valid ? "true" : "false",
                                  state.mode_text != nullptr ? state.mode_text : control_mode_to_text(state.mode),
                                  state.timestamp_ms,
                                  state.update_count,
                                  static_cast<double>(state.posture_score),
                                  static_cast<double>(state.pitch_score),
                                  static_cast<double>(state.distance_score),
                                  static_cast<double>(state.yaw_score),
                                  static_cast<double>(state.roll_score),
                                  static_cast<double>(state.center_score),
                                  static_cast<double>(state.face_lost_score),
                                  static_cast<double>(state.duration_score),
                                  static_cast<double>(state.fuzzy_level),
                                  static_cast<double>(state.target_brightness),
                                  static_cast<double>(state.target_cct_k),
                                  static_cast<double>(state.warm_pwm_virtual),
                                  static_cast<double>(state.cool_pwm_virtual),
                                  static_cast<double>(state.warm_ratio),
                                  static_cast<double>(state.cool_ratio),
                                  static_cast<double>(state.warm_pwm_raw),
                                  static_cast<double>(state.cool_pwm_raw),
                                  state.led_mix_clipped ? "true" : "false",
                                  state.led_mix_safe_test_limited ? "true" : "false",
                                  state.voice_alert_required ? "true" : "false",
                                  state.app_alert_required ? "true" : "false",
                                  state.normal_ms,
                                  state.fuzzy_ms,
                                  state.alert_ms,
                                  state.bad_posture_ms,
                                  state.severe_posture_ms,
                                  state.fusion_valid ? "true" : "false",
                                  state.fusion_target_valid ? "true" : "false",
                                  state.fusion_face_valid ? "true" : "false",
                                  state.fusion_range_valid ? "true" : "false",
                                  state.suspected_head_down_no_face ? "true" : "false",
                                  state.target_present_by_range ? "true" : "false",
                                  state.recent_face_valid ? "true" : "false",
                                  state.face_lost_ms,
                                  state.fusion_age_ms,
                                  state.fusion_frame_id,
                                  state.fusion_reason != nullptr ? state.fusion_reason : "unknown",
                                  state.reason != nullptr ? state.reason : "unknown");
    if (len <= 0 || static_cast<size_t>(len) >= sizeof(json)) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, len);
}

static const char *range_state_text_from_values(bool valid, uint16_t distance_mm)
{
    if (!valid || distance_mm == 0) {
        return "invalid";
    }
    if (distance_mm < AILAMP_DISTANCE_MIN_MM) {
        return "too_near";
    }
    if (distance_mm > AILAMP_DISTANCE_MAX_MM) {
        return "too_far";
    }
    return "ok";
}

static esp_err_t system_result_handler(httpd_req_t *req)
{
    s_http_system_result_count++;
    face_http_snapshot_t face = {};
    range_state_t range = {};
    fusion_state_t fusion = {};
    control_state_t control = {};
    LedOutputSnapshot led = {};

    const bool has_face = face_http_result_get_snapshot(&face);
    const bool has_range = range_sensor_get_snapshot(&range);
    const bool has_fusion = fusion_state_get_snapshot(&fusion);
    const bool has_control = control_policy_get_snapshot(&control);
    const bool has_led = led_output_get_snapshot(&led);

    if (!has_face) {
        face.valid = false;
        face.tracking_state = "lost";
    }
    if (!has_range) {
        range.valid = false;
        range.status_text = "not_initialized";
    }
    if (!has_fusion) {
        fusion.valid = false;
        fusion.state_text = "not_initialized";
        fusion.invalid_reason = "not_initialized";
    }
    if (!has_control) {
        control.valid = false;
        control.mode = control_mode_t::SENSOR_INVALID;
        control.mode_text = "SENSOR_INVALID";
        control.reason = "not_initialized";
    }
    if (!has_led) {
        led.initialized = false;
        led.enabled = false;
        led.fault = true;
        led.common_anode_cob = AILAMP_LED_COMMON_ANODE_COB != 0;
        led.active_high = AILAMP_LED_PWM_ACTIVE_HIGH != 0;
        led.swap_warm_cool_channels = AILAMP_LED_SWAP_WARM_COOL_CHANNELS != 0;
        led.safe_test_limit_enable = AILAMP_LED_SAFE_TEST_LIMIT_ENABLE != 0;
        led.warm_gpio = static_cast<int>(AILAMP_LED_WARM_PWM_GPIO);
        led.cool_gpio = static_cast<int>(AILAMP_LED_COOL_PWM_GPIO);
        led.pwm_freq_hz = AILAMP_LED_PWM_FREQ_HZ;
        led.duty_resolution_bits = AILAMP_LED_PWM_RES_BITS;
        led.max_duty = AILAMP_LED_PWM_MAX_DUTY;
        led.source_mode = "unknown";
        led.source_reason = "snapshot_failed";
        led.test_mode = "unknown";
        led.test_mode_active = false;
        led.fault_reason = "snapshot_failed";
    }

    const bool range_too_near = has_range && range.valid && range.distance_mm > 0 &&
                                range.distance_mm < AILAMP_DISTANCE_MIN_MM;
    const bool range_too_far = has_range && range.valid && range.distance_mm > AILAMP_DISTANCE_MAX_MM;
    const uint16_t filtered_distance =
        has_fusion && fusion.filtered_distance_mm > 0 ? fusion.filtered_distance_mm : range.distance_mm;
    const char *range_state_text = range_state_text_from_values(has_range && range.valid, range.distance_mm);

    static constexpr size_t JSON_BUF_SIZE = 12288;
    char json[JSON_BUF_SIZE];
    size_t offset = 0;
    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

    bool ok = true;
    ok = ok && json_append(json,
                           sizeof(json),
                           &offset,
                           "{\n"
                           "  \"timestamp_ms\": %" PRIu32 ",\n"
                           "  \"face\": {\n"
                           "    \"valid\": %s,\n"
                           "    \"count\": %d,\n"
                           "    \"display_count\": %d,\n"
                           "    \"score\": %.3f,\n"
                           "    \"state\": \"%s\",\n"
                           "    \"frame_id\": %" PRIu32 ",\n"
                           "    \"inference_ms\": %d,\n"
                           "    \"hit_rate\": %.2f,\n"
                           "    \"frame_width\": %d,\n"
                           "    \"frame_height\": %d,\n"
                           "    \"faces\": [",
                           now_ms,
                           has_face && face.valid ? "true" : "false",
                           face.actual_face_count,
                           face.display_face_count,
                           static_cast<double>(face.face_score),
                           face.tracking_state != nullptr ? face.tracking_state : "lost",
                           face.frame_id,
                           face.inference_ms,
                           static_cast<double>(face.hit_rate),
                           face.frame_width,
                           face.frame_height);

    for (int i = 0; ok && i < face.overlay_face_count; i++) {
        const auto &item = face.faces[i];
        ok = ok && json_append(json,
                               sizeof(json),
                               &offset,
                               "%s\n"
                               "      {\"x1\": %d, \"y1\": %d, \"x2\": %d, \"y2\": %d, "
                               "\"w\": %d, \"h\": %d, \"score\": %.3f, \"landmarks_valid\": %s",
                               i == 0 ? "" : ",",
                               item.x1,
                               item.y1,
                               item.x2,
                               item.y2,
                               item.w,
                               item.h,
                               static_cast<double>(item.score),
                               item.landmarks_valid ? "true" : "false");
        if (ok && item.landmarks_valid) {
            ok = ok && json_append(json,
                                   sizeof(json),
                                   &offset,
                                   ", \"landmarks\": {"
                                   "\"left_eye\": {\"x\": %.1f, \"y\": %.1f}, "
                                   "\"right_eye\": {\"x\": %.1f, \"y\": %.1f}, "
                                   "\"nose\": {\"x\": %.1f, \"y\": %.1f}, "
                                   "\"left_mouth\": {\"x\": %.1f, \"y\": %.1f}, "
                                   "\"right_mouth\": {\"x\": %.1f, \"y\": %.1f}}",
                                   static_cast<double>(item.landmarks.left_eye.x),
                                   static_cast<double>(item.landmarks.left_eye.y),
                                   static_cast<double>(item.landmarks.right_eye.x),
                                   static_cast<double>(item.landmarks.right_eye.y),
                                   static_cast<double>(item.landmarks.nose.x),
                                   static_cast<double>(item.landmarks.nose.y),
                                   static_cast<double>(item.landmarks.left_mouth.x),
                                   static_cast<double>(item.landmarks.left_mouth.y),
                                   static_cast<double>(item.landmarks.right_mouth.x),
                                   static_cast<double>(item.landmarks.right_mouth.y));
        }
        ok = ok && json_append(json, sizeof(json), &offset, "}");
    }

    ok = ok && json_append(json,
                           sizeof(json),
                           &offset,
                           "\n"
                           "    ]\n"
                           "  },\n");

    ok = ok && json_append(json,
                           sizeof(json),
                           &offset,
                           "  \"pose\": {\n"
                           "    \"valid\": %s,\n"
                           "    \"pitch\": %.2f,\n"
                           "    \"yaw\": %.2f,\n"
                           "    \"roll\": %.2f,\n"
                           "    \"filtered_pitch\": %.2f,\n"
                           "    \"filtered_yaw\": %.2f,\n"
                           "    \"filtered_roll\": %.2f\n"
                           "  },\n",
                           has_face && face.pose_valid ? "true" : "false",
                           static_cast<double>(face.pitch),
                           static_cast<double>(face.yaw),
                           static_cast<double>(face.roll),
                           static_cast<double>(face.filtered_pitch),
                           static_cast<double>(face.filtered_yaw),
                           static_cast<double>(face.filtered_roll));

    ok = ok && json_append(json,
                           sizeof(json),
                           &offset,
                           "  \"range\": {\n"
                           "    \"valid\": %s,\n"
                           "    \"distance_mm\": %u,\n"
                           "    \"filtered_distance_mm\": %u,\n"
                           "    \"state\": \"%s\",\n"
                           "    \"too_near\": %s,\n"
                           "    \"too_far\": %s\n"
                           "  },\n",
                           has_range && range.valid ? "true" : "false",
                           static_cast<unsigned>(range.distance_mm),
                           static_cast<unsigned>(filtered_distance),
                           range_state_text,
                           range_too_near ? "true" : "false",
                           range_too_far ? "true" : "false");

    ok = ok && json_append(json,
                           sizeof(json),
                           &offset,
                           "  \"fusion\": {\n"
                           "    \"valid\": %s,\n"
                           "    \"target_valid\": %s,\n"
                           "    \"face_valid\": %s,\n"
                           "    \"range_valid\": %s,\n"
                           "    \"state\": \"%s\",\n"
                           "    \"confidence\": %.2f,\n"
                           "    \"reason\": \"%s\",\n"
                           "    \"suspected_head_down_no_face\": %s,\n"
                           "    \"face_lost_ms\": %" PRIu32 ",\n"
                           "    \"target_present_by_range\": %s,\n"
                           "    \"recent_face_valid\": %s\n"
                           "  },\n",
                           has_fusion && fusion.valid ? "true" : "false",
                           has_fusion && fusion.target_valid ? "true" : "false",
                           has_fusion && fusion.face_valid ? "true" : "false",
                           has_fusion && fusion.range_valid ? "true" : "false",
                           fusion.state_text != nullptr ? fusion.state_text : "unknown",
                           static_cast<double>(fusion.fusion_confidence),
                           fusion.invalid_reason != nullptr ? fusion.invalid_reason : "unknown",
                           has_fusion && fusion.suspected_head_down_no_face ? "true" : "false",
                           has_fusion ? fusion.face_lost_ms : 0,
                           has_fusion && fusion.target_present_by_range ? "true" : "false",
                           has_fusion && fusion.recent_face_valid ? "true" : "false");

    ok = ok && json_append(json,
                           sizeof(json),
                           &offset,
                           "  \"control\": {\n"
                           "    \"valid\": %s,\n"
                           "    \"mode\": \"%s\",\n"
                           "    \"posture_score\": %.2f,\n"
                           "    \"pitch_score\": %.2f,\n"
                           "    \"distance_score\": %.2f,\n"
                           "    \"yaw_score\": %.2f,\n"
                           "    \"roll_score\": %.2f,\n"
                           "    \"center_score\": %.2f,\n"
                           "    \"face_lost_score\": %.2f,\n"
                           "    \"duration_score\": %.2f,\n"
                           "    \"fuzzy_level\": %.2f,\n"
                           "    \"brightness\": %.3f,\n"
                           "    \"cct_k\": %.0f,\n"
                           "    \"warm_pwm_virtual\": %.3f,\n"
                           "    \"cool_pwm_virtual\": %.3f,\n"
                           "    \"voice_alert_required\": %s,\n"
                           "    \"app_alert_required\": %s,\n"
                           "    \"reason\": \"%s\",\n"
                           "    \"face_lost_ms\": %" PRIu32 ",\n"
                           "    \"suspected_head_down_no_face\": %s,\n"
                           "    \"target_present_by_range\": %s\n"
                           "  },\n"
                           "  \"hardware_power\": {\n"
                           "    \"battery_nominal_v\": 22.2,\n"
                           "    \"battery_full_v\": 25.2,\n"
                           "    \"esp32_supply_from_lm2596\": true,\n"
                           "    \"lm2596_output_target_v\": 5.0,\n"
                           "    \"led_driver_control_vcc\": 5.0,\n"
                           "    \"led_driver_control_header\": \"J3: GND/PWM2/PWM1/VCC\",\n"
                           "    \"cob_type\": \"common_anode_dual_cct\",\n"
                           "    \"cob_pad\": \"+/C-/W-\",\n"
                           "    \"cob_spec\": \"20-22V/300mA/7W/Ra95\",\n"
                           "    \"warning\": \"COB must be driven by constant-current LED driver\"\n"
                           "  },\n"
                           "  \"led_mix\": {\n"
                           "    \"valid\": %s,\n"
                           "    \"warm_cct_k\": %d,\n"
                           "    \"cool_cct_k\": %d,\n"
                           "    \"use_min_cct_k\": %d,\n"
                           "    \"normal_cct_k\": %d,\n"
                           "    \"alert_cct_k\": %d,\n"
                           "    \"use_max_cct_k\": %d,\n"
                           "    \"total_pwm_limit\": %.2f,\n"
                           "    \"safe_test_limit_enable\": %s,\n"
                           "    \"safe_test_pwm_limit\": %.2f,\n"
                           "    \"brightness_limited\": %.3f,\n"
                           "    \"cct_limited\": %.0f,\n"
                           "    \"warm_ratio\": %.3f,\n"
                           "    \"cool_ratio\": %.3f,\n"
                           "    \"warm_pwm_raw\": %.3f,\n"
                           "    \"cool_pwm_raw\": %.3f,\n"
                           "    \"warm_pwm\": %.3f,\n"
                           "    \"cool_pwm\": %.3f,\n"
                           "    \"clipped\": %s,\n"
                           "    \"safe_test_limited\": %s\n"
                           "  },\n"
                           "  \"led_output\": {\n"
                           "    \"valid\": %s,\n"
                           "    \"initialized\": %s,\n"
                           "    \"enabled\": %s,\n"
                           "    \"fault\": %s,\n"
                           "    \"common_anode_cob\": %s,\n"
                           "    \"active_high\": %s,\n"
                           "    \"swap_warm_cool_channels\": %s,\n"
                           "    \"safe_test_limit_enable\": %s,\n"
                           "    \"warm_gpio\": %d,\n"
                           "    \"cool_gpio\": %d,\n"
                           "    \"pwm_freq_hz\": %" PRIu32 ",\n"
                           "    \"duty_resolution_bits\": %u,\n"
                           "    \"max_duty\": %" PRIu32 ",\n"
                           "    \"warm_target\": %.3f,\n"
                           "    \"cool_target\": %.3f,\n"
                           "    \"warm_current\": %.3f,\n"
                           "    \"cool_current\": %.3f,\n"
                           "    \"warm_duty_logical\": %" PRIu32 ",\n"
                           "    \"cool_duty_logical\": %" PRIu32 ",\n"
                           "    \"warm_duty_output\": %" PRIu32 ",\n"
                           "    \"cool_duty_output\": %" PRIu32 ",\n"
                           "    \"source_mode\": \"%s\",\n"
                           "    \"source_reason\": \"%s\",\n"
                           "    \"test_mode\": \"%s\",\n"
                           "    \"test_mode_active\": %s,\n"
                           "    \"update_count\": %" PRIu32 ",\n"
                           "    \"last_update_ms\": %" PRIu32 ",\n"
                           "    \"fault_reason\": \"%s\"\n"
                           "  },\n"
                           "  \"debug\": {\n"
                           "    \"http\": {\n"
                           "      \"stream_count\": %" PRIu32 ",\n"
                           "      \"system_result_count\": %" PRIu32 ",\n"
                           "      \"status_count\": %" PRIu32 ",\n"
                           "      \"face_result_count\": %" PRIu32 ",\n"
                           "      \"range_result_count\": %" PRIu32 ",\n"
                           "      \"fusion_result_count\": %" PRIu32 ",\n"
                           "      \"control_result_count\": %" PRIu32 "\n"
                           "    }\n"
                           "  }\n"
                           "}\n",
                           has_control && control.valid ? "true" : "false",
                           control.mode_text != nullptr ? control.mode_text : control_mode_to_text(control.mode),
                           static_cast<double>(control.posture_score),
                           static_cast<double>(control.pitch_score),
                           static_cast<double>(control.distance_score),
                           static_cast<double>(control.yaw_score),
                           static_cast<double>(control.roll_score),
                           static_cast<double>(control.center_score),
                           static_cast<double>(control.face_lost_score),
                           static_cast<double>(control.duration_score),
                           static_cast<double>(control.fuzzy_level),
                           static_cast<double>(control.target_brightness),
                           static_cast<double>(control.target_cct_k),
                           static_cast<double>(control.warm_pwm_virtual),
                           static_cast<double>(control.cool_pwm_virtual),
                           control.voice_alert_required ? "true" : "false",
                           control.app_alert_required ? "true" : "false",
                           control.reason != nullptr ? control.reason : "unknown",
                           has_control ? control.face_lost_ms : 0,
                           has_control && control.suspected_head_down_no_face ? "true" : "false",
                           has_control && control.target_present_by_range ? "true" : "false",
                           has_control ? "true" : "false",
                           AILAMP_COB_WARM_CCT_K,
                           AILAMP_COB_COOL_CCT_K,
                           AILAMP_USE_MIN_CCT_K,
                           AILAMP_USE_NORMAL_CCT_K,
                           AILAMP_USE_ALERT_CCT_K,
                           AILAMP_USE_MAX_CCT_K,
                           static_cast<double>(AILAMP_LED_TOTAL_PWM_LIMIT),
                           AILAMP_LED_SAFE_TEST_LIMIT_ENABLE ? "true" : "false",
                           static_cast<double>(AILAMP_LED_SAFE_TEST_PWM_LIMIT),
                           static_cast<double>(control.target_brightness),
                           static_cast<double>(control.target_cct_k),
                           static_cast<double>(control.warm_ratio),
                           static_cast<double>(control.cool_ratio),
                           static_cast<double>(control.warm_pwm_raw),
                           static_cast<double>(control.cool_pwm_raw),
                           static_cast<double>(control.warm_pwm_virtual),
                           static_cast<double>(control.cool_pwm_virtual),
                           control.led_mix_clipped ? "true" : "false",
                           control.led_mix_safe_test_limited ? "true" : "false",
                           has_led ? "true" : "false",
                           has_led && led.initialized ? "true" : "false",
                           has_led && led.enabled ? "true" : "false",
                           !has_led || led.fault ? "true" : "false",
                           led.common_anode_cob ? "true" : "false",
                           led.active_high ? "true" : "false",
                           led.swap_warm_cool_channels ? "true" : "false",
                           led.safe_test_limit_enable ? "true" : "false",
                           has_led ? led.warm_gpio : static_cast<int>(AILAMP_LED_WARM_PWM_GPIO),
                           has_led ? led.cool_gpio : static_cast<int>(AILAMP_LED_COOL_PWM_GPIO),
                           has_led ? led.pwm_freq_hz : static_cast<uint32_t>(AILAMP_LED_PWM_FREQ_HZ),
                           static_cast<unsigned>(has_led ? led.duty_resolution_bits : AILAMP_LED_PWM_RES_BITS),
                           has_led ? led.max_duty : static_cast<uint32_t>(AILAMP_LED_PWM_MAX_DUTY),
                           static_cast<double>(has_led ? led.warm_target : 0.0f),
                           static_cast<double>(has_led ? led.cool_target : 0.0f),
                           static_cast<double>(has_led ? led.warm_current : 0.0f),
                           static_cast<double>(has_led ? led.cool_current : 0.0f),
                           has_led ? led.warm_duty_logical : 0,
                           has_led ? led.cool_duty_logical : 0,
                           has_led ? led.warm_duty_output : 0,
                           has_led ? led.cool_duty_output : 0,
                           has_led && led.source_mode != nullptr ? led.source_mode : "unknown",
                           has_led && led.source_reason != nullptr ? led.source_reason : "snapshot_failed",
                           has_led && led.test_mode != nullptr ? led.test_mode : "unknown",
                           has_led && led.test_mode_active ? "true" : "false",
                           has_led ? led.update_count : 0,
                           has_led ? led.last_update_ms : 0,
                           has_led && led.fault_reason != nullptr ? led.fault_reason : "snapshot_failed",
                           s_http_stream_count,
                           s_http_system_result_count,
                           s_http_status_count,
                           s_http_face_result_count,
                           s_http_range_result_count,
                           s_http_fusion_result_count,
                           s_http_control_result_count);

    if (!ok) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_type(req, JSON_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, offset);
}

#if AILAMP_LED_TEST_HTTP_ENABLE
static esp_err_t led_test_handler(httpd_req_t *req)
{
    char query[64] = {};
    char mode_value[16] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "mode", mode_value, sizeof(mode_value)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "missing mode");
    }

    LedOutputTestMode mode = LedOutputTestMode::NORMAL;
    if (std::strcmp(mode_value, "off") == 0) {
        mode = LedOutputTestMode::OFF;
    } else if (std::strcmp(mode_value, "warm") == 0) {
        mode = LedOutputTestMode::WARM;
    } else if (std::strcmp(mode_value, "cool") == 0) {
        mode = LedOutputTestMode::COOL;
    } else if (std::strcmp(mode_value, "both") == 0) {
        mode = LedOutputTestMode::BOTH;
    } else if (std::strcmp(mode_value, "normal") == 0) {
        mode = LedOutputTestMode::NORMAL;
    } else {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "invalid mode");
    }

    const esp_err_t err = led_output_set_test_mode(mode);
    if (err != ESP_OK) {
        return httpd_resp_send_500(req);
    }

    char json[80];
    const int len = std::snprintf(json,
                                  sizeof(json),
                                  "{\"ok\":true,\"mode\":\"%s\"}\n",
                                  led_output_test_mode_to_text(mode));
    if (len <= 0 || static_cast<size_t>(len) >= sizeof(json)) {
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, JSON_CONTENT_TYPE);
    return httpd_resp_send(req, json, len);
}
#endif

static esp_err_t stream_handler(httpd_req_t *req)
{
    s_http_stream_count++;
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
    const httpd_uri_t fusion_result_uri = {
        .uri = "/fusion_result",
        .method = HTTP_GET,
        .handler = fusion_result_handler,
        .user_ctx = nullptr,
    };
    const httpd_uri_t control_result_uri = {
        .uri = "/control_result",
        .method = HTTP_GET,
        .handler = control_result_handler,
        .user_ctx = nullptr,
    };
    const httpd_uri_t system_result_uri = {
        .uri = "/system_result",
        .method = HTTP_GET,
        .handler = system_result_handler,
        .user_ctx = nullptr,
    };
#if AILAMP_LED_TEST_HTTP_ENABLE
    const httpd_uri_t led_test_uri = {
        .uri = "/led_test",
        .method = HTTP_GET,
        .handler = led_test_handler,
        .user_ctx = nullptr,
    };
#endif

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &capture_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &face_result_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &result_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &range_result_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &fusion_result_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &control_result_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &system_result_uri));
#if AILAMP_LED_TEST_HTTP_ENABLE
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &led_test_uri));
#endif

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
