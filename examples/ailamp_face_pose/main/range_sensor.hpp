#pragma once

#include <cstdint>

struct range_state_t {
    bool initialized;
    bool valid;
    uint16_t distance_mm;
    uint8_t status;
    uint32_t timestamp_ms;
    uint32_t read_count;
    uint32_t error_count;
    uint32_t consecutive_ok_count;
    uint32_t consecutive_fail_count;
    uint32_t read_ms;
    const char *status_text;
};

bool range_sensor_init(void);
void range_sensor_start_task(void);
bool range_sensor_get_snapshot(range_state_t *out);
void range_sensor_reset(void);
