#pragma once

struct PoseRaw {
    bool valid;
    float pitch;
    float yaw;
    float roll;
};

struct PoseFiltered {
    bool valid;
    float pitch;
    float yaw;
    float roll;
    int lost_count;
};

struct PoseFilterDebug {
    bool initialized;
    int lost_count;
    int reset_count;
    const char *last_reset_reason;
    float alpha;
    float max_delta;
};

class PoseEmaFilter {
public:
    PoseEmaFilter();
    void reset();
    PoseFiltered update(const PoseRaw &raw, bool face_present);
    PoseFiltered get() const;
    PoseFilterDebug debug() const;

private:
    void reset_to_raw(const PoseRaw &raw, const char *reason);

    bool m_initialized;
    bool m_had_face;
    bool m_has_seen_valid;
    PoseFiltered m_state;
    PoseFilterDebug m_debug;
};

void pose_calibration_set_neutral(float pitch, float yaw, float roll);
void pose_calibration_reset();
PoseRaw pose_calibration_apply(const PoseFiltered &filtered);
