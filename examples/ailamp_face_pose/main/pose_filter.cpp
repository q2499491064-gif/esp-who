#include "pose_filter.hpp"

#include <algorithm>
#include <cmath>

static constexpr int kPoseLostInvalidCount = 5;
static constexpr float kBaseAlpha = 0.25f;
static constexpr float kFastAlpha = 0.65f;
static constexpr float kFastThresholdDeg = 12.0f;
static constexpr float kYawJumpResetDeg = 35.0f;
static constexpr float kPitchJumpResetDeg = 30.0f;
static constexpr float kRollJumpResetDeg = 30.0f;

struct PoseNeutralOffset {
    float pitch;
    float yaw;
    float roll;
};

static PoseNeutralOffset s_neutral_offset = {};

PoseEmaFilter::PoseEmaFilter() : m_initialized(false), m_had_face(false), m_has_seen_valid(false), m_state({}), m_debug({})
{
    m_debug.last_reset_reason = "none";
    m_debug.alpha = kBaseAlpha;
}

void PoseEmaFilter::reset()
{
    m_initialized = false;
    m_had_face = false;
    m_has_seen_valid = false;
    m_state = {};
    m_debug.initialized = false;
    m_debug.lost_count = 0;
    m_debug.alpha = kBaseAlpha;
    m_debug.max_delta = 0.0f;
    m_debug.last_reset_reason = "manual";
}

void PoseEmaFilter::reset_to_raw(const PoseRaw &raw, const char *reason)
{
    m_state.valid = true;
    m_state.pitch = raw.pitch;
    m_state.yaw = raw.yaw;
    m_state.roll = raw.roll;
    m_state.lost_count = 0;
    m_initialized = true;
    m_has_seen_valid = true;
    m_debug.initialized = true;
    m_debug.lost_count = 0;
    m_debug.reset_count++;
    m_debug.last_reset_reason = reason;
    m_debug.alpha = 1.0f;
}

PoseFiltered PoseEmaFilter::update(const PoseRaw &raw, bool face_present)
{
    if (!face_present) {
        m_had_face = false;
        m_debug.last_reset_reason = "none";
        m_debug.alpha = 0.0f;
        m_debug.max_delta = 0.0f;
        if (m_initialized) {
            m_state.lost_count++;
            m_debug.lost_count = m_state.lost_count;
            if (m_state.lost_count > kPoseLostInvalidCount) {
                m_state.valid = false;
                m_initialized = false;
                m_debug.initialized = false;
                m_debug.reset_count++;
                m_debug.last_reset_reason = "face_lost";
            }
        } else if (m_state.lost_count <= kPoseLostInvalidCount) {
            m_state.lost_count++;
            m_debug.lost_count = m_state.lost_count;
        }
        return m_state;
    }

    if (!raw.valid) {
        m_had_face = true;
        m_debug.last_reset_reason = "none";
        m_debug.alpha = 0.0f;
        m_debug.max_delta = 0.0f;
        m_debug.initialized = m_initialized;
        m_debug.lost_count = m_state.lost_count;
        return m_state;
    }

    const bool face_reacquired = !m_had_face && m_initialized;
    m_had_face = true;

    if (!m_initialized) {
        reset_to_raw(raw, m_has_seen_valid ? "face_reacquired" : "first_valid");
        return m_state;
    }
    if (face_reacquired) {
        reset_to_raw(raw, "face_reacquired");
        return m_state;
    }

    const float delta_pitch = std::fabs(raw.pitch - m_state.pitch);
    const float delta_yaw = std::fabs(raw.yaw - m_state.yaw);
    const float delta_roll = std::fabs(raw.roll - m_state.roll);
    m_debug.max_delta = std::max(delta_pitch, std::max(delta_yaw, delta_roll));

    if (delta_yaw > kYawJumpResetDeg) {
        reset_to_raw(raw, "yaw_jump");
        return m_state;
    }
    if (delta_pitch > kPitchJumpResetDeg) {
        reset_to_raw(raw, "pitch_jump");
        return m_state;
    }
    if (delta_roll > kRollJumpResetDeg) {
        reset_to_raw(raw, "roll_jump");
        return m_state;
    }

    const float alpha = m_debug.max_delta > kFastThresholdDeg ? kFastAlpha : kBaseAlpha;
    const float keep = 1.0f - alpha;
    m_state.pitch = alpha * raw.pitch + keep * m_state.pitch;
    m_state.yaw = alpha * raw.yaw + keep * m_state.yaw;
    m_state.roll = alpha * raw.roll + keep * m_state.roll;
    m_state.valid = true;
    m_state.lost_count = 0;

    m_debug.initialized = true;
    m_debug.lost_count = 0;
    m_debug.last_reset_reason = "none";
    m_debug.alpha = alpha;
    return m_state;
}

PoseFiltered PoseEmaFilter::get() const
{
    return m_state;
}

PoseFilterDebug PoseEmaFilter::debug() const
{
    return m_debug;
}

void pose_calibration_set_neutral(float pitch, float yaw, float roll)
{
    s_neutral_offset.pitch = pitch;
    s_neutral_offset.yaw = yaw;
    s_neutral_offset.roll = roll;
}

void pose_calibration_reset()
{
    s_neutral_offset = {};
}

PoseRaw pose_calibration_apply(const PoseFiltered &filtered)
{
    if (!filtered.valid) {
        return {};
    }
    return {
        true,
        filtered.pitch - s_neutral_offset.pitch,
        filtered.yaw - s_neutral_offset.yaw,
        filtered.roll - s_neutral_offset.roll,
    };
}
