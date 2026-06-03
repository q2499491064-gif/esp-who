#include "face_pose.hpp"

#include <cmath>

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kRadToDeg = 180.0f / kPi;
static constexpr float kYawScaleDeg = 55.0f;
static constexpr float kNeutralPitchRatio = 0.50f;
static constexpr float kPitchScaleDeg = 60.0f;
static constexpr float kMinEyeDistPx = 8.0f;
static constexpr float kMinFaceVerticalPx = 12.0f;
static constexpr float kPosePitchMinDeg = -45.0f;
static constexpr float kPosePitchMaxDeg = 45.0f;
static constexpr float kPoseYawMinDeg = -60.0f;
static constexpr float kPoseYawMaxDeg = 60.0f;
static constexpr float kPoseRollMinDeg = -45.0f;
static constexpr float kPoseRollMaxDeg = 45.0f;

static float clamp_pose(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static bool point_in_frame(float x, float y, int frame_width, int frame_height)
{
    return frame_width > 0 && frame_height > 0 && x >= 0.0f && y >= 0.0f && x < frame_width && y < frame_height;
}

HeadPose estimate_head_pose_from_5pts(const FaceLandmarks5 &lm, int frame_width, int frame_height)
{
    HeadPose pose = {};
    if (!lm.valid) {
        return pose;
    }

    if (!point_in_frame(lm.left_eye_x, lm.left_eye_y, frame_width, frame_height) ||
        !point_in_frame(lm.right_eye_x, lm.right_eye_y, frame_width, frame_height) ||
        !point_in_frame(lm.nose_x, lm.nose_y, frame_width, frame_height) ||
        !point_in_frame(lm.left_mouth_x, lm.left_mouth_y, frame_width, frame_height) ||
        !point_in_frame(lm.right_mouth_x, lm.right_mouth_y, frame_width, frame_height)) {
        return pose;
    }

    const float eye_dx = lm.right_eye_x - lm.left_eye_x;
    const float eye_dy = lm.right_eye_y - lm.left_eye_y;
    const float eye_dist = std::sqrt(eye_dx * eye_dx + eye_dy * eye_dy);
    if (eye_dist < kMinEyeDistPx) {
        return pose;
    }

    const float eye_center_x = (lm.left_eye_x + lm.right_eye_x) * 0.5f;
    const float eye_center_y = (lm.left_eye_y + lm.right_eye_y) * 0.5f;
    const float mouth_center_y = (lm.left_mouth_y + lm.right_mouth_y) * 0.5f;
    const float face_vertical = mouth_center_y - eye_center_y;
    if (face_vertical < kMinFaceVerticalPx) {
        return pose;
    }

    const float yaw_raw = (lm.nose_x - eye_center_x) / eye_dist;
    const float pitch_raw = (lm.nose_y - eye_center_y) / face_vertical;

    pose.valid = true;
    pose.roll_deg = clamp_pose(std::atan2(eye_dy, eye_dx) * kRadToDeg, kPoseRollMinDeg, kPoseRollMaxDeg);
    pose.yaw_deg = clamp_pose(yaw_raw * kYawScaleDeg, kPoseYawMinDeg, kPoseYawMaxDeg);
    pose.pitch_deg = clamp_pose((pitch_raw - kNeutralPitchRatio) * kPitchScaleDeg, kPosePitchMinDeg, kPosePitchMaxDeg);
    return pose;
}
