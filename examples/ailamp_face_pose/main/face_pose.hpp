#pragma once

struct FaceLandmarks5 {
    bool valid;
    float left_eye_x;
    float left_eye_y;
    float right_eye_x;
    float right_eye_y;
    float nose_x;
    float nose_y;
    float left_mouth_x;
    float left_mouth_y;
    float right_mouth_x;
    float right_mouth_y;
};

struct HeadPose {
    bool valid;
    float pitch_deg;
    float yaw_deg;
    float roll_deg;
};

HeadPose estimate_head_pose_from_5pts(const FaceLandmarks5 &lm, int frame_width, int frame_height);
