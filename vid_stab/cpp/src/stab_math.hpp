#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

struct StabConfig {
  int w = 640;
  int h = 480;
  float hfov_deg = 50.f;

  float video_delay_s = 0.0f;

  float gain = 1.0f;
  float smooth_alpha = 1.0f;
  float max_tilt_rad = static_cast<float>(35.0 * M_PI / 180.0);
};

struct StabState {
  bool have_ref = false;
  Eigen::Quaternionf q_ref = Eigen::Quaternionf::Identity();

  float pitch_f = 0.f;
  float roll_f = 0.f;
};

inline Eigen::Matrix3f make_K(int w, int h, float hfov_deg) {
  const float fx = 0.5f * w / std::tan((hfov_deg * float(M_PI) / 180.f) * 0.5f);
  const float fy = fx;
  const float cx = 0.5f * w;
  const float cy = 0.5f * h;

  Eigen::Matrix3f K;
  K << fx, 0.f, cx,
       0.f, fy, cy,
       0.f, 0.f, 1.f;
  return K;
}

// FLU -> OpenCV camera coords (x right, y down, z forward)
// cv_x = -flu_y, cv_y = -flu_z, cv_z = flu_x
inline Eigen::Matrix3f R_flu_to_cv() {
  Eigen::Matrix3f R;
  R << 0.f, -1.f, 0.f,
       0.f,  0.f,-1.f,
       1.f,  0.f, 0.f;
  return R;
}

// EXACTLY matches your Arduino quatToEulerZYX (q=(w,x,y,z))
// Returns [yaw, pitch, roll] in radians
inline Eigen::Vector3f ypr_zyx_from_quat_arduino(const Eigen::Quaternionf& q) {
  const float w = q.w(), x = q.x(), y = q.y(), z = q.z();

  // roll (x)
  const float sinr_cosp = 2.0f * (w*x + y*z);
  const float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
  const float roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y)
  const float sinp = 2.0f * (w*y - z*x);
  float pitch;
  if (std::abs(sinp) >= 1.0f) pitch = std::copysign(float(M_PI)/2.0f, sinp);
  else pitch = std::asin(sinp);

  // yaw (z)
  const float siny_cosp = 2.0f * (w*z + x*y);
  const float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
  const float yaw = std::atan2(siny_cosp, cosy_cosp);

  return {yaw, pitch, roll};
}