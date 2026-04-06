#pragma once

#include "IMU.h"
#include <cmath>

namespace IMUHelpers {

inline void normalizeQuat(Quaternion &q)
{
    float len = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (len <= 0.0f) return;
    q.w /= len;
    q.x /= len;
    q.y /= len;
    q.z /= len;
}

inline void normalizeAccel(Vec3 &v)
{
    float len = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len <= 0.000001f) return;
    v.x /= len;
    v.y /= len;
    v.z /= len;
}

// Remap IMU axes so that:
//  - new +X = old +Y
//  - new +Y = old +Z
//  - new +Z = old +X
inline void rotateIMUAxes(Vec3 &v)
{
    float ox = v.x;
    float oy = v.y;
    float oz = v.z;
    v.x = -oy;
    v.y = -oz;
    v.z = ox;
}

inline Quaternion quaternionFromAccel(const Vec3 &accel)
{
    // roll/pitch from accelerometer; yaw unknown so set to 0.
    float ax = accel.x;
    float ay = accel.y;
    float az = accel.z;

    float roll = std::atan2(ay, az);
    float pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
    float yaw = 0.0f;

    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    normalizeQuat(q);
    return q;
}

inline Quaternion slerp(const Quaternion &a, const Quaternion &b, float t)
{
    // Normalize to avoid error accumulation
    Quaternion qa = a;
    Quaternion qb = b;
    normalizeQuat(qa);
    normalizeQuat(qb);

    float dot = qa.w*qb.w + qa.x*qb.x + qa.y*qb.y + qa.z*qb.z;
    if (dot < 0.0f) {
        dot = -dot;
        qb.w = -qb.w;
        qb.x = -qb.x;
        qb.y = -qb.y;
        qb.z = -qb.z;
    }

    const float DOT_THRESHOLD = 0.9995f;
    if (dot > DOT_THRESHOLD) {
        // Linear interpolation to avoid precision issues
        Quaternion res;
        res.w = qa.w + t * (qb.w - qa.w);
        res.x = qa.x + t * (qb.x - qa.x);
        res.y = qa.y + t * (qb.y - qa.y);
        res.z = qa.z + t * (qb.z - qa.z);
        normalizeQuat(res);
        return res;
    }

    float theta_0 = std::acos(dot);
    float theta = theta_0 * t;
    float sin_theta = std::sin(theta);
    float sin_theta_0 = std::sin(theta_0);

    float s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    Quaternion out;
    out.w = s0 * qa.w + s1 * qb.w;
    out.x = s0 * qa.x + s1 * qb.x;
    out.y = s0 * qa.y + s1 * qb.y;
    out.z = s0 * qa.z + s1 * qb.z;
    normalizeQuat(out);
    return out;
}

inline Quaternion lowPassQuat(const Quaternion &previous, const Quaternion &current, float alpha)
{
    if (alpha <= 0.0f) return previous;
    if (alpha >= 1.0f) return current;
    return slerp(previous, current, alpha);
}

inline void accelToQuatAndFilter(const Vec3 &rawAccel, Quaternion &outQuat, Quaternion &prevQuat, float alpha = 0.1f)
{
    Vec3 normalized = rawAccel;
    normalizeAccel(normalized);

    Quaternion current = quaternionFromAccel(normalized);

    if (prevQuat.w == 0.0f && prevQuat.x == 0.0f && prevQuat.y == 0.0f && prevQuat.z == 0.0f) {
        // first measurement
        outQuat = current;
    } else {
        outQuat = lowPassQuat(prevQuat, current, alpha);
    }

    prevQuat = outQuat;
}

// Mahony filter: Integrates gyro rates and corrects using acceleration error.
// Similar to the Arduino imu_code.ino implementation.
// Call this once per IMU sample with gyro (rad/s), accel (m/s²), and dt (seconds).
inline void mahonyUpdate(Quaternion &q, const Vec3 &gyro_rad_s, const Vec3 &accel_m_s2, float dt, float Kp = 5.0f)
{
    // Normalize acceleration to get gravity direction
    Vec3 a = accel_m_s2;
    normalizeAccel(a);

    // Compute gravity direction predicted by current quaternion
    float gx_from_q = 2.0f * (q.x * q.z - q.w * q.y);
    float gy_from_q = 2.0f * (q.y * q.z + q.w * q.x);
    float gz_from_q = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);

    float g_mag = std::sqrt(gx_from_q * gx_from_q + gy_from_q * gy_from_q + gz_from_q * gz_from_q);
    if (g_mag > 0.0001f) {
        gx_from_q /= g_mag;
        gy_from_q /= g_mag;
        gz_from_q /= g_mag;
    }

    // Cross-product error between measured and predicted gravity
    float g_err_x = a.y * gz_from_q - a.z * gy_from_q;
    float g_err_y = a.z * gx_from_q - a.x * gz_from_q;
    float g_err_z = a.x * gy_from_q - a.y * gx_from_q;

    // Corrected gyro rates
    float wx = gyro_rad_s.x + Kp * g_err_x;
    float wy = gyro_rad_s.y + Kp * g_err_y;
    float wz = gyro_rad_s.z + Kp * g_err_z;

    // Compute quaternion derivative (delta quaternion from angular rate)
    float theta = std::sqrt(wx * wx + wy * wy + wz * wz) * dt;

    Quaternion dq;
    if (theta < 1e-6f) {
        // Small-angle approximation
        float halfdt = 0.5f * dt;
        dq.w = 1.0f;
        dq.x = wx * halfdt;
        dq.y = wy * halfdt;
        dq.z = wz * halfdt;
    } else {
        float invw = 1.0f / std::sqrt(wx * wx + wy * wy + wz * wz);
        float ux = wx * invw, uy = wy * invw, uz = wz * invw;
        float half = 0.5f * theta;
        float s = std::sin(half);
        dq.w = std::cos(half);
        dq.x = ux * s;
        dq.y = uy * s;
        dq.z = uz * s;
    }

    // Update quaternion: q_new = q * dq (Hamilton product)
    Quaternion q_new;
    q_new.w = q.w * dq.w - q.x * dq.x - q.y * dq.y - q.z * dq.z;
    q_new.x = q.w * dq.x + q.x * dq.w + q.y * dq.z - q.z * dq.y;
    q_new.y = q.w * dq.y - q.x * dq.z + q.y * dq.w + q.z * dq.x;
    q_new.z = q.w * dq.z + q.x * dq.y - q.y * dq.x + q.z * dq.w;

    normalizeQuat(q_new);
    q = q_new;
}

// Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
// Uses ZYX Euler convention (yaw-pitch-roll).
inline void quatToEulerZYX(const Quaternion &q, float &roll, float &pitch, float &yaw)
{
    float w = q.w, x = q.x, y = q.y, z = q.z;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    if (std::fabs(sinp) >= 1.0f)
        pitch = std::copysign(static_cast<float>(M_PI) / 2.0f, sinp);
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// Converts radians to degrees
inline float radToDeg(float rad)
{
    return rad * 57.29577951308232f; // 180 / π
}

// ::Quaternion quat_from_accel(float ax, float ay, float az)
// {
//     ::Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};

//     float norm = std::sqrt(ax*ax + ay*ay + az*az);
//     if (norm < 1e-6f)
//         return q;
//     ax /= norm;
//     ay /= norm;
//     az /= norm;

//     float roll  = std::atan2(ay, az);
//     float pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
//     float yaw   = 0.0f;

//     float cy = cosf(yaw * 0.5f);
//     float sy = sinf(yaw * 0.5f);
//     float cp = cosf(pitch * 0.5f);
//     float sp = sinf(pitch * 0.5f);
//     float cr = cosf(roll * 0.5f);
//     float sr = sinf(roll * 0.5f);

//     q.w = cr*cp*cy + sr*sp*sy;
//     q.x = sr*cp*cy - cr*sp*sy;
//     q.y = cr*sp*cy + sr*cp*sy;
//     q.z = cr*cp*sy - sr*sp*cy;

// //     return q;
// }

} // namespace IMUHelpers
