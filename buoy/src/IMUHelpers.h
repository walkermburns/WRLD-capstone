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
