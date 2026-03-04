#include "IMUHelpers.h"

#include <cmath>

namespace IMUHelpers {

::Quaternion quat_from_accel(float ax, float ay, float az)
{
    ::Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};

    float norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f)
        return q;
    ax /= norm;
    ay /= norm;
    az /= norm;

    float roll  = std::atan2(ay, az);
    float pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
    float yaw   = 0.0f;

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;

    return q;
}

::Quaternion quat_normalize(const ::Quaternion &q)
{
    float mag = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (mag < 1e-6f)
        return ::Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
    return ::Quaternion{q.w/mag, q.x/mag, q.y/mag, q.z/mag};
}

::Quaternion quat_lerp(const ::Quaternion &a,
                      const ::Quaternion &b,
                      float alpha)
{
    // simple linear interpolation and normalise afterwards
    ::Quaternion r;
    r.w = a.w + alpha * (b.w - a.w);
    r.x = a.x + alpha * (b.x - a.x);
    r.y = a.y + alpha * (b.y - a.y);
    r.z = a.z + alpha * (b.z - a.z);
    return quat_normalize(r);
}

} // namespace IMUHelpers
