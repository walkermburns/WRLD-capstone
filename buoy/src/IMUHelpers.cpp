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

} // namespace IMUHelpers
