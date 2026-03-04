#pragma once

#include "IMU.h"   // provides Quaternion type

// small collection of IMU‑related math helpers that live in the buoy
// project.  the base station has its own MathHelpers, so we avoid
// depending on that code here.
namespace IMUHelpers {

// estimate orientation quaternion from accelerometer vector alone. yaw is
// left at zero since a single gravity measurement can't determine heading.
// input components need not be normalised.
::Quaternion quat_from_accel(float ax, float ay, float az);

// return a unit quaternion in the same direction as `q`.
::Quaternion quat_normalize(const ::Quaternion &q);

// linearly interpolate between two quaternions then normalise result.
// alpha=0 returns `a`, alpha=1 returns `b`.
::Quaternion quat_lerp(const ::Quaternion &a,
                      const ::Quaternion &b,
                      float alpha);

} // namespace IMUHelpers
