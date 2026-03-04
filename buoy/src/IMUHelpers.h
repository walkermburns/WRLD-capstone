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

} // namespace IMUHelpers
