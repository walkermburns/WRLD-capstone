#pragma once

#include <cstdint>

// small helper types make the intent clear and let you add utility methods
struct Vec3 {
    float x, y, z;
};

struct Quaternion {
    float w, x, y, z;
};

// primary data container used by all IMU drivers.  remains POD so it can be
// copied into queues and sent over the network without any allocator or
// ownership issues.
struct IMUData {
    Vec3 accel;        // m/s²
    Vec3 gyro;         // °/s
    Quaternion quat;   // unit
};


// abstract base class defining the public interface for all IMU drivers.
// drivers such as BNO055 inherit from this so that higher‑level code can
// treat sensors polymorphically (for instance, swapping in a mocked
// implementation for testing).

class IMUInterface {
public:
    virtual ~IMUInterface() = default;

    // initialise the sensor; returns true on success.
    virtual bool init() = 0;

    // read the latest sample from the device.  The reference returned
    // remains valid until the next call to readSensor().
    virtual const IMUData &readSensor() = 0;
};