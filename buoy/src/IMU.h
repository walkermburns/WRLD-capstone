#pragma once

#include <cstdint>

// simple data container for readings produced by any IMU implementation.
// keeping it POD allows it to be copied/stored in queues without hairy
// ownership semantics.

typedef struct {
    float ax, ay, az;   // m/s^2
    float gx, gy, gz;   // deg/s
    float qw, qx, qy, qz; // unit quaternion
} IMUData_t;

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
    virtual const IMUData_t &readSensor() = 0;
};