#pragma once

#include "IMU.h"
#include "I2CTransport.h"
#include "BNO055_registers.h"

#include <string>

// Driver for the Bosch BNO055 IMU.  The implementation hides all of the
// chip‑specific register magic and talks to the sensor over an
// I2CTransport instance.  The public API is defined by IMUInterface so
// that calling code can remain sensor‑agnostic.

class BNO055Driver : public IMUInterface {
public:
    BNO055Driver(const std::string &i2cDevice = "/dev/i2c-1", uint8_t address = 0x28);

    bool init() override;
    const IMUData_t &readSensor() override;

private:
    I2CTransport transport;
    IMUData_t data;
};