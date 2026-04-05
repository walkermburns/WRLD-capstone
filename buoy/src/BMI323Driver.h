#pragma once

#include "IMU.h"
#include "SPITransport.h"
#include "bmi323.h"
#include "IMUHelpers.h"

class BMI323Driver : public IMUInterface {
public:
    BMI323Driver(const std::string &spiDevice = "/dev/spidev0.0", uint32_t spiSpeedHz = 10000000);
    bool init() override;
    const IMUData &readSensor() override;
    // Calibrate gyro bias by averaging `samples` readings while sensor is stationary.
    // Returns true on success and stores the computed bias which is subtracted
    // from subsequent `readSensor()` results.
    bool calibrateGyro(size_t samples = 500, int delayMs = 2);
    void setGyroBias(const Vec3 &bias) { gyroBias = bias; }

private:
    static BMI3_INTF_RET_TYPE spiReadCallback(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static BMI3_INTF_RET_TYPE spiWriteCallback(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static void delayCallback(uint32_t period, void *intf_ptr);

    float convertAccel(int16_t raw);
    float convertGyro(int16_t raw);

    SPITransport transport;
    struct bmi3_dev dev = {};
    IMUData data = {};
    // Static default gyro bias (dps). Change these values if you want
    // a different hardcoded offset instead of running calibration.
    Vec3 gyroBias = {0.12207f, 0.0610352f, 0.0610352f};
};
