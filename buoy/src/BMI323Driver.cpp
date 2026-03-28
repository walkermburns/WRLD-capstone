#include "BMI323Driver.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

BMI323Driver::BMI323Driver(const std::string &spiDevice, uint32_t spiSpeedHz)
    : transport(spiDevice, spiSpeedHz, 0, 8)
{
    dev = {};
}

BMI3_INTF_RET_TYPE BMI323Driver::spiReadCallback(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    SPITransport *transport = static_cast<SPITransport *>(intf_ptr);
    return transport->bmi3Read(reg_addr, reg_data, len, intf_ptr);
}

BMI3_INTF_RET_TYPE BMI323Driver::spiWriteCallback(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    SPITransport *transport = static_cast<SPITransport *>(intf_ptr);
    return transport->bmi3Write(reg_addr, reg_data, len, intf_ptr);
}

void BMI323Driver::delayCallback(uint32_t period, void *intf_ptr)
{
    SPITransport::delayUs(period, intf_ptr);
}

float BMI323Driver::convertAccel(int16_t raw)
{
    constexpr float g_range = 2.0f;
    uint8_t resolution = dev.resolution ? dev.resolution : 16;
    float half_scale = static_cast<float>(1u << (resolution - 1));
    float value_g = (static_cast<float>(raw) * g_range) / half_scale;
    return value_g * 9.80665f; // m/s^2
}

float BMI323Driver::convertGyro(int16_t raw)
{
    constexpr float dps_range = 2000.0f;
    uint8_t resolution = dev.resolution ? dev.resolution : 16;
    float half_scale = static_cast<float>(1u << (resolution - 1));
    return (static_cast<float>(raw) * dps_range) / half_scale;
}

bool BMI323Driver::init()
{
    if (!transport.openDevice()) {
        std::cerr << "[BMI323] failed to open SPI device\n";
        return false;
    }

    dev.intf = BMI3_SPI_INTF;
    dev.intf_ptr = &transport;
    dev.read = spiReadCallback;
    dev.write = spiWriteCallback;
    dev.delay_us = delayCallback;

    int8_t rslt = bmi323_init(&dev);
    if (rslt != BMI323_OK) {
        std::cerr << "[BMI323] bmi323_init failed: " << static_cast<int>(rslt) << "\n";
        return false;
    }

    // Config accel
    {
        struct bmi3_sens_config cfg = {};
        cfg.type = BMI323_ACCEL;
        rslt = bmi323_get_sensor_config(&cfg, 1, &dev);
        if (rslt != BMI323_OK) {
            std::cerr << "[BMI323] get accel config failed: " << static_cast<int>(rslt) << "\n";
            return false;
        }
        cfg.cfg.acc.odr = BMI3_ACC_ODR_100HZ;
        cfg.cfg.acc.range = BMI3_ACC_RANGE_2G;
        cfg.cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;
        cfg.cfg.acc.avg_num = BMI3_ACC_AVG64;
        cfg.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;
        rslt = bmi323_set_sensor_config(&cfg, 1, &dev);
        if (rslt != BMI323_OK) {
            std::cerr << "[BMI323] set accel config failed: " << static_cast<int>(rslt) << "\n";
            return false;
        }
    }

    // Config gyro
    {
        struct bmi3_sens_config cfg = {};
        cfg.type = BMI323_GYRO;
        rslt = bmi323_get_sensor_config(&cfg, 1, &dev);
        if (rslt != BMI323_OK) {
            std::cerr << "[BMI323] get gyro config failed: " << static_cast<int>(rslt) << "\n";
            return false;
        }
        cfg.cfg.gyr.odr = BMI3_GYR_ODR_100HZ;
        cfg.cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;
        cfg.cfg.gyr.bwp = BMI3_GYR_BW_ODR_QUARTER;
        cfg.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
        cfg.cfg.gyr.avg_num = BMI3_GYR_AVG1;
        rslt = bmi323_set_sensor_config(&cfg, 1, &dev);
        if (rslt != BMI323_OK) {
            std::cerr << "[BMI323] set gyro config failed: " << static_cast<int>(rslt) << "\n";
            return false;
        }
    }

    // Initialize quaternion to identity for Mahony filter
    data.quat = {1.0f, 0.0f, 0.0f, 0.0f};
    
    // Read initial accel and set quaternion to match it (avoids startup transient)
    struct bmi3_sensor_data sensors[1] = {};
    sensors[0].type = BMI323_ACCEL;
    rslt = bmi323_get_sensor_data(sensors, 1, &dev);
    if (rslt == BMI323_OK) {
        data.accel.x = convertAccel(sensors[0].sens_data.acc.x);
        data.accel.y = convertAccel(sensors[0].sens_data.acc.y);
        data.accel.z = convertAccel(sensors[0].sens_data.acc.z);
        
        Vec3 accelG = { data.accel.x, data.accel.y, data.accel.z };
        accelG.x /= 9.80665f;
        accelG.y /= 9.80665f;
        accelG.z /= 9.80665f;
        
        data.quat = IMUHelpers::quaternionFromAccel(accelG);
    }

    std::cout << "[BMI323] initialized successfully\n";
    
    // Discard first few stale sensor reads to let chip stabilize
    for (int i = 0; i < 10; ++i) {
        struct bmi3_sensor_data dummy[2] = {};
        dummy[0].type = BMI323_ACCEL;
        dummy[1].type = BMI323_GYRO;
        bmi323_get_sensor_data(dummy, 2, &dev);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    return true;
}

const IMUData &BMI323Driver::readSensor()
{
    struct bmi3_sensor_data sensors[2] = {};
    sensors[0].type = BMI323_ACCEL;
    sensors[1].type = BMI323_GYRO;

    int8_t rslt = bmi323_get_sensor_data(sensors, 2, &dev);
    if (rslt != BMI323_OK) {
        // Keep previous measurement on error.
        return data;
    }

    data.accel.x = convertAccel(sensors[0].sens_data.acc.x);
    data.accel.y = convertAccel(sensors[0].sens_data.acc.y);
    data.accel.z = convertAccel(sensors[0].sens_data.acc.z);

    data.gyro.x = convertGyro(sensors[1].sens_data.gyr.x);
    data.gyro.y = convertGyro(sensors[1].sens_data.gyr.y);
    data.gyro.z = convertGyro(sensors[1].sens_data.gyr.z);

    // Apply IMU axis remapping requested by user so that:
    // new_x = old_y, new_y = old_z, new_z = old_x
    IMUHelpers::rotateIMUAxes(data.accel);
    IMUHelpers::rotateIMUAxes(data.gyro);

    // Subtract calibrated gyro bias (in dps)
    data.gyro.x -= gyroBias.x;
    data.gyro.y -= gyroBias.y;
    data.gyro.z -= gyroBias.z;

    // Convert gyro from dps to rad/s
    const float DEG_TO_RAD = 0.0174533f; // π/180
    Vec3 gyroRadS = {
        data.gyro.x * DEG_TO_RAD,
        data.gyro.y * DEG_TO_RAD,
        data.gyro.z * DEG_TO_RAD
    };

    // Assume 100 Hz sampling (10 ms per sample, matching main.cpp usleep(10000))
    const float dt = 0.01f;

    // Run Mahony filter with Kp=5.0 (matching Arduino code)
    IMUHelpers::mahonyUpdate(data.quat, gyroRadS, data.accel, dt, 5.0f);

    return data;
}

bool BMI323Driver::calibrateGyro(size_t samples, int delayMs)
{
    if (!transport.isOpen()) {
        std::cerr << "[BMI323] device not open for calibration\n";
        return false;
    }

    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    for (size_t i = 0; i < samples; ++i) {
        struct bmi3_sensor_data sensors[1] = {};
        sensors[0].type = BMI323_GYRO;

        int8_t rslt = bmi323_get_sensor_data(sensors, 1, &dev);
        if (rslt != BMI323_OK) {
            std::cerr << "[BMI323] calibration read failed: " << static_cast<int>(rslt) << "\n";
            return false;
        }

        float gx = convertGyro(sensors[0].sens_data.gyr.x);
        float gy = convertGyro(sensors[0].sens_data.gyr.y);
        float gz = convertGyro(sensors[0].sens_data.gyr.z);

        sumX += gx;
        sumY += gy;
        sumZ += gz;

        std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
    }

    gyroBias.x = static_cast<float>(sumX / static_cast<double>(samples));
    gyroBias.y = static_cast<float>(sumY / static_cast<double>(samples));
    gyroBias.z = static_cast<float>(sumZ / static_cast<double>(samples));

    std::cout << "[BMI323] gyro bias calibrated: " << gyroBias.x << ", " << gyroBias.y << ", " << gyroBias.z << " dps\n";
    return true;
}
