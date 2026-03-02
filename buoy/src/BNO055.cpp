#include "BNO055.h"

#include <unistd.h> // usleep
#include <cstdio>

// this driver delegates all of the I2C mechanics to the transport class and
// uses the register definitions from the dedicated header.

BNO055Driver::BNO055Driver(const std::string &device, uint8_t address)
    : transport(device, address)
{
}

bool BNO055Driver::init()
{
    if (!transport.openDevice())
        return false;

    // Force PAGE 0
    if (!transport.writeRegister(BNO055_regs::Register::PAGE_ID, 0x00))
        return false;
    usleep(10000);

    // Check CHIP ID
    uint8_t chip;
    if (!transport.readRegister(BNO055_regs::Register::CHIP_ID, &chip, 1))
        return false;

    if (chip != 0xA0) {
        printf("BNO055 not detected (CHIP_ID=0x%02X)\n", chip);
        return false;
    }

    // Enter config mode
    if (!transport.writeRegister(BNO055_regs::Register::OPR_MODE,
                                 BNO055_regs::OperationMode::MODE_CONFIG))
        return false;
    usleep(30000);

    // Normal power
    if (!transport.writeRegister(BNO055_regs::Register::PWR_MODE,
                                 BNO055_regs::PowerMode::POWER_NORMAL))
        return false;
    usleep(10000);

    // NDOF fusion mode
    if (!transport.writeRegister(BNO055_regs::Register::OPR_MODE,
                                 BNO055_regs::OperationMode::MODE_NDOF))
        return false;
    usleep(100000); // allow fusion engine to start

    printf("BNO055 initialized successfully\n");
    return true;
}

const IMUData &BNO055Driver::readSensor()
{
    uint8_t imu_data[12];
    if (!transport.readRegister(BNO055_regs::Register::ACC_DATA, imu_data,
                                sizeof(imu_data))) {
        // reading failure; return previous sample
        return data;
    }

    int16_t accX = (imu_data[1] << 8) | imu_data[0];
    int16_t accY = (imu_data[3] << 8) | imu_data[2];
    int16_t accZ = (imu_data[5] << 8) | imu_data[4];
    int16_t gyrX = (imu_data[7] << 8) | imu_data[6];
    int16_t gyrY = (imu_data[9] << 8) | imu_data[8];
    int16_t gyrZ = (imu_data[11] << 8) | imu_data[10];

    data.accel.x = accX * 0.01f;
    data.accel.y = accY * 0.01f;
    data.accel.z = accZ * 0.01f;

    data.gyro.x = gyrX / 16.0f;
    data.gyro.y = gyrY / 16.0f;
    data.gyro.z = gyrZ / 16.0f;

    uint8_t qdata[8];
    if (transport.readRegister(BNO055_regs::Register::QUA_DATA, qdata,
                               sizeof(qdata))) {
        int16_t qw = (qdata[1] << 8) | qdata[0];
        int16_t qx = (qdata[3] << 8) | qdata[2];
        int16_t qy = (qdata[5] << 8) | qdata[4];
        int16_t qz = (qdata[7] << 8) | qdata[6];

        constexpr float scale = 1.0f / (1 << 14);
        data.quat.w = qw * scale;
        data.quat.x = qx * scale;
        data.quat.y = qy * scale;
        data.quat.z = qz * scale;
    }

    return data;
}