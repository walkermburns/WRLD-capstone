#include "BNO055.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cstdio>

IMU::IMU() {
    this->i2cDevice = "/dev/i2c-1";
    this->i2cAddr = 0x28;
}

bool IMU::init()
{

    g_fd = open(i2cDevice, O_RDWR);
    if (g_fd < 0) {
        perror("I2C open failed");
        return false;
    }

    if (ioctl(g_fd, I2C_SLAVE, i2cAddr) < 0) {
        perror("I2C ioctl failed");
        return false;
    }

    // Force PAGE 0
    uint8_t cmd[2] = {REG_PAGE_ID, 0x00};
    write(g_fd, cmd, 2);
    usleep(10000);

    // Check CHIP ID
    uint8_t reg = REG_CHIP_ID;
    uint8_t chip;
    write(g_fd, &reg, 1);
    read(g_fd, &chip, 1);

    if (chip != 0xA0) {
        printf("BNO055 not detected (CHIP_ID=0x%02X)\n", chip);
        return false;
    }

    // Enter config mode
    cmd[0] = REG_OPR_MODE;
    cmd[1] = MODE_CONFIG;
    write(g_fd, cmd, 2);
    usleep(30000);

    // Normal power
    cmd[0] = REG_PWR_MODE;
    cmd[1] = POWER_NORMAL;
    write(g_fd, cmd, 2);
    usleep(10000);

    // NDOF fusion mode
    cmd[0] = REG_OPR_MODE;
    cmd[1] = MODE_NDOF;
    write(g_fd, cmd, 2);
    usleep(100000); // allow fusion engine to start

    printf("BNO055 initialized successfully\n");
    return true;
}

const IMUData_t& IMU::read_sensor()
{
    constexpr uint8_t REG_ACC_DATA = 0x08;
    constexpr uint8_t REG_GYR_DATA = 0x14;
    constexpr uint8_t REG_QUA_DATA = 0x20;

    uint8_t buffer[14]; // 6 accel + 6 gyro + 8 quaternion (we'll read separately)

    // -------- Accel + Gyro (12 bytes total) --------
    uint8_t reg = REG_ACC_DATA;
    write(g_fd, &reg, 1);

    uint8_t imu_data[12];
    read(g_fd, imu_data, 12);

    int16_t accX = (imu_data[1] << 8) | imu_data[0];
    int16_t accY = (imu_data[3] << 8) | imu_data[2];
    int16_t accZ = (imu_data[5] << 8) | imu_data[4];

    int16_t gyrX = (imu_data[7] << 8) | imu_data[6];
    int16_t gyrY = (imu_data[9] << 8) | imu_data[8];
    int16_t gyrZ = (imu_data[11] << 8) | imu_data[10];

    data.ax = accX * 0.01f;
    data.ay = accY * 0.01f;
    data.az = accZ * 0.01f;

    data.gx = gyrX / 16.0f;
    data.gy = gyrY / 16.0f;
    data.gz = gyrZ / 16.0f;

    // -------- Quaternion (8 bytes) --------
    reg = REG_QUA_DATA;
    write(g_fd, &reg, 1);

    uint8_t qdata[8];
    read(g_fd, qdata, 8);

    int16_t qw = (qdata[1] << 8) | qdata[0];
    int16_t qx = (qdata[3] << 8) | qdata[2];
    int16_t qy = (qdata[5] << 8) | qdata[4];
    int16_t qz = (qdata[7] << 8) | qdata[6];

    constexpr float scale = 1.0f / (1 << 14);

    data.qw = qw * scale;
    data.qx = qx * scale;
    data.qy = qy * scale;
    data.qz = qz * scale;

    return data;
}

// void IMU::read_sensor() {

//     int fd = open(i2cDevice, O_RDWR);
//     if (fd < 0) {
//         perror("open");
//         return;
//     }

//     if (ioctl(fd, I2C_SLAVE, i2cAddr) < 0) {
//         perror("ioctl");
//         close(fd);
//         return;
//     }

//     uint8_t reg, val;

//     // ---- Force PAGE 0 ----
//     uint8_t pageCmd[2] = {REG_PAGE_ID, 0x00};
//     write(fd, pageCmd, 2);
//     usleep(10000);

//     // ---- Read and print IDs ----
//     auto readReg = [&](uint8_t r) -> uint8_t {
//         write(fd, &r, 1);
//         read(fd, &val, 1);
//         return val;
//     };

//     uint8_t chipID = readReg(0x00);
//     uint8_t accID  = readReg(0x01);
//     uint8_t magID  = readReg(0x02);
//     uint8_t gyrID  = readReg(0x03);

//     printf("BNO055 IDs:\n");
//     printf("  CHIP_ID: 0x%02X (expected 0xA0)\n", chipID);
//     printf("  ACC_ID : 0x%02X\n", accID);
//     printf("  MAG_ID : 0x%02X\n", magID);
//     printf("  GYR_ID : 0x%02X\n", gyrID);

//     if (chipID != 0xA0) {
//         printf("ERROR: BNO055 not detected correctly!\n");
//         close(fd);
//         return;
//     }

//     // ---- CONFIG mode ----
//     uint8_t cmd[2];
//     cmd[0] = REG_OPR_MODE;
//     cmd[1] = MODE_CONFIG;
//     write(fd, cmd, 2);
//     usleep(30000);

//     // ---- Normal power ----
//     cmd[0] = REG_PWR_MODE;
//     cmd[1] = POWER_NORMAL;
//     write(fd, cmd, 2);
//     usleep(10000);

//     // ---- NDOF mode ----
//     cmd[0] = REG_OPR_MODE;
//     cmd[1] = MODE_NDOF;
//     write(fd, cmd, 2);

//     // IMPORTANT: wait for fusion engine
//     usleep(100000); // 100 ms

//     // ---- Read accelerometer ----
//     reg = REG_ACC_DATA;
//     write(fd, &reg, 1);

//     uint8_t accBuf[6];
//     read(fd, accBuf, 6);

//     int16_t accX = (accBuf[1] << 8) | accBuf[0];
//     int16_t accY = (accBuf[3] << 8) | accBuf[2];
//     int16_t accZ = (accBuf[5] << 8) | accBuf[4];

//     float ax = accX * 0.01f;
//     float ay = accY * 0.01f;
//     float az = accZ * 0.01f;

//     // ---- Read gyroscope ----
//     reg = REG_GYR_DATA;
//     write(fd, &reg, 1);

//     uint8_t gyrBuf[6];
//     read(fd, gyrBuf, 6);

//     int16_t gyrX = (gyrBuf[1] << 8) | gyrBuf[0];
//     int16_t gyrY = (gyrBuf[3] << 8) | gyrBuf[2];
//     int16_t gyrZ = (gyrBuf[5] << 8) | gyrBuf[4];

//     float gx = gyrX / 16.0f;
//     float gy = gyrY / 16.0f;
//     float gz = gyrZ / 16.0f;

//     // ---- Print data ----
//     printf("Accel [m/s^2]: X=%7.2f  Y=%7.2f  Z=%7.2f\n", ax, ay, az);
//     printf("Gyro  [deg/s]: X=%7.2f  Y=%7.2f  Z=%7.2f\n", gx, gy, gz);
//     printf("--------------------------------------------------\n");

//     close(fd);
//     return;
// }