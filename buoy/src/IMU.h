#pragma once
#include <stdint.h>

typedef struct {
    float x;
    float y;
    float z;
} mora_float_vec_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} mora_int_vec_t;

// typedef struct {
//     mora_int_vec_t acc_raw;
//     mora_float_vec_t acc;
//     mora_int_vec_t gyro_raw;
//     mora_float_vec_t gyro;
// } IMUdata_t;

typedef struct {
    float ax, ay, az;   // m/s^2
    float gx, gy, gz;   // deg/s
    float qw, qx, qy, qz; // unit quaternion
} IMUData_t;