#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

// 200 Hz update
const uint32_t PERIOD_US = 5000;   // 1/200 s --> us
static uint32_t next_us = 0;

// 100 Hz print
const uint32_t PRINT_MS = 10;      // 1/20 s --> ms
static uint32_t next_print_ms = 0;

struct Quat {float w, x, y, z; };
static Quat q = {1.0, 0, 0, 0};
static const float Kp = 1.0f;

static const float gyro_x_bias = 0.013;
static const float gyro_y_bias = -0.053;
static const float gyro_z_bias = -0.01;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(10);
}

void loop() {
  uint32_t now_us = micros();
  // float dt = (current_millis - previous_millis) / 1000.0; //s

  if ((int32_t)(now_us - next_us) >=0) {
    next_us += PERIOD_US;
    float dt = PERIOD_US * 1e-6f;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gyro_x_meas = g.gyro.x - gyro_x_bias;
    float gyro_y_meas = g.gyro.y - gyro_y_bias;
    float gyro_z_meas = g.gyro.z - gyro_z_bias;

    mahony_update(gyro_x_meas, gyro_y_meas, gyro_z_meas, a.acceleration.x, a.acceleration.y, a.acceleration.z, dt);
  }

  uint32_t now_ms = millis();
  if ((int32_t)(now_ms - next_print_ms) >= 0) {
    next_print_ms += PRINT_MS;

    float roll, pitch, yaw;
    quatToEulerZYX(q, roll, pitch, yaw);

    // Serial.print("roll:");
    Serial.print(rad2deg(roll), 2);
    Serial.print(",");
    // Serial.print("pitch:");
    Serial.print(rad2deg(pitch), 2);
    Serial.print(",");
    // Serial.print("yaw:");
    Serial.print(rad2deg(yaw), 2); 
    Serial.print(",");
    Serial.print(q.w, 2);
    Serial.print(",");
    Serial.print(q.x, 2);
    Serial.print(",");
    Serial.print(q.y, 2);
    Serial.print(",");
    Serial.println(q.z, 2);


  }

  // if (dt >= interval) {
    
    
    // if (dt >= 0.1) {
        /* Print out the values */
        // Serial.print("AccelX:");
        // Serial.print(a.acceleration.x);
        // Serial.print(",");
        // // Serial.print("AccelY:");
        // Serial.print(a.acceleration.y);
        // Serial.print(",");
        // // Serial.print("AccelZ:");
        // Serial.print(a.acceleration.z);
        // Serial.print(",");
        // Serial.print("GyroX:");
        // Serial.print(gyro_x_meas, 4);
        // Serial.print(",");
        // // Serial.print("GyroY:");
        // Serial.print(gyro_y_meas, 4);
        // Serial.print(",");
        // // Serial.print("GyroZ:");
        // Serial.print(gyro_z_meas, 4);
        // Serial.println("");

        

    // }

    

  // }
}

void mahony_update(float wx_rad_s, float wy_rad_s, float wz_rad_s, float ax_m_s2, float ay_m_s2, float az_m_s2, float dt) {
  // Normalize accel
  float accel_mag = sqrt(ax_m_s2 * ax_m_s2 + ay_m_s2 * ay_m_s2 + az_m_s2 * az_m_s2);
  float accel_x_dir = ax_m_s2 / accel_mag;
  float accel_y_dir = ay_m_s2 / accel_mag;
  float accel_z_dir = az_m_s2 / accel_mag;

  // Serial.print("accel_x_dir:");
  // Serial.print(accel_x_dir);
  // Serial.print(",");
  // Serial.print("accel_y_dir:");
  // Serial.print(accel_y_dir);
  // Serial.print(",");
  // Serial.print("accel_z_dir:");
  // Serial.print(accel_z_dir);
  // Serial.println("");
  

  // Find gravity direction based on current quaternion
  float gx_from_q, gy_from_q, gz_from_q;
  gravity_from_quat(q, gx_from_q, gy_from_q, gz_from_q);

  float g_err_x = accel_y_dir * gz_from_q - accel_z_dir * gy_from_q;
  float g_err_y = accel_z_dir * gx_from_q - accel_x_dir * gz_from_q;
  float g_err_z = accel_x_dir * gy_from_q - accel_y_dir * gx_from_q;

  // Serial.print("g_quat_x_dir:");
  // Serial.print(gx_from_q);
  // Serial.print(",");
  // Serial.print("g_quat_y_dir:");
  // Serial.print(gy_from_q);
  // Serial.print(",");
  // Serial.print("g_quat_z_dir:");
  // Serial.print(gz_from_q);
  // Serial.println("");

  // Serial.print("g_err_x:");
  // Serial.print(g_err_x);
  // Serial.print(",");
  // Serial.print("g_err_y:");
  // Serial.print(g_err_y);
  // Serial.print(",");
  // Serial.print("g_err_z:");
  // Serial.print(g_err_z);
  // Serial.println("");

  wx_rad_s = wx_rad_s + Kp * g_err_x;
  wy_rad_s = wy_rad_s + Kp * g_err_y;
  wz_rad_s = wz_rad_s + Kp * g_err_z;

  Quat dq = delta_quat_from_ang_rate(wx_rad_s, wy_rad_s, wz_rad_s, dt);
  q = quatMul(q, dq);
  quatNormalize(q);

}

static inline void gravity_from_quat(const Quat &q, float &gx, float &gy, float &gz) {
  gx = 2.0f * (q.x * q.z - q.w * q.y);
  gy = 2.0f * (q.y * q.z + q.w * q.x);
  gz = 1.0f- 2.0f * (q.x * q.x + q.y * q.y);

  float g_mag = sqrt(gx * gx + gy * gy + gz * gz);
  gx = gx / g_mag;
  gy = gy / g_mag;
  gz = gz / g_mag;

}

static inline Quat delta_quat_from_ang_rate(float wx, float wy, float wz, float dt) {
  float theta = sqrtf(wx * wx + wy * wy + wz * wz) * dt;
  if (theta < 1e-6f) {
    // small-angle approximation
    float halfdt = 0.5f * dt;
    return {1.0f, wx * halfdt, wy * halfdt, wz * halfdt};
  } else {
    float invw = 1.0f / sqrtf(wx * wx + wy * wy + wz * wz);
    float ux = wx * invw, uy = wy * invw, uz = wz * invw;
    float half = 0.5f * theta;
    float s = sinf(half);
    return {cosf(half), ux * s, uy * s, uz * s};
  }
}

static inline Quat quatMul(const Quat &a, const Quat &b) {
  // Hamilton product: a ⊗ b
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

static inline void quatNormalize(Quat &q) {
  float n = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  if (n <= 0.0f) { q = {1,0,0,0}; return; }
  float invn = 1.0f / sqrtf(n);
  q.w *= invn; q.x *= invn; q.y *= invn; q.z *= invn;
}

static inline float rad2deg(float r) { return r * 57.29577951308232f; }

// q = (w,x,y,z)
void quatToEulerZYX(const Quat &q, float &roll, float &pitch, float &yaw) {
  float w = q.w, x = q.x, y = q.y, z = q.z;

  // Roll (x-axis rotation), FLU
  float sinr_cosp = 2.0f * (w*x + y*z);
  float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
  roll = atan2f(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation), FLU
  float sinp = 2.0f * (w*y - z*x);
  if (fabsf(sinp) >= 1.0f)
    pitch = copysignf((float)M_PI/2.0f, sinp);
  else
    pitch = asinf(sinp);

  // Yaw (z-axis rotation), FLU
  float siny_cosp = 2.0f * (w*z + x*y);
  float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
  yaw = atan2f(siny_cosp, cosy_cosp);
}




