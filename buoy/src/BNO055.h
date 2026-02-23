#include "IMU.h" // class template for IMUs that BNO055 will override

class IMU {
    public:

        IMU();

        void read_sensor();

        const IMUdata_t& get_IMU_data() { return data; };

    private:
        char *i2cDevice = nullptr;
        uint8_t i2cAddr = 0x0;
        IMUdata_t data;
};