#include "IMU.h" // class template for IMUs that BNO055 will override

constexpr uint8_t REG_CHIP_ID  = 0x00;
constexpr uint8_t REG_PAGE_ID  = 0x07;
constexpr uint8_t REG_OPR_MODE = 0x3D;
constexpr uint8_t REG_PWR_MODE = 0x3E;
constexpr uint8_t REG_SYS_TRIGGER = 0x3F;

constexpr uint8_t MODE_CONFIG  = 0x00;
constexpr uint8_t MODE_NDOF    = 0x0C;
constexpr uint8_t POWER_NORMAL = 0x00;

class IMU {
    public:

        IMU();

        bool init();
        const IMUData_t& read_sensor();

        // const IMUdata_t& get_IMU_data() { return data; };

    private:
        char *i2cDevice = nullptr;
        uint8_t i2cAddr = 0x0;
        int g_fd = -1;
        IMUData_t data;
};