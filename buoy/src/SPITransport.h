#pragma once

#include <cstdint>
#include <string>

#include "bmi3_defs.h"

class SPITransport {
public:
    SPITransport(const std::string &device = "/dev/spidev0.0", uint32_t speedHz = 10000000,
                 uint8_t mode = 0, uint8_t bitsPerWord = 8);
    ~SPITransport();

    bool openDevice();
    bool isOpen() const;

    BMI3_INTF_RET_TYPE bmi3Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    BMI3_INTF_RET_TYPE bmi3Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static void delayUs(uint32_t period, void *intf_ptr);

private:
    std::string devicePath_;
    int fd_;
    uint32_t speedHz_;
    uint8_t mode_;
    uint8_t bitsPerWord_;
};
