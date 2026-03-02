#pragma once

#include <cstdint>
#include <string>

// Simple Linux I2C transport wrapper that encapsulates file descriptor
// handling, register reads/writes, and low-level error checking.  Higher
// level sensor drivers can depend on this class instead of dealing with
// ioctl/open/close themselves.

class I2CTransport {
public:
    I2CTransport(const std::string &devicePath, uint8_t address);
    ~I2CTransport();

    // open the underlying /dev/i2c-* device and set the slave address
    bool openDevice();

    // low‑level primitives exposed for convenience.  They return false on
    // error and print a perror() message.
    bool writeBytes(const uint8_t *data, size_t length);
    bool readBytes(uint8_t *buffer, size_t length);

    // utilities for accessing registers in the common "write register
    // address then read/write data" idiom that most sensor chips use.
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, void *buf, size_t len);

private:
    std::string devicePath_;
    uint8_t address_;
    int fd_ = -1;
};
