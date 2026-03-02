#include "I2CTransport.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdio>

I2CTransport::I2CTransport(const std::string &devicePath, uint8_t address)
    : devicePath_(devicePath), address_(address), fd_(-1) {}

I2CTransport::~I2CTransport()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool I2CTransport::openDevice()
{
    fd_ = open(devicePath_.c_str(), O_RDWR);
    if (fd_ < 0) {
        perror("I2C open failed");
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        perror("I2C ioctl failed");
        close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

bool I2CTransport::writeBytes(const uint8_t *data, size_t length)
{
    if (fd_ < 0) {
        fprintf(stderr, "I2C write attempted on unopened device\n");
        return false;
    }

    ssize_t written = write(fd_, data, length);
    if (written != (ssize_t)length) {
        perror("I2C write failed");
        return false;
    }

    return true;
}

bool I2CTransport::readBytes(uint8_t *buffer, size_t length)
{
    if (fd_ < 0) {
        fprintf(stderr, "I2C read attempted on unopened device\n");
        return false;
    }

    ssize_t read_bytes = read(fd_, buffer, length);
    if (read_bytes != (ssize_t)length) {
        perror("I2C read failed");
        return false;
    }

    return true;
}

bool I2CTransport::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return writeBytes(buf, sizeof(buf));
}

bool I2CTransport::readRegister(uint8_t reg, void *buf, size_t len)
{
    if (!writeBytes(&reg, 1))
        return false;
    return readBytes(reinterpret_cast<uint8_t *>(buf), len);
}
