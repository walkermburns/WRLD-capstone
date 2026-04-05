#include "SPITransport.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <vector>

SPITransport::SPITransport(const std::string &device, uint32_t speedHz, uint8_t mode, uint8_t bitsPerWord)
    : devicePath_(device), fd_(-1), speedHz_(speedHz), mode_(mode), bitsPerWord_(bitsPerWord) {}

SPITransport::~SPITransport()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool SPITransport::openDevice()
{
    fd_ = open(devicePath_.c_str(), O_RDWR);
    if (fd_ < 0) {
        perror("SPI open failed");
        return false;
    }

    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0) {
        perror("SPI set mode failed");
        close(fd_);
        fd_ = -1;
        return false;
    }

    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord_) < 0) {
        perror("SPI set bits per word failed");
        close(fd_);
        fd_ = -1;
        return false;
    }

    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speedHz_) < 0) {
        perror("SPI set speed failed");
        close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

bool SPITransport::isOpen() const
{
    return fd_ >= 0;
}

BMI3_INTF_RET_TYPE SPITransport::bmi3Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;

    if (fd_ < 0 || reg_data == nullptr || len == 0)
        return -1;

    std::vector<uint8_t> tx(len + 1);
    std::vector<uint8_t> rx(len + 1);

    tx[0] = reg_addr;
    // remaining bytes are dummy 0

    struct spi_ioc_transfer tr = {};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    tr.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    tr.len = len + 1;
    tr.speed_hz = speedHz_;
    tr.bits_per_word = bitsPerWord_;

    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        perror("SPI transfer read failed");
        return -1;
    }

    std::memcpy(reg_data, rx.data() + 1, len);
    return BMI3_INTF_RET_SUCCESS;
}

BMI3_INTF_RET_TYPE SPITransport::bmi3Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;

    if (fd_ < 0 || reg_data == nullptr || len == 0)
        return -1;

    std::vector<uint8_t> tx(len + 1);
    tx[0] = reg_addr;
    std::memcpy(tx.data() + 1, reg_data, len);

    struct spi_ioc_transfer tr = {};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    tr.rx_buf = 0;
    tr.len = len + 1;
    tr.speed_hz = speedHz_;
    tr.bits_per_word = bitsPerWord_;

    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        perror("SPI transfer write failed");
        return -1;
    }

    return BMI3_INTF_RET_SUCCESS;
}

void SPITransport::delayUs(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    usleep(period);
}
