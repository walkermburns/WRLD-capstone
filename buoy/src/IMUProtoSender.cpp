#include "IMUProtoSender.h"
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <string>
#include <cstdint>

IMUProtoSender::IMUProtoSender(const char *ip, int port)
    : sock_(-1)
{
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0)
        return;

    addr_ = {};
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(port);
    inet_pton(AF_INET, ip, &addr_.sin_addr);
}

IMUProtoSender::~IMUProtoSender()
{
    if (sock_ >= 0) {
        close(sock_);
        sock_ = -1;
    }
}

bool IMUProtoSender::sendIMU(const IMUData &data)
{
    if (sock_ < 0)
        return false;

    buoy_proto::IMU_proto msg;
    msg.set_acc_x(data.accel.x);
    msg.set_acc_y(data.accel.y);
    msg.set_acc_z(data.accel.z);
    msg.set_gyr_x(data.gyro.x);
    msg.set_gyr_y(data.gyro.y);
    msg.set_gyr_z(data.gyro.z);
    msg.set_quat_w(data.quat.w);
    msg.set_quat_x(data.quat.x);
    msg.set_quat_y(data.quat.y);
    msg.set_quat_z(data.quat.z);

    uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    msg.set_timestamp(ts);

    std::string buffer;
    if (!msg.SerializeToString(&buffer))
        return false;

    ssize_t sent = sendto(sock_, buffer.data(), buffer.size(), 0,
                          (sockaddr*)&addr_, sizeof(addr_));
    return sent == (ssize_t)buffer.size();
}
