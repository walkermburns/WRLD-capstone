#pragma once

#include "IMU.h"          // defines IMUData
#include "buoy.pb.h"
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <chrono>
#include <cstdint>

// Small helper that owns a UDP socket and knows how to serialize and send
// protobuf messages for IMU data.  The heavy lifting of queueing is still
// handled in main; this class simply takes populated IMUData instances and
// transmits them on the configured address.

class IMUProtoSender {
public:
    IMUProtoSender(const char *ip, int port);
    ~IMUProtoSender();

    // send a single IMU message; returns false if the socket failed.
    bool sendIMU(const IMUData &data);

private:
    int sock_;
    sockaddr_in addr_;
};
