#include "BuoyNode.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>   // for setw, left

// static member definitions
std::mutex BuoyNode::printMutex;

BuoyNode::BuoyNode(std::string name, int imuPort, Callback cb)
    : name_(std::move(name)), imuPort_(imuPort), sock_(-1), callback_(std::move(cb)),
      running_(false)
{
}

BuoyNode::~BuoyNode()
{
    stop();
}

bool BuoyNode::start()
{
    if (running_)
        return true; // already started

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        perror("socket");
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(imuPort_);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock_);
        sock_ = -1;
        return false;
    }

    running_ = true;
    thread_ = std::thread(&BuoyNode::receiveLoop, this);
    return true;
}

void BuoyNode::stop()
{
    // flip flag first so receiveLoop knows it's time to exit
    if (running_) {
        running_ = false;
    }

    if (sock_ >= 0) {
        // shutdown the descriptor before closing; this will unblock a
        // blocking recv() call on another thread, causing it to return
        // -1 with errno=EBADF/ENOTCONN etc.
        shutdown(sock_, SHUT_RD);
        close(sock_);
        sock_ = -1;
    }

    if (thread_.joinable())
        thread_.join();
}

void BuoyNode::receiveLoop()
{
    char buffer[1500];

    while (running_) {
        ssize_t len = recv(sock_, buffer, sizeof(buffer), 0);
        if (len <= 0) {
            if (len < 0) {
                // if the socket was closed due to stop(), errno may be
                // EBADF or EINVAL; treat that as a normal shutdown.
                if (errno != EINTR && errno != EBADF && errno != EINVAL) {
                    perror("recv");
                }
            }
            break; // exit loop when socket is closed or fatal error
        }

        buoy_proto::IMU_proto msg;
        if (!msg.ParseFromArray(buffer, len)) {
            std::cerr << "[" << name_ << "] failed to parse protobuf (size=" << len << ")\n";
            continue;
        }

        if (callback_) {
            callback_(name_, msg);
        }
    }
}

// static helper -------------------------------------------------------------

void BuoyNode::printMessage(const std::string &name,
                            const buoy_proto::IMU_proto &msg)
{
    std::lock_guard<std::mutex> lock(printMutex);

    const int W = 8;
    std::cout << '[' << name << "]\n";
    std::cout << "  " << std::left << std::setw(W) << "acc_x"  << ": " << msg.acc_x()  << "\n"
              << "  " << std::left << std::setw(W) << "acc_y"  << ": " << msg.acc_y()  << "\n"
              << "  " << std::left << std::setw(W) << "acc_z"  << ": " << msg.acc_z()  << "\n"
              << "  " << std::left << std::setw(W) << "gyr_x"  << ": " << msg.gyr_x()  << "\n"
              << "  " << std::left << std::setw(W) << "gyr_y"  << ": " << msg.gyr_y()  << "\n"
              << "  " << std::left << std::setw(W) << "gyr_z"  << ": " << msg.gyr_z()  << "\n"
              << "  " << std::left << std::setw(W) << "quat_w" << ": " << msg.quat_w() << "\n"
              << "  " << std::left << std::setw(W) << "quat_x" << ": " << msg.quat_x() << "\n"
              << "  " << std::left << std::setw(W) << "quat_y" << ": " << msg.quat_y() << "\n"
              << "  " << std::left << std::setw(W) << "quat_z" << ": " << msg.quat_z() << "\n"
              << "  " << std::left << std::setw(W) << "timestamp" << ": " << msg.timestamp() << "\n";
    std::cout << std::endl;
}
