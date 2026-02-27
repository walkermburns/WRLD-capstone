#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "buoy.pb.h"

static std::atomic<bool> running{true};
static std::mutex dataMutex;
static buoy_proto::IMU_proto latestIMU;
static std::atomic<bool> hasData{false};

// ==============================
// UDP Receive Thread
// ==============================
void receiveLoop(int port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("bind failed");
        return;
    }

    char buffer[1024];

    while (running)
    {
        ssize_t len = recv(sock, buffer, sizeof(buffer), 0);
        if (len <= 0)
            continue;

        buoy_proto::IMU_proto msg;
        if (msg.ParseFromArray(buffer, len))
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            latestIMU = msg;
            hasData = true;
        }
    }

    close(sock);
}

// ==============================
// Print Thread (2 Hz)
// ==============================
void printLoop()
{
    while (running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (!hasData)
            continue;

        buoy_proto::IMU_proto copy;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            copy = latestIMU;
        }

        std::cout << "Accel: "
                  << copy.acc_x() << " "
                  << copy.acc_y() << " "
                  << copy.acc_z() << " | Gyro: "
                  << copy.gyr_x() << " "
                  << copy.gyr_y() << " "
                  << copy.gyr_z() << " | Quat: "
                  << copy.quat_w() << " "
                  << copy.quat_x() << " "
                  << copy.quat_y() << " "
                  << copy.quat_z() << " | ts: "
                  << copy.timestamp()
                  << std::endl;
    }
}

// ==============================
// Main
// ==============================
int main()
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    std::thread recvThread(receiveLoop, 5000);
    std::thread printerThread(printLoop);

    std::cout << "IMU Receiver listening on port 5000...\n";

    std::this_thread::sleep_for(std::chrono::minutes(10));
    running = false;

    recvThread.join();
    printerThread.join();

    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}