
// #include <iostream>
// #include <string>
// #include <random>
// #include <chrono>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include <unistd.h>

// int main() {
    
//     IMU imu;
    
//     try {
//         while (true) {
//             imu.read_sensor();
//             usleep(200000);
//         }
//     } catch (const std::exception& e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//     }
// }

#include "BNO055.h"
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <chrono>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "buoy.pb.h"

static std::atomic<bool> running{true};
static std::queue<IMUData_t> imuQueue;
static std::mutex queueMutex;

// =============================
// Sensor Thread (100 Hz)
// =============================
void sensorLoop(IMU imu)
{
    while (running)
    {
        IMUData_t data = imu.read_sensor();

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            imuQueue.push(data);
        }

        usleep(10000); // 100 Hz
    }
}

// =============================
// Sender Thread (UDP)
// =============================
void senderLoop(const char* ip, int port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &addr.sin_addr);

    while (running)
    {
        IMUData_t data;

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (imuQueue.empty())
                continue;

            data = imuQueue.front();
            imuQueue.pop();
        }

        buoy_proto::IMU_proto msg;

        msg.set_acc_x(data.ax);
        msg.set_acc_y(data.ay);
        msg.set_acc_z(data.az);
        msg.set_gyr_x(data.gx);
        msg.set_gyr_y(data.gy);
        msg.set_gyr_z(data.gz);
        msg.set_quat_w(data.qw);
        msg.set_quat_x(data.qx);
        msg.set_quat_y(data.qy);
        msg.set_quat_z(data.qz);

        uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        msg.set_timestamp(ts);

        std::string buffer;
        msg.SerializeToString(&buffer);

        sendto(sock,
               buffer.data(),
               buffer.size(),
               0,
               (sockaddr*)&addr,
               sizeof(addr));
    }

    close(sock);
}

// =============================
// Main
// =============================
int main()
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    IMU imu;

    if (!imu.init()) {
        return -1;
    }

    std::thread sensorThread(sensorLoop, imu);
    std::thread networkThread(senderLoop, "192.168.1.9", 5000);

    std::this_thread::sleep_for(std::chrono::seconds(60));
    running = false;

    sensorThread.join();
    networkThread.join();

    google::protobuf::ShutdownProtobufLibrary();
}