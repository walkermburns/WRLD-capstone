
#include "BNO055.h"
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <chrono>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <csignal>
#include "buoy.pb.h"

static std::atomic<bool> running{true};
static std::queue<IMUData> imuQueue;
static std::mutex queueMutex;

// =============================
// Sensor Thread (100 Hz)
// =============================
void sensorLoop(IMUInterface &imu)
{
    while (running)
    {
        IMUData data = imu.readSensor();

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
        IMUData data;

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (imuQueue.empty())
                continue;

            data = imuQueue.front();
            imuQueue.pop();
        }

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

    // create concrete sensor implementation; defaults match original
    BNO055Driver imu;

    if (!imu.init()) {
        return -1;
    }

    // pass by reference to avoid slicing
    std::thread sensorThread(sensorLoop, std::ref(imu));
    std::thread networkThread(senderLoop, "192.168.1.9", 5000);

    // instead of sleeping a fixed time, stay alive until a signal
    // requests shutdown.  signal handler simply clears the atomic.
    std::signal(SIGINT, [](int){ running = false; });
    std::signal(SIGTERM, [](int){ running = false; });

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    sensorThread.join();
    networkThread.join();

    google::protobuf::ShutdownProtobufLibrary();
}