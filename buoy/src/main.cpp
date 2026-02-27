
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
#include "IMUProtoSender.h"

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
void senderLoop(IMUProtoSender &sender)
{
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

        sender.sendIMU(data);
    }
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

    // create sender instance and give it to the thread
    IMUProtoSender sender("192.168.1.9", 5000);
    std::thread networkThread(senderLoop, std::ref(sender));

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