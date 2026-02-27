
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
#include "VideoStreamer.h"

static std::atomic<bool> running{true};
static std::queue<IMUData> imuQueue;
static std::mutex queueMutex;

// =============================
// Sensor Thread (100 Hz)
// =============================
void sensorLoop(IMUInterface &imu)
{
    std::cout << "[sensor] started\n";
    while (running)
    {
        IMUData data = imu.readSensor();

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            imuQueue.push(data);
        }

        usleep(10000); // 100 Hz
    }
    std::cout << "[sensor] exiting\n";
}

// =============================
// Sender Thread (UDP)
// =============================
void senderLoop(IMUProtoSender &sender)
{
    std::cout << "[sender] started\n";
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

        if (!sender.sendIMU(data))
            std::cerr << "[sender] failed to send IMU\n";
    }
    std::cout << "[sender] exiting\n";
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
    std::cout << "[main] starting sensor thread\n";
    std::thread sensorThread(sensorLoop, std::ref(imu));

    // create sender instance and give it to the thread
    IMUProtoSender sender("192.168.1.9", 5000);
    std::cout << "[main] starting sender thread\n";
    std::thread networkThread(senderLoop, std::ref(sender));

    // simultaneously start the video streamer; uses an independent port
    VideoStreamer video("192.168.1.9", 5001);
    std::cout << "[main] video streamer constructed\n";
    video.start();
    std::cout << "[main] video thread started\n";

    // instead of sleeping a fixed time, stay alive until a signal
    // requests shutdown.  signal handler simply clears the atomic.
    std::signal(SIGINT, [](int){ running = false; });
    std::signal(SIGTERM, [](int){ running = false; });

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    sensorThread.join();
    networkThread.join();
    std::cout << "[main] imu & sender threads joined\n";
    video.stop();
    std::cout << "[main] video stopped\n";

    google::protobuf::ShutdownProtobufLibrary();
}