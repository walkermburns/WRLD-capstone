
#include "BNO055.h"
#include "BMI323Driver.h"
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
#include "Config.h"
#include <condition_variable>
#include <pthread.h>

static std::atomic<bool> running{true};
static std::queue<IMUData> imuQueue;
static std::mutex queueMutex;
static std::condition_variable dataCv;

// =============================
// Sensor Thread (100 Hz)
// =============================
void sensorLoop(IMUInterface &imu)
{
    pthread_setname_np(pthread_self(), "sensor");
    std::cout << "[sensor] started\n";
    int loopCount = 0;

    while (running)
    {
        IMUData data = imu.readSensor();

        // 10Hz validation output while operating at 100Hz
        if ((++loopCount % 10) == 0) {
            std::cout << "[sensor] accel (g): "
                      << data.accel.x << ", " << data.accel.y << ", " << data.accel.z
                      << " | gyro (dps): " << data.gyro.x << ", " << data.gyro.y << ", " << data.gyro.z
                      << " | quat: " << data.quat.w << ", " << data.quat.x << ", " << data.quat.y << ", " << data.quat.z
                      << "\n";
        }

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            imuQueue.push(data);
        }
        dataCv.notify_one();

        usleep(10000); // 100 Hz
    }

    std::cout << "[sensor] exiting\n";
}

// =============================
// Sender Thread (UDP)
// =============================
void senderLoop(IMUProtoSender &sender)
{
    pthread_setname_np(pthread_self(), "sender");
    std::cout << "[sender] started\n";
    while (running)
    {
        IMUData data;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            dataCv.wait(lock, []{
                return !imuQueue.empty() || !running;
            });

            if (!running && imuQueue.empty())
                break;

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

    // read configuration describing this target
    TargetConfig cfg;
    if (!loadTargetConfig("../src/configs/targets.yaml", cfg)) {
        std::cerr << "Failed to load target configuration\n";
        return -1;
    }

    std::cout << "[main] loaded config: baseIp=" << cfg.baseIp
              << " buoyIp=" << cfg.buoyIp
              << " imuPort=" << cfg.imuPort
              << " videoPort=" << cfg.videoPort << "\n";

    // create concrete sensor implementation; keep old driver available, now using BMI323
    // BNO055Driver imu;
    BMI323Driver imu;

    if (!imu.init()) {
        return -1;
    }

    // pass by reference to avoid slicing
    std::cout << "[main] starting sensor thread\n";
    std::thread sensorThread(sensorLoop, std::ref(imu));

    // create sender instance based on config and give it to the thread
    IMUProtoSender sender(cfg.baseIp.c_str(), cfg.imuPort);
    std::thread networkThread(senderLoop, std::ref(sender));

    // simultaneously start the video streamer; uses an independent port
    VideoStreamer video(cfg.baseIp, cfg.videoPort);
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

    // shutdown video immediately; other threads may still be finishing
    video.stop();
    std::cout << "[main] video stopped\n";

    sensorThread.join();
    if (networkThread.joinable()) {
        networkThread.join();
    }
    std::cout << "[main] imu & sender threads joined\n";

    google::protobuf::ShutdownProtobufLibrary();
}