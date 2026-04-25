
#include "BNO055.h"
#include "BMI323Driver.h"
#include "IMUHelpers.h"
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <chrono>
#include <fstream>
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
static std::ofstream csvFile;
static bool enableCsvLogging = false;  // Set to false to disable CSV logging

// =============================
// Sensor Thread (100 Hz)
// =============================
void sensorLoop(IMUInterface &imu)
{
    pthread_setname_np(pthread_self(), "sensor");
    std::cout << "[sensor] started\n";
    int loopCount = 0;
    auto startTime = std::chrono::high_resolution_clock::now();

    while (running)
    {
        IMUData data = imu.readSensor();

        // Calculate elapsed time in milliseconds
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();

        // Log to CSV at every sample (100 Hz)
        if (enableCsvLogging && csvFile.is_open()) {
            float roll, pitch, yaw;
            IMUHelpers::quatToEulerZYX(data.quat, roll, pitch, yaw);

            csvFile << elapsedMs << ","
                    << data.accel.x << "," << data.accel.y << "," << data.accel.z << ","
                    << data.gyro.x << "," << data.gyro.y << "," << data.gyro.z << ","
                    << IMUHelpers::radToDeg(roll) << "," << IMUHelpers::radToDeg(pitch) << "," << IMUHelpers::radToDeg(yaw) << ","
                    << data.quat.w << "," << data.quat.x << "," << data.quat.y << "," << data.quat.z << "\n";
        }

        // 10Hz validation output while operating at 100Hz
        if ((++loopCount % 10) == 0) {
            float roll, pitch, yaw;
            IMUHelpers::quatToEulerZYX(data.quat, roll, pitch, yaw);

            std::cout << "[sensor] accel (m/s2): "
                      << data.accel.x << ", " << data.accel.y << ", " << data.accel.z
                      << " | gyro (dps): " << data.gyro.x << ", " << data.gyro.y << ", " << data.gyro.z
                      << " | euler (deg): " << IMUHelpers::radToDeg(roll) << ", " << IMUHelpers::radToDeg(pitch) << ", " << IMUHelpers::radToDeg(yaw)
                      << " | quat: " << data.quat.w << ", " << data.quat.x << ", " << data.quat.y << ", " << data.quat.z
                      << "\n";
        }

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            imuQueue.push(data);
        }
        dataCv.notify_all();

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
              << " videoPort=" << cfg.videoPort
              << " video2Port=" << cfg.video2Port
              << " camera_name=" << cfg.cameraName
              << " camera2_name=" << cfg.camera2Name << "\n";

    // create concrete sensor implementation; keep old driver available, now using BMI323
    // BNO055Driver imu;
    BMI323Driver imu;

    if (!imu.init()) {
        return -1;
    }

    // Open CSV log file if enabled
    if (enableCsvLogging) {
        csvFile.open("imu_log.csv");
        if (csvFile.is_open()) {
            csvFile << "time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll_deg,pitch_deg,yaw_deg,quat_w,quat_x,quat_y,quat_z\n";
            csvFile.flush();
            std::cout << "[main] CSV logging to imu_log.csv\n";
        } else {
            std::cerr << "[main] failed to open imu_log.csv for writing\n";
        }
    }

    // pass by reference to avoid slicing
    std::cout << "[main] starting sensor thread\n";
    std::thread sensorThread(sensorLoop, std::ref(imu));

    // create sender instance based on config and give it to the thread
    IMUProtoSender sender(cfg.baseIp.c_str(), cfg.imuPort);
    std::thread networkThread(senderLoop, std::ref(sender));

    // simultaneously start the video streamer; uses an independent port
    VideoStreamer video(cfg.baseIp,
                        cfg.videoPort,
                        cfg.video2Port,
                        cfg.cameraName,
                        cfg.camera2Name);
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

    // Notify all waiting threads to wake up and exit
    {
        std::lock_guard<std::mutex> lock(queueMutex);
    }
    dataCv.notify_all();

    // Give threads a brief moment to process
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    video.stop();
    std::cout << "[main] video stopped\n";

    sensorThread.join();
    if (networkThread.joinable()) {
        networkThread.join();
    }
    std::cout << "[main] imu & sender threads joined\n";

    // Close CSV file
    if (csvFile.is_open()) {
        csvFile.close();
        std::cout << "[main] CSV log closed\n";
    }

    google::protobuf::ShutdownProtobufLibrary();
}