#include <iostream>
#include <vector>
#include <memory>
#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>
#include <string>
#include <cctype>

#include "Config.h"
#include "BuoyNode.h"
#include "buoy.pb.h"
#include "VideoComposite.h"
#include <filesystem>   // existence check for shader path

static std::atomic<bool> running{true};

static std::string lowerCopy(std::string s)
{
    for (char &c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

static VideoComposite::CameraMount parseCameraMount(const std::string &mount)
{
    const std::string m = lowerCopy(mount);
    if (m == "rear" || m == "rear_yaw_180" || m == "rearyaw180" ||
        m == "back" || m == "backward" || m == "reverse") {
        return VideoComposite::CameraMount::RearYaw180;
    }
    return VideoComposite::CameraMount::Front;
}

int main()
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    TargetsConfig cfg;
    if (!loadTargetsConfig("../../targets.yaml", cfg)) {
        std::cerr << "failed to load targets configuration\n";
        return -1;
    }

    std::cout << "[main] base IP=" << cfg.global.baseLanIp << "\n";

    std::string shaderPath = "../src/warp.frag";
    if (!std::filesystem::exists(shaderPath)) {
        std::cerr << "[main] shader path does not exist: '" << shaderPath
                  << "' (cwd=" << std::filesystem::current_path() << ")\n";
    }

    std::vector<std::unique_ptr<BuoyNode>> nodes;
    std::vector<VideoComposite::CameraFeedConfig> feeds;

    for (const auto &n : cfg.targets) {
        if (n.imuPort == 0) {
            std::cerr << "[main] warning: node '" << n.name << "' has no imu_port; "
                      << "its cameras will be skipped\n";
            continue;
        }

        auto node = std::make_unique<BuoyNode>(n.name, n.imuPort);
        if (!node->start()) {
            std::cerr << "[main] failed to start node " << n.name << "\n";
            continue;
        }

        const int imu_node_index = static_cast<int>(nodes.size());
        std::cout << "[main] listening for " << n.name << " IMU on UDP "
                  << n.imuPort << " as node index " << imu_node_index << "\n";
        nodes.emplace_back(std::move(node));

        if (n.cameras.empty()) {
            std::cerr << "[main] warning: node '" << n.name
                      << "' has no cameras/video_port, skipping video feeds\n";
            continue;
        }

        int camera_count_for_node = 0;
        for (const auto &cam : n.cameras) {
            if (cam.videoPort == 0) {
                std::cerr << "[main] warning: camera '" << cam.name
                          << "' on node '" << n.name
                          << "' has no video_port, skipping\n";
                continue;
            }

            VideoComposite::CameraFeedConfig feed;
            feed.name = n.name + "_" + cam.name;
            feed.video_port = cam.videoPort;
            feed.metadata_port = (cam.metadataPort != 0) ? cam.metadataPort : cam.videoPort + 1;
            feed.imu_node_index = imu_node_index;
            feed.mount = parseCameraMount(cam.mount);
            feed.layout.xpos = cam.x;
            feed.layout.ypos = cam.y;
            feed.layout.width = cam.width;
            feed.layout.height = cam.height;

            std::cout << "[main] camera feed '" << feed.name
                      << "': video UDP " << feed.video_port
                      << ", metadata UDP " << feed.metadata_port
                      << ", imu node " << feed.imu_node_index
                      << ", mount " << cam.mount
                      << ", layout {" << feed.layout.xpos << ", "
                      << feed.layout.ypos << ", "
                      << feed.layout.width << ", "
                      << feed.layout.height << "}\n";

            feeds.push_back(feed);
            ++camera_count_for_node;
        }

        if (camera_count_for_node == 0) {
            std::cerr << "[main] warning: node '" << n.name
                      << "' started IMU but no valid camera feeds were configured\n";
        }
    }

    if (feeds.empty()) {
        std::cerr << "[main] no valid camera feeds configured\n";
        for (auto &node : nodes)
            node->stop();
        google::protobuf::ShutdownProtobufLibrary();
        return -1;
    }

    VideoComposite vc(shaderPath, feeds);
    vc.setBuoyNodes(&nodes);

    auto sigHandler = [](int){ running = false; };
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);
    std::signal(SIGABRT, sigHandler);
    std::signal(SIGHUP,  sigHandler);

    // Keep GStreamer/AppKit initialization on the main thread. On macOS,
    // gst_macos_main must not run from a worker thread.
    std::thread stop_watcher([&]() {
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        vc.stop();
    });

    try {
        vc.start();
    } catch (const std::exception &e) {
        std::cerr << "VideoComposite error: " << e.what() << "\n";
        running = false;
    }

    running = false;
    if (stop_watcher.joinable())
        stop_watcher.join();

    for (auto &node : nodes)
        node->stop();

    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}
