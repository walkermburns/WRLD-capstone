#include <iostream>
#include <vector>
#include <memory>
#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>

#include "Config.h"
#include "BuoyNode.h"
#include "buoy.pb.h"
#include "VideoComposite.h"
#include <filesystem>   // existence check for shader path

static std::atomic<bool> running{true};

int main()
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    TargetsConfig cfg;
    if (!loadTargetsConfig("../../targets.yaml", cfg)) {
        std::cerr << "failed to load targets configuration\n";
        return -1;
    }

    std::cout << "[main] base IP=" << cfg.global.baseLanIp << "\n";

    // prepare shader and video port list early so we can construct the
    // VideoComposite before wiring up the IMU callbacks.
    std::string shaderPath = "../src/warp.frag";
    if (!std::filesystem::exists(shaderPath)) {
        std::cerr << "[main] shader path does not exist: '" << shaderPath
                  << "' (cwd=" << std::filesystem::current_path() << ")\n";
    }

    std::vector<int> videoPorts;
    for (auto &n : cfg.targets) {
        if (n.videoPort != 0) {
            videoPorts.push_back(n.videoPort);
        } else {
            std::cerr << "[main] warning: node '" << n.name
                      << "' has no videoPort, skipping" << "\n";
        }
    }

    // construct composite prior to node creation so callback lambdas can
    // capture it by reference.
    VideoComposite vc(shaderPath, videoPorts);

    std::vector<std::unique_ptr<BuoyNode>> nodes;
    for (auto &n : cfg.targets) {
        if (n.imuPort == 0) {
            std::cerr << "[main] warning: node '" << n.name << "' has no imu_port\n";
            continue;
        }

        // callback updates the composite's quaternion state
        BuoyNode::Callback cb = [&vc](const std::string & /*name*/, const buoy_proto::IMU_proto &msg) {
            vc.updateQuaternion(msg);
        };

        auto node = std::make_unique<BuoyNode>(n.name, n.imuPort, cb);
        if (node->start()) {
            std::cout << "[main] listening for " << n.name << " on port "
                      << n.imuPort << "\n";
            nodes.emplace_back(std::move(node));
        } else {
            std::cerr << "[main] failed to start node " << n.name << "\n";
        }
    }

    try {
        vc.start();
    } catch (const std::exception &e) {
        std::cerr << "VideoComposite error: " << e.what() << "\n";
        return -1;
    }

    // signal handling to allow clean shutdown.  the handler is intentionally
    // simple/async-signal-safe: it just flips an atomic flag.  the main loop
    // notices the change, then tears the nodes down in its own thread context
    // where it's safe to close sockets, join threads, etc.  the operating
    // system will also automatically close any file descriptors if the process
    // is killed, so there's no risk of port leakage even on crashes.
    auto sigHandler = [](int){ running = false; };
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);
    std::signal(SIGABRT, sigHandler); // in case of abort() the flag is useful
    std::signal(SIGHUP,  sigHandler); // hangup (e.g. terminal close)

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // tear down
    for (auto &node : nodes)
        node->stop();

    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}
