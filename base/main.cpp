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

static std::atomic<bool> running{true};

// previously we used a custom callback+mutex; the helper in BuoyNode now
// takes care of printing in the desired format.

int main()
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    TargetsConfig cfg;
    if (!loadTargetsConfig("../../targets.yaml", cfg)) {
        std::cerr << "failed to load targets configuration\n";
        return -1;
    }

    std::cout << "[main] base IP=" << cfg.global.baseLanIp << "\n";

    // callback simply forwards to BuoyNode's static printer helper
    auto onMsg = BuoyNode::printMessage;

    std::vector<std::unique_ptr<BuoyNode>> nodes;
    for (auto &n : cfg.targets) {
        if (n.imuPort == 0) {
            std::cerr << "[main] warning: node '" << n.name << "' has no imu_port\n";
            continue;
        }
        auto node = std::make_unique<BuoyNode>(n.name, n.imuPort, onMsg);
        if (node->start()) {
            std::cout << "[main] listening for " << n.name << " on port "
                      << n.imuPort << "\n";
            nodes.emplace_back(std::move(node));
        } else {
            std::cerr << "[main] failed to start node " << n.name << "\n";
        }
    }

    // signal handling to allow clean shutdown
    std::signal(SIGINT, [](int){ running = false; });
    std::signal(SIGTERM, [](int){ running = false; });

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // tear down
    for (auto &node : nodes)
        node->stop();

    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}
