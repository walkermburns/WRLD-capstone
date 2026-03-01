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

    std::vector<std::unique_ptr<BuoyNode>> nodes;
    for (auto &n : cfg.targets) {
        if (n.imuPort == 0) {
            std::cerr << "[main] warning: node '" << n.name << "' has no imu_port\n";
            continue;
        }
        auto node = std::make_unique<BuoyNode>(n.name, n.imuPort);
        if (node->start()) {
            std::cout << "[main] listening for " << n.name << " on port "
                      << n.imuPort << "\n";
            nodes.emplace_back(std::move(node));
        } else {
            std::cerr << "[main] failed to start node " << n.name << "\n";
        }
    }

    try {
        std::string shaderPath = "../distort.frag";
        VideoComposite vc(shaderPath);
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
