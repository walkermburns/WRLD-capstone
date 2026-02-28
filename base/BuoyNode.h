#pragma once

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include "buoy.pb.h"

// A receiver object that binds a UDP socket to the given port and listens for
// incoming IMU protobuf messages from a single buoy node.  Each instance owns
// its own thread and socket; when a new message arrives the provided callback
// is invoked on the internal thread.

class BuoyNode {
public:
    using Callback = std::function<void(const std::string &name,
                                        const buoy_proto::IMU_proto &msg)>;

    BuoyNode(std::string name, int imuPort, Callback cb);
    ~BuoyNode();

    // helper that prints a message in the preferred multiline, aligned
    // format.  A mutex is held internally so multiple nodes may safely call
    // this concurrently.
    static void printMessage(const std::string &name,
                             const buoy_proto::IMU_proto &msg);

    // start the internal receive thread and bind socket.  Returns false on
    // failure (socket creation/bind error).
    bool start();

    // request termination and wait for thread to finish.  Safe to call even if
    // start() failed or was never called.
    void stop();

private:
    // mutex used by printMessage
    static std::mutex printMutex;

private:
    void receiveLoop();

    std::string name_;
    int imuPort_;
    int sock_;
    Callback callback_;
    std::thread thread_;
    std::atomic<bool> running_;
};
