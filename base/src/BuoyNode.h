#pragma once

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include "buoy.pb.h"
#include "MathHelpers.h"  // for Quaternion used in getter/storage

// A receiver object that binds a UDP socket to the given port and listens for
// incoming IMU protobuf messages from a single buoy node.  Each instance owns
// its own thread and socket; when a new message arrives the provided callback
// is invoked on the internal thread.  In addition to delivering callbacks the
// node now computes a 3x3 homography matrix (h) from each message and stores
// it for external consumers.

class BuoyNode {
public:
    using Callback = std::function<void(const std::string &name,
                                        const buoy_proto::IMU_proto &msg)>;

    // normal constructor takes a callback; for common use cases the
    // instance will simply print received messages with optional throttling.
    // this overload users the static printMessage helper directly so callers
    // can omit the callback entirely.
    BuoyNode(std::string name, int imuPort, Callback cb);
    BuoyNode(std::string name, int imuPort);
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

    // return the most recently received orientation quaternion.  if no
    // message has been received yet the default identity quaternion is
    // returned.  thread-safe; callers may invoke from any thread while
    // the internal receive thread is running.
    MathHelpers::Quaternion getQuaternion() const;

    // return the latest 3x3 inverse homography matrix computed from the
    // last IMU message.  entries are returned in row-major order.  if no
    // message has arrived yet the identity matrix is returned.  protected by
    // the same mutex as the quaternion so simultaneous access is safe.
    std::array<float,9> getHinv() const;

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

    // last orientation from the IMU, updated inside receiveLoop
    mutable std::mutex quatMutex_;
    MathHelpers::Quaternion lastQuat_;

    // homography state generated from the quaternion (see receiveLoop)
    std::array<float,9> lastHinv_;

    // camera/intrinsic parameters used for homography calculation
    float cam_w = 1920.0f;
    float cam_h = 1080.0f;
    float cam_hfov_deg = 50.0f;
    float K[9];
    float Kinv[9];
    float Rflu2cv_mat[9];
    float last_good_Hinv[9];
    bool have_ref = false;
    MathHelpers::Quaternion quat_ref;

    // smoothing state (same semantics as earlier VideoComposite code)
    float corr_smooth_alpha = 1.0f;
    float corr_pitch_filt = 0.0f;
    float corr_roll_filt = 0.0f;

    // helper to initialize camera matrices; called from constructors
    void initCameraMatrices();
};
