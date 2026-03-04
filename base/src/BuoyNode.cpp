#include "BuoyNode.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>   // for setw, left
#include <unordered_map>
#include <chrono>

using namespace MathHelpers; // simplify quaternion/matrix calls

// static member definitions
std::mutex BuoyNode::printMutex;

BuoyNode::BuoyNode(std::string name, int imuPort, Callback cb)
    : name_(std::move(name)), imuPort_(imuPort), sock_(-1), callback_(std::move(cb)),
      running_(false), lastQuat_()
{
    initCameraMatrices();
}

BuoyNode::BuoyNode(std::string name, int imuPort)
    : BuoyNode(std::move(name), imuPort, BuoyNode::printMessage)
{
}

BuoyNode::~BuoyNode()
{
    // ensure we leave the socket in a clean state; stop() is idempotent and
    // safe to call even if start() failed or was never invoked.  the member
    // variable will be reset and the internal thread (if any) joined before
    // destruction.
    stop();
}

bool BuoyNode::start()
{
    if (running_)
        return true; // already started

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        perror("socket");
        return false;
    }

    // make the address/port reusable immediately after the socket is closed
    // (even if the previous owner crashed or is stuck in TIME_WAIT).
    // SO_REUSEADDR is generally sufficient for UDP; on some platforms
    // SO_REUSEPORT allows multiple listeners on the same port if desired.
    int opt = 1;
    if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt(SO_REUSEADDR)");
        // non-fatal: continue without reuse
    }
#ifdef SO_REUSEPORT
    if (setsockopt(sock_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
        perror("setsockopt(SO_REUSEPORT)");
        // not fatal; not all systems support it
    }
#endif

    // optionally make the socket non‑blocking so that a stray recv can't hang
    // the thread; the shutdown logic already makes recv return with error
    // so this is just defensive.  if non-blocking is used the receiveLoop
    // would need to handle EAGAIN/EWOULDBLOCK (currently it does not).
    // fcntl(sock_, F_SETFL, O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(imuPort_);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock_);
        sock_ = -1;
        return false;
    }

    running_ = true;
    thread_ = std::thread(&BuoyNode::receiveLoop, this);
    return true;
}

void BuoyNode::stop()
{
    // flip flag first so receiveLoop knows it's time to exit
    if (running_) {
        running_ = false;
    }

    if (sock_ >= 0) {
        // shutdown the descriptor before closing; this will unblock a
        // blocking recv() call on another thread, causing it to return
        // -1 with errno=EBADF/ENOTCONN etc.
        shutdown(sock_, SHUT_RD);
        close(sock_);
        sock_ = -1;
    }

    if (thread_.joinable())
        thread_.join();
}

void BuoyNode::receiveLoop()
{
    char buffer[1500];

    while (running_) {
        ssize_t len = recv(sock_, buffer, sizeof(buffer), 0);
        if (len <= 0) {
            if (len < 0) {
                // if the socket was closed due to stop(), errno may be
                // EBADF or EINVAL; treat that as a normal shutdown.
                if (errno != EINTR && errno != EBADF && errno != EINVAL) {
                    perror("recv");
                }
            }
            break; // exit loop when socket is closed or fatal error
        }

        buoy_proto::IMU_proto msg;
        if (!msg.ParseFromArray(buffer, len)) {
            std::cerr << "[" << name_ << "] failed to parse protobuf (size=" << len << ")\n";
            continue;
        }

        // compute quaternion and homography together under lock using helper
        {
            std::lock_guard<std::mutex> lock(quatMutex_);
            lastQuat_.w = msg.quat_w();
            lastQuat_.x = msg.quat_x();
            lastQuat_.y = msg.quat_y();
            lastQuat_.z = msg.quat_z();

            float Hinv[9];
            bool ok = compute_homography_from_quat(lastQuat_, quat_ref, have_ref,
                                                   corr_smooth_alpha,
                                                   corr_pitch_filt, corr_roll_filt,
                                                   Rflu2cv_mat, K, Kinv,
                                                   cam_w, cam_h, Hinv);
            if (ok) {
                for (int i = 0; i < 9; ++i)
                    lastHinv_[i] = Hinv[i];
            }
        }

        if (callback_) {
            callback_(name_, msg);
        }
    }
}

// helper ---------------------------------------------------------------

void BuoyNode::initCameraMatrices() {
    // delegate to MathHelpers helper for consistency with VideoComposite
    init_camera_matrices(cam_w, cam_h, cam_hfov_deg, K, Kinv,
                         Rflu2cv_mat, last_good_Hinv);
    // also set our public storage to identity
    for (int i = 0; i < 9; ++i)
        lastHinv_[i] = (i % 4 == 0) ? 1.0f : 0.0f;
}

// accessor ---------------------------------------------------------------

MathHelpers::Quaternion BuoyNode::getQuaternion() const {
    std::lock_guard<std::mutex> lock(quatMutex_);
    return lastQuat_;
}

std::array<float,9> BuoyNode::getHinv() const {
    std::array<float,9> out;
    std::lock_guard<std::mutex> lock(quatMutex_);
    for (int i = 0; i < 9; ++i)
        out[i] = lastHinv_[i];
    return out;
}

// static helper -------------------------------------------------------------

void BuoyNode::printMessage(const std::string &name,
                            const buoy_proto::IMU_proto &msg)
{
    // throttle to at most one print per-node every 500ms.  the same mutex that
    // protects the iostream ensures the map is safe too.
    std::lock_guard<std::mutex> lock(printMutex);

    using Clock = std::chrono::steady_clock;
    static std::unordered_map<std::string, Clock::time_point> lastPrint;

    auto now = Clock::now();
    auto it = lastPrint.find(name);
    if (it != lastPrint.end() && now - it->second < std::chrono::milliseconds(500)) {
        return; // skip this message
    }
    lastPrint[name] = now;

    const int W = 8;
    std::cout << '[' << name << "]\n";
    std::cout << "  " << std::left << std::setw(W) << "acc_x"  << ": " << msg.acc_x()  << "\n"
              << "  " << std::left << std::setw(W) << "acc_y"  << ": " << msg.acc_y()  << "\n"
              << "  " << std::left << std::setw(W) << "acc_z"  << ": " << msg.acc_z()  << "\n"
              << "  " << std::left << std::setw(W) << "gyr_x"  << ": " << msg.gyr_x()  << "\n"
              << "  " << std::left << std::setw(W) << "gyr_y"  << ": " << msg.gyr_y()  << "\n"
              << "  " << std::left << std::setw(W) << "gyr_z"  << ": " << msg.gyr_z()  << "\n"
              << "  " << std::left << std::setw(W) << "quat_w" << ": " << msg.quat_w() << "\n"
              << "  " << std::left << std::setw(W) << "quat_x" << ": " << msg.quat_x() << "\n"
              << "  " << std::left << std::setw(W) << "quat_y" << ": " << msg.quat_y() << "\n"
              << "  " << std::left << std::setw(W) << "quat_z" << ": " << msg.quat_z() << "\n"
              << "  " << std::left << std::setw(W) << "timestamp" << ": " << msg.timestamp() << "\n";
    std::cout << std::endl;
}
