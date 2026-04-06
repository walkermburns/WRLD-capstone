#include "BuoyNode.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <iomanip>   // for setw, left
#include <unordered_map>
#include <chrono>
#include <cerrno>
#include <algorithm>
#include <cmath>
#include <cstdlib>

using namespace MathHelpers; // simplify quaternion/matrix calls

namespace {

// normalize quaternion to unit length; falls back to identity if degenerate
static Quaternion normalize_quat(const Quaternion &q)
{
    float n2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (n2 <= 1e-12f) {
        return Quaternion{};
    }
    float invn = 1.0f / std::sqrt(n2);
    Quaternion r;
    r.w = q.w * invn;
    r.x = q.x * invn;
    r.y = q.y * invn;
    r.z = q.z * invn;
    return r;
}

// quaternion from rotation vector (axis * angle), where components are radians
static Quaternion rotvec_to_quat(float rx, float ry, float rz)
{
    float angle = std::sqrt(rx*rx + ry*ry + rz*rz);
    Quaternion q;
    if (angle < 1e-9f) {
        q.w = 1.0f;
        q.x = 0.5f * rx;
        q.y = 0.5f * ry;
        q.z = 0.5f * rz;
        return normalize_quat(q);
    }

    float half = 0.5f * angle;
    float s = std::sin(half) / angle;
    q.w = std::cos(half);
    q.x = rx * s;
    q.y = ry * s;
    q.z = rz * s;
    return normalize_quat(q);
}

// convert a small rotation quaternion to rotation-vector form
static void quat_to_rotvec(const Quaternion &qin, float &rx, float &ry, float &rz)
{
    Quaternion q = normalize_quat(qin);

    // choose shortest-path representation
    if (q.w < 0.0f) {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    float vnorm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
    if (vnorm < 1e-9f) {
        rx = ry = rz = 0.0f;
        return;
    }

    float angle = 2.0f * std::atan2(vnorm, q.w);
    float scale = angle / vnorm;
    rx = q.x * scale;
    ry = q.y * scale;
    rz = q.z * scale;
}

} // namespace

static float rad_to_deg(float radians)
{
    return radians * 180.0f / static_cast<float>(M_PI);
}

// static member definitions
std::mutex BuoyNode::printMutex;

BuoyNode::BuoyNode(std::string name, int imuPort, Callback cb)
    : name_(std::move(name)), imuPort_(imuPort), sock_(-1), callback_(std::move(cb)),
      running_(false), lastQuat_()
{
    initCameraMatrices();
    initAnchor();
    initDebug();
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

    // optionally make the socket non-blocking so that a stray recv can't hang
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

void BuoyNode::initAnchor()
{
    // default to image center unless env variables override it
    anchor_x_ = 0.5f * cam_w;
    anchor_y_ = 0.5f * cam_h;

    const char *ax = getenv("IMU_STAB_ANCHOR_X");
    const char *ay = getenv("IMU_STAB_ANCHOR_Y");
    if (ax && *ax) {
        anchor_x_ = std::strtof(ax, nullptr);
    }
    if (ay && *ay) {
        anchor_y_ = std::strtof(ay, nullptr);
    }
}

void BuoyNode::initDebug()
{
    const char *path = getenv("IMU_STAB_CSV");
    if (!path || path[0] == '\0')
        return;

    debugCsvFile_.open(path, std::ios::out | std::ios::trunc);
    if (!debugCsvFile_.is_open()) {
        std::cerr << "[" << name_ << "] failed to open IMU stabilization CSV '" << path << "'\n";
        return;
    }

    debugCsvEnabled_ = true;
    debugCsvFile_ << "timestamp_us,yaw_deg,pitch_deg,roll_deg,pitch_cmd_deg,roll_cmd_deg,corr_pitch_filt_deg,corr_roll_filt_deg,anchor_x,anchor_y,ok\n";
    debugCsvFile_.flush();
    std::cerr << "[" << name_ << "] IMU stabilization CSV logging to " << path << "\n";
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

            const uint64_t ts = msg.timestamp();

            lastQuat_.w = msg.quat_w();
            lastQuat_.x = msg.quat_x();
            lastQuat_.y = msg.quat_y();
            lastQuat_.z = msg.quat_z();
            lastQuat_ = normalize_quat(lastQuat_);

            // keep a short quaternion history so we can estimate angular velocity
            // from the latest two orientation samples and predict slightly ahead
            // to compensate for display / decode latency.
            quat_hist_.emplace_back(ts, lastQuat_);
            while (!quat_hist_.empty() && ts > quat_hist_.front().first + 300000ull) {
                quat_hist_.pop_front();
            }

            // update filtered angular velocity using the newest two quaternion samples
            if (quat_hist_.size() >= 2) {
                const auto &a = quat_hist_[quat_hist_.size() - 2];
                const auto &b = quat_hist_[quat_hist_.size() - 1];

                if (b.first > a.first && b.first != last_omega_ts_) {
                    Quaternion dq = quat_mult(b.second, quat_inverse(a.second));

                    float rvx = 0.0f, rvy = 0.0f, rvz = 0.0f;
                    quat_to_rotvec(dq, rvx, rvy, rvz);

                    float dt = std::max(1e-6f, float(b.first - a.first) * 1e-6f);

                    float ox = std::clamp(rvx / dt, -omega_clamp_, omega_clamp_);
                    float oy = std::clamp(rvy / dt, -omega_clamp_, omega_clamp_);
                    float oz = std::clamp(rvz / dt, -omega_clamp_, omega_clamp_);

                    omega_filt_[0] = (1.0f - omega_alpha_) * omega_filt_[0] + omega_alpha_ * ox;
                    omega_filt_[1] = (1.0f - omega_alpha_) * omega_filt_[1] + omega_alpha_ * oy;
                    omega_filt_[2] = (1.0f - omega_alpha_) * omega_filt_[2] + omega_alpha_ * oz;

                    last_omega_ts_ = b.first;
                }
            }

            // predict forward a small amount to offset end-to-end latency
            Quaternion q_pred = lastQuat_;
            {
                float dt_pred = std::clamp(predict_lead_s_, 0.0f, predict_max_s_);
                Quaternion dq_pred = rotvec_to_quat(omega_filt_[0] * dt_pred,
                                                    omega_filt_[1] * dt_pred,
                                                    omega_filt_[2] * dt_pred);
                q_pred = quat_mult(dq_pred, lastQuat_);
                q_pred = normalize_quat(q_pred);
            }

            float Hinv[9];
            StabilizationDebug debug;
            bool ok = compute_homography_from_quat(q_pred, quat_ref, have_ref,
                                                   corr_smooth_alpha,
                                                   corr_pitch_filt, corr_roll_filt,
                                                   Rflu2cv_mat, K, Kinv,
                                                   cam_w, cam_h, Hinv,
                                                   &debug);
            if (ok && (anchor_x_ != 0.0f || anchor_y_ != 0.0f)) {
                float anchoredHinv[9];
                anchor_homography(Hinv, anchor_x_, anchor_y_, anchoredHinv);
                for (int i = 0; i < 9; ++i)
                    Hinv[i] = anchoredHinv[i];
            }
            if (debugCsvEnabled_) {
                debugCsvFile_ << ts << ","
                              << rad_to_deg(debug.yaw) << ","
                              << rad_to_deg(debug.pitch) << ","
                              << rad_to_deg(debug.roll) << ","
                              << rad_to_deg(debug.pitch_cmd) << ","
                              << rad_to_deg(debug.roll_cmd) << ","
                              << rad_to_deg(debug.pitch_filt) << ","
                              << rad_to_deg(debug.roll_filt) << ","
                              << anchor_x_ << ","
                              << anchor_y_ << ","
                              << (ok ? 1 : 0) << "\n";
                debugCsvFile_.flush();
            }
            if (ok) {
                for (int i = 0; i < 9; ++i)
                    lastHinv_[i] = Hinv[i];

                // record timestamped matrix; keep only past ~300ms of samples
                hist_.emplace_back(ts, std::array<float,9>{});
                for (int i = 0; i < 9; ++i)
                    hist_.back().second[i] = Hinv[i];

                // drop old entries (older than 300000us)
                while (!hist_.empty() && ts > hist_.front().first + 300000ull) {
                    hist_.pop_front();
                }
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

// bool BuoyNode::getHinvAt(uint64_t timestamp, std::array<float,9> &out) const {
//     std::lock_guard<std::mutex> lock(quatMutex_);
//     if (hist_.empty())
//         return false;
//     uint64_t bestDiff = UINT64_MAX;
//     bool found = false;
//     for (const auto &p : hist_) {
//         uint64_t diff = (p.first > timestamp) ? p.first - timestamp : timestamp - p.first;
//         if (diff < bestDiff) {
//             bestDiff = diff;
//             out = p.second;
//             found = true;
//         }
//     }
//     return found;
// }

bool BuoyNode::getHinvAt(uint64_t timestamp, std::array<float,9> &out,
                         uint64_t *best_diff_us) const {
    std::lock_guard<std::mutex> lock(quatMutex_);
    if (hist_.empty())
        return false;

    uint64_t bestDiff = UINT64_MAX;
    bool found = false;

    for (const auto &p : hist_) {
        uint64_t diff = (p.first > timestamp) ? (p.first - timestamp)
                                              : (timestamp - p.first);
        if (diff < bestDiff) {
            bestDiff = diff;
            out = p.second;
            found = true;
        }
    }

    if (best_diff_us)
        *best_diff_us = bestDiff;

    return found;
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