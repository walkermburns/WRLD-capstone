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

static float quat_dot_local(const Quaternion &a, const Quaternion &b)
{
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

static Quaternion quat_slerp(Quaternion a, Quaternion b, float u)
{
    a = normalize_quat(a);
    b = normalize_quat(b);

    float d = quat_dot_local(a, b);

    // shortest-path slerp
    if (d < 0.0f) {
        d = -d;
        b.w = -b.w;
        b.x = -b.x;
        b.y = -b.y;
        b.z = -b.z;
    }

    u = std::clamp(u, 0.0f, 1.0f);

    // nearly identical: lerp + renormalize
    if (d > 0.9995f) {
        Quaternion q;
        q.w = a.w + u * (b.w - a.w);
        q.x = a.x + u * (b.x - a.x);
        q.y = a.y + u * (b.y - a.y);
        q.z = a.z + u * (b.z - a.z);
        return normalize_quat(q);
    }

    d = std::clamp(d, -1.0f, 1.0f);
    const float theta = std::acos(d);
    const float s = std::sin(theta);

    Quaternion q;
    const float wa = std::sin((1.0f - u) * theta) / s;
    const float wb = std::sin(u * theta) / s;

    q.w = wa * a.w + wb * b.w;
    q.x = wa * a.x + wb * b.x;
    q.y = wa * a.y + wb * b.y;
    q.z = wa * a.z + wb * b.z;
    return normalize_quat(q);
}

} // namespace

static float rad_to_deg(float radians)
{
    return radians * 180.0f / static_cast<float>(M_PI);
}

static void make_rear_Rflu2cv(const float R_front[9], float R_rear[9])
{
    // Rear camera is modeled as the front camera rotated 180 deg about
    // the buoy/body Z axis.  R_rear = R_front * Rz(pi).
    // Since Rz(pi) = diag(-1, -1, 1), this flips columns 0 and 1.
    R_rear[0] = -R_front[0];
    R_rear[1] = -R_front[1];
    R_rear[2] =  R_front[2];

    R_rear[3] = -R_front[3];
    R_rear[4] = -R_front[4];
    R_rear[5] =  R_front[5];

    R_rear[6] = -R_front[6];
    R_rear[7] = -R_front[7];
    R_rear[8] =  R_front[8];
}

// static member definitions
std::mutex BuoyNode::printMutex;

BuoyNode::BuoyNode(std::string name, int imuPort, Callback cb)
    : name_(std::move(name)), imuPort_(imuPort), sock_(-1), callback_(std::move(cb)),
      running_(false), lastQuat_()
{
    initCameraMatrices();
    initDebugCsv();
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

void BuoyNode::initDebugCsv()
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
    debugCsvFile_ << "timestamp_us,yaw_deg,pitch_deg,roll_deg,pitch_cmd_deg,roll_cmd_deg,corr_pitch_filt_deg,corr_roll_filt_deg,ok\n";
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

            Quaternion q_new;
            q_new.w = msg.quat_w();
            q_new.x = msg.quat_x();
            q_new.y = msg.quat_y();
            q_new.z = msg.quat_z();
            q_new = normalize_quat(q_new);

            // enforce sign continuity: q and -q represent the same
            // orientation, but discontinuous flips destabilize interpolation
            // and prediction.
            if (!quat_hist_.empty()) {
                const Quaternion &q_prev = quat_hist_.back().second;
                float dot = q_prev.w*q_new.w + q_prev.x*q_new.x + q_prev.y*q_new.y + q_prev.z*q_new.z;
                if (dot < 0.0f) {
                    q_new.w = -q_new.w;
                    q_new.x = -q_new.x;
                    q_new.y = -q_new.y;
                    q_new.z = -q_new.z;
                }
            }

            lastQuat_ = q_new;

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
            if (debugCsvEnabled_) {
                debugCsvFile_ << ts << ","
                              << rad_to_deg(debug.yaw) << ","
                              << rad_to_deg(debug.pitch) << ","
                              << rad_to_deg(debug.roll) << ","
                              << rad_to_deg(debug.pitch_cmd) << ","
                              << rad_to_deg(debug.roll_cmd) << ","
                              << rad_to_deg(debug.pitch_filt) << ","
                              << rad_to_deg(debug.roll_filt) << ","
                              << (ok ? 1 : 0) << "\n";
                debugCsvFile_.flush();
            }
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

std::array<float,9> BuoyNode::getHinv() const
{
    return getHinvForCamera(false);
}

std::array<float,9> BuoyNode::getHinvForCamera(bool rear_facing) const
{
    std::array<float,9> out;
    std::lock_guard<std::mutex> lock(quatMutex_);

    if (!rear_facing) {
        for (int i = 0; i < 9; ++i)
            out[i] = lastHinv_[i];
        return out;
    }

    float R_camera[9];
    make_rear_Rflu2cv(Rflu2cv_mat, R_camera);

    float Hinv[9];
    Quaternion quat_ref_local = quat_ref;
    bool have_ref_local = have_ref;
    float pitch_filt_local = corr_pitch_filt;
    float roll_filt_local = corr_roll_filt;

    bool ok = compute_homography_from_quat(lastQuat_,
                                           quat_ref_local,
                                           have_ref_local,
                                           corr_smooth_alpha,
                                           pitch_filt_local,
                                           roll_filt_local,
                                           R_camera,
                                           K,
                                           Kinv,
                                           cam_w,
                                           cam_h,
                                           Hinv,
                                           nullptr);
    if (ok) {
        for (int i = 0; i < 9; ++i)
            out[i] = Hinv[i];
    } else {
        for (int i = 0; i < 9; ++i)
            out[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    }
    return out;
}

bool BuoyNode::getHinvAt(uint64_t timestamp,
                         std::array<float,9> &out,
                         uint64_t *best_diff_us) const
{
    return getHinvAtForCamera(timestamp, out, false, best_diff_us);
}

bool BuoyNode::getHinvAtForCamera(uint64_t timestamp,
                                  std::array<float,9> &out,
                                  bool rear_facing,
                                  uint64_t *best_diff_us) const
{
    std::lock_guard<std::mutex> lock(quatMutex_);

    if (quat_hist_.size() < 2) {
        return false;
    }

    const uint64_t t_first = quat_hist_.front().first;
    const uint64_t t_last  = quat_hist_.back().first;

    if (timestamp + max_interp_gap_us_ < t_first) {
        if (best_diff_us) {
            *best_diff_us = t_first - timestamp;
        }
        return false;
    }

    Quaternion q_query{};

    if (timestamp <= t_last) {
        bool bracket_found = false;
        uint64_t bestDiff = UINT64_MAX;

        for (size_t k = 1; k < quat_hist_.size(); ++k) {
            const uint64_t ta = quat_hist_[k - 1].first;
            const uint64_t tb = quat_hist_[k].first;

            uint64_t diff_a = (ta > timestamp) ? (ta - timestamp) : (timestamp - ta);
            uint64_t diff_b = (tb > timestamp) ? (tb - timestamp) : (timestamp - tb);
            bestDiff = std::min(bestDiff, std::min(diff_a, diff_b));

            if (ta <= timestamp && timestamp <= tb && tb > ta) {
                const float u = float(timestamp - ta) / float(tb - ta);
                q_query = quat_slerp(quat_hist_[k - 1].second, quat_hist_[k].second, u);
                bracket_found = true;
                if (best_diff_us) {
                    *best_diff_us = bestDiff;
                }
                break;
            }
        }

        if (!bracket_found) {
            return false;
        }
    } else {
        const uint64_t dt_us = timestamp - t_last;
        if (dt_us > max_predict_us_) {
            if (best_diff_us) {
                *best_diff_us = dt_us;
            }
            return false;
        }

        Quaternion q_last = quat_hist_.back().second;
        float dt_pred = std::clamp(float(dt_us) * 1e-6f + predict_lead_s_,
                                   0.0f,
                                   predict_max_s_);

        Quaternion dq_pred = rotvec_to_quat(omega_filt_[0] * dt_pred,
                                            omega_filt_[1] * dt_pred,
                                            omega_filt_[2] * dt_pred);
        q_query = quat_mult(dq_pred, q_last);
        q_query = normalize_quat(q_query);

        if (best_diff_us) {
            *best_diff_us = dt_us;
        }
    }

    float Hinv[9];
    Quaternion quat_ref_local = quat_ref;
    bool have_ref_local = have_ref;
    float pitch_filt_local = corr_pitch_filt;
    float roll_filt_local = corr_roll_filt;

    

    float R_camera[9];
    if (rear_facing) {
        make_rear_Rflu2cv(Rflu2cv_mat, R_camera);
    } else {
        for (int j = 0; j < 9; ++j)
            R_camera[j] = Rflu2cv_mat[j];
    }

    bool ok = compute_homography_from_quat(q_query,
                                           quat_ref_local,
                                           have_ref_local,
                                           corr_smooth_alpha,
                                           pitch_filt_local,
                                           roll_filt_local,
                                           R_camera,
                                           K,
                                           Kinv,
                                           cam_w,
                                           cam_h,
                                           Hinv,
                                           nullptr);
    if (!ok) {
        return false;
    }

    for (int j = 0; j < 9; ++j) {
        out[j] = Hinv[j];
    }
    return true;
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
    // std::cout << '[' << name << "]\n";
    // std::cout << "  " << std::left << std::setw(W) << "acc_x"  << ": " << msg.acc_x()  << "\n"
    //           << "  " << std::left << std::setw(W) << "acc_y"  << ": " << msg.acc_y()  << "\n"
    //           << "  " << std::left << std::setw(W) << "acc_z"  << ": " << msg.acc_z()  << "\n"
    //           << "  " << std::left << std::setw(W) << "gyr_x"  << ": " << msg.gyr_x()  << "\n"
    //           << "  " << std::left << std::setw(W) << "gyr_y"  << ": " << msg.gyr_y()  << "\n"
    //           << "  " << std::left << std::setw(W) << "gyr_z"  << ": " << msg.gyr_z()  << "\n"
    //           << "  " << std::left << std::setw(W) << "quat_w" << ": " << msg.quat_w() << "\n"
    //           << "  " << std::left << std::setw(W) << "quat_x" << ": " << msg.quat_x() << "\n"
    //           << "  " << std::left << std::setw(W) << "quat_y" << ": " << msg.quat_y() << "\n"
    //           << "  " << std::left << std::setw(W) << "quat_z" << ": " << msg.quat_z() << "\n"
    //           << "  " << std::left << std::setw(W) << "timestamp" << ": " << msg.timestamp() << "\n";
    // std::cout << std::endl;
}
