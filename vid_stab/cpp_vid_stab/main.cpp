
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// =============================
// User settings
// =============================
static const std::string PORT = "/dev/ttyUSB0";
static const int BAUD = 460800;

static const std::string CAMERA_DEVICE = "/dev/video0";
static const int RES_W = 640;
static const int RES_H = 480;
static const int CAMERA_FPS = 30;
static const double HFOV_DEG = 50.0;

static double VIDEO_DELAY_S = 0.01;
static double ROT_SMOOTH_ALPHA = 1.0;
static bool USE_QUAT_CONJUGATE = false;

static double IMU_BUFFER_TIME_S = 4.0;
static int IMU_RATE_HZ_EST = 100;
static double FRAME_BUFFER_TIME_S = 4.0;

static bool SHOW_PREVIEW = true;

// =============================
// Data types
// =============================
struct QuatSample {
    double t_rx_s;
    Eigen::Quaterniond q;
};

struct FrameItem {
    double t_s;          // host receive time
    double pts_s;        // GStreamer buffer PTS if available, else -1
    cv::Mat frame_bgr;
};

struct SharedState {
    std::deque<QuatSample> quat_buffer;
    std::mutex mtx;
    std::atomic<bool> stop_flag{false};
};

// =============================
// Utility
// =============================
static double now_s() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    auto dt = std::chrono::duration<double>(clock::now() - t0).count();
    return dt;
}

static std::string to_string_fixed(double x, int prec = 1) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(prec) << x;
    return oss.str();
}

static Eigen::Vector3d normalize_vec(const Eigen::Vector3d& v) {
    double n = v.norm();
    if (n < 1e-12) return v;
    return v / n;
}

static Eigen::Quaterniond slerp_two(const Eigen::Quaterniond& qa,
                                    const Eigen::Quaterniond& qb,
                                    double alpha) {
    alpha = std::clamp(alpha, 0.0, 1.0);
    if (alpha <= 0.0) return qa;
    if (alpha >= 1.0) return qb;
    return qa.slerp(alpha, qb).normalized();
}

static Eigen::Quaterniond rotation_align_a_to_b(const Eigen::Vector3d& a_in,
                                                const Eigen::Vector3d& b_in) {
    Eigen::Vector3d a = normalize_vec(a_in);
    Eigen::Vector3d b = normalize_vec(b_in);

    double c = std::clamp(a.dot(b), -1.0, 1.0);

    if (c > 1.0 - 1e-8) {
        return Eigen::Quaterniond::Identity();
    }

    if (c < -1.0 + 1e-8) {
        Eigen::Vector3d trial(1.0, 0.0, 0.0);
        if (std::abs(a.dot(trial)) > 0.9) {
            trial = Eigen::Vector3d(0.0, 1.0, 0.0);
        }
        Eigen::Vector3d axis = normalize_vec(a.cross(trial));
        return Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, axis));
    }

    Eigen::Vector3d v = a.cross(b);
    double s = v.norm();

    Eigen::Matrix3d vx;
    vx <<     0.0, -v.z(),  v.y(),
           v.z(),    0.0, -v.x(),
          -v.y(),  v.x(),   0.0;

    Eigen::Matrix3d Rm = Eigen::Matrix3d::Identity()
                       + vx
                       + vx * vx * ((1.0 - c) / (s * s));

    return Eigen::Quaterniond(Rm).normalized();
}

static bool homography_is_safe(const cv::Mat& H, int w, int h) {
    std::vector<cv::Point2d> pts = {
        {0.0, 0.0},
        {double(w - 1), 0.0},
        {0.0, double(h - 1)},
        {double(w - 1), double(h - 1)},
        {double(w) * 0.5, double(h) * 0.5}
    };

    for (const auto& p : pts) {
        cv::Mat x = (cv::Mat_<double>(3,1) << p.x, p.y, 1.0);
        cv::Mat q = H * x;
        double z = q.at<double>(2,0);
        if (std::abs(z) < 1e-6) return false;
        double u = q.at<double>(0,0) / z;
        double v = q.at<double>(1,0) / z;
        if (!std::isfinite(u) || !std::isfinite(v)) return false;
        if (std::abs(u) > 5.0 * w || std::abs(v) > 5.0 * h) return false;
    }
    return true;
}

// =============================
// IMU serial
// =============================
static speed_t baud_to_termios(int baud) {
    switch (baud) {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B460800;
    }
}

static bool open_serial_fd(const std::string& port, int baud, int& fd_out) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return false;

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return false;
    }

    cfsetospeed(&tty, baud_to_termios(baud));
    cfsetispeed(&tty, baud_to_termios(baud));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return false;
    }

    fd_out = fd;
    return true;
}

static bool parse_imu_line(const std::string& line, Eigen::Quaterniond& q_out) {
    // sample_idx,mcu_time_us,qw,qx,qy,qz
    std::stringstream ss(line);
    std::string tok;
    std::vector<std::string> parts;
    while (std::getline(ss, tok, ',')) parts.push_back(tok);

    if (parts.size() < 6) return false;

    try {
        (void)std::stoi(parts[0]);
        (void)std::stoll(parts[1]);

        double w = std::stod(parts[2]);
        double x = std::stod(parts[3]);
        double y = std::stod(parts[4]);
        double z = std::stod(parts[5]);

        if (USE_QUAT_CONJUGATE) {
            x = -x; y = -y; z = -z;
        }

        q_out = Eigen::Quaterniond(w, x, y, z).normalized();
        return true;
    } catch (...) {
        return false;
    }
}

static void imu_reader_thread(SharedState& state) {
    int fd = -1;
    if (!open_serial_fd(PORT, BAUD, fd)) {
        std::cerr << "Failed to open serial: " << PORT << std::endl;
        return;
    }

    std::string line;
    line.reserve(256);

    bool have_prev_q = false;
    Eigen::Quaterniond prev_q = Eigen::Quaterniond::Identity();

    while (!state.stop_flag.load()) {
        char ch = 0;
        int n = read(fd, &ch, 1);
        if (n <= 0) continue;

        if (ch == '\n') {
            Eigen::Quaterniond q;
            if (parse_imu_line(line, q)) {
                // Enforce quaternion sign continuity: q and -q represent the
                // same physical orientation, but sign flips can make debug
                // Euler angles jump wildly.
                if (have_prev_q && prev_q.dot(q) < 0.0) {
                    q.coeffs() *= -1.0;
                }
                prev_q = q;
                have_prev_q = true;

                double t = now_s();
                std::lock_guard<std::mutex> lock(state.mtx);
                state.quat_buffer.push_back({t, q});
                const size_t max_len = static_cast<size_t>(IMU_BUFFER_TIME_S * IMU_RATE_HZ_EST);
                while (state.quat_buffer.size() > max_len) {
                    state.quat_buffer.pop_front();
                }
            }
            line.clear();
        } else if (ch != '\r') {
            line.push_back(ch);
        }
    }

    close(fd);
}

static bool get_quat_at_query_time(SharedState& state,
                                   double t_query,
                                   Eigen::Quaterniond& q_out,
                                   double& newest_imu_time) {
    std::lock_guard<std::mutex> lock(state.mtx);
    if (state.quat_buffer.size() < 2) return false;

    newest_imu_time = state.quat_buffer.back().t_rx_s;

    if (t_query <= state.quat_buffer.front().t_rx_s) {
        q_out = state.quat_buffer.front().q;
        return true;
    }
    if (t_query >= state.quat_buffer.back().t_rx_s) {
        q_out = state.quat_buffer.back().q;
        return true;
    }

    for (size_t i = 1; i < state.quat_buffer.size(); ++i) {
        const auto& a = state.quat_buffer[i - 1];
        const auto& b = state.quat_buffer[i];
        if (a.t_rx_s <= t_query && t_query <= b.t_rx_s) {
            double u = (t_query - a.t_rx_s) / (b.t_rx_s - a.t_rx_s);
            q_out = a.q.slerp(u, b.q).normalized();
            return true;
        }
    }

    return false;
}

// =============================
// Native GStreamer appsink input
// =============================
struct GstInput {
    GstElement* pipeline = nullptr;
    GstElement* appsink = nullptr;
};

static bool open_gst_input(GstInput& in) {
    std::ostringstream oss;
    oss
        << "v4l2src device=" << CAMERA_DEVICE << " io-mode=2 ! "
        << "video/x-raw,format=YUY2,width=" << RES_W
        << ",height=" << RES_H
        << ",framerate=" << CAMERA_FPS << "/1 ! "
        << "appsink name=sink drop=true max-buffers=1 sync=false";

    GError* err = nullptr;
    in.pipeline = gst_parse_launch(oss.str().c_str(), &err);
    if (!in.pipeline) {
        if (err) {
            std::cerr << "gst_parse_launch failed: " << err->message << std::endl;
            g_error_free(err);
        }
        return false;
    }

    in.appsink = gst_bin_get_by_name(GST_BIN(in.pipeline), "sink");
    if (!in.appsink) {
        std::cerr << "Failed to get appsink by name." << std::endl;
        return false;
    }

    gst_app_sink_set_emit_signals((GstAppSink*)in.appsink, false);
    gst_app_sink_set_drop((GstAppSink*)in.appsink, true);
    gst_app_sink_set_max_buffers((GstAppSink*)in.appsink, 1);

    GstStateChangeReturn ret = gst_element_set_state(in.pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Failed to set pipeline to PLAYING." << std::endl;
        return false;
    }

    return true;
}

static void close_gst_input(GstInput& in) {
    if (in.pipeline) {
        gst_element_set_state(in.pipeline, GST_STATE_NULL);
    }
    if (in.appsink) {
        gst_object_unref(in.appsink);
        in.appsink = nullptr;
    }
    if (in.pipeline) {
        gst_object_unref(in.pipeline);
        in.pipeline = nullptr;
    }
}

static bool pull_yuy2_frame(GstInput& in, cv::Mat& frame_raw, double& pts_s) {
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(in.appsink), 50000000); // 50 ms
    if (!sample) return false;

    GstBuffer* buffer = gst_sample_get_buffer(sample);

    GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (GST_CLOCK_TIME_IS_VALID(pts)) {
        pts_s = static_cast<double>(pts) / 1e9;
    } else {
        pts_s = -1.0;
    }

    GstCaps* caps = gst_sample_get_caps(sample);
    if (!buffer || !caps) {
        gst_sample_unref(sample);
        return false;
    }

    GstStructure* s = gst_caps_get_structure(caps, 0);
    int width = 0, height = 0;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return false;
    }

    // YUY2 is 2 bytes per pixel -> CV_8UC2
    cv::Mat wrapped(height, width, CV_8UC2, (void*)map.data);
    frame_raw = wrapped.clone();

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return true;
}

// =============================
// Main
// =============================
int main() {
    gst_init(nullptr, nullptr);

    std::cout << "Opening native GStreamer appsink input..." << std::endl;

    GstInput gst_in;
    if (!open_gst_input(gst_in)) {
        return 1;
    }

    SharedState state;
    std::thread imu_thread(imu_reader_thread, std::ref(state));

    const double fx = 0.5 * RES_W / std::tan(HFOV_DEG * M_PI / 180.0 / 2.0);
    const double fy = fx;
    const double cx = RES_W / 2.0;
    const double cy = RES_H / 2.0;

    cv::Mat K = (cv::Mat_<double>(3,3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    cv::Mat Kinv = K.inv();

    Eigen::Matrix3d R_flu_to_cv;
    R_flu_to_cv <<
        0, -1,  0,
        0,  0, -1,
        1,  0,  0;

    std::deque<FrameItem> frame_buffer;
    const size_t max_frame_buf = static_cast<size_t>(FRAME_BUFFER_TIME_S * CAMERA_FPS);

    Eigen::Quaterniond R_stab_filt = Eigen::Quaterniond::Identity();

    using clock_type = std::chrono::steady_clock;

    double loop_fps_smoothed = 0.0;
    double cap_ms_smoothed = 0.0;
    double cvt_ms_smoothed = 0.0;
    double imu_interp_ms_smoothed = 0.0;
    double stab_ms_smoothed = 0.0;
    double warp_ms_smoothed = 0.0;
    double loop_ms_smoothed = 0.0;

    double last_display_pts_s = -1.0;
    double pts_dt_ms_smoothed = 0.0;
    double pts_fps_smoothed = 0.0;

    const double timing_alpha = 0.1;
    auto smooth_update = [&](double& s, double x) {
        if (s <= 0.0) s = x;
        else s = (1.0 - timing_alpha) * s + timing_alpha * x;
    };

    double last_print_s = now_s();

    while (true) {
        auto loop_t0 = clock_type::now();

        cv::Mat frame_raw;
        double pts_s = -1.0;
        auto t_cap0 = clock_type::now();
        bool ok = pull_yuy2_frame(gst_in, frame_raw, pts_s);
        auto t_cap1 = clock_type::now();
        if (!ok) continue;

        double cap_ms = std::chrono::duration<double, std::milli>(t_cap1 - t_cap0).count();
        smooth_update(cap_ms_smoothed, cap_ms);

        cv::Mat frame_bgr;
        auto t_cvt0 = clock_type::now();
        cv::cvtColor(frame_raw, frame_bgr, cv::COLOR_YUV2BGR_YUY2);
        auto t_cvt1 = clock_type::now();
        double cvt_ms = std::chrono::duration<double, std::milli>(t_cvt1 - t_cvt0).count();
        smooth_update(cvt_ms_smoothed, cvt_ms);

        double t_now = now_s();
        frame_buffer.push_back({t_now, pts_s, frame_bgr});
        while (frame_buffer.size() > max_frame_buf) {
            frame_buffer.pop_front();
        }

        double t_target = t_now - VIDEO_DELAY_S;
        bool have_frame = false;
        FrameItem frame_to_display;

        while (!frame_buffer.empty() && frame_buffer.front().t_s <= t_target) {
            frame_to_display = frame_buffer.front();
            frame_buffer.pop_front();
            have_frame = true;
        }

        if (!have_frame) continue;

        if (frame_to_display.pts_s >= 0.0 && last_display_pts_s >= 0.0) {
            double pts_dt_ms = (frame_to_display.pts_s - last_display_pts_s) * 1000.0;
            smooth_update(pts_dt_ms_smoothed, pts_dt_ms);

            if (pts_dt_ms > 1e-6) {
                double pts_fps = 1000.0 / pts_dt_ms;
                smooth_update(pts_fps_smoothed, pts_fps);
            }
        }
        last_display_pts_s = frame_to_display.pts_s;

        double newest_imu_time = 0.0;
        Eigen::Quaterniond R_cur_flu;
        auto t_imu0 = clock_type::now();
        bool got_q = get_quat_at_query_time(state, frame_to_display.t_s, R_cur_flu, newest_imu_time);
        auto t_imu1 = clock_type::now();
        if (!got_q) continue;

        double imu_interp_ms = std::chrono::duration<double, std::milli>(t_imu1 - t_imu0).count();
        smooth_update(imu_interp_ms_smoothed, imu_interp_ms);

        auto t_stab0 = clock_type::now();

        Eigen::Vector3d up_world(0.0, 0.0, 1.0);
        Eigen::Vector3d up_body = R_cur_flu.conjugate() * up_world;
        Eigen::Vector3d up_cam = normalize_vec(R_flu_to_cv * up_body);
        Eigen::Vector3d img_up_cam(0.0, -1.0, 0.0);

        Eigen::Quaterniond R_stab_target_cv = rotation_align_a_to_b(up_cam, img_up_cam);
        R_stab_filt = slerp_two(R_stab_filt, R_stab_target_cv, ROT_SMOOTH_ALPHA);

        Eigen::Matrix3d Rcv = R_stab_filt.toRotationMatrix();
        cv::Mat H = (cv::Mat_<double>(3,3) <<
            Rcv(0,0), Rcv(0,1), Rcv(0,2),
            Rcv(1,0), Rcv(1,1), Rcv(1,2),
            Rcv(2,0), Rcv(2,1), Rcv(2,2));
        H = K * H * Kinv;

        auto t_stab1 = clock_type::now();
        double stab_ms = std::chrono::duration<double, std::milli>(t_stab1 - t_stab0).count();
        smooth_update(stab_ms_smoothed, stab_ms);

        cv::Mat warped;
        auto t_warp0 = clock_type::now();
        if (!homography_is_safe(H, RES_W, RES_H)) {
            warped = frame_to_display.frame_bgr.clone();
        } else {
            cv::warpPerspective(frame_to_display.frame_bgr, warped, H, cv::Size(RES_W, RES_H),
                                cv::INTER_LINEAR, cv::BORDER_REPLICATE);
        }
        auto t_warp1 = clock_type::now();
        double warp_ms = std::chrono::duration<double, std::milli>(t_warp1 - t_warp0).count();
        smooth_update(warp_ms_smoothed, warp_ms);

        auto loop_t1 = clock_type::now();
        double loop_ms = std::chrono::duration<double, std::milli>(loop_t1 - loop_t0).count();
        smooth_update(loop_ms_smoothed, loop_ms);

        double loop_fps_inst = (loop_ms > 1e-6) ? (1000.0 / loop_ms) : 0.0;
        smooth_update(loop_fps_smoothed, loop_fps_inst);

        double frame_delay_ms = (t_now - frame_to_display.t_s) * 1000.0;
        double imu_age_ms = (t_now - newest_imu_time) * 1000.0;

        // Euler angles are only for rough debugging and can jump by 180 deg due to
        // branch cuts. The stabilization does not depend on these values.
        Eigen::Vector3d eul = R_cur_flu.toRotationMatrix().eulerAngles(2, 1, 0);
        double yaw_deg = eul[0] * 180.0 / M_PI;
        double pitch_deg = eul[1] * 180.0 / M_PI;
        double roll_deg = eul[2] * 180.0 / M_PI;

        if (now_s() - last_print_s >= 1.0) {
            size_t imu_buf_sz = 0;
            {
                std::lock_guard<std::mutex> lock(state.mtx);
                imu_buf_sz = state.quat_buffer.size();
            }

            std::cout
                << "[CUR] yaw=" << to_string_fixed(yaw_deg, 2)
                << " pitch=" << to_string_fixed(pitch_deg, 2)
                << " roll=" << to_string_fixed(roll_deg, 2)
                << " deg  [frame_delay] " << to_string_fixed(frame_delay_ms, 2) << " ms"
                << "  [pts_dt] " << to_string_fixed(pts_dt_ms_smoothed, 2) << " ms"
                << "  [pts_fps] " << to_string_fixed(pts_fps_smoothed, 1)
                << "  [imu_age] " << to_string_fixed(imu_age_ms, 2) << " ms"
                << "  [loop_fps] " << to_string_fixed(loop_fps_smoothed, 1)
                << "  [cap] " << to_string_fixed(cap_ms_smoothed, 1) << " ms"
                << "  [cvt] " << to_string_fixed(cvt_ms_smoothed, 1) << " ms"
                << "  [imu_interp] " << to_string_fixed(imu_interp_ms_smoothed, 2) << " ms"
                << "  [stab] " << to_string_fixed(stab_ms_smoothed, 2) << " ms"
                << "  [warp] " << to_string_fixed(warp_ms_smoothed, 1) << " ms"
                << "  [loop] " << to_string_fixed(loop_ms_smoothed, 1) << " ms"
                << "  [imu_buf] " << imu_buf_sz
                << "  [frame_buf] " << frame_buffer.size()
                << std::endl;

            last_print_s = now_s();
        }

        if (SHOW_PREVIEW) {
            cv::putText(warped, "PTS FPS: " + to_string_fixed(pts_fps_smoothed, 1),
                        {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,255,255}, 2);
            cv::putText(warped, "Loop FPS: " + to_string_fixed(loop_fps_smoothed, 1),
                        {20, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
            cv::putText(warped, "Cap: " + to_string_fixed(cap_ms_smoothed, 1) + " ms",
                        {20, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
            cv::putText(warped, "Cvt: " + to_string_fixed(cvt_ms_smoothed, 1) + " ms",
                        {20, 120}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
            cv::putText(warped, "Warp: " + to_string_fixed(warp_ms_smoothed, 1) + " ms",
                        {20, 150}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
            cv::putText(warped, "Delay: " + to_string_fixed(frame_delay_ms, 1) + " ms",
                        {20, 180}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);

            cv::imshow("stabilized_native_appsink", warped);
            int key = cv::waitKey(1);
            if (key == 'q' || key == 27) break;
        }
    }

    state.stop_flag.store(true);
    if (imu_thread.joinable()) imu_thread.join();
    close_gst_input(gst_in);
    return 0;
}
