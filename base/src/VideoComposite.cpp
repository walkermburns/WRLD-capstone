#include "VideoComposite.h"
#include "MathHelpers.h"
#include "BuoyNode.h"

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/gl/gl.h>
#include <gst/app/gstappsink.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <cstring>
#include <mutex>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <cctype>

using namespace MathHelpers;

VideoComposite *VideoComposite::s_instance = nullptr;

struct BranchProbeCtx {
    VideoComposite *self = nullptr;
    int branch_index = -1;
};

struct DetectorSinkCtx {
    VideoComposite *self = nullptr;
};

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer)
{
    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        GError *err = nullptr;
        gchar *dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        g_printerr("[gst] ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
        if (dbg) {
            g_printerr("[gst] Debug info: %s\n", dbg);
            g_free(dbg);
        }
        g_error_free(err);
        break;
    }
    case GST_MESSAGE_WARNING: {
        GError *err = nullptr;
        gchar *dbg = nullptr;
        gst_message_parse_warning(msg, &err, &dbg);
        g_printerr("[gst] WARNING from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
        if (dbg) {
            g_printerr("[gst] Debug info: %s\n", dbg);
            g_free(dbg);
        }
        g_error_free(err);
        break;
    }
    default:
        break;
    }
    return TRUE;
}

#pragma pack(push, 1)
struct VideoTimestampPacket {
    uint32_t frame_idx_be;
    uint64_t timestamp_us_be;
};
#pragma pack(pop)

static uint64_t ntohll_u64(uint64_t x)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return (static_cast<uint64_t>(ntohl(static_cast<uint32_t>(x & 0xFFFFFFFFULL))) << 32) |
           ntohl(static_cast<uint32_t>(x >> 32));
#else
    return x;
#endif
}

static std::string make_default_record_path()
{
    std::time_t now = std::time(nullptr);
    std::tm tm_now{};
#if defined(_WIN32)
    localtime_s(&tm_now, &now);
#else
    localtime_r(&now, &tm_now);
#endif

    std::ostringstream oss;
    oss << "composite_recording_"
        << std::put_time(&tm_now, "%Y%m%d_%H%M%S")
        << ".mkv";
    return oss.str();
}

VideoComposite::VideoComposite(const std::string &shaderPath,
                               const std::vector<int> &ports)
    : pipeline(nullptr), mix_element(nullptr), main_loop_(nullptr), live_k1(0.3f), live_zoom(1.1f),
      live_w(1920.0f), live_h(1080.0f),
      live_h00(1.0f), live_h01(0.0f), live_h02(0.0f),
      live_h10(0.0f), live_h11(1.0f), live_h12(0.0f),
      live_h20(0.0f), live_h21(0.0f), live_h22(1.0f),
      num_src(0), uniforms(nullptr), stab(), branch_active(), quat_(), nodes_(nullptr)
{
    video_ports = ports;
    num_src = static_cast<int>(video_ports.size());
    stab.assign(num_src, nullptr);
    branch_active.assign(num_src, false);
    frame_pts_to_imu_offset_us_.assign(num_src, 0LL);
    sender_ts_queue_us_.assign(num_src, std::deque<FrameMetaEntry>{});

    last_applied_H_.assign(num_src, std::array<float,9>{});
    last_applied_H_valid_.assign(num_src, false);
    last_used_frame_ts_us_.assign(num_src, 0ull);
    metadata_seen_.assign(num_src, false);
    for (int k = 0; k < num_src; ++k) {
        for (int j = 0; j < 9; ++j) {
            last_applied_H_[k][j] = (j % 4 == 0) ? 1.0f : 0.0f;
        }
    }

    if (const char *main_rec_env = std::getenv("RECORD_OUTPUT")) {
        main_record_enabled_ = std::atoi(main_rec_env) != 0;
    }
    if (const char *det_env = std::getenv("ENABLE_PERSON_DETECTION")) {
        person_detection_enabled_ = std::atoi(det_env) != 0;
    }
    if (const char *w_env = std::getenv("PERSON_DET_WIDTH")) {
        detector_width_ = std::max(64, std::atoi(w_env));
    }
    if (const char *h_env = std::getenv("PERSON_DET_HEIGHT")) {
        detector_height_ = std::max(64, std::atoi(h_env));
    }
    if (const char *p_env = std::getenv("PERSON_DET_PERIOD_MS")) {
        detector_period_ms_ = std::max(1, std::atoi(p_env));
    }
    if (const char *prev_env = std::getenv("SHOW_PERSON_DET_PREVIEW")) {
        detector_preview_enabled_ = std::atoi(prev_env) != 0;
    }
    if (const char *backend_env = std::getenv("PERSON_DET_BACKEND")) {
        detector_backend_ = backend_env;
        std::transform(detector_backend_.begin(), detector_backend_.end(), detector_backend_.begin(),
                       [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    }
#ifdef HAVE_ORT
    if ((detector_backend_ == "yolov8" || detector_backend_ == "yolo" || detector_backend_ == "ort") ) {
        yolo_ort_ready_ = initYoloOrtSession();
        if (!yolo_ort_ready_) {
            detector_backend_ = "hog";
        }
    }
#else
    if (detector_backend_ == "yolov8" || detector_backend_ == "yolo" || detector_backend_ == "ort") {
        g_printerr("[detector] YOLOv8 backend requested but this build does not include ONNX Runtime. Falling back to HOG\n");
        detector_backend_ = "hog";
    }
#endif

    std::ifstream in(shaderPath);
    if (!in) {
        std::string msg = "failed to open shader file '" + shaderPath + "'";
        std::cerr << msg << "\n";
        throw std::runtime_error(msg);
    }
    std::ostringstream buf;
    buf << in.rdbuf();
    shader_code = buf.str();
    if (shader_code.empty()) {
        std::string msg = "shader file '" + shaderPath + "' is empty";
        std::cerr << msg << "\n";
        throw std::runtime_error(msg);
    }

    uniforms = gst_structure_new("uniforms",
                                 "k1", G_TYPE_FLOAT, live_k1,
                                 "zoom", G_TYPE_FLOAT, live_zoom,
                                 "w", G_TYPE_FLOAT, live_w,
                                 "h", G_TYPE_FLOAT, live_h,
                                 "h00", G_TYPE_FLOAT, live_h00,
                                 "h01", G_TYPE_FLOAT, live_h01,
                                 "h02", G_TYPE_FLOAT, live_h02,
                                 "h10", G_TYPE_FLOAT, live_h10,
                                 "h11", G_TYPE_FLOAT, live_h11,
                                 "h12", G_TYPE_FLOAT, live_h12,
                                 "h20", G_TYPE_FLOAT, live_h20,
                                 "h21", G_TYPE_FLOAT, live_h21,
                                 "h22", G_TYPE_FLOAT, live_h22,
                                 NULL);
}

VideoComposite::~VideoComposite()
{
    stop_requested_ = true;
    stopDetectorThread();
    stop_metadata_receivers();
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        for (int i = 0; i < num_src && i < (int)stab.size(); i++) {
            if (stab[i]) gst_object_unref(stab[i]);
        }
        gst_object_unref(pipeline);
    }
    if (uniforms) {
        gst_structure_free(uniforms);
        uniforms = nullptr;
    }

    if (detector_preview_timer_id_ != 0) {
        g_source_remove(detector_preview_timer_id_);
        detector_preview_timer_id_ = 0;
    }

    if (detector_preview_enabled_ && detector_preview_window_created_) {
        try {
            cv::destroyWindow(detector_preview_window_name_);
            cv::waitKey(1);
        } catch (...) {
        }
        detector_preview_window_created_ = false;
    }
}



gboolean VideoComposite::detector_preview_tick_cb(gpointer user_data)
{
    auto *self = static_cast<VideoComposite *>(user_data);
    if (!self || !self->detector_preview_enabled_)
        return G_SOURCE_CONTINUE;

    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(self->detector_preview_mutex_);
        if (!self->detector_preview_frame_ready_)
            return G_SOURCE_CONTINUE;
        frame = self->detector_preview_frame_.clone();
        self->detector_preview_frame_ready_ = false;
    }

    if (frame.empty())
        return G_SOURCE_CONTINUE;

    try {
        if (!self->detector_preview_window_created_) {
            cv::namedWindow(self->detector_preview_window_name_, cv::WINDOW_NORMAL);
            cv::resizeWindow(self->detector_preview_window_name_, frame.cols, frame.rows);
            self->detector_preview_window_created_ = true;
        }
        cv::imshow(self->detector_preview_window_name_, frame);
        cv::waitKey(1);
    } catch (...) {
    }

    return G_SOURCE_CONTINUE;
}
void VideoComposite::start()
{
    stop_requested_ = false;
    gst_init(nullptr, nullptr);
    start_metadata_receivers();
#ifdef __APPLE__
    s_instance = this;
    gst_macos_main((GstMainFunc)VideoComposite::run_pipeline, 0, nullptr, this);
#else
    run_pipeline(this);
#endif
}

void VideoComposite::stop()
{
    stop_requested_ = true;
    stopDetectorThread();
    stop_metadata_receivers();
    if (main_loop_) {
        g_main_loop_quit(main_loop_);
    }
}

void VideoComposite::start_metadata_receivers()
{
    if (meta_threads_running_)
        return;
    meta_threads_running_ = true;
    meta_threads_.clear();
    for (int i = 0; i < num_src; ++i) {
        const int meta_port = video_ports[i] + 1;
        meta_threads_.emplace_back(&VideoComposite::metadata_receiver_loop, this, i, meta_port);
    }
}

void VideoComposite::stop_metadata_receivers()
{
    if (!meta_threads_running_)
        return;
    meta_threads_running_ = false;
    for (auto &t : meta_threads_) {
        if (t.joinable())
            t.join();
    }
    meta_threads_.clear();
}

void VideoComposite::metadata_receiver_loop(int branch_index, int meta_port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("metadata socket");
        return;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(meta_port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        perror("metadata bind");
        close(sock);
        return;
    }

    g_print("[meta rx %d] listening on UDP %d\n", branch_index, meta_port);

    while (meta_threads_running_) {
        VideoTimestampPacket pkt{};
        ssize_t n = recv(sock, &pkt, sizeof(pkt), 0);
        if (n < 0) {
            continue;
        }
        if (n != (ssize_t)sizeof(pkt)) {
            continue;
        }

        FrameMetaEntry stamp{};
        stamp.frame_idx = ntohl(pkt.frame_idx_be);
        stamp.timestamp_us = ntohll_u64(pkt.timestamp_us_be);

        {
            std::lock_guard<std::mutex> lock(sender_ts_mutex_);
            auto &q = sender_ts_queue_us_[branch_index];
            q.push_back(stamp);
            while (q.size() > 64) {
                q.pop_front();
            }
        }

        static thread_local uint64_t dbg_counter = 0;
        if ((++dbg_counter % 30) == 0) {
            size_t qsize = 0;
            {
                std::lock_guard<std::mutex> lock(sender_ts_mutex_);
                qsize = sender_ts_queue_us_[branch_index].size();
            }
            g_print("[meta rx %d] frame_idx=%" G_GUINT64_FORMAT
                    " ts_us=%" G_GUINT64_FORMAT " qsize=%zu\n",
                    branch_index,
                    stamp.frame_idx,
                    stamp.timestamp_us,
                    qsize);
        }
    }

    close(sock);
}

void VideoComposite::setUniforms(float k1, float zoom, float w, float h,
                                 float h00, float h01, float h02,
                                 float h10, float h11, float h12,
                                 float h20, float h21, float h22)
{
    live_k1 = k1;
    live_zoom = zoom;
    live_w = w;
    live_h = h;
    live_h00 = h00; live_h01 = h01; live_h02 = h02;
    live_h10 = h10; live_h11 = h11; live_h12 = h12;
    live_h20 = h20; live_h21 = h21; live_h22 = h22;

    if (uniforms) {
        gst_structure_set(uniforms,
                          "k1", G_TYPE_FLOAT, live_k1,
                          "zoom", G_TYPE_FLOAT, live_zoom,
                          "w", G_TYPE_FLOAT, live_w,
                          "h", G_TYPE_FLOAT, live_h,
                          "h00", G_TYPE_FLOAT, live_h00,
                          "h01", G_TYPE_FLOAT, live_h01,
                          "h02", G_TYPE_FLOAT, live_h02,
                          "h10", G_TYPE_FLOAT, live_h10,
                          "h11", G_TYPE_FLOAT, live_h11,
                          "h12", G_TYPE_FLOAT, live_h12,
                          "h20", G_TYPE_FLOAT, live_h20,
                          "h21", G_TYPE_FLOAT, live_h21,
                          "h22", G_TYPE_FLOAT, live_h22,
                          NULL);
    }
}

void VideoComposite::updateQuaternion(const buoy_proto::IMU_proto &msg)
{
    quat_.w = msg.quat_w();
    quat_.x = msg.quat_x();
    quat_.y = msg.quat_y();
    quat_.z = msg.quat_z();
}

void VideoComposite::startDetectorThread()
{
    if (!person_detection_enabled_ || detector_thread_running_)
        return;
    detector_thread_running_ = true;
    detector_thread_ = std::thread(&VideoComposite::detectorWorkerLoop, this);
}

void VideoComposite::stopDetectorThread()
{
    if (!detector_thread_running_)
        return;
    detector_thread_running_ = false;
    detector_cv_.notify_all();
    if (detector_thread_.joinable())
        detector_thread_.join();
}


#ifdef HAVE_ORT
bool VideoComposite::initYoloOrtSession()
{
    const char *model_env = std::getenv("YOLO_MODEL_PATH");
    if (!model_env || std::string(model_env).empty()) {
        g_printerr("[detector] YOLO_MODEL_PATH not set; cannot initialize YOLOv8 backend\n");
        return false;
    }

    try {
        ort_session_options_.SetIntraOpNumThreads(1);
        ort_session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        ort_session_ = std::make_unique<Ort::Session>(ort_env_, model_env, ort_session_options_);

        Ort::AllocatorWithDefaultOptions allocator;
        auto in_name = ort_session_->GetInputNameAllocated(0, allocator);
        ort_input_name_ = in_name.get();

        auto input_type_info = ort_session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
        auto input_shape = input_type_info.GetShape();
        if (input_shape.size() >= 4) {
            if (input_shape[2] > 0) yolo_model_input_h_ = static_cast<int>(input_shape[2]);
            if (input_shape[3] > 0) yolo_model_input_w_ = static_cast<int>(input_shape[3]);
        }

        ort_output_names_.clear();
        const size_t out_count = ort_session_->GetOutputCount();
        ort_output_names_.reserve(out_count);
        for (size_t i = 0; i < out_count; ++i) {
            auto out_name = ort_session_->GetOutputNameAllocated(i, allocator);
            ort_output_names_.push_back(out_name.get());
        }
        g_print("[detector] YOLOv8 ONNX Runtime backend initialized: %s\n", model_env);
        return true;
    } catch (const std::exception &e) {
        g_printerr("[detector] Failed to initialize YOLOv8 ONNX Runtime backend: %s\n", e.what());
        return false;
    }
}
#endif

std::vector<VideoComposite::DetectionBox> VideoComposite::runDetector(const cv::Mat &bgr)
{
    if (detector_backend_ == "yolov8" || detector_backend_ == "yolo" || detector_backend_ == "ort") {
        return runYoloV8Detector(bgr);
    }
    return runHogPersonDetector(bgr);
}

std::vector<VideoComposite::DetectionBox> VideoComposite::runYoloV8Detector(const cv::Mat &bgr)
{
#ifndef HAVE_ORT
    (void)bgr;
    return {};
#else
    std::vector<DetectionBox> boxes_out;
    if (!yolo_ort_ready_ || !ort_session_) {
        return boxes_out;
    }

    const int input_w = yolo_model_input_w_ > 0 ? yolo_model_input_w_ : detector_width_;
    const int input_h = yolo_model_input_h_ > 0 ? yolo_model_input_h_ : detector_height_;

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(input_w, input_h));

    std::vector<float> input_tensor_values(static_cast<size_t>(3 * input_w * input_h));
    for (int y = 0; y < input_h; ++y) {
        const cv::Vec3b* row = resized.ptr<cv::Vec3b>(y);
        for (int x = 0; x < input_w; ++x) {
            const cv::Vec3b& px = row[x];
            const size_t idx = static_cast<size_t>(y * input_w + x);
            input_tensor_values[idx] = px[2] / 255.0f;
            input_tensor_values[input_w * input_h + idx] = px[1] / 255.0f;
            input_tensor_values[2 * input_w * input_h + idx] = px[0] / 255.0f;
        }
    }

    const std::array<int64_t, 4> input_shape{1, 3, input_h, input_w};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_tensor_values.data(), input_tensor_values.size(),
        input_shape.data(), input_shape.size());

    std::vector<const char*> output_names;
    output_names.reserve(ort_output_names_.size());
    for (const auto& s : ort_output_names_) output_names.push_back(s.c_str());

    const char* input_names[] = { ort_input_name_.c_str() };

    auto output_tensors = ort_session_->Run(Ort::RunOptions{nullptr},
                                            input_names,
                                            &input_tensor,
                                            1,
                                            output_names.data(),
                                            output_names.size());
                                            
    if (output_tensors.empty()) {
        return boxes_out;
    }

    Ort::Value &outv = output_tensors[0];
    auto type_info = outv.GetTensorTypeAndShapeInfo();
    auto shape = type_info.GetShape();
    const float* data = outv.GetTensorData<float>();
    if (!data || shape.size() < 3) {
        return boxes_out;
    }

    int rows = 0;
    int dims = 0;
    bool transposed = false;
    if (shape.size() == 3) {
        const int d1 = static_cast<int>(shape[1]);
        const int d2 = static_cast<int>(shape[2]);
        if (d1 > 0 && d1 <= 256 && d2 > d1) {
            dims = d1;
            rows = d2;
            transposed = true;   // [1, dims, rows], common for YOLOv8 ONNX
        } else {
            rows = d1;
            dims = d2;
            transposed = false;  // [1, rows, dims]
        }
    } else {
        return boxes_out;
    }

    if (dims < 4 || rows <= 0) {
        return boxes_out;
    }

    std::vector<cv::Rect> rects;
    std::vector<float> scores;
    const int person_class = 0;
    const bool has_objectness = (dims >= 6 && dims < 84);

    for (int i = 0; i < rows; ++i) {
        auto getv = [&](int d)->float {
            if (transposed) return data[d * rows + i];
            return data[i * dims + d];
        };

        const float a = getv(0);
        const float b = getv(1);
        const float c = getv(2);
        const float d = getv(3);

        float score = 0.0f;
        if (has_objectness) {
            const float objectness = getv(4);
            float class_score = 1.0f;
            if (5 + person_class < dims) class_score = getv(5 + person_class);
            score = objectness * class_score;
        } else if (4 + person_class < dims) {
            score = getv(4 + person_class);
        }
        if (score < detector_conf_threshold_) continue;

        float x1 = 0.0f, y1 = 0.0f, x2 = 0.0f, y2 = 0.0f;

        const bool looks_like_xyxy = (c > a) && (d > b);
        if (looks_like_xyxy) {
            x1 = a; y1 = b; x2 = c; y2 = d;
        } else {
            const float cx = a;
            const float cy = b;
            const float w = c;
            const float h = d;
            x1 = cx - 0.5f * w;
            y1 = cy - 0.5f * h;
            x2 = cx + 0.5f * w;
            y2 = cy + 0.5f * h;
        }

        const bool normalized = std::fabs(x1) <= 2.0f && std::fabs(y1) <= 2.0f &&
                                std::fabs(x2) <= 2.0f && std::fabs(y2) <= 2.0f;
        if (normalized) {
            x1 *= static_cast<float>(bgr.cols);
            x2 *= static_cast<float>(bgr.cols);
            y1 *= static_cast<float>(bgr.rows);
            y2 *= static_cast<float>(bgr.rows);
        } else {
            const float sx = static_cast<float>(bgr.cols) / static_cast<float>(input_w);
            const float sy = static_cast<float>(bgr.rows) / static_cast<float>(input_h);
            x1 *= sx; x2 *= sx;
            y1 *= sy; y2 *= sy;
        }

        int left = static_cast<int>(std::round(std::min(x1, x2)));
        int top = static_cast<int>(std::round(std::min(y1, y2)));
        int right = static_cast<int>(std::round(std::max(x1, x2)));
        int bottom = static_cast<int>(std::round(std::max(y1, y2)));

        left = std::max(0, std::min(left, bgr.cols - 1));
        top = std::max(0, std::min(top, bgr.rows - 1));
        right = std::max(0, std::min(right, bgr.cols));
        bottom = std::max(0, std::min(bottom, bgr.rows));

        int bw = right - left;
        int bh = bottom - top;
        if (bw <= 1 || bh <= 1) continue;

        rects.emplace_back(left, top, bw, bh);
        scores.push_back(score);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(rects, scores, detector_conf_threshold_, detector_nms_threshold_, keep);
    boxes_out.reserve(keep.size());
    for (int idx : keep) {
        DetectionBox box;
        box.x = rects[idx].x;
        box.y = rects[idx].y;
        box.w = rects[idx].width;
        box.h = rects[idx].height;
        box.score = scores[idx];
        boxes_out.push_back(box);
    }
    return boxes_out;
#endif
}

std::vector<VideoComposite::DetectionBox> VideoComposite::runHogPersonDetector(const cv::Mat &bgr)
{
    static thread_local cv::HOGDescriptor hog;
    static thread_local bool hog_ready = false;
    if (!hog_ready) {
        hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        hog_ready = true;
    }

    std::vector<cv::Rect> found;
    std::vector<double> weights;
    hog.detectMultiScale(bgr, found, weights, 0.0, cv::Size(8, 8), cv::Size(16, 16), 1.05, 2.0, false);

    std::vector<DetectionBox> boxes;
    boxes.reserve(found.size());
    for (size_t k = 0; k < found.size(); ++k) {
        const auto &r = found[k];
        if (r.width <= 0 || r.height <= 0)
            continue;
        if (r.height < bgr.rows / 8)
            continue;
        DetectionBox box;
        box.x = r.x;
        box.y = r.y;
        box.w = r.width;
        box.h = r.height;
        box.score = (k < weights.size()) ? weights[k] : 0.0;
        boxes.push_back(box);
    }
    return boxes;
}

void VideoComposite::detectorWorkerLoop()
{
    while (detector_thread_running_) {
        cv::Mat frame_copy;
        {
            std::unique_lock<std::mutex> lock(detector_frame_mutex_);
            detector_cv_.wait(lock, [this]{
                return !detector_thread_running_ || detector_have_new_frame_;
            });
            if (!detector_thread_running_)
                break;

            const auto now = std::chrono::steady_clock::now();
            if (detector_last_run_tp_.time_since_epoch().count() != 0) {
                const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - detector_last_run_tp_).count();
                if (dt_ms < detector_period_ms_) {
                    lock.unlock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(detector_period_ms_ - dt_ms));
                    continue;
                }
            }

            frame_copy = detector_latest_frame_.clone();
            detector_have_new_frame_ = false;
            detector_last_run_tp_ = std::chrono::steady_clock::now();
        }

        if (frame_copy.empty())
            continue;

        std::vector<DetectionBox> boxes = runDetector(frame_copy);
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            latest_composite_boxes_ = boxes;
            latest_composite_detection_tp_ = std::chrono::steady_clock::now();
        }

        if (detector_preview_enabled_) {
            cv::Mat preview = frame_copy.clone();
            for (const auto &box : boxes) {
                cv::rectangle(preview, cv::Rect(box.x, box.y, box.w, box.h), cv::Scalar(0, 255, 0), 2);
                std::ostringstream label;
                label << "person " << std::fixed << std::setprecision(2) << box.score;
                cv::putText(preview, label.str(),
                            cv::Point(box.x, std::max(18, box.y - 6)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1, cv::LINE_AA);
            }
            {
                std::lock_guard<std::mutex> lock(detector_preview_mutex_);
                detector_preview_frame_ = std::move(preview);
                detector_preview_frame_ready_ = true;
            }
        }
    }
}

GstFlowReturn VideoComposite::detector_new_sample_cb(GstAppSink *sink, gpointer user_data)
{
    auto *ctx = static_cast<DetectorSinkCtx *>(user_data);
    if (!ctx || !ctx->self)
        return GST_FLOW_OK;

    VideoComposite *self = ctx->self;
    if (!self->person_detection_enabled_)
        return GST_FLOW_OK;

    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (!sample)
        return GST_FLOW_OK;

    GstCaps *caps = gst_sample_get_caps(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!caps || !buffer) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    GstVideoInfo vinfo;
    gst_video_info_init(&vinfo);
    if (!gst_video_info_from_caps(&vinfo, caps)) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    GstVideoFrame vframe;
    if (!gst_video_frame_map(&vframe, &vinfo, buffer, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    const int width = GST_VIDEO_INFO_WIDTH(&vinfo);
    const int height = GST_VIDEO_INFO_HEIGHT(&vinfo);
    cv::Mat bgr(height,
                width,
                CV_8UC3,
                GST_VIDEO_FRAME_PLANE_DATA(&vframe, 0),
                GST_VIDEO_FRAME_PLANE_STRIDE(&vframe, 0));

    {
        std::lock_guard<std::mutex> lock(self->detector_frame_mutex_);
        self->detector_latest_frame_ = bgr.clone();
        self->detector_have_new_frame_ = true;
    }
    self->detector_cv_.notify_one();

    gst_video_frame_unmap(&vframe);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

GstPadProbeReturn VideoComposite::imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                               gpointer user_data)
{
    if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER))
        return GST_PAD_PROBE_OK;

    auto *ctx = static_cast<BranchProbeCtx *>(user_data);
    if (!ctx || !ctx->self)
        return GST_PAD_PROBE_OK;

    VideoComposite *self = ctx->self;
    const int i = ctx->branch_index;
    if (i < 0 || i >= self->num_src)
        return GST_PAD_PROBE_OK;

    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf)
        return GST_PAD_PROBE_OK;

    uint64_t frame_ts_us = 0;
    bool have_frame_ts = false;
    bool used_sender_ts = false;
    bool used_pts_fallback = false;
    size_t sender_q_size_after_pop = 0;

    {
        std::lock_guard<std::mutex> lock(self->sender_ts_mutex_);
        auto &q = self->sender_ts_queue_us_[i];

        if (q.size() >= 2) {
            while (q.size() > 2) {
                q.pop_front();
            }
            frame_ts_us = q.front().timestamp_us;
            q.pop_front();
            have_frame_ts = true;
            used_sender_ts = true;
            self->metadata_seen_[i] = true;
        } else if (q.size() == 1) {
            frame_ts_us = q.front().timestamp_us;
            q.pop_front();
            have_frame_ts = true;
            used_sender_ts = true;
            self->metadata_seen_[i] = true;
        }

        sender_q_size_after_pop = q.size();
    }

    if (!have_frame_ts) {
        if (i < (int)self->metadata_seen_.size() &&
            self->metadata_seen_[i] &&
            i < (int)self->last_applied_H_valid_.size() &&
            self->last_applied_H_valid_[i]) {
            frame_ts_us = 0;
            have_frame_ts = false;
            used_pts_fallback = false;
        } else if (GST_BUFFER_PTS_IS_VALID(buf)) {
            frame_ts_us = GST_BUFFER_PTS(buf) / 1000;
            have_frame_ts = true;
            used_pts_fallback = true;
        }
    }

    int64_t query_ts_us = static_cast<int64_t>(frame_ts_us);
    if (i < (int)self->frame_pts_to_imu_offset_us_.size()) {
        query_ts_us += self->frame_pts_to_imu_offset_us_[i];
    }

    uint64_t frame_dt_us = 0;
    if (used_sender_ts && i < (int)self->last_used_frame_ts_us_.size()) {
        uint64_t &last_ts = self->last_used_frame_ts_us_[i];
        if (last_ts != 0 && frame_ts_us > last_ts) {
            frame_dt_us = frame_ts_us - last_ts;
        }
        last_ts = frame_ts_us;
    }

    int idx = i;
    const char *env = getenv("IMU_ALL");
    if (env) {
        char *end = nullptr;
        long v = strtol(env, &end, 10);
        if (end != env && v >= 0 && v < self->num_src) {
            idx = static_cast<int>(v);
        }
    }

    std::array<float,9> H{};
    for (int j = 0; j < 9; ++j)
        H[j] = (j % 4 == 0) ? 1.0f : 0.0f;

    uint64_t best_diff_us = 0;
    bool got_hinv = false;
    bool used_latest_fallback = false;
    bool held_due_to_outlier = false;
    bool reused_last_H_due_to_missing_meta = false;

    if (!used_sender_ts &&
        i < (int)self->metadata_seen_.size() &&
        self->metadata_seen_[i] &&
        i < (int)self->last_applied_H_valid_.size() &&
        self->last_applied_H_valid_[i]) {
        H = self->last_applied_H_[i];
        got_hinv = true;
        reused_last_H_due_to_missing_meta = true;
    }

    if (!got_hinv && self->nodes_ && idx < (int)self->nodes_->size()) {
        if (have_frame_ts && query_ts_us >= 0) {
            got_hinv = (*self->nodes_)[idx]->getHinvAt(
                static_cast<uint64_t>(query_ts_us), H, &best_diff_us);
        }

        if (!got_hinv) {
            auto mat = (*self->nodes_)[idx]->getHinv();
            for (int j = 0; j < 9; ++j)
                H[j] = mat[j];
            got_hinv = true;
            used_latest_fallback = true;
        }
    }

    if (i < (int)self->last_applied_H_valid_.size() &&
        self->last_applied_H_valid_[i] &&
        frame_dt_us > self->frame_dt_outlier_us_) {
        H = self->last_applied_H_[i];
        held_due_to_outlier = true;
    } else if (i < (int)self->last_applied_H_valid_.size() &&
               self->last_applied_H_valid_[i]) {
        const float a = self->homography_blend_alpha_;
        for (int j = 0; j < 9; ++j) {
            H[j] = (1.0f - a) * self->last_applied_H_[i][j] + a * H[j];
        }
    }

    if (i < (int)self->last_applied_H_.size()) {
        self->last_applied_H_[i] = H;
        self->last_applied_H_valid_[i] = true;
    }

    GstStructure *vars = gst_structure_new("uniforms",
                                           "k1", G_TYPE_FLOAT, self->live_k1,
                                           "zoom", G_TYPE_FLOAT, self->live_zoom,
                                           "w", G_TYPE_FLOAT, 1920.0f,
                                           "h", G_TYPE_FLOAT, 1080.0f,
                                           "h00", G_TYPE_FLOAT, H[0],
                                           "h01", G_TYPE_FLOAT, H[1],
                                           "h02", G_TYPE_FLOAT, H[2],
                                           "h10", G_TYPE_FLOAT, H[3],
                                           "h11", G_TYPE_FLOAT, H[4],
                                           "h12", G_TYPE_FLOAT, H[5],
                                           "h20", G_TYPE_FLOAT, H[6],
                                           "h21", G_TYPE_FLOAT, H[7],
                                           "h22", G_TYPE_FLOAT, H[8],
                                           NULL);

    char name[32];
    snprintf(name, sizeof(name), "lens%d", i);
    GstElement *shader = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
    if (shader) {
        g_object_set(shader, "uniforms", vars, NULL);
        gst_object_unref(shader);
    }
    gst_structure_free(vars);

    if (i < (int)self->stab.size() && self->stab[i]) {
        g_object_set(self->stab[i],
                     "translation-x", 0.0f,
                     "translation-y", 0.0f,
                     NULL);
    }

    static uint64_t print_counter = 0;
    if ((++print_counter % 50) == 0) {
        int64_t offset_us = 0;
        if (i < (int)self->frame_pts_to_imu_offset_us_.size()) {
            offset_us = self->frame_pts_to_imu_offset_us_[i];
        }

        g_print("[branch %d] frame_ts_us=%" G_GUINT64_FORMAT
                " frame_dt_us=%" G_GUINT64_FORMAT
                " query_ts_us=%" G_GINT64_FORMAT
                " offset_us=%" G_GINT64_FORMAT
                " best_diff_us=%" G_GUINT64_FORMAT
                " fallback=%d"
                " hold=%d"
                " reuse_last=%d"
                " have_frame_ts=%d"
                " used_sender_ts=%d"
                " used_pts_fallback=%d"
                " sender_q_size=%zu\n",
                i,
                frame_ts_us,
                frame_dt_us,
                query_ts_us,
                offset_us,
                best_diff_us,
                used_latest_fallback ? 1 : 0,
                held_due_to_outlier ? 1 : 0,
                reused_last_H_due_to_missing_meta ? 1 : 0,
                have_frame_ts ? 1 : 0,
                used_sender_ts ? 1 : 0,
                used_pts_fallback ? 1 : 0,
                sender_q_size_after_pop);
    }

    return GST_PAD_PROBE_OK;
}

void *VideoComposite::run_pipeline(gpointer user_data)
{
    VideoComposite *self = static_cast<VideoComposite *>(user_data);
    if (!self) {
        self = s_instance;
    }
    if (!self) {
        g_printerr("run_pipeline: user_data is NULL and no fallback instance\n");
        return NULL;
    }

    g_print("run_pipeline: self=%p\n", (void*)self);

    std::vector<BranchProbeCtx*> probe_ctxs;
    std::vector<DetectorSinkCtx*> detector_ctxs;

    self->pipeline = gst_pipeline_new("video_pipeline");
    if (!self->pipeline) {
        g_printerr("run_pipeline: failed to create pipeline\n");
        return NULL;
    }

    GstElement *mix = gst_element_factory_make("glvideomixer", "mix");
    GstElement *convert = gst_element_factory_make("glcolorconvert", "conv");
    GstElement *tee = gst_element_factory_make("tee", "out_tee");

    GstElement *display_queue = gst_element_factory_make("queue", "display_queue");
    GstElement *fps = gst_element_factory_make("fpsdisplaysink", "fps");
    GstElement *videosink = gst_element_factory_make("glimagesink", "vsink");

    GstElement *record_queue = self->main_record_enabled_ ? gst_element_factory_make("queue", "record_queue") : nullptr;
    GstElement *download = gst_element_factory_make("gldownload", "record_gldownload");
    GstElement *record_split = gst_element_factory_make("tee", "record_split");
    GstElement *record_enc_queue = self->main_record_enabled_ ? gst_element_factory_make("queue", "record_enc_queue") : nullptr;
    GstElement *record_convert = self->main_record_enabled_ ? gst_element_factory_make("videoconvert", "record_convert") : nullptr;
    GstElement *enc = self->main_record_enabled_ ? gst_element_factory_make("x264enc", "record_enc") : nullptr;
    GstElement *parse = self->main_record_enabled_ ? gst_element_factory_make("h264parse", "record_parse_out") : nullptr;
    GstElement *mux = self->main_record_enabled_ ? gst_element_factory_make("matroskamux", "record_mux") : nullptr;
    GstElement *sink = self->main_record_enabled_ ? gst_element_factory_make("filesink", "record_sink") : nullptr;

    GstElement *det_queue = nullptr;
    GstElement *det_convert = nullptr;
    GstElement *det_scale = nullptr;
    GstElement *det_caps = nullptr;
    GstElement *det_sink = nullptr;

    if (self->person_detection_enabled_) {
        det_queue = gst_element_factory_make("queue", "det_queue");
        det_convert = gst_element_factory_make("videoconvert", "det_convert");
        det_scale = gst_element_factory_make("videoscale", "det_scale");
        det_caps = gst_element_factory_make("capsfilter", "det_caps");
        det_sink = gst_element_factory_make("appsink", "det_sink");
    }

    if (!mix || !convert || !tee || !display_queue || !fps || !videosink || !download || !record_split ||
        (self->main_record_enabled_ && (!record_queue || !record_convert || !record_enc_queue || !enc || !parse || !mux || !sink))) {
        g_printerr("run_pipeline: could not create core elements\n");
        return NULL;
    }

    if (self->person_detection_enabled_ && (!det_queue || !det_convert || !det_scale || !det_caps || !det_sink)) {
        g_printerr("run_pipeline: could not create detector side branch\n");
        return NULL;
    }

    const char *record_path_env = std::getenv("GST_RECORD_PATH");
    std::string default_record_path = make_default_record_path();
    const char *record_path = (record_path_env && record_path_env[0] != '\0')
                                ? record_path_env
                                : default_record_path.c_str();

    g_object_set(mix, "background", 1, NULL);
    g_object_set(fps,
                 "video-sink", videosink,
                 "text-overlay", TRUE,
                 "sync", FALSE,
                 NULL);
    if (self->main_record_enabled_) {
        g_object_set(enc,
                     "tune", 0x00000004,
                     "speed-preset", 1,
                     "bitrate", 10000,
                     NULL);
        g_object_set(mux,
                     "streamable", FALSE,
                     NULL);
        g_object_set(sink,
                     "location", record_path,
                     "sync", FALSE,
                     "async", FALSE,
                     NULL);
    }

    gst_bin_add_many(GST_BIN(self->pipeline),
                     mix, convert, tee,
                     display_queue, fps,
                     NULL);
    if (self->main_record_enabled_) {
        gst_bin_add_many(GST_BIN(self->pipeline),
                         record_queue, download, record_split, record_enc_queue, record_convert, enc, parse, mux, sink,
                         NULL);
    } else {
        gst_bin_add_many(GST_BIN(self->pipeline),
                         download, record_split,
                         NULL);
    }

    if (self->person_detection_enabled_) {
        gst_bin_add_many(GST_BIN(self->pipeline),
                         det_queue, det_convert, det_scale, det_caps, det_sink,
                         NULL);

        g_object_set(det_queue,
                     "leaky", 2,
                     "max-size-buffers", 1,
                     "max-size-bytes", 0,
                     "max-size-time", 0,
                     NULL);

        g_object_set(det_sink,
                     "emit-signals", TRUE,
                     "max-buffers", 1,
                     "drop", TRUE,
                     "sync", FALSE,
                     "async", FALSE,
                     "wait-on-eos", FALSE,
                     "enable-last-sample", FALSE,
                     "qos", FALSE,
                     NULL);

        GstCaps *detector_caps = gst_caps_from_string((std::ostringstream()
                                    << "video/x-raw,format=BGR,width=" << self->detector_width_
                                    << ",height=" << self->detector_height_).str().c_str());
        g_object_set(det_caps, "caps", detector_caps, NULL);
        gst_caps_unref(detector_caps);
    }

    if (!gst_element_link_many(mix, convert, tee, NULL)) {
        g_printerr("run_pipeline: failed to link core trunk\n");
        return NULL;
    }

    if (!gst_element_link_many(display_queue, fps, NULL)) {
        g_printerr("run_pipeline: failed to link display branch\n");
        return NULL;
    }

    if (self->main_record_enabled_) {
        if (!gst_element_link_many(record_queue, download, record_split, NULL)) {
            g_printerr("run_pipeline: failed to link record trunk\n");
            return NULL;
        }

        if (!gst_element_link_many(record_enc_queue, record_convert, enc, parse, mux, sink, NULL)) {
            g_printerr("run_pipeline: failed to link record encode branch\n");
            return NULL;
        }
    } else {
        if (!gst_element_link_many(download, record_split, NULL)) {
            g_printerr("run_pipeline: failed to link CPU split trunk\n");
            return NULL;
        }
    }

    if (self->person_detection_enabled_) {
        if (!gst_element_link_many(det_queue, det_convert, det_scale, det_caps, det_sink, NULL)) {
            g_printerr("run_pipeline: failed to link detector branch\n");
            return NULL;
        }
    }

    GstPad *tee_display_pad = gst_element_request_pad_simple(tee, "src_%u");
    GstPad *display_sink_pad = gst_element_get_static_pad(display_queue, "sink");
    GstPad *tee_cpu_pad = gst_element_request_pad_simple(tee, "src_%u");
    GstPad *cpu_sink_pad = gst_element_get_static_pad(self->main_record_enabled_ ? record_queue : download, "sink");

    if (!tee_display_pad || !display_sink_pad || !tee_cpu_pad || !cpu_sink_pad) {
        g_printerr("run_pipeline: failed to get tee/request pads\n");
        if (tee_display_pad) gst_object_unref(tee_display_pad);
        if (display_sink_pad) gst_object_unref(display_sink_pad);
        if (tee_cpu_pad) gst_object_unref(tee_cpu_pad);
        if (cpu_sink_pad) gst_object_unref(cpu_sink_pad);
        return NULL;
    }

    if (gst_pad_link(tee_display_pad, display_sink_pad) != GST_PAD_LINK_OK) {
        g_printerr("run_pipeline: failed to link tee to display branch\n");
        gst_object_unref(tee_display_pad);
        gst_object_unref(display_sink_pad);
        gst_object_unref(tee_cpu_pad);
        gst_object_unref(cpu_sink_pad);
        return NULL;
    }

    if (gst_pad_link(tee_cpu_pad, cpu_sink_pad) != GST_PAD_LINK_OK) {
        g_printerr("run_pipeline: failed to link tee to CPU branch\n");
        gst_object_unref(tee_display_pad);
        gst_object_unref(display_sink_pad);
        gst_object_unref(tee_cpu_pad);
        gst_object_unref(cpu_sink_pad);
        return NULL;
    }

    gst_object_unref(tee_display_pad);
    gst_object_unref(display_sink_pad);
    gst_object_unref(tee_cpu_pad);
    gst_object_unref(cpu_sink_pad);

    if (self->main_record_enabled_) {
        // Split the CPU path so detection cannot interfere with the original live GL display path.
        GstPad *split_rec_pad = gst_element_request_pad_simple(record_split, "src_%u");
        GstPad *rec_sink_pad2 = gst_element_get_static_pad(record_enc_queue, "sink");
        if (!split_rec_pad || !rec_sink_pad2) {
            g_printerr("run_pipeline: failed to get record-split encode pads\n");
            if (split_rec_pad) gst_object_unref(split_rec_pad);
            if (rec_sink_pad2) gst_object_unref(rec_sink_pad2);
            return NULL;
        }
        if (gst_pad_link(split_rec_pad, rec_sink_pad2) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link record split to encode branch\n");
            gst_object_unref(split_rec_pad);
            gst_object_unref(rec_sink_pad2);
            return NULL;
        }
        gst_object_unref(split_rec_pad);
        gst_object_unref(rec_sink_pad2);
    }

    if (self->person_detection_enabled_) {
        GstPad *split_det_pad = gst_element_request_pad_simple(record_split, "src_%u");
        GstPad *det_sink_pad = gst_element_get_static_pad(det_queue, "sink");
        if (!split_det_pad || !det_sink_pad) {
            g_printerr("run_pipeline: failed to get detector split pads\n");
            if (split_det_pad) gst_object_unref(split_det_pad);
            if (det_sink_pad) gst_object_unref(det_sink_pad);
            return NULL;
        }
        if (gst_pad_link(split_det_pad, det_sink_pad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link record split to detector branch\n");
            gst_object_unref(split_det_pad);
            gst_object_unref(det_sink_pad);
            return NULL;
        }
        gst_object_unref(split_det_pad);
        gst_object_unref(det_sink_pad);

        auto *det_ctx = new DetectorSinkCtx();
        det_ctx->self = self;
        detector_ctxs.push_back(det_ctx);
        g_signal_connect(det_sink, "new-sample", G_CALLBACK(VideoComposite::detector_new_sample_cb), det_ctx);
        self->startDetectorThread();

        if (self->detector_preview_enabled_ && self->detector_preview_timer_id_ == 0) {
            self->detector_preview_timer_id_ = g_timeout_add(33, VideoComposite::detector_preview_tick_cb, self);
        }
    }

    if (self->main_record_enabled_) {
        g_print("Recording composite output to: %s\n", record_path);
    } else {
        g_print("Composite output recording disabled (RECORD_OUTPUT=0)\n");
    }

    struct SinkLayout { gint xpos, ypos, width, height; };
    std::vector<SinkLayout> layouts;
    layouts.reserve(self->num_src);
    for (int i = 0; i < self->num_src; ++i) {
        SinkLayout l;
        l.xpos = i * 1920;
        l.ypos = 0;
        l.width = 1920;
        l.height = 1080;
        layouts.push_back(l);
    }

    gint bg_width = (self->num_src > 0 ? self->num_src : 1) * 1920;
    gint bg_height = 1080;

    {
        GstElement *bg_src = gst_element_factory_make("videotestsrc", "bg_src");
        GstElement *bg_caps = gst_element_factory_make("capsfilter", "bg_caps");
        GstElement *bg_glup = gst_element_factory_make("glupload", "bg_glup");
        if (!bg_src || !bg_caps || !bg_glup) {
            g_printerr("run_pipeline: failed to create background elements\n");
            return NULL;
        }
        g_object_set(bg_src, "pattern", 0, "is-live", TRUE, NULL);
        GstCaps *bgcaps = gst_caps_from_string((std::ostringstream() << "video/x-raw,width="
                                                                     << bg_width
                                                                     << ",height="
                                                                     << bg_height
                                                                     << ",framerate=60/1").str().c_str());
        g_object_set(bg_caps, "caps", bgcaps, NULL);
        gst_caps_unref(bgcaps);
        gst_bin_add_many(GST_BIN(self->pipeline), bg_src, bg_caps, bg_glup, NULL);
        if (!gst_element_link_many(bg_src, bg_caps, bg_glup, NULL)) {
            g_printerr("run_pipeline: failed to link background branch\n");
            return NULL;
        }
        GstPad *bg_sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
        GstPad *bg_srcpad = gst_element_get_static_pad(bg_glup, "src");
        g_object_set(bg_sinkpad, "xpos", 0, "ypos", 0, "width", bg_width, "height", bg_height, "zorder", 0, NULL);
        if (gst_pad_link(bg_srcpad, bg_sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link background to mixer\n");
            gst_object_unref(bg_srcpad);
            gst_object_unref(bg_sinkpad);
            return NULL;
        }
        gst_object_unref(bg_srcpad);
        gst_object_unref(bg_sinkpad);
    }

    self->stab.assign(self->num_src, nullptr);

    for (int i = 0; i < self->num_src; ++i) {
        char udpsrc_nm[32], jitter_nm[32], depay_nm[32], parse_nm[32], dec_nm[32];
        char conv_nm[32], glup_nm[32], queue_nm[32], shader_nm[32], stab_nm[32];
        snprintf(udpsrc_nm, sizeof(udpsrc_nm), "udpsrc%d", i);
        snprintf(jitter_nm, sizeof(jitter_nm), "jitter%d", i);
        snprintf(depay_nm, sizeof(depay_nm), "depay%d", i);
        snprintf(parse_nm, sizeof(parse_nm), "parse%d", i);
        snprintf(dec_nm, sizeof(dec_nm), "dec%d", i);
        snprintf(conv_nm, sizeof(conv_nm), "conv%d", i);
        snprintf(glup_nm, sizeof(glup_nm), "glup%d", i);
        snprintf(queue_nm, sizeof(queue_nm), "queue%d", i);
        snprintf(shader_nm, sizeof(shader_nm), "lens%d", i);
        snprintf(stab_nm, sizeof(stab_nm), "stab%d", i);

        GstElement *udpsrc = gst_element_factory_make("udpsrc", udpsrc_nm);
        GstElement *jitter = gst_element_factory_make("rtpjitterbuffer", jitter_nm);
        GstElement *depay  = gst_element_factory_make("rtph264depay", depay_nm);
        GstElement *parse  = gst_element_factory_make("h264parse", parse_nm);
        GstElement *dec    = gst_element_factory_make("avdec_h264", dec_nm);
        GstElement *conv   = gst_element_factory_make("videoconvert", conv_nm);
        GstElement *glup   = gst_element_factory_make("glupload", glup_nm);
        GstElement *shader = gst_element_factory_make("glshader", shader_nm);
        GstElement *stab   = gst_element_factory_make("gltransformation", stab_nm);
        GstElement *queue  = gst_element_factory_make("queue", queue_nm);

        if (!udpsrc || !jitter || !depay || !parse || !dec || !conv || !glup || !shader || !stab || !queue) {
            g_printerr("run_pipeline: failed to create udp branch %d elements\n", i);
            return NULL;
        }

        int port = (i < (int)self->video_ports.size() ? self->video_ports[i] : 0);
        g_object_set(udpsrc, "port", port, NULL);

        GstCaps *rtpcaps = gst_caps_from_string("application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000");
        g_object_set(udpsrc, "caps", rtpcaps, NULL);
        gst_caps_unref(rtpcaps);
        g_object_set(jitter, "latency", 0, "drop-on-latency", TRUE, NULL);

        gst_bin_add_many(GST_BIN(self->pipeline), udpsrc, jitter, depay, parse, dec, conv, glup, shader, stab, queue, NULL);
        if (!gst_element_link_many(udpsrc, jitter, depay, parse, dec, conv, glup, shader, stab, queue, NULL)) {
            g_printerr("run_pipeline: udp branch %d link failed\n", i);
            return NULL;
        }

        g_object_set(shader, "fragment", self->shader_code.c_str(), NULL);
        self->stab[i] = GST_ELEMENT(gst_object_ref(stab));

        GstPad *sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
        const auto &l = layouts[i];
        g_object_set(sinkpad,
                     "xpos", l.xpos,
                     "ypos", l.ypos,
                     "width", l.width,
                     "height", l.height,
                     "zorder", i + 1,
                     NULL);

        GstPad *srcpad = gst_element_get_static_pad(queue, "src");
        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link branch %d to mixer\n", i);
            gst_object_unref(srcpad);
            gst_object_unref(sinkpad);
            return NULL;
        }
        gst_object_unref(srcpad);
        gst_object_unref(sinkpad);

        auto *ctx = new BranchProbeCtx();
        ctx->self = self;
        ctx->branch_index = i;
        probe_ctxs.push_back(ctx);

        GstPad *imu_pad = gst_element_get_static_pad(glup, "src");
        if (imu_pad) {
            gst_pad_add_probe(imu_pad,
                              GST_PAD_PROBE_TYPE_BUFFER,
                              VideoComposite::imu_probe_cb,
                              ctx,
                              NULL);
            gst_object_unref(imu_pad);
        }
    }

    GstBus *bus = gst_element_get_bus(self->pipeline);
    gst_bus_add_watch(bus, bus_call, nullptr);
    gst_object_unref(bus);

    gst_element_set_state(self->pipeline, GST_STATE_PLAYING);
    self->main_loop_ = g_main_loop_new(NULL, FALSE);
    g_print("Running base station. Press Ctrl+C to stop.\n");
    if (self->person_detection_enabled_) {
        g_print("Person detection side branch is ENABLED (%s, %d ms period, %dx%d input)\n",
                self->detector_backend_.c_str(), self->detector_period_ms_, self->detector_width_, self->detector_height_);
        g_print("Live display path is unchanged from the original pipeline\n");
    }
    g_main_loop_run(self->main_loop_);

    self->stopDetectorThread();

    GstBus *shutdown_bus = gst_element_get_bus(self->pipeline);
    gst_element_send_event(self->pipeline, gst_event_new_eos());

    GstMessage *shutdown_msg = gst_bus_timed_pop_filtered(
        shutdown_bus,
        5 * GST_SECOND,
        static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));

    if (!shutdown_msg) {
        g_printerr("run_pipeline: timed out waiting for EOS during shutdown; finalizing file may be incomplete\n");
    } else {
        gst_message_unref(shutdown_msg);
    }
    gst_object_unref(shutdown_bus);

    gst_element_set_state(self->pipeline, GST_STATE_NULL);
    gst_element_get_state(self->pipeline, NULL, NULL, 2 * GST_SECOND);
    for (auto *s : self->stab) {
        if (s)
            gst_object_unref(s);
    }
    gst_object_unref(self->pipeline);
    self->pipeline = nullptr;
    if (self->main_loop_) {
        g_main_loop_unref(self->main_loop_);
        self->main_loop_ = nullptr;
    }
    for (auto *ctx : probe_ctxs)
        delete ctx;
    for (auto *ctx : detector_ctxs)
        delete ctx;
    return NULL;
}

void VideoComposite::setBuoyNodes(std::vector<std::unique_ptr<BuoyNode>> *nodes)
{
    nodes_ = nodes;
}
