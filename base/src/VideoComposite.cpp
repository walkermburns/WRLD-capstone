#include "VideoComposite.h"
#include "MathHelpers.h"
#include "BuoyNode.h"

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/gl/gl.h>
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
#include <algorithm>

using namespace MathHelpers;

VideoComposite *VideoComposite::s_instance = nullptr;

struct BranchProbeCtx {
    VideoComposite *self = nullptr;
    int branch_index = -1;
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


static std::vector<VideoComposite::CameraFeedConfig>
make_feeds_from_legacy_ports(const std::vector<int> &ports)
{
    std::vector<VideoComposite::CameraFeedConfig> feeds;
    feeds.reserve(ports.size());
    for (size_t i = 0; i < ports.size(); ++i) {
        VideoComposite::CameraFeedConfig cfg;
        cfg.name = "camera" + std::to_string(i);
        cfg.video_port = ports[i];
        cfg.metadata_port = ports[i] + 1;
        cfg.imu_node_index = static_cast<int>(i);
        cfg.mount = VideoComposite::CameraMount::Front;
        cfg.layout.xpos = static_cast<int>(i) * 1920;
        cfg.layout.ypos = 0;
        cfg.layout.width = 1920;
        cfg.layout.height = 1080;
        feeds.push_back(cfg);
    }
    return feeds;
}

VideoComposite::VideoComposite(const std::string &shaderPath,
                               const std::vector<int> &ports)
    : VideoComposite(shaderPath, make_feeds_from_legacy_ports(ports))
{
}

VideoComposite::VideoComposite(const std::string &shaderPath,
                               const std::vector<CameraFeedConfig> &feeds)
    : pipeline(nullptr), mix_element(nullptr), main_loop_(nullptr), live_k1(0.3f), live_zoom(1.1f),
      live_w(1920.0f), live_h(1080.0f),
      live_h00(1.0f), live_h01(0.0f), live_h02(0.0f),
      live_h10(0.0f), live_h11(1.0f), live_h12(0.0f),
      live_h20(0.0f), live_h21(0.0f), live_h22(1.0f),
      num_src(0), uniforms(nullptr), stab(), branch_active(), quat_(), nodes_(nullptr)
{
    feeds_ = feeds;
    num_src = static_cast<int>(feeds_.size());
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
}

void VideoComposite::start()
{
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
        const int meta_port = feeds_[i].metadata_port;
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

    int idx = self->feeds_[i].imu_node_index;
    const bool rear_facing = (self->feeds_[i].mount == CameraMount::RearYaw180);

    const char *env = getenv("IMU_ALL");
    if (env) {
        char *end = nullptr;
        long v = strtol(env, &end, 10);
        if (end != env && self->nodes_ && v >= 0 && v < (long)self->nodes_->size()) {
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

    if (!got_hinv && self->nodes_ && idx >= 0 && idx < (int)self->nodes_->size()) {
        if (have_frame_ts && query_ts_us >= 0) {
            got_hinv = (*self->nodes_)[idx]->getHinvAtForCamera(
                static_cast<uint64_t>(query_ts_us), H, rear_facing, &best_diff_us);
        }

        if (!got_hinv) {
            auto mat = (*self->nodes_)[idx]->getHinvForCamera(rear_facing);
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

    GstElement *record_queue = gst_element_factory_make("queue", "record_queue");
    GstElement *download = gst_element_factory_make("gldownload", "record_gldownload");
    GstElement *record_convert = gst_element_factory_make("videoconvert", "record_convert");
    GstElement *record_caps = gst_element_factory_make("capsfilter", "record_caps");
    GstElement *enc = gst_element_factory_make("x264enc", "record_enc");
    GstElement *parse = gst_element_factory_make("h264parse", "record_parse_out");
    GstElement *mux = gst_element_factory_make("matroskamux", "record_mux");
    GstElement *sink = gst_element_factory_make("filesink", "record_sink");

    if (!mix || !convert || !tee || !display_queue || !fps || !videosink ||
        !record_queue || !download || !record_convert || !record_caps || !enc || !parse || !mux || !sink) {
        g_printerr("run_pipeline: could not create core elements\n");
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

    gst_bin_add_many(GST_BIN(self->pipeline),
                     mix, convert, tee,
                     display_queue, fps,
                     record_queue, download, record_convert, record_caps, enc, parse, mux, sink,
                     NULL);

    if (!gst_element_link_many(mix, convert, tee, NULL)) {
        g_printerr("run_pipeline: failed to link core trunk\n");
        return NULL;
    }

    if (!gst_element_link_many(display_queue, fps, NULL)) {
        g_printerr("run_pipeline: failed to link display branch\n");
        return NULL;
    }

    if (!gst_element_link_many(record_queue, download, record_convert, record_caps, enc, parse, mux, sink, NULL)) {
        g_printerr("run_pipeline: failed to link record branch\n");
        return NULL;
    }

    GstPad *tee_display_pad = gst_element_request_pad_simple(tee, "src_%u");
    GstPad *display_sink_pad = gst_element_get_static_pad(display_queue, "sink");
    GstPad *tee_record_pad = gst_element_request_pad_simple(tee, "src_%u");
    GstPad *record_sink_pad = gst_element_get_static_pad(record_queue, "sink");

    if (!tee_display_pad || !display_sink_pad || !tee_record_pad || !record_sink_pad) {
        g_printerr("run_pipeline: failed to get tee/request pads\n");
        if (tee_display_pad) gst_object_unref(tee_display_pad);
        if (display_sink_pad) gst_object_unref(display_sink_pad);
        if (tee_record_pad) gst_object_unref(tee_record_pad);
        if (record_sink_pad) gst_object_unref(record_sink_pad);
        return NULL;
    }

    if (gst_pad_link(tee_display_pad, display_sink_pad) != GST_PAD_LINK_OK) {
        g_printerr("run_pipeline: failed to link tee to display branch\n");
        gst_object_unref(tee_display_pad);
        gst_object_unref(display_sink_pad);
        gst_object_unref(tee_record_pad);
        gst_object_unref(record_sink_pad);
        return NULL;
    }

    if (gst_pad_link(tee_record_pad, record_sink_pad) != GST_PAD_LINK_OK) {
        g_printerr("run_pipeline: failed to link tee to record branch\n");
        gst_object_unref(tee_display_pad);
        gst_object_unref(display_sink_pad);
        gst_object_unref(tee_record_pad);
        gst_object_unref(record_sink_pad);
        return NULL;
    }

    gst_object_unref(tee_display_pad);
    gst_object_unref(display_sink_pad);
    gst_object_unref(tee_record_pad);
    gst_object_unref(record_sink_pad);

    g_print("Recording composite output to: %s\n", record_path);

    struct SinkLayout { gint xpos, ypos, width, height; };
    std::vector<SinkLayout> layouts;
    layouts.reserve(self->num_src);

    gint bg_width = 1;
    gint bg_height = 1;

    for (int i = 0; i < self->num_src; ++i) {
        const auto &cfg = self->feeds_[i];
        SinkLayout l;
        l.xpos = cfg.layout.xpos;
        l.ypos = cfg.layout.ypos;
        l.width = cfg.layout.width;
        l.height = cfg.layout.height;
        layouts.push_back(l);

        bg_width = std::max(bg_width, static_cast<gint>(l.xpos + l.width));
        bg_height = std::max(bg_height, static_cast<gint>(l.ypos + l.height));
    }

    // Force a stable raw-video format and canvas size before x264enc.
    // Without this, the recorder can briefly negotiate a default 1x1 stream
    // before glvideomixer settles on the real composite size, and matroskamux
    // rejects the later caps change.
    {
        std::ostringstream oss;
        oss << "video/x-raw,format=I420,width="
            << bg_width
            << ",height="
            << bg_height
            << ",framerate=60/1";
        GstCaps *rcaps = gst_caps_from_string(oss.str().c_str());
        g_object_set(record_caps, "caps", rcaps, NULL);
        gst_caps_unref(rcaps);
    }

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

        int port = self->feeds_[i].video_port;
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
    g_main_loop_run(self->main_loop_);

    GstBus *shutdown_bus = gst_element_get_bus(self->pipeline);
    gst_element_send_event(self->pipeline, gst_event_new_eos());

    GstMessage *shutdown_msg = gst_bus_timed_pop_filtered(
        shutdown_bus,
        5 * GST_SECOND,
        static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));

    if (!shutdown_msg) {
        g_printerr("run_pipeline: timed out waiting for EOS during shutdown; finalizing file may be incomplete\n");
    } else {
        if (GST_MESSAGE_TYPE(shutdown_msg) == GST_MESSAGE_ERROR) {
            GError *err = nullptr;
            gchar *dbg = nullptr;
            gst_message_parse_error(shutdown_msg, &err, &dbg);
            g_printerr("run_pipeline: shutdown error from %s: %s\n",
                       GST_OBJECT_NAME(shutdown_msg->src),
                       err ? err->message : "unknown error");
            if (dbg) {
                g_printerr("run_pipeline: shutdown debug info: %s\n", dbg);
                g_free(dbg);
            }
            if (err) {
                g_error_free(err);
            }
        }
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
    return NULL;
}

void VideoComposite::setBuoyNodes(std::vector<std::unique_ptr<BuoyNode>> *nodes)
{
    nodes_ = nodes;
}
