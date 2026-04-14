#include "VideoComposite.h"
#include "MathHelpers.h"   // quaternion/matrix math moved out of class
#include "BuoyNode.h"      // need full definition for getQuaternion

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/gl/gl.h>
#include <gst/rtp/rtp.h>    // parse RTP headers and extensions
#include <math.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>      // for std::cerr
#include <cstring>
#include <mutex>

using namespace MathHelpers; // bring helpers into current namespace


// static member definition
VideoComposite *VideoComposite::s_instance = nullptr;

struct BranchProbeCtx {
    VideoComposite *self = nullptr;
    int branch_index = -1;
};

// bus watch callback used in run_pipeline; reports errors/warnings from
// elements (GL shader compile issues typically show up here).
static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer /*user_data*/)
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
    return TRUE; /* keep bus watch alive */
}

// NOTE: quaternion/matrix helpers have been moved to MathHelpers.{h,cpp}
// so we no longer define them here.  VideoComposite.cpp simply includes the
// header and uses the free functions via namespace MathHelpers.

// probe that reads one-byte RFC5285 extension id=1 carrying 64-bit microsecond
// timestamp and stores it for the corresponding branch.
GstPadProbeReturn VideoComposite::rtp_timestamp_probe_cb(GstPad *pad,
                                                         GstPadProbeInfo *info,
                                                         gpointer user_data)
{
    auto *ctx = static_cast<BranchProbeCtx *>(user_data);
    if (!ctx || !ctx->self)
        return GST_PAD_PROBE_OK;

    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf)
        return GST_PAD_PROBE_OK;

    GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
    if (!gst_rtp_buffer_map(buf, GST_MAP_READ, &rtp))
        return GST_PAD_PROBE_OK;

    bool found = false;
    uint64_t ts_us = 0;

    gboolean ext = gst_rtp_buffer_get_extension(&rtp);
    if (ext) {
        guint16 profile = 0;
        gpointer data = nullptr;
        guint wordlen = 0;
        if (gst_rtp_buffer_get_extension_data(&rtp, &profile, &data, &wordlen)) {
            if (profile == 0xBEDE && wordlen * 4 >= 12) {
                guint8 *extdata = static_cast<guint8 *>(data);
                memcpy(&ts_us, extdata + 1, sizeof(ts_us));
                ts_us = GUINT64_FROM_BE(ts_us);
                found = true;
            }
        }
    }

    gst_rtp_buffer_unmap(&rtp);

    {
        std::lock_guard<std::mutex> lock(ctx->self->frameTsMutex_);
        if (ctx->branch_index >= 0 && ctx->branch_index < (int)ctx->self->latest_frame_ts_us_.size()) {
            if (found) {
                ctx->self->latest_frame_ts_us_[ctx->branch_index] = ts_us;
                ctx->self->latest_frame_ts_valid_[ctx->branch_index] = true;
            } else {
                ctx->self->latest_frame_ts_valid_[ctx->branch_index] = false;
            }
        }
    }

    return GST_PAD_PROBE_OK;
}

VideoComposite::VideoComposite(const std::string &shaderPath,
                                   const std::vector<int> &ports)
    : pipeline(nullptr), mix_element(nullptr), live_k1(0.3f), live_zoom(1.1f),
      live_w(1920.0f), live_h(1080.0f),
      live_h00(1.0f), live_h01(0.0f), live_h02(0.0f),
      live_h10(0.0f), live_h11(1.0f), live_h12(0.0f),
      live_h20(0.0f), live_h21(0.0f), live_h22(1.0f),
      // default to four incoming UDP streams; we add a separate background
      // videotestsrc below rather than counting it here.
      num_src(0), uniforms(nullptr), stab(), branch_active(), quat_(), nodes_(nullptr) {
    // copy port list and derive source count
    video_ports = ports;
    num_src = static_cast<int>(video_ports.size());
    stab.assign(num_src, nullptr);
    branch_active.assign(num_src, false);

    latest_frame_ts_us_.assign(num_src, 0);
    latest_frame_ts_valid_.assign(num_src, false);

    // (camera matrix initialization is now handled inside BuoyNode; not
    // needed here)

    // load shader file
    std::ifstream in(shaderPath);
    if (!in) {
        std::string msg = "failed to open shader file '" + shaderPath + "'";
        // log unconditionally so we see the bad path even if the exception is
        // accidentally swallowed further up the call chain.
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
    /* allocate and clear vector to match num_src; entries beyond
       num_src are not used */
    stab.assign(num_src, nullptr);

    // note: nodes_ pointer will be configured by caller afterwards via
    // setBuoyNodes().  we intentionally do not try to access it here since
    // main may not have created the nodes vector yet.

    /* prepare the uniforms structure once and reuse it; include every
       value the shader might reference so that the structure can be updated
       in one place later.  the matrix defaults correspond to identity. */
    // start quaternion data as identity rotation
    float r00 = 1.0f, r01 = 0.0f, r02 = 0.0f;
    float r10 = 0.0f, r11 = 1.0f, r12 = 0.0f;
    float r20 = 0.0f, r21 = 0.0f, r22 = 1.0f;
    float ir00 = r00, ir01 = r10, ir02 = r20;
    float ir10 = r01, ir11 = r11, ir12 = r21;
    float ir20 = r02, ir21 = r12, ir22 = r22;

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

VideoComposite::~VideoComposite() {
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        /* unref only the entries we actually created */
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

void VideoComposite::start() {
    gst_init(nullptr, nullptr);

#ifdef __APPLE__
    // store instance in case gst_macos_main fails to forward our user_data
    s_instance = this;
    gst_macos_main((GstMainFunc)VideoComposite::run_pipeline, 0, nullptr, this);
#else
    run_pipeline(this);
#endif
}

// setter helpers -----------------------------------------------------------

void VideoComposite::setUniforms(float k1,
                                    float zoom,
                                    float w,
                                    float h,
                                    float h00, float h01, float h02,
                                    float h10, float h11, float h12,
                                    float h20, float h21, float h22) {
    live_k1 = k1;
    live_zoom = zoom;
    live_w = w;
    live_h = h;
    live_h00 = h00; live_h01 = h01; live_h02 = h02;
    live_h10 = h10; live_h11 = h11; live_h12 = h12;
    live_h20 = h20; live_h21 = h21; live_h22 = h22;

    // the caller provides the inverse-homography entries; just echo them
    std::cout << "[VideoComposite] homography Hinv:\n"
              << "  " << h00 << " " << h01 << " " << h02 << "\n"
              << "  " << h10 << " " << h11 << " " << h12 << "\n"
              << "  " << h20 << " " << h21 << " " << h22 << "\n";

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
    } else {
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
}

// called by BuoyNode callback; pulls quaternion components out of the
// protobuf message, updates our internal copy, and print them so we know the
// callback is firing.
void VideoComposite::updateQuaternion(const buoy_proto::IMU_proto &msg) {
    // this helper is now primarily kept for legacy/testing; most clients
    // will update quat_ via the nodes_ vector instead of a callback.
    quat_.w = msg.quat_w();
    quat_.x = msg.quat_x();
    quat_.y = msg.quat_y();
    quat_.z = msg.quat_z();
}

// static callbacks ----------------------------------------------------------

/* pad probe that runs for buffers entering an individual shader branch.
   It uses the per-branch video sender timestamp to fetch the closest IMU-derived
   inverse homography from the matching BuoyNode and applies it immediately to
   that branch's shader. */
GstPadProbeReturn VideoComposite::imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                               gpointer user_data) {
    if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER))
        return GST_PAD_PROBE_OK;

    auto *ctx = static_cast<BranchProbeCtx *>(user_data);
    if (!ctx || !ctx->self)
        return GST_PAD_PROBE_OK;

    VideoComposite *self = ctx->self;
    const int i = ctx->branch_index;
    if (i < 0 || i >= self->num_src)
        return GST_PAD_PROBE_OK;

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

    uint64_t frame_ts_us = 0;
    bool have_frame_ts = false;
    {
        std::lock_guard<std::mutex> lock(self->frameTsMutex_);
        if (i < (int)self->latest_frame_ts_us_.size()) {
            have_frame_ts = self->latest_frame_ts_valid_[i];
            frame_ts_us = self->latest_frame_ts_us_[i];
        }
    }

    uint64_t best_diff_us = 0;
    bool got_hinv = false;
    if (self->nodes_ && idx < (int)self->nodes_->size()) {
        if (have_frame_ts) {
            got_hinv = (*self->nodes_)[idx]->getHinvAt(frame_ts_us, H, &best_diff_us);
        }
        if (!got_hinv) {
            auto mat = (*self->nodes_)[idx]->getHinv();
            for (int j = 0; j < 9; ++j)
                H[j] = mat[j];
            got_hinv = true;
        }
    }

    // for (int j = 0; j < 9; ++j)
    //     H[j] = (j % 4 == 0) ? 1.0f : 0.0f;

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

    char name[16];
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
    if ((++print_counter % 240) == 0) {
        g_print("[branch %d] frame_ts_us=%" G_GUINT64_FORMAT
        " best_diff_us=%" G_GUINT64_FORMAT
        " have_frame_ts=%d\n",
        i, frame_ts_us, best_diff_us, have_frame_ts ? 1 : 0);
    }

    return GST_PAD_PROBE_OK;
}

void *VideoComposite::run_pipeline(gpointer user_data) {
    VideoComposite *self = static_cast<VideoComposite *>(user_data);
    if (!self) {
        self = s_instance;
    }
    if (!self) {
        g_printerr("run_pipeline: user_data is NULL and no fallback instance\n");
        return NULL;
    }

    g_print("run_pipeline: self=%p\n", (void*)self);

    GMainLoop *loop = nullptr;
    std::vector<BranchProbeCtx*> probe_ctxs;

    self->pipeline = gst_pipeline_new("video_pipeline");
    if (!self->pipeline) {
        g_printerr("run_pipeline: failed to create pipeline\n");
        return NULL;
    }

    GstElement *mix = gst_element_factory_make("glvideomixer", "mix");
    GstElement *convert = gst_element_factory_make("glcolorconvert", "conv");
    GstElement *fps = gst_element_factory_make("fpsdisplaysink", "fps");
    GstElement *videosink = gst_element_factory_make("glimagesink", "vsink");

    if (!mix || !convert || !fps || !videosink) {
        g_printerr("run_pipeline: could not create core elements\n");
        return NULL;
    }

    g_object_set(mix, "background", 1, NULL);
    g_object_set(fps,
                 "video-sink", videosink,
                 "text-overlay", TRUE,
                 "sync", TRUE,
                 NULL);

    gst_bin_add_many(GST_BIN(self->pipeline), mix, convert, fps, NULL);
    if (!gst_element_link_many(mix, convert, fps, NULL)) {
        g_printerr("run_pipeline: failed to link core elements\n");
        return NULL;
    }

    struct SinkLayout { gint xpos, ypos, width, height; };
    std::vector<SinkLayout> layouts;
    layouts.reserve(self->num_src);
    for (int i = 0; i < self->num_src; ++i) {
        SinkLayout l;
        l.xpos   = i * 1920;
        l.ypos   = 0;
        l.width  = 1920;
        l.height = 1080;
        layouts.push_back(l);
    }

    gint bg_width  = (self->num_src > 0 ? self->num_src : 1) * 1920;
    gint bg_height = 1080;

    /* --- background branch --- */
    {
        GstElement *bg_src  = gst_element_factory_make("videotestsrc", "bg_src");
        GstElement *bg_caps = gst_element_factory_make("capsfilter", "bg_caps");
        GstElement *bg_glup = gst_element_factory_make("glupload", "bg_glup");

        if (!bg_src || !bg_caps || !bg_glup) {
            g_printerr("run_pipeline: failed to create background elements\n");
            return NULL;
        }

        g_object_set(bg_src, "pattern", 0, "is-live", TRUE, NULL);

        GstCaps *bgcaps = gst_caps_from_string(
            (std::ostringstream() << "video/x-raw,width=" << bg_width
                                  << ",height=" << bg_height
                                  << ",framerate=60/1").str().c_str());
        g_object_set(bg_caps, "caps", bgcaps, NULL);
        gst_caps_unref(bgcaps);

        gst_bin_add_many(GST_BIN(self->pipeline), bg_src, bg_caps, bg_glup, NULL);
        if (!gst_element_link_many(bg_src, bg_caps, bg_glup, NULL)) {
            g_printerr("run_pipeline: failed to link background branch\n");
            return NULL;
        }

        GstPad *bg_sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
        if (!bg_sinkpad) {
            g_printerr("run_pipeline: could not get mixer pad for background\n");
            return NULL;
        }

        g_object_set(bg_sinkpad,
                     "xpos", 0,
                     "ypos", 0,
                     "width", bg_width,
                     "height", bg_height,
                     "zorder", 0,
                     NULL);

        GstPad *bg_srcpad = gst_element_get_static_pad(bg_glup, "src");
        if (gst_pad_link(bg_srcpad, bg_sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link background to mixer\n");
            gst_object_unref(bg_srcpad);
            gst_object_unref(bg_sinkpad);
            return NULL;
        }

        gst_object_unref(bg_srcpad);
        gst_object_unref(bg_sinkpad);
    }

    /* --- UDP source branches --- */
    self->stab.assign(self->num_src, nullptr);

    for (int i = 0; i < self->num_src; ++i) {
        char udpsrc_nm[32], jitter_nm[32], depay_nm[32], parse_nm[32], dec_nm[32];
        char conv_nm[32], rate_nm[32], capsf_nm[32], glup_nm[32], queue_nm[32];
        char shader_nm[32], stab_nm[32];

        snprintf(udpsrc_nm, sizeof(udpsrc_nm), "udpsrc%d", i);
        snprintf(jitter_nm, sizeof(jitter_nm), "jitter%d", i);
        snprintf(depay_nm, sizeof(depay_nm), "depay%d", i);
        snprintf(parse_nm, sizeof(parse_nm), "parse%d", i);
        snprintf(dec_nm, sizeof(dec_nm), "dec%d", i);
        snprintf(conv_nm, sizeof(conv_nm), "conv%d", i);
        snprintf(rate_nm, sizeof(rate_nm), "rate%d", i);
        snprintf(capsf_nm, sizeof(capsf_nm), "capsf%d", i);
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
        GstElement *rate   = gst_element_factory_make("videorate", rate_nm);
        GstElement *capsf  = gst_element_factory_make("capsfilter", capsf_nm);
        GstElement *glup   = gst_element_factory_make("glupload", glup_nm);
        GstElement *shader = gst_element_factory_make("glshader", shader_nm);
        GstElement *stab   = gst_element_factory_make("gltransformation", stab_nm);
        GstElement *queue  = gst_element_factory_make("queue", queue_nm);

        if (!udpsrc || !jitter || !depay || !parse || !dec || !conv ||
            !rate || !capsf || !glup || !shader || !stab || !queue) {
            g_printerr("run_pipeline: failed to create udp branch %d elements\n", i);
            return NULL;
        }

        int port = (i < (int)self->video_ports.size() ? self->video_ports[i] : 0);
        if (port > 0) {
            g_object_set(udpsrc, "port", port, NULL);
        } else {
            g_printerr("run_pipeline: invalid port for branch %d\n", i);
        }

        GstCaps *rtpcaps = gst_caps_from_string(
            "application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000");
        g_object_set(udpsrc, "caps", rtpcaps, NULL);
        gst_caps_unref(rtpcaps);

        g_object_set(jitter, "latency", 0, "drop-on-latency", TRUE, NULL);

        GstCaps *caps2 = gst_caps_from_string("video/x-raw,framerate=60/1");
        g_object_set(capsf, "caps", caps2, NULL);
        gst_caps_unref(caps2);

        gst_bin_add_many(GST_BIN(self->pipeline),
                         udpsrc, jitter, depay, parse, dec,
                         conv, rate, capsf, glup, shader, stab, queue, NULL);

        if (!gst_element_link_many(udpsrc, jitter, depay, parse, dec,
                                   conv, rate, capsf, glup, shader, stab, queue, NULL)) {
            g_printerr("run_pipeline: udp branch %d link failed\n", i);
            return NULL;
        }

        g_object_set(shader, "fragment", self->shader_code.c_str(), NULL);

        self->stab[i] = GST_ELEMENT(gst_object_ref(stab));

        GstPad *sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
        if (!sinkpad) {
            g_printerr("run_pipeline: could not get mixer pad for branch %d\n", i);
            return NULL;
        }

        const SinkLayout &l = layouts[i];
        g_object_set(sinkpad,
                     "xpos", l.xpos,
                     "ypos", l.ypos,
                     "width", l.width,
                     "height", l.height,
                     "zorder", i + 1,
                     NULL);

        GstPad *srcpad = gst_element_get_static_pad(queue, "src");
        if (!srcpad) {
            g_printerr("run_pipeline: failed to get queue src pad for branch %d\n", i);
            gst_object_unref(sinkpad);
            return NULL;
        }

        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link branch %d to mixer\n", i);
            gst_object_unref(srcpad);
            gst_object_unref(sinkpad);
            return NULL;
        }

        gst_object_unref(srcpad);
        gst_object_unref(sinkpad);

        /* per-branch probes */
        auto *ctx = new BranchProbeCtx();
        ctx->self = self;
        ctx->branch_index = i;
        probe_ctxs.push_back(ctx);

        {
            GstPad *rtp_pad = gst_element_get_static_pad(jitter, "sink");
            if (rtp_pad) {
                gst_pad_add_probe(rtp_pad,
                                  GST_PAD_PROBE_TYPE_BUFFER,
                                  VideoComposite::rtp_timestamp_probe_cb,
                                  ctx,
                                  NULL);
                gst_object_unref(rtp_pad);
            } else {
                g_printerr("run_pipeline: failed to get jitter sink pad for branch %d\n", i);
            }
        }

        {
            GstPad *imu_pad = gst_element_get_static_pad(glup, "src");
            if (imu_pad) {
                gst_pad_add_probe(imu_pad,
                                  GST_PAD_PROBE_TYPE_BUFFER,
                                  VideoComposite::imu_probe_cb,
                                  ctx,
                                  NULL);
                gst_object_unref(imu_pad);
            } else {
                g_printerr("run_pipeline: failed to get glup src pad for branch %d\n", i);
            }
        }
    }

    {
        GstBus *bus = gst_element_get_bus(self->pipeline);
        gst_bus_add_watch(bus, bus_call, nullptr);
        gst_object_unref(bus);
    }

    gst_element_set_state(self->pipeline, GST_STATE_PLAYING);

    loop = g_main_loop_new(NULL, FALSE);
    g_print("Running base station. Press Ctrl+C to stop.\n");
    g_main_loop_run(loop);

    gst_element_set_state(self->pipeline, GST_STATE_NULL);

    for (auto *s : self->stab) {
        if (s)
            gst_object_unref(s);
    }

    gst_object_unref(self->pipeline);
    g_main_loop_unref(loop);

    for (auto *ctx : probe_ctxs)
        delete ctx;

    return NULL;
}


// ---------------------------------------------------------------------------
// new methods

void VideoComposite::setBuoyNodes(std::vector<std::unique_ptr<BuoyNode>> *nodes) {
    nodes_ = nodes;
}