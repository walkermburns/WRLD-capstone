#include "VideoComposite.h"
#include "MathHelpers.h"   // quaternion/matrix math moved out of class
#include "BuoyNode.h"      // need full definition for getQuaternion

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/gl/gl.h>
#include <gst/rtp/rtp.h>    // parse RTP headers and extensions
#include <glib-unix.h>
#include <math.h>
#include <stdio.h>
#include <signal.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>      // for std::cerr

using namespace MathHelpers; // bring helpers into current namespace


// static member definition
VideoComposite *VideoComposite::s_instance = nullptr;

struct PipelineRunContext {
    GMainLoop *loop;
    GstElement *pipeline;
    gboolean eos_requested;
};

static gboolean request_eos_and_stop(gpointer user_data)
{
    PipelineRunContext *ctx = static_cast<PipelineRunContext *>(user_data);
    if (!ctx || !ctx->pipeline)
        return G_SOURCE_REMOVE;

    if (!ctx->eos_requested) {
        ctx->eos_requested = TRUE;
        g_print("[gst] Signal received; sending EOS so MP4 can be finalized...\n");
        gst_element_send_event(ctx->pipeline, gst_event_new_eos());
    }
    // Keep the source installed; we remove it explicitly during shutdown.
    return G_SOURCE_CONTINUE;
}

// bus watch callback used in run_pipeline; reports errors/warnings from
// elements (GL shader compile issues typically show up here).
static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer user_data)
{
    PipelineRunContext *ctx = static_cast<PipelineRunContext *>(user_data);
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
        if (ctx && ctx->loop) {
            g_main_loop_quit(ctx->loop);
        }
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
    case GST_MESSAGE_EOS:
        g_print("[gst] EOS received; stopping main loop\n");
        if (ctx && ctx->loop) {
            g_main_loop_quit(ctx->loop);
        }
        break;
    default:
        break;
    }
    return TRUE; /* keep bus watch alive */
}

// NOTE: quaternion/matrix helpers have been moved to MathHelpers.{h,cpp}
// so we no longer define them here.  VideoComposite.cpp simply includes the
// header and uses the free functions via namespace MathHelpers.

// probe that reads one-byte RFC5285 extension id=1 carrying 64-bit microsecond
// timestamp and prints it alongside sequence number.
static GstPadProbeReturn rtp_timestamp_probe(GstPad *pad, GstPadProbeInfo *info,
                                             gpointer /*user_data*/)
{
    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf)
        return GST_PAD_PROBE_OK;

    GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
    if (gst_rtp_buffer_map(buf, GST_MAP_READ, &rtp)) {
        guint16 seq = gst_rtp_buffer_get_seq(&rtp);
        guint32 ts = gst_rtp_buffer_get_timestamp(&rtp);
        gboolean ext = gst_rtp_buffer_get_extension(&rtp);
        if (ext) {
            guint16 profile;
            gpointer data;
            guint wordlen;
            if (gst_rtp_buffer_get_extension_data(&rtp, &profile, &data, &wordlen)) {
                if (profile == 0xBEDE && wordlen * 4 >= 12) {
                    guint8 *extdata = static_cast<guint8 *>(data);
                    guint64 ts_us;
                    memcpy(&ts_us, extdata + 1, sizeof(ts_us));
                    ts_us = GUINT64_FROM_BE(ts_us);
                    g_print("[rtp] seq=%u ts=%u ts_us=%" G_GUINT64_FORMAT "\n",
                            seq, ts, ts_us);
                }
            }
        }
        gst_rtp_buffer_unmap(&rtp);
    }
    return GST_PAD_PROBE_OK;
}

VideoComposite::VideoComposite(const std::string &shaderPath,
                                   const std::vector<int> &ports)
    : pipeline(nullptr), mix_element(nullptr), live_k1(0.3f), live_zoom(1.1f),
      live_w(0.0f), live_h(0.0f),
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

    // initialize compositor-side matrix smoothing state to identity so the
    // first applied homography blends in from a well-defined baseline.
    last_applied_hinv_.assign(num_src, std::array<float,9>{});
    for (int i = 0; i < num_src; ++i) {
        for (int j = 0; j < 9; ++j)
            last_applied_hinv_[i][j] = (j % 4 == 0) ? 1.0f : 0.0f;
    }

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

     /* Prepare uniforms once and reuse them every frame.  Defaults are
         identity transform + current distortion settings. */

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

/* pad probe that runs once for each buffer passing through the mixer source.
       We use it to fetch the latest orientation from the first buoy node (if one
       has been provided) and propagate uniforms/transforms. */
GstPadProbeReturn VideoComposite::imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                               gpointer user_data) {
    if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER))
        return GST_PAD_PROBE_OK;
    VideoComposite *self = static_cast<VideoComposite *>(user_data);

    // precompute Hinv for each source (identity if missing).  also
    // respect IMU_ALL environment variable which forces a single node's
    // matrix to be applied to all shaders.
    std::vector<std::array<float,9>> hinvs(self->num_src);

    // check IMU_ALL once per call; console might set/unset at runtime
    int forceIndex = -1;
    const char *env = getenv("IMU_ALL");
    if (env) {
        char *end;
        long v = strtol(env, &end, 10);
        if (end != env && v >= 0 && v < self->num_src) {
            forceIndex = static_cast<int>(v);
        }
    }

    for (int i = 0; i < self->num_src; ++i) {
        int idx = (forceIndex >= 0) ? forceIndex : i;

        // pull the latest homography from the corresponding IMU node
        std::array<float,9> target;
        if (self->nodes_ && idx < (int)self->nodes_->size()) {
            auto mat = (*self->nodes_)[idx]->getHinv();
            for (int j = 0; j < 9; ++j)
                target[j] = mat[j];
        } else {
            for (int j = 0; j < 9; ++j)
                target[j] = (j % 4 == 0) ? 1.0f : 0.0f;
        }

        // compositor-side smoothing: low-pass the final Hinv sent to the shader.
        // this is intentionally separate from the IMU-side corr_smooth_alpha,
        // which smooths roll/pitch before the homography is built.
        std::array<float,9> filt = self->last_applied_hinv_[i];
        for (int j = 0; j < 9; ++j) {
            filt[j] += self->shader_hinv_alpha_ * (target[j] - filt[j]);
        }

        self->last_applied_hinv_[i] = filt;
        hinvs[i] = filt;
    }

    // set the uniforms (keep k1/zoom constant for now)
    float k1 = self->live_k1;
    float zoom = self->live_zoom;
    // camera dimensions are fixed (hard-coded to match BuoyNode defaults)

    /* push updated uniforms into each shader element individually, so that
       each branch uses its own matrix.  we update the shared gst_structure
       for convenience before applying to each shader. */
    for (int i = 0; i < self->num_src; i++) {
        // update homography entries
        const auto &H = hinvs[i];
        gst_structure_set(self->uniforms,
                          "k1", G_TYPE_FLOAT, k1,
                          "zoom", G_TYPE_FLOAT, zoom,
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
            g_object_set(shader, "uniforms", self->uniforms, NULL);
            gst_object_unref(shader);
        }
    }

    /* keep stabilization elements centered (no translation) */
    for (int i = 0; i < self->num_src && i < (int)self->stab.size(); i++) {
        if (self->stab[i]) {
            g_object_set(self->stab[i],
                         "translation-x", 0.0f,
                         "translation-y", 0.0f,
                         NULL);
        }
    }
    return GST_PAD_PROBE_OK;
}

bool VideoComposite::buildStage1And2OutputBranches(PipelineElements &elems) {
    // ------------------------------------------------------------------
    // Stage 1: compositor output and fan-out point
    // Pipeline: glvideomixer -> glcolorconvert -> tee
    elems.mix = gst_element_factory_make("glvideomixer", "mix");
    elems.convert = gst_element_factory_make("glcolorconvert", "conv");
    elems.tee = gst_element_factory_make("tee", "split");

    // ------------------------------------------------------------------
    // Stage 2A: display branch (interactive preview with FPS overlay)
    // Pipeline: tee -> queue -> fpsdisplaysink -> glimagesink
    elems.display_queue = gst_element_factory_make("queue", "display_queue");
    elems.fps = gst_element_factory_make("fpsdisplaysink", "fps");
    elems.videosink = gst_element_factory_make("glimagesink", "vsink");

    // ------------------------------------------------------------------
    // Stage 2B: recording branch (mp4 output)
    // Pipeline: tee -> queue -> gldownload -> videoconvert -> x264enc
    //          -> h264parse -> mp4mux -> filesink
    elems.record_queue = gst_element_factory_make("queue", "record_queue");
    elems.download = gst_element_factory_make("gldownload", "record_download");
    elems.record_convert = gst_element_factory_make("videoconvert", "record_convert");
    elems.record_enc = gst_element_factory_make("x264enc", "record_enc");
    elems.record_parse = gst_element_factory_make("h264parse", "record_parse");
    elems.record_mux = gst_element_factory_make("mp4mux", "record_mux");
    elems.record_sink = gst_element_factory_make("filesink", "record_sink");

    if (!elems.mix || !elems.convert || !elems.tee || !elems.display_queue ||
        !elems.fps || !elems.videosink || !elems.record_queue || !elems.download ||
        !elems.record_convert || !elems.record_enc || !elems.record_parse ||
        !elems.record_mux || !elems.record_sink) {
        g_printerr("run_pipeline: could not create core elements\n");
        return false;
    }

    mix_element = elems.mix;

    g_object_set(elems.mix, "background", 1, NULL);
    g_object_set(elems.fps,
                 "video-sink", elems.videosink,
                 "text-overlay", TRUE,
                 "sync", TRUE,
                 NULL);

    // Recording quality/latency knobs to tune first when changing behavior.
    g_object_set(elems.record_enc,
                 "speed-preset", 1,   // ultrafast
                 "tune", 0x00000004,  // zerolatency
                 "key-int-max", 60,
                 NULL);
    g_object_set(elems.record_mux, "faststart", TRUE, NULL);

    // Change this naming pattern if you want deterministic output paths.
    char record_path[128];
    guint64 now_us = g_get_real_time();
    snprintf(record_path, sizeof(record_path), "video_composite_%" G_GUINT64_FORMAT ".mp4", now_us);
    g_object_set(elems.record_sink, "location", record_path, NULL);
    g_print("[record] writing composite output to %s\n", record_path);

    /* Add all top-level branch elements.  videosink is not added directly
       because fpsdisplaysink owns it through its "video-sink" property. */
    gst_bin_add_many(GST_BIN(pipeline),
                     elems.mix, elems.convert, elems.tee,
                     elems.display_queue, elems.fps,
                     elems.record_queue, elems.download, elems.record_convert,
                     elems.record_enc, elems.record_parse,
                     elems.record_mux, elems.record_sink,
                     NULL);

    if (!gst_element_link_many(elems.mix, elems.convert, elems.tee, NULL)) {
        g_printerr("run_pipeline: failed to link core elements\n");
        return false;
    }
    if (!gst_element_link_many(elems.display_queue, elems.fps, NULL)) {
        g_printerr("run_pipeline: failed to link display branch\n");
        return false;
    }
    if (!gst_element_link_many(elems.record_queue, elems.download,
                               elems.record_convert, elems.record_enc,
                               elems.record_parse, elems.record_mux,
                               elems.record_sink, NULL)) {
        g_printerr("run_pipeline: failed to link record branch\n");
        return false;
    }

    // Dynamic tee links: one src pad per branch.
    GstPad *tee_display_src = gst_element_request_pad_simple(elems.tee, "src_%u");
    GstPad *display_sink = gst_element_get_static_pad(elems.display_queue, "sink");
    if (!tee_display_src || !display_sink ||
        gst_pad_link(tee_display_src, display_sink) != GST_PAD_LINK_OK) {
        g_printerr("run_pipeline: failed to connect tee to display branch\n");
        if (tee_display_src) gst_object_unref(tee_display_src);
        if (display_sink) gst_object_unref(display_sink);
        return false;
    }
    gst_object_unref(tee_display_src);
    gst_object_unref(display_sink);

    GstPad *tee_record_src = gst_element_request_pad_simple(elems.tee, "src_%u");
    GstPad *record_sinkpad = gst_element_get_static_pad(elems.record_queue, "sink");
    if (!tee_record_src || !record_sinkpad ||
        gst_pad_link(tee_record_src, record_sinkpad) != GST_PAD_LINK_OK) {
        g_printerr("run_pipeline: failed to connect tee to record branch\n");
        if (tee_record_src) gst_object_unref(tee_record_src);
        if (record_sinkpad) gst_object_unref(record_sinkpad);
        return false;
    }
    gst_object_unref(tee_record_src);
    gst_object_unref(record_sinkpad);
    return true;
}

bool VideoComposite::buildStage3LayoutAndProbe(GstElement *mix,
                                               std::vector<SinkLayout> &layouts,
                                               gint &bg_width,
                                               gint &bg_height) {
    // ------------------------------------------------------------------
    // Stage 3: layout plan for glvideomixer sink pads
    // Edit this block to change source placement/size on the output canvas.
    layouts.clear();
    layouts.reserve(num_src);
    for (int i = 0; i < num_src; ++i) {
        SinkLayout l;
        l.xpos = i * 1920;
        l.ypos = 0;
        l.width = 1920;
        l.height = 1080;
        layouts.push_back(l);
    }

    // Background canvas dimensions should cover all source tiles.
    bg_width = (num_src > 0 ? num_src : 1) * 1920;
    bg_height = 1080;

    // Probe mixer output so shader uniforms can be refreshed every frame.
    GstPad *probe_pad = gst_element_get_static_pad(mix, "src");
    if (probe_pad) {
        gst_pad_add_probe(probe_pad, GST_PAD_PROBE_TYPE_BUFFER,
                          imu_probe_cb, this, NULL);
        gst_object_unref(probe_pad);
    }
    return true;
}

bool VideoComposite::buildStage4BackgroundBranch(GstElement *mix,
                                                 gint bg_width,
                                                 gint bg_height) {
    // ------------------------------------------------------------------
    // Stage 4: background branch (zorder 0)
    // Pipeline: videotestsrc -> capsfilter -> glupload -> mixer sink pad
    GstElement *bg_src = gst_element_factory_make("videotestsrc", "bg_src");
    GstElement *bg_caps = gst_element_factory_make("capsfilter", "bg_caps");
    GstElement *bg_glup = gst_element_factory_make("glupload", "bg_glup");
    if (!bg_src || !bg_caps || !bg_glup) {
        g_printerr("run_pipeline: failed to create background elements\n");
        return false;
    }

    // Change pattern here if you want a non-default background.
    g_object_set(bg_src, "pattern", 0, "is-live", TRUE, NULL);
    GstCaps *bgcaps = gst_caps_from_string(
        (std::ostringstream() << "video/x-raw,width=" << bg_width
         << ",height=" << bg_height << ",framerate=60/1").str().c_str());
    g_object_set(bg_caps, "caps", bgcaps, NULL);
    gst_caps_unref(bgcaps);

    gst_bin_add_many(GST_BIN(pipeline), bg_src, bg_caps, bg_glup, NULL);
    if (!gst_element_link_many(bg_src, bg_caps, bg_glup, NULL)) {
        g_printerr("run_pipeline: failed to link background branch\n");
        return false;
    }

    GstPad *bg_sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
    if (!bg_sinkpad) {
        g_printerr("run_pipeline: could not get mixer pad for background\n");
        return false;
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
        return false;
    }
    gst_object_unref(bg_srcpad);
    gst_object_unref(bg_sinkpad);
    return true;
}

bool VideoComposite::buildStage5UdpSourceBranches(
    GstElement *mix,
    const std::vector<SinkLayout> &layouts) {
    // ------------------------------------------------------------------
    // Stage 5: one branch per UDP video source
    // Pipeline (per source): udpsrc -> jitter -> depay -> parse -> decode
    //                       -> videoconvert -> videorate -> caps -> glupload
    //                       -> glshader -> gltransformation -> queue -> mixer
    for (int i = 0; i < num_src; ++i) {
        GstElement *udpsrc = gst_element_factory_make("udpsrc", NULL);
        GstElement *jitter = gst_element_factory_make("rtpjitterbuffer", NULL);
        GstElement *depay = gst_element_factory_make("rtph264depay", NULL);
        if (jitter) {
            GstPad *j_src = gst_element_get_static_pad(jitter, "src");
            if (j_src) {
                gst_pad_add_probe(j_src, GST_PAD_PROBE_TYPE_BUFFER,
                                  rtp_timestamp_probe, NULL, NULL);
                gst_object_unref(j_src);
            }
        }
        GstElement *parse = gst_element_factory_make("h264parse", NULL);
        GstElement *dec = gst_element_factory_make("avdec_h264", NULL);
        GstElement *conv = gst_element_factory_make("videoconvert", NULL);
        GstElement *rate = gst_element_factory_make("videorate", NULL);
        GstElement *capsf = gst_element_factory_make("capsfilter", NULL);
        GstElement *glup = gst_element_factory_make("glupload", NULL);
        char shader_nm[16], stab_nm[16];
        snprintf(shader_nm, sizeof(shader_nm), "lens%d", i);
        snprintf(stab_nm, sizeof(stab_nm), "stab%d", i);
        GstElement *shader = gst_element_factory_make("glshader", shader_nm);
        GstElement *stab_elem = gst_element_factory_make("gltransformation", stab_nm);
        GstElement *queue = gst_element_factory_make("queue", NULL);

        if (!udpsrc || !jitter || !depay || !parse || !dec || !conv ||
            !rate || !capsf || !glup || !shader || !stab_elem || !queue) {
            g_printerr("run_pipeline: failed to create udp branch %d elements\n", i);
            return false;
        }

        int port = (i < (int)video_ports.size() ? video_ports[i] : 0);
        if (port > 0) {
            g_object_set(udpsrc, "port", port, NULL);
        } else {
            g_printerr("run_pipeline: invalid port for branch %d\n", i);
        }

        GstCaps *rtpcaps = gst_caps_from_string(
            "application/x-rtp,media=video,encoding-name=H264,"
            "payload=96,clock-rate=90000");
        g_object_set(udpsrc, "caps", rtpcaps, NULL);
        gst_caps_unref(rtpcaps);
        g_object_set(jitter, "latency", 0, "drop-on-latency", TRUE, NULL);

        GstCaps *caps2 = gst_caps_from_string("video/x-raw,framerate=60/1");
        g_object_set(capsf, "caps", caps2, NULL);
        gst_caps_unref(caps2);

        gst_bin_add_many(GST_BIN(pipeline),
                         udpsrc, jitter, depay, parse, dec,
                         conv, rate, capsf, glup, shader, stab_elem, queue, NULL);

        if (!gst_element_link_many(udpsrc, jitter, depay, parse, dec,
                                   conv, rate, capsf, glup, shader, stab_elem,
                                   queue, NULL)) {
            g_printerr("run_pipeline: udp branch %d link failed\n", i);
            return false;
        }

        GstPad *sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
        if (!sinkpad) {
            g_printerr("run_pipeline: could not get mixer pad for branch %d\n", i);
            return false;
        }

        const SinkLayout &l = layouts[i];
        g_object_set(sinkpad,
                     "xpos", l.xpos,
                     "ypos", l.ypos,
                     "width", l.width,
                     "height", l.height,
                     NULL);

        GstPad *srcpad = gst_element_get_static_pad(queue, "src");
        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link branch %d to mixer\n", i);
            return false;
        }
        gst_object_unref(srcpad);
        g_object_set(sinkpad, "zorder", i + 1, NULL);
        gst_object_unref(sinkpad);
    }
    return true;
}

void VideoComposite::buildStage6ConfigureShadersAndStabCache() {
    // ------------------------------------------------------------------
    // Stage 6: post-build configuration and runtime loop
    // Configure shader source and cache stabilization elements by name.
    for (int i = 0; i < num_src; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader_elem = gst_bin_get_by_name(GST_BIN(pipeline), name);
        if (shader_elem) {
            g_object_set(shader_elem, "fragment", shader_code.c_str(), NULL);
            gst_object_unref(shader_elem);
        }
    }

    stab.assign(num_src, nullptr);
    for (int i = 0; i < num_src; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "stab%d", i);
        stab[i] = gst_bin_get_by_name(GST_BIN(pipeline), name);
    }
}

void *VideoComposite::run_pipeline(gpointer user_data) {
    // nothing special here, but we might in future forward nodes to the
    // thread etc.
    VideoComposite *self = static_cast<VideoComposite *>(user_data);
    if (!self) {
        // fallback to static instance (macOS behaviour can be weird)
        self = s_instance;
    }
    if (!self) {
        g_printerr("run_pipeline: user_data is NULL and no fallback instance\n");
        return NULL;
    }
    g_print("run_pipeline: self=%p\n", (void*)self);
    GMainLoop *loop;

    /* create pipeline and elements manually so the number/resolution/position
       of mixer sinks can be changed at runtime. */
    self->pipeline = gst_pipeline_new("video_pipeline");
    if (!self->pipeline) {
        g_printerr("run_pipeline: failed to create pipeline\n");
        return NULL;
    }

    PipelineElements elems;
    if (!self->buildStage1And2OutputBranches(elems))
        return NULL;

    std::vector<SinkLayout> layouts;
    gint bg_width = 0;
    gint bg_height = 0;
    if (!self->buildStage3LayoutAndProbe(elems.mix, layouts, bg_width, bg_height))
        return NULL;
    if (!self->buildStage4BackgroundBranch(elems.mix, bg_width, bg_height))
        return NULL;
    if (!self->buildStage5UdpSourceBranches(elems.mix, layouts))
        return NULL;
    self->buildStage6ConfigureShadersAndStabCache();

    loop = g_main_loop_new(NULL, FALSE);
    PipelineRunContext run_ctx{loop, self->pipeline, FALSE};

    /* Bus watch handles diagnostics and EOS-driven graceful exit. */
    {
        GstBus *bus = gst_element_get_bus(self->pipeline);
        gst_bus_add_watch(bus, bus_call, &run_ctx);
        gst_object_unref(bus);
    }

    guint sigint_source = g_unix_signal_add(SIGINT, request_eos_and_stop, &run_ctx);
    guint sigterm_source = g_unix_signal_add(SIGTERM, request_eos_and_stop, &run_ctx);

    gst_element_set_state(self->pipeline, GST_STATE_PLAYING);

    g_print("Running Fake IMU Stabilization Test at 4K (4x 1080p). Press Ctrl+C to stop.\n");
    g_main_loop_run(loop);

    if (sigint_source)
        g_source_remove(sigint_source);
    if (sigterm_source)
        g_source_remove(sigterm_source);

    if (!run_ctx.eos_requested) {
        // If the loop exited for another reason, still force EOS so mp4mux
        // writes final container metadata before pipeline teardown.
        gst_element_send_event(self->pipeline, gst_event_new_eos());
        GstBus *bus = gst_element_get_bus(self->pipeline);
        GstMessage *final_msg = gst_bus_timed_pop_filtered(
            bus,
            3 * GST_SECOND,
            (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
        if (final_msg)
            gst_message_unref(final_msg);
        gst_object_unref(bus);
    }

    gst_element_set_state(self->pipeline, GST_STATE_NULL);
    for (auto *s : self->stab) {
        if (s) gst_object_unref(s);
    }
    self->stab.clear();
    gst_object_unref(self->pipeline);
    self->pipeline = nullptr;
    g_main_loop_unref(loop);

    return NULL;
}


// ---------------------------------------------------------------------------
// new methods

void VideoComposite::setBuoyNodes(std::vector<std::unique_ptr<BuoyNode>> *nodes) {
    nodes_ = nodes;
}