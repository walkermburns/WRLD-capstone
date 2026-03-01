#include "VideoComposite.h"
#include <gst/video/video.h>
#include <gst/gl/gl.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>


// static member definition
VideoComposite *VideoComposite::s_instance = nullptr;

VideoComposite::VideoComposite(const std::string &shaderPath)
    : pipeline(nullptr), live_k1(0.3f), live_zoom(1.1f),
      // default to four incoming UDP streams; we add a separate background
      // videotestsrc below rather than counting it here.
      num_src(2), uniforms(nullptr), stab() {
    // load shader file
    std::ifstream in(shaderPath);
    if (!in) {
        throw std::runtime_error("failed to open shader file " + shaderPath);
    }
    std::ostringstream buf;
    buf << in.rdbuf();
    shader_code = buf.str();
    if (shader_code.empty()) {
        throw std::runtime_error("shader file " + shaderPath + " is empty");
    }
    /* allocate and clear vector to match num_src; entries beyond
       num_src are not used */
    stab.assign(num_src, nullptr);

    /* prepare the uniforms structure once and reuse it */
    uniforms = gst_structure_new("uniforms",
                                 "k1", G_TYPE_FLOAT, live_k1,
                                 "zoom", G_TYPE_FLOAT, live_zoom,
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

void VideoComposite::setUniforms(float k1, float zoom) {
    live_k1 = k1;
    live_zoom = zoom;
    if (uniforms) {
        gst_structure_set(uniforms,
                          "k1", G_TYPE_FLOAT, live_k1,
                          "zoom", G_TYPE_FLOAT, live_zoom,
                          NULL);
    } else {
        uniforms = gst_structure_new("uniforms",
                                     "k1", G_TYPE_FLOAT, live_k1,
                                     "zoom", G_TYPE_FLOAT, live_zoom,
                                     NULL);
    }
}

// static callbacks ----------------------------------------------------------

/* pad probe that runs once for each buffer passing through the mixer source.
   We use it to update fake IMU state and propagate uniforms/transforms. */

GstPadProbeReturn VideoComposite::imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                               gpointer user_data) {
    if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER))
        return GST_PAD_PROBE_OK;
    VideoComposite *self = static_cast<VideoComposite *>(user_data);

    static float time_t = 0.0f;
    time_t += 0.1f;

    gfloat sway_x = sin(time_t) * 0.1f;
    gfloat sway_y = cos(time_t * 0.5f) * 0.05f;

    float k1 = 0.3f + (sin(time_t) * 1.0f);
    float zoom = 1.1f;

    self->setUniforms(k1, zoom);

    /* push updated uniforms into all shader elements */
    for (int i = 0; i < self->num_src; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
        if (shader) {
            g_object_set(shader, "uniforms", self->uniforms, NULL);
            gst_object_unref(shader);
        }
    }

    for (int i = 0; i < self->num_src && i < (int)self->stab.size(); i++) {
        if (self->stab[i]) {
            g_object_set(self->stab[i],
                         "translation-x", sway_x,
                         "translation-y", sway_y,
                         NULL);
        }
    }
    return GST_PAD_PROBE_OK;
}

void *VideoComposite::run_pipeline(gpointer user_data) {
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
    GError *error = NULL;

    /* create pipeline and elements manually so the number/resolution/position
       of mixer sinks can be changed at runtime. */
    self->pipeline = gst_pipeline_new("video_pipeline");
    if (!self->pipeline) {
        g_printerr("run_pipeline: failed to create pipeline\n");
        return NULL;
    }

    GstElement *mix = gst_element_factory_make("glvideomixer", "mix");
    GstElement *convert = gst_element_factory_make("glcolorconvert", "conv");
    GstElement *fps = gst_element_factory_make("fpsdisplaysink", "fps");
    GstElement *videosink = gst_element_factory_make("autovideosink", "vsink");

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

    /* add all elements to pipeline (videosink must be owned to avoid premature
       free when fpsdisplaysink grabs it) */
    gst_bin_add_many(GST_BIN(self->pipeline), mix, convert, fps, videosink, NULL);
    if (!gst_element_link_many(mix, convert, fps, NULL)) {
        g_printerr("run_pipeline: failed to link core elements\n");
        return NULL;
    }
    /* no need to link fps->videosink; fpsdisplaysink handles its child sink */

    /* we'll use mix pointer below when attaching branches */

    /* dynamic configuration for mixer sinks; change values or number here
       we want a 2x2 grid for the UDP sources and a full‑screen background test
       pattern.  The background element is created separately below. */
    struct SinkLayout { gint xpos, ypos, width, height; };
    std::vector<SinkLayout> layouts;
    layouts.reserve(self->num_src);
    for (int i = 0; i < self->num_src; ++i) {
        /* compute side-by-side horizontal layout */
        SinkLayout l;
        l.xpos  = i * 1920;  // each stream offset horizontally
        l.ypos  = 0;
        l.width = 1920;
        l.height= 1080;
        layouts.push_back(l);
    }
    /* background resolution should cover all sources horizontally */
    gint bg_width = self->num_src * 1920;
    gint bg_height = 1080; // match individual source height

    /* add pad probe to mixer src so we can update IMU/uniforms per frame */
    GstPad *probe_pad = gst_element_get_static_pad(mix, "src");
    if (probe_pad) {
        gst_pad_add_probe(probe_pad, GST_PAD_PROBE_TYPE_BUFFER,
                          imu_probe_cb, self, NULL);
        gst_object_unref(probe_pad);
    }

    /*--- background branch ------------------------------------------------*/
    {
        GstElement *bg_src   = gst_element_factory_make("videotestsrc", "bg_src");
        GstElement *bg_caps  = gst_element_factory_make("capsfilter", "bg_caps");
        GstElement *bg_glup  = gst_element_factory_make("glupload", "bg_glup");
        if (!bg_src || !bg_caps || !bg_glup) {
            g_printerr("run_pipeline: failed to create background elements\n");
            return NULL;
        }
        g_object_set(bg_src, "pattern", 0, "is-live", TRUE, NULL);
        GstCaps *bgcaps = gst_caps_from_string(
            (std::ostringstream() << "video/x-raw,width=" << bg_width
             << ",height=" << bg_height << ",framerate=60/1").str().c_str());
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
            return NULL;
        }
        gst_object_unref(bg_srcpad);
        gst_object_unref(bg_sinkpad);
    }

    /* create udp source branches and hook them to mixer pads */
    for (int i = 0; i < self->num_src; ++i) {
        GstElement *udpsrc    = gst_element_factory_make("udpsrc", NULL);
        GstElement *jitter    = gst_element_factory_make("rtpjitterbuffer", NULL);
        GstElement *depay     = gst_element_factory_make("rtph264depay", NULL);
        GstElement *parse     = gst_element_factory_make("h264parse", NULL);
        GstElement *dec       = gst_element_factory_make("avdec_h264", NULL);
        GstElement *conv      = gst_element_factory_make("videoconvert", NULL);
        GstElement *rate      = gst_element_factory_make("videorate", NULL);
        GstElement *capsf     = gst_element_factory_make("capsfilter", NULL);
        GstElement *glup      = gst_element_factory_make("glupload", NULL);
        char shader_nm[16], stab_nm[16];
        snprintf(shader_nm, sizeof(shader_nm), "lens%d", i);
        snprintf(stab_nm, sizeof(stab_nm), "stab%d", i);
        GstElement *shader    = gst_element_factory_make("glshader", shader_nm);
        GstElement *stab      = gst_element_factory_make("gltransformation", stab_nm);
        GstElement *queue     = gst_element_factory_make("queue", NULL);

        if (!udpsrc || !jitter || !depay || !parse || !dec || !conv ||
            !rate || !capsf || !glup || !shader || !stab || !queue) {
            g_printerr("run_pipeline: failed to create udp branch %d elements\n", i);
            return NULL;
        }

        /* configure udp source and RTP caps */
        g_object_set(udpsrc, "port", 5101 + i, NULL);
        GstCaps *rtpcaps = gst_caps_from_string(
            "application/x-rtp,media=video,encoding-name=H264,"
            "payload=96,clock-rate=90000");
        g_object_set(udpsrc, "caps", rtpcaps, NULL);
        gst_caps_unref(rtpcaps);
        // increase latency if there are issues, higher latency will cause frame
        // drop and desync if source is delayed or doesn't exist.
        g_object_set(jitter, "latency", 0, "drop-on-latency", TRUE, NULL);

        GstCaps *caps2 = gst_caps_from_string("video/x-raw,framerate=60/1");
        g_object_set(capsf, "caps", caps2, NULL);
        gst_caps_unref(caps2);

        gst_bin_add_many(GST_BIN(self->pipeline),
                         udpsrc, jitter, depay, parse, dec,
                         conv, rate, capsf, glup, shader, stab, queue, NULL);

        if (!gst_element_link_many(udpsrc, jitter, depay, parse, dec,
                                   conv, rate, capsf, glup, shader, stab,
                                   queue, NULL)) {
            g_printerr("run_pipeline: udp branch %d link failed\n", i);
            return NULL;
        }

        GstPad *sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
        if (!sinkpad) {
            g_printerr("run_pipeline: could not get mixer pad for branch %d\n", i);
            return NULL;
        }

        /* apply dynamic layout
           (could be reassigned while running by reconfiguring pad properties) */
        const SinkLayout &l = layouts[i];
        g_object_set(sinkpad,
                     "xpos",   l.xpos,
                     "ypos",   l.ypos,
                     "width",  l.width,
                     "height", l.height,
                     NULL);

        /* link the *queue* output, not the stab element; queue is last in
           the branch chain so it provides a stable src pad for the mixer */
        GstPad *srcpad = gst_element_get_static_pad(queue, "src");
        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link branch %d to mixer\n", i);
            return NULL;
        }
        gst_object_unref(srcpad);
        /* set ordering so background stays at bottom */
        g_object_set(sinkpad, "zorder", i + 1, NULL);
        gst_object_unref(sinkpad);
    }

    /* shaders and stabs can still be configured by name; use num_src */
    for (int i = 0; i < self->num_src; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader_elem = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
        if (shader_elem) {
            g_object_set(shader_elem, "fragment", self->shader_code.c_str(), NULL);
            /* no signals; uniforms will be updated via pad probe each frame */
            gst_object_unref(shader_elem);
        }
    }

    /* update stab pointers vector to match current num_src */
    self->stab.assign(self->num_src, nullptr);
    for (int i = 0; i < self->num_src; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "stab%d", i);
        self->stab[i] = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
    }


    gst_element_set_state(self->pipeline, GST_STATE_PLAYING);

    loop = g_main_loop_new(NULL, FALSE);
    g_print("Running Fake IMU Stabilization Test at 4K (4x 1080p). Press Ctrl+C to stop.\n");
    g_main_loop_run(loop);

    gst_element_set_state(self->pipeline, GST_STATE_NULL);
    for (auto *s : self->stab) {
        if (s) gst_object_unref(s);
    }
    gst_object_unref(self->pipeline);
    g_main_loop_unref(loop);

    return NULL;
}
