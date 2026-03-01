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
      num_sinks(4), uniforms(nullptr) {
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
    for (int i = 0; i < 4; ++i)
        stab[i] = nullptr;

    /* prepare the uniforms structure once and reuse it */
    uniforms = gst_structure_new("uniforms",
                                 "k1", G_TYPE_FLOAT, live_k1,
                                 "zoom", G_TYPE_FLOAT, live_zoom,
                                 NULL);
}

VideoComposite::~VideoComposite() {
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        for(int i=0;i<4;i++){
            if(stab[i]) gst_object_unref(stab[i]);
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

gboolean VideoComposite::on_draw_signal(GstElement *glfilter, GstGLShader *shader,
                                        guint texture, guint width, guint height,
                                        gpointer user_data) {
    VideoComposite *self = static_cast<VideoComposite *>(user_data);
    gst_gl_shader_set_uniform_1f(shader, "k1", self->live_k1);
    gst_gl_shader_set_uniform_1f(shader, "zoom", self->live_zoom);
    return FALSE;
}


gboolean VideoComposite::update_fake_imu_cb(gpointer user_data) {
    VideoComposite *self = static_cast<VideoComposite *>(user_data);
    static float time_t = 0.0f;
    time_t += 0.1f;

    gfloat sway_x = sin(time_t) * 0.1f;
    gfloat sway_y = cos(time_t * 0.5f) * 0.05f;
    gfloat rotation = sin(time_t * 0.8f) * 5.0f;

    float k1 = 0.3f + (sin(time_t) * 1.0f);
    float zoom = 1.1f;

    /* update the shared structure instead of creating a new one */
    self->setUniforms(k1, zoom);

    for (int i = 0; i < self->num_sinks; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
        if (shader) {
            g_object_set(shader, "uniforms", self->uniforms, NULL);
            gst_object_unref(shader);
        }
    }

    for (int i = 0; i < self->num_sinks; i++) {
        if (self->stab[i]) {
            g_object_set(self->stab[i],
                "translation-x", sway_x,
                "translation-y", sway_y,
                NULL);
        }
    }
    return TRUE;
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

    /* dynamic configuration for mixer sinks; change values or number here */
    struct SinkLayout { gint xpos, ypos, width, height; };
    std::vector<SinkLayout> layouts;
    layouts.reserve(self->num_sinks);
    for (int i = 0; i < self->num_sinks; ++i) {
        /* default tiling, 2x2 grid at 1920x1080 each */
        SinkLayout l;
        l.xpos   = (i & 1) ? 1920 : 0;
        l.ypos   = (i & 2) ? 1080 : 0;
        l.width  = 1920;
        l.height = 1080;
        layouts.push_back(l);
    }

    /* create videotestsrc branches and hook them to mixer pads */
    for (int i = 0; i < self->num_sinks; ++i) {
        GstElement *src   = gst_element_factory_make("videotestsrc", NULL);
        GstElement *rate  = gst_element_factory_make("videorate", NULL);
        GstElement *caps1 = gst_element_factory_make("capsfilter", NULL);
        GstElement *caps2 = gst_element_factory_make("capsfilter", NULL);
        GstElement *glup  = gst_element_factory_make("glupload", NULL);
        char shader_nm[16], stab_nm[16];
        snprintf(shader_nm, sizeof(shader_nm), "lens%d", i);
        snprintf(stab_nm, sizeof(stab_nm), "stab%d", i);
        GstElement *shader = gst_element_factory_make("glshader", shader_nm);
        GstElement *stab   = gst_element_factory_make("gltransformation", stab_nm);

        if (!src || !rate || !caps1 || !caps2 || !glup || !shader || !stab) {
            g_printerr("run_pipeline: failed to create branch %d elements\n", i);
            return NULL;
        }

        g_object_set(src, "pattern", (i < 2) ? 0 : 1, NULL);
        GstCaps *c1 = gst_caps_from_string(
            "video/x-raw,width=1920,height=1080,framerate=30/1");
        GstCaps *c2 = gst_caps_from_string(
            "video/x-raw,framerate=60/1");
        g_object_set(caps1, "caps", c1, NULL);
        g_object_set(caps2, "caps", c2, NULL);
        gst_caps_unref(c1);
        gst_caps_unref(c2);

        gst_bin_add_many(GST_BIN(self->pipeline),
                         src, caps1, rate, caps2, glup, shader, stab, NULL);

        if (!gst_element_link(src, caps1) ||
            !gst_element_link(caps1, rate) ||
            !gst_element_link(rate, caps2) ||
            !gst_element_link_many(caps2, glup, shader, stab, NULL)) {
            g_printerr("run_pipeline: branch %d link failed\n", i);
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

        GstPad *srcpad = gst_element_get_static_pad(stab, "src");
        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link branch %d to mixer\n", i);
            return NULL;
        }
        gst_object_unref(srcpad);
        gst_object_unref(sinkpad);
    }

    /* shaders and stabs can still be configured by name; use num_sinks */
    for (int i = 0; i < self->num_sinks; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader_elem = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
        if (shader_elem) {
            g_object_set(shader_elem, "fragment", self->shader_code.c_str(), NULL);
            g_signal_connect(shader_elem, "update-shader", G_CALLBACK(on_draw_signal), self);
            gst_object_unref(shader_elem);
        }
    }

    /* update stab pointers; clear any remaining entries */
    for (int i = 0; i < 4; ++i) {
        if (i < self->num_sinks) {
            char name[16];
            snprintf(name, sizeof(name), "stab%d", i);
            self->stab[i] = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
        } else {
            if (self->stab[i]) {
                gst_object_unref(self->stab[i]);
                self->stab[i] = nullptr;
            }
        }
    }

    g_timeout_add(16, update_fake_imu_cb, self);

    gst_element_set_state(self->pipeline, GST_STATE_PLAYING);

    loop = g_main_loop_new(NULL, FALSE);
    g_print("Running Fake IMU Stabilization Test at 4K (4x 1080p). Press Ctrl+C to stop.\n");
    g_main_loop_run(loop);

    gst_element_set_state(self->pipeline, GST_STATE_NULL);
    for(int i=0;i<4;i++){
        if(self->stab[i]) gst_object_unref(self->stab[i]);
    }
    gst_object_unref(self->pipeline);
    g_main_loop_unref(loop);

    return NULL;
}
