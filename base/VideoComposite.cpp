#include "VideoComposite.h"
#include <gst/video/video.h>
#include <gst/gl/gl.h>
#include <math.h>
#include <stdio.h>

// Fragment shader string moved here for convenience
static const gchar *distort_shader = 
    "#version 100\n"
    "#ifdef GL_ES\n"
    "precision mediump float;\n"
    "#endif\n"
    "varying vec2 v_texcoord;\n"
    "uniform sampler2D tex;\n"
    "uniform float k1;\n" // Custom uniform for distortion
    "uniform float zoom;\n" // Custom uniform for zoom
    "void main () {\n"
    "  vec2 uv = (v_texcoord - 0.5) * 1.1;\n"
    "  float r2 = dot(uv, uv);\n"
    "  vec2 distorted_uv = uv * (1.0 + k1 * r2);\n"
    "  gl_FragColor = texture2D(tex, distorted_uv + 0.5);\n"
    "}\n";

// static member definition
VideoComposite *VideoComposite::s_instance = nullptr;

VideoComposite::VideoComposite()
    : pipeline(nullptr), live_k1(0.3f), live_zoom(1.1f) {
    for (int i = 0; i < 4; ++i)
        stab[i] = nullptr;
}

VideoComposite::~VideoComposite() {
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        for(int i=0;i<4;i++){
            if(stab[i]) gst_object_unref(stab[i]);
        }
        gst_object_unref(pipeline);
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

    GstStructure *vars = gst_structure_new("uniforms",
        "k1", G_TYPE_FLOAT, k1,
        "zoom", G_TYPE_FLOAT, zoom,
        NULL);

    for (int i = 0; i < 4; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
        if (shader) {
            g_object_set(shader, "uniforms", vars, NULL);
            gst_object_unref(shader);
        }
    }

    gst_structure_free(vars);

    for (int i = 0; i < 4; i++) {
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

    // build the pipeline string without embedded "//" comments
    const gchar *pipeline_str =
        "glvideomixer name=mix background=1 "
        "sink_0::xpos=0 sink_0::ypos=0 sink_0::width=1920 sink_0::height=1080 "
        "sink_1::xpos=1920 sink_1::ypos=0 sink_1::width=1920 sink_1::height=1080 "
        "sink_2::xpos=0 sink_2::ypos=1080 sink_2::width=1920 sink_2::height=1080 "
        "sink_3::xpos=1920 sink_3::ypos=1080 sink_3::width=1920 sink_3::height=1080 ! "
        "glcolorconvert ! "
        "fpsdisplaysink video-sink=autovideosink text-overlay=true sync=true ";

    self->pipeline = gst_parse_launch(pipeline_str, &error);
    if (error != NULL) {
        g_printerr("Could not build pipeline: %s\n", error->message);
        g_clear_error(&error);
        return NULL;
    }

    /* obtain the mixer so we can attach branches to it */
    GstElement *mix = gst_bin_get_by_name(GST_BIN(self->pipeline), "mix");
    if (!mix) {
        g_printerr("run_pipeline: mixer element not found\n");
        return NULL;
    }

    /* create four videotestsrc -> ... -> gltransformation branches in code */
    for (int i = 0; i < 4; ++i) {
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

        /* configure elements exactly as in the original string */
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

        /* link step-by-step to diagnose failures if any */
        if (!gst_element_link(src, caps1)) {
            g_printerr("run_pipeline: src->caps1 link failed on branch %d\n", i);
            return NULL;
        }
        if (!gst_element_link(caps1, rate)) {
            g_printerr("run_pipeline: caps1->rate link failed on branch %d\n", i);
            return NULL;
        }
        if (!gst_element_link(rate, caps2)) {
            g_printerr("run_pipeline: rate->caps2 link failed on branch %d\n", i);
            return NULL;
        }
        if (!gst_element_link_many(caps2, glup, shader, stab, NULL)) {
            g_printerr("run_pipeline: caps2->glup/shader/stab link failed on branch %d\n", i);
            return NULL;
        }

        GstPad *sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
        if (!sinkpad) {
            g_printerr("run_pipeline: could not get mixer pad for branch %d\n", i);
            return NULL;
        }

        /* set pad properties for positioning */
        g_object_set(sinkpad,
                     "xpos",   (i & 1) ? 1920 : 0,
                     "ypos",   (i & 2) ? 1080 : 0,
                     "width",  1920,
                     "height", 1080,
                     NULL);

        GstPad *srcpad = gst_element_get_static_pad(stab, "src");
        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link branch %d to mixer\n", i);
            return NULL;
        }
        gst_object_unref(srcpad);
        gst_object_unref(sinkpad);
    }

    /* shaders and stabs can still be configured by name */
    for (int i = 0; i < 4; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader_elem = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
        if (shader_elem) {
            g_object_set(shader_elem, "fragment", distort_shader, NULL);
            g_signal_connect(shader_elem, "update-shader", G_CALLBACK(on_draw_signal), self);
            gst_object_unref(shader_elem);
        }
    }

    self->stab[0] = gst_bin_get_by_name(GST_BIN(self->pipeline), "stab0");
    self->stab[1] = gst_bin_get_by_name(GST_BIN(self->pipeline), "stab1");
    self->stab[2] = gst_bin_get_by_name(GST_BIN(self->pipeline), "stab2");
    self->stab[3] = gst_bin_get_by_name(GST_BIN(self->pipeline), "stab3");

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
