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
        "sink_0::zorder=0 "
        "sink_1::xpos=0 sink_0::ypos=0 sink_0::width=1920 sink_0::height=1080 "
        "sink_2::xpos=1920 sink_1::ypos=0 sink_1::width=1920 sink_1::height=1080 "
        "sink_3::xpos=0 sink_2::ypos=1080 sink_2::width=1920 sink_2::height=1080 "
        "sink_4::xpos=1920 sink_3::ypos=1080 sink_3::width=1920 sink_3::height=1080 ! "
        "glcolorconvert ! "
        "fpsdisplaysink video-sink=autovideosink text-overlay=true sync=false "
        "videotestsrc is-live=true pattern=0 ! video/x-raw,width=3840,height=2160,framerate=60/1 ! glupload ! mix.sink_0 "
        "udpsrc port=5101 caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000\" ! rtpjitterbuffer latency=200 drop_on_latency=true ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videorate ! video/x-raw,framerate=60/1 ! glupload ! glshader name=lens0 ! gltransformation name=stab0 ! queue ! mix.sink_1 "
        "udpsrc port=5102 caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000\" ! rtpjitterbuffer latency=200 drop_on_latency=true ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videorate ! video/x-raw,framerate=60/1 ! glupload ! "
            "glshader name=lens1 ! gltransformation name=stab1 ! queue ! mix.sink_2 "
        "udpsrc port=5103 caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000\" ! rtpjitterbuffer latency=200 drop_on_latency=true ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videorate ! video/x-raw,framerate=60/1 ! glupload ! "
            "glshader name=lens2 ! gltransformation name=stab2 ! queue ! mix.sink_3 "
        "udpsrc port=5104 caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000\" ! rtpjitterbuffer latency=200 drop_on_latency=true ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videorate ! video/x-raw,framerate=60/1 ! glupload ! "
            "glshader name=lens3 ! gltransformation name=stab3 ! queue ! mix.sink_4";

    self->pipeline = gst_parse_launch(pipeline_str, &error);
    if (error != NULL) {
        g_printerr("Could not build pipeline: %s\n", error->message);
        g_clear_error(&error);
        return NULL;
    }

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
