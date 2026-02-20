#include <gst/gst.h>
#include <math.h>
#include <stdio.h>

#include <gst/video/video.h>
#include <gst/gl/gl.h> // <--- Add this for GstGLShader and uniform functions
#include <math.h>
#include <stdio.h>

float live_k1 = 0.3f;
float live_zoom = 1.1f;

const gchar *distort_shader = 
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

// Structure to hold our elements so the timer callback can access them
typedef struct _CustomData {
    GstElement *pipeline;
    GstElement *stab[4];
} CustomData;

// The signal signature for "update-shader"
static void on_update_shader(GstElement *glshader, GstGLShader *shader, gpointer user_data) {
    // These functions push your C variables to the GPU uniforms
    gst_gl_shader_set_uniform_1f(shader, "k1", live_k1);
    gst_gl_shader_set_uniform_1f(shader, "zoom", live_zoom);
}

static gboolean on_draw_signal(GstElement *glfilter, GstGLShader *shader, 
                               guint texture, guint width, guint height, 
                               gpointer user_data) {
    // Set the uniforms on the shader object provided by the signal
    gst_gl_shader_set_uniform_1f(shader, "k1", live_k1);
    gst_gl_shader_set_uniform_1f(shader, "zoom", live_zoom);
    
    // Return FALSE to let GStreamer proceed with the default draw
    return FALSE; 
}


// This function acts as our "Fake IMU" data feed
static gboolean update_fake_imu_cb(gpointer user_data) {
    CustomData *data = (CustomData *)user_data;
    static float time_t = 0.0f;
    
    time_t += 0.1f;
    
    // Generate our fake IMU sine wave (Sway and Rotation)
    gfloat sway_x = sin(time_t) * 0.1f; 
    gfloat sway_y = cos(time_t * 0.5f) * 0.05f; 
    gfloat rotation = sin(time_t * 0.8f) * 5.0f; 

    float k1 = 0.3 + (sin(time_t) * 1.0);
    float zoom = 1.1;

    GstStructure *vars = gst_structure_new("uniforms",
        "k1", G_TYPE_FLOAT, k1,
        "zoom", G_TYPE_FLOAT, zoom,
        NULL);

    // 3. Push the structure to all 4 shader elements
    for (int i = 0; i < 4; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader = gst_bin_get_by_name(GST_BIN(data->pipeline), name);
        if (shader) {
            // The "vars" property is the magic bridge to the uniforms
            g_object_set(shader, "uniforms", vars, NULL);
            gst_object_unref(shader);
        }
    }

    gst_structure_free(vars);

    // Apply the correction to all 4 video streams
    for (int i = 0; i < 4; i++) {
        if (data->stab[i]) {
            g_object_set(data->stab[i], 
                "translation-x", sway_x,
                "translation-y", sway_y,
                // "rotation-z", rotation,
                NULL);
        }
    }
    return TRUE; 
}

// We move our main logic into a dedicated function.
// On macOS, the main thread MUST be reserved for the UI/NSApplication run loop.
static void * run_pipeline(gpointer user_data) {
    CustomData data;
    GMainLoop *loop;
    GError *error = NULL;

    // 1080p is 1920x1080. 
    // A 2x2 grid of 1080p streams results in a 4K output window (3840x2160).
    const gchar *pipeline_str = 
        "glvideomixer name=mix background=1 "
        "  sink_0::xpos=0 sink_0::ypos=0 sink_0::width=1920 sink_0::height=1080 "
        "  sink_1::xpos=1920 sink_1::ypos=0 sink_1::width=1920 sink_1::height=1080 "
        "  sink_2::xpos=0 sink_2::ypos=1080 sink_2::width=1920 sink_2::height=1080 "
        "  sink_3::xpos=1920 sink_3::ypos=1080 sink_3::width=1920 sink_3::height=1080 ! "
        "glcolorconvert ! "
        "fpsdisplaysink video-sink=autovideosink text-overlay=true sync=true "
        
        // Stream 0
        "videotestsrc pattern=0 ! video/x-raw,width=1920,height=1080,framerate=30/1 ! videorate ! video/x-raw,framerate=60/1 ! glupload ! "
        "glshader name=lens0 ! gltransformation name=stab0 ! mix.sink_0 "
        
        // Stream 1
        "videotestsrc pattern=0 ! video/x-raw,width=1920,height=1080,framerate=30/1 ! videorate ! video/x-raw,framerate=60/1 ! glupload ! "
        "glshader name=lens1 ! gltransformation name=stab1 ! mix.sink_1 "
        
        // Stream 2
        "videotestsrc pattern=1 ! video/x-raw,width=1920,height=1080,framerate=30/1 ! videorate ! video/x-raw,framerate=60/1 ! glupload ! "
        "glshader name=lens2 ! gltransformation name=stab2 ! mix.sink_2 "
        
        // Stream 3
        "videotestsrc pattern=1 ! video/x-raw,width=1920,height=1080,framerate=30/1 ! videorate ! video/x-raw,framerate=60/1 ! glupload ! "
        "glshader name=lens3 ! gltransformation name=stab3 ! mix.sink_3";

    // Launch the pipeline
    data.pipeline = gst_parse_launch(pipeline_str, &error);
    if (error != NULL) {
        g_printerr("Could not build pipeline: %s\n", error->message);
        g_clear_error(&error);
        return NULL;
    }

    // 3. Manually inject the shader string into the 4 lens elements
    for (int i = 0; i < 4; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader_elem = gst_bin_get_by_name(GST_BIN(data.pipeline), name);
        if (shader_elem) {
            g_object_set(shader_elem, "fragment", distort_shader, NULL);
            g_signal_connect(shader_elem, "update-shader", G_CALLBACK(on_draw_signal), NULL);
            gst_object_unref(shader_elem);
            g_print("Injected shader into %s\n", name);
        }
    }

    // Get the stabilization elements from the pipeline so we can control them
    data.stab[0] = gst_bin_get_by_name(GST_BIN(data.pipeline), "stab0");
    data.stab[1] = gst_bin_get_by_name(GST_BIN(data.pipeline), "stab1");
    data.stab[2] = gst_bin_get_by_name(GST_BIN(data.pipeline), "stab2");
    data.stab[3] = gst_bin_get_by_name(GST_BIN(data.pipeline), "stab3");

    // Set up a GLib timer to fire every 33ms (~30fps)
    g_timeout_add(16, update_fake_imu_cb, &data);

    // Start playing
    gst_element_set_state(data.pipeline, GST_STATE_PLAYING);

    // Create and run the main loop
    loop = g_main_loop_new(NULL, FALSE);
    g_print("Running Fake IMU Stabilization Test at 4K (4x 1080p). Press Ctrl+C to stop.\n");
    g_main_loop_run(loop);

    // Clean up
    gst_element_set_state(data.pipeline, GST_STATE_NULL);
    for(int i=0; i<4; i++) {
        if(data.stab[i]) gst_object_unref(data.stab[i]);
    }
    gst_object_unref(data.pipeline);
    g_main_loop_unref(loop);

    return NULL;
}

int main(int argc, char *argv[]) {
    // Initialize GStreamer first
    gst_init(&argc, &argv);

#ifdef __APPLE__
    // macOS requires an active NSApplication loop on the main thread for UI and OpenGL contexts.
    // gst_macos_main sets this up and runs our actual logic in a secondary thread.
    return gst_macos_main((GstMainFunc)run_pipeline, argc, argv, NULL);
#else
    // For Linux/Windows, we just run the logic directly on the main thread.
    run_pipeline(NULL);
    return 0;
#endif
}