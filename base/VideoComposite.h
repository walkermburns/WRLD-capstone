#ifndef VIDEOCOMPOSITE_H
#define VIDEOCOMPOSITE_H

#include <gst/gst.h>
#include <gst/gl/gl.h> // for GstGLShader
#include <string>

class VideoComposite {
public:
    // path to fragment shader read from disk
    explicit VideoComposite(const std::string &shaderPath);
    ~VideoComposite();

    // start the pipeline; on macOS this will be run from a secondary thread
    void start();

private:
    GstElement *pipeline;
    GstElement *stab[4];

    // configuration
    int num_sinks;                  // number of mixer sinks/branches
    GstStructure *uniforms;         // shared k1/zoom structure for shaders

    // path/content of the distortion shader
    std::string shader_code;

    // shader uniforms (current live values)
    float live_k1;
    float live_zoom;

    static gboolean on_draw_signal(GstElement *glfilter, GstGLShader *shader,
                                   guint texture, guint width, guint height,
                                   gpointer user_data);
    static GstPadProbeReturn imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                          gpointer user_data);

    // helper setters
    void setUniforms(float k1, float zoom);

    // entry-point used with gst_macos_main
    static void *run_pipeline(gpointer user_data);

    // When running on macOS, gst_macos_main sometimes doesn’t forward the
    // user_data pointer.  We keep a static fallback instance here so the
    // callback can still access the object.
    static VideoComposite *s_instance;
};

#endif // VIDEOCOMPOSITE_H