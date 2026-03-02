#ifndef VIDEOCOMPOSITE_H
#define VIDEOCOMPOSITE_H

#include <gst/gst.h>
#include <gst/gl/gl.h> // for GstGLShader
#include <string>
#include <vector>

class VideoComposite {
public:
    // path to fragment shader read from disk and list of UDP ports
    explicit VideoComposite(const std::string &shaderPath,
                            const std::vector<int> &ports);
    ~VideoComposite();

    // start the pipeline; on macOS this will be run from a secondary thread
    void start();

private:
    GstElement *pipeline;
    GstElement *mix_element;        // cached mixer for callbacks
    std::vector<GstElement*> stab;   // stabilization elements, one per source

    // configuration
    int num_src;                    // number of mixer sources/branches
    std::vector<int> video_ports;   // UDP ports corresponding to each branch
    GstStructure *uniforms;         // shared k1/zoom structure for shaders

    // runtime state tracking which branches have been linked
    std::vector<bool> branch_active;

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