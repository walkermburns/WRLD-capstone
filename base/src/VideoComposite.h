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
    GstStructure *uniforms;         // shared structure containing all shader uniforms

    // runtime state tracking which branches have been linked
    std::vector<bool> branch_active;

    // path/content of the distortion shader
    std::string shader_code;

    // shader uniforms (current live values)
    float live_k1;
    float live_zoom;
    float live_w;
    float live_h;
    float live_h00, live_h01, live_h02;
    float live_h10, live_h11, live_h12;
    float live_h20, live_h21, live_h22;

    static gboolean on_draw_signal(GstElement *glfilter, GstGLShader *shader,
                                   guint texture, guint width, guint height,
                                   gpointer user_data);
    static GstPadProbeReturn imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                          gpointer user_data);

    // helper setter for all shader uniforms.  Parameters have default values
    // so callers can update only a subset; the members stored in the object
    // keep the most recent values.
    void setUniforms(float k1,
                     float zoom,
                     float w = 1920.0f,
                     float h = 1080.0f,
                     float h00 = 1.0f, float h01 = 0.0f, float h02 = 0.0f,
                     float h10 = 0.0f, float h11 = 1.0f, float h12 = 0.0f,
                     float h20 = 0.0f, float h21 = 0.0f, float h22 = 1.0f);

    // entry-point used with gst_macos_main
    static void *run_pipeline(gpointer user_data);

    // When running on macOS, gst_macos_main sometimes doesn’t forward the
    // user_data pointer.  We keep a static fallback instance here so the
    // callback can still access the object.
    static VideoComposite *s_instance;
};

#endif // VIDEOCOMPOSITE_H