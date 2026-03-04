#ifndef VIDEOCOMPOSITE_H
#define VIDEOCOMPOSITE_H

#include <gst/gst.h>
#include <gst/gl/gl.h> // for GstGLShader
#include "buoy.pb.h"            // for IMU_proto message
#include <string>
#include <vector>

class VideoComposite {
public:
    // small helper type for quaternion state (w,x,y,z).  we avoid bringing
    // in Eigen here to keep the dependency list minimal; only a simple
    // struct is required for storage and callbacks.
    struct Quaternion {
        float w{1.0f};
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
    };

    // path to fragment shader read from disk and list of UDP ports
    explicit VideoComposite(const std::string &shaderPath,
                            const std::vector<int> &ports);
    ~VideoComposite();

    // start the pipeline; on macOS this will be run from a secondary thread
    void start();

    // update the stored quaternion from a protobuf IMU message.  this is
    // the method that will be bound as a BuoyNode callback; for now it
    // merely copies the values and prints them to stdout so we can verify
    // it's being invoked.
    void updateQuaternion(const buoy_proto::IMU_proto &msg);

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
    
    // latest quaternion state from a buoy IMU message.  updated via
    // updateQuaternion().  public getter kept simple if callers need to
    // inspect values.
    Quaternion quat_; 

    // camera intrinsics / homography bookkeeping --------------------------------
    float cam_w = 1920.0f;          // image width used when building K
    float cam_h = 1080.0f;          // image height
    float cam_hfov_deg = 50.0f;     // horizontal field of view in degrees

    // pre‑computed 3x3 matrices stored row-major (K, its inverse, and the
    // FLU->CV conversion).  keeping them as plain arrays avoids pulling
    // Eigen into the header; the implementation file can use whichever
    // math helpers it likes.
    float K[9];
    float Kinv[9];
    float Rflu2cv_mat[9];
    float last_good_Hinv[9];   // remember last usable homography to avoid bad data
    bool have_ref = false;     // whether quat_ref has been initialized
    Quaternion quat_ref;

    float live_zoom;
    float live_w;
    float live_h;
    float live_h00, live_h01, live_h02;
    float live_h10, live_h11, live_h12;
    float live_h20, live_h21, live_h22;

    // smoothing/filtering state (mirrors gst_warp_imu.py behaviour)
    // 1.0 = no smoothing, values closer to 0 increase smoothing
    float corr_smooth_alpha = 1.0f;
    float corr_pitch_filt = 0.0f;
    float corr_roll_filt = 0.0f;

    // small helper routines implemented in VideoComposite.cpp
    static void ypr_from_quat(const Quaternion &q, float &yaw, float &pitch, float &roll);
    static Quaternion quat_inverse(const Quaternion &q);
    static Quaternion quat_mult(const Quaternion &a, const Quaternion &b);
    static void make_K(float w, float h, float hfov_deg, float outK[9]);
    static void mult3x3(const float a[9], const float b[9], float out[9]);
    static void transpose3x3(const float a[9], float out[9]);
    static bool invert3x3(const float m[9], float out[9]);
    static bool homography_is_safe(const float Hinv[9], float w, float h);

    static gboolean on_draw_signal(GstElement *glfilter, GstGLShader *shader,
                                   guint texture, guint width, guint height,
                                   gpointer user_data);
    static GstPadProbeReturn imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                          gpointer user_data);

    // helper used by macOS entry point
    static void *run_pipeline(gpointer user_data);

    // When running on macOS, gst_macos_main sometimes doesn’t forward the
    // user_data pointer.  We keep a static fallback instance here so the
    // callback can still access the object.
    static VideoComposite *s_instance;

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

};

#endif // VIDEOCOMPOSITE_H