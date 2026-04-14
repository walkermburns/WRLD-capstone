#ifndef VIDEOCOMPOSITE_H
#define VIDEOCOMPOSITE_H

#include <gst/gst.h>
#include <gst/gl/gl.h> // for GstGLShader
#include "buoy.pb.h"            // for IMU_proto message
#include "MathHelpers.h"        // quaternion/matrix utilities
#include <string>
#include <vector>
#include <array>

// forward declare BuoyNode so we can hold a pointer to the vector without
// including its header (avoids circular dependency)
class BuoyNode;

class VideoComposite {
public:
    // small helper type for quaternion state (w,x,y,z).  we avoid bringing
    // in Eigen here to keep the dependency list minimal; only a simple
    // struct is required for storage and callbacks.
    // quaternion stored by the IMU; alias to helpers type
    using Quaternion = MathHelpers::Quaternion;

    // path to fragment shader read from disk and list of UDP ports
    explicit VideoComposite(const std::string &shaderPath,
                            const std::vector<int> &ports);
    ~VideoComposite();

    // provide access to the list of buoy nodes maintained in main.  the
    // composite will poll the first entry for its quaternion each frame.  the
    // pointer is not owned; it must remain valid for the lifetime of the
    // VideoComposite instance.
    void setBuoyNodes(std::vector<std::unique_ptr<BuoyNode>> *nodes);

    // start the pipeline; on macOS this will be run from a secondary thread
    void start();

    // legacy helper: update the stored quaternion from a protobuf IMU
    // message.  once H matrix support is fully working this will no longer
    // be used; the compositor instead polls the BuoyNode for its precomputed
    // homography.  for now it still offers a simple copy variant useful for
    // testing.
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

    // non-owning pointer to the IMU nodes vector held by main
    std::vector<std::unique_ptr<BuoyNode>> *nodes_ = nullptr;

    // path/content of the distortion shader
    std::string shader_code;

    // shader uniforms (current live values)
    float live_k1;
    
    // (previously stored quaternion; no longer used when H matrix is
    // obtained directly from BuoyNode.  kept here temporarily for
    // backwards compatibility with updateQuaternion.)
    Quaternion quat_; 

    // camera intrinsics / homography bookkeeping --------------------------------
    // (moved into BuoyNode; the composite no longer needs these values)

    float live_zoom;
    float live_w;
    float live_h;
    float live_h00, live_h01, live_h02;
    float live_h10, live_h11, live_h12;
    float live_h20, live_h21, live_h22;

    // per-branch latest sender timestamp (microseconds) extracted from RTP
    // extension headers.  Written by RTP pad probes and read by shader probes.
    std::mutex frameTsMutex_;
    std::vector<uint64_t> latest_frame_ts_us_;
    std::vector<bool> latest_frame_ts_valid_;

    // quaternion/matrix helper functions are defined in MathHelpers
    // (no need to redeclare here).  implementations live in MathHelpers.{h,cpp}

    static gboolean on_draw_signal(GstElement *glfilter, GstGLShader *shader,
                                   guint texture, guint width, guint height,
                                   gpointer user_data);
    static GstPadProbeReturn imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                          gpointer user_data);
    static GstPadProbeReturn rtp_timestamp_probe_cb(GstPad *pad,
                                                    GstPadProbeInfo *info,
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