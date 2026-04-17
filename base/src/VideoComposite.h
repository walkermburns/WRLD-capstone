#ifndef VIDEOCOMPOSITE_H
#define VIDEOCOMPOSITE_H

#include <gst/gst.h>
#include <gst/gl/gl.h>
#include "buoy.pb.h"
#include "MathHelpers.h"
#include <string>
#include <vector>
#include <array>
#include <deque>
#include <cstdint>
#include <mutex>
#include <memory>
#include <thread>
#include <atomic>

class BuoyNode;

class VideoComposite {
public:
    using Quaternion = MathHelpers::Quaternion;

    explicit VideoComposite(const std::string &shaderPath,
                            const std::vector<int> &ports);
    ~VideoComposite();

    void setBuoyNodes(std::vector<std::unique_ptr<BuoyNode>> *nodes);
    void start();
    void stop();
    void updateQuaternion(const buoy_proto::IMU_proto &msg);

private:
    GstElement *pipeline;
    GstElement *mix_element;
    GMainLoop *main_loop_;
    std::vector<GstElement*> stab;

    int num_src;
    std::vector<int> video_ports;
    GstStructure *uniforms;

    std::vector<bool> branch_active;
    std::vector<std::unique_ptr<BuoyNode>> *nodes_ = nullptr;

    std::string shader_code;

    float live_k1;
    Quaternion quat_;

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

    static void *run_pipeline(gpointer user_data);
    static VideoComposite *s_instance;

    void setUniforms(float k1,
                     float zoom,
                     float w = 1920.0f,
                     float h = 1080.0f,
                     float h00 = 1.0f, float h01 = 0.0f, float h02 = 0.0f,
                     float h10 = 0.0f, float h11 = 1.0f, float h12 = 0.0f,
                     float h20 = 0.0f, float h21 = 0.0f, float h22 = 1.0f);

    struct FrameMetaEntry {
        uint64_t frame_idx;
        uint64_t timestamp_us;
    };

    std::vector<std::deque<FrameMetaEntry>> sender_ts_queue_us_;
    mutable std::mutex sender_ts_mutex_;

    std::vector<int64_t> frame_pts_to_imu_offset_us_;

    std::vector<std::thread> meta_threads_;
    std::atomic<bool> meta_threads_running_{false};

    void start_metadata_receivers();
    void stop_metadata_receivers();
    void metadata_receiver_loop(int branch_index, int meta_port);

    std::vector<std::array<float,9>> last_applied_H_;
    std::vector<bool> last_applied_H_valid_;
    std::vector<uint64_t> last_used_frame_ts_us_;
    std::vector<bool> metadata_seen_;

    uint64_t frame_dt_outlier_us_ = 60000;
    float homography_blend_alpha_ = 1.0f;
};

#endif // VIDEOCOMPOSITE_H
