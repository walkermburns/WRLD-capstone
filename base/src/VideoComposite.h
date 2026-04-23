#ifndef VIDEOCOMPOSITE_H
#define VIDEOCOMPOSITE_H

#include <gst/gst.h>
#include <gst/gl/gl.h>
#include "buoy.pb.h"
#include "MathHelpers.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#ifdef HAVE_ORT
#include <onnxruntime_cxx_api.h>
#endif
#include <string>
#include <vector>
#include <array>
#include <deque>
#include <cstdint>
#include <mutex>
#include <memory>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <chrono>

typedef struct _GstAppSink GstAppSink;

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

    struct DetectionBox {
        int x = 0;
        int y = 0;
        int w = 0;
        int h = 0;
        double score = 0.0;
    };

    static gboolean on_draw_signal(GstElement *glfilter, GstGLShader *shader,
                                   guint texture, guint width, guint height,
                                   gpointer user_data);
    static GstPadProbeReturn imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                          gpointer user_data);
    static GstFlowReturn detector_new_sample_cb(GstAppSink *sink,
                                                gpointer user_data);
    static gboolean detector_preview_tick_cb(gpointer user_data);

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

    // Detection side branch. No live-display overlay so timing stays close to the original path.
    bool person_detection_enabled_ = false;
    std::string detector_backend_ = "hog";
    int detector_width_ = 640;
    int detector_height_ = 360;
    int detector_period_ms_ = 200;

    float detector_conf_threshold_ = 0.35f;
    float detector_nms_threshold_ = 0.45f;
    bool detector_preview_enabled_ = true;
    bool main_record_enabled_ = true;
    bool detector_preview_window_created_ = false;
    std::string detector_preview_window_name_ = "Person Detection Debug";
    guint detector_preview_timer_id_ = 0;
    std::atomic<bool> stop_requested_{false};

    std::thread detector_thread_;
    std::atomic<bool> detector_thread_running_{false};
    std::condition_variable detector_cv_;
    std::mutex detector_frame_mutex_;
    cv::Mat detector_latest_frame_;
    bool detector_have_new_frame_ = false;
    std::chrono::steady_clock::time_point detector_last_run_tp_{};

    std::vector<DetectionBox> latest_composite_boxes_;
    std::chrono::steady_clock::time_point latest_composite_detection_tp_{};
    std::mutex detection_mutex_;

    std::mutex detector_preview_mutex_;
    cv::Mat detector_preview_frame_;
    bool detector_preview_frame_ready_ = false;

    void startDetectorThread();
    void stopDetectorThread();
    void detectorWorkerLoop();
    std::vector<DetectionBox> runHogPersonDetector(const cv::Mat &bgr);
    std::vector<DetectionBox> runYoloV8Detector(const cv::Mat &bgr);
    std::vector<DetectionBox> runDetector(const cv::Mat &bgr);
#ifdef HAVE_ORT
    bool initYoloOrtSession();
    bool yolo_ort_ready_ = false;
    Ort::Env ort_env_{ORT_LOGGING_LEVEL_WARNING, "person_det"};
    Ort::SessionOptions ort_session_options_;
    std::unique_ptr<Ort::Session> ort_session_;
    std::string ort_input_name_;
    std::vector<std::string> ort_output_names_;
    int yolo_model_input_w_ = 640;
    int yolo_model_input_h_ = 640;
#endif
};

#endif // VIDEOCOMPOSITE_H
