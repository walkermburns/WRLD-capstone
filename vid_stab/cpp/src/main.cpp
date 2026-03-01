#include <gst/gst.h>
#include <glib.h>

#include <Eigen/Geometry>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <stdexcept>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <optional>

#include "serial_imu.hpp"
#include "stab_math.hpp"

// -----------------------------
// Config
// -----------------------------
static constexpr int RES_W = 640;
static constexpr int RES_H = 480;

// Latency compensation + prediction knobs (mirrors your “better python” version)
static constexpr double COMPENSATE_FRAC = 1.0;   // 1.0 = full display-age compensation
static constexpr double IMU_LEAD_S = 0.0;        // extra lead on top of measured latency
static constexpr double PREDICT_MAX_S = 0.12;    // cap forward prediction horizon
static constexpr float  OMEGA_ALPHA = 0.25f;     // omega low-pass (0.15–0.35 typical)
static constexpr float  OMEGA_CLAMP = 25.0f;     // rad/s clamp to reject spikes

// Pipeline: name glshader and sink so we can attach probes.
// NOTE: keep latency low (leaky queues).
static const char* PIPELINE_DESC =
    "v4l2src device=/dev/video0 io-mode=2 ! "
    "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "videoconvert ! video/x-raw,format=RGBA ! "
    "glupload ! "
    "glshader name=stab qos=true ! "
    "queue name=qdisp max-size-buffers=1 leaky=downstream ! "
    "glimagesink name=vsink sync=false";

static std::string read_file(const char* path) {
  FILE* f = std::fopen(path, "rb");
  if (!f) throw std::runtime_error(std::string("Failed to open file: ") + path);

  std::fseek(f, 0, SEEK_END);
  long n = std::ftell(f);
  std::rewind(f);

  std::string s;
  s.resize(static_cast<size_t>(n));
  size_t got = std::fread(s.data(), 1, s.size(), f);
  std::fclose(f);

  if (got != s.size()) throw std::runtime_error(std::string("Failed to read file: ") + path);
  return s;
}

static gboolean on_bus_msg(GstBus* bus, GstMessage* msg, gpointer user_data) {
  (void)bus;
  auto* loop = static_cast<GMainLoop*>(user_data);

  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      GError* err = nullptr;
      gchar* dbg = nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      g_printerr("[Gst ERROR] %s\n", err ? err->message : "unknown");
      if (dbg) g_printerr("[Gst DEBUG] %s\n", dbg);
      if (err) g_error_free(err);
      if (dbg) g_free(dbg);
      g_main_loop_quit(loop);
      break;
    }
    case GST_MESSAGE_EOS:
      g_main_loop_quit(loop);
      break;
    default:
      break;
  }
  return TRUE;
}

struct AppState {
  GstElement* pipeline = nullptr;
  GstElement* glshader = nullptr;

  SerialImuReader imu{"/dev/ttyUSB0", 115200};

  StabConfig cfg;
  StabState stab;

  Eigen::Matrix3f K;
  Eigen::Matrix3f Kinv;
  Eigen::Matrix3f Rflu2cv;

  Eigen::Matrix3f last_good_Hinv = Eigen::Matrix3f::Identity();

  // running_time (sec) -> steady_time (sec) mapping
  bool tsync_initialized = false;
  double offset_steady_minus_running_s = 0.0;

  // Display-side age estimate (updated from vsink probe)
  std::atomic<double> display_age_s{0.0};

  // Filtered omega predictor state
  Eigen::Vector3f omega_filt = Eigen::Vector3f::Zero();
  std::chrono::steady_clock::time_point last_omega_t{};
  bool have_omega = false;

  // Frame-driven uniforms: probe computes, idle applies
  std::mutex uni_mtx;
  bool pending = false;
  Eigen::Matrix3f pending_Hinv = Eigen::Matrix3f::Identity();
  std::atomic<bool> apply_scheduled{false};

  std::chrono::steady_clock::time_point last_print = std::chrono::steady_clock::now();
};

static double gst_running_time_s(GstElement* pipeline) {
  GstClock* clock = gst_element_get_clock(pipeline);
  if (!clock) return -1.0;
  GstClockTime now = gst_clock_get_time(clock);
  gst_object_unref(clock);

  GstClockTime base = gst_element_get_base_time(pipeline);
  if (now < base) return -1.0;

  return double(now - base) / double(GST_SECOND);
}

static GstStructure* make_uniforms_from_Hinv(const Eigen::Matrix3f& Hinv, int w, int h) {
  char buf[1024];
  std::snprintf(
      buf, sizeof(buf),
      "uniforms, "
      "w=(float)%d, h=(float)%d, "
      "h00=(float)%g, h01=(float)%g, h02=(float)%g, "
      "h10=(float)%g, h11=(float)%g, h12=(float)%g, "
      "h20=(float)%g, h21=(float)%g, h22=(float)%g",
      w, h,
      Hinv(0,0), Hinv(0,1), Hinv(0,2),
      Hinv(1,0), Hinv(1,1), Hinv(1,2),
      Hinv(2,0), Hinv(2,1), Hinv(2,2));

  return gst_structure_new_from_string(buf);
}

static bool homography_is_safe(const Eigen::Matrix3f& Hinv, int w, int h) {
  for (int yi = 0; yi < 3; yi++) {
    float y = (h - 1) * (yi / 2.f);
    for (int xi = 0; xi < 3; xi++) {
      float x = (w - 1) * (xi / 2.f);
      Eigen::Vector3f p(x, y, 1.f);
      Eigen::Vector3f q = Hinv * p;
      float z = q.z();
      if (std::abs(z) < 1e-2f) return false;
      Eigen::Vector2f uv = q.head<2>() / z;
      if (uv.array().abs().maxCoeff() > 5.f * std::max(w, h)) return false;
    }
  }
  return true;
}

// Apply uniforms on the main thread ASAP (idle callback).
static gboolean apply_uniforms_idle(gpointer user_data) {
  auto* app = static_cast<AppState*>(user_data);
  while (true) {
    Eigen::Matrix3f Hinv;
    {
      std::lock_guard<std::mutex> lk(app->uni_mtx);
      if (!app->pending) {
        app->apply_scheduled.store(false);
        return G_SOURCE_REMOVE;
      }
      Hinv = app->pending_Hinv;
      app->pending = false;
    }

    GstStructure* s = make_uniforms_from_Hinv(Hinv, app->cfg.w, app->cfg.h);
    if (s) {
      g_object_set(G_OBJECT(app->glshader), "uniforms", s, nullptr);
      gst_structure_free(s);
    }
    // If a newer Hinv arrived while we applied, loop again.
  }
}

// Late probe at vsink: estimate display-side age (how old frames are when shown).
static GstPadProbeReturn on_vsink_probe(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
  (void)pad;
  auto* app = static_cast<AppState*>(user_data);

  if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER)) return GST_PAD_PROBE_OK;
  GstBuffer* buf = gst_pad_probe_info_get_buffer(info);
  if (!buf) return GST_PAD_PROBE_OK;

  GstClockTime pts = GST_BUFFER_PTS(buf);
  if (pts == GST_CLOCK_TIME_NONE) return GST_PAD_PROBE_OK;

  const double rt_frame = double(pts) / double(GST_SECOND);
  const double rt_now = gst_running_time_s(app->pipeline);
  if (rt_now <= 0.0) return GST_PAD_PROBE_OK;

  double age = std::max(0.0, rt_now - rt_frame);
  age = std::clamp(age, 0.0, 0.25);

  // Simple EMA on atomic
  const double prev = app->display_age_s.load(std::memory_order_relaxed);
  constexpr double alpha = 0.2;
  const double filt = (1.0 - alpha) * prev + alpha * age;
  app->display_age_s.store(filt, std::memory_order_relaxed);

  return GST_PAD_PROBE_OK;
}

// IMU orientation at time with filtered-omega prediction.
static std::optional<Eigen::Quaternionf> quat_at_with_prediction(
    AppState* app,
    const std::chrono::steady_clock::time_point& t_query) {

  SerialImuReader::Sample s1, s2;
  if (!app->imu.latest_two(s1, s2)) return std::nullopt;

  // Update omega_filt when new sample arrives
  if (!app->have_omega || s2.t != app->last_omega_t) {
    const double dt = std::chrono::duration<double>(s2.t - s1.t).count();
    if (dt > 1e-6) {
      Eigen::Quaternionf dq = s2.q * s1.q.conjugate(); // s1->s2
      dq.normalize();
      Eigen::AngleAxisf aa(dq);
      Eigen::Vector3f rotvec = aa.axis() * aa.angle();
      Eigen::Vector3f omega_inst = rotvec / float(dt);

      omega_inst = omega_inst.cwiseMax(Eigen::Vector3f::Constant(-OMEGA_CLAMP))
                           .cwiseMin(Eigen::Vector3f::Constant( OMEGA_CLAMP));

      app->omega_filt = (1.f - OMEGA_ALPHA) * app->omega_filt + OMEGA_ALPHA * omega_inst;
      app->last_omega_t = s2.t;
      app->have_omega = true;
    }
  }

  // In-range: real interpolation
  if (t_query <= s2.t) {
    return app->imu.quat_at(t_query);
  }

  // Predict forward from latest
  const double dt_pred = std::clamp(std::chrono::duration<double>(t_query - s2.t).count(),
                                    0.0, PREDICT_MAX_S);

  Eigen::Vector3f rv = app->omega_filt * float(dt_pred);
  const float ang = rv.norm();

  Eigen::Quaternionf q_pred = s2.q;
  if (ang > 1e-9f) {
    Eigen::AngleAxisf aa(ang, rv / ang);
    q_pred = s2.q * Eigen::Quaternionf(aa);
  }
  q_pred.normalize();
  return q_pred;
}

// Early probe at glshader sink: compute Hinv for this buffer, schedule apply ASAP.
static GstPadProbeReturn on_shader_probe(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
  (void)pad;
  auto* app = static_cast<AppState*>(user_data);

  if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER)) return GST_PAD_PROBE_OK;
  GstBuffer* buf = gst_pad_probe_info_get_buffer(info);
  if (!buf) return GST_PAD_PROBE_OK;

  GstClockTime pts = GST_BUFFER_PTS(buf);
  if (pts == GST_CLOCK_TIME_NONE) return GST_PAD_PROBE_OK;
  const double rt_frame = double(pts) / double(GST_SECOND);

  // Initialize running->steady mapping
  if (!app->tsync_initialized) {
    const double rt_now = gst_running_time_s(app->pipeline);
    if (rt_now <= 0.0) return GST_PAD_PROBE_OK;
    const double steady_now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    app->offset_steady_minus_running_s = steady_now - rt_now;
    app->tsync_initialized = true;
    g_print("[tsync] initialized offset_steady_minus_running=%.6f\n", app->offset_steady_minus_running_s);
  }

  // Frame time as steady_clock
  const double steady_frame_s = rt_frame + app->offset_steady_minus_running_s;
  const auto t_frame = std::chrono::steady_clock::time_point(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(steady_frame_s)));

  // Latency compensation uses display-side age
  const double disp_age = app->display_age_s.load(std::memory_order_relaxed);
  const double lead_s = COMPENSATE_FRAC * disp_age + IMU_LEAD_S;
  const auto t_query = t_frame + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                 std::chrono::duration<double>(lead_s));

  if (app->imu.size() < 20) return GST_PAD_PROBE_OK;

  auto q_cur_opt = quat_at_with_prediction(app, t_query);
  if (!q_cur_opt) return GST_PAD_PROBE_OK;
  const Eigen::Quaternionf q_cur = *q_cur_opt;

  // Capture reference once (preserve initial tilt)
  if (!app->stab.have_ref) {
    auto q_ref_opt = app->imu.quat_at(t_frame);
    if (!q_ref_opt) return GST_PAD_PROBE_OK;
    app->stab.q_ref = *q_ref_opt;
    app->stab.have_ref = true;
    g_print("[ref] reference locked\n");
  }
  const Eigen::Quaternionf q_ref = app->stab.q_ref;

  // Relative rotation (matches your working python convention)
  const Eigen::Quaternionf q_rel = q_cur * q_ref.conjugate();
  const Eigen::Vector3f ypr_rel = ypr_zyx_from_quat_arduino(q_rel); // [yaw, pitch, roll] rad

  const float pitch = ypr_rel.y();
  const float roll  = ypr_rel.z();

  // EMA smoothing (optional)
  app->stab.pitch_f = (1.f - app->cfg.smooth_alpha) * app->stab.pitch_f + app->cfg.smooth_alpha * pitch;
  app->stab.roll_f  = (1.f - app->cfg.smooth_alpha) * app->stab.roll_f  + app->cfg.smooth_alpha * roll;

  // Cancel pitch/roll (note the negative signs)
  const float pitch_c = std::clamp(-app->cfg.gain * app->stab.pitch_f, -app->cfg.max_tilt_rad, app->cfg.max_tilt_rad);
  const float roll_c  = std::clamp(-app->cfg.gain * app->stab.roll_f,  -app->cfg.max_tilt_rad, app->cfg.max_tilt_rad);

  const Eigen::AngleAxisf Rx(roll_c,  Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf Ry(pitch_c, Eigen::Vector3f::UnitY());
  const Eigen::Matrix3f R_corr_flu = (Ry * Rx).toRotationMatrix();

  // Homography H = K * R * K^-1, shader uses Hinv
  const Eigen::Matrix3f R_corr_cv = app->Rflu2cv * R_corr_flu.inverse() * app->Rflu2cv.transpose();
  const Eigen::Matrix3f H = app->K * R_corr_cv * app->Kinv;
  Eigen::Matrix3f Hinv = H.inverse();

  if (!Hinv.allFinite() || !homography_is_safe(Hinv, app->cfg.w, app->cfg.h)) {
    Hinv = app->last_good_Hinv;
  } else {
    app->last_good_Hinv = Hinv;
  }

  // Store for main-thread apply
  {
    std::lock_guard<std::mutex> lk(app->uni_mtx);
    app->pending_Hinv = Hinv;
    app->pending = true;
  }

  // Schedule apply ASAP on main loop (no timer)
  if (!app->apply_scheduled.exchange(true)) {
    g_idle_add_full(G_PRIORITY_HIGH_IDLE, apply_uniforms_idle, app, nullptr);
  }

  // Debug ~1 Hz
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - app->last_print).count() >= 1.0) {
    const Eigen::Vector3f ypr_cur = ypr_zyx_from_quat_arduino(q_cur) * (180.f / float(M_PI));
    g_print("[CUR] yaw=%.2f pitch=%.2f roll=%.2f deg | display_age=%.1f ms | lead=%.1f ms\n",
            ypr_cur.x(), ypr_cur.y(), ypr_cur.z(),
            disp_age * 1000.0, lead_s * 1000.0);
    app->last_print = now;
  }

  return GST_PAD_PROBE_OK;
}

int main(int argc, char** argv) {
  gst_init(&argc, &argv);

  AppState app;
  app.cfg.w = RES_W;
  app.cfg.h = RES_H;
  app.cfg.hfov_deg = 50.f;
  app.cfg.video_delay_s = 0.0f;

  app.cfg.gain = 1.0f;
  app.cfg.smooth_alpha = 1.0f;   // 1.0 = no EMA smoothing (most responsive)
  app.cfg.max_tilt_rad = float(35.0 * M_PI / 180.0);

  app.K = make_K(app.cfg.w, app.cfg.h, app.cfg.hfov_deg);
  app.Kinv = app.K.inverse();
  app.Rflu2cv = R_flu_to_cv();

  app.imu.set_use_conjugate(false);
  if (!app.imu.start()) {
    g_printerr("Failed to start IMU serial reader\n");
    return 1;
  }

  // Build pipeline
  GError* err = nullptr;
  GstElement* pipeline = gst_parse_launch(PIPELINE_DESC, &err);
  if (!pipeline) {
    g_printerr("Failed to create pipeline: %s\n", err ? err->message : "unknown");
    if (err) g_error_free(err);
    return 1;
  }
  app.pipeline = pipeline;

  GstElement* glshader = gst_bin_get_by_name(GST_BIN(pipeline), "stab");
  if (!glshader) {
    g_printerr("Could not find element named 'stab'\n");
    gst_object_unref(pipeline);
    return 1;
  }
  app.glshader = glshader;

  GstElement* vsink = gst_bin_get_by_name(GST_BIN(pipeline), "vsink");
  if (!vsink) {
    g_printerr("Could not find element named 'vsink'\n");
    gst_object_unref(glshader);
    gst_object_unref(pipeline);
    return 1;
  }

  // Load shader (run from vid_stab/cpp)
  const char* shader_path = "../shaders/warp.frag";
  try {
    std::string frag = read_file(shader_path);
    g_object_set(G_OBJECT(glshader), "fragment", frag.c_str(), nullptr);
    g_object_set(G_OBJECT(glshader), "update-shader", TRUE, nullptr);
  } catch (const std::exception& e) {
    g_printerr("%s\n", e.what());
    gst_object_unref(vsink);
    gst_object_unref(glshader);
    gst_object_unref(pipeline);
    return 1;
  }

  // Install probes
  {
    GstPad* pad = gst_element_get_static_pad(vsink, "sink");
    if (!pad) {
      g_printerr("Could not get vsink sink pad\n");
      gst_object_unref(vsink);
      gst_object_unref(glshader);
      gst_object_unref(pipeline);
      return 1;
    }
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, on_vsink_probe, &app, nullptr);
    gst_object_unref(pad);
  }
  {
    GstPad* pad = gst_element_get_static_pad(glshader, "sink");
    if (!pad) {
      g_printerr("Could not get glshader sink pad\n");
      gst_object_unref(vsink);
      gst_object_unref(glshader);
      gst_object_unref(pipeline);
      return 1;
    }
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, on_shader_probe, &app, nullptr);
    gst_object_unref(pad);
  }

  // Main loop + bus
  GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
  GstBus* bus = gst_element_get_bus(pipeline);
  gst_bus_add_watch(bus, on_bus_msg, loop);
  gst_object_unref(bus);

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_print("Running (C++ latency-comp + filtered omega + frame-driven uniforms). Ctrl+C to exit.\n");
  g_main_loop_run(loop);

  // Cleanup
  gst_element_set_state(pipeline, GST_STATE_NULL);
  g_main_loop_unref(loop);

  app.imu.stop();

  gst_object_unref(vsink);
  gst_object_unref(glshader);
  gst_object_unref(pipeline);
  return 0;
}