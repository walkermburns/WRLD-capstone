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

#include "serial_imu.hpp"
#include "stab_math.hpp"

static constexpr int RES_W = 640;
static constexpr int RES_H = 480;

static const char* PIPELINE_DESC =
    "v4l2src device=/dev/video0 io-mode=2 ! "
    "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "videoconvert ! video/x-raw,format=RGBA ! "
    "glupload ! "
    "glshader name=stab qos=true ! "
    "glimagesink sync=false";

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

  // --- PTS sync state ---
  std::atomic<double> last_rt_frame_s{-1.0};  // running-time seconds for latest frame
  bool tsync_initialized = false;
  double offset_steady_minus_running_s = 0.0; // steady_time = running_time + offset

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

// Pad probe: capture buffer PTS (running time) safely (NO uniforms here)
static GstPadProbeReturn on_frame_probe(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
  (void)pad;
  auto* app = static_cast<AppState*>(user_data);

  if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER)) {
    return GST_PAD_PROBE_OK;
  }
  GstBuffer* buf = gst_pad_probe_info_get_buffer(info);
  if (!buf) return GST_PAD_PROBE_OK;

  GstClockTime pts = GST_BUFFER_PTS(buf);
  if (pts == GST_CLOCK_TIME_NONE) return GST_PAD_PROBE_OK;

  double rt_s = double(pts) / double(GST_SECOND);
  app->last_rt_frame_s.store(rt_s, std::memory_order_relaxed);

  return GST_PAD_PROBE_OK;
}

// 30 Hz timer: compute + apply uniforms on main thread using frame-time
static gboolean tick_update_uniforms(gpointer user_data) {
  auto* app = static_cast<AppState*>(user_data);

  static int tick = 0;
  tick++;

  // Must have received at least one frame timestamp
  double rt_frame = app->last_rt_frame_s.load(std::memory_order_relaxed);
  if (rt_frame < 0.0) {
    if (tick % 60 == 0) g_print("[tick] waiting for first frame PTS...\n");
    return TRUE;
  }

  // Initialize running->steady mapping once (like your Python perf_counter offset)
  if (!app->tsync_initialized) {
    double rt_now = gst_running_time_s(app->pipeline);
    if (rt_now > 0.0) {
      double steady_now = std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      app->offset_steady_minus_running_s = steady_now - rt_now;
      app->tsync_initialized = true;
      g_print("[tsync] initialized offset_steady_minus_running=%.6f\n", app->offset_steady_minus_running_s);
    } else {
      return TRUE;
    }
  }

  // Convert frame running-time -> steady_clock time_point
  double steady_frame_s = rt_frame + app->offset_steady_minus_running_s;
  auto t_frame = std::chrono::steady_clock::time_point(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(steady_frame_s)));

  // Apply desired delay (this now truly means "relative to displayed frame time")
  auto t_ref = t_frame - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(app->cfg.video_delay_s));

  // Need IMU data
  if (app->imu.size() < 20) {
    GstStructure* s = make_uniforms_from_Hinv(Eigen::Matrix3f::Identity(), app->cfg.w, app->cfg.h);
    if (s) { g_object_set(G_OBJECT(app->glshader), "uniforms", s, nullptr); gst_structure_free(s); }
    return TRUE;
  }

  // Query IMU at frame time
  auto q_cur_opt = app->imu.quat_at(t_frame);
  if (!q_cur_opt) {
    // out-of-range: hold last good
    GstStructure* s = make_uniforms_from_Hinv(app->last_good_Hinv, app->cfg.w, app->cfg.h);
    if (s) { g_object_set(G_OBJECT(app->glshader), "uniforms", s, nullptr); gst_structure_free(s); }
    return TRUE;
  }
  Eigen::Quaternionf q_cur = *q_cur_opt;

  // Lock reference once we can query it
  if (!app->stab.have_ref) {
    auto q_ref_init = app->imu.quat_at(t_ref);
    if (!q_ref_init) {
      // not enough history yet; keep identity
      GstStructure* s = make_uniforms_from_Hinv(Eigen::Matrix3f::Identity(), app->cfg.w, app->cfg.h);
      if (s) { g_object_set(G_OBJECT(app->glshader), "uniforms", s, nullptr); gst_structure_free(s); }
      return TRUE;
    }
    app->stab.q_ref = *q_ref_init;
    app->stab.have_ref = true;
    g_print("[ref] reference locked\n");
  }
  Eigen::Quaternionf q_ref = app->stab.q_ref;

  // Debug once per second
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - app->last_print).count() >= 1.0) {
    Eigen::Vector3f ypr_cur = ypr_zyx_from_quat_arduino(q_cur) * (180.f / float(M_PI));
    g_print("[CUR] yaw=%.2f pitch=%.2f roll=%.2f deg | rt_frame=%.3f\n",
            ypr_cur.x(), ypr_cur.y(), ypr_cur.z(), rt_frame);
    app->last_print = now;
  }

  // Relative rotation (your working convention)
  Eigen::Quaternionf q_rel = q_cur * q_ref.conjugate();
  Eigen::Vector3f ypr_rel = ypr_zyx_from_quat_arduino(q_rel); // rad

  float pitch = ypr_rel.y();
  float roll  = ypr_rel.z();

  // Smooth EMA (now this should behave as expected)
  app->stab.pitch_f = (1.f - app->cfg.smooth_alpha) * app->stab.pitch_f + app->cfg.smooth_alpha * pitch;
  app->stab.roll_f  = (1.f - app->cfg.smooth_alpha) * app->stab.roll_f  + app->cfg.smooth_alpha * roll;

  float pitch_c = std::clamp(app->cfg.gain * app->stab.pitch_f, -app->cfg.max_tilt_rad, app->cfg.max_tilt_rad);
  float roll_c  = std::clamp(app->cfg.gain * app->stab.roll_f,  -app->cfg.max_tilt_rad, app->cfg.max_tilt_rad);

  // R_stab_flu = Ry(pitch) * Rx(roll) (yaw=0)
  Eigen::AngleAxisf Rx(roll_c,  Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf Ry(pitch_c, Eigen::Vector3f::UnitY());
  Eigen::Matrix3f R_stab_flu = (Ry * Rx).toRotationMatrix();

  Eigen::Matrix3f R_stab_cv = app->Rflu2cv * R_stab_flu * app->Rflu2cv.transpose();
  Eigen::Matrix3f H = app->K * R_stab_cv * app->Kinv;
  Eigen::Matrix3f Hinv = H.inverse();

  if (!Hinv.allFinite() || !homography_is_safe(Hinv, app->cfg.w, app->cfg.h)) {
    Hinv = app->last_good_Hinv;
  } else {
    app->last_good_Hinv = Hinv;
  }

  GstStructure* s = make_uniforms_from_Hinv(Hinv, app->cfg.w, app->cfg.h);
  if (s) {
    g_object_set(G_OBJECT(app->glshader), "uniforms", s, nullptr);
    gst_structure_free(s);
  }

  return TRUE;
}

int main(int argc, char** argv) {
  gst_init(&argc, &argv);

  AppState app;
  app.cfg.w = RES_W;
  app.cfg.h = RES_H;
  app.cfg.hfov_deg = 50.f;

  // With PTS sync, you typically want this 0.0 (unless you intentionally lag reference)
  app.cfg.video_delay_s = 0.0f;

  app.cfg.gain = 1.0f;          // try 2.0f temporarily if you want to exaggerate
  app.cfg.smooth_alpha = 1.0f; // 1.0f disables EMA smoothing
  app.cfg.max_tilt_rad = float(35.0 * M_PI / 180.0);

  app.K = make_K(app.cfg.w, app.cfg.h, app.cfg.hfov_deg);
  app.Kinv = app.K.inverse();
  app.Rflu2cv = R_flu_to_cv();

  // Start IMU reader
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

  // Load shader (run from vid_stab/cpp)
  const char* shader_path = "../shaders/warp.frag";
  try {
    std::string frag = read_file(shader_path);
    g_object_set(G_OBJECT(glshader), "fragment", frag.c_str(), nullptr);
    g_object_set(G_OBJECT(glshader), "update-shader", TRUE, nullptr);
  } catch (const std::exception& e) {
    g_printerr("%s\n", e.what());
    gst_object_unref(glshader);
    gst_object_unref(pipeline);
    return 1;
  }

  // Install pad probe to capture buffer PTS (frame times)
  GstPad* sinkpad = gst_element_get_static_pad(glshader, "sink");
  if (!sinkpad) {
    g_printerr("Could not get glshader sink pad\n");
    gst_object_unref(glshader);
    gst_object_unref(pipeline);
    return 1;
  }
  gst_pad_add_probe(sinkpad, GST_PAD_PROBE_TYPE_BUFFER, on_frame_probe, &app, nullptr);
  gst_object_unref(sinkpad);

  // Main loop + bus
  GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
  GstBus* bus = gst_element_get_bus(pipeline);
  gst_bus_add_watch(bus, on_bus_msg, loop);
  gst_object_unref(bus);

  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  guint tid = g_timeout_add(16, tick_update_uniforms, &app);
  g_print("Timer installed, id=%u\n", tid);
  g_print("Running (C++ v1.5 PTS-synced). Ctrl+C to exit.\n");

  g_main_loop_run(loop);

  gst_element_set_state(pipeline, GST_STATE_NULL);
  g_main_loop_unref(loop);

  app.imu.stop();

  gst_object_unref(glshader);
  gst_object_unref(pipeline);
  return 0;
}