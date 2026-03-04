#include "VideoComposite.h"
#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/gl/gl.h>
#include <gst/rtp/rtp.h>    // parse RTP headers and extensions
#include <math.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>      // for std::cerr
#include <algorithm>     // for std::clamp


// static member definition
VideoComposite *VideoComposite::s_instance = nullptr;

// bus watch callback used in run_pipeline; reports errors/warnings from
// elements (GL shader compile issues typically show up here).
static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer /*user_data*/)
{
    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        GError *err = nullptr;
        gchar *dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        g_printerr("[gst] ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
        if (dbg) {
            g_printerr("[gst] Debug info: %s\n", dbg);
            g_free(dbg);
        }
        g_error_free(err);
        break;
    }
    case GST_MESSAGE_WARNING: {
        GError *err = nullptr;
        gchar *dbg = nullptr;
        gst_message_parse_warning(msg, &err, &dbg);
        g_printerr("[gst] WARNING from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
        if (dbg) {
            g_printerr("[gst] Debug info: %s\n", dbg);
            g_free(dbg);
        }
        g_error_free(err);
        break;
    }
    default:
        break;
    }
    return TRUE; /* keep bus watch alive */
}

// simple quaternion/matrix helpers ------------------------------------------------
VideoComposite::Quaternion VideoComposite::quat_inverse(const VideoComposite::Quaternion &q)
{
    VideoComposite::Quaternion r;
    r.w = q.w;
    r.x = -q.x;
    r.y = -q.y;
    r.z = -q.z;
    return r;
}

VideoComposite::Quaternion VideoComposite::quat_mult(const VideoComposite::Quaternion &a,
                                            const VideoComposite::Quaternion &b)
{
    VideoComposite::Quaternion r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

// convert quaternion to yaw/pitch/roll following Arduino ZYX convention
void VideoComposite::ypr_from_quat(const Quaternion &q, float &yaw, float &pitch, float &roll)
{
    // reference implementation from vid_stab/cpp/stab_math.hpp
    float w = q.w;
    float x = q.x;
    float y = q.y;
    float z = q.z;

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w*x + y*z);
    float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (w*y - z*x);
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(M_PI/2.0f, sinp);
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w*z + x*y);
    float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// 3x3 matrix utilities ------------------------------------------------------
void VideoComposite::make_K(float w, float h, float hfov_deg, float outK[9])
{
    float fx = 0.5f * w / tanf(hfov_deg * (M_PI/180.0f) * 0.5f);
    float fy = fx;
    float cx = 0.5f * w;
    float cy = 0.5f * h;
    outK[0] = fx;  outK[1] = 0.0f; outK[2] = cx;
    outK[3] = 0.0f; outK[4] = fy;  outK[5] = cy;
    outK[6] = 0.0f; outK[7] = 0.0f; outK[8] = 1.0f;
}

void VideoComposite::mult3x3(const float a[9], const float b[9], float out[9])
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out[r*3 + c] = a[r*3+0]*b[0*3+c] + a[r*3+1]*b[1*3+c] + a[r*3+2]*b[2*3+c];
        }
    }
}

void VideoComposite::transpose3x3(const float a[9], float out[9])
{
    out[0] = a[0]; out[1] = a[3]; out[2] = a[6];
    out[3] = a[1]; out[4] = a[4]; out[5] = a[7];
    out[6] = a[2]; out[7] = a[5]; out[8] = a[8];
}

bool VideoComposite::invert3x3(const float m[9], float out[9])
{
    float det = m[0]*(m[4]*m[8] - m[5]*m[7])
              - m[1]*(m[3]*m[8] - m[5]*m[6])
              + m[2]*(m[3]*m[7] - m[4]*m[6]);
    if (fabs(det) < 1e-9f)
        return false;
    float invdet = 1.0f / det;
    out[0] =  (m[4]*m[8] - m[5]*m[7]) * invdet;
    out[1] = -(m[1]*m[8] - m[2]*m[7]) * invdet;
    out[2] =  (m[1]*m[5] - m[2]*m[4]) * invdet;
    out[3] = -(m[3]*m[8] - m[5]*m[6]) * invdet;
    out[4] =  (m[0]*m[8] - m[2]*m[6]) * invdet;
    out[5] = -(m[0]*m[5] - m[2]*m[3]) * invdet;
    out[6] =  (m[3]*m[7] - m[4]*m[6]) * invdet;
    out[7] = -(m[0]*m[7] - m[1]*m[6]) * invdet;
    out[8] =  (m[0]*m[4] - m[1]*m[3]) * invdet;
    return true;
}

bool VideoComposite::homography_is_safe(const float Hinv[9], float w, float h)
{
    // sample a 3x3 grid spanning the image
    float xs[3] = {0.0f, (w - 1.0f) * 0.5f, w - 1.0f};
    float ys[3] = {0.0f, (h - 1.0f) * 0.5f, h - 1.0f};
    float limit = 5.0f * std::max(w, h);

    for (int iy = 0; iy < 3; ++iy) {
        for (int ix = 0; ix < 3; ++ix) {
            float x = xs[ix];
            float y = ys[iy];
            // multiply Hinv by [x y 1]^T
            float qx = Hinv[0]*x + Hinv[1]*y + Hinv[2];
            float qy = Hinv[3]*x + Hinv[4]*y + Hinv[5];
            float qz = Hinv[6]*x + Hinv[7]*y + Hinv[8];
            if (fabs(qz) < 1e-2f)
                return false;
            float uvx = qx / qz;
            float uvy = qy / qz;
            if (fabs(uvx) > limit || fabs(uvy) > limit)
                return false;
        }
    }
    return true;
}

// probe that reads one‑byte RFC5285 extension id=1 carrying 64-bit microsecond
// timestamp and prints it alongside sequence number.
static GstPadProbeReturn rtp_timestamp_probe(GstPad *pad, GstPadProbeInfo *info,
                                             gpointer /*user_data*/)
{
    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf)
        return GST_PAD_PROBE_OK;

    GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
    if (gst_rtp_buffer_map(buf, GST_MAP_READ, &rtp)) {
        guint16 seq = gst_rtp_buffer_get_seq(&rtp);
        guint32 ts = gst_rtp_buffer_get_timestamp(&rtp);
        gboolean ext = gst_rtp_buffer_get_extension(&rtp);
        if (ext) {
            guint16 profile;
            gpointer data;
            guint wordlen;
            if (gst_rtp_buffer_get_extension_data(&rtp, &profile, &data, &wordlen)) {
                if (profile == 0xBEDE && wordlen * 4 >= 12) {
                    guint8 *extdata = static_cast<guint8 *>(data);
                    guint64 ts_us;
                    memcpy(&ts_us, extdata + 1, sizeof(ts_us));
                    ts_us = GUINT64_FROM_BE(ts_us);
                    g_print("[rtp] seq=%u ts=%u ts_us=%" G_GUINT64_FORMAT "\n",
                            seq, ts, ts_us);
                }
            }
        }
        gst_rtp_buffer_unmap(&rtp);
    }
    return GST_PAD_PROBE_OK;
}

VideoComposite::VideoComposite(const std::string &shaderPath,
                                   const std::vector<int> &ports)
    : pipeline(nullptr), mix_element(nullptr), live_k1(0.3f), live_zoom(1.1f),
      live_w(0.0f), live_h(0.0f),
      live_h00(1.0f), live_h01(0.0f), live_h02(0.0f),
      live_h10(0.0f), live_h11(1.0f), live_h12(0.0f),
      live_h20(0.0f), live_h21(0.0f), live_h22(1.0f),
      // default to four incoming UDP streams; we add a separate background
      // videotestsrc below rather than counting it here.
      num_src(0), uniforms(nullptr), stab(), branch_active(), quat_() {
    // copy port list and derive source count
    video_ports = ports;
    num_src = static_cast<int>(video_ports.size());
    stab.assign(num_src, nullptr);
    branch_active.assign(num_src, false);

    // compute camera matrices used for homography (defaults from the
    // header are used; you can expose setters if you need to change them)
    make_K(cam_w, cam_h, cam_hfov_deg, K);
    invert3x3(K, Kinv);
    // FLU -> OpenCV conversion hard‑coded
    Rflu2cv_mat[0] = 0.0f;  Rflu2cv_mat[1] = -1.0f; Rflu2cv_mat[2] = 0.0f;
    Rflu2cv_mat[3] = 0.0f;  Rflu2cv_mat[4] =  0.0f; Rflu2cv_mat[5] = -1.0f;
    Rflu2cv_mat[6] = 1.0f;  Rflu2cv_mat[7] =  0.0f; Rflu2cv_mat[8] =  0.0f;
    // identity fallback homography
    for (int i = 0; i < 9; ++i)
        last_good_Hinv[i] = (i % 4 == 0) ? 1.0f : 0.0f;

    // load shader file
    std::ifstream in(shaderPath);
    if (!in) {
        std::string msg = "failed to open shader file '" + shaderPath + "'";
        // log unconditionally so we see the bad path even if the exception is
        // accidentally swallowed further up the call chain.
        std::cerr << msg << "\n";
        throw std::runtime_error(msg);
    }
    std::ostringstream buf;
    buf << in.rdbuf();
    shader_code = buf.str();
    if (shader_code.empty()) {
        std::string msg = "shader file '" + shaderPath + "' is empty";
        std::cerr << msg << "\n";
        throw std::runtime_error(msg);
    }
    /* allocate and clear vector to match num_src; entries beyond
       num_src are not used */
    stab.assign(num_src, nullptr);

    /* prepare the uniforms structure once and reuse it; include every
       value the shader might reference so that the structure can be updated
       in one place later.  the matrix defaults correspond to identity. */
    // start quaternion data as identity rotation
    float r00 = 1.0f, r01 = 0.0f, r02 = 0.0f;
    float r10 = 0.0f, r11 = 1.0f, r12 = 0.0f;
    float r20 = 0.0f, r21 = 0.0f, r22 = 1.0f;
    float ir00 = r00, ir01 = r10, ir02 = r20;
    float ir10 = r01, ir11 = r11, ir12 = r21;
    float ir20 = r02, ir21 = r12, ir22 = r22;

    uniforms = gst_structure_new("uniforms",
                                 "k1", G_TYPE_FLOAT, live_k1,
                                 "zoom", G_TYPE_FLOAT, live_zoom,
                                 "w", G_TYPE_FLOAT, live_w,
                                 "h", G_TYPE_FLOAT, live_h,
                                 "h00", G_TYPE_FLOAT, live_h00,
                                 "h01", G_TYPE_FLOAT, live_h01,
                                 "h02", G_TYPE_FLOAT, live_h02,
                                 "h10", G_TYPE_FLOAT, live_h10,
                                 "h11", G_TYPE_FLOAT, live_h11,
                                 "h12", G_TYPE_FLOAT, live_h12,
                                 "h20", G_TYPE_FLOAT, live_h20,
                                 "h21", G_TYPE_FLOAT, live_h21,
                                 "h22", G_TYPE_FLOAT, live_h22,
                                 "r00", G_TYPE_FLOAT, r00,
                                 "r01", G_TYPE_FLOAT, r01,
                                 "r02", G_TYPE_FLOAT, r02,
                                 "r10", G_TYPE_FLOAT, r10,
                                 "r11", G_TYPE_FLOAT, r11,
                                 "r12", G_TYPE_FLOAT, r12,
                                 "r20", G_TYPE_FLOAT, r20,
                                 "r21", G_TYPE_FLOAT, r21,
                                 "r22", G_TYPE_FLOAT, r22,
                                 "ir00", G_TYPE_FLOAT, ir00,
                                 "ir01", G_TYPE_FLOAT, ir01,
                                 "ir02", G_TYPE_FLOAT, ir02,
                                 "ir10", G_TYPE_FLOAT, ir10,
                                 "ir11", G_TYPE_FLOAT, ir11,
                                 "ir12", G_TYPE_FLOAT, ir12,
                                 "ir20", G_TYPE_FLOAT, ir20,
                                 "ir21", G_TYPE_FLOAT, ir21,
                                 "ir22", G_TYPE_FLOAT, ir22,
                                 NULL);
}

VideoComposite::~VideoComposite() {
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        /* unref only the entries we actually created */
        for (int i = 0; i < num_src && i < (int)stab.size(); i++) {
            if (stab[i]) gst_object_unref(stab[i]);
        }
        gst_object_unref(pipeline);
    }
    if (uniforms) {
        gst_structure_free(uniforms);
        uniforms = nullptr;
    }
}

void VideoComposite::start() {
    gst_init(nullptr, nullptr);

#ifdef __APPLE__
    // store instance in case gst_macos_main fails to forward our user_data
    s_instance = this;
    gst_macos_main((GstMainFunc)VideoComposite::run_pipeline, 0, nullptr, this);
#else
    run_pipeline(this);
#endif
}

// setter helpers -----------------------------------------------------------

void VideoComposite::setUniforms(float k1,
                                    float zoom,
                                    float w,
                                    float h,
                                    float h00, float h01, float h02,
                                    float h10, float h11, float h12,
                                    float h20, float h21, float h22) {
    live_k1 = k1;
    live_zoom = zoom;
    live_w = w;
    live_h = h;
    live_h00 = h00; live_h01 = h01; live_h02 = h02;
    live_h10 = h10; live_h11 = h11; live_h12 = h12;
    live_h20 = h20; live_h21 = h21; live_h22 = h22;

    // the caller provides the inverse-homography entries; just echo them
    std::cout << "[VideoComposite] homography Hinv:\n"
              << "  " << h00 << " " << h01 << " " << h02 << "\n"
              << "  " << h10 << " " << h11 << " " << h12 << "\n"
              << "  " << h20 << " " << h21 << " " << h22 << "\n";

    if (uniforms) {
        gst_structure_set(uniforms,
                          "k1", G_TYPE_FLOAT, live_k1,
                          "zoom", G_TYPE_FLOAT, live_zoom,
                          "w", G_TYPE_FLOAT, live_w,
                          "h", G_TYPE_FLOAT, live_h,
                          "h00", G_TYPE_FLOAT, live_h00,
                          "h01", G_TYPE_FLOAT, live_h01,
                          "h02", G_TYPE_FLOAT, live_h02,
                          "h10", G_TYPE_FLOAT, live_h10,
                          "h11", G_TYPE_FLOAT, live_h11,
                          "h12", G_TYPE_FLOAT, live_h12,
                          "h20", G_TYPE_FLOAT, live_h20,
                          "h21", G_TYPE_FLOAT, live_h21,
                          "h22", G_TYPE_FLOAT, live_h22,
                          NULL);
    } else {
        uniforms = gst_structure_new("uniforms",
                                     "k1", G_TYPE_FLOAT, live_k1,
                                     "zoom", G_TYPE_FLOAT, live_zoom,
                                     "w", G_TYPE_FLOAT, live_w,
                                     "h", G_TYPE_FLOAT, live_h,
                                     "h00", G_TYPE_FLOAT, live_h00,
                                     "h01", G_TYPE_FLOAT, live_h01,
                                     "h02", G_TYPE_FLOAT, live_h02,
                                     "h10", G_TYPE_FLOAT, live_h10,
                                     "h11", G_TYPE_FLOAT, live_h11,
                                     "h12", G_TYPE_FLOAT, live_h12,
                                     "h20", G_TYPE_FLOAT, live_h20,
                                     "h21", G_TYPE_FLOAT, live_h21,
                                     "h22", G_TYPE_FLOAT, live_h22,
                                     NULL);
    }
}

// called by BuoyNode callback; pulls quaternion components out of the
// protobuf message, updates our internal copy, and print them so we know the
// callback is firing.
void VideoComposite::updateQuaternion(const buoy_proto::IMU_proto &msg) {
    quat_.w = msg.quat_w();
    quat_.x = msg.quat_x();
    quat_.y = msg.quat_y();
    quat_.z = msg.quat_z();

    // std::cout << "[VideoComposite] updateQuaternion called: "
    //           << "w=" << quat_.w << " x=" << quat_.x
    //           << " y=" << quat_.y << " z=" << quat_.z << "\n";
}

// static callbacks ----------------------------------------------------------

/* pad probe that runs once for each buffer passing through the mixer source.
   We use it to update fake IMU state and propagate uniforms/transforms. */

GstPadProbeReturn VideoComposite::imu_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                               gpointer user_data) {
    if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER))
        return GST_PAD_PROBE_OK;
    VideoComposite *self = static_cast<VideoComposite *>(user_data);

    // ensure we have a reference quaternion (captured on first buffer)
    if (!self->have_ref) {
        self->quat_ref = self->quat_;
        self->have_ref = true;
        std::cout << "[VideoComposite] captured reference orientation\n";
    }

    // compute relative rotation q_rel = q_cur * inv(q_ref)
    Quaternion q_ref_inv = quat_inverse(self->quat_ref);
    Quaternion q_rel = quat_mult(self->quat_, q_ref_inv);

    float yaw, pitch, roll;
    ypr_from_quat(q_rel, yaw, pitch, roll);

    // compute corrective pitch/roll (negative of measured tilt)
    const float GAIN = 1.0f;
    const float MAX_TILT_RAD = static_cast<float>(35.0 * M_PI / 180.0);
    float pitch_c = std::clamp(-GAIN * pitch, -MAX_TILT_RAD, MAX_TILT_RAD);
    float roll_c  = std::clamp(-GAIN * roll,  -MAX_TILT_RAD, MAX_TILT_RAD);

    // apply smoothing if requested (alpha<1), mirroring Python version
    if (self->corr_smooth_alpha < 1.0f) {
        // move filtered values towards current by factor alpha
        self->corr_pitch_filt += self->corr_smooth_alpha * (pitch_c - self->corr_pitch_filt);
        self->corr_roll_filt  += self->corr_smooth_alpha * (roll_c  - self->corr_roll_filt);
        pitch_c = self->corr_pitch_filt;
        roll_c  = self->corr_roll_filt;
    } else {
        // keep filter state up-to-date so changes to alpha are smooth
        self->corr_pitch_filt = pitch_c;
        self->corr_roll_filt  = roll_c;
    }

    // build 3x3 rotation matrix for R_corr = Ry(pitch_c)*Rx(roll_c)
    float cp = cosf(pitch_c);
    float sp = sinf(pitch_c);
    float cr = cosf(roll_c);
    float sr = sinf(roll_c);
    float R_corr[9];
    R_corr[0] = cp;
    R_corr[1] = sp * sr;
    R_corr[2] = sp * cr;
    R_corr[3] = 0.0f;
    R_corr[4] = cr;
    R_corr[5] = -sr;
    R_corr[6] = -sp;
    R_corr[7] = cp * sr;
    R_corr[8] = cp * cr;

    // convert to OpenCV coord system and form homography
    float R_corr_inv[9];
    transpose3x3(R_corr, R_corr_inv);
    float temp[9];
    mult3x3(self->Rflu2cv_mat, R_corr_inv, temp);
    float R_stab_cv[9];
    float Rflu2cv_T[9];
    transpose3x3(self->Rflu2cv_mat, Rflu2cv_T);
    mult3x3(temp, Rflu2cv_T, R_stab_cv);

    float H[9];
    mult3x3(self->K, R_stab_cv, temp);
    mult3x3(temp, self->Kinv, H);

    float Hinv[9];
    bool valid = invert3x3(H, Hinv);
    if (valid) {
        // basic finite check plus homography safety
        for (int i = 0; i < 9; ++i) {
            if (!std::isfinite(Hinv[i])) {
                valid = false;
                break;
            }
        }
        if (valid && !homography_is_safe(Hinv, self->cam_w, self->cam_h))
            valid = false;
    }
    if (!valid) {
        // fall back to last good matrix (initialized to identity)
        memcpy(Hinv, self->last_good_Hinv, sizeof(Hinv));
    } else {
        memcpy(self->last_good_Hinv, Hinv, sizeof(Hinv));
    }

    // set the uniforms (keep k1/zoom constant for now)
    float k1 = self->live_k1;
    float zoom = self->live_zoom;
    self->setUniforms(k1, zoom,
                      self->cam_w, self->cam_h,
                      Hinv[0], Hinv[1], Hinv[2],
                      Hinv[3], Hinv[4], Hinv[5],
                      Hinv[6], Hinv[7], Hinv[8]);

    /* push updated uniforms into all shader elements */
    for (int i = 0; i < self->num_src; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
        if (shader) {
            g_object_set(shader, "uniforms", self->uniforms, NULL);
            gst_object_unref(shader);
        }
    }

    /* keep stabilization elements centered (no translation) */
    for (int i = 0; i < self->num_src && i < (int)self->stab.size(); i++) {
        if (self->stab[i]) {
            g_object_set(self->stab[i],
                         "translation-x", 0.0f,
                         "translation-y", 0.0f,
                         NULL);
        }
    }
    return GST_PAD_PROBE_OK;
}

void *VideoComposite::run_pipeline(gpointer user_data) {
    VideoComposite *self = static_cast<VideoComposite *>(user_data);
    if (!self) {
        // fallback to static instance (macOS behaviour can be weird)
        self = s_instance;
    }
    if (!self) {
        g_printerr("run_pipeline: user_data is NULL and no fallback instance\n");
        return NULL;
    }
    g_print("run_pipeline: self=%p\n", (void*)self);
    GMainLoop *loop;
    GError *error = NULL;

    /* create pipeline and elements manually so the number/resolution/position
       of mixer sinks can be changed at runtime. */
    self->pipeline = gst_pipeline_new("video_pipeline");
    if (!self->pipeline) {
        g_printerr("run_pipeline: failed to create pipeline\n");
        return NULL;
    }

    GstElement *mix = gst_element_factory_make("glvideomixer", "mix");
    GstElement *convert = gst_element_factory_make("glcolorconvert", "conv");
    GstElement *fps = gst_element_factory_make("fpsdisplaysink", "fps");
    GstElement *videosink = gst_element_factory_make("autovideosink", "vsink");

    if (!mix || !convert || !fps || !videosink) {
        g_printerr("run_pipeline: could not create core elements\n");
        return NULL;
    }

    g_object_set(mix, "background", 1, NULL);
    g_object_set(fps,
                 "video-sink", videosink,
                 "text-overlay", TRUE,
                 "sync", TRUE,
                 NULL);

    /* add all elements to pipeline (videosink must be owned to avoid premature
       free when fpsdisplaysink grabs it) */
    gst_bin_add_many(GST_BIN(self->pipeline), mix, convert, fps, videosink, NULL);
    if (!gst_element_link_many(mix, convert, fps, NULL)) {
        g_printerr("run_pipeline: failed to link core elements\n");
        return NULL;
    }
    /* no need to link fps->videosink; fpsdisplaysink handles its child sink */

    /* we'll use mix pointer below when attaching branches */

    /* dynamic configuration for mixer sinks; change values or number here
       we want a 2x2 grid for the UDP sources and a full‑screen background test
       pattern.  The background element is created separately below. */
    struct SinkLayout { gint xpos, ypos, width, height; };
    std::vector<SinkLayout> layouts;
    layouts.reserve(self->num_src);
    for (int i = 0; i < self->num_src; ++i) {
        /* compute side-by-side horizontal layout */
        SinkLayout l;
        l.xpos  = i * 1920;  // each stream offset horizontally
        l.ypos  = 0;
        l.width = 1920;
        l.height= 1080;
        layouts.push_back(l);
    }
    /* background resolution should cover all sources horizontally */
    gint bg_width = (self->num_src > 0 ? self->num_src : 1) * 1920;
    gint bg_height = 1080; // match individual source height

    /* add pad probe to mixer src so we can update IMU/uniforms per frame */
    GstPad *probe_pad = gst_element_get_static_pad(mix, "src");
    if (probe_pad) {
        gst_pad_add_probe(probe_pad, GST_PAD_PROBE_TYPE_BUFFER,
                          imu_probe_cb, self, NULL);
        gst_object_unref(probe_pad);
    }

    /*--- background branch ------------------------------------------------*/
    {
        GstElement *bg_src   = gst_element_factory_make("videotestsrc", "bg_src");
        GstElement *bg_caps  = gst_element_factory_make("capsfilter", "bg_caps");
        GstElement *bg_glup  = gst_element_factory_make("glupload", "bg_glup");
        if (!bg_src || !bg_caps || !bg_glup) {
            g_printerr("run_pipeline: failed to create background elements\n");
            return NULL;
        }
        g_object_set(bg_src, "pattern", 0, "is-live", TRUE, NULL);
        GstCaps *bgcaps = gst_caps_from_string(
            (std::ostringstream() << "video/x-raw,width=" << bg_width
             << ",height=" << bg_height << ",framerate=60/1").str().c_str());
        g_object_set(bg_caps, "caps", bgcaps, NULL);
        gst_caps_unref(bgcaps);

        gst_bin_add_many(GST_BIN(self->pipeline), bg_src, bg_caps, bg_glup, NULL);
        if (!gst_element_link_many(bg_src, bg_caps, bg_glup, NULL)) {
            g_printerr("run_pipeline: failed to link background branch\n");
            return NULL;
        }

        GstPad *bg_sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
        if (!bg_sinkpad) {
            g_printerr("run_pipeline: could not get mixer pad for background\n");
            return NULL;
        }
        g_object_set(bg_sinkpad,
                     "xpos", 0,
                     "ypos", 0,
                     "width", bg_width,
                     "height", bg_height,
                     "zorder", 0,
                     NULL);
        GstPad *bg_srcpad = gst_element_get_static_pad(bg_glup, "src");
        if (gst_pad_link(bg_srcpad, bg_sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link background to mixer\n");
            return NULL;
        }
        gst_object_unref(bg_srcpad);
        gst_object_unref(bg_sinkpad);
    }

    /* create udp source branches and hook them to mixer pads */
    for (int i = 0; i < self->num_src; ++i) {
        GstElement *udpsrc    = gst_element_factory_make("udpsrc", NULL);
        GstElement *jitter    = gst_element_factory_make("rtpjitterbuffer", NULL);
        GstElement *depay     = gst_element_factory_make("rtph264depay", NULL);
        if (jitter) {
            GstPad *j_src = gst_element_get_static_pad(jitter, "src");
            if (j_src) {
                gst_pad_add_probe(j_src, GST_PAD_PROBE_TYPE_BUFFER,
                                  rtp_timestamp_probe, NULL, NULL);
                gst_object_unref(j_src);
            }
        }
        GstElement *parse     = gst_element_factory_make("h264parse", NULL);
        GstElement *dec       = gst_element_factory_make("avdec_h264", NULL);
        GstElement *conv      = gst_element_factory_make("videoconvert", NULL);
        GstElement *rate      = gst_element_factory_make("videorate", NULL);
        GstElement *capsf     = gst_element_factory_make("capsfilter", NULL);
        GstElement *glup      = gst_element_factory_make("glupload", NULL);
        char shader_nm[16], stab_nm[16];
        snprintf(shader_nm, sizeof(shader_nm), "lens%d", i);
        snprintf(stab_nm, sizeof(stab_nm), "stab%d", i);
        GstElement *shader    = gst_element_factory_make("glshader", shader_nm);
        GstElement *stab      = gst_element_factory_make("gltransformation", stab_nm);
        GstElement *queue     = gst_element_factory_make("queue", NULL);

        if (!udpsrc || !jitter || !depay || !parse || !dec || !conv ||
            !rate || !capsf || !glup || !shader || !stab || !queue) {
            g_printerr("run_pipeline: failed to create udp branch %d elements\n", i);
            return NULL;
        }

        /* configure udp source and RTP caps */
        int port = (i < (int)self->video_ports.size() ? self->video_ports[i] : 0);
        if (port > 0) {
            g_object_set(udpsrc, "port", port, NULL);
        } else {
            g_printerr("run_pipeline: invalid port for branch %d\n", i);
        }
        GstCaps *rtpcaps = gst_caps_from_string(
            "application/x-rtp,media=video,encoding-name=H264,"
            "payload=96,clock-rate=90000");
        g_object_set(udpsrc, "caps", rtpcaps, NULL);
        gst_caps_unref(rtpcaps);
        // increase latency if there are issues, higher latency will cause frame
        // drop and desync if source is delayed or doesn't exist.
        g_object_set(jitter, "latency", 0, "drop-on-latency", TRUE, NULL);

        GstCaps *caps2 = gst_caps_from_string("video/x-raw,framerate=60/1");
        g_object_set(capsf, "caps", caps2, NULL);
        gst_caps_unref(caps2);

        gst_bin_add_many(GST_BIN(self->pipeline),
                         udpsrc, jitter, depay, parse, dec,
                         conv, rate, capsf, glup, shader, stab, queue, NULL);

        if (!gst_element_link_many(udpsrc, jitter, depay, parse, dec,
                                   conv, rate, capsf, glup, shader, stab,
                                   queue, NULL)) {
            g_printerr("run_pipeline: udp branch %d link failed\n", i);
            return NULL;
        }

        GstPad *sinkpad = gst_element_request_pad_simple(mix, "sink_%u");
        if (!sinkpad) {
            g_printerr("run_pipeline: could not get mixer pad for branch %d\n", i);
            return NULL;
        }

        /* apply dynamic layout
           (could be reassigned while running by reconfiguring pad properties) */
        const SinkLayout &l = layouts[i];
        g_object_set(sinkpad,
                     "xpos",   l.xpos,
                     "ypos",   l.ypos,
                     "width",  l.width,
                     "height", l.height,
                     NULL);

        /* link the *queue* output, not the stab element; queue is last in
           the branch chain so it provides a stable src pad for the mixer */
        GstPad *srcpad = gst_element_get_static_pad(queue, "src");
        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("run_pipeline: failed to link branch %d to mixer\n", i);
            return NULL;
        }
        gst_object_unref(srcpad);
        /* set ordering so background stays at bottom */
        g_object_set(sinkpad, "zorder", i + 1, NULL);
        gst_object_unref(sinkpad);
    }

    /* shaders and stabs can still be configured by name; use num_src */
    for (int i = 0; i < self->num_src; i++) {
        char name[16];
        snprintf(name, sizeof(name), "lens%d", i);
        GstElement *shader_elem = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
        if (shader_elem) {
            g_object_set(shader_elem, "fragment", self->shader_code.c_str(), NULL);
            /* no signals; uniforms will be updated via pad probe each frame */
            gst_object_unref(shader_elem);
        }
    }

    /* update stab pointers vector to match current num_src */
    self->stab.assign(self->num_src, nullptr);
    for (int i = 0; i < self->num_src; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "stab%d", i);
        self->stab[i] = gst_bin_get_by_name(GST_BIN(self->pipeline), name);
    }

    /* install bus watch so we can see errors/warnings from any element (e.g.
       shader compilation failures).  message callback will continue running
       until the pipeline is torn down. */
    {
        GstBus *bus = gst_element_get_bus(self->pipeline);
        gst_bus_add_watch(bus, bus_call, nullptr);
        gst_object_unref(bus);
    }

    gst_element_set_state(self->pipeline, GST_STATE_PLAYING);

    loop = g_main_loop_new(NULL, FALSE);
    g_print("Running Fake IMU Stabilization Test at 4K (4x 1080p). Press Ctrl+C to stop.\n");
    g_main_loop_run(loop);

    gst_element_set_state(self->pipeline, GST_STATE_NULL);
    for (auto *s : self->stab) {
        if (s) gst_object_unref(s);
    }
    gst_object_unref(self->pipeline);
    g_main_loop_unref(loop);

    return NULL;
}
