#include "MathHelpers.h"
#include <algorithm>
#include <cmath>

namespace MathHelpers {

Quaternion quat_inverse(const Quaternion &q)
{
    Quaternion r;
    r.w = q.w;
    r.x = -q.x;
    r.y = -q.y;
    r.z = -q.z;
    return r;
}

Quaternion quat_mult(const Quaternion &a, const Quaternion &b)
{
    Quaternion r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

void ypr_from_quat(const Quaternion &q, float &yaw, float &pitch, float &roll)
{
    float w = q.w;
    float x = q.x;
    float y = q.y;
    float z = q.z;

    float sinr_cosp = 2.0f * (w*x + y*z);
    float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w*y - z*x);
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(M_PI/2.0f, sinp);
    else
        pitch = std::asin(sinp);

    float siny_cosp = 2.0f * (w*z + x*y);
    float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void make_K(float w, float h, float hfov_deg, float outK[9])
{
    float fx = 0.5f * w / tanf(hfov_deg * (M_PI/180.0f) * 0.5f);
    float fy = fx;
    float cx = 0.5f * w;
    float cy = 0.5f * h;
    outK[0] = fx;  outK[1] = 0.0f; outK[2] = cx;
    outK[3] = 0.0f; outK[4] = fy;  outK[5] = cy;
    outK[6] = 0.0f; outK[7] = 0.0f; outK[8] = 1.0f;
}

void mult3x3(const float a[9], const float b[9], float out[9])
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out[r*3 + c] = a[r*3+0]*b[0*3+c] + a[r*3+1]*b[1*3+c] + a[r*3+2]*b[2*3+c];
        }
    }
}

void transpose3x3(const float a[9], float out[9])
{
    out[0] = a[0]; out[1] = a[3]; out[2] = a[6];
    out[3] = a[1]; out[4] = a[4]; out[5] = a[7];
    out[6] = a[2]; out[7] = a[5]; out[8] = a[8];
}

bool invert3x3(const float m[9], float out[9])
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

bool homography_is_safe(const float Hinv[9], float w, float h)
{
    float xs[3] = {0.0f, (w - 1.0f) * 0.5f, w - 1.0f};
    float ys[3] = {0.0f, (h - 1.0f) * 0.5f, h - 1.0f};
    float limit = 5.0f * std::max(w, h);

    for (int iy = 0; iy < 3; ++iy) {
        for (int ix = 0; ix < 3; ++ix) {
            float x = xs[ix];
            float y = ys[iy];
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


// additional helpers moved from BuoyNode

void init_camera_matrices(float w, float h, float hfov_deg,
                           float K[9], float Kinv[9],
                           float Rflu2cv[9], float last_good_Hinv[9]) {
    make_K(w, h, hfov_deg, K);
    invert3x3(K, Kinv);
    Rflu2cv[0] = 0.0f;  Rflu2cv[1] = -1.0f; Rflu2cv[2] = 0.0f;
    Rflu2cv[3] = 0.0f;  Rflu2cv[4] =  0.0f; Rflu2cv[5] = -1.0f;
    Rflu2cv[6] = 1.0f;  Rflu2cv[7] =  0.0f; Rflu2cv[8] =  0.0f;
    for (int i = 0; i < 9; ++i)
        last_good_Hinv[i] = (i % 4 == 0) ? 1.0f : 0.0f;
}

bool compute_homography_from_quat(const Quaternion &quat_cur,
                                  Quaternion &quat_ref,
                                  bool &have_ref,
                                  float corr_alpha,
                                  float &corr_pitch_filt,
                                  float &corr_roll_filt,
                                  const float Rflu2cv[9],
                                  const float K[9],
                                  const float Kinv[9],
                                  float cam_w,
                                  float cam_h,
                                  float outHinv[9]) {
    if (!have_ref) {
        quat_ref = quat_cur;
        have_ref = true;
        return false;
    }

    Quaternion q_ref_inv = quat_inverse(quat_ref);
    Quaternion q_rel = quat_mult(quat_cur, q_ref_inv);
    float yaw, pitch, roll;
    ypr_from_quat(q_rel, yaw, pitch, roll);

    const float GAIN = 1.0f;
    const float MAX_TILT_RAD = static_cast<float>(35.0 * M_PI / 180.0);
    float pitch_c = std::clamp(-GAIN * pitch, -MAX_TILT_RAD, MAX_TILT_RAD);
    float roll_c  = std::clamp(-GAIN * roll,  -MAX_TILT_RAD, MAX_TILT_RAD);

    if (corr_alpha < 1.0f) {
        corr_pitch_filt += corr_alpha * (pitch_c - corr_pitch_filt);
        corr_roll_filt  += corr_alpha * (roll_c  - corr_roll_filt);
        pitch_c = corr_pitch_filt;
        roll_c  = corr_roll_filt;
    } else {
        corr_pitch_filt = pitch_c;
        corr_roll_filt  = roll_c;
    }

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

    float R_corr_inv[9];
    transpose3x3(R_corr, R_corr_inv);
    float temp[9];
    mult3x3(Rflu2cv, R_corr_inv, temp);
    float R_stab_cv[9];
    float Rflu2cv_T[9];
    transpose3x3(Rflu2cv, Rflu2cv_T);
    mult3x3(temp, Rflu2cv_T, R_stab_cv);

    float H[9];
    mult3x3(K, R_stab_cv, temp);
    mult3x3(temp, Kinv, H);

    float Hinv[9];
    bool valid = invert3x3(H, Hinv);
    if (valid) {
        for (int i = 0; i < 9; ++i) {
            if (!std::isfinite(Hinv[i])) {
                valid = false;
                break;
            }
        }
        if (valid && !homography_is_safe(Hinv, cam_w, cam_h))
            valid = false;
    }
    if (!valid)
        return false;
    for (int i = 0; i < 9; ++i)
        outHinv[i] = Hinv[i];
    return true;
}

} // namespace MathHelpers
