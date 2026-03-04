#ifndef MATHHELPERS_H
#define MATHHELPERS_H

#include <cmath>

namespace MathHelpers {

// simple quaternion type matching the one previously inside VideoComposite
struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

Quaternion quat_inverse(const Quaternion &q);
Quaternion quat_mult(const Quaternion &a, const Quaternion &b);

// convert quaternion to yaw/pitch/roll following Arduino ZYX convention
void ypr_from_quat(const Quaternion &q, float &yaw, float &pitch, float &roll);

// 3x3 matrix utilities (row-major arrays)
void make_K(float w, float h, float hfov_deg, float outK[9]);
void mult3x3(const float a[9], const float b[9], float out[9]);
void transpose3x3(const float a[9], float out[9]);
bool invert3x3(const float m[9], float out[9]);

// validity check for inverse homography matrix (same logic as gst_warp_imu.py)
bool homography_is_safe(const float Hinv[9], float w, float h);

// initialize camera intrinsics / helper matrices used for homography
// calculation.  the arrays K, Kinv, Rflu2cv, and last_good_Hinv must point to
// 9-element storage.  last_good_Hinv will be filled with identity.
void init_camera_matrices(float w, float h, float hfov_deg,
                           float K[9], float Kinv[9],
                           float Rflu2cv[9], float last_good_Hinv[9]);

// compute an inverse homography matrix based on the current quaternion
// reading and optional smoothing state.  the function updates quat_ref and
// have_ref so that callers can keep a persistent reference orientation; if
// have_ref is false it will be set and quat_ref initialized.  corr_pitch_filt
// and corr_roll_filt are used/updated for smoothing when corr_alpha < 1.
// returns true if resulting Hinv is valid (otherwise it leaves outHinv
// untouched and returns false).
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
                                  float outHinv[9]);

} // namespace MathHelpers

#endif // MATHHELPERS_H
