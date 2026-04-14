import cv2 as cv
import math
import serial
import time
import threading
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

PORT = "/dev/ttyUSB0"
BAUD = 460800

ARDUINO_RATE_HZ = 100
CAMERA_FPS = 30
BUFFER_TIME_LENGTH_S = 4.0

VIDEO_DELAY_S = 0.01
ROT_SMOOTH_ALPHA = 1.0
USE_QUAT_CONJUGATE = False
SHOW_PREVIEW = True

quat_buffer = deque(maxlen=int(BUFFER_TIME_LENGTH_S * ARDUINO_RATE_HZ))
frame_buffer = deque(maxlen=int(BUFFER_TIME_LENGTH_S * CAMERA_FPS))

buffer_lock = threading.Lock()
stop_threads_flag = threading.Event()


def normalize_vec(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v.copy()
    return v / n


def slerp_two(qa: R, qb: R, alpha: float) -> R:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    if alpha <= 0.0:
        return qa
    if alpha >= 1.0:
        return qb

    key_times = np.array([0.0, 1.0], dtype=float)
    key_rots = R.concatenate([qa, qb])
    s = Slerp(key_times, key_rots)
    return s([alpha])[0]


def rotation_align_a_to_b(a_in: np.ndarray, b_in: np.ndarray) -> R:
    """
    Return rotation that maps vector a onto vector b.
    Mirrors the C++ rotation_align_a_to_b() logic.
    """
    a = normalize_vec(a_in)
    b = normalize_vec(b_in)

    c = float(np.clip(np.dot(a, b), -1.0, 1.0))

    if c > 1.0 - 1e-8:
        return R.identity()

    if c < -1.0 + 1e-8:
        trial = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(np.dot(a, trial)) > 0.9:
            trial = np.array([0.0, 1.0, 0.0], dtype=float)
        axis = normalize_vec(np.cross(a, trial))
        return R.from_rotvec(math.pi * axis)

    v = np.cross(a, b)
    s = np.linalg.norm(v)

    vx = np.array([
        [0.0,    -v[2],  v[1]],
        [v[2],   0.0,   -v[0]],
        [-v[1],  v[0],   0.0]
    ], dtype=float)

    Rm = np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))
    return R.from_matrix(Rm)


def homography_is_safe(H: np.ndarray, w: int, h: int) -> bool:
    pts = np.array([
        [0.0, 0.0],
        [w - 1.0, 0.0],
        [0.0, h - 1.0],
        [w - 1.0, h - 1.0],
        [0.5 * w, 0.5 * h]
    ], dtype=float)

    for px, py in pts:
        x = np.array([px, py, 1.0], dtype=float)
        q = H @ x
        z = q[2]
        if abs(z) < 1e-6:
            return False
        u = q[0] / z
        v = q[1] / z
        if not np.isfinite(u) or not np.isfinite(v):
            return False
        if abs(u) > 5.0 * w or abs(v) > 5.0 * h:
            return False
    return True


def parse_imu_data(serial_line: str):
    """
    Arduino format:
    sample_idx,mcu_time_us,qw,qx,qy,qz
    """
    parts = serial_line.split(",")
    if len(parts) != 6:
        raise ValueError(f"Expected 6 fields, got {len(parts)}: {serial_line}")

    sample_idx = int(parts[0])
    mcu_time_us = int(parts[1])

    qw = float(parts[2])
    qx = float(parts[3])
    qy = float(parts[4])
    qz = float(parts[5])

    if USE_QUAT_CONJUGATE:
        qx, qy, qz = -qx, -qy, -qz

    quat_xyzw = np.array([qx, qy, qz, qw], dtype=float)
    norm = np.linalg.norm(quat_xyzw)
    if norm <= 1e-12:
        raise ValueError("Quaternion norm is zero")
    quat_xyzw /= norm

    return sample_idx, mcu_time_us, quat_xyzw


def read_arduino_serial(ser: serial.Serial):
    have_prev_q = False
    prev_q = None

    while not stop_threads_flag.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue

            sample_idx, mcu_time_us, quat_xyzw = parse_imu_data(line)

            # Enforce quaternion sign continuity, matching the C++ logic.
            if have_prev_q and np.dot(prev_q, quat_xyzw) < 0.0:
                quat_xyzw = -quat_xyzw

            prev_q = quat_xyzw.copy()
            have_prev_q = True

            t_host = time.perf_counter()

            with buffer_lock:
                quat_buffer.append((t_host, quat_xyzw, sample_idx, mcu_time_us))

        except Exception:
            continue


def get_quat_at_query_time(t_query: float):
    """
    Interpolate quaternion in host time.
    Returns:
        rot: scipy Rotation
        newest_imu_time: float
    """
    with buffer_lock:
        data = list(quat_buffer)

    if len(data) < 2:
        return None, None

    newest_imu_time = data[-1][0]

    time_array = np.array([item[0] for item in data], dtype=float)
    quat_array = np.array([item[1] for item in data], dtype=float)

    valid_idx = [0]
    for i in range(1, len(time_array)):
        if time_array[i] > time_array[valid_idx[-1]]:
            valid_idx.append(i)

    if len(valid_idx) < 2:
        return None, None

    time_array = time_array[valid_idx]
    quat_array = quat_array[valid_idx]

    if t_query <= time_array[0]:
        return R.from_quat(quat_array[0]), newest_imu_time
    if t_query >= time_array[-1]:
        return R.from_quat(quat_array[-1]), newest_imu_time

    rots = R.from_quat(quat_array)
    s = Slerp(time_array, rots)
    return s([t_query])[0], newest_imu_time


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1.0 / 200.0)
    time.sleep(0.5)
    ser.reset_input_buffer()
    time.sleep(0.5)

    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    res_w = 640
    res_h = 480
    cap.set(cv.CAP_PROP_FRAME_WIDTH, res_w)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, res_h)

    hfov_deg = 50.0
    fx = 0.5 * res_w / math.tan(math.radians(hfov_deg) / 2.0)
    fy = fx
    cx = res_w / 2.0
    cy = res_h / 2.0

    K = np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0]
    ], dtype=float)
    Kinv = np.linalg.inv(K)

    # Same FLU -> CV convention as the C++ code
    R_flu_to_cv = np.array([
        [0.0, -1.0,  0.0],
        [0.0,  0.0, -1.0],
        [1.0,  0.0,  0.0]
    ], dtype=float)

    arduino_thread = threading.Thread(
        target=read_arduino_serial,
        args=(ser,),
        daemon=True
    )
    arduino_thread.start()

    R_stab_filt = R.identity()

    loop_fps_smoothed = 0.0
    warp_ms_smoothed = 0.0
    imu_interp_ms_smoothed = 0.0
    last_print_s = time.perf_counter()

    def smooth_update(s, x, alpha=0.1):
        if s <= 0.0:
            return x
        return (1.0 - alpha) * s + alpha * x

    try:
        while True:
            loop_t0 = time.perf_counter()

            ok, frame = cap.read()
            if not ok:
                break

            t_now = time.perf_counter()
            frame_buffer.append((t_now, frame))

            t_target = t_now - VIDEO_DELAY_S
            have_frame = False
            frame_to_display = None

            while frame_buffer and frame_buffer[0][0] <= t_target:
                frame_to_display = frame_buffer.popleft()
                have_frame = True

            if not have_frame:
                if cv.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            display_frame_time, display_frame = frame_to_display

            imu_t0 = time.perf_counter()
            R_cur_flu, newest_imu_time = get_quat_at_query_time(display_frame_time)
            imu_t1 = time.perf_counter()

            if R_cur_flu is None:
                cv.imshow("stabilized_horizon_lock", display_frame)
                if cv.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            imu_interp_ms = (imu_t1 - imu_t0) * 1000.0
            imu_interp_ms_smoothed = smooth_update(imu_interp_ms_smoothed, imu_interp_ms)

            # -----------------------------------------------------------------
            # Horizon-lock logic matched to the C++ file:
            #
            # up_world = [0,0,1]
            # up_body  = R_cur_flu.conjugate() * up_world
            # up_cam   = normalize(R_flu_to_cv * up_body)
            # img_up_cam = [0,-1,0]
            # R_stab_target_cv = rotation_align_a_to_b(up_cam, img_up_cam)
            # R_stab_filt = slerp_two(R_stab_filt, R_stab_target_cv, alpha)
            # H = K * Rcv * K^-1
            # -----------------------------------------------------------------
            up_world = np.array([0.0, 0.0, 1.0], dtype=float)

            # Equivalent to Eigen quaternion conjugate acting on vector
            up_body = R_cur_flu.inv().apply(up_world)
            up_cam = normalize_vec(R_flu_to_cv @ up_body)

            img_up_cam = np.array([0.0, -1.0, 0.0], dtype=float)

            R_stab_target_cv = rotation_align_a_to_b(up_cam, img_up_cam)
            R_stab_filt = slerp_two(R_stab_filt, R_stab_target_cv, ROT_SMOOTH_ALPHA)

            Rcv = R_stab_filt.as_matrix()
            H = K @ Rcv @ Kinv

            warp_t0 = time.perf_counter()
            if not homography_is_safe(H, res_w, res_h):
                warped = display_frame.copy()
            else:
                warped = cv.warpPerspective(
                    display_frame,
                    H,
                    (res_w, res_h),
                    flags=cv.INTER_LINEAR,
                    borderMode=cv.BORDER_REPLICATE
                )
            warp_t1 = time.perf_counter()

            warp_ms = (warp_t1 - warp_t0) * 1000.0
            warp_ms_smoothed = smooth_update(warp_ms_smoothed, warp_ms)

            loop_t1 = time.perf_counter()
            loop_ms = (loop_t1 - loop_t0) * 1000.0
            loop_fps = 1000.0 / loop_ms if loop_ms > 1e-6 else 0.0
            loop_fps_smoothed = smooth_update(loop_fps_smoothed, loop_fps)

            frame_delay_ms = (t_now - display_frame_time) * 1000.0
            imu_age_ms = (t_now - newest_imu_time) * 1000.0 if newest_imu_time is not None else -1.0

            euler_angs_deg = R_cur_flu.as_euler("zyx", degrees=True)
            yaw_deg = euler_angs_deg[0]
            pitch_deg = euler_angs_deg[1]
            roll_deg = euler_angs_deg[2]

            if time.perf_counter() - last_print_s >= 1.0:
                with buffer_lock:
                    imu_buf_sz = len(quat_buffer)

                print(
                    f"[CUR] yaw={yaw_deg:.2f} pitch={pitch_deg:.2f} roll={roll_deg:.2f} deg"
                    f"  [frame_delay] {frame_delay_ms:.2f} ms"
                    f"  [imu_age] {imu_age_ms:.2f} ms"
                    f"  [loop_fps] {loop_fps_smoothed:.1f}"
                    f"  [imu_interp] {imu_interp_ms_smoothed:.2f} ms"
                    f"  [warp] {warp_ms_smoothed:.1f} ms"
                    f"  [imu_buf] {imu_buf_sz}"
                    f"  [frame_buf] {len(frame_buffer)}"
                )
                last_print_s = time.perf_counter()

            if SHOW_PREVIEW:
                cv.putText(warped, f"Loop FPS: {loop_fps_smoothed:.1f}",
                           (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv.putText(warped, f"Warp: {warp_ms_smoothed:.1f} ms",
                           (20, 60), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv.putText(warped, f"Delay: {frame_delay_ms:.1f} ms",
                           (20, 90), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv.putText(warped, f"IMU age: {imu_age_ms:.1f} ms",
                           (20, 120), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                cv.imshow("stabilized_horizon_lock", warped)
                key = cv.waitKey(1)
                if key == ord("q") or key == 27:
                    break

    finally:
        stop_threads_flag.set()
        cap.release()
        ser.close()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()