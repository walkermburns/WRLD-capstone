#!/usr/bin/env python3
"""
gst_warp_imu.py

GStreamer (GL) homography warp driven by Arduino IMU quaternion stream.

Includes:
- Thread safety: never set glshader uniforms inside the pad-probe (streaming thread).
  The probe only computes the latest Hinv; a GLib timer applies uniforms on the main thread.
- Startup stability: until IMU is "ready" and we have a valid Hinv, we force identity.
- Reliable debug prints: time-based throttle.
- FLU conventions:
    IMU is FLU: +x forward, +y left, +z up.
    Map FLU -> OpenCV camera coords (x right, y down, z forward):
        cv_x = -flu_y, cv_y = -flu_z, cv_z = flu_x
- Stabilization convention (your working one):
    R_rel = R_cur_flu * R_ref_flu.inv()

NEW:
- Smooth correction (EMA) on pitch/roll correction angles to reduce jitter.

Run:
  /usr/bin/python3 gst_warp_imu.py
"""
import sys
import math
import time
import threading
from collections import deque

import numpy as np
import serial
from scipy.spatial.transform import Rotation as R, Slerp

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib


# -----------------------------
# User settings
# -----------------------------
PORT = "/dev/ttyUSB0"
BAUD = 115200

ARDUINO_PRINT_HZ = 100
BUFFER_TIME_S = 6.0

VIDEO_DELAY_S = 1.0
RES_W, RES_H = 640, 480
HFOV_DEG = 50.0

USE_QUAT_CONJUGATE = False

GAIN = 1.0
MAX_TILT_RAD = math.radians(35.0)

# Smoothing (EMA) on correction angles:
# smaller => smoother but more lag; larger => snappier but more jitter
SMOOTH_ALPHA = 0.95

PRINT_EVERY_S = 0.5

PIPELINE_DESC = f"""
v4l2src device=/dev/video0 io-mode=2 !
video/x-raw,format=YUY2,width={RES_W},height={RES_H},framerate=30/1 !
queue max-size-buffers=1 leaky=downstream !
videoconvert ! video/x-raw,format=RGBA !
glupload !
glshader name=stab qos=true !
glimagesink sync=false
"""


# -----------------------------
# IMU buffer (perf_counter time base)
# -----------------------------
quat_buffer = deque(maxlen=int(BUFFER_TIME_S * ARDUINO_PRINT_HZ))  # (t_perf, [x,y,z,w])
buf_lock = threading.Lock()
stop_flag = threading.Event()


def parse_imu_line(line: str):
    """
    Expected CSV from Arduino:
      roll_deg,pitch_deg,yaw_deg,w,x,y,z
    Returns quaternion in SciPy order [x,y,z,w].
    """
    parts = line.split(",")
    if len(parts) < 7:
        raise ValueError("Not enough fields")
    w = float(parts[3])
    x = float(parts[4])
    y = float(parts[5])
    z = float(parts[6])

    if USE_QUAT_CONJUGATE:
        x, y, z = -x, -y, -z

    return [x, y, z, w]


def imu_thread_fn():
    ser = serial.Serial(PORT, BAUD, timeout=1.0 / 75.0)
    time.sleep(0.5)
    ser.reset_input_buffer()
    time.sleep(0.2)

    while not stop_flag.is_set():
        try:
            line = ser.read_until().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            q_xyzw = parse_imu_line(line)
            t = time.perf_counter()
            with buf_lock:
                quat_buffer.append((t, q_xyzw))
        except Exception:
            continue

    try:
        ser.close()
    except Exception:
        pass


def get_rot_at_perf_time(t_query: float) -> R:
    with buf_lock:
        if len(quat_buffer) < 2:
            raise RuntimeError("Need more IMU samples")
        times = np.array([t for (t, _) in quat_buffer], dtype=float)
        quats = np.array([q for (_, q) in quat_buffer], dtype=float)

    rots = R.from_quat(quats)
    slerp = Slerp(times, rots)
    return slerp([t_query])[0]


# -----------------------------
# Time mapping: pipeline running-time -> perf_counter
# -----------------------------
def gst_running_time_s(pipeline: Gst.Pipeline) -> float:
    clock = pipeline.get_clock()
    if clock is None:
        return 0.0
    now = clock.get_time()
    base = pipeline.get_base_time()
    if now < base:
        return 0.0
    return float(now - base) / float(Gst.SECOND)


# -----------------------------
# Uniform helper (force float typing)
# -----------------------------
def make_uniforms_from_Hinv(Hinv: np.ndarray) -> Gst.Structure:
    uni = (
        f"uniforms, "
        f"w=(float){RES_W}, h=(float){RES_H}, "
        f"h00=(float){Hinv[0,0]}, h01=(float){Hinv[0,1]}, h02=(float){Hinv[0,2]}, "
        f"h10=(float){Hinv[1,0]}, h11=(float){Hinv[1,1]}, h12=(float){Hinv[1,2]}, "
        f"h20=(float){Hinv[2,0]}, h21=(float){Hinv[2,1]}, h22=(float){Hinv[2,2]}"
    )
    return Gst.Structure.new_from_string(uni)


def homography_is_safe(Hinv: np.ndarray, w: int, h: int) -> bool:
    xs = np.linspace(0.0, w - 1.0, 3)
    ys = np.linspace(0.0, h - 1.0, 3)
    pts = np.array([[x, y, 1.0] for y in ys for x in xs], dtype=float).T  # 3x9

    q = Hinv @ pts
    z = q[2, :]
    if np.any(np.abs(z) < 1e-2):
        return False

    uv = (q[:2, :] / z).T
    if np.any(np.abs(uv) > 5.0 * max(w, h)):
        return False
    return True


def on_bus_message(bus: Gst.Bus, msg: Gst.Message, loop: GLib.MainLoop):
    t = msg.type
    if t == Gst.MessageType.ERROR:
        err, debug = msg.parse_error()
        print(f"[Gst ERROR] {err}", file=sys.stderr)
        if debug:
            print(f"[Gst DEBUG] {debug}", file=sys.stderr)
        loop.quit()
    elif t == Gst.MessageType.EOS:
        loop.quit()


def main():
    Gst.init(None)

    th = threading.Thread(target=imu_thread_fn, daemon=True)
    th.start()

    # Camera intrinsics
    fx = 0.5 * RES_W / math.tan(math.radians(HFOV_DEG) / 2.0)
    fy = fx
    cx = RES_W / 2.0
    cy = RES_H / 2.0
    K = np.array([[fx, 0.0, cx],
                  [0.0, fy, cy],
                  [0.0, 0.0, 1.0]], dtype=float)
    Kinv = np.linalg.inv(K)

    # FLU -> OpenCV camera coords
    R_flu_to_cv = np.array([[0, -1,  0],
                            [0,  0, -1],
                            [1,  0,  0]], dtype=float)

    pipe = Gst.parse_launch(PIPELINE_DESC)
    glshader = pipe.get_by_name("stab")
    if glshader is None:
        raise RuntimeError("Could not find glshader named 'stab'")

    with open("../shaders/warp.frag", "r", encoding="utf-8") as f:
        glshader.set_property("fragment", f.read())
    glshader.set_property("update-shader", True)

    loop = GLib.MainLoop()
    bus = pipe.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus_message, loop)

    # Shared state for safe uniform updates
    lock = threading.Lock()
    latest_Hinv = np.eye(3, dtype=float)
    have_update = True

    # Time sync: perf ≈ running + offset
    tsync_initialized = False
    offset_perf_minus_running = 0.0

    # Reference orientation (set once)
    R_ref_flu = None

    # Smoothing state (EMA) for correction angles
    pitch_f = 0.0
    roll_f = 0.0

    # Keep last good matrix
    last_good_Hinv = np.eye(3, dtype=float)
    bad_count = 0

    # Debug print throttle
    last_print_wall = 0.0

    sinkpad = glshader.get_static_pad("sink")
    if sinkpad is None:
        raise RuntimeError("Could not get glshader sink pad")

    def probe_cb(pad, info):
        nonlocal tsync_initialized, offset_perf_minus_running
        nonlocal R_ref_flu, latest_Hinv, have_update
        nonlocal last_good_Hinv, bad_count
        nonlocal last_print_wall
        nonlocal pitch_f, roll_f

        buf = info.get_buffer()
        if buf is None:
            return Gst.PadProbeReturn.OK

        # Frame time in pipeline running-time
        if buf.pts != Gst.CLOCK_TIME_NONE:
            rt_frame = float(buf.pts) / float(Gst.SECOND)
        else:
            rt_frame = gst_running_time_s(pipe)
            if rt_frame <= 0.0:
                return Gst.PadProbeReturn.OK

        # Initialize mapping once
        if not tsync_initialized:
            rt_now = gst_running_time_s(pipe)
            if rt_now <= 0.0:
                return Gst.PadProbeReturn.OK
            offset_perf_minus_running = time.perf_counter() - rt_now
            tsync_initialized = True

        t_frame_perf = rt_frame + offset_perf_minus_running
        t_target_perf = t_frame_perf - VIDEO_DELAY_S

        # Ensure IMU buffer is ready + query times are in-range
        with buf_lock:
            if len(quat_buffer) < 20:
                with lock:
                    latest_Hinv = np.eye(3, dtype=float)
                    have_update = True
                return Gst.PadProbeReturn.OK
            t_min = quat_buffer[0][0]
            t_max = quat_buffer[-1][0]

        if not (t_min <= t_frame_perf <= t_max and t_min <= t_target_perf <= t_max):
            with lock:
                latest_Hinv = last_good_Hinv
                have_update = True
            return Gst.PadProbeReturn.OK

        try:
            R_cur_flu = get_rot_at_perf_time(t_frame_perf)

            # Set reference ONCE
            if R_ref_flu is None:
                R_ref_flu = get_rot_at_perf_time(t_target_perf)

        except Exception:
            return Gst.PadProbeReturn.OK

        # Debug: absolute angles
        now = time.time()
        if now - last_print_wall >= PRINT_EVERY_S:
            ypr_cur = R_cur_flu.as_euler("zyx", degrees=True)
            print(
                f"[CUR] yaw={ypr_cur[0]: .2f} pitch={ypr_cur[1]: .2f} roll={ypr_cur[2]: .2f} deg",
                flush=True
            )
            last_print_wall = now

        # Your working relative rotation convention
        R_rel = R_cur_flu * R_ref_flu.inv()

        # Roll/pitch-only correction (zero yaw)
        yaw, pitch, roll = R_rel.as_euler("zyx", degrees=False)

        # Smooth the correction angles (EMA)
        pitch_f = (1.0 - SMOOTH_ALPHA) * pitch_f + SMOOTH_ALPHA * pitch
        roll_f  = (1.0 - SMOOTH_ALPHA) * roll_f  + SMOOTH_ALPHA * roll

        pitch_c = float(np.clip(GAIN * pitch_f, -MAX_TILT_RAD, MAX_TILT_RAD))
        roll_c  = float(np.clip(GAIN * roll_f,  -MAX_TILT_RAD, MAX_TILT_RAD))

        R_stab_flu = R.from_euler("zyx", [0.0, pitch_c, roll_c], degrees=False)

        # Homography from rotation
        R_stab_cv = R_flu_to_cv @ R_stab_flu.as_matrix() @ R_flu_to_cv.T
        H = K @ R_stab_cv @ Kinv

        # Inverse mapping (dest->src)
        try:
            Hinv = np.linalg.inv(H)
        except np.linalg.LinAlgError:
            bad_count += 1
            Hinv = last_good_Hinv

        # Reject invalid / unsafe matrices
        if not np.isfinite(Hinv).all():
            bad_count += 1
            Hinv = last_good_Hinv
        elif not homography_is_safe(Hinv, RES_W, RES_H):
            bad_count += 1
            Hinv = last_good_Hinv
        else:
            last_good_Hinv = Hinv

        if bad_count > 0 and bad_count % 120 == 0:
            print(f"[warn] rejected frames so far: {bad_count}", flush=True)

        with lock:
            latest_Hinv = Hinv
            have_update = True

        return Gst.PadProbeReturn.OK

    sinkpad.add_probe(Gst.PadProbeType.BUFFER, probe_cb)

    # Main-thread timer: apply uniforms safely
    def apply_uniforms():
        nonlocal have_update
        with lock:
            if not have_update:
                return True
            H = latest_Hinv.copy()
            have_update = False

        glshader.set_property("uniforms", make_uniforms_from_Hinv(H))
        return True

    GLib.timeout_add(33, apply_uniforms)

    pipe.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        stop_flag.set()
        pipe.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()