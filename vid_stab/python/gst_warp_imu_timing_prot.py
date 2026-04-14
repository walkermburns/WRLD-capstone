#!/usr/bin/env python3
"""
gst_warp_imu_timing_proof.py

Proof-of-concept for the original shader timing issue.

This script intentionally keeps the architecture similar to the original project:
- live GStreamer video pipeline
- glshader doing the warp
- IMU arriving asynchronously on a serial thread
- homography computed from IMU using frame timestamps

Two apply modes:
1) idle_coalesced
   - mimics the old gst_warp_imu.py behavior
   - probe computes Hinv, but uniform application is deferred onto the GLib main thread
   - if multiple buffers arrive before the apply callback runs, intermediate Hinv values are overwritten

2) probe_direct
   - same live shader pipeline
   - same probe timing
   - but uniform application happens immediately in the probe for that buffer

If probe_direct reduces jitter relative to idle_coalesced, the likely issue in the
original project is shader-uniform timing/coalescing rather than the stabilization math.

Controls:
    q      quit
    m      toggle apply mode
    [ ]    decrease/increase ROT_SMOOTH_ALPHA
    - =    decrease/increase COMPENSATE_FRAC
    p      toggle console prints

Expected IMU serial format:
    sample_idx,mcu_time_us,qw,qx,qy,qz
"""

import sys
import math
import time
import threading
from collections import deque
from pathlib import Path

import numpy as np
import serial
from scipy.spatial.transform import Rotation as R, Slerp

import os
os.environ["GST_GL_API"] = "opengl"

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib


# -----------------------------
# User settings
# -----------------------------
PORT = "/dev/ttyUSB0"
BAUD = 460800

IMU_RATE_HZ_EST = 100
BUFFER_TIME_S = 6.0

RES_W, RES_H = 640, 480
HFOV_DEG = 50.0

USE_QUAT_CONJUGATE = False

GAIN = 1.0
MAX_TILT_RAD = math.radians(180.0)

ROT_SMOOTH_ALPHA = 1.0
IMU_LEAD_S = 0.0
PREDICT_MAX_S = 0.12
COMPENSATE_FRAC = 1.0

OMEGA_ALPHA = 0.25
OMEGA_CLAMP = 25.0

PRINT_EVERY_S = 1.0

APPLY_MODE = "idle_coalesced"   # "idle_coalesced" or "probe_direct"
PAUSE_PRINT = False

PIPELINE_DESC = f"""
v4l2src device=/dev/video0 io-mode=2 !
video/x-raw,format=YUY2,width={RES_W},height={RES_H},framerate=30/1 !
queue max-size-buffers=1 leaky=downstream !
videoconvert ! video/x-raw,format=RGBA !
glupload !
glshader name=stab qos=true !
queue name=qdisp max-size-buffers=1 leaky=downstream !
glimagesink name=vsink sync=false
"""


# -----------------------------
# IMU buffer (perf_counter time base)
# -----------------------------
quat_buffer = deque(maxlen=int(BUFFER_TIME_S * IMU_RATE_HZ_EST))  # (t_perf, [x,y,z,w])
buf_lock = threading.Lock()
stop_flag = threading.Event()


def parse_imu_line(line: str):
    """
    Expected compact format:
        sample_idx,mcu_time_us,qw,qx,qy,qz
    Returns scipy-order quaternion [x,y,z,w].
    """
    parts = line.split(",")
    if len(parts) < 6:
        raise ValueError("Not enough fields")

    _sample_idx = int(parts[0])
    _mcu_time_us = int(parts[1])

    w = float(parts[2])
    x = float(parts[3])
    y = float(parts[4])
    z = float(parts[5])

    if USE_QUAT_CONJUGATE:
        x, y, z = -x, -y, -z

    q = np.array([x, y, z, w], dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Degenerate quaternion")
    return q / n


def imu_thread_fn():
    """
    Serial thread:
    - parse quaternions
    - host-timestamp them
    - enforce quaternion sign continuity to make interpolation/debugging stable
    """
    last_q = None

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

            if last_q is not None and np.dot(last_q, q_xyzw) < 0.0:
                q_xyzw = -q_xyzw
            last_q = q_xyzw

            t = time.perf_counter()
            with buf_lock:
                quat_buffer.append((t, q_xyzw))
        except Exception:
            continue

    try:
        ser.close()
    except Exception:
        pass


def get_rot_at_perf_time_local(t_query: float) -> R:
    """Local SLERP between two nearest IMU samples."""
    with buf_lock:
        if len(quat_buffer) < 2:
            raise RuntimeError("Need more IMU samples")

        t0 = quat_buffer[0][0]
        tN = quat_buffer[-1][0]
        if t_query < t0 or t_query > tN:
            raise RuntimeError("Query out of range")

        hi = 0
        while hi < len(quat_buffer) and quat_buffer[hi][0] < t_query:
            hi += 1

        if hi == 0:
            return R.from_quat(quat_buffer[0][1])
        if hi >= len(quat_buffer):
            return R.from_quat(quat_buffer[-1][1])

        ta, qa = quat_buffer[hi - 1]
        tb, qb = quat_buffer[hi]
        u = (t_query - ta) / (tb - ta + 1e-12)
        u = float(np.clip(u, 0.0, 1.0))

    Ra = R.from_quat(qa)
    Rb = R.from_quat(qb)
    s = Slerp([0.0, 1.0], R.from_quat([Ra.as_quat(), Rb.as_quat()]))
    return s([u])[0]


def slerp_two(Ra: R, Rb: R, alpha: float) -> R:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    if alpha <= 0.0:
        return Ra
    if alpha >= 1.0:
        return Rb
    s = Slerp([0.0, 1.0], R.from_quat([Ra.as_quat(), Rb.as_quat()]))
    return s([alpha])[0]


def gst_running_time_s(pipeline: Gst.Pipeline) -> float:
    clock = pipeline.get_clock()
    if clock is None:
        return 0.0
    now = clock.get_time()
    base = pipeline.get_base_time()
    if now < base:
        return 0.0
    return float(now - base) / float(Gst.SECOND)


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
    pts = np.array([[x, y, 1.0] for y in ys for x in xs], dtype=float).T

    q = Hinv @ pts
    z = q[2, :]
    if np.any(np.abs(z) < 1e-2):
        return False

    uv = (q[:2, :] / z).T
    if np.any(~np.isfinite(uv)):
        return False
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
    global APPLY_MODE, ROT_SMOOTH_ALPHA, COMPENSATE_FRAC, PAUSE_PRINT

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

    # FLU -> OpenCV camera coords (x right, y down, z forward)
    R_flu_to_cv = np.array([[0, -1,  0],
                            [0,  0, -1],
                            [1,  0,  0]], dtype=float)

    pipe = Gst.parse_launch(PIPELINE_DESC)
    glshader = pipe.get_by_name("stab")
    if glshader is None:
        raise RuntimeError("Could not find glshader named 'stab'")

    vsink = pipe.get_by_name("vsink")
    if vsink is None:
        raise RuntimeError("Could not find glimagesink named 'vsink'")

    # Load shader
    shader_path = Path(__file__).resolve().parent / "warp.frag"
    if not shader_path.exists():
        # fall back to sibling shader folder layout like old project
        shader_path = Path(__file__).resolve().parent.parent / "shaders" / "warp.frag"

    frag_src = shader_path.read_text(encoding="utf-8")
    if len(frag_src) < 50:
        raise RuntimeError(f"Shader looks empty: {shader_path}")
    glshader.set_property("fragment", frag_src)
    glshader.set_property("update-shader", True)

    loop = GLib.MainLoop()
    bus = pipe.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus_message, loop)

    # perf ≈ running + offset
    tsync_initialized = False
    offset_perf_minus_running = 0.0

    # Correction smoothing
    R_corr_filt = R.identity()

    # Display latency estimate (smoothed)
    extra_lat_s = 0.0
    extra_lat_alpha = 0.2

    # Filtered omega predictor state
    omega_filt = np.zeros(3, dtype=float)
    last_omega_time = 0.0

    last_print = 0.0

    # Stats specifically about coalescing / apply path
    apply_lock = threading.Lock()
    pending_Hinv = np.eye(3, dtype=float)
    apply_scheduled = False
    coalesced_overwrites = 0
    applied_callbacks = 0
    direct_applies = 0

    def rot_at_time_with_filtered_prediction(t_query: float) -> R:
        nonlocal omega_filt, last_omega_time

        with buf_lock:
            if len(quat_buffer) < 3:
                raise RuntimeError("Need more IMU samples")
            t_min = quat_buffer[0][0]
            t_max = quat_buffer[-1][0]
            t1, q1 = quat_buffer[-2]
            t2, q2 = quat_buffer[-1]

        if t_query < t_min:
            raise RuntimeError("Query too old")

        # update omega_filt once per new sample
        if t2 != last_omega_time and t2 > t1:
            R1 = R.from_quat(q1)
            R2 = R.from_quat(q2)
            dR = R2 * R1.inv()
            omega_inst = dR.as_rotvec() / max(float(t2 - t1), 1e-6)
            omega_inst = np.clip(omega_inst, -OMEGA_CLAMP, OMEGA_CLAMP)
            omega_filt = (1.0 - OMEGA_ALPHA) * omega_filt + OMEGA_ALPHA * omega_inst
            last_omega_time = t2

        if t_query <= t_max:
            return get_rot_at_perf_time_local(t_query)

        dt_pred = float(np.clip(t_query - t_max, 0.0, PREDICT_MAX_S))
        R2 = R.from_quat(q2)
        return R2 * R.from_rotvec(omega_filt * dt_pred)

    def _apply_pending_uniforms():
        """
        Runs on GLib main thread.
        Applies the latest pending Hinv (coalesced).
        """
        nonlocal apply_scheduled, pending_Hinv, applied_callbacks
        with apply_lock:
            H = pending_Hinv.copy()
            apply_scheduled = False

        glshader.set_property("uniforms", make_uniforms_from_Hinv(H))
        applied_callbacks += 1
        return False

    def schedule_apply(Hinv: np.ndarray):
        """
        Called from streaming thread (pad probe).
        Coalesce: if one callback is already queued, overwrite the pending matrix.
        """
        nonlocal apply_scheduled, pending_Hinv, coalesced_overwrites
        with apply_lock:
            if apply_scheduled:
                coalesced_overwrites += 1
            pending_Hinv = Hinv
            if apply_scheduled:
                return
            apply_scheduled = True

        GLib.idle_add(_apply_pending_uniforms, priority=GLib.PRIORITY_HIGH)

    # Late probe: estimate display-side age
    vsink_pad = vsink.get_static_pad("sink")
    if vsink_pad is None:
        raise RuntimeError("Could not get vsink sink pad")

    def late_probe_cb(pad, info):
        nonlocal extra_lat_s
        buf = info.get_buffer()
        if buf is None or buf.pts == Gst.CLOCK_TIME_NONE:
            return Gst.PadProbeReturn.OK

        rt_frame_late = float(buf.pts) / float(Gst.SECOND)
        rt_now_late = gst_running_time_s(pipe)
        if rt_now_late <= 0.0:
            return Gst.PadProbeReturn.OK

        age_display = max(0.0, rt_now_late - rt_frame_late)
        age_display = float(np.clip(age_display, 0.0, 0.25))
        extra_lat_s = (1.0 - extra_lat_alpha) * extra_lat_s + extra_lat_alpha * age_display
        return Gst.PadProbeReturn.OK

    vsink_pad.add_probe(Gst.PadProbeType.BUFFER, late_probe_cb)

    # Early probe: compute Hinv from the current buffer's PTS
    sinkpad = glshader.get_static_pad("sink")
    if sinkpad is None:
        raise RuntimeError("Could not get glshader sink pad")

    last_good_Hinv = np.eye(3, dtype=float)

    def probe_cb(pad, info):
        nonlocal tsync_initialized, offset_perf_minus_running
        nonlocal R_corr_filt
        nonlocal last_print
        nonlocal last_good_Hinv
        nonlocal direct_applies
        global APPLY_MODE

        buf = info.get_buffer()
        if buf is None:
            return Gst.PadProbeReturn.OK

        if buf.pts != Gst.CLOCK_TIME_NONE:
            rt_frame = float(buf.pts) / float(Gst.SECOND)
        else:
            rt_frame = gst_running_time_s(pipe)
            if rt_frame <= 0.0:
                return Gst.PadProbeReturn.OK

        if not tsync_initialized:
            rt_now0 = gst_running_time_s(pipe)
            if rt_now0 <= 0.0:
                return Gst.PadProbeReturn.OK
            offset_perf_minus_running = time.perf_counter() - rt_now0
            tsync_initialized = True

        rt_now = gst_running_time_s(pipe)
        if rt_now <= 0.0:
            return Gst.PadProbeReturn.OK

        lat_probe = float(np.clip(max(0.0, rt_now - rt_frame), 0.0, 0.25))
        lead_total = COMPENSATE_FRAC * extra_lat_s + IMU_LEAD_S
        t_frame_perf = rt_frame + offset_perf_minus_running + lead_total

        with buf_lock:
            if len(quat_buffer) < 20:
                Hinv = np.eye(3, dtype=float)
                if APPLY_MODE == "probe_direct":
                    glshader.set_property("uniforms", make_uniforms_from_Hinv(Hinv))
                    direct_applies += 1
                else:
                    schedule_apply(Hinv)
                return Gst.PadProbeReturn.OK

            t_min = quat_buffer[0][0]

        if t_frame_perf < t_min:
            Hinv = last_good_Hinv
            if APPLY_MODE == "probe_direct":
                glshader.set_property("uniforms", make_uniforms_from_Hinv(Hinv))
                direct_applies += 1
            else:
                schedule_apply(Hinv)
            return Gst.PadProbeReturn.OK

        try:
            R_cur_flu = rot_at_time_with_filtered_prediction(t_frame_perf)
        except Exception:
            Hinv = last_good_Hinv
            if APPLY_MODE == "probe_direct":
                glshader.set_property("uniforms", make_uniforms_from_Hinv(Hinv))
                direct_applies += 1
            else:
                schedule_apply(Hinv)
            return Gst.PadProbeReturn.OK

        if not PAUSE_PRINT:
            now_wall = time.time()
            if now_wall - last_print >= PRINT_EVERY_S:
                ypr_cur = R_cur_flu.as_euler("zyx", degrees=True)
                print(
                    f"[{APPLY_MODE}] "
                    f"yaw={ypr_cur[0]: .2f} pitch={ypr_cur[1]: .2f} roll={ypr_cur[2]: .2f} deg  "
                    f"probe_age={lat_probe*1000:.1f} ms  "
                    f"display_age={extra_lat_s*1000:.1f} ms  "
                    f"lead_used={lead_total*1000:.1f} ms  "
                    f"coalesced_overwrites={coalesced_overwrites}  "
                    f"idle_applies={applied_callbacks}  "
                    f"direct_applies={direct_applies}",
                    flush=True
                )
                last_print = now_wall

        # Pure horizon lock: ignore yaw completely, cancel only absolute pitch/roll
        yaw_cur, pitch_cur, roll_cur = R_cur_flu.as_euler("zyx", degrees=False)

        pitch_c = float(np.clip(-GAIN * pitch_cur, -MAX_TILT_RAD, MAX_TILT_RAD))
        roll_c  = float(np.clip(-GAIN * roll_cur,  -MAX_TILT_RAD, MAX_TILT_RAD))

        R_corr = R.from_euler("zyx", [0.0, pitch_c, roll_c], degrees=False)

        if ROT_SMOOTH_ALPHA < 1.0:
            R_corr_filt = slerp_two(R_corr_filt, R_corr, ROT_SMOOTH_ALPHA)
            R_use = R_corr_filt
        else:
            R_use = R_corr

        # Build homography. Your shader expects inverse homography
        R_stab_cv = R_flu_to_cv @ R_use.inv().as_matrix() @ R_flu_to_cv.T
        H = K @ R_stab_cv @ Kinv

        try:
            Hinv = np.linalg.inv(H)
        except np.linalg.LinAlgError:
            Hinv = last_good_Hinv

        if (not np.isfinite(Hinv).all()) or (not homography_is_safe(Hinv, RES_W, RES_H)):
            Hinv = last_good_Hinv
        else:
            last_good_Hinv = Hinv

        # ---- This is the thing being proven ----
        if APPLY_MODE == "probe_direct":
            # Apply immediately for this buffer, keeping the live shader pipeline intact.
            # This is still the same architecture, but without the deferred/coalesced handoff.
            try:
                glshader.set_property("uniforms", make_uniforms_from_Hinv(Hinv))
                direct_applies += 1
            except Exception:
                # fallback if the platform objects to direct probe-thread property update
                schedule_apply(Hinv)
        else:
            # Original-style deferred/coalesced apply
            schedule_apply(Hinv)

        return Gst.PadProbeReturn.OK

    sinkpad.add_probe(Gst.PadProbeType.BUFFER, probe_cb)

    # Key polling on main thread
    def key_tick():
        global APPLY_MODE, ROT_SMOOTH_ALPHA, COMPENSATE_FRAC, PAUSE_PRINT
        import select

        # Non-blocking stdin key read
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == "q":
                loop.quit()
                return False
            elif ch == "m":
                APPLY_MODE = "probe_direct" if APPLY_MODE == "idle_coalesced" else "idle_coalesced"
                print(f"Switched APPLY_MODE to {APPLY_MODE}", flush=True)
            elif ch == "[":
                ROT_SMOOTH_ALPHA = max(0.05, ROT_SMOOTH_ALPHA - 0.05)
                print(f"ROT_SMOOTH_ALPHA = {ROT_SMOOTH_ALPHA:.2f}", flush=True)
            elif ch == "]":
                ROT_SMOOTH_ALPHA = min(1.0, ROT_SMOOTH_ALPHA + 0.05)
                print(f"ROT_SMOOTH_ALPHA = {ROT_SMOOTH_ALPHA:.2f}", flush=True)
            elif ch == "-":
                COMPENSATE_FRAC = max(0.0, COMPENSATE_FRAC - 0.1)
                print(f"COMPENSATE_FRAC = {COMPENSATE_FRAC:.2f}", flush=True)
            elif ch == "=":
                COMPENSATE_FRAC = min(1.5, COMPENSATE_FRAC + 0.1)
                print(f"COMPENSATE_FRAC = {COMPENSATE_FRAC:.2f}", flush=True)
            elif ch == "p":
                PAUSE_PRINT = not PAUSE_PRINT
                print(f"PAUSE_PRINT = {PAUSE_PRINT}", flush=True)
        return True

    # Need stdin nonblocking-ish for key polling in terminal
    try:
        import tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
    except Exception:
        old_settings = None

    GLib.timeout_add(30, key_tick)

    # Start pipeline
    pipe.set_state(Gst.State.PLAYING)
    print(
        "Controls: q quit | m toggle apply mode | [ ] alpha | - = compensate_frac | p pause prints",
        flush=True,
    )

    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag.set()
        pipe.set_state(Gst.State.NULL)
        if old_settings is not None:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass


if __name__ == "__main__":
    main()