import cv2 as cv
import math
import serial
import time
import threading
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

PORT = "/dev/ttyUSB0"
BAUD = 115200

arduino_rate_hz = 50
camera_fps = 30
buffer_time_length_s = 4

video_delay_s = 1

quat_buffer = deque(maxlen = buffer_time_length_s * arduino_rate_hz)
ypr_buffer = deque(maxlen = buffer_time_length_s * arduino_rate_hz)
frame_buffer = deque(maxlen = buffer_time_length_s * camera_fps)

stop_threads_flag = threading.Event()

def parse_imu_data(serial_line: str):
    quat_data = {}
    ypr_data = {}
    split_array = serial_line.split(',')
    
    ypr_data['roll_deg'] = float(split_array[0])
    ypr_data['pitch_deg'] = float(split_array[1])
    ypr_data['yaw_deg'] = float(split_array[2])
    quat_data = [float(split_array[4]), float(split_array[5]), float(split_array[6]), float(split_array[3])]
    # quat_data['w'] = float(split_array[3])
    # quat_data['x'] = float(split_array[4])
    # quat_data['y'] = float(split_array[5])
    # quat_data['z'] = float(split_array[6])
    
    return quat_data, ypr_data

def read_arduino_serial(ser: serial.Serial):
    while not stop_threads_flag.is_set():
        try:
            line = ser.read_until().decode("utf-8", errors="replace").strip()
            # print(line)
            if not line:
                # print('Invalid line')
                continue
            
            # if line:
            #     print(line)
            
            quat_data, ypr_data = parse_imu_data(line)
            
            t = time.perf_counter()
            
            quat_buffer.append([t, quat_data])
            ypr_buffer.append([t, ypr_data])
            
            # print(len(ypr_buffer))
            
        except Exception:
            continue
    
def get_quat_at_query_time(t_query):
    
    time_array = np.array([t[0] for t in quat_buffer], dtype=float)
    quat_array = np.array([q[1] for q in quat_buffer], dtype=float)
    # quats_x = quats_dict['x']
    # print(';assdf')
    rots = R.from_quat(quat_array)
    slerp_operation = Slerp(time_array, rots)
    return slerp_operation([t_query])[0]


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1.0/75.0)
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

    hfov_deg = 50
    fx = 0.5 * res_w / math.tan(math.radians(hfov_deg) / 2)
    fy = fx
    cx = res_w / 2
    cy = res_h / 2
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=float)
    Kinv = np.linalg.inv(K)   

    R_ref_flu = None

    arduino_thread = threading.Thread(target=read_arduino_serial, args=(ser,), daemon=True)
    arduino_thread.start()

    frame_buffer_to_display = None
    first_iter = True

    u = np.array([0.0, 0.0, 1.0])

    R_flu_to_cv = np.array([[0, -1,  0],
                            [0,  0, -1],
                            [1,  0,  0]], dtype=float)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            t_now = time.perf_counter()
            t_frame = t_now
            frame_buffer.append([t_frame, frame])

            t_target = t_now - video_delay_s

            if first_iter and len(quat_buffer) >= 2:
                R_ref_flu = get_quat_at_query_time(t_target)
                first_iter = False

            # Pop frames up to the target time
            while frame_buffer and frame_buffer[0][0] <= t_target:
                frame_buffer_to_display = frame_buffer.popleft()

            if (frame_buffer_to_display is not None) and (not first_iter) and len(quat_buffer) >= 2:
                display_frame_time = frame_buffer_to_display[0]
                display_frame = frame_buffer_to_display[1]

                R_cur_flu = get_quat_at_query_time(display_frame_time)
                
                euler_angs_deg = R_cur_flu.as_euler("zyx", degrees=True)

                R_rel = R_ref_flu * R_cur_flu.inv()

                v = R_rel.apply(u)
                R_tilt, _ = R.align_vectors([u], [v])   # maps v -> u
                R_stab_flu = R_tilt

                R_stab_cv = R_flu_to_cv @ R_stab_flu.as_matrix() @ R_flu_to_cv.T
                H = K @ R_stab_cv @ Kinv

                warped_frame = cv.warpPerspective(
                    display_frame, H, (res_w, res_h),
                    flags=cv.INTER_LINEAR,
                    borderMode=cv.BORDER_REPLICATE
                )

                side_by_side = np.hstack((display_frame, warped_frame))

                cv.putText(side_by_side, "Original", (20, 40),
                           cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                cv.putText(side_by_side, "Roll/Pitch Stabilized", (res_w + 20, 40),
                           cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                cv.putText(side_by_side, f"roll: {euler_angs_deg[2]: .2f} [deg]", (20, 90),
                           cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                cv.putText(side_by_side, f"pitch: {euler_angs_deg[1]: .2f} [deg]", (20, 140),
                           cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                cv.putText(side_by_side, f"yaw: {euler_angs_deg[0]: .2f} [deg]", (20, 190),
                           cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                

                cv.imshow("Original | Warped", side_by_side)

            if cv.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        stop_threads_flag.set()
        cap.release()
        ser.close()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()
