"""
WASAKA HEXAPOD - AUTONOMOUS NAVIGATION (BEST 2026)
===================================================
Sistem navigasi otonom terintegrasi dengan:
1. Multi-layer Vision Perception:
   - Canny edge density (L/C/R) dengan ambang batas adaptif
   - Optical Flow Farneback (motion parallax proximity cue)
   - Segmentasi warna lantai HSV (appearance-based obstacle detection)
   - Free-space column profile (kemudi halus di celah sempit)
   - YOLO11 Obstacle Detector (klasifikasi halangan besar vs kecil)
2. Closed-Loop Stabilitas & Proprioception:
   - IMU BNO055 heading lock (PID yaw) & tilt safety
   - Body Leveler PID (kompensasi geometrik kontur tanah tidak rata)
3. FSM Decision:
   - CRUISE -> AVOID -> BACKUP -> SCAN -> FAULT
4. UI & Monitoring:
   - OpenCV Rich Dashboard (telemetri, vision masks, PID terms, flow vector)
   - OLED 128x64 display worker
   - CSV session logging

KONTROL KEYBOARD (saat dashboard aktif):
  'q' : Keluar & shutdown servo dengan aman
  'c' : Rekalibrasi warna lantai (ground segmentation)
  'p' : Toggle Body Leveler PID ON / OFF
  'e' : Toggle Deteksi Edge (Canny) ON / OFF
  's' : Toggle Standby (berhenti & tegak) / Jalan

AUTHOR: Wasaka Robotic Team
"""

import math
import os
import platform
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Pastikan root direktori ada di sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Import modul core
from core.locomotion import Hexapod
from core.imu_sensor import HeadingSensor, wrap_pi, wait_for_start
from core.body_leveler import BodyLeveler
from core.oled_display import OledDisplay, OledWorker

# Import modul vision
from vision.camera import open_camera, CameraStream
from vision.object_detector import ObjectDetector

# Import modul internal autonomous
from apps.autonomous.perception import (
    preprocess_frame, get_roi, calculate_density,
    FlowEstimator, AdaptiveThreshold, GroundSegmenter, FreeSpaceProfile,
    fuse_blocked, decide_navigation, lerp, SessionLogger,
    COLOR_GREEN, COLOR_RED, COLOR_YELLOW, COLOR_CYAN
)
from apps.autonomous.fsm import NavFSM

IS_WINDOWS = platform.system() == "Windows"

# Konfigurasi Kamera & Persepsi
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240
ROI_TOP_RATIO = 0.5
OBSTACLE_THRESHOLD = 15.0
IGNORE_THRESHOLD   = 5.0

# Feature Toggles
USE_THREADED_CAMERA    = True
USE_OPTICAL_FLOW       = True
USE_FSM                = True
USE_IMU_SAFETY         = True
USE_ADAPTIVE_THRESHOLD = True
USE_GROUND_SEG         = True
USE_FREESPACE_PROFILE  = True
USE_OLED               = not IS_WINDOWS
USE_LOGGING            = True
USE_IMU                = True
USE_YOLO               = True

# PID Heading Lock
KP_YAW = 0.8
KI_YAW = 0.05
WZ_MAX = 0.8
I_MAX  = 0.3

# Konfigurasi Kecepatan
SPEED_FORWARD = 1.0
SPEED_STRAFE  = 1.0
SMOOTHING     = 0.3


def build_dashboard(w, h, live_cam, edges, flow_val, state, decision,
                    vx, vy, wz, fps, dL, dC, dR, blk_L, blk_C, blk_R,
                    imu_ok, heading_err, attitude, calib, nonfloor_mask=None,
                    free_boundary=None):
    """Membuat tampilan visual OpenCV dashboard lengkap."""
    dash = np.zeros((h, w, 3), dtype=np.uint8)
    dash[:] = (34, 26, 8)  # Latar belakang gelap elegan

    # 1. Live Camera Panel (Kiri Atas)
    cam_h, cam_w = 240, 320
    cam_x, cam_y = 20, 20
    if live_cam is not None:
        cv2.rectangle(dash, (cam_x - 2, cam_y - 2), (cam_x + cam_w + 2, cam_y + cam_h + 2), (122, 110, 46), 1)
        dash[cam_y:cam_y + cam_h, cam_x:cam_x + cam_w] = cv2.resize(live_cam, (cam_w, cam_h))
        cv2.putText(dash, "KAMERA UTAMA", (cam_x + 8, cam_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # 2. Edge / Canny Panel (Kanan dari Kamera)
    edge_x, edge_y = cam_x + cam_w + 20, 20
    if edges is not None:
        edge_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        dash[edge_y:edge_y + cam_h, edge_x:edge_x + cam_w] = cv2.resize(edge_bgr, (cam_w, cam_h))
        cv2.rectangle(dash, (edge_x - 2, edge_y - 2), (edge_x + cam_w + 2, edge_y + cam_h + 2), (122, 110, 46), 1)
        cv2.putText(dash, "CANNY EDGES", (edge_x + 8, edge_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # 3. Ground Segmentation Mask (Jika aktif)
    mask_x, mask_y = edge_x + cam_w + 20, 20
    if nonfloor_mask is not None:
        mask_bgr = cv2.cvtColor(nonfloor_mask, cv2.COLOR_GRAY2BGR)
        dash[mask_y:mask_y + cam_h, mask_x:mask_x + cam_w] = cv2.resize(mask_bgr, (cam_w, cam_h))
        cv2.rectangle(dash, (mask_x - 2, mask_y - 2), (mask_x + cam_w + 2, mask_y + cam_h + 2), (122, 110, 46), 1)
        cv2.putText(dash, "GROUND SEGMENTATION", (mask_x + 8, mask_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # 4. Telemetri & Status Panel (Bawah)
    stat_y = cam_y + cam_h + 30
    cv2.putText(dash, f"STATE : {state}", (cam_x, stat_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(dash, f"AKSI  : {decision}", (cam_x, stat_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if decision == "FORWARD" else (0, 255, 255), 2)
    cv2.putText(dash, f"FPS   : {fps:.1f}", (cam_x, stat_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # Densitas & Status Blocked
    col_l = (0, 0, 255) if blk_L else (0, 255, 0)
    col_c = (0, 0, 255) if blk_C else (0, 255, 0)
    col_r = (0, 0, 255) if blk_R else (0, 255, 0)
    cv2.putText(dash, f"DENSITAS L: {dL:4.1f}%", (cam_x + 200, stat_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col_l, 1)
    cv2.putText(dash, f"DENSITAS C: {dC:4.1f}%", (cam_x + 200, stat_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col_c, 1)
    cv2.putText(dash, f"DENSITAS R: {dR:4.1f}%", (cam_x + 200, stat_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col_r, 1)

    # Velocity output
    cv2.putText(dash, f"VELOCITY: vx={vx:+.2f}  vy={vy:+.2f}  wz={wz:+.2f}", (cam_x + 400, stat_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # IMU Data
    roll, pitch = attitude
    s, g, a, m = calib
    imu_str = f"IMU: Roll={roll:+5.1f}  Pitch={pitch:+5.1f}  ErrYaw={math.degrees(heading_err):+5.1f}"
    cal_str = f"CALIB: Sys={s} Gyro={g} Accel={a} Mag={m}"
    cv2.putText(dash, imu_str, (cam_x + 400, stat_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 180) if imu_ok else (100, 100, 100), 1)
    cv2.putText(dash, cal_str, (cam_x + 400, stat_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

    return dash


def run_autonomous():
    print("=" * 60)
    print("  WASAKA HEXAPOD - AUTONOMOUS NAVIGATION SYSTEM")
    print("=" * 60)

    # 1. Inisialisasi Servo Hexapod
    robot = Hexapod()
    try:
        robot.connect()
        robot.enable_torque()
        robot.stand_by()
    except Exception as e:
        print(f"[WARN] Inisialisasi servo hexapod gagal ({e}) -> lanjut mode simulasi.")

    # 2. Inisialisasi Sensor & Kontroler
    imu = HeadingSensor(enabled=USE_IMU)
    leveler = BodyLeveler(enabled=True)
    fsm = NavFSM(use_imu_safety=USE_IMU_SAFETY) if USE_FSM else None
    flow_est = FlowEstimator() if USE_OPTICAL_FLOW else None
    adapt = AdaptiveThreshold() if USE_ADAPTIVE_THRESHOLD else None
    ground_seg = GroundSegmenter() if USE_GROUND_SEG else None
    freespace = FreeSpaceProfile() if USE_FREESPACE_PROFILE else None
    logger = SessionLogger(enabled=USE_LOGGING)

    # 3. Inisialisasi Detektor Objek YOLO (Opsional)
    detector = None
    if USE_YOLO:
        try:
            detector = ObjectDetector(weights_path="models/yolo11n.onnx", conf_thresh=0.45)
            print("[INFO] YOLO11 Obstacle Detector aktif.")
        except Exception as e:
            print(f"[WARN] YOLO11 tidak dapat dimuat ({e}), navigasi beralih ke edge+flow.")

    # 4. Inisialisasi OLED
    oled_worker = None
    if USE_OLED:
        try:
            oled = OledDisplay(enabled=True, i2c=getattr(imu, "i2c", None))
            oled.show_boot(title="WASAKA", subtitle="AUTONOMOUS")
            oled_worker = OledWorker(oled).start()
        except Exception as e:
            print(f"[WARN] OLED gagal aktif ({e}).")

    # 5. Inisialisasi Kamera
    cap, cam_idx = open_camera(width=640, height=480)
    if cap is None:
        print("[FATAL] Kamera tidak ditemukan!")
        robot.shutdown()
        return

    camera = CameraStream(cap, threaded=USE_THREADED_CAMERA).start()

    # Tunggu Enter & Kalibrasi IMU
    wait_for_start(imu)
    print("[INFO] Memulai siklus otonom...")

    # Variabel Loop
    cur_vx, cur_vy, cur_wz = 0.0, 0.0, 0.0
    heading_target = imu.yaw() if (imu.ok and imu.yaw() is not None) else 0.0
    i_err_yaw = 0.0
    prev_time = time.time()
    fps = 0.0
    is_standby = False
    edge_detect_on = True

    try:
        while True:
            ret, frame, _ = camera.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue

            now = time.time()
            dt = max(1e-4, now - prev_time)
            prev_time = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

            # --- A. Pemrosesan Citra ---
            resized, enhanced, edges = preprocess_frame(frame, FRAME_WIDTH, FRAME_HEIGHT)
            roi_edges, roi_top = get_roi(edges, ROI_TOP_RATIO)
            rw = roi_edges.shape[1]
            tw = rw // 3

            dL = calculate_density(roi_edges[:, :tw]) if edge_detect_on else 0.0
            dC = calculate_density(roi_edges[:, tw:2 * tw]) if edge_detect_on else 0.0
            dR = calculate_density(roi_edges[:, 2 * tw:]) if edge_detect_on else 0.0

            # Adaptive Threshold
            obst_th, ign_th = adapt.update_and_get(dL, dC, dR) if adapt else (OBSTACLE_THRESHOLD, IGNORE_THRESHOLD)

            # Optical Flow
            roi_gray, _ = get_roi(enhanced, ROI_TOP_RATIO)
            flow_mags = flow_est.compute(roi_gray) if flow_est else None

            # Ground Segmentation
            nonfloor_mask = None
            ground_fracs = (None, None, None)
            wall_front = False
            if ground_seg:
                nonfloor_mask, ground_fracs, wall_front = ground_seg.process(resized, roi_top)

            # FreeSpace Profile
            free_boundary = None
            sL, sC, sR = dL, dC, dR
            if freespace:
                roi_mask = roi_edges if nonfloor_mask is None else cv2.bitwise_or(roi_edges, nonfloor_mask[roi_top:, :])
                (fL, fC, fR), free_boundary = freespace.compute(roi_mask)
                sL, sC, sR = (1.0 - fL) * 100.0, (1.0 - fC) * 100.0, (1.0 - fR) * 100.0

            # Fusi Blok
            is_moving = cur_vx < -0.05
            blk_L = fuse_blocked(dL, flow_mags[0] if flow_mags else None, ground_fracs[0], obst_th, ign_th, is_moving)
            blk_C = fuse_blocked(dC, flow_mags[1] if flow_mags else None, ground_fracs[1], obst_th, ign_th, is_moving)
            blk_R = fuse_blocked(dR, flow_mags[2] if flow_mags else None, ground_fracs[2], obst_th, ign_th, is_moving)

            # Evaluasi Deteksi YOLO (Jika aktif)
            yolo_obs_loc = "Aman"
            if detector:
                dets, yolo_obs_loc = detector.detect(frame)
                # Jika objek menyentuh bagian bawah frame -> rintangan dekat
                for d in dets:
                    if d["danger"] and d["touches_bottom"]:
                        blk_C = True

            # Keputusan Navigasi
            decision, dec_col = decide_navigation(dL, dC, dR, blk_L, blk_C, blk_R, ign_th, sL, sC, sR)

            # --- B. FSM & Kontrol Gerak ---
            imu.poll()
            tilt_now = imu.is_tilted()

            if fsm:
                cmd = fsm.update(decision, blk_L, blk_C, blk_R, dL, dR, tilt_now, wall_front)
                target_vx = cmd["vx"]
                target_vy = cmd["vy"]
                yaw_cmd = cmd["yaw_cmd"]
                hold_heading = cmd["hold_heading"]
                state = fsm.state
            else:
                target_vx = -SPEED_FORWARD if decision == "FORWARD" else 0.0
                target_vy = -SPEED_STRAFE if decision == "LEFT" else (SPEED_STRAFE if decision == "RIGHT" else 0.0)
                yaw_cmd = 0.0
                hold_heading = True
                state = "CRUISE"

            # Heading-Lock PID
            heading_err = 0.0
            if hold_heading and imu.ok and imu.yaw() is not None:
                heading_err = wrap_pi(heading_target - imu.yaw())
                i_err_yaw = max(-I_MAX, min(I_MAX, i_err_yaw + heading_err * dt))
                target_wz = max(-WZ_MAX, min(WZ_MAX, KP_YAW * heading_err + KI_YAW * i_err_yaw))
            else:
                target_wz = yaw_cmd
                if imu.ok and imu.yaw() is not None:
                    heading_target = imu.yaw()
                    i_err_yaw = 0.0

            # Haluskan kecepatan
            cur_vx = lerp(cur_vx, target_vx, SMOOTHING)
            cur_vy = lerp(cur_vy, target_vy, SMOOTHING)
            cur_wz = lerp(cur_wz, target_wz, SMOOTHING)

            if is_standby:
                cur_vx, cur_vy, cur_wz = 0.0, 0.0, 0.0

            # --- C. Eksekusi Gerak Servo + PID Leveling ---
            roll_deg, pitch_deg = imu.attitude()
            if not is_standby:
                leveler.step(robot, now, cur_vx, cur_vy, cur_wz, roll_deg, pitch_deg)
            else:
                leveler.hold_level(robot, roll_deg, pitch_deg)

            # --- D. Logging & Dashboard ---
            logger.log(state, decision, dL, dC, dR, flow_mags[1] if flow_mags else None,
                       cur_vx, cur_vy, cur_wz, heading_err, roll_deg, pitch_deg, fps)

            dash = build_dashboard(
                1080, 540, resized, edges, flow_mags, state, decision,
                cur_vx, cur_vy, cur_wz, fps, dL, dC, dR, blk_L, blk_C, blk_R,
                imu.ok, heading_err, (roll_deg, pitch_deg), imu.calibration(),
                nonfloor_mask, free_boundary
            )

            cv2.imshow("Wasaka Hexapod - Autonomous Dashboard", dash)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c') and ground_seg:
                ground_seg.calibrate(resized)
                print("[INFO] Ground color rekalibrasi selesai.")
            elif key == ord('p'):
                leveler.set_enabled(not leveler.enabled)
                print(f"[PID-LEVEL] {'ON' if leveler.enabled else 'OFF'}")
            elif key == ord('e'):
                edge_detect_on = not edge_detect_on
                print(f"[EDGE] {'ON' if edge_detect_on else 'OFF'}")
            elif key == ord('s'):
                is_standby = not is_standby
                print(f"[STANDBY] {'ON' if is_standby else 'OFF'}")

    finally:
        print("[INFO] Menghentikan subsistem...")
        camera.stop()
        robot.shutdown()
        logger.close()
        if oled_worker:
            oled_worker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Selesai.")


if __name__ == "__main__":
    run_autonomous()
