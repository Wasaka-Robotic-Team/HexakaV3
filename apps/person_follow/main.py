"""
WASAKA HEXAPOD - PERSON FOLLOWING APPLICATION
=============================================
Aplikasi interaktif mengikuti orang untuk pameran / bazar PKKMB:
1. Capture frame kamera (V4L2 / DSHOW).
2. Deteksi manusia via PersonDetector (YOLOv8 ONNX).
3. Pelacakan kontinu via TargetTracker dengan histeresis anti-flicker.
4. Regulasi kecepatan & arah belok via FollowController.
5. Animasi interaktif: Sapaan membungkuk (Greeting), Napas (Breathing), Melambai (Wave).
6. Tampilan visual live & integrasi OLED.

AUTHOR: Wasaka Robotic Team
"""

import math
import os
import platform
import sys
import time
from pathlib import Path

import cv2

# Setup path ke root proyek
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from core.locomotion import Hexapod
from core.oled_display import OledDisplay, OledWorker
from vision.camera import open_camera
from vision.person_detector import PersonDetector
from vision.tracker import TargetTracker
from apps.person_follow.controller import FollowController
from apps.person_follow.animations import AnimationController

IS_WINDOWS = platform.system() == "Windows"


def draw_visuals(frame, detections, target, track_state, ctrl_state, fps):
    """Menggambar informasi deteksi dan tracking pada frame."""
    h, w = frame.shape[:2]

    # Gambar semua deteksi kandidat (kuning tipis)
    for det in detections:
        x1, y1, x2, y2 = map(int, det["bbox"])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 200), 1)

    # Gambar target aktif (hijau tebal)
    if target is not None:
        x1, y1, x2, y2 = map(int, target["bbox"])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cx, cy = int(target["cx"]), int(target["cy"])
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # Garis dari tengah frame ke target
        cv2.line(frame, (w // 2, h // 2), (cx, cy), (255, 255, 0), 1)
        cv2.putText(frame, f"TARGET {target['conf']:.2f}", (x1, max(15, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Garis tengah acuan
    cv2.line(frame, (w // 2, 0), (w // 2, h), (100, 100, 100), 1)

    # Status text
    status_col = (0, 255, 0) if track_state == "TRACKING" else ((0, 255, 255) if track_state == "LOST_GRACE" else (0, 0, 255))
    cv2.putText(frame, f"TRACK: {track_state}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_col, 2)
    cv2.putText(frame, f"ACTION: {ctrl_state}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


def run_person_follow():
    print("=" * 60)
    print("  WASAKA HEXAPOD - PERSON FOLLOWING APPLICATION")
    print("=" * 60)

    # 1. Inisialisasi Servo Hexapod
    robot = Hexapod()
    try:
        robot.connect()
        robot.enable_torque()
        robot.stand_by()
    except Exception as e:
        print(f"[WARN] Inisialisasi servo gagal ({e}) -> lanjut mode simulasi.")

    # 2. Inisialisasi Kamera
    cap, cam_idx = open_camera(width=640, height=480)
    if cap is None:
        print("[FATAL] Gagal membuka kamera.")
        robot.shutdown()
        return

    ret, frame = cap.read()
    if not ret or frame is None:
        print("[FATAL] Kamera tidak menghasilkan frame.")
        cap.release()
        robot.shutdown()
        return

    frame_h, frame_w = frame.shape[:2]

    # 3. Inisialisasi AI & Kontroler
    detector = PersonDetector(weights_path="models/yolov8n.onnx", conf_thres=0.4, img_size=320)
    tracker = TargetTracker(switch_area_ratio=1.3, max_center_jump=120, lost_grace_frames=8)
    controller = FollowController(
        frame_w=frame_w,
        frame_h=frame_h,
        use_width=True,
        w_target_px=int(frame_w * 0.38),
        w_too_close_px=int(frame_w * 0.52),
        w_deadband_px=int(frame_w * 0.03),
        backward_speed=-0.25,
    )
    anim = AnimationController(robot)

    # 4. Inisialisasi OLED (Jika di RPi)
    oled_worker = None
    if not IS_WINDOWS:
        try:
            oled = OledDisplay(enabled=True)
            oled.show_boot(title="WASAKA", subtitle="PERSON FOLLOW")
            oled_worker = OledWorker(oled).start()
        except Exception as e:
            print(f"[WARN] OLED tidak tersedia ({e}).")

    prev_t = time.time()
    fps = 0.0
    phase = 0.0
    prev_track_state = "SEARCH"
    ctrl_state = "IDLE"

    print("[INFO] Memulai loop person following. Tekan 'q' untuk berhenti.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            now = time.time()
            dt = max(1e-4, now - prev_t)
            prev_t = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

            # Deteksi & Pelacakan
            detections = detector.detect(frame)
            target, track_state = tracker.update(detections)

            # Transisi Target Baru (SEARCH -> TRACKING) -> Jalankan salam sapaan
            if track_state == "TRACKING" and prev_track_state == "SEARCH":
                anim.start_greeting()
            prev_track_state = track_state

            # Eksekusi Kontrol / Animasi
            if anim.is_greeting():
                ctrl_state = "GREETING"
                anim.play_greeting(dt)
            elif target is not None:
                turn_rate, forward_speed, ctrl_state = controller.compute(target, dt)
                if ctrl_state == "HOLD_DISTANCE":
                    anim.play_breathing(dt)
                else:
                    phase += dt
                    # Koordinat robot: vx negatif = maju
                    vx = max(-1.0, min(1.0, -forward_speed))
                    wz = max(-1.0, min(1.0, turn_rate))
                    robot.step(phase, vx, 0.0, wz)
            else:
                ctrl_state = "IDLE (SEARCH)"
                anim.play_idle(dt)

            # Tampilkan Visual
            draw_visuals(frame, detections, target, track_state, ctrl_state, fps)
            cv2.imshow("Wasaka Hexapod - Person Following", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        print("[INFO] Menghentikan subsistem person following...")
        cap.release()
        robot.stand_by()
        robot.shutdown()
        if oled_worker:
            oled_worker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Selesai.")


if __name__ == "__main__":
    run_person_follow()
