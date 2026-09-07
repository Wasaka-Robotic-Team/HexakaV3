"""
WASAKA HEXAPOD - GAMEPAD TELEOPERATION APPLICATION
===================================================
Kontrol remote manual Hexapod Wasaka menggunakan joystick/gamepad Xbox (Pygame):
- Gerak manual 3-DOF: Maju/Mundur, Strafe Kiri/Kanan, Putar/Rotasi.
- Pengaturan parameter dinamis: Ketinggian bodi (Z), tinggi angkatan kaki, kecepatan langkah.
- Animasi ekspresif: Sapaan salam (Greeting/Bungkuk), Napas (Breathing), Dadah (Wave).
- Toggle Mode Otonom Person Follow via tombol BACK.

Dependensi:
    pip install pygame dynamixel-sdk opencv-python onnxruntime

AUTHOR: Wasaka Robotic Team
"""

import math
import os
import platform
import random
import sys
import threading
import time
from pathlib import Path

# Setup path ke root proyek
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Pygame Guard
try:
    import pygame
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False

from core.locomotion import Hexapod, calculate_ik, LEGS
from apps.person_follow.controller import FollowController
from apps.person_follow.animations import AnimationController

try:
    from vision.camera import open_camera
    from vision.person_detector import PersonDetector
    from vision.tracker import TargetTracker
    PERSON_FOLLOW_AVAILABLE = True
except Exception:
    PERSON_FOLLOW_AVAILABLE = False

import cv2

# Mapping Tombol Xbox
BUTTON_MAP = {
    'A': 0, 'B': 1, 'X': 3, 'Y': 4,
    'LB': 6, 'RB': 7, 'Back': 10, 'Start': 11,
    'L3': 8, 'R3': 9, 'Guide': 12
}

# Batas parameter gait
Z_GROUND_MIN,    Z_GROUND_MAX    = 3.0,  9.0
LIFT_HEIGHT_MIN, LIFT_HEIGHT_MAX = 0.5,  5.0
SPEED_MIN,       SPEED_MAX       = 0.01, 0.20
DPAD_TILT_STEP                   = 0.5
DPAD_TILT_MAX                    = 20.0


class GaitParams:
    """Parameter gerak yang dibagikan antar-thread."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.vx = 0.0
        self.vy = 0.0
        self.v_rot = 0.0
        self.z_ground = 5.0
        self.lift_height = 2.5
        self.stride_length = 6.0
        self.speed = 0.06
        self.delay = 0.01
        self.body_pitch = 0.0
        self.body_roll = 0.0
        self.x_rest = 7.0

        self.standby = False
        self.running = True
        self.play_greeting = False
        self.idle_anim = False
        self.person_follow = False

    def any_motion(self):
        return abs(self.vx) > 0.01 or abs(self.vy) > 0.01 or abs(self.v_rot) > 0.01


def robot_loop(params: GaitParams, robot: Hexapod, anim: AnimationController):
    """Thread pengendali servo hexapod berdasarkan state GaitParams."""
    prev_t = time.time()
    phase = 0.0

    while params.running:
        now = time.time()
        dt = max(1e-4, now - prev_t)
        prev_t = now

        # Jika mode person-follow aktif, kontrol servo diambil alih thread person-follow
        if params.person_follow:
            time.sleep(0.01)
            continue

        if params.standby:
            robot.stand_by(x=params.x_rest, y=0.0, z=params.z_ground)
            time.sleep(0.05)
            continue

        # Animasi Greeting
        if params.play_greeting:
            anim.start_greeting()
            params.play_greeting = False

        if anim.is_greeting():
            anim.play_greeting(dt)
            time.sleep(params.delay)
            continue

        # Animasi Idle
        if params.idle_anim and not params.any_motion():
            anim.play_idle(dt)
            time.sleep(params.delay)
            continue

        # Gerak Manual Gamepad
        phase += dt
        vx = max(-1.0, min(1.0, params.vx))
        vy = max(-1.0, min(1.0, params.vy))
        wz = max(-1.0, min(1.0, params.v_rot))

        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(wz) < 0.01:
            anim.play_breathing(dt, amplitude=0.3)
        else:
            # Di locomotion Wasaka, vx negatif = maju
            robot.step(phase, -vx, vy, wz)

        time.sleep(params.delay)


def person_follow_loop(params: GaitParams, robot: Hexapod, anim: AnimationController):
    """Thread penanganan mode otonom person following saat tombol Back ditekan."""
    if not PERSON_FOLLOW_AVAILABLE:
        return

    cap = None
    detector = None
    tracker = None
    ctrl = None
    prev_t = time.time()
    prev_track_state = "SEARCH"
    phase = 0.0
    show_window = "DISPLAY" in os.environ or platform.system() == "Windows"

    while params.running:
        if not params.person_follow:
            if cap is not None and cap.isOpened():
                cap.release()
                cap = None
                if show_window:
                    cv2.destroyAllWindows()
                print("[INFO] Mode Person Follow dinonaktifkan.")
            time.sleep(0.1)
            continue

        if cap is None or not cap.isOpened():
            cap, _ = open_camera(width=640, height=480)
            if cap is None:
                print("[ERROR] Kamera tidak dapat dibuka.")
                params.person_follow = False
                continue

            ret, frame = cap.read()
            if not ret or frame is None:
                params.person_follow = False
                cap.release()
                cap = None
                continue

            h, w = frame.shape[:2]
            detector = PersonDetector(weights_path="models/yolov8n.onnx", conf_thres=0.4, img_size=320)
            tracker = TargetTracker(switch_area_ratio=1.3, max_center_jump=120, lost_grace_frames=8)
            ctrl = FollowController(
                frame_w=w, frame_h=h, use_width=True,
                w_target_px=int(w * 0.38), w_too_close_px=int(w * 0.52),
                w_deadband_px=int(w * 0.03), backward_speed=-0.25
            )
            prev_t = time.time()
            prev_track_state = "SEARCH"
            print("[INFO] Mode Person Follow AKTIF.")

        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        now = time.time()
        dt = max(1e-4, now - prev_t)
        prev_t = now

        dets = detector.detect(frame)
        target, track_state = tracker.update(dets)
        turn_rate, fwd_speed, ctrl_state = ctrl.compute(target, dt)

        if track_state == "TRACKING" and prev_track_state == "SEARCH":
            anim.start_greeting()
        prev_track_state = track_state

        if track_state == "SEARCH":
            anim.play_idle(dt)
        elif anim.is_greeting():
            anim.play_greeting(dt)
        else:
            phase += dt
            vx = max(-1.0, min(1.0, -fwd_speed))
            wz = max(-1.0, min(1.0, turn_rate))
            if abs(vx) < 0.01 and abs(wz) < 0.01:
                anim.play_breathing(dt)
            else:
                robot.step(phase, vx, 0.0, wz)

        if show_window:
            for det in dets:
                x1, y1, x2, y2 = map(int, det["bbox"])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if target is not None:
                x1, y1, x2, y2 = map(int, target["bbox"])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.putText(frame, f"State: {track_state} | {ctrl_state}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("Hexapod Person Follow (Gamepad Mode)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                params.person_follow = False


def apply_deadzone(val, dz=0.08):
    return 0.0 if abs(val) < dz else val


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def gamepad_loop(params: GaitParams):
    """Loop penanganan event joystick pygame."""
    if not HAS_PYGAME:
        print("[ERROR] Pygame belum terinstall. Jalankan: pip install pygame")
        params.running = False
        return

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[GAMEPAD] Tidak ada gamepad terdeteksi! Pastikan controller Xbox terhubung via USB/Bluetooth.")
        params.running = False
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[GAMEPAD] Terdeteksi: {joy.get_name()}")
    print_controls()

    btn_a_prev = False
    btn_x_prev = False
    btn_b_prev = False
    btn_back_prev = False

    while params.running:
        pygame.event.pump()

        # Jika mode person follow aktif, tombol Back mematikan mode
        if params.person_follow:
            if joy.get_button(BUTTON_MAP['Back']):
                if not btn_back_prev:
                    params.person_follow = False
                    print("[GAMEPAD] Mode Person Follow DIMATIKAN.")
                    btn_back_prev = True
            else:
                btn_back_prev = False

            if joy.get_button(BUTTON_MAP['Start']):
                params.running = False
                break
            time.sleep(0.02)
            continue

        n = joy.get_numaxes()
        lx = apply_deadzone(joy.get_axis(0)) if n > 0 else 0.0
        ly = apply_deadzone(joy.get_axis(1)) if n > 1 else 0.0
        params.vx = -ly
        params.vy = lx

        rx = apply_deadzone(joy.get_axis(2)) if n > 2 else 0.0
        params.v_rot = rx

        # Trigger RT / LT
        rt_raw = joy.get_axis(4) if n > 4 else -1.0
        lt_raw = joy.get_axis(5) if n > 5 else -1.0
        rt = (rt_raw + 1.0) / 2.0
        lt = (lt_raw + 1.0) / 2.0
        if rt > 0.05:
            params.lift_height = clamp(params.lift_height + rt * 0.03, LIFT_HEIGHT_MIN, LIFT_HEIGHT_MAX)
        if lt > 0.05:
            params.lift_height = clamp(params.lift_height - lt * 0.03, LIFT_HEIGHT_MIN, LIFT_HEIGHT_MAX)

        # Tombol
        btn_rb = joy.get_button(BUTTON_MAP['RB'])
        btn_lb = joy.get_button(BUTTON_MAP['LB'])
        btn_y = joy.get_button(BUTTON_MAP['Y'])
        btn_b = joy.get_button(BUTTON_MAP['B'])
        btn_a = joy.get_button(BUTTON_MAP['A'])
        btn_x = joy.get_button(BUTTON_MAP['X'])
        btn_back = joy.get_button(BUTTON_MAP['Back'])
        btn_start = joy.get_button(BUTTON_MAP['Start'])

        if btn_rb:
            params.speed = clamp(params.speed + 0.001, SPEED_MIN, SPEED_MAX)
        if btn_lb:
            params.speed = clamp(params.speed - 0.001, SPEED_MIN, SPEED_MAX)

        if btn_y:
            params.reset()
            print("[GAMEPAD] Parameter direset ke default.")

        if btn_b and not btn_b_prev:
            params.standby = not params.standby
            print(f"[GAMEPAD] Standby: {'ON' if params.standby else 'OFF'}")
        btn_b_prev = btn_b

        if btn_a and not btn_a_prev:
            params.play_greeting = True
            params.idle_anim = False
            print("[GAMEPAD] Animasi Greeting dijalankan.")
        btn_a_prev = btn_a

        if btn_x and not btn_x_prev:
            params.idle_anim = not params.idle_anim
            print(f"[GAMEPAD] Animasi Idle: {'ON' if params.idle_anim else 'OFF'}")
        btn_x_prev = btn_x

        if btn_back and not btn_back_prev:
            if PERSON_FOLLOW_AVAILABLE:
                params.person_follow = not params.person_follow
                print(f"[GAMEPAD] Mode Person Follow: {'ON' if params.person_follow else 'OFF'}")
            else:
                print("[GAMEPAD] Modul Person Follow tidak tersedia.")
        btn_back_prev = btn_back

        if btn_start:
            params.running = False
            break

        time.sleep(0.01)

    pygame.quit()


def print_controls():
    print("""
-------------------------------------------------------------
KONTROL GAMEPAD XBOX:
  Left Stick       : Maju/Mundur (Y) & Strafe (X)
  Right Stick X    : Rotasi Badan (Putar di tempat)
  RT / LT          : Naikkan / Turunkan Tinggi Langkah Kaki
  RB / LB          : Percepat / Perlambat Kecepatan Langkah
  Tombol A         : Animasi Sapaan / Membungkuk (Greeting)
  Tombol X         : Toggle Animasi Idle (Breathing / Waving)
  Tombol B         : Toggle Standby Pose
  Tombol Y         : Reset Parameter ke Default
  Tombol BACK      : Toggle Mode Person Follow Otomatis
  Tombol START     : Keluar Program
-------------------------------------------------------------
""")


def run_teleop():
    print("=" * 60)
    print("  WASAKA HEXAPOD - GAMEPAD TELEOPERATION")
    print("=" * 60)

    params = GaitParams()
    robot = Hexapod()
    try:
        robot.connect()
        robot.enable_torque()
        robot.stand_by()
    except Exception as e:
        print(f"[WARN] Inisialisasi servo gagal ({e}) -> mode simulasi.")

    anim = AnimationController(robot)

    # Jalankan thread eksekusi robot
    t_robot = threading.Thread(target=robot_loop, args=(params, robot, anim), daemon=True)
    t_robot.start()

    # Jalankan thread person follow
    t_follow = threading.Thread(target=person_follow_loop, args=(params, robot, anim), daemon=True)
    t_follow.start()

    # Jalankan loop gamepad di thread utama
    try:
        gamepad_loop(params)
    finally:
        params.running = False
        print("[INFO] Menghentikan teleoperasi...")
        robot.stand_by()
        robot.shutdown()
        print("[INFO] Selesai.")


if __name__ == "__main__":
    run_teleop()
