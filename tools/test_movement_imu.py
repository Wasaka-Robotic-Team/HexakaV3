"""
WASAKA HEXAPOD - TEST GERAKAN & IMU (GUI SLIDER)
=================================================
Harness GUI Tkinter untuk menguji servo Dynamixel dan sensor IMU BNO055 secara manual.
Berguna untuk kalibrasi arah gerak, pengujian torsi, dan respon IMU sebelum autonomous run.

ALUR PENGUJIAN:
  1. Connect Servo -> Enable Torque -> Stand By
  2. Mulai Jalan -> Geser slider vx/vy/wz atau gunakan keyboard:
     - Panah Atas / Bawah : Maju / Mundur
     - Panah Kiri / Kanan : Geser Kiri / Kanan
     - Tombol A / D       : Putar Kiri / Kanan
     - Spasi              : Berhenti (semua kecepatan 0)

AUTHOR: Wasaka Robotic Team
"""

import math
import sys
import time
from pathlib import Path

# Setup path ke root proyek
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError as e:
    raise SystemExit(f"[FATAL] Tkinter tidak tersedia ({e}). Install: sudo apt install python3-tk")

from core.locomotion import Hexapod, LEGS, ALL_IDS, DEVICENAME, BAUDRATE, HAS_DYNAMIXEL
from core.imu_sensor import HeadingSensor


class TestMovementIMUApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Wasaka Hexapod - Uji Gerak & IMU")
        self.root.geometry("820x620")

        self.robot = None
        self.imu = HeadingSensor(enabled=True)

        self.is_walking = False
        self.phase = 0.0
        self.last_t = time.time()

        self._build_ui()
        self._bind_keys()
        self._update_loop()

    def _build_ui(self):
        # 1. Panel Koneksi & Servo
        frame_top = ttk.LabelFrame(self.root, text="Kontrol Hardware & Servo", padding=10)
        frame_top.pack(fill="x", padx=10, pady=5)

        ttk.Button(frame_top, text="Connect Servo", command=self._connect_servo).pack(side="left", padx=5)
        ttk.Button(frame_top, text="Enable Torque", command=self._enable_torque).pack(side="left", padx=5)
        ttk.Button(frame_top, text="Stand By", command=self._standby).pack(side="left", padx=5)
        ttk.Button(frame_top, text="Disable Torque", command=self._disable_torque).pack(side="left", padx=5)

        # 2. Panel Slider Kecepatan
        frame_sliders = ttk.LabelFrame(self.root, text="Slider Kecepatan (vx, vy, wz)", padding=10)
        frame_sliders.pack(fill="x", padx=10, pady=5)

        self.val_vx = tk.DoubleVar(value=0.0)
        self.val_vy = tk.DoubleVar(value=0.0)
        self.val_wz = tk.DoubleVar(value=0.0)

        self._create_slider(frame_sliders, "Maju / Mundur (vx):", self.val_vx, -1.0, 1.0)
        self._create_slider(frame_sliders, "Geser Samping (vy):", self.val_vy, -1.0, 1.0)
        self._create_slider(frame_sliders, "Putar / Rotasi (wz):", self.val_wz, -1.0, 1.0)

        frame_walk = ttk.Frame(frame_sliders)
        frame_walk.pack(fill="x", pady=5)
        self.btn_walk = ttk.Button(frame_walk, text="Mulai Jalan", command=self._toggle_walk)
        self.btn_walk.pack(side="left", padx=5)
        ttk.Button(frame_walk, text="STOP (Reset 0)", command=self._stop_all).pack(side="left", padx=5)

        # 3. Panel IMU Live
        frame_imu = ttk.LabelFrame(self.root, text="Status IMU BNO055", padding=10)
        frame_imu.pack(fill="x", padx=10, pady=5)

        self.lbl_imu_euler = ttk.Label(frame_imu, text="Orientasi: Roll=0.0°  Pitch=0.0°  Yaw=0.0°", font=("Consolas", 11))
        self.lbl_imu_euler.pack(anchor="w", pady=2)

        self.lbl_imu_calib = ttk.Label(frame_imu, text="Kalibrasi: Sys=0 Gyro=0 Accel=0 Mag=0", font=("Consolas", 10))
        self.lbl_imu_calib.pack(anchor="w", pady=2)

        # 4. Log Box
        frame_log = ttk.LabelFrame(self.root, text="Log Aktivitas", padding=10)
        frame_log.pack(fill="both", expand=True, padx=10, pady=5)

        self.txt_log = tk.Text(frame_log, height=8, font=("Consolas", 9))
        self.txt_log.pack(fill="both", expand=True)

    def _create_slider(self, parent, label, var, from_, to_):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=3)
        ttk.Label(row, text=label, width=22).pack(side="left")
        scale = ttk.Scale(row, from_=from_, to=to_, variable=var, orient="horizontal")
        scale.pack(side="left", fill="x", expand=True, padx=5)
        lbl_val = ttk.Label(row, text="0.00", width=6)
        lbl_val.pack(side="left")
        var.trace_add("write", lambda *args: lbl_val.config(text=f"{var.get():+.2f}"))

    def _bind_keys(self):
        self.root.bind("<Up>", lambda e: self.val_vx.set(min(1.0, self.val_vx.get() + 0.2)))
        self.root.bind("<Down>", lambda e: self.val_vx.set(max(-1.0, self.val_vx.get() - 0.2)))
        self.root.bind("<Left>", lambda e: self.val_vy.set(max(-1.0, self.val_vy.get() - 0.2)))
        self.root.bind("<Right>", lambda e: self.val_vy.set(min(1.0, self.val_vy.get() + 0.2)))
        self.root.bind("a", lambda e: self.val_wz.set(max(-1.0, self.val_wz.get() - 0.2)))
        self.root.bind("d", lambda e: self.val_wz.set(min(1.0, self.val_wz.get() + 0.2)))
        self.root.bind("<space>", lambda e: self._stop_all())

    def _log(self, msg):
        self.txt_log.insert("end", f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.txt_log.see("end")

    def _connect_servo(self):
        try:
            self.robot = Hexapod()
            self.robot.connect()
            self._log(f"Koneksi servo berhasil di {DEVICENAME} @ {BAUDRATE} bps.")
        except Exception as e:
            self._log(f"Gagal koneksi servo: {e}")

    def _enable_torque(self):
        if self.robot:
            self.robot.enable_torque()
            self._log("Torque servo aktif.")

    def _disable_torque(self):
        if self.robot:
            self.robot.disable_torque()
            self._log("Torque servo dinonaktifkan.")

    def _standby(self):
        if self.robot:
            self.robot.stand_by()
            self._log("Robot dalam pose Standby.")

    def _toggle_walk(self):
        self.is_walking = not self.is_walking
        self.btn_walk.config(text="Stop Jalan" if self.is_walking else "Mulai Jalan")
        self._log(f"Status jalan: {'AKTIF' if self.is_walking else 'STOP'}")

    def _stop_all(self):
        self.val_vx.set(0.0)
        self.val_vy.set(0.0)
        self.val_wz.set(0.0)
        self.is_walking = False
        self.btn_walk.config(text="Mulai Jalan")
        if self.robot:
            self.robot.stand_by()
        self._log("Semua kecepatan direset ke 0.")

    def _update_loop(self):
        now = time.time()
        dt = max(1e-4, now - self.last_t)
        self.last_t = now

        # Update IMU
        self.imu.poll()
        roll, pitch = self.imu.attitude()
        yaw_rad = self.imu.yaw()
        yaw_deg = math.degrees(yaw_rad) if yaw_rad is not None else 0.0
        s, g, a, m = self.imu.calibration()

        self.lbl_imu_euler.config(text=f"Orientasi: Roll={roll:+5.1f}°  Pitch={pitch:+5.1f}°  Yaw={yaw_deg:+5.1f}°")
        self.lbl_imu_calib.config(text=f"Kalibrasi: Sys={s} Gyro={g} Accel={a} Mag={m}")

        # Update Gerak Gait
        if self.is_walking and self.robot:
            self.phase += dt
            # Koordinat gait Wasaka: vx negatif = maju
            vx = -self.val_vx.get()
            vy = self.val_vy.get()
            wz = self.val_wz.get()
            self.robot.step(self.phase, vx, vy, wz)

        self.root.after(20, self._update_loop)

    def on_close(self):
        self.is_walking = False
        if self.robot:
            self.robot.stand_by()
            self.robot.shutdown()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = TestMovementIMUApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
