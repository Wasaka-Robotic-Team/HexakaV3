"""
WASAKA HEXAPOD - TEST BODY LEVELING PID (GUI TUNING)
=====================================================
Harness GUI Tkinter untuk menguji & men-tuning kontroler PID Body Leveling.

Fitur:
- Slider real-time untuk Kp, Ki, Kd, D-Filter Alpha, Deadband, dan Max dz.
- Tombol Tare untuk menetapkan orientasi datar fisik saat ini.
- Toggle Live Mode PI vs PID (mematikan komponen D untuk melihat bedanya).
- Mode Statis (robot berdiri diam sambil menjaga bodi datar saat diangkat/dimiringkan).
- Mode Berjalan (gait aktif sambil leveling meredam kontur tanah tidak rata).
- Tampilan live P, I, D terms dan kompensasi dz untuk setiap kaki.

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

from core.locomotion import Hexapod, DEVICENAME, BAUDRATE, MOUNT_ANGLES
from core.imu_sensor import HeadingSensor
from core.body_leveler import BodyLeveler


class TestLevelPIDGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Wasaka Hexapod - Tuning Body Leveling PID")
        self.root.geometry("880x700")

        self.robot = None
        self.imu = HeadingSensor(enabled=True)
        self.leveler = BodyLeveler(enabled=False)

        self.is_walking = False
        self.is_leveling = False
        self.d_enabled = True
        self.phase = 0.0
        self.last_t = time.time()

        # Variable slider gain
        self.var_kp = tk.DoubleVar(value=1.0)
        self.var_ki = tk.DoubleVar(value=0.8)
        self.var_kd = tk.DoubleVar(value=0.4)
        self.var_d_filter = tk.DoubleVar(value=0.25)
        self.var_deadband = tk.DoubleVar(value=0.5)
        self.var_max_dz = tk.DoubleVar(value=2.5)

        self._build_ui()
        self._loop()

    def _build_ui(self):
        # 1. Baris Tombol Kontrol Atas
        frame_ctrl = ttk.LabelFrame(self.root, text="Kontrol Status", padding=8)
        frame_ctrl.pack(fill="x", padx=10, pady=5)

        ttk.Button(frame_ctrl, text="Connect Servo", command=self._connect).pack(side="left", padx=4)
        ttk.Button(frame_ctrl, text="Stand By", command=self._standby).pack(side="left", padx=4)
        ttk.Button(frame_ctrl, text="[TARE] Nolkan Fisik", command=self._tare).pack(side="left", padx=4)

        self.btn_level = ttk.Button(frame_ctrl, text="LEVELING: OFF", command=self._toggle_leveling)
        self.btn_level.pack(side="left", padx=8)

        self.btn_d = ttk.Button(frame_ctrl, text="[D] Term: ON", command=self._toggle_d)
        self.btn_d.pack(side="left", padx=4)

        self.btn_walk = ttk.Button(frame_ctrl, text="Mulai Jalan", command=self._toggle_walk)
        self.btn_walk.pack(side="left", padx=8)

        # 2. Slider Parameter PID
        frame_sliders = ttk.LabelFrame(self.root, text="Tuning Parameter PID", padding=8)
        frame_sliders.pack(fill="x", padx=10, pady=5)

        self._add_slider(frame_sliders, "Kp (Proportional):", self.var_kp, 0.0, 3.0, self._update_gains)
        self._add_slider(frame_sliders, "Ki (Integral):", self.var_ki, 0.0, 2.0, self._update_gains)
        self._add_slider(frame_sliders, "Kd (Derivative):", self.var_kd, 0.0, 1.5, self._update_gains)
        self._add_slider(frame_sliders, "D-Filter Alpha:", self.var_d_filter, 0.05, 0.95, self._update_gains)
        self._add_slider(frame_sliders, "Deadband (°):", self.var_deadband, 0.0, 3.0, self._update_gains)
        self._add_slider(frame_sliders, "Max dz (cm):", self.var_max_dz, 0.5, 5.0, self._update_gains)

        # 3. Status Sinyal P, I, D & IMU
        frame_mon = ttk.LabelFrame(self.root, text="Monitoring Kontrol Loop Tertutup", padding=8)
        frame_mon.pack(fill="x", padx=10, pady=5)

        self.lbl_imu = ttk.Label(frame_mon, text="IMU: Roll=+0.0°  Pitch=+0.0°", font=("Consolas", 11, "bold"))
        self.lbl_imu.pack(anchor="w", pady=2)

        self.lbl_terms = ttk.Label(
            frame_mon,
            text="P_roll=0.0°  I_roll=0.0°  D_roll=0.0° | U_roll=0.0°\nP_pitch=0.0° I_pitch=0.0° D_pitch=0.0° | U_pitch=0.0°",
            font=("Consolas", 10)
        )
        self.lbl_terms.pack(anchor="w", pady=2)

        # 4. Offset dz Per Kaki
        frame_legs = ttk.LabelFrame(self.root, text="Offset Ketinggian Vertikal Per Kaki (dz)", padding=8)
        frame_legs.pack(fill="x", padx=10, pady=5)

        self.lbl_legs = {}
        row_f = ttk.Frame(frame_legs)
        row_f.pack(fill="x")
        for i, leg_name in enumerate(MOUNT_ANGLES.keys()):
            lbl = ttk.Label(row_f, text=f"{leg_name[:8]}: +0.00 cm", font=("Consolas", 9), width=18)
            lbl.pack(side="left", padx=2)
            self.lbl_legs[leg_name] = lbl

        # 5. Log
        frame_log = ttk.LabelFrame(self.root, text="Log", padding=8)
        frame_log.pack(fill="both", expand=True, padx=10, pady=5)
        self.txt_log = tk.Text(frame_log, height=6, font=("Consolas", 9))
        self.txt_log.pack(fill="both", expand=True)

    def _add_slider(self, parent, text, var, from_, to_, callback):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=text, width=20).pack(side="left")
        scale = ttk.Scale(row, from_=from_, to=to_, variable=var, orient="horizontal", command=lambda v: callback())
        scale.pack(side="left", fill="x", expand=True, padx=5)
        lbl_v = ttk.Label(row, text=f"{var.get():.2f}", width=6)
        lbl_v.pack(side="left")
        var.trace_add("write", lambda *args: lbl_v.config(text=f"{var.get():.2f}"))

    def _log(self, msg):
        self.txt_log.insert("end", f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.txt_log.see("end")

    def _connect(self):
        try:
            self.robot = Hexapod()
            self.robot.connect()
            self.robot.enable_torque()
            self.robot.stand_by()
            self._log("Servo terhubung dan standby.")
        except Exception as e:
            self._log(f"Koneksi servo gagal: {e}")

    def _standby(self):
        if self.robot:
            self.robot.stand_by()
            self._log("Robot dalam pose Standby.")

    def _tare(self):
        r, p = self.imu.attitude()
        self.leveler.tare(r, p)
        self._log(f"Tare setpoint datar nol pada Roll={r:+.2f}°, Pitch={p:+.2f}°")

    def _toggle_leveling(self):
        self.is_leveling = not self.is_leveling
        self.leveler.set_enabled(self.is_leveling)
        self.btn_level.config(text="LEVELING: ON" if self.is_leveling else "LEVELING: OFF")
        self._log(f"Leveling closed-loop: {'ON' if self.is_leveling else 'OFF'}")

    def _toggle_d(self):
        self.d_enabled = not self.d_enabled
        kd = self.var_kd.get() if self.d_enabled else 0.0
        self.leveler.set_gains(kd=kd)
        self.btn_d.config(text="[D] Term: ON" if self.d_enabled else "[D] Term: OFF (PI)")
        self._log(f"Derivative D term: {'ON' if self.d_enabled else 'OFF (Mode PI)'}")

    def _toggle_walk(self):
        self.is_walking = not self.is_walking
        self.btn_walk.config(text="Stop Jalan" if self.is_walking else "Mulai Jalan")
        self._log(f"Gait gerak jalan: {'ON' if self.is_walking else 'STOP'}")

    def _update_gains(self):
        kd = self.var_kd.get() if self.d_enabled else 0.0
        self.leveler.set_gains(
            kp=self.var_kp.get(),
            ki=self.var_ki.get(),
            kd=kd
        )
        self.leveler.d_filter_alpha = self.var_d_filter.get()
        self.leveler.deadband_rad = math.radians(self.var_deadband.get())
        self.leveler.max_dz = self.var_max_dz.get()

    def _loop(self):
        now = time.time()
        dt = max(1e-4, now - self.last_t)
        self.last_t = now

        # Baca IMU
        self.imu.poll()
        r, p = self.imu.attitude()
        self.lbl_imu.config(text=f"IMU: Roll={r:+5.2f}°  Pitch={p:+5.2f}°")

        # Eksekusi Stabilitas & Gerak
        if self.is_walking and self.robot:
            self.phase += dt
            self.leveler.step(self.robot, self.phase, -0.4, 0.0, 0.0, r, p)
        elif self.is_leveling and self.robot:
            self.leveler.hold_level(self.robot, r, p)
        else:
            # Tetap hitung untuk monitoring GUI
            self.leveler.compute_offsets(r, p, is_walking=False)

        # Update Tampilan Monitoring
        st = self.leveler.status()
        t_str = (
            f"P_roll={st['p_roll']:+5.2f}°  I_roll={st['i_roll']:+5.2f}°  D_roll={st['d_roll']:+5.2f}° | U_roll={st['u_roll']:+5.2f}°\n"
            f"P_pitch={st['p_pitch']:+5.2f}° I_pitch={st['i_pitch']:+5.2f}° D_pitch={st['d_pitch']:+5.2f}° | U_pitch={st['u_pitch']:+5.2f}°"
        )
        self.lbl_terms.config(text=t_str)

        offsets = st["offsets"]
        for leg_name, val in offsets.items():
            if leg_name in self.lbl_legs:
                self.lbl_legs[leg_name].config(text=f"{leg_name[:8]}: {val:+5.2f} cm")

        self.root.after(20, self._loop)

    def on_close(self):
        self.is_walking = False
        self.is_leveling = False
        if self.robot:
            self.robot.stand_by()
            self.robot.shutdown()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = TestLevelPIDGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
