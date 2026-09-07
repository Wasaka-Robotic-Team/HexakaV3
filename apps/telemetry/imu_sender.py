"""
WASAKA HEXAPOD - IMU TELEMETRY SENDER (ROBOT SIDE)
===================================================
Mengirim paket data orientasi BNO055 (roll, pitch, yaw, quaternion, kalibrasi)
via UDP secara real-time (~50 Hz) ke laptop monitor / pameran.

Berjalan di thread daemon terpisah (non-blocking) agar tidak mengganggu
timing servo atau loop kontrol robot.

AUTHOR: Wasaka Robotic Team
"""

import json
import math
import socket
import threading
import time

DEFAULT_TARGET_IP   = "127.0.0.1"   # Ganti dengan IP laptop di jaringan WiFi/LAN
DEFAULT_TARGET_PORT = 5005          # Port penerima UDP di bridge.py
DEFAULT_HZ          = 50


class ImuTelemetrySender:
    """
    Thread pengirim telemetri UDP untuk Digital Twin.
    """

    def __init__(self,
                 imu_sensor,
                 target_ip=DEFAULT_TARGET_IP,
                 target_port=DEFAULT_TARGET_PORT,
                 hz=DEFAULT_HZ,
                 enabled=True):
        self.imu = imu_sensor
        self.ip = target_ip
        self.port = target_port
        self.interval = 1.0 / max(1, hz)
        self.enabled = enabled
        self._sock = None
        self._thread = None
        self._running = False
        self._seq = 0
        self._t0 = time.time()

    def start(self):
        if not self.enabled:
            return self
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
            print(f"[TWIN-UDP] Mengirim telemetri ke {self.ip}:{self.port} (~{int(1.0/self.interval)} Hz)")
        except Exception as e:
            print(f"[WARN] Inisialisasi UDP Telemetry gagal ({e}).")
            self._running = False
        return self

    def _loop(self):
        while self._running:
            t_start = time.time()
            self._send_one()
            elapsed = time.time() - t_start
            rem = self.interval - elapsed
            if rem > 0:
                time.sleep(rem)

    def _send_one(self):
        if not self.imu:
            return

        roll, pitch = self.imu.attitude()
        yaw_rad = self.imu.yaw()
        yaw_deg = math.degrees(yaw_rad) if yaw_rad is not None else 0.0
        quat = getattr(self.imu, "quaternion", lambda: (1.0, 0.0, 0.0, 0.0))()
        calib = self.imu.calibration()

        payload = {
            "seq": self._seq,
            "t_ms": int((time.time() - self._t0) * 1000),
            "roll": round(roll, 2),
            "pitch": round(pitch, 2),
            "yaw_deg": round(yaw_deg, 2),
            "quat": [round(x, 4) for x in quat],
            "calib": {
                "sys": calib[0],
                "gyro": calib[1],
                "accel": calib[2],
                "mag": calib[3]
            },
            "ok": self.imu.ok
        }

        try:
            data = json.dumps(payload).encode("utf-8")
            self._sock.sendto(data, (self.ip, self.port))
            self._seq += 1
        except Exception:
            pass

    def stop(self):
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
