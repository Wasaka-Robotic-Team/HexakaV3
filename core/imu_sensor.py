"""
WASAKA HEXAPOD - IMU SENSOR MODULE (BNO055)
============================================
Driver sensor orientasi 9-DOF BNO055 via I2C.

Menyediakan:
- Pembacaan Euler angle: yaw (heading), roll, pitch
- Pembacaan Quaternion untuk visualisasi 3D (Digital Twin)
- Monitoring status kalibrasi (system, gyroscope, accelerometer, magnetometer)
- Deteksi kemiringan bahaya (tilt detection safety)
- Pembagian I2C bus bersama OLED SSD1306 (mencegah konflik dual-bus di Raspberry Pi)
- Fallback simulasi aman saat dijalankan di PC/Laptop tanpa hardware.

AUTHOR: Wasaka Robotic Team
"""

import math
import sys
import time

IMU_YAW_SIGN   = -1.0    # -1.0 bila BNO055 terbalik (Z ke bawah)
TILT_LIMIT_DEG = 35.0    # batas sudut roll/pitch aman sebelum safety stop


def wrap_pi(angle):
    """Bungkus sudut ke interval [-pi, pi] (mencari selisih heading terpendek)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class HeadingSensor:
    """
    Pembungkus IMU BNO055 (NDOF mode).

    Membuat bus I2C tunggal yang dapat dibagikan dengan OLED (OledDisplay)
    untuk menghindari crash 'Device or resource busy' pada I2C0/I2C1.
    """

    def __init__(self, sign=IMU_YAW_SIGN, enabled=True, tilt_limit_deg=TILT_LIMIT_DEG):
        self.ok = False
        self.sign = sign
        self.tilt_limit = tilt_limit_deg
        self.sensor = None
        self.i2c = None
        self._yaw = None
        self._roll = 0.0
        self._pitch = 0.0
        self._quat = (1.0, 0.0, 0.0, 0.0)

        if not enabled:
            print("[IMU] Dinonaktifkan (enabled=False) -> mode open-loop.")
            return

        try:
            import board
            import busio
            import adafruit_bno055

            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
            self.ok = True
            print("[IMU] BNO055 terdeteksi (mode NDOF / AHRS absolut).")
        except Exception as e:
            print(f"[IMU] BNO055 tidak tersedia ({e}) -> mode simulasi / open-loop.")

    def poll(self):
        """Baca orientasi sekali; perbarui cache yaw/roll/pitch/quaternion."""
        if not self.ok or self.sensor is None:
            return

        try:
            euler = self.sensor.euler  # (heading, roll, pitch) dalam derajat
            if euler and euler[0] is not None:
                self._yaw = self.sign * math.radians(euler[0])
                if euler[1] is not None:
                    self._roll = float(euler[1])
                if euler[2] is not None:
                    self._pitch = float(euler[2])
            else:
                self._yaw = None

            q = self.sensor.quaternion
            if q and all(x is not None for x in q):
                self._quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        except Exception:
            self._yaw = None

    def yaw(self):
        """Mengembalikan heading yaw saat ini (radian), atau None jika gagal."""
        return self._yaw

    def attitude(self):
        """Mengembalikan sudut bodi saat ini: (roll_deg, pitch_deg)."""
        return self._roll, self._pitch

    def quaternion(self):
        """Mengembalikan orientasi quaternion: (w, x, y, z)."""
        return self._quat

    def is_tilted(self):
        """True bila kemiringan melewati batas aman (robot terbalik/terguling)."""
        if not self.ok:
            return False
        return abs(self._roll) > self.tilt_limit or abs(self._pitch) > self.tilt_limit

    def calibration(self):
        """Status kalibrasi 4 sensor: (sys, gyro, accel, mag), skala 0..3 (3 = full)."""
        if not self.ok or self.sensor is None:
            return (0, 0, 0, 0)
        try:
            return self.sensor.calibration_status
        except Exception:
            return (0, 0, 0, 0)


def wait_for_start(imu):
    """
    Tampilkan status kalibrasi live & tunggu pengguna menekan tombol ENTER.
    """
    print("\n=== KALIBRASI & KESIAPAN IMU ===")
    if imu.ok:
        print("  - Gerakkan robot pola ANGKA-8 untuk mengalibrasi MAG (target mag>=2).")
        print("  - Letakkan robot pada lantai datar untuk mengalibrasi ACCEL/SYS.")
        print("  - Posisikan robot menghadap arah target.")
    else:
        print("  - IMU tidak aktif, robot beroperasi tanpa feedback orientasi.")
    print("  >> Tekan ENTER untuk MULAI <<\n")

    if not sys.stdin.isatty():
        print("[INFO] Stdin non-interaktif -> langsung memulai.")
        return

    try:
        import select
        while True:
            imu.poll()
            s, g, a, m = imu.calibration()
            status = (f"  CALIB sys={s} gyro={g} accel={a} mag={m}"
                      if imu.ok else "  (IMU offline)")
            print(f"{status}   [ENTER=mulai]      ", end="\r", flush=True)
            r, _, _ = select.select([sys.stdin], [], [], 0.2)
            if r:
                sys.stdin.readline()
                break
    except Exception:
        # Fallback sederhana untuk Windows console
        input()
    print()
