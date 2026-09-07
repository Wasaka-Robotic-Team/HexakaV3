"""
PID CONTROLLER - Stabilitas Robot Berjalan (Wasaka Robotic Team)
================================================================

Full PID (Proportional-Integral-Derivative) controller dengan fitur:
  • Anti-windup integral (clamping)
  • Low-pass filter pada derivative (menghilangkan noise IMU)
  • Output clamping (melindungi servo/motor dari perintah berlebihan)
  • Reset otomatis saat setpoint berubah drastis

Dirancang untuk mengontrol balance robot saat berjalan di medan tidak rata
menggunakan data orientasi dari BNO055 IMU (roll, pitch, yaw).

AUTHOR: Wasaka Robotic Team
"""

import time
import math


class PIDController:
    """
    PID Controller dengan anti-windup dan derivative filtering.

    Cocok untuk kontrol stabilitas robot berjalan di permukaan tidak rata.
    Menggunakan rumus:
        output = Kp*e + Ki*∫e·dt + Kd*(de/dt)

    Dimana derivative menggunakan low-pass filter untuk mengurangi noise:
        filtered_derivative = α * raw_derivative + (1-α) * prev_derivative
    """

    def __init__(self, kp, ki, kd,
                 setpoint=0.0,
                 output_min=-100.0,
                 output_max=100.0,
                 integral_limit=50.0,
                 derivative_filter_alpha=0.3,
                 deadband=0.0,
                 name="PID"):
        """
        Parameters
        ----------
        kp : float
            Gain Proporsional — koreksi langsung terhadap error.
            Semakin besar → respons semakin agresif, tapi bisa osilasi.

        ki : float
            Gain Integral — menghilangkan steady-state error.
            Semakin besar → koreksi drift, tapi bisa windup/overshoot.

        kd : float
            Gain Derivative — meredam osilasi dan memprediksi error.
            KRUSIAL untuk terrain tidak rata! Tanpa ini robot goyang/wobble.

        setpoint : float
            Nilai target (misal: 0° untuk robot tegak).

        output_min, output_max : float
            Batas output PID (lindungi servo dari perintah berlebihan).

        integral_limit : float
            Batas anti-windup untuk akumulasi integral.

        derivative_filter_alpha : float (0.0 - 1.0)
            Koefisien low-pass filter derivative.
            0.1 = sangat halus (lambat bereaksi)
            0.5 = medium
            1.0 = tanpa filter (noisy, tidak disarankan)

        deadband : float
            Error di bawah nilai ini diabaikan (mengurangi jitter servo).

        name : str
            Nama controller untuk logging (misal: "Roll", "Pitch").
        """
        # Gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Setpoint & limits
        self.setpoint = setpoint
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.deadband = deadband
        self.name = name

        # Derivative filter
        self.alpha = derivative_filter_alpha

        # Internal state
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._prev_time = None
        self._prev_output = 0.0

        # Diagnostik (bisa dibaca untuk tuning)
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_error = 0.0
        self.last_dt = 0.0

    def reset(self):
        """Reset state internal PID (panggil saat ganti mode/posisi)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._prev_time = None
        self._prev_output = 0.0

    def set_gains(self, kp=None, ki=None, kd=None):
        """Update gains secara live (berguna untuk auto-tuning)."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    def compute(self, measurement, dt=None, setpoint=None):
        """
        Hitung output PID berdasarkan pengukuran saat ini.

        Parameters
        ----------
        measurement : float
            Nilai sensor saat ini (atau setpoint jika dipanggil dengan 3 posisi).
        dt : float or None
            Delta waktu dalam detik.
        setpoint : float or None
            Nilai target opsional untuk meng-override self.setpoint.

        Returns
        -------
        float
            Output PID yang sudah di-clamp ke [output_min, output_max].
        """
        # Dukung pemanggilan fleksibel: compute(setpoint, measurement, dt)
        if setpoint is not None:
            self.setpoint = float(setpoint)
        elif dt is not None and isinstance(dt, (int, float)) and not isinstance(measurement, (int, float)):
            # Jika argumen tertukar
            pass

        now = time.time()

        # Hitung dt otomatis jika tidak diberikan
        if dt is None:
            if self._prev_time is None:
                self._prev_time = now
                self._prev_error = self.setpoint - measurement
                return 0.0
            dt = now - self._prev_time
        self._prev_time = now

        # Guard: dt terlalu kecil atau negatif
        if dt <= 1e-6:
            return self._prev_output

        # ---- ERROR ----
        error = self.setpoint - measurement

        # Deadband: abaikan error kecil (mengurangi jitter)
        if abs(error) < self.deadband:
            error = 0.0

        # ---- PROPORTIONAL ----
        p_term = self.kp * error

        # ---- INTEGRAL (dengan anti-windup) ----
        self._integral += error * dt

        # Anti-windup: clamp integral agar tidak menumpuk tanpa batas
        if self._integral > self.integral_limit:
            self._integral = self.integral_limit
        elif self._integral < -self.integral_limit:
            self._integral = -self.integral_limit

        i_term = self.ki * self._integral

        # ---- DERIVATIVE (dengan low-pass filter) ----
        raw_derivative = (error - self._prev_error) / dt

        # Low-pass filter: menghaluskan derivative agar tidak noisy
        # filtered = alpha * raw + (1 - alpha) * previous
        filtered_derivative = (self.alpha * raw_derivative +
                               (1.0 - self.alpha) * self._prev_derivative)

        d_term = self.kd * filtered_derivative

        # Simpan state untuk iterasi berikutnya
        self._prev_error = error
        self._prev_derivative = filtered_derivative

        # ---- TOTAL OUTPUT ----
        output = p_term + i_term + d_term

        # Clamp output ke batas aman
        output = max(self.output_min, min(self.output_max, output))

        # Simpan diagnostik
        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term
        self.last_error = error
        self.last_dt = dt
        self._prev_output = output

        return output

    def diagnostics(self):
        """Kembalikan dict diagnostik untuk debugging/tuning."""
        return {
            "name": self.name,
            "error": round(self.last_error, 3),
            "P": round(self.last_p, 3),
            "I": round(self.last_i, 3),
            "D": round(self.last_d, 3),
            "output": round(self._prev_output, 3),
            "integral_accum": round(self._integral, 3),
            "dt_ms": round(self.last_dt * 1000, 1),
        }

    def __repr__(self):
        return (f"PIDController(name={self.name!r}, "
                f"Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, "
                f"setpoint={self.setpoint})")


class DualAxisStabilizer:
    """
    Stabilizer dua sumbu (Roll + Pitch) untuk robot berjalan.

    Menggunakan dua PID controller terpisah untuk sumbu roll dan pitch,
    dengan input dari BNO055 IMU. Output bisa digunakan untuk menyesuaikan:
    - Tinggi kaki (compensate kemiringan medan)
    - Sudut hip/ankle servo
    - Offset gait trajectory

    Penggunaan:
        stabilizer = DualAxisStabilizer(imu_sensor)
        stabilizer.start()

        # Di dalam gait loop:
        roll_corr, pitch_corr = stabilizer.get_correction()
        # Terapkan koreksi ke servo...

        stabilizer.stop()
    """

    # ---- Default PID gains (SESUAIKAN dengan robot Anda!) ----
    # Nilai awal konservatif — tuning diperlukan untuk setiap robot.
    DEFAULT_ROLL_GAINS  = {"kp": 2.0,  "ki": 0.3,  "kd": 0.8}
    DEFAULT_PITCH_GAINS = {"kp": 2.5,  "ki": 0.4,  "kd": 1.0}

    def __init__(self, imu_sensor,
                 roll_gains=None,
                 pitch_gains=None,
                 target_roll=0.0,
                 target_pitch=0.0,
                 hz=50,
                 output_limit=30.0,
                 enabled=True):
        """
        Parameters
        ----------
        imu_sensor : HeadingSensor
            Objek IMU yang sudah ada di main.py (sama yang dipakai telemetry).

        roll_gains, pitch_gains : dict or None
            {"kp": float, "ki": float, "kd": float}
            Jika None, gunakan default gains.

        target_roll, target_pitch : float
            Setpoint target dalam derajat (0° = robot tegak).

        hz : int
            Frekuensi update loop PID.

        output_limit : float
            Batas maksimum koreksi (derajat atau unit servo).

        enabled : bool
            Jika False, stabilizer tidak aktif.
        """
        self.imu = imu_sensor
        self.enabled = enabled
        self.interval = 1.0 / max(1, hz)

        # PID gains
        rg = roll_gains or self.DEFAULT_ROLL_GAINS
        pg = pitch_gains or self.DEFAULT_PITCH_GAINS

        # Buat PID controller untuk setiap sumbu
        self.pid_roll = PIDController(
            kp=rg["kp"], ki=rg["ki"], kd=rg["kd"],
            setpoint=target_roll,
            output_min=-output_limit, output_max=output_limit,
            integral_limit=output_limit * 0.6,
            derivative_filter_alpha=0.3,
            deadband=0.5,       # <0.5° dianggap stabil
            name="Roll"
        )
        self.pid_pitch = PIDController(
            kp=pg["kp"], ki=pg["ki"], kd=pg["kd"],
            setpoint=target_pitch,
            output_min=-output_limit, output_max=output_limit,
            integral_limit=output_limit * 0.6,
            derivative_filter_alpha=0.3,
            deadband=0.5,
            name="Pitch"
        )

        # Thread state
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._roll_correction = 0.0
        self._pitch_correction = 0.0

        # Logging
        self._log_interval = 1.0  # log setiap 1 detik
        self._last_log_time = 0

    def start(self):
        """Mulai thread stabilizer."""
        if not self.enabled:
            print("[STAB] Stabilizer dinonaktifkan.")
            return self
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, name="stabilizer", daemon=True)
        self._thread.start()
        print(f"[STAB] DualAxisStabilizer aktif @ {1/self.interval:.0f} Hz")
        print(f"[STAB] Roll PID:  Kp={self.pid_roll.kp} "
              f"Ki={self.pid_roll.ki} Kd={self.pid_roll.kd}")
        print(f"[STAB] Pitch PID: Kp={self.pid_pitch.kp} "
              f"Ki={self.pid_pitch.ki} Kd={self.pid_pitch.kd}")
        return self

    def stop(self):
        """Hentikan thread stabilizer."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        print("[STAB] Stabilizer berhenti.")

    def get_correction(self):
        """
        Ambil koreksi stabilisasi terbaru (thread-safe).

        Returns
        -------
        (roll_correction, pitch_correction) : tuple of float
            Koreksi dalam derajat (atau unit yang sama dengan output_limit).
            Positif = koreksi ke satu arah, negatif = koreksi ke arah lawan.
        """
        with self._lock:
            return self._roll_correction, self._pitch_correction

    def get_diagnostics(self):
        """Ambil diagnostik kedua PID controller."""
        return {
            "roll": self.pid_roll.diagnostics(),
            "pitch": self.pid_pitch.diagnostics(),
        }

    def _loop(self):
        """Main loop stabilizer — berjalan di daemon thread."""
        import threading as _thr  # noqa: lazy import for safety

        while self._running:
            t0 = time.time()

            try:
                if not self.imu.ok:
                    # IMU tidak siap, skip update
                    time.sleep(self.interval)
                    continue

                # Baca orientasi dari IMU (cached, bukan transaksi I2C baru)
                roll, pitch = self.imu.attitude()   # derajat

                # Hitung koreksi PID
                roll_out = self.pid_roll.compute(roll)
                pitch_out = self.pid_pitch.compute(pitch)

                # Simpan koreksi (thread-safe)
                with self._lock:
                    self._roll_correction = roll_out
                    self._pitch_correction = pitch_out

                # Logging periodik
                now = time.time()
                if now - self._last_log_time >= self._log_interval:
                    self._last_log_time = now
                    rd = self.pid_roll.diagnostics()
                    pd = self.pid_pitch.diagnostics()
                    print(f"[STAB] Roll: err={rd['error']:+.1f}° "
                          f"P={rd['P']:+.2f} I={rd['I']:+.2f} "
                          f"D={rd['D']:+.2f} → out={rd['output']:+.2f} | "
                          f"Pitch: err={pd['error']:+.1f}° "
                          f"P={pd['P']:+.2f} I={pd['I']:+.2f} "
                          f"D={pd['D']:+.2f} → out={pd['output']:+.2f}")

            except Exception as e:
                print(f"[STAB] Error: {e}")

            elapsed = time.time() - t0
            wait = self.interval - elapsed
            if wait > 0:
                time.sleep(wait)
