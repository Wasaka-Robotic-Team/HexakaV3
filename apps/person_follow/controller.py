"""
WASAKA HEXAPOD - FOLLOW CONTROLLER MODULE
==========================================
Konversi koordinat target deteksi manusia -> instruksi kemudi hexapod (turn_rate, forward_speed).

Logika Kontrol:
- Heading (Arah Belok): PID berbasis penyimpangan horizontal (offset x) pusat
  bounding box terhadap titik tengah frame kamera.
- Jarak (Maju/Mundur): Kontrol proporsional dengan deadband berbasis LEBAR (w)
  bounding box (lebih stabil terhadap crop parsial atas/bawah frame):
  1. TOO_CLOSE    : Target terlalu dekat (w >= w_too_close) -> robot mundur.
  2. HOLD_DISTANCE: Target di jarak ideal (dalam rentang deadband) -> robot berhenti.
  3. APPROACH     : Target terlalu jauh (w < w_target) -> robot maju mengejar target.

AUTHOR: Wasaka Robotic Team
"""

from core.pid_controller import PIDController


class FollowController:
    """
    Kontroler navigasi pengikut orang berbasis bounding box.
    """

    def __init__(self,
                 frame_w=640,
                 frame_h=480,
                 use_width=True,
                 w_target_px=240,
                 w_too_close_px=330,
                 w_deadband_px=20,
                 h_target_px=220,
                 h_too_close_px=280,
                 h_deadband_px=15,
                 heading_pid=(0.9, 0.0, 0.15),
                 forward_gain=0.01,
                 forward_max=1.0,
                 backward_speed=-0.25):
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.use_width = use_width

        self.w_target = w_target_px
        self.w_too_close = w_too_close_px
        self.w_deadband = w_deadband_px

        self.h_target = h_target_px
        self.h_too_close = h_too_close_px
        self.h_deadband = h_deadband_px

        kp, ki, kd = heading_pid
        self.heading_pid = PIDController(kp=kp, ki=ki, kd=kd, output_min=-1.0, output_max=1.0)
        self.forward_gain = forward_gain
        self.forward_max = forward_max
        self.backward_speed = backward_speed

    def reset(self):
        self.heading_pid.reset()

    def compute(self, target, dt):
        """
        Menghitung kecepatan putar dan maju dari target yang terdeteksi.

        Args:
            target: dict dari TargetTracker atau None
            dt: selang waktu per frame (detik)

        Returns:
            tuple: (turn_rate [-1..1], forward_speed [-1..1], state_str)
        """
        if target is None:
            self.heading_pid.reset()
            return 0.0, 0.0, "IDLE"

        cx = target.get("cx", self.frame_w / 2.0)

        # Pilih metrik ukuran: Lebar (w) lebih stabil dibanding Tinggi (h)
        if self.use_width:
            bbox = target.get("bbox", (0, 0, 0, 0))
            val = target.get("w", bbox[2] - bbox[0])
            target_val = self.w_target
            too_close_val = self.w_too_close
            deadband_val = self.w_deadband
        else:
            val = target.get("h", 0.0)
            target_val = self.h_target
            too_close_val = self.h_too_close
            deadband_val = self.h_deadband

        # 1. Kontrol Heading (Turn Rate)
        err_x = (cx - self.frame_w / 2.0) / (self.frame_w / 2.0)
        turn_rate = self.heading_pid.compute(measurement=-err_x, dt=dt, setpoint=0.0)

        # 2. Kontrol Jarak (Forward Speed)
        if val >= too_close_val:
            forward_speed = self.backward_speed
            state = "TOO_CLOSE"
        elif abs(val - target_val) <= deadband_val:
            forward_speed = 0.0
            state = "HOLD_DISTANCE"
        elif val < target_val:
            forward_speed = min(self.forward_max, self.forward_gain * (target_val - val))
            state = "APPROACH"
        else:
            forward_speed = 0.0
            state = "HOLD_DISTANCE"

        return turn_rate, forward_speed, state
