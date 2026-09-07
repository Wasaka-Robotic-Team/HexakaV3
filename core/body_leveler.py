"""
WASAKA HEXAPOD - BODY LEVELER MODULE (CLOSED-LOOP STABILIZATION)
================================================================
Kontroler loop tertutup yang menjaga bodi robot tetap horizontal (datar
terhadap gravitasi) ketika berjalan di atas rintangan atau kontur tidak rata.

Membaca sudut roll & pitch dari IMU BNO055, menghitung koreksi PID, dan
menambahkan offset ketinggian vertikal (dz) pada masing-masing dari 6 kaki.

PRINSIP KOMPENSASI GEOMETRIS:
    Untuk setiap kaki pada posisi (px, py) terhadap titik pusat bodi:
        dz = px * u_pitch + py * u_roll
    di mana u_pitch dan u_roll adalah sinyal koreksi dari PID:
        u = Kp * error + Ki * integral(error) + Kd * derivative(error)

AUTHOR: Wasaka Robotic Team
"""

import math
import time

try:
    from .locomotion import (
        LEGS,
        GROUP_A_KEYS,
        MOUNT_ANGLES,
        X_REST,
        Z_GROUND,
        get_leg_target,
        calculate_ik,
        DXL_LOBYTE,
        DXL_HIBYTE,
        HAS_DYNAMIXEL,
    )
except ImportError:
    from core.locomotion import (
        LEGS,
        GROUP_A_KEYS,
        MOUNT_ANGLES,
        X_REST,
        Z_GROUND,
        get_leg_target,
        calculate_ik,
        DXL_LOBYTE,
        DXL_HIBYTE,
        HAS_DYNAMIXEL,
    )


def _clamp(val, low, high):
    return max(low, min(high, val))


class BodyLeveler:
    """
    Kontroler penyeimbang bodi hexapod berbasis PID.
    """

    def __init__(self,
                 kp=1.0,
                 ki=0.8,
                 kd=0.4,
                 deadband_deg=0.5,
                 max_dz_cm=2.5,
                 i_max=0.35,
                 d_filter_alpha=0.25,
                 sign_roll=1.0,
                 sign_pitch=-1.0,
                 use_input_filter=True,
                 input_filter_alpha=0.3,
                 walking_gain_scale=0.6,
                 enabled=True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadband_rad = math.radians(deadband_deg)
        self.max_dz = max_dz_cm
        self.i_max = i_max
        self.d_filter_alpha = d_filter_alpha
        self.sign_roll = sign_roll
        self.sign_pitch = sign_pitch

        self.use_input_filter = use_input_filter
        self.input_filter_alpha = input_filter_alpha
        self.walking_gain_scale = walking_gain_scale

        self.enabled = enabled

        # State integral & derivative
        self.i_roll = 0.0
        self.i_pitch = 0.0
        self._prev_e_roll = 0.0
        self._prev_e_pitch = 0.0
        self._filt_d_roll = 0.0
        self._filt_d_pitch = 0.0
        self._last_t = None

        # Filter input
        self._filt_roll_deg = 0.0
        self._filt_pitch_deg = 0.0
        self._input_filter_init = False

        # Tare / offset kalibrasi datar fisik
        self.tare_roll = 0.0
        self.tare_pitch = 0.0

        # Cache telemetri
        self.last_offsets = {k: 0.0 for k in MOUNT_ANGLES}
        self.last_roll_deg = 0.0
        self.last_pitch_deg = 0.0
        self.last_p_roll = 0.0
        self.last_p_pitch = 0.0
        self.last_d_roll = 0.0
        self.last_d_pitch = 0.0
        self.last_u_roll = 0.0
        self.last_u_pitch = 0.0

    def set_enabled(self, state: bool):
        self.enabled = bool(state)
        if not self.enabled:
            self.reset()

    def set_gains(self, kp=None, ki=None, kd=None):
        if kp is not None:
            self.kp = float(kp)
        if ki is not None:
            self.ki = float(ki)
        if kd is not None:
            self.kd = float(kd)

    def tare(self, current_roll, current_pitch):
        """Set orientasi fisik saat ini sebagai acuan 'datar nol'."""
        self.tare_roll = current_roll
        self.tare_pitch = current_pitch
        self.reset()

    def reset(self):
        """Reset akumulasi integral, memori derivatif, dan filter."""
        self.i_roll = 0.0
        self.i_pitch = 0.0
        self._prev_e_roll = 0.0
        self._prev_e_pitch = 0.0
        self._filt_d_roll = 0.0
        self._filt_d_pitch = 0.0
        self._last_t = None
        self._input_filter_init = False
        self.last_offsets = {k: 0.0 for k in MOUNT_ANGLES}

    def compute_offsets(self, roll_deg, pitch_deg, is_walking=False):
        """
        Hitung offset tinggi Z (dz) per kaki dari sudut roll & pitch.
        """
        if not self.enabled:
            self.last_offsets = {k: 0.0 for k in MOUNT_ANGLES}
            self.last_roll_deg = roll_deg
            self.last_pitch_deg = pitch_deg
            return self.last_offsets

        # Kurangi tare
        roll_eff = roll_deg - self.tare_roll
        pitch_eff = pitch_deg - self.tare_pitch

        # Low-pass filter input
        if self.use_input_filter:
            if not self._input_filter_init:
                self._filt_roll_deg = roll_eff
                self._filt_pitch_deg = pitch_eff
                self._input_filter_init = True
            else:
                a_in = self.input_filter_alpha
                self._filt_roll_deg = a_in * roll_eff + (1.0 - a_in) * self._filt_roll_deg
                self._filt_pitch_deg = a_in * pitch_eff + (1.0 - a_in) * self._filt_pitch_deg
            roll_in = self._filt_roll_deg
            pitch_in = self._filt_pitch_deg
        else:
            roll_in = roll_eff
            pitch_in = pitch_eff

        roll_rad = math.radians(roll_in) * self.sign_roll
        pitch_rad = math.radians(pitch_in) * self.sign_pitch

        # Deadband
        e_roll = 0.0 if abs(roll_rad) < self.deadband_rad else roll_rad
        e_pitch = 0.0 if abs(pitch_rad) < self.deadband_rad else pitch_rad

        now = time.time()
        dt = 0.02 if self._last_t is None else min(now - self._last_t, 0.1)
        self._last_t = now

        scale = self.walking_gain_scale if is_walking else 1.0
        kp_eff = self.kp * scale
        ki_eff = self.ki * scale
        kd_eff = self.kd * scale

        # 1. Komponen Proportional
        p_roll = kp_eff * e_roll
        p_pitch = kp_eff * e_pitch

        # 2. Komponen Integral
        self.i_roll = _clamp(self.i_roll + e_roll * dt, -self.i_max, self.i_max)
        self.i_pitch = _clamp(self.i_pitch + e_pitch * dt, -self.i_max, self.i_max)
        i_roll = ki_eff * self.i_roll
        i_pitch = ki_eff * self.i_pitch

        # 3. Komponen Derivative (dengan filter low-pass)
        if dt > 1e-6:
            raw_d_roll = (e_roll - self._prev_e_roll) / dt
            raw_d_pitch = (e_pitch - self._prev_e_pitch) / dt
        else:
            raw_d_roll = 0.0
            raw_d_pitch = 0.0

        a_d = self.d_filter_alpha
        self._filt_d_roll = a_d * raw_d_roll + (1.0 - a_d) * self._filt_d_roll
        self._filt_d_pitch = a_d * raw_d_pitch + (1.0 - a_d) * self._filt_d_pitch
        self._prev_e_roll = e_roll
        self._prev_e_pitch = e_pitch

        d_roll = kd_eff * self._filt_d_roll
        d_pitch = kd_eff * self._filt_d_pitch

        # Total sinyal kontrol
        u_roll = p_roll + i_roll + d_roll
        u_pitch = p_pitch + i_pitch + d_pitch

        # Cache monitoring
        self.last_p_roll = p_roll
        self.last_p_pitch = p_pitch
        self.last_d_roll = d_roll
        self.last_d_pitch = d_pitch
        self.last_u_roll = u_roll
        self.last_u_pitch = u_pitch

        # Hitung dz per kaki
        offsets = {}
        for name, mount in MOUNT_ANGLES.items():
            xb = X_REST * math.cos(mount)
            yb = X_REST * math.sin(mount)
            dz = xb * u_pitch + yb * u_roll
            offsets[name] = _clamp(dz, -self.max_dz, self.max_dz)

        self.last_offsets = offsets
        self.last_roll_deg = roll_deg
        self.last_pitch_deg = pitch_deg
        return offsets

    def step(self, robot, t, vx, vy, wz, roll_deg, pitch_deg):
        """
        Pengganti robot.step(t, vx, vy, wz) yang menyisipkan kompensasi dz
        leveling pada setiap kaki sebelum dikirim ke servo.
        """
        is_walking = (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01)
        offsets = self.compute_offsets(roll_deg, pitch_deg, is_walking=is_walking)

        if robot is None or getattr(robot, "is_simulation", False) or not HAS_DYNAMIXEL:
            return

        robot.sync.clearParam()
        for leg_name, leg_ids in LEGS.items():
            is_group_a = leg_name in GROUP_A_KEYS
            x_t, y_t, z_t = get_leg_target(t, is_group_a, leg_name, vx, vy, wz)
            z_t += offsets[leg_name]
            v_c, v_f, v_t = calculate_ik(x_t, y_t, z_t)
            v_c = max(0, min(1023, v_c))
            v_f = max(0, min(1023, v_f))
            v_t = max(0, min(1023, v_t))
            robot.sync.addParam(leg_ids[0], [DXL_LOBYTE(v_c), DXL_HIBYTE(v_c)])
            robot.sync.addParam(leg_ids[1], [DXL_LOBYTE(v_f), DXL_HIBYTE(v_f)])
            robot.sync.addParam(leg_ids[2], [DXL_LOBYTE(v_t), DXL_HIBYTE(v_t)])
        robot.sync.txPacket()

    def hold_level(self, robot, roll_deg, pitch_deg, x_rest=7.0, z_rest=5.0):
        """
        Mode leveling statis: robot diam di tempat, bodi tetap datar saat dimiringkan.
        """
        offsets = self.compute_offsets(roll_deg, pitch_deg, is_walking=False)

        if robot is None or getattr(robot, "is_simulation", False) or not HAS_DYNAMIXEL:
            return

        robot.sync.clearParam()
        for leg_name, leg_ids in LEGS.items():
            z_t = z_rest + offsets[leg_name]
            v_c, v_f, v_t = calculate_ik(x_rest, 0.0, z_t)
            v_c = max(0, min(1023, v_c))
            v_f = max(0, min(1023, v_f))
            v_t = max(0, min(1023, v_t))
            robot.sync.addParam(leg_ids[0], [DXL_LOBYTE(v_c), DXL_HIBYTE(v_c)])
            robot.sync.addParam(leg_ids[1], [DXL_LOBYTE(v_f), DXL_HIBYTE(v_f)])
            robot.sync.addParam(leg_ids[2], [DXL_LOBYTE(v_t), DXL_HIBYTE(v_t)])
        robot.sync.txPacket()

    def status(self):
        """Mengembalikan kamus status lengkap untuk dashboard/telemetri."""
        return {
            "enabled": self.enabled,
            "roll": self.last_roll_deg,
            "pitch": self.last_pitch_deg,
            "u_roll": math.degrees(self.last_u_roll),
            "u_pitch": math.degrees(self.last_u_pitch),
            "p_roll": math.degrees(self.last_p_roll),
            "p_pitch": math.degrees(self.last_p_pitch),
            "i_roll": math.degrees(self.ki * self.i_roll),
            "i_pitch": math.degrees(self.ki * self.i_pitch),
            "d_roll": math.degrees(self.last_d_roll),
            "d_pitch": math.degrees(self.last_d_pitch),
            "offsets": dict(self.last_offsets),
        }
