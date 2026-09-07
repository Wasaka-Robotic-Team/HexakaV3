"""
WASAKA HEXAPOD - NAVIGATION FINITE STATE MACHINE (FSM)
======================================================
Decision layer navigasi otonom & mekanisme recovery:
  CRUISE : Maju lurus, heading-lock aktif (kondisi default).
  AVOID  : Strafe ke sisi kosong (heading tetap). Timeout -> BACKUP.
  BACKUP : Mundur sebentar bila jalan terhalang total -> SCAN.
  SCAN   : Putar di tempat mencari celah (heading-lock nonaktif). Center aman -> CRUISE.
  FAULT  : Robot miring/terbalik/diangkat -> berhenti darurat (safety override).

AUTHOR: Wasaka Robotic Team
"""

import time

# Konstanta timing default
AVOID_TIMEOUT = 3.0    # detik sebelum menyerah strafe dan mundur
BACKUP_TIME   = 1.5    # detik mundur
SCAN_TIME     = 2.5    # detik putar tiap arah
SCAN_YAW      = 0.6    # kecepatan putar saat scan
TILT_DEBOUNCE = 0.3    # detik miring kontinu sebelum masuk FAULT
TILT_CLEAR    = 0.5    # detik tegak kontinu sebelum keluar dari FAULT

# Konstanta kecepatan (normalisasi 0..1)
SPEED_FORWARD              = 1.0
SPEED_STRAFE               = 1.0
SPEED_FORWARD_WHILE_STRAFE = 0.3
SPEED_BACKUP               = 0.7


class NavFSM:
    """
    Finite State Machine untuk navigasi otonom hexapod.
    """

    def __init__(self,
                 avoid_timeout=AVOID_TIMEOUT,
                 backup_time=BACKUP_TIME,
                 scan_time=SCAN_TIME,
                 scan_yaw=SCAN_YAW,
                 speed_forward=SPEED_FORWARD,
                 speed_strafe=SPEED_STRAFE,
                 speed_fwd_strafe=SPEED_FORWARD_WHILE_STRAFE,
                 speed_backup=SPEED_BACKUP,
                 use_imu_safety=True):
        self.state = "CRUISE"
        self.t_enter = time.time()
        self.avoid_side = "LEFT"
        self.scan_dir = 1.0
        self._tilt_since = None
        self._clear_since = None

        self.avoid_timeout = avoid_timeout
        self.backup_time = backup_time
        self.scan_time = scan_time
        self.scan_yaw = scan_yaw
        self.speed_forward = speed_forward
        self.speed_strafe = speed_strafe
        self.speed_fwd_strafe = speed_fwd_strafe
        self.speed_backup = speed_backup
        self.use_imu_safety = use_imu_safety

    def _go(self, new_state):
        if new_state != self.state:
            self.state = new_state
            self.t_enter = time.time()

    def _elapsed(self):
        return time.time() - self.t_enter

    def update(self, decision, blk_L, blk_C, blk_R, dL, dR, tilt_now, wall_front=False):
        """
        Update state machine dan kembalikan instruksi gerak.

        Returns:
            dict: {
                'vx': float (-mundur / +maju),
                'vy': float (-kiri / +kanan),
                'yaw_cmd': float,
                'hold_heading': bool
            }
        """
        now = time.time()

        # 1. Safety Override: Tilt / Roboh / Diangkat
        if self.use_imu_safety:
            if tilt_now:
                self._tilt_since = self._tilt_since or now
                self._clear_since = None
                if now - self._tilt_since >= TILT_DEBOUNCE:
                    self._go("FAULT")
            else:
                self._clear_since = self._clear_since or now
                self._tilt_since = None

        if self.state == "FAULT":
            if (not tilt_now) and self._clear_since and (now - self._clear_since >= TILT_CLEAR):
                self._go("CRUISE")
            return dict(vx=0.0, vy=0.0, yaw_cmd=0.0, hold_heading=False)

        # 2. Wall-in-front override (Appearance layer ground-seg mepet dinding)
        if wall_front and self.state in ("CRUISE", "AVOID"):
            self._go("BACKUP")

        path_clear = (not blk_C) and (not (blk_L and blk_R))

        # --- CRUISE ---
        if self.state == "CRUISE":
            if decision in ("LEFT", "RIGHT"):
                self.avoid_side = decision
                self._go("AVOID")
            else:
                # Catatan: Pada koordinat gait hexapod Wasaka, vx negatif = maju
                return dict(vx=-self.speed_forward, vy=0.0, yaw_cmd=0.0, hold_heading=True)

        # --- AVOID ---
        if self.state == "AVOID":
            if path_clear:
                self._go("CRUISE")
                return dict(vx=-self.speed_forward, vy=0.0, yaw_cmd=0.0, hold_heading=True)
            if self._elapsed() > self.avoid_timeout:
                self._go("BACKUP")
            else:
                if decision in ("LEFT", "RIGHT"):
                    self.avoid_side = decision
                vy = -self.speed_strafe if self.avoid_side == "LEFT" else self.speed_strafe
                return dict(vx=-self.speed_fwd_strafe, vy=vy, yaw_cmd=0.0, hold_heading=True)

        # --- BACKUP ---
        if self.state == "BACKUP":
            if self._elapsed() > self.backup_time:
                self.scan_dir = 1.0 if dL <= dR else -1.0
                self._go("SCAN")
            else:
                return dict(vx=self.speed_backup, vy=0.0, yaw_cmd=0.0, hold_heading=True)

        # --- SCAN ---
        if self.state == "SCAN":
            if not blk_C:
                self._go("CRUISE")
                return dict(vx=-self.speed_forward, vy=0.0, yaw_cmd=0.0, hold_heading=True)
            if self._elapsed() > self.scan_time:
                self.scan_dir *= -1.0
                self.t_enter = now
            return dict(vx=0.0, vy=0.0, yaw_cmd=self.scan_yaw * self.scan_dir, hold_heading=False)

        return dict(vx=-self.speed_forward, vy=0.0, yaw_cmd=0.0, hold_heading=True)
