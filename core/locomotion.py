"""
WASAKA HEXAPOD - LOCOMOTION MODULE
===================================
Inverse Kinematics + Tripod Gait Generator + Dynamixel AX Servo Driver.

Berisi:
- Konfigurasi servo Dynamixel (18 servo, ID 1-18, Protocol 1.0, 1Mbps)
- Geometri kaki 3-DOF & Inverse Kinematics (calculate_ik)
- Tripod Gait Generator (get_leg_target)
- Kelas Hexapod untuk kontrol gerak servo tingkat tinggi.

Jika dynamixel_sdk tidak terinstall (misal diuji di laptop), modul tetap
dapat di-import dan beralih ke mode simulasi (stub/no-op) tanpa crash.

AUTHOR: Wasaka Robotic Team
"""

import time
import math
import platform

# Coba import Dynamixel SDK dengan fallback simulasi
try:
    from dynamixel_sdk import (
        PortHandler,
        PacketHandler,
        GroupSyncWrite,
        DXL_LOBYTE,
        DXL_HIBYTE,
    )
    HAS_DYNAMIXEL = True
except ImportError:
    HAS_DYNAMIXEL = False
    PortHandler = None
    PacketHandler = None
    GroupSyncWrite = None
    DXL_LOBYTE = lambda v: int(v) & 0xFF
    DXL_HIBYTE = lambda v: (int(v) >> 8) & 0xFF


# ============================================================
# 1. HARDWARE CONFIGURATION - SERVO DYNAMIXEL
# ============================================================
IS_WINDOWS = platform.system() == "Windows"

ADDR_AX_TORQUE_ENABLE = 24
ADDR_AX_GOAL_POSITION = 30
PROTOCOL_VERSION      = 1.0
BAUDRATE              = 1000000
DEVICENAME            = 'COM21' if IS_WINDOWS else '/dev/ttyUSB0'

# Link lengths (cm)
L1, L2, L3 = 3.4, 4.0, 5.45
CENTER_VAL  = 512
DEG_TO_DYN  = 1023 / 300.0

# Leg IDs (Coxa, Femur, Tibia) - searah jarum jam
LEGS = {
    "L1_DepanKiri":     [1,  2,  3],      # Front Left
    "L2_TengahKiri":    [4,  5,  6],      # Middle Left
    "L3_BelakangKiri":  [7,  8,  9],      # Rear Left
    "L4_DepanKanan":    [10, 11, 12],     # Front Right
    "L5_TengahKanan":   [13, 14, 15],     # Middle Right
    "L6_BelakangKanan": [16, 17, 18],     # Rear Right
}
GROUP_A_KEYS = ["L1_DepanKiri", "L3_BelakangKiri", "L5_TengahKanan"]
ALL_IDS = [sid for leg in LEGS.values() for sid in leg]

# Mounting angles (relation to forward direction)
MOUNT_ANGLES = {
    "L1_DepanKiri":     math.radians(45),
    "L2_TengahKiri":    math.radians(90),
    "L3_BelakangKiri":  math.radians(135),
    "L4_DepanKanan":    math.radians(-45),
    "L5_TengahKanan":   math.radians(-90),
    "L6_BelakangKanan": math.radians(-135),
}


# ============================================================
# 2. GAIT CONFIGURATION - TRIPOD LOCOMOTION
# ============================================================
X_REST        = 7.0      # rest position x (cm)
Z_GROUND      = 5.0      # ground level z (cm)
LIFT_HEIGHT   = 4.0      # swing foot height (cm)
STRIDE_LENGTH = 3.5      # stride length (cm)
STEP_DURATION = 1.0      # duration of 1 gait cycle (seconds)
DUTY          = 0.4      # 40% swing, 60% stance
ROT_STRIDE    = 2.5      # tangential stride per |wz|=1 (cm) untuk steering yaw


# ============================================================
# 3. INVERSE KINEMATICS
# ============================================================

def calculate_ik(x, y, z):
    """
    Menghitung sudut servo (coxa, femur, tibia) dari koordinat kartesius 3D (x, y, z).

    Args:
        x: posisi sumbu longitudinal kaki (cm)
        y: posisi lateral kaki (cm)
        z: posisi vertikal kaki (cm)

    Returns:
        tuple: (coxa_val, femur_val, tibia_val) dalam unit servo Dynamixel (0-1023)
    """
    if x == 0 and y == 0:
        x = 0.001
    gamma_rad = math.atan2(y, x)
    L_eff = math.sqrt(x**2 + y**2) - L1
    D = math.sqrt(L_eff**2 + z**2)
    if D > (L2 + L3):
        D = (L2 + L3) - 0.01
    alpha1 = math.atan2(z, L_eff)
    alpha2 = math.acos(max(-1.0, min(1.0, (L2**2 + D**2 - L3**2) / (2 * L2 * D))))
    theta_femur_rad = alpha1 - alpha2
    beta = math.acos(max(-1.0, min(1.0, (L2**2 + L3**2 - D**2) / (2 * L2 * L3))))
    theta_tibia_rad = math.pi - beta
    coxa_val  = int(CENTER_VAL - (math.degrees(gamma_rad)       * DEG_TO_DYN))
    femur_val = int(CENTER_VAL + (math.degrees(theta_femur_rad) * DEG_TO_DYN))
    tibia_val = int(CENTER_VAL - (math.degrees(theta_tibia_rad) * DEG_TO_DYN))
    return coxa_val, femur_val, tibia_val


# ============================================================
# 4. GAIT TARGET GENERATOR - TRIPOD
# ============================================================

def get_leg_target(t, is_group_a, leg_name, vx, vy, wz=0.0):
    """
    Target posisi kaki dalam siklus tripod gait.

    Args:
        t: waktu berjalan dalam detik
        is_group_a: boolean grup kaki (A atau B)
        leg_name: nama kaki dari LEGS.keys()
        vx: kecepatan maju (+1) / mundur (-1)
        vy: kecepatan geser samping kanan (+1) / kiri (-1)
        wz: kecepatan rotasi badan (+1 putar kanan / -1 putar kiri)
    """
    phase_offset = 0.0 if is_group_a else (STEP_DURATION / 2.0)
    cycle_time = (t + phase_offset) % STEP_DURATION
    progress = cycle_time / STEP_DURATION

    mount = MOUNT_ANGLES[leg_name]

    trans_mag = math.sqrt(vx**2 + vy**2)
    if trans_mag > 0.001:
        move_angle = math.atan2(vy, vx)
        stride_trans_x = -STRIDE_LENGTH * trans_mag * math.cos(mount - move_angle)
        stride_trans_y =  STRIDE_LENGTH * trans_mag * math.sin(mount - move_angle)
    else:
        stride_trans_x = 0.0
        stride_trans_y = 0.0

    # Komponen rotasi (steering yaw): tangensial seragam tiap kaki (lokal +y)
    stride_x = stride_trans_x
    stride_y = stride_trans_y + ROT_STRIDE * wz

    stride_total = math.sqrt(stride_x**2 + stride_y**2)
    max_stride = STRIDE_LENGTH * 1.2
    if stride_total > max_stride:
        scale = max_stride / stride_total
        stride_x *= scale
        stride_y *= scale

    if progress < DUTY:
        z_offset = LIFT_HEIGHT * math.sin(2 * math.pi * progress)
        s = progress / DUTY - 0.5
    else:
        z_offset = 0.0
        s = 0.5 - (progress - DUTY) / (1.0 - DUTY)

    # Angkat kaki proporsional terhadap total gerak (termasuk yaw)
    motion_mag = min(math.sqrt(vx**2 + vy**2 + (ROT_STRIDE * wz / STRIDE_LENGTH) ** 2), 1.0)
    z_offset *= motion_mag

    x_target = X_REST + stride_x * s
    y_target = stride_y * s
    z_target = Z_GROUND - z_offset

    return x_target, y_target, z_target


# ============================================================
# 5. HEXAPOD SERVO DRIVER
# ============================================================

class Hexapod:
    """
    Driver hardware hexapod (Dynamixel AX, protokol 1.0).

    Mengabstraksi komunikasi serial servo 18-DOF. Jika dynamixel_sdk tidak
    tersedia, objek akan beroperasi dalam mode simulasi tanpa error.
    """

    def __init__(self, devicename=DEVICENAME, baudrate=BAUDRATE):
        self.devicename = devicename
        self.baudrate = baudrate
        self.is_simulation = not HAS_DYNAMIXEL
        self.port = None
        self.packet = None
        self.sync = None
        self._standby = None

        if HAS_DYNAMIXEL:
            try:
                self.port = PortHandler(devicename)
                self.packet = PacketHandler(PROTOCOL_VERSION)
                self.sync = GroupSyncWrite(self.port, self.packet, ADDR_AX_GOAL_POSITION, 2)
            except Exception as e:
                print(f"[WARN] Inisialisasi port Dynamixel gagal ({e}), aktifkan mode simulasi.")
                self.is_simulation = True

    def connect(self):
        if self.is_simulation:
            print(f"[SIMULASI] Hexapod terhubung di {self.devicename} (mode no-op).")
            return self

        if not self.port.openPort():
            raise RuntimeError(f"Gagal membuka serial port {self.devicename}!")
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f"Gagal set baudrate ke {self.baudrate}!")
        print(f"[INFO] Hexapod connected on {self.devicename} @ {self.baudrate} bps.")
        return self

    def enable_torque(self):
        if self.is_simulation:
            return
        for sid in ALL_IDS:
            self.packet.write1ByteTxRx(self.port, sid, ADDR_AX_TORQUE_ENABLE, 1)

    def disable_torque(self):
        if self.is_simulation:
            return
        for sid in ALL_IDS:
            self.packet.write1ByteTxRx(self.port, sid, ADDR_AX_TORQUE_ENABLE, 0)

    def move_servo(self, dxl_id, goal_position):
        if self.is_simulation:
            return
        self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_AX_GOAL_POSITION, goal_position)

    def stand_by(self, x=7.0, y=0.0, z=5.0):
        """Pose berdiri netral; posisi ini disimpan & dipakai saat shutdown."""
        c, f, t = calculate_ik(x, y, z)
        self._standby = (c, f, t)
        if self.is_simulation:
            return
        for leg in LEGS.values():
            self.move_servo(leg[0], c)
            self.move_servo(leg[1], f)
            self.move_servo(leg[2], t)

    def step(self, t, vx, vy, wz=0.0):
        """Kirim 1 frame gait untuk seluruh 18 servo sekaligus via GroupSyncWrite."""
        if self.is_simulation:
            return

        self.sync.clearParam()
        for leg_name, leg_ids in LEGS.items():
            is_group_a = leg_name in GROUP_A_KEYS
            x_t, y_t, z_t = get_leg_target(t, is_group_a, leg_name, vx, vy, wz)
            v_c, v_f, v_t = calculate_ik(x_t, y_t, z_t)
            v_c = max(0, min(1023, v_c))
            v_f = max(0, min(1023, v_f))
            v_t = max(0, min(1023, v_t))
            self.sync.addParam(leg_ids[0], [DXL_LOBYTE(v_c), DXL_HIBYTE(v_c)])
            self.sync.addParam(leg_ids[1], [DXL_LOBYTE(v_f), DXL_HIBYTE(v_f)])
            self.sync.addParam(leg_ids[2], [DXL_LOBYTE(v_t), DXL_HIBYTE(v_t)])
        self.sync.txPacket()

    def shutdown(self):
        """Kembali ke posisi standby, melepas torsi, lalu menutup port serial."""
        if self._standby is not None and not self.is_simulation:
            c, f, t = self._standby
            for leg in LEGS.values():
                self.move_servo(leg[0], c)
                self.move_servo(leg[1], f)
                self.move_servo(leg[2], t)
            time.sleep(0.5)
        try:
            self.disable_torque()
        finally:
            if self.port is not None and not self.is_simulation:
                self.port.closePort()
        print("[INFO] Hexapod servo driver shutdown selesai.")
