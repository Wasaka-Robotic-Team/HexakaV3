"""
WASAKA HEXAPOD - TIKTOK CHOREOGRAPHY ANIMATOR
=============================================
Koreografi gerakan ekspresif 18-DOF untuk live stream interaktif:
- wave()    : Melambaikan kaki depan ke penonton (Gift Bunga / Rose)
- spin()    : Berputar 360° di tempat dengan tripod gait (Gift Donut / Balon)
- dance()   : Bergoyang roll & pitch ritmis mengikuti irama (Gift TikTok / Musik)
- tiptoe()  : Mendorong bodi terangkat tinggi di ujung kaki (Gift Hati / Star)
- crouch()  : Merendahkan bodi hampir menyentuh lantai lalu bangkit (Gift Kopi / Ice Cream)
- bow()     : Membungkuk hormat santun ke arah kamera (Gift Mahkota / Follower Baru)
- twist()   : Menggelengkan badan kiri-kanan cepat di tempat (Milestone 100+ Likes)
- breathe() : Gerakan napas lembut saat standby menunggu hadiah penonton

AUTHOR: Wasaka Robotic Team
"""

import math
import time
from core.locomotion import calculate_ik, LEGS, MOUNT_ANGLES, X_REST, Z_GROUND


class TikTokAnimator:
    """
    Eksekutor animasi fisik robot hexapod untuk interaksi TikTok Live.
    """

    def __init__(self, robot=None):
        self.robot = robot

    def animate_breathe(self, phase, amplitude=0.4, period=2.5):
        """Gerakan bodi naik-turun halus menyerupai napas saat diam."""
        if self.robot is None:
            return
        z = Z_GROUND + amplitude * math.sin(2 * math.pi * phase / period)
        self.robot.stand_by(x=X_REST, y=0.0, z=z)

    def animate_wave(self, phase):
        """
        Kaki depan kanan (L4) terangkat tinggi ke atas dan melambai antusias.
        5 kaki lainnya menopang bodi dengan kokoh di lantai.
        """
        if self.robot is None or getattr(self.robot, "is_simulation", False):
            return

        z_base = Z_GROUND + 0.25 * math.sin(2 * math.pi * phase / 2.0)
        c_std, f_std, t_std = calculate_ik(X_REST, 0.0, z_base)

        # Coxa (servo 10 - pangkal): melambai kiri-kanan secara ritmis
        c_wave = c_std + int(85 * math.sin(7.5 * phase))
        # Femur (servo 11 - tengah): terangkat tinggi ke atas
        f_wave = max(100, f_std - 190)
        # Tibia (servo 12 - ujung): terangkat tegak dan mengibas
        t_wave = min(950, t_std + 240 + int(60 * math.sin(7.5 * phase)))

        for leg_name, leg_ids in LEGS.items():
            if leg_name == "L4_DepanKanan":
                self.robot.move_servo(leg_ids[0], c_wave)
                self.robot.move_servo(leg_ids[1], f_wave)
                self.robot.move_servo(leg_ids[2], t_wave)
            else:
                self.robot.move_servo(leg_ids[0], c_std)
                self.robot.move_servo(leg_ids[1], f_std)
                self.robot.move_servo(leg_ids[2], t_std)

    def animate_spin(self, phase, direction=1.0):
        """Berputar di tempat (rotasi yaw) menggunakan tripod gait."""
        if self.robot is None:
            return
        wz = 0.85 * direction
        self.robot.step(phase, vx=0.0, vy=0.0, wz=wz)

    def animate_dance(self, phase):
        """
        Bergoyang roll & pitch ritmis (Dance / Joget).
        Telapak kaki tetap menempel di lantai sementara bodi bergoyang lentur.
        """
        if self.robot is None or getattr(self.robot, "is_simulation", False):
            return

        roll_deg = 12.0 * math.sin(4.5 * phase)
        pitch_deg = 7.0 * math.cos(4.5 * phase)
        z_pulse = Z_GROUND + 0.6 * math.sin(9.0 * phase)

        u_roll = math.radians(roll_deg)
        u_pitch = math.radians(pitch_deg)

        for leg_name, leg_ids in LEGS.items():
            mount = MOUNT_ANGLES[leg_name]
            xb = X_REST * math.cos(mount)
            yb = X_REST * math.sin(mount)
            dz = xb * u_pitch + yb * u_roll

            c, f, t = calculate_ik(X_REST, 0.0, z_pulse + dz)
            self.robot.move_servo(leg_ids[0], c)
            self.robot.move_servo(leg_ids[1], f)
            self.robot.move_servo(leg_ids[2], t)

    def animate_tiptoe(self, progress):
        """
        Menaikkan kaki / Berjinjit tinggi di atas ujung claws (progress 0..1).
        Bodi terangkat ke ketinggian maksimal secara anggun lalu turun kembali.
        """
        if self.robot is None:
            return
        lift_factor = math.sin(math.pi * max(0.0, min(1.0, progress)))
        z_tiptoe = Z_GROUND + 3.2 * lift_factor
        x_tiptoe = X_REST - 0.7 * lift_factor  # Kaki lebih tegak lurus
        self.robot.stand_by(x=x_tiptoe, y=0.0, z=z_tiptoe)

    def animate_crouch(self, progress):
        """
        Merendahkan bodi / Jongkok hingga dekat lantai (progress 0..1).
        Kaki mekar keluar dan lutut menekuk dalam lalu bangkit kembali.
        """
        if self.robot is None:
            return
        drop_factor = math.sin(math.pi * max(0.0, min(1.0, progress)))
        z_crouch = max(2.2, Z_GROUND - 2.8 * drop_factor)
        x_crouch = X_REST + 1.2 * drop_factor
        self.robot.stand_by(x=x_crouch, y=0.0, z=z_crouch)

    def animate_bow(self, progress):
        """
        Membungkuk hormat (Bow greeting) ke penonton di depan kamera (progress 0..1).
        Bagian depan merunduk, bagian belakang sedikit terangkat santun.
        """
        if self.robot is None or getattr(self.robot, "is_simulation", False):
            return

        bow_factor = math.sin(math.pi * max(0.0, min(1.0, progress)))
        pitch_rad = math.radians(-14.0 * bow_factor)

        for leg_name, leg_ids in LEGS.items():
            mount = MOUNT_ANGLES[leg_name]
            xb = X_REST * math.cos(mount)
            dz = xb * pitch_rad
            c, f, t = calculate_ik(X_REST, 0.0, Z_GROUND + dz)
            self.robot.move_servo(leg_ids[0], c)
            self.robot.move_servo(leg_ids[1], f)
            self.robot.move_servo(leg_ids[2], t)

    def animate_twist(self, phase):
        """Menggelengkan badan / Twist kiri-kanan cepat di tempat."""
        if self.robot is None or getattr(self.robot, "is_simulation", False):
            return

        c_std, f_std, t_std = calculate_ik(X_REST, 0.0, Z_GROUND)
        twist_offset = int(70 * math.sin(11.0 * phase))

        for leg_ids in LEGS.values():
            self.robot.move_servo(leg_ids[0], c_std + twist_offset)
            self.robot.move_servo(leg_ids[1], f_std)
            self.robot.move_servo(leg_ids[2], t_std)
