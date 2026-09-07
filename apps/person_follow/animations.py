"""
WASAKA HEXAPOD - ROBOT ANIMATIONS MODULE
========================================
Modul animasi ekspresif untuk demo pameran dan interaksi pengunjung:
1. Greeting (Membungkuk ramah saat pertama kali mendeteksi orang).
2. Idle Breathing (Gerakan naik-turun tubuh menyerupai napas saat diam).
3. Idle Waving (Mengangkat kaki depan kanan dan melambai ke arah penonton).

AUTHOR: Wasaka Robotic Team
"""

import math
import random
import time
from core.locomotion import calculate_ik, LEGS


class AnimationController:
    """
    Pengendali animasi ekspresif robot hexapod.
    """

    def __init__(self, robot):
        self.robot = robot
        self.phase = 0.0
        self.greeting_timer = 0.0
        self.idle_timer = 0.0
        self.idle_duration = 3.5
        self.idle_mode = "BREATHE"

    def is_greeting(self):
        return self.greeting_timer > 0.0

    def start_greeting(self, duration=1.2):
        self.greeting_timer = duration

    def play_greeting(self, dt):
        """Animasi membungkuk ramah (bow cycle) saat target baru terdeteksi."""
        if self.robot is None or getattr(self.robot, "is_simulation", False):
            self.greeting_timer -= dt
            return

        self.phase += dt
        self.greeting_timer -= dt

        # Siklus 1.2 detik: 0.6s turun, 0.6s naik
        if self.greeting_timer > 0.6:
            progress = (1.2 - self.greeting_timer) / 0.6
        else:
            progress = max(0.0, self.greeting_timer / 0.6)

        z = 5.0 - 1.2 * progress
        x = 7.0 + 0.8 * progress
        self.robot.stand_by(x=x, y=0.0, z=z)

    def play_breathing(self, dt, period=2.5, amplitude=0.4):
        """Animasi bernapas saat posisi HOLD_DISTANCE."""
        self.phase += dt
        z_breathing = 5.0 + amplitude * math.sin(2 * math.pi * self.phase / period)
        if self.robot is not None:
            self.robot.stand_by(x=7.0, y=0.0, z=z_breathing)

    def play_idle(self, dt):
        """Animasi acak (Breathing & Dadah Kaki Depan) saat tidak ada target (SEARCH)."""
        self.phase += dt
        self.idle_timer += dt

        # Ganti mode acak setiap beberapa detik
        if self.idle_timer >= self.idle_duration:
            self.idle_timer = 0.0
            self.idle_duration = random.uniform(3.0, 5.0)
            self.idle_mode = random.choice(["BREATHE", "WAVE"])

        if self.idle_mode == "BREATHE":
            self.play_breathing(dt)
        elif self.idle_mode == "WAVE":
            if self.robot is None or getattr(self.robot, "is_simulation", False):
                return

            z_stand = 5.0 + 0.3 * math.sin(2 * math.pi * self.phase / 2.5)
            c_std, f_std, t_std = calculate_ik(7.0, 0.0, z_stand)

            # L4_DepanKanan diangkat & melambai
            c_wave = c_std + int(75 * math.sin(7.0 * self.phase))
            f_wave = max(100, f_std - 180)
            t_wave = min(950, t_std + 230 + int(50 * math.sin(7.0 * self.phase)))

            for leg_name, leg_ids in LEGS.items():
                if leg_name == "L4_DepanKanan":
                    self.robot.move_servo(leg_ids[0], c_wave)
                    self.robot.move_servo(leg_ids[1], f_wave)
                    self.robot.move_servo(leg_ids[2], t_wave)
                else:
                    self.robot.move_servo(leg_ids[0], c_std)
                    self.robot.move_servo(leg_ids[1], f_std)
                    self.robot.move_servo(leg_ids[2], t_std)
