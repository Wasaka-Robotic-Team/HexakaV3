"""
WASAKA HEXAPOD - TIKTOK LIVE INTERACTIVE SYSTEM (MAIN ENTRYPOINT)
=================================================================
Aplikasi utama siaran langsung TikTok Interaktif untuk Hexapod Wasaka:
1. Menghubungkan ke live room TikTok (@username) via TikTokLiveListener.
2. Memproses antrian hadiah (Gift Queue) secara berurutan.
3. Menjalankan animasi fisik 18-DOF yang sesuai:
   - 🌹 Bunga (Rose)        -> Melambai / Dadah (wave)
   - 🍩 Donut / Balon       -> Berputar 360° (spin)
   - 🎵 Musik / TikTok      -> Bergoyang ritmis (dance)
   - 💖 Hati (Heart)        -> Berjinjit tinggi (tiptoe)
   - ☕ Kopi / Ice Cream    -> Merendah / Jongkok (crouch)
   - 👑 Mahkota / Follow    -> Membungkuk hormat (bow)
   - 👍 Likes Milestone     -> Twist badan (twist)
4. Bernapas halus (breathe) saat idle menunggu hadiah berikutnya.
5. Mendukung mode offline simulator (--test) untuk latihan sebelum siaran live.

Penggunaan:
    python apps/tiktok_live/main.py --user @wasakarobotic
    python apps/tiktok_live/main.py --test

AUTHOR: Wasaka Robotic Team
"""

import argparse
import os
import platform
import sys
import time
from pathlib import Path

# Setup path ke root proyek
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from core.locomotion import Hexapod
from core.oled_display import OledDisplay, OledWorker
from apps.tiktok_live.config import DEFAULT_TIKTOK_USERNAME
from apps.tiktok_live.animator import TikTokAnimator
from apps.tiktok_live.queue_manager import AnimationQueueManager
from apps.tiktok_live.tiktok_client import TikTokLiveListener
from apps.tiktok_live.mock_trigger import MockTriggerKeyboard

IS_WINDOWS = platform.system() == "Windows"


def run_tiktok_live(username=DEFAULT_TIKTOK_USERNAME, test_mode=False):
    print("=" * 60)
    print("  WASAKA HEXAPOD - TIKTOK LIVE INTERACTIVE SYSTEM")
    print("=" * 60)

    # 1. Inisialisasi Servo Hexapod
    robot = Hexapod()
    try:
        robot.connect()
        robot.enable_torque()
        robot.stand_by()
    except Exception as e:
        print(f"[WARN] Inisialisasi servo gagal ({e}) -> lanjut mode simulasi.")

    animator = TikTokAnimator(robot)
    queue_mgr = AnimationQueueManager()

    # 2. Inisialisasi OLED (Jika di RPi)
    oled_worker = None
    if not IS_WINDOWS:
        try:
            oled = OledDisplay(enabled=True)
            oled.show_boot(title="WASAKA", subtitle="TIKTOK LIVE")
            oled_worker = OledWorker(oled).start()
        except Exception as e:
            print(f"[WARN] OLED tidak tersedia ({e}).")

    running = True

    def stop_callback():
        nonlocal running
        running = False

    listener = None
    mock_trigger = None

    # 3. Pilih Mode: Live Asli vs Simulasi Uji Coba Offline
    if test_mode:
        print("[MODE] Berjalan dalam mode SIMULASI OFFLINE (--test).")
        mock_trigger = MockTriggerKeyboard(queue_mgr, on_quit_callback=stop_callback)
        mock_trigger.start()
    else:
        listener = TikTokLiveListener(username, queue_mgr)
        if listener.is_available():
            started = listener.start()
            if not started:
                print("[INFO] Beralih otomatis ke mode simulator keyboard.")
                mock_trigger = MockTriggerKeyboard(queue_mgr, on_quit_callback=stop_callback)
                mock_trigger.start()
        else:
            print("[INFO] Beralih otomatis ke mode simulator keyboard (--test).")
            mock_trigger = MockTriggerKeyboard(queue_mgr, on_quit_callback=stop_callback)
            mock_trigger.start()

    print("\n[INFO] Robot standby di meja live. Siap merespons hadiah penonton!\n")

    current_task = None
    anim_start_time = 0.0
    phase = 0.0
    prev_time = time.time()

    try:
        while running:
            now = time.time()
            dt = max(1e-4, now - prev_time)
            prev_time = now
            phase += dt

            # 4. Ambil animasi baru dari antrian jika robot sedang idle
            if current_task is None:
                current_task = queue_mgr.pop_next()
                if current_task is not None:
                    anim_start_time = now
                    print(f"\n>>> [AKSI SEKARANG] {current_task.anim_name.upper()} untuk {current_task.sender} ({current_task.gift_name})")

            # 5. Eksekusi Animasi
            if current_task is not None:
                elapsed = now - anim_start_time
                progress = min(1.0, elapsed / max(0.1, current_task.duration))
                anim_name = current_task.anim_name

                # Router Animasi
                if anim_name == "wave":
                    animator.animate_wave(phase)
                elif anim_name == "spin":
                    animator.animate_spin(phase, direction=1.0)
                elif anim_name == "dance":
                    animator.animate_dance(phase)
                elif anim_name == "tiptoe":
                    animator.animate_tiptoe(progress)
                elif anim_name == "crouch":
                    animator.animate_crouch(progress)
                elif anim_name == "bow":
                    animator.animate_bow(progress)
                elif anim_name == "twist":
                    animator.animate_twist(phase)
                else:
                    animator.animate_dance(phase)

                # Cek apakah durasi animasi telah selesai
                if elapsed >= current_task.duration:
                    print(f">>> [SELESAI] Animasi {current_task.anim_name.upper()} rampung. Kembali standby.")
                    robot.stand_by()
                    queue_mgr.finish_current()
                    current_task = None
            else:
                # 6. Idle State: Napas halus menanti hadiah berikutnya
                animator.animate_breathe(phase)

            time.sleep(0.02)  # ~50 Hz update loop

    except KeyboardInterrupt:
        print("\n[INFO] Menghentikan program...")
    finally:
        running = False
        if listener:
            listener.stop()
        if mock_trigger:
            mock_trigger.stop()
        if oled_worker:
            oled_worker.stop()
        robot.stand_by()
        robot.shutdown()
        print("[INFO] Robot berhasil shutdown dengan aman.")


def main():
    parser = argparse.ArgumentParser(description="Wasaka Hexapod - TikTok Live Interactive System")
    parser.add_argument("--user", type=str, default=DEFAULT_TIKTOK_USERNAME,
                        help=f"Username streamer TikTok Live (default: {DEFAULT_TIKTOK_USERNAME})")
    parser.add_argument("--test", "--mock", action="store_true",
                        help="Jalankan mode uji coba interaktif offline tanpa koneksi live TikTok")
    args = parser.parse_args()

    run_tiktok_live(username=args.user, test_mode=args.test)


if __name__ == "__main__":
    main()
