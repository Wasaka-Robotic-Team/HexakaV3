"""
WASAKA HEXAPOD - TIKTOK LIVE MOCK TRIGGER (OFFLINE TEST HARNESS)
================================================================
Alat simulasi uji coba lokal interaktif tanpa harus sedang siaran langsung TikTok:
Memungkinkan operator/pengembang menguji seluruh gerakan animasi robot melalui
input keyboard terminal secara langsung.

PILIHAN TOMBOL CEPAT:
  [1] : 🌹 Bunga / Rose         -> Melambai / Dadah (wave)
  [2] : 🍩 Donut / Balon        -> Berputar di tempat (spin)
  [3] : 🎵 TikTok / Musik       -> Bergoyang ritmis (dance)
  [4] : 💖 Hati / Love          -> Berjinjit tinggi (tiptoe)
  [5] : ☕ Kopi / Ice Cream     -> Merendah / Jongkok (crouch)
  [6] : 👑 Mahkota / Follower   -> Membungkuk hormat (bow)
  [7] : 👍 Likes 100x           -> Twist geleng badan (twist)
  [q] : Keluar

AUTHOR: Wasaka Robotic Team
"""

import sys
import threading
import time


class MockTriggerKeyboard:
    """Thread pembaca tombol keyboard untuk simulasi gift offline."""

    KEY_MAP = {
        "1": ("wave", "Bunga (Rose)", "Penonton_1"),
        "2": ("spin", "Donut", "Penonton_2"),
        "3": ("dance", "TikTok Music", "Penonton_3"),
        "4": ("tiptoe", "Hati (Heart)", "Penonton_4"),
        "5": ("crouch", "Kopi Dingin", "Penonton_5"),
        "6": ("bow", "New Follower", "Sultan_Wasaka"),
        "7": ("twist", "100 Likes", "Netizen_TikTok"),
    }

    def __init__(self, queue_manager, on_quit_callback=None):
        self.queue = queue_manager
        self.on_quit = on_quit_callback
        self._thread = None
        self._running = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def print_menu(self):
        print("""
╔══════════════════════════════════════════════════════════════╗
║        SIMULATOR INTERAKTIF TIKTOK LIVE (PILIH TOMBOL)       ║
╠══════════════════════════════════════════════════════════════╣
║  [1] 🌹 Bunga / Rose        → Melambai / Dadah (wave)        ║
║  [2] 🍩 Donut / Balon       → Berputar di tempat (spin)      ║
║  [3] 🎵 TikTok / Musik      → Bergoyang / Joget (dance)      ║
║  [4] 💖 Hati / Love         → Berjinjit tinggi (tiptoe)      ║
║  [5] ☕ Kopi / Ice Cream    → Merendah / Jongkok (crouch)    ║
║  [6] 👑 Follower Baru       → Membungkuk hormat (bow)        ║
║  [7] 👍 Likes Milestone     → Twist geleng badan (twist)     ║
║  [q] Keluar Program                                          ║
╚══════════════════════════════════════════════════════════════╝
Ketik angka tombol lalu tekan ENTER:
""")

    def _loop(self):
        self.print_menu()
        while self._running:
            try:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.1)
                    continue

                cmd = line.strip().lower()
                if cmd == "q":
                    print("[MOCK] Menghentikan simulasi...")
                    self._running = False
                    if self.on_quit:
                        self.on_quit()
                    break

                if cmd in self.KEY_MAP:
                    anim_name, gift_name, sender = self.KEY_MAP[cmd]
                    self.queue.enqueue(anim_name, sender=sender, gift_name=gift_name, count=1)
                else:
                    print(f"[MOCK] Tombol '{cmd}' tidak dikenal. Pilih [1-7] atau [q].")
            except Exception:
                break

    def stop(self):
        self._running = False
