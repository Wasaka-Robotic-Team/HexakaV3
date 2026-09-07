"""
WASAKA HEXAPOD - TIKTOK LIVE QUEUE MANAGER
===========================================
Pengelola antrian animasi (thread-safe queue) untuk TikTok Live.

Ketika siaran langsung ramai penonton, seringkali hadiah (gifts) dikirim secara
bersamaan atau bertubi-tubi (streak/combo). Queue Manager ini memastikan:
1. Setiap animasi dijalankan berurutan (FIFO) tanpa tumpang-tindih perintah servo.
2. Streak gift dapat digabung atau dihitung durasi tambahannya secara proporsional.
3. Menampilkan status antrian dan gift terkini untuk telemetri atau layar OLED.

AUTHOR: Wasaka Robotic Team
"""

import queue
import threading
import time
from .config import ANIM_DURATIONS, DEFAULT_UNMAPPED_GIFT_ANIM


class AnimationTask:
    """Representasi satu tugas animasi hadiah dalam antrian."""

    def __init__(self, anim_name, sender="Penonton", gift_name="Gift", count=1):
        self.anim_name = anim_name
        self.sender = sender
        self.gift_name = gift_name
        self.count = max(1, count)

        base_duration = ANIM_DURATIONS.get(anim_name, 3.0)
        # Tambahan durasi kecil jika streak combo banyak (max cap 10 detik)
        if count > 1:
            self.duration = min(10.0, base_duration + min(5.0, count * 0.4))
        else:
            self.duration = base_duration

        self.created_at = time.time()

    def __str__(self):
        return f"{self.sender} mengirim {self.gift_name} (x{self.count}) -> [{self.anim_name.upper()}] ({self.duration:.1f}s)"


class AnimationQueueManager:
    """
    Manajer antrian thread-safe untuk mengelola request animasi.
    """

    def __init__(self, max_queue_size=50):
        self._queue = queue.Queue(maxsize=max_queue_size)
        self._lock = threading.Lock()
        self.current_task = None
        self.total_processed = 0

    def enqueue(self, anim_name, sender="Penonton", gift_name="Gift", count=1):
        """Menambahkan tugas animasi ke antrian."""
        task = AnimationTask(anim_name, sender, gift_name, count)
        try:
            self._queue.put_nowait(task)
            print(f"[QUEUE] + {task}")
            return True
        except queue.Full:
            print(f"[WARN] Antrian animasi penuh! Melewatkan {gift_name} dari {sender}.")
            return False

    def pop_next(self):
        """Mengambil tugas berikutnya dari antrian (non-blocking)."""
        try:
            task = self._queue.get_nowait()
            with self._lock:
                self.current_task = task
                self.total_processed += 1
            return task
        except queue.Empty:
            with self._lock:
                self.current_task = None
            return None

    def finish_current(self):
        """Menandai tugas aktif saat ini telah selesai."""
        with self._lock:
            self.current_task = None

    def get_queue_length(self):
        return self._queue.qsize()

    def get_current_info(self):
        with self._lock:
            if self.current_task is None:
                return "IDLE", "Menunggu Hadiah...", 0
            return (
                self.current_task.anim_name,
                f"{self.current_task.sender} ({self.current_task.gift_name})",
                self._queue.qsize()
            )
