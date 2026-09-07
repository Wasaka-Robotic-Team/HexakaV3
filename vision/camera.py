"""
WASAKA HEXAPOD - CAMERA MODULE
===============================
Inisialisasi kamera dengan fallback indeks otomatis & threaded capture loop.

Menyediakan:
- open_camera(): Otomatis mencoba indeks kamera (0-5) dengan backend yang tepat
  (DSHOW untuk Windows, V4L2 untuk Linux/Raspberry Pi/Jetson).
- CameraStream: Pembacaan frame di thread terpisah (decoupled I/O) agar gait
  robot tidak mengalami 'jitter' / 'lag' saat proses inferensi atau I/O kamera.

AUTHOR: Wasaka Robotic Team
"""

import platform
import threading
import time
import cv2

IS_WINDOWS = platform.system() == "Windows"
DEFAULT_BACKEND = cv2.CAP_DSHOW if IS_WINDOWS else cv2.CAP_V4L2


def open_camera(source=None, width=640, height=480, backend=None):
    """
    Membuka kamera dengan fallback indeks otomatis (0..5).

    Args:
        source: Indeks kamera spesifik (int), jika None akan mencoba 1 lalu 0..5
        width: Lebar frame yang diminta
        height: Tinggi frame yang diminta
        backend: cv2 VideoCapture backend (default DSHOW di Windows, V4L2 di Linux)

    Returns:
        tuple: (cv2.VideoCapture, index_yang_berhasil) atau (None, -1)
    """
    if backend is None:
        backend = DEFAULT_BACKEND

    candidates = []
    if source is not None:
        candidates.append(source)
    else:
        # Default prioritas: di Windows biasanya webcam eksternal ada di index 1
        preferred = 1 if IS_WINDOWS else 0
        candidates = [preferred, 0, 1, 2, 3, 4, 5]

    seen = set()
    for idx in candidates:
        if idx in seen:
            continue
        seen.add(idx)

        cap = cv2.VideoCapture(idx, backend)
        if not cap.isOpened() and backend != cv2.CAP_ANY:
            # Coba fallback ke default backend jika DSHOW/V4L2 gagal
            cap.release()
            cap = cv2.VideoCapture(idx)

        if cap.isOpened():
            # Tes apakah frame benar-benar bisa dibaca
            ret, _ = cap.read()
            if ret:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FPS, 30)
                try:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                except Exception:
                    pass
                print(f"[KAMERA] Berhasil membuka kamera di indeks {idx} ({width}x{height})")
                return cap, idx
            cap.release()

    print("[ERROR] Gagal membuka kamera pada semua indeks kandidat (0-5)!")
    return None, -1


class CameraStream:
    """
    Pengambil frame kamera ter-thread (non-blocking).
    """

    def __init__(self, cap, threaded=True):
        self.cap = cap
        self.threaded = threaded
        self.lock = threading.Lock()
        self.frame = None
        self.count = 0
        self.running = False
        self.thread = None

    def start(self):
        if not self.threaded or self.cap is None:
            return self
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return self

    def _loop(self):
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.01)
                continue
            ret, f = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = f
                    self.count += 1
            else:
                time.sleep(0.005)

    def read(self):
        """Mengembalikan (ret, frame, count)."""
        if self.threaded:
            with self.lock:
                if self.frame is None:
                    return False, None, self.count
                return True, self.frame.copy(), self.count
        if self.cap is None:
            return False, None, self.count
        ret, f = self.cap.read()
        if ret:
            self.count += 1
        return ret, f, self.count

    def read_frame(self):
        """Mengembalikan (ret, frame) sederhana."""
        ret, f, _ = self.read()
        return ret, f

    def stop(self):
        self.running = False
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def is_opened(self):
        return self.cap is not None and self.cap.isOpened()
