"""
OLED STATUS DISPLAY - SSD1306 128x64 via I2C (BERBAGI BUS dengan BNO055)
=======================================================================

Modul tampilan status ringkas di OLED, TERPISAH & stabil (jarang berubah).
Menampilkan state FSM, aksi, kecepatan (vx/vy/wz), FPS, indikator halangan
L/C/R, dan status IMU/peringatan WALL pada panel 128x64.

I2C SATU BUS untuk DUA device:
    I2C itu bus -> BNO055 (mis. 0x28) & OLED SSD1306 (0x3C) boleh berbagi pin
    SDA/SCL yang sama. KUNCI: bus `busio.I2C` hanya boleh dibuat SEKALI. Karena
    itu main.py membuat bus di HeadingSensor lalu mem-passing-kannya ke sini
    (parameter `i2c`). Bila `i2c=None`, modul ini membuat bus sendiri (dipakai
    hanya bila IMU tidak aktif).

Throttle:
    Refresh penuh SSD1306 lewat I2C relatif lambat. Update di-throttle ke
    OLED_PERIOD (default ~5 Hz) supaya tidak mengganggu timing gait di main loop.

Dependensi (Raspberry Pi):
    pip install adafruit-circuitpython-ssd1306 pillow
    (sudah otomatis menarik adafruit-blinka untuk board/busio)
Di laptop tanpa library/hardware -> import aman, objek auto-nonaktif (no-op).

AUTHOR: Wasaka Robotic Team
"""

import sys
import os
import time
import math

# Pastikan direktori file ini ada di sys.path agar import lokal oled_logo sukses
_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

# Library OLED hanya ada di RPi. Di laptop -> _OLED_LIBS=False, modul jadi no-op.
try:
    import board
    import busio
    import adafruit_ssd1306
    from PIL import Image, ImageDraw, ImageFont, ImageOps
    _OLED_LIBS = True
except Exception:
    _OLED_LIBS = False

# Bitmap logo embedded (hasil ekspor Arduino). Opsional -> di-guard.
try:
    import oled_logo
except Exception:
    oled_logo = None


# Label aksi (samakan istilah dengan dashboard)
ACTION_LABEL = {
    "FORWARD": "MAJU", "IGNORE": "MAJU",
    "LEFT": "GESER KIRI", "RIGHT": "GESER KANAN",
}


class OledDisplay:
    """Tampilan status SSD1306 128x64. Semua method aman dipanggil walau OLED
    tidak tersedia (akan jadi no-op)."""

    def __init__(self, enabled=True, i2c=None, addr=0x3C, width=128, height=64,
                 period=0.20, logo_path="logo1.png", invert_logo=False):
        self.ok = False
        self.w = int(width)
        self.h = int(height)
        self.period = period
        self.logo_path = logo_path
        self.invert_logo = invert_logo
        self._last = 0.0
        self.disp = None

        if not enabled:
            print("[OLED] Dinonaktifkan -> dilewati.")
            return
        if not _OLED_LIBS:
            print("[OLED] Library tak tersedia (adafruit_ssd1306/PIL/board) -> dilewati.")
            return
        try:
            if i2c is None:
                i2c = busio.I2C(board.SCL, board.SDA)  # hanya bila IMU tak buat bus
            self.disp = adafruit_ssd1306.SSD1306_I2C(self.w, self.h, i2c, addr=addr)
            self.disp.fill(0)
            self.disp.show()
            self.image = Image.new("1", (self.w, self.h))
            self.draw = ImageDraw.Draw(self.image)
            self.font = ImageFont.load_default()
            self.ok = True
            print(f"[OLED] SSD1306 {self.w}x{self.h} @ 0x{addr:02X} aktif (I2C shared).")
        except Exception as e:
            print(f"[OLED] Gagal init ({e}) -> dilewati.")

    # ---- util ----
    def _text_w(self, text):
        try:
            return int(self.draw.textlength(text, font=self.font))
        except Exception:
            return len(text) * 6

    def _center(self, text, y):
        self.draw.text(((self.w - self._text_w(text)) // 2, y), text,
                       font=self.font, fill=255)

    def _blit(self):
        self.disp.image(self.image)
        self.disp.show()

    # ---- layar ----
    def _render_embedded_logo(self):
        """Render bitmap logo embedded (oled_logo.LOGO_BITMAP) meniru sketch
        Arduino: drawBitmap(..., SSD1306_BLACK, SSD1306_WHITE) -> goresan logo
        (bit '0') MENYALA, latar (bit '1'/0xff) GELAP. PIL 'frombytes("1")'
        memetakan bit '1' -> putih, jadi default kita INVERT agar cocok."""
        if oled_logo is None or not getattr(oled_logo, "LOGO_BITMAP", None):
            return False
        raw = Image.frombytes("1", (oled_logo.LOGO_W, oled_logo.LOGO_H),
                              bytes(oled_logo.LOGO_BITMAP)).convert("L")
        if not self.invert_logo:
            raw = ImageOps.invert(raw)   # tiru BLACK-on-WHITE Arduino
        if (raw.width, raw.height) != (self.w, self.h):
            raw = raw.resize((self.w, self.h))
        self.image.paste(raw.convert("1"), (0, 0))
        return True

    def show_boot(self, title="WASAKA", subtitle="HEXAPOD"):
        """Splash awal. Hanya merender logo dari oled_logo.py."""
        if not self.ok:
            return
        self.draw.rectangle((0, 0, self.w, self.h), outline=0, fill=0)
        drew = False

        # Hanya merender bitmap embedded dari oled_logo.py
        try:
            drew = self._render_embedded_logo()
        except Exception as e:
            print(f"[OLED] Gagal memuat logo dari oled_logo.py: {e}")
            drew = False

        if not drew:
            self._center("LOGO ERROR", 24)
        self._blit()

    def show_status(self, state, decision, vx, vy, wz, fps, blk,
                    imu_ok=False, heading_err=0.0, calib=(0, 0, 0, 0),
                    wall_front=False):
        """Panel status utama (di-throttle ke self.period)."""
        if not self.ok:
            return
        now = time.time()
        if now - self._last < self.period:
            return
        self._last = now

        d = self.draw
        d.rectangle((0, 0, self.w, self.h), outline=0, fill=0)

        # Header bar (state) - teks gelap di atas blok terang
        d.rectangle((0, 0, self.w - 1, 12), outline=255, fill=255)
        d.text((2, 2), str(state), font=self.font, fill=0)
        d.text((self.w - 40, 2), f"{fps:3.0f}FP", font=self.font, fill=0)

        # Aksi
        d.text((0, 15), f"Aksi:{ACTION_LABEL.get(decision, decision)}",
               font=self.font, fill=255)

        # Kecepatan
        d.text((0, 26), f"x{vx:+.1f} y{vy:+.1f} w{wz:+.2f}", font=self.font, fill=255)

        # Indikator halangan L / C / R (kotak terisi = blocked)
        d.text((0, 39), "OBS", font=self.font, fill=255)
        x0 = 28
        for i, b in enumerate(blk):
            bx = x0 + i * 24
            d.rectangle((bx, 38, bx + 16, 50), outline=255, fill=(255 if b else 0))
            d.text((bx + 5, 52), ["L", "C", "R"][i], font=self.font, fill=255)

        # Status kanan-bawah: WALL! / heading / open-loop
        if wall_front:
            d.text((96, 40), "WALL", font=self.font, fill=255)
        elif imu_ok:
            d.text((92, 40), f"{math.degrees(heading_err):+4.0f}", font=self.font, fill=255)
        else:
            d.text((92, 40), "OPN", font=self.font, fill=255)

        self._blit()

    def clear(self):
        if not self.ok:
            return
        try:
            self.disp.fill(0)
            self.disp.show()
        except Exception:
            pass


class OledWorker:
    """Simple background worker placeholder for OLED display.

    Provides a minimal `start()` / `stop()` API so application modules can
    import and control an OLED worker without requiring a complex
    implementation. The worker runs a lightweight thread (daemon) that
    idles until stopped. This is intentionally minimal: apps currently only
    call `OledWorker(oled).start()` and later `stop()`.
    """

    def __init__(self, oled: OledDisplay, interval: float = 0.2):
        self.oled = oled
        self.interval = float(interval)
        self._stop_event = __import__('threading').Event()
        self._thread = __import__('threading').Thread(target=self._run, daemon=True)

    def start(self):
        try:
            if not getattr(self.oled, "ok", False):
                return self
            if not self._thread.is_alive():
                self._thread.start()
        except Exception:
            pass
        return self

    def _run(self):
        while not self._stop_event.is_set():
            # Idle loop; concrete updates may be pushed directly to `oled`.
            try:
                self._stop_event.wait(self.interval)
            except Exception:
                break

    def stop(self):
        try:
            self._stop_event.set()
            if self._thread.is_alive():
                self._thread.join(timeout=0.5)
        except Exception:
            pass
