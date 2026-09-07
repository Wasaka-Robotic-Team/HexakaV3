"""
WASAKA HEXAPOD - AUTONOMOUS PERCEPTION PIPELINE
===============================================
Modul pemrosesan citra multi-layer untuk navigasi otonom:
1. Structural Layer : Preprocessing CLAHE + Canny Edge Density (L, C, R).
2. Temporal Layer   : Optical Flow (Farneback dense flow) untuk motion parallax proximity cue.
3. Appearance Layer : HSV Ground Segmentation (histogram backprojection).
4. Geometry Layer   : Free-space column profile untuk steering halus di celah sempit.
5. Fusi & Keputusan : Penggabungan multi-sensor cue untuk menentukan aksi navigasi.
6. Session Logging  : Pencatatan metrik performa ke file CSV untuk analisis pasca-misi.

AUTHOR: Wasaka Robotic Team
"""

import os
import csv
import time
from datetime import datetime
import cv2
import numpy as np

# Parameter default
FLOW_PROC_WIDTH    = 160
FLOW_REJECT        = 0.8
FLOW_CONFIRM       = 2.2
FLOW_EMA           = 0.5

GROUND_REF_X       = (0.35, 0.65)
GROUND_REF_Y       = (0.82, 1.00)
GROUND_HIST_BINS   = (30, 32)
GROUND_BACKPROJ_TH = 40
GROUND_BLOCK_FRAC  = 0.35
GROUND_WALL_FRAC   = 0.60
GROUND_UPDATE_EMA  = 0.05
GROUND_OPEN_KSIZE  = 5

FREESPACE_SMOOTH   = 9

OBSTACLE_THRESHOLD = 15.0
IGNORE_THRESHOLD   = 5.0
HYSTERESIS_MARGIN  = 3.0

COLOR_GREEN  = (0, 255, 0)
COLOR_RED    = (0, 0, 255)
COLOR_YELLOW = (0, 255, 255)
COLOR_CYAN   = (255, 255, 0)


def preprocess_frame(frame, width=320, height=240, canny_low=50, canny_high=150, blur_k=(5, 5), morph_k=3):
    """
    Resize, grayscale, CLAHE, Gaussian blur, Canny edge detection, dan dilasi morfologi.
    """
    resized = cv2.resize(frame, (width, height))
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)

    blurred = cv2.GaussianBlur(enhanced, blur_k, 0)
    edges = cv2.Canny(blurred, canny_low, canny_high)

    morph_kernel = np.ones((morph_k, morph_k), np.uint8)
    edges = cv2.dilate(edges, morph_kernel, iterations=1)

    return resized, enhanced, edges


def get_roi(img, top_ratio=0.5):
    """Ambil Region of Interest (ROI) pada bagian bawah citra."""
    h = img.shape[0]
    roi_top = int(h * top_ratio)
    return img[roi_top:h, :], roi_top


def calculate_density(region):
    """Hitung densitas piksel edge (persentase piksel putih 0-100%)."""
    if region is None or region.size == 0:
        return 0.0
    return (np.count_nonzero(region) / float(region.size)) * 100.0


class FlowEstimator:
    """Estimasi magnitudo optical flow Farneback per region (kiri, tengah, kanan)."""

    def __init__(self, proc_width=FLOW_PROC_WIDTH, ema_factor=FLOW_EMA):
        self.proc_width = proc_width
        self.ema_factor = ema_factor
        self.prev = None
        self.ema = [0.0, 0.0, 0.0]

    def compute(self, gray_roi):
        h, w = gray_roi.shape[:2]
        if w > self.proc_width:
            scale = self.proc_width / float(w)
            small = cv2.resize(gray_roi, (self.proc_width, max(1, int(h * scale))))
        else:
            small = gray_roi

        if self.prev is None or self.prev.shape != small.shape:
            self.prev = small
            return None

        flow = cv2.calcOpticalFlowFarneback(
            self.prev, small, None,
            pyr_scale=0.5, levels=2, winsize=15,
            iterations=2, poly_n=5, poly_sigma=1.2, flags=0
        )
        self.prev = small

        mag = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
        tw = mag.shape[1] // 3
        raw = [
            float(np.median(mag[:, 0:tw])),
            float(np.median(mag[:, tw:2 * tw])),
            float(np.median(mag[:, 2 * tw:])),
        ]
        for i in range(3):
            self.ema[i] = self.ema_factor * raw[i] + (1.0 - self.ema_factor) * self.ema[i]
        return tuple(self.ema)


class AdaptiveThreshold:
    """Mengadaptasi ambang batas edge berdasarkan nilai dasar tekstur lantai."""

    def __init__(self, base_obstacle=OBSTACLE_THRESHOLD, base_ignore=IGNORE_THRESHOLD):
        self.base_obstacle = base_obstacle
        self.base_ignore = base_ignore
        self.baseline = 0.0
        self.init = False

    def update_and_get(self, dL, dC, dR):
        floor = min(dL, dC, dR)
        if not self.init:
            self.baseline = floor
            self.init = True
        else:
            self.baseline = 0.02 * floor + 0.98 * self.baseline
        obstacle = max(self.base_obstacle, self.baseline + 10.0)
        ignore = max(self.base_ignore, self.baseline + 2.0)
        return obstacle, ignore


class GroundSegmenter:
    """Segmentasi lantai berbasis warna HSV dan backprojection histogram."""

    def __init__(self, open_ksize=GROUND_OPEN_KSIZE):
        self.hist = None
        self.ready = False
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_ksize, open_ksize))

    def _ref_rect(self, w, h):
        x0 = int(GROUND_REF_X[0] * w); x1 = int(GROUND_REF_X[1] * w)
        y0 = int(GROUND_REF_Y[0] * h); y1 = int(GROUND_REF_Y[1] * h)
        return x0, y0, x1, y1

    def _calc_hist(self, hsv_patch):
        hist = cv2.calcHist([hsv_patch], [0, 1], None, list(GROUND_HIST_BINS), [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
        return hist

    def calibrate(self, bgr):
        h, w = bgr.shape[:2]
        x0, y0, x1, y1 = self._ref_rect(w, h)
        patch = bgr[y0:y1, x0:x1]
        if patch.size == 0:
            return
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        self.hist = self._calc_hist(hsv)
        self.ready = True

    def process(self, bgr, roi_top):
        if not self.ready:
            return None, (None, None, None), False

        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        prob = cv2.calcBackProject([hsv], [0, 1], self.hist, [0, 180, 0, 256], 1)
        floor = prob >= GROUND_BACKPROJ_TH
        nonfloor = np.where(floor, 0, 255).astype(np.uint8)
        nonfloor = cv2.morphologyEx(nonfloor, cv2.MORPH_OPEN, self.kernel)
        nonfloor = cv2.morphologyEx(nonfloor, cv2.MORPH_CLOSE, self.kernel)

        def _frac(a):
            return float(np.count_nonzero(a)) / max(1, a.size)

        roi = nonfloor[roi_top:h, :]
        rw = roi.shape[1]; tw = rw // 3
        fL = _frac(roi[:, 0:tw])
        fC = _frac(roi[:, tw:2 * tw])
        fR = _frac(roi[:, 2 * tw:])

        x0, y0, x1, y1 = self._ref_rect(w, h)
        ref_frac = _frac(nonfloor[y0:y1, x0:x1])
        wall_front = ref_frac >= GROUND_WALL_FRAC

        if GROUND_UPDATE_EMA > 0.0 and ref_frac < 0.15:
            new_hist = self._calc_hist(hsv[y0:y1, x0:x1])
            self.hist = (1.0 - GROUND_UPDATE_EMA) * self.hist + GROUND_UPDATE_EMA * new_hist
            cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX)

        return nonfloor, (fL, fC, fR), wall_front


class FreeSpaceProfile:
    """Profil ketinggian ruang kosong per kolom citra untuk panduan kemudi."""

    def __init__(self, smooth_k=FREESPACE_SMOOTH):
        self.smooth_k = smooth_k
        self.boundary = None

    def compute(self, roi_mask):
        h, w = roi_mask.shape[:2]
        obst = roi_mask > 0
        flipped = obst[::-1, :]
        has = flipped.any(axis=0)
        first = np.argmax(flipped, axis=0)
        free_h = np.where(has, first, h).astype(np.float32)

        k = max(1, self.smooth_k | 1)
        if k > 1 and w >= k:
            free_h = np.convolve(free_h, np.ones(k, np.float32) / k, mode="same")

        self.boundary = (h - free_h).astype(np.int32)

        frac = free_h / float(h)
        tw = w // 3
        freeL = float(np.mean(frac[0:tw]))
        freeC = float(np.mean(frac[tw:2 * tw]))
        freeR = float(np.mean(frac[2 * tw:]))
        return (freeL, freeC, freeR), self.boundary


def fuse_blocked(edge_density, flow_mag, ground_frac,
                 obstacle_th, ignore_th, moving_forward,
                 use_optical_flow=True, use_ground_seg=True):
    """Fusi sinyal halangan dari edge, optical flow, dan ground segmentation."""
    blocked = edge_density >= obstacle_th

    if use_optical_flow and flow_mag is not None and moving_forward:
        if blocked and flow_mag < FLOW_REJECT:
            blocked = False
        if (not blocked) and flow_mag > FLOW_CONFIRM and edge_density >= ignore_th:
            blocked = True

    if use_ground_seg and ground_frac is not None and ground_frac >= GROUND_BLOCK_FRAC:
        blocked = True

    return blocked


_prev_decision = "FORWARD"

def decide_navigation(dL, dC, dR, blk_L, blk_C, blk_R, ignore_th,
                      sL=None, sC=None, sR=None):
    """
    Menentukan keputusan arah navigasi (FORWARD, LEFT, RIGHT, IGNORE).
    """
    global _prev_decision

    if sL is None:
        sL, sC, sR = dL, dC, dR

    if (dL < ignore_th and dC < ignore_th and dR < ignore_th
            and not (blk_L or blk_C or blk_R)):
        _prev_decision = "IGNORE"
        return "IGNORE", COLOR_CYAN

    if blk_C:
        if blk_L and blk_R:
            decision = "LEFT" if sL <= sR else "RIGHT"
        elif blk_L:
            decision = "RIGHT"
        elif blk_R:
            decision = "LEFT"
        else:
            if _prev_decision == "LEFT":
                decision = "RIGHT" if sR < sL - HYSTERESIS_MARGIN else "LEFT"
            elif _prev_decision == "RIGHT":
                decision = "LEFT" if sL < sR - HYSTERESIS_MARGIN else "RIGHT"
            else:
                decision = "LEFT" if sL <= sR else "RIGHT"
    elif blk_L and blk_R:
        decision = "FORWARD"
    elif blk_L:
        decision = "RIGHT"
    elif blk_R:
        decision = "LEFT"
    else:
        decision = "FORWARD"

    _prev_decision = decision
    color = COLOR_YELLOW if decision in ("LEFT", "RIGHT") else COLOR_GREEN
    return decision, color


def lerp(current, target, factor):
    """Interpolasi linear kecepatan agar transisi gerak halus."""
    return current + (target - current) * factor


class SessionLogger:
    """Pencatat telemetri sesi berjalan ke format CSV."""

    def __init__(self, enabled=True, log_dir="logs"):
        self.enabled = enabled
        self.file = None
        self.writer = None
        if not enabled:
            return

        try:
            os.makedirs(log_dir, exist_ok=True)
            filename = f"session_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
            filepath = os.path.join(log_dir, filename)
            self.file = open(filepath, mode="w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.file)
            self.writer.writerow([
                "timestamp", "state", "decision",
                "density_L", "density_C", "density_R",
                "flow_mag", "vx", "vy", "wz",
                "heading_error", "roll", "pitch", "fps"
            ])
            print(f"[LOG] Sesi dicatat ke: {filepath}")
        except Exception as e:
            print(f"[WARN] Gagal inisialisasi SessionLogger ({e}).")
            self.enabled = False

    def log(self, state, decision, dL, dC, dR, flow_mag, vx, vy, wz, heading_error, roll, pitch, fps):
        if not self.enabled or self.writer is None:
            return
        try:
            self.writer.writerow([
                f"{time.time():.3f}", state, decision,
                f"{dL:.2f}", f"{dC:.2f}", f"{dR:.2f}",
                f"{flow_mag:.2f}" if flow_mag is not None else "None",
                f"{vx:.3f}", f"{vy:.3f}", f"{wz:.3f}",
                f"{heading_error:.3f}", f"{roll:.2f}", f"{pitch:.2f}",
                f"{fps:.1f}"
            ])
        except Exception:
            pass

    def close(self):
        if self.file is not None:
            try:
                self.file.close()
            except Exception:
                pass
            self.file = None
