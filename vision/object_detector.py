"""
WASAKA HEXAPOD - OBJECT DETECTOR MODULE (YOLO11)
=================================================
Wrapper deteksi rintangan berbasis YOLO11n ONNX Runtime.

Dioptimasi untuk performa tinggi pada SBC (Raspberry Pi 4 / Jetson Nano)
serta PC:
- Preprocessing cepat (resize INTER_NEAREST, normalize contiguous)
- Post-processing vectorized (menghindari loop iterasi 8400 box)
- Identifikasi posisi rintangan (KIRI, TENGAH, KANAN, Aman)
- Evaluasi ketinggian bbox terhadap batas bawah frame (apakah objek bisa
  dilewati dengan PID leveling atau harus dihindari).

AUTHOR: Wasaka Robotic Team
"""

import os
from pathlib import Path
import cv2
import numpy as np

try:
    import onnxruntime as ort
    HAS_ORT = True
except ImportError:
    ort = None
    HAS_ORT = False


class ObjectDetector:
    """
    Detektor rintangan berbasis model YOLO11n ONNX.
    """

    def __init__(self,
                 weights_path="models/yolo11n.onnx",
                 conf_thresh=0.45,
                 area_threshold=80000,
                 input_size=320,
                 num_threads=4):
        if not HAS_ORT:
            raise ImportError("Library 'onnxruntime' belum terinstall. Jalankan: pip install onnxruntime")

        self.weights_path = weights_path
        self.conf_thresh = conf_thresh
        self.area_threshold = area_threshold
        self.input_size = input_size
        self.num_threads = num_threads
        self.session = None
        self.input_name = None
        self._load_model()

    def _resolve_model_path(self):
        project_root = Path(__file__).resolve().parent.parent
        raw_path = self.weights_path.strip()

        candidates = [
            Path(raw_path),
            project_root / raw_path,
            project_root / "models" / raw_path,
            project_root / "models" / "yolo11n.onnx",
        ]

        if raw_path.endswith(".pt"):
            base = raw_path[:-3]
            candidates.insert(1, project_root / f"{base}.onnx")
            candidates.insert(2, project_root / "models" / f"{Path(base).name}.onnx")

        seen = set()
        for c in candidates:
            try:
                resolved = c.resolve(strict=False)
            except Exception:
                continue
            if resolved in seen:
                continue
            seen.add(resolved)
            if resolved.exists() and resolved.is_file():
                return str(resolved)

        raise FileNotFoundError(
            f"File model tidak ditemukan: {self.weights_path}.\n"
            f"Pastikan file yolo11n.onnx tersedia di folder 'models/'."
        )

    def _load_model(self):
        model_path = self._resolve_model_path()
        print(f"[INFO] Memuat ObjectDetector: {model_path}")

        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        opts.intra_op_num_threads = self.num_threads
        opts.inter_op_num_threads = 1
        opts.enable_cpu_mem_arena = True
        opts.enable_mem_pattern = True
        opts.enable_profiling = False

        providers = ort.get_available_providers()
        selected_providers = []
        if "CUDAExecutionProvider" in providers:
            selected_providers.append("CUDAExecutionProvider")
        selected_providers.append("CPUExecutionProvider")

        self.session = ort.InferenceSession(model_path, sess_options=opts, providers=selected_providers)
        self.input_name = self.session.get_inputs()[0].name

        # Warmup dummy inference
        dummy = np.zeros((1, 3, self.input_size, self.input_size), dtype=np.float32)
        self.session.run(None, {self.input_name: dummy})

    def detect(self, frame_bgr):
        """
        Menjalankan inferensi pada frame BGR.

        Returns:
            tuple: (detections_list, obs_location_str)
            detections_list: [(x, y, w, h, score, is_danger, is_touch_bottom), ...]
            obs_location_str: "Aman", "KIRI", "TENGAH", atau "KANAN"
        """
        if self.session is None or frame_bgr is None:
            return [], "Aman"

        frame_h, frame_w = frame_bgr.shape[:2]

        # Preprocess
        resized = cv2.resize(frame_bgr, (self.input_size, self.input_size), interpolation=cv2.INTER_NEAREST)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        blob = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32) * (1.0 / 255.0)
        blob = blob[np.newaxis]

        # Inferensi
        outputs = self.session.run(None, {self.input_name: blob})
        output = outputs[0]

        preds = output[0].squeeze().T  # [num_anchors, 4 + classes]
        scores_all = preds[:, 4:]
        max_scores = scores_all.max(axis=1)

        mask = max_scores > self.conf_thresh
        if not mask.any():
            return [], "Aman"

        preds = preds[mask]
        scores = max_scores[mask]

        cx, cy, w, h = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
        areas = w * h

        sx = frame_w / float(self.input_size)
        sy = frame_h / float(self.input_size)

        x1 = ((cx - w * 0.5) * sx).astype(np.int32)
        y1 = ((cy - h * 0.5) * sy).astype(np.int32)
        bw = (w * sx).astype(np.int32)
        bh = (h * sy).astype(np.int32)

        # Evaluasi letak rintangan bahaya
        obs_loc = "Aman"
        danger_mask = areas > self.area_threshold
        if danger_mask.any():
            danger_areas = areas[danger_mask]
            danger_cx = cx[danger_mask]
            idx_max = np.argmax(danger_areas)
            cx_val = danger_cx[idx_max]
            third = self.input_size / 3.0
            if cx_val < third:
                obs_loc = "KIRI"
            elif cx_val > third * 2.0:
                obs_loc = "KANAN"
            else:
                obs_loc = "TENGAH"

        # Cek apakah bbox menyentuh batas bawah frame kamera
        bottom_threshold_y = frame_h * 0.88

        detections = []
        for i in range(len(scores)):
            y_bottom = y1[i] + bh[i]
            touches_bottom = y_bottom >= bottom_threshold_y
            is_danger = bool(danger_mask[i])

            detections.append({
                "bbox": (int(x1[i]), int(y1[i]), int(x1[i] + bw[i]), int(y1[i] + bh[i])),
                "score": float(scores[i]),
                "danger": is_danger,
                "touches_bottom": touches_bottom,
                "cx": float(x1[i] + bw[i] / 2.0),
                "cy": float(y1[i] + bh[i] / 2.0),
                "area": float(bw[i] * bh[i]),
            })

        return detections, obs_loc
