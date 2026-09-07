"""
WASAKA HEXAPOD - PERSON DETECTOR MODULE
========================================
Wrapper inferensi YOLOv8 ONNX untuk deteksi manusia (Person Following).

Menggunakan ONNX Runtime untuk inferensi model dan OpenCV DNN NMS untuk
penyaringan bounding box yang cepat tanpa memerlukan PyTorch pada runtime.

Mendukung model YOLOv8 resmi COCO (80 kelas, target class=0) maupun model
kustom 1-kelas (person).

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


class PersonDetector:
    """
    Detektor manusia berbasis YOLOv8 ONNX Runtime.
    """

    def __init__(self,
                 weights_path="models/yolov8n.onnx",
                 conf_thres=0.4,
                 nms_thres=0.45,
                 img_size=320,
                 device="auto"):
        if not HAS_ORT:
            raise ImportError("Library 'onnxruntime' belum terinstall. Jalankan: pip install onnxruntime")

        self.device = device.lower()
        self.conf_thres = conf_thres
        self.nms_thres = nms_thres
        self.img_size = img_size
        self.weights_path = weights_path
        self.target_class_id = 0  # 0 = 'person' pada dataset COCO
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
            project_root / "models" / "custom_person" / raw_path,
            project_root / "models" / "yolov8n.onnx",
            project_root / "models" / "custom_person" / "model.onnx",
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
            f"Pastikan file .onnx tersedia di folder 'models/'."
        )

    def _get_providers(self):
        available = set(ort.get_available_providers())
        if self.device == "cpu":
            return ["CPUExecutionProvider"]

        providers = []
        if "TensorrtExecutionProvider" in available:
            providers.append("TensorrtExecutionProvider")
        elif "CUDAExecutionProvider" in available:
            providers.append("CUDAExecutionProvider")

        providers.append("CPUExecutionProvider")
        return providers

    def _load_model(self):
        model_path = self._resolve_model_path()
        providers = self._get_providers()
        print(f"[INFO] Memuat PersonDetector: {model_path} (providers: {providers})")
        self.session = ort.InferenceSession(model_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name

    def detect(self, frame_bgr):
        """
        Deteksi manusia dalam frame BGR.

        Returns:
            list of dict: [{bbox: (x1, y1, x2, y2), conf, area, h, w, cx, cy}, ...]
        """
        if self.session is None or frame_bgr is None:
            return []

        h, w = frame_bgr.shape[:2]

        blob = cv2.dnn.blobFromImage(
            frame_bgr,
            scalefactor=1.0 / 255.0,
            size=(self.img_size, self.img_size),
            swapRB=True,
            crop=False,
        )

        outputs = self.session.run(None, {self.input_name: blob})
        return self._postprocess(outputs[0], frame_w=w, frame_h=h)

    def _postprocess(self, output, frame_w, frame_h):
        if output.ndim == 3:
            pred = output[0]
        elif output.ndim == 2:
            pred = output
        else:
            return []

        # YOLOv8 format: [4 + num_classes, num_anchors] -> transpose ke [num_anchors, 4 + num_classes]
        pred = np.transpose(pred)
        num_cols = pred.shape[1]
        num_classes = num_cols - 4

        boxes = []
        confidences = []
        scale_x = frame_w / float(self.img_size)
        scale_y = frame_h / float(self.img_size)

        for row in pred:
            x_center, y_center, w_box, h_box = row[:4]

            if num_classes == 1:
                conf = row[4]
            else:
                conf = row[4 + self.target_class_id]

            if conf < self.conf_thres:
                continue

            x1 = (x_center - w_box / 2.0) * scale_x
            y1 = (y_center - h_box / 2.0) * scale_y
            w_scaled = w_box * scale_x
            h_scaled = h_box * scale_y

            if w_scaled <= 0 or h_scaled <= 0:
                continue

            boxes.append([float(x1), float(y1), float(w_scaled), float(h_scaled)])
            confidences.append(float(conf))

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_thres, self.nms_thres)
        if len(indices) == 0:
            return []

        if isinstance(indices, np.ndarray):
            indices = indices.flatten()

        dets = []
        for idx in indices:
            x1, y1, w_box, h_box = boxes[idx]
            conf = confidences[idx]
            x2 = max(0.0, min(float(frame_w), x1 + w_box))
            y2 = max(0.0, min(float(frame_h), y1 + h_box))
            x1 = max(0.0, min(float(frame_w), x1))
            y1 = max(0.0, min(float(frame_h), y1))

            dets.append({
                "bbox": (float(x1), float(y1), float(x2), float(y2)),
                "conf": float(conf),
                "area": float((x2 - x1) * (y2 - y1)),
                "h": float(y2 - y1),
                "w": float(x2 - x1),
                "cx": float(x1 + (x2 - x1) / 2.0),
                "cy": float(y1 + (y2 - y1) / 2.0),
            })

        return dets
