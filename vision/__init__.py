"""
WASAKA HEXAPOD - VISION MODULE
==============================
Modul persepsi komputer dan deteksi berbasis AI:
- camera: Threaded CameraStream dengan multi-index fallback otomatis.
- tracker: TargetTracker dengan hysteresis anti-flicker untuk pelacakan target tunggal.
- person_detector: Detektor YOLOv8 untuk manusia (Person Following).
- object_detector: Detektor YOLO11 untuk klasifikasi rintangan otonom.
"""

from .camera import CameraStream
from .tracker import TargetTracker

__all__ = [
    "CameraStream",
    "TargetTracker",
]
