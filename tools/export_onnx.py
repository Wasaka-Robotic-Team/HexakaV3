"""
WASAKA HEXAPOD - MODEL EXPORT UTILITY (.pt -> .onnx)
=====================================================
Utilitas untuk mengekspor bobot model PyTorch YOLO (.pt) ke format ONNX
dengan ukuran input resolusi yang dioptimalkan untuk SBC (320x320 / 640x640).

Penggunaan:
    python tools/export_onnx.py --model models/yolov8n.pt --imgsz 320
    python tools/export_onnx.py --model models/yolo11n.pt --imgsz 320

AUTHOR: Wasaka Robotic Team
"""

import argparse
import os
import shutil
import sys
from pathlib import Path


def export_model(model_path, imgsz=320, opset=12):
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] Library 'ultralytics' belum terinstall.")
        print("Install dengan perintah: pip install ultralytics")
        return False

    path_obj = Path(model_path)
    if not path_obj.exists():
        print(f"[ERROR] File bobot model tidak ditemukan: {model_path}")
        return False

    print(f"[INFO] Memuat model {model_path}...")
    model = YOLO(str(path_obj))

    print(f"[INFO] Mengekspor ke ONNX (imgsz={imgsz}, opset={opset})...")
    try:
        exported_path = model.export(format="onnx", imgsz=imgsz, opset=opset)
        print(f"[INFO] Ekspor sukses! Tersimpan di: {exported_path}")

        # Salin ke folder models/
        dest_dir = Path("models")
        dest_dir.mkdir(exist_ok=True)
        dest_file = dest_dir / Path(exported_path).name
        shutil.copy(exported_path, dest_file)
        print(f"[INFO] Model disalin ke: {dest_file}")
        return True
    except Exception as e:
        print(f"[ERROR] Gagal melakukan ekspor: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Ekspor model YOLO PyTorch (.pt) ke ONNX.")
    parser.add_argument("--model", type=str, default="models/yolov8n.pt", help="Path ke file .pt")
    parser.add_argument("--imgsz", type=int, default=320, help="Resolusi input model (default 320)")
    parser.add_argument("--opset", type=int, default=12, help="ONNX opset version (default 12)")
    args = parser.parse_args()

    export_model(args.model, args.imgsz, args.opset)


if __name__ == "__main__":
    main()
