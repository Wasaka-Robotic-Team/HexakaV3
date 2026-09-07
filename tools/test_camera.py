"""
WASAKA HEXAPOD - CAMERA & PERCEPTION TEST HARNESS
=================================================
Alat uji kamera dan persepsi visual komputer (tanpa menggerakkan servo/IMU).
Sangat aman dan praktis dijalankan di laptop maupun Raspberry Pi / Jetson:
- Menguji ketersediaan kamera pada berbagai indeks (0-5)
- Menampilkan visualisasi feed kamera, edge detection (Canny), dan deteksi AI
- Mengukur framerate (FPS) live

Penggunaan:
    python tools/test_camera.py
    python tools/test_camera.py --cam 0
    python tools/test_camera.py --mode person

KONTROL:
  'q' : Keluar
  'c' : Rekalibrasi warna lantai (ground-seg)

AUTHOR: Wasaka Robotic Team
"""

import argparse
import sys
import time
from pathlib import Path
import cv2

# Setup path ke root proyek
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision.camera import open_camera, CameraStream
from apps.autonomous.perception import preprocess_frame, get_roi, calculate_density, GroundSegmenter


def main():
    parser = argparse.ArgumentParser(description="Test kamera dan subsistem visi Wasaka Hexapod.")
    parser.add_argument("--cam", type=int, default=None, help="Indeks kamera (contoh: 0, 1)")
    parser.add_argument("--mode", type=str, default="obstacle", choices=["obstacle", "person"],
                        help="Mode uji: 'obstacle' (Canny edge + ground-seg) atau 'person' (YOLOv8)")
    parser.add_argument("--width", type=int, default=640, help="Lebar frame")
    parser.add_argument("--height", type=int, default=480, help="Tinggi frame")
    args = parser.parse_args()

    print("=" * 60)
    print("  WASAKA HEXAPOD - CAMERA TEST HARNESS")
    print("=" * 60)

    cap, idx_used = open_camera(source=args.cam, width=args.width, height=args.height)
    if cap is None:
        print("[ERROR] Kamera tidak dapat dibuka.")
        return

    camera = CameraStream(cap, threaded=True).start()
    ground_seg = GroundSegmenter()

    person_detector = None
    if args.mode == "person":
        try:
            from vision.person_detector import PersonDetector
            person_detector = PersonDetector(weights_path="models/yolov8n.onnx")
            print("[INFO] PersonDetector YOLOv8 berhasil dimuat.")
        except Exception as e:
            print(f"[WARN] Gagal memuat PersonDetector ({e}).")

    prev_t = time.time()
    fps = 0.0

    print("[INFO] Kamera aktif. Tekan 'q' untuk keluar, 'c' untuk rekalibrasi ground.")

    try:
        while True:
            ret, frame, _ = camera.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            now = time.time()
            dt = max(1e-4, now - prev_t)
            prev_t = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

            display = frame.copy()

            if args.mode == "obstacle":
                resized, enhanced, edges = preprocess_frame(frame, 320, 240)
                roi_edges, roi_top = get_roi(edges, 0.5)
                tw = roi_edges.shape[1] // 3

                dL = calculate_density(roi_edges[:, :tw])
                dC = calculate_density(roi_edges[:, tw:2 * tw])
                dR = calculate_density(roi_edges[:, 2 * tw:])

                # Ground segmentation
                nonfloor, _, _ = ground_seg.process(resized, roi_top)

                cv2.putText(display, f"FPS: {fps:.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(display, f"Densitas  L:{dL:.1f}%  C:{dC:.1f}%  R:{dR:.1f}%", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                # Thumbnail edge mask di pojok kanan atas
                edge_thumb = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                edge_thumb = cv2.resize(edge_thumb, (160, 120))
                display[10:130, display.shape[1] - 170:display.shape[1] - 10] = edge_thumb

            elif args.mode == "person" and person_detector:
                dets = person_detector.detect(frame)
                for d in dets:
                    x1, y1, x2, y2 = map(int, d["bbox"])
                    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(display, f"Person {d['conf']:.2f}", (x1, max(15, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(display, f"FPS: {fps:.1f} | Detections: {len(dets)}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("Wasaka Camera Test", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c') and args.mode == "obstacle":
                ground_seg.calibrate(resized)
                print("[INFO] Ground color rekalibrasi selesai.")

    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("[INFO] Kamera ditutup.")


if __name__ == "__main__":
    main()
