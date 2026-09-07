"""
WASAKA HEXAPOD - UNIFIED CLI LAUNCHER
======================================
Pusat eksekusi terpadu seluruh mode operasi dan alat uji robot hexapod Wasaka.

Penggunaan:
    python main.py --mode autonomous   # Navigasi otonom & obstacle avoidance (BEST 2026)
    python main.py --mode follow       # Interaktif mengikuti orang (Bazar PKKMB)
    python main.py --mode teleop       # Remote manual Gamepad Xbox via Pygame
    python main.py --mode telemetry    # Digital Twin WebSocket bridge server
    python main.py --mode test-cam     # Uji kamera & pipeline visi komputer
    python main.py --mode test-gui     # GUI manual slider uji servo & IMU
    python main.py --mode test-level   # GUI kalibrasi & tuning gain PID leveling

AUTHOR: Wasaka Robotic Team
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Wasaka Hexapod Unified Platform",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Pilihan Mode:
  autonomous   : Navigasi otonom kompetisi BEST (Canny + Flow + GroundSeg + PID Leveling)
  follow       : Mode interaksi mengikuti orang (YOLOv8 + Hysteresis Tracking + Animations)
  teleop       : Kendali manual remote Gamepad Xbox (Pygame)
  telemetry    : Digital Twin bridge server (UDP -> WebSocket untuk browser 3D)
  test-cam     : Uji kamera & filter visi tanpa servo
  test-gui     : GUI slider Tkinter untuk uji gerak servo & orientasi IMU
  test-level   : GUI tuning parameter PID closed-loop body leveling
        """
    )

    parser.add_argument(
        "-m", "--mode",
        type=str,
        default="autonomous",
        choices=[
            "autonomous", "auto",
            "follow", "person",
            "teleop", "gamepad",
            "telemetry", "bridge", "twin",
            "test-cam",
            "test-gui", "test-move",
            "test-level"
        ],
        help="Mode operasi atau modul uji yang ingin dijalankan (default: autonomous)"
    )

    args = parser.parse_args()
    mode = args.mode.lower()

    if mode in ("autonomous", "auto"):
        from apps.autonomous.main import run_autonomous
        run_autonomous()

    elif mode in ("follow", "person"):
        from apps.person_follow.main import run_person_follow
        run_person_follow()

    elif mode in ("teleop", "gamepad"):
        from apps.teleop.gamepad import run_teleop
        run_teleop()

    elif mode in ("telemetry", "bridge", "twin"):
        from apps.telemetry.bridge import run_bridge
        run_bridge()

    elif mode == "test-cam":
        from tools.test_camera import main as run_test_cam
        run_test_cam()

    elif mode in ("test-gui", "test-move"):
        from tools.test_movement_imu import main as run_test_gui
        run_test_gui()

    elif mode == "test-level":
        from tools.test_leveling_pid import main as run_test_level
        run_test_level()

    else:
        print(f"[ERROR] Mode tidak dikenal: {mode}")
        parser.print_help()


if __name__ == "__main__":
    main()
