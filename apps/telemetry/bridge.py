"""
WASAKA HEXAPOD - DIGITAL TWIN BRIDGE (LAPTOP SIDE)
===================================================
Menjalankan jembatan UDP -> WebSocket di laptop pameran:
1. Menerima paket UDP dari robot (port 5005).
2. Menyiarkan (broadcast) data ke browser via WebSocket (port 8765).
3. Browser membuka `dashboard.html` untuk visualisasi 3D secara langsung.

Penggunaan:
    python -m apps.telemetry.bridge
    lalu buka apps/telemetry/dashboard.html di browser.

AUTHOR: Wasaka Robotic Team
"""

import asyncio
import json
import socket
import sys
import threading
import time

try:
    import websockets
except ImportError:
    print("[ERROR] websockets belum terinstall. Jalankan: pip install websockets")
    sys.exit(1)

UDP_PORT = 5005
WS_PORT  = 8765

_latest = None
_latest_lock = threading.Lock()
_clients = set()
_rx_count = 0
_last_rpi_ip = "—"


def udp_receiver_thread():
    """Thread penerima paket data UDP dari robot."""
    global _latest, _rx_count, _last_rpi_ip
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", UDP_PORT))
        print(f"[BRIDGE] UDP receiver mendengarkan di 0.0.0.0:{UDP_PORT}")
    except OSError as e:
        print(f"[BRIDGE] Gagal bind UDP port {UDP_PORT}: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            packet = json.loads(data.decode("utf-8"))
            with _latest_lock:
                _latest = packet
                _rx_count += 1
                _last_rpi_ip = addr[0]
        except Exception:
            pass


async def ws_handler(websocket):
    """Handler koneksi WebSocket browser dashboard."""
    _clients.add(websocket)
    print(f"[BRIDGE] Browser terhubung dari {websocket.remote_address} (Total client: {len(_clients)})")
    try:
        while True:
            await asyncio.sleep(0.02)  # ~50 Hz update ke browser
            with _latest_lock:
                pkt = _latest

            if pkt is not None:
                await websocket.send(json.dumps(pkt))
    except (websockets.exceptions.ConnectionClosed, asyncio.CancelledError):
        pass
    finally:
        _clients.discard(websocket)
        print(f"[BRIDGE] Browser disconnect (Sisa client: {len(_clients)})")


def monitor_thread():
    """Thread monitoring status di terminal laptop."""
    last_cnt = 0
    while True:
        time.sleep(2.0)
        rate = (_rx_count - last_cnt) / 2.0
        last_cnt = _rx_count
        with _latest_lock:
            pkt = _latest

        if pkt:
            r = pkt.get("roll", 0.0)
            p = pkt.get("pitch", 0.0)
            y = pkt.get("yaw_deg", 0.0)
            print(f"[STATUS] Dari {_last_rpi_ip} | {rate:.1f} Hz | R={r:+5.1f} P={p:+5.1f} Y={y:+5.1f} | Clients={len(_clients)}")
        else:
            print(f"[STATUS] Menunggu paket dari robot di port {UDP_PORT}... (Clients: {len(_clients)})")


async def main_async():
    # Jalankan thread UDP receiver
    t_udp = threading.Thread(target=udp_receiver_thread, daemon=True)
    t_udp.start()

    # Jalankan thread monitor terminal
    t_mon = threading.Thread(target=monitor_thread, daemon=True)
    t_mon.start()

    print(f"[BRIDGE] Memulai WebSocket server di ws://0.0.0.0:{WS_PORT}")
    print(f"[BRIDGE] Buka 'apps/telemetry/dashboard.html' di browser Anda.")
    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
        await asyncio.Future()  # Jalankan selamanya


def run_bridge():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Berhenti.")


if __name__ == "__main__":
    run_bridge()
