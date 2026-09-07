# 🤖 WASAKA HEXAPOD - Unified Robotics Platform

Repositori resmi **Tim Robotika Wasaka** yang menggabungkan seluruh fungsionalitas robot hexapod 18-DOF ke dalam satu arsitektur modular, terstandarisasi, dan bersih (*clean code*).

Basis kode ini menyatukan dua misi utama yang sebelumnya terpisah:
1. **Autonomous Navigation (Kompetisi BEST 2026)**: Navigasi otonom dengan *multi-layer vision perception*, stabilisasi bodi tertutup (*closed-loop PID body leveling*), penghindaran rintangan, dan Digital Twin 3D.
2. **Person Following & Teleoperation (Demo Pameran PKKMB)**: Pelacakan manusia (*human tracking*) cerdas dengan *hysteresis anti-flicker*, animasi interaktif (sapaan bungkuk, napas, lambaian kaki), dan kendali jarak jauh manual stik Xbox.

---

## 📁 Struktur Arsitektur Modular

```
Code/
├── core/                       # [FONDASI HARDWARE & KINEMATIKA]
│   ├── locomotion.py           # Driver Dynamixel AX-12A, 3-DOF IK, Tripod Gait generator
│   ├── oled_display.py         # Driver display SSD1306 OLED 128x64 I2C & threaded worker
│   ├── oled_logo.py            # Bitmap aset logo untuk layar OLED
│   ├── imu_sensor.py           # Driver IMU 9-DOF BNO055, heading-lock PID & safety tilt
│   ├── pid_controller.py       # Kontroler PID serbaguna (anti-windup & derivative filter)
│   └── body_leveler.py         # Closed-loop horizontal stabilization (PID body leveling)
│
├── vision/                     # [PERSEPSI VISUAL & AI INFERENCE]
│   ├── camera.py               # Robust threaded camera stream dengan multi-index fallback (0-5)
│   ├── person_detector.py      # Wrapper detektor manusia YOLOv8 ONNX (OpenCV NMS)
│   ├── object_detector.py      # Wrapper detektor rintangan YOLO11 ONNX (vectorized inference)
│   └── tracker.py              # Pelacak target tunggal berbasis kontinuitas & histeresis
│
├── apps/                       # [APLIKASI / MODE MISI UTAMA]
│   ├── autonomous/             # Mode Otonom Kompetisi BEST
│   │   ├── main.py             # Navigasi otonom terintegrasi + OpenCV dashboard
│   │   ├── perception.py       # Multi-layer perception: Edge, Optical Flow, Ground-Seg, FreeSpace
│   │   └── fsm.py              # Finite State Machine (CRUISE, AVOID, BACKUP, SCAN, FAULT)
│   │
│   ├── person_follow/          # Mode Interaktif Mengikuti Orang (Bazar PKKMB)
│   │   ├── main.py             # Loop utama person following + animasi
│   │   ├── controller.py       # FollowController (PID heading + distance regulator)
│   │   └── animations.py       # Animasi robot (salam membungkuk, bernapas, lambaian kaki)
│   │
│   ├── teleop/                 # Mode Kendali Jarak Jauh Manual
│   │   └── gamepad.py          # Kontrol stik Xbox via Pygame + auto-toggle person follow
│   │
│   └── telemetry/              # Mode Digital Twin 3D
│       ├── imu_sender.py       # Pengirim data UDP non-blocking dari robot (~50 Hz)
│       ├── bridge.py           # Jembatan UDP -> WebSocket di laptop
│       └── dashboard.html      # UI visualisasi 3D orientasi robot di browser
│
├── tools/                      # [ALAT UJI & KALIBRASI]
│   ├── test_camera.py          # Uji kamera & pipeline visi tanpa servo (aman di laptop)
│   ├── test_movement_imu.py    # GUI Tkinter manual slider uji gerak servo & IMU
│   ├── test_leveling_pid.py    # GUI Tkinter tuning parameter PID body leveling
│   └── export_onnx.py          # Utilitas ekspor model PyTorch (.pt) ke ONNX
│
├── models/                     # Bobot Model Jaringan Saraf Tiruan
│   ├── yolo11n.onnx            # Model deteksi rintangan
│   ├── yolov8n.onnx            # Model deteksi manusia COCO
│   └── custom_person/          # Model kustom 1-kelas khusus manusia
│
├── main.py                     # Unified CLI Launcher (Pusat kendali seluruh mode)
├── requirements.txt            # Daftar dependensi Python
└── README.md                   # Dokumentasi sistem terpadu
```

---

## ⚡ Quick Start

### 1. Instalasi Dependensi

#### Di PC / Laptop (Mode Uji & Simulasi):
```bash
pip install opencv-python numpy onnxruntime pygame websockets
```

#### Di Robot (Raspberry Pi 4 / Jetson Nano):
```bash
# Dependensi sistem (I2C, Tkinter, dll.)
sudo apt update && sudo apt install -y python3-tk i2c-tools

# Dependensi Python
pip3 install opencv-python numpy onnxruntime pygame websockets \
             adafruit-circuitpython-bno055 adafruit-circuitpython-ssd1306 \
             adafruit-blinka pillow dynamixel-sdk
```

---

## 🚀 Menjalankan Robot (Unified CLI Launcher)

Semua mode dan alat uji dapat dipanggil langsung melalui berkas pusat `main.py`:

### 1. Mode Navigasi Otonom (BEST 2026)
Navigasi mandiri menghindari rintangan dengan fusi multi-layer visi dan stabilisasi bodi PID:
```bash
python main.py --mode autonomous
```
*Kontrol tombol keyboard saat dashboard aktif:*
- `p` : Nyalakan / Matikan fitur **PID Body Leveler**
- `e` : Nyalakan / Matikan deteksi **Canny Edge**
- `s` : Posisikan robot **Standby** / Kembali jalan
- `c` : Rekalibrasi warna referensi lantai (*Ground Segmentation*)
- `q` : Matikan sistem dan matikan torsi servo dengan aman

### 2. Mode Mengikuti Orang (Demo PKKMB)
Mendeteksi dan mengikuti orang secara proporsional dengan respon animasi sapaan dan pelacak anti-flicker:
```bash
python main.py --mode follow
```

### 3. Mode Remote Gamepad Xbox (Teleoperasi)
Kendali manual menggunakan stik Xbox Controller via Bluetooth/USB:
```bash
python main.py --mode teleop
```
*Pemetaan Tombol Utama:*
- **Left Stick** : Maju/Mundur & Geser (Strafe)
- **Right Stick X** : Rotasi Badan di tempat
- **RT / LT** : Naik / Turunkan ketinggian langkah kaki
- **RB / LB** : Percepat / Perlambat langkah
- **Tombol A** : Jalankan animasi sapaan membungkuk (*Greeting*)
- **Tombol X** : Toggle animasi *Idle* (bernapas & melambaikan kaki)
- **Tombol BACK** : Toggle Mode Otomatis *Person Follow*

### 4. Mode Digital Twin 3D (Telemetri Laptop)
Visualisasi orientasi bodi robot secara nirkabel di laptop:
```bash
# 1. Jalankan bridge server di laptop
python main.py --mode telemetry

# 2. Buka dashboard di browser
# Buka file: apps/telemetry/dashboard.html
```

### 5. Alat Uji & Kalibrasi
```bash
# Uji kamera & filter visi tanpa servo (laptop friendly):
python main.py --mode test-cam

# GUI slider kontrol manual servo & pembacaan IMU:
python main.py --mode test-gui

# GUI tuning gain PID body leveling real-time:
python main.py --mode test-level
```

*(Catatan: Setiap modul aplikasi di folder `apps/` dan alat di `tools/` juga dapat dijalankan secara mandiri, misalnya `python apps/autonomous/main.py`).*

---

## 🔧 Skema Pengkabelan Hardware (Wiring Diagram)

### 1. I2C Bus Bersama (Shared I2C Bus - GPIO 2 & 3)
Sensor IMU BNO055 dan Layar OLED SSD1306 terhubung pada bus fisik I2C yang sama di Raspberry Pi:
```
Raspberry Pi Pin 3 (SDA / GPIO 2) ──┬─── BNO055 IMU (Address 0x28)
                                    └─── SSD1306 OLED (Address 0x3C)

Raspberry Pi Pin 5 (SCL / GPIO 3) ──┬─── BNO055 IMU
                                    └─── SSD1306 OLED
```

### 2. Serial Bus Servo Dynamixel AX-12A
18 buah servo daisy-chain terhubung ke adaptor USB-to-RS485/TTL (U2D2 / USB Adapter):
```
Port Serial : /dev/ttyUSB0 (Linux) atau COM21 (Windows)
Baudrate    : 1,000,000 bps (1 Mbps)
Protokol    : Dynamixel Protocol 1.0

ID Servo (18 Unit):
- L1 (Depan Kiri)     : ID 1, 2, 3   (Coxa, Femur, Tibia)
- L2 (Tengah Kiri)    : ID 4, 5, 6
- L3 (Belakang Kiri)  : ID 7, 8, 9
- L4 (Depan Kanan)    : ID 10, 11, 12
- L5 (Tengah Kanan)   : ID 13, 14, 15
- L6 (Belakang Kanan) : ID 16, 17, 18
```

---

## 🎯 Penjelasan Subsistem Kunci

### 1. Multi-Layer Perception (apps/autonomous/perception.py)
Robot mendeteksi lingkungan menggunakan pendekatan komprehensif:
1. **Edge Density (Canny)**: Mendeteksi kontur fisik rintangan pada region Kiri, Tengah, dan Kanan.
2. **Optical Flow (Farneback)**: Menghitung efek *motion parallax*; membedakan lantai datar yang bertekstur (flow kecil) dari objek yang benar-benar mendekat (flow besar).
3. **Ground Segmentation (HSV Backprojection)**: Mempelajari warna lantai di depan robot. Objek yang tidak sesuai warna lantai langsung ditandai sebagai halangan meskipun permukaannya mulus (tanpa edge).
4. **Free-space Column Profile**: Menganalisis kedalaman ruang bebas per kolom piksel untuk memberikan kemudi lembut saat melewati celah sempit.

### 2. Closed-Loop PID Body Leveling (core/body_leveler.py)
Menjaga bodi hexapod tetap horizontal terhadap gravitasi saat melewati medan berbatu, balok kayu, atau gundukan:
$$\Delta z = p_x \cdot u_{\text{pitch}} + p_y \cdot u_{\text{roll}}$$
Di mana $u$ dihitung menggunakan PID dengan *anti-windup* dan *derivative low-pass filter* untuk mencegah osilasi saat berjalan.

### 3. Anti-Flicker Target Tracker (vision/tracker.py)
Memilih target manusia berdasarkan kontinuitas jarak spasial ($L_2$ norm) dan perbandingan rasio area. Target tidak akan berpindah ke orang lain yang melintas sesaat di depan kamera, menjamin demonstrasi pameran berlangsung stabil.

---

## 🐛 Troubleshooting

| Kendala | Penyebab Umum | Solusi |
|---|---|---|
| **Kamera tidak terbuka** | Permission / device index berubah | Pastikan user tergabung di grup video (`sudo usermod -a -G video $USER`). `vision/camera.py` otomatis mencoba index 0 s.d. 5. |
| **Servo tidak merespons** | Baudrate salah / serial port terkunci | Cek `ls -l /dev/ttyUSB*`. Pastikan baudrate diatur ke `1000000`. |
| **IMU / OLED Device Busy** | Pembuatan dua bus I2C terpisah | Kode telah disatukan: `core/imu_sensor.py` membuat bus I2C yang di-share langsung ke `core/oled_display.py`. |
| **Target Tracker melompat** | Area orang lain terlalu besar di bazar | Naikkan parameter `switch_area_ratio` (misal 1.5) pada `apps/person_follow/main.py`. |
| **Body robot berosilasi saat leveling** | Gain Kd terlalu agresif atau filter bising | Kurangi gain Kd melalui `main.py --mode test-level` atau naikkan `d_filter_alpha`. |

---

**Wasaka Robotic Team 2026**
*Designed with Clean Architecture, High Performance, & Robust Reliability.*
