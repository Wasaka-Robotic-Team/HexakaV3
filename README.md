# 🤖 WASAKA HEXAPOD - Unified Robotics Platform

Repositori resmi **Tim Robotika Wasaka** yang menggabungkan seluruh fungsionalitas robot hexapod 18-DOF ke dalam satu arsitektur modular, terstandarisasi, dan bersih (*clean code*).

Basis kode ini menyatukan tiga misi utama:
1. **Autonomous Navigation (Kompetisi BEST 2026)**: Navigasi otonom dengan *multi-layer vision perception*, stabilisasi bodi tertutup (*closed-loop PID body leveling*), penghindaran rintangan, dan Digital Twin 3D.
2. **Person Following & Teleoperation (Demo Pameran PKKMB)**: Pelacakan manusia (*human tracking*) cerdas dengan *hysteresis anti-flicker*, animasi interaktif (sapaan bungkuk, napas, lambaian kaki), dan kendali jarak jauh manual stik Xbox.
3. **Interactive TikTok Live Streaming (Siaran Langsung Interaktif)**: Merespons hadiah (*gifts*), komentar, likes, dan follow penonton TikTok Live dengan animasi fisik robot secara real-time.

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
│   ├── tiktok_live/            # Mode Siaran Langsung Interaktif TikTok Live
│   │   ├── main.py             # Entrypoint loop live TikTok + eksekusi animasi
│   │   ├── config.py           # Pemetaan gift TikTok ke jenis animasi robot
│   │   ├── animator.py         # Koreografi kinematika gerak 18-DOF servo
│   │   ├── queue_manager.py    # Pengelola antrian hadiah (anti-tabrakan saat banjir gift)
│   │   ├── tiktok_client.py    # Listener real-time WebSocket room TikTok Live
│   │   └── mock_trigger.py     # Simulator keyboard offline untuk uji coba tanpa live
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
# Opsional untuk siaran TikTok nyata:
pip install TikTokLive
```

#### Di Robot (Raspberry Pi 4 / Jetson Nano):
```bash
# Dependensi sistem (I2C, Tkinter, dll.)
sudo apt update && sudo apt install -y python3-tk i2c-tools

# Dependensi Python
pip3 install opencv-python numpy onnxruntime pygame websockets \
             adafruit-circuitpython-bno055 adafruit-circuitpython-ssd1306 \
             adafruit-blinka pillow dynamixel-sdk TikTokLive
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

### 4. Mode TikTok Live Interactive Gift
Menghubungkan robot ke siaran langsung TikTok Live streamer:
```bash
# Siaran Langsung Nyata:
python main.py --mode tiktok --user @username_anda

# Mode Uji Coba / Simulasi Offline (Tekan tombol keyboard 1-7 untuk memicu animasi):
python main.py --mode tiktok --test
```

#### 🎁 Tabel Pemetaan Hadiah (Gifts) ke Gerakan Robot:
| Hadiah TikTok (Gifts) | Aksi Robot | Deskripsi Gerakan |
|---|---|---|
| 🌹 **Bunga / Rose** | `wave` | Melambaikan kaki depan kanan ke penonton secara antusias |
| 🍩 **Donut / Balon** | `spin` | Berputar 360° di tempat dengan tripod gait |
| 🎵 **TikTok / Musik** | `dance` | Bodi bergoyang ritmis ke kiri-kanan dan depan-belakang |
| 💖 **Hati / Heart** | `tiptoe` | Bodi terangkat maksimal, robot berdiri di atas ujung kaki (*tiptoes*) |
| ☕ **Kopi / Ice Cream** | `crouch` | Bodi merendah ke lantai (jongkok) lalu bangkit kembali |
| 👑 **Follower Baru** | `bow` | Bodi depan merunduk membungkuk hormat santun ke kamera |
| 👍 **100+ Likes** | `twist` | Bodi menggeleng cepat ke kiri-kanan di tempat |
| ⏳ **Idle (Menunggu)** | `breathe` | Gerakan napas sinusoidal lembut agar robot tetap hidup |

*(Pemetaan hadiah dapat dikustomisasi dengan mudah pada `apps/tiktok_live/config.py`).*

### 5. Mode Digital Twin 3D (Telemetri Laptop)
Visualisasi orientasi bodi robot secara nirkabel di laptop:
```bash
# 1. Jalankan bridge server di laptop
python main.py --mode telemetry

# 2. Buka dashboard di browser
# Buka file: apps/telemetry/dashboard.html
```

### 6. Alat Uji & Kalibrasi
```bash
# Uji kamera & filter visi tanpa servo (laptop friendly):
python main.py --mode test-cam

# GUI slider kontrol manual servo & pembacaan IMU:
python main.py --mode test-gui

# GUI tuning gain PID body leveling real-time:
python main.py --mode test-level
```

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

## 🐛 Troubleshooting

| Kendala | Penyebab Umum | Solusi |
|---|---|---|
| **Kamera tidak terbuka** | Permission / device index berubah | Pastikan user tergabung di grup video (`sudo usermod -a -G video $USER`). `vision/camera.py` otomatis mencoba index 0 s.d. 5. |
| **Servo tidak merespons** | Baudrate salah / serial port terkunci | Cek `ls -l /dev/ttyUSB*`. Pastikan baudrate diatur ke `1000000`. |
| **IMU / OLED Device Busy** | Pembuatan dua bus I2C terpisah | Kode telah disatukan: `core/imu_sensor.py` membuat bus I2C yang di-share langsung ke `core/oled_display.py`. |
| **TikTokLive belum terinstall** | Library belum ada di environment | Jalankan `pip install TikTokLive` atau gunakan mode simulasi `--test`. |
| **Live stream TikTok tidak terbaca** | Akun sedang tidak siaran live | Pastikan streamer sedang aktif bersiaran (Live) saat menjalankan program. |

---

**Wasaka Robotic Team 2026**
*Designed with Clean Architecture, High Performance, & Robust Reliability.*
