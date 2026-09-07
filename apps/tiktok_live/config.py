"""
WASAKA HEXAPOD - TIKTOK LIVE CONFIGURATION
==========================================
Konfigurasi akun TikTok Live dan pemetaan hadiah (gifts) ke animasi robot.

Anda dapat menambahkan atau mengubah pemetaan nama gift sesuai kebutuhan siaran.
Nama gift cocok baik dalam Bahasa Inggris maupun Bahasa Indonesia (case-insensitive).

AUTHOR: Wasaka Robotic Team
"""

# Akun TikTok streamer default (bisa di-override lewat argumen CLI --user)
DEFAULT_TIKTOK_USERNAME = "@wasakarobotic"

# ============================================================
# PEMETAAN GIFT TIKTOK -> ANIMASI ROBOT
# ============================================================
GIFT_MAP = {
    # 🌹 Melambai / Dadah (Wave)
    "rose": "wave",
    "bunga": "wave",
    "flower": "wave",
    "little crown": "wave",

    # 🍩 Berputar di Tempat (Spin)
    "doughnut": "spin",
    "donut": "spin",
    "football": "spin",
    "ball": "spin",
    "balon": "spin",
    "capybara": "spin",
    "panda": "spin",

    # 🎵 Bergoyang / Joget / Dance (Dance)
    "tiktok": "dance",
    "music": "dance",
    "speaker": "dance",
    "mic": "dance",
    "guitar": "dance",
    "gitar": "dance",
    "dj": "dance",
    "lion": "dance",
    "singa": "dance",
    "whale": "dance",

    # 💖 Menaikkan Kaki / Berjinjit Tinggi (Tiptoe)
    "heart": "tiptoe",
    "hati": "tiptoe",
    "love": "tiptoe",
    "finger heart": "tiptoe",
    "hand heart": "tiptoe",
    "star": "tiptoe",
    "bintang": "tiptoe",

    # ☕ Merendah / Jongkok Bodi (Crouch)
    "coffee": "crouch",
    "kopi": "crouch",
    "ice cream": "crouch",
    "ice cream cone": "crouch",
    "perfume": "crouch",
    "lollipop": "crouch",
    "candy": "crouch",

    # 👑 Membungkuk / Hormat (Bow)
    "crown": "bow",
    "mahkota": "bow",
    "hat and mustache": "bow",
    "top hat": "bow",
    "trophy": "bow",
}

# Animasi fallback jika nama gift belum terdaftar
DEFAULT_UNMAPPED_GIFT_ANIM = "dance"

# ============================================================
# TRIGGER EVENT LAINNYA
# ============================================================
# Pemicu saat ada follower baru
FOLLOW_ANIMATION = "bow"

# Pemicu milestone likes (misal setiap kelipatan 100 like)
LIKE_STEP_THRESHOLD = 100
LIKE_ANIMATION = "twist"

# Pemicu kata kunci di kolom komentar live chat
COMMENT_TRIGGERS = {
    "!dadah": "wave",
    "!melambai": "wave",
    "!putar": "spin",
    "!spin": "spin",
    "!joget": "dance",
    "!goyang": "dance",
    "!dance": "dance",
    "!jinjit": "tiptoe",
    "!tinggi": "tiptoe",
    "!jongkok": "crouch",
    "!merendah": "crouch",
    "!hormat": "bow",
    "!bungkuk": "bow",
    "!twist": "twist",
}

# Durasi standar tiap jenis animasi (detik)
ANIM_DURATIONS = {
    "wave": 3.5,
    "spin": 3.8,
    "dance": 4.5,
    "tiptoe": 3.0,
    "crouch": 3.0,
    "bow": 2.2,
    "twist": 2.0,
    "breathe": 2.5,
}
