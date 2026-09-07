"""
WASAKA HEXAPOD - TARGET TRACKER MODULE
=======================================
Pelacakan target tunggal dengan mekanisme histeresis (anti-flicker).

Dirancang khusus untuk menjaga kontinuitas target di keramaian (misalnya saat
demo pameran / bazar kampus PKKMB), agar robot tidak mudah teralihkan ke orang
lain yang melintas sesaat di depan kamera.

Strategi:
1. Belum ada target aktif -> pilih bounding box terbesar (asumsi: orang terdekat).
2. Sudah ada target aktif -> cari kandidat terdekat secara posisi (kontinuitas spasial),
   bukan yang terbesar lagi.
3. Pergantian target hanya terjadi jika target lama hilang melampaui `lost_grace_frames`
   ATAU ada target baru yang jauh lebih besar secara proporsional (`switch_area_ratio`).

State:
- SEARCH: Tidak ada target valid terdeteksi (robot menjalankan animasi idle / mencari).
- TRACKING: Target valid sedang aktif diikuti.
- LOST_GRACE: Target hilang sesaat (oklusi sementara); robot mempertahankan perintah terakhir.

AUTHOR: Wasaka Robotic Team
"""

import math


class TargetTracker:
    """
    Pelacak target tunggal berbasis kontinuitas posisi & histeresis luas.
    """

    def __init__(self, switch_area_ratio=1.3, max_center_jump=120, lost_grace_frames=8):
        """
        Parameters
        ----------
        switch_area_ratio : float
            Rasio kelipatan area target baru untuk meng-override target lama yang masih ada.
        max_center_jump : float / int
            Jarak maksimum (piksel) pusat bounding box antar-frame agar dianggap target yang sama.
        lost_grace_frames : int
            Jumlah frame toleransi saat target hilang sebelum beralih ke state SEARCH.
        """
        self.switch_area_ratio = float(switch_area_ratio)
        self.max_center_jump = float(max_center_jump)
        self.lost_grace_frames = int(lost_grace_frames)
        self.current = None
        self.lost_count = 0

    def reset(self):
        """Reset pelacak ke kondisi awal."""
        self.current = None
        self.lost_count = 0

    def _state(self):
        if self.current is None:
            return "SEARCH"
        if self.lost_count > 0:
            return "LOST_GRACE"
        return "TRACKING"

    def update(self, detections):
        """
        Memperbarui status pelacakan berdasarkan daftar deteksi pada frame saat ini.

        Args:
            detections: List of dict hasil detektor [{'bbox': (x1,y1,x2,y2), 'cx': cx, 'cy': cy, 'area': area, ...}]

        Returns:
            tuple: (current_target_dict_atau_None, state_string)
        """
        if not detections:
            self.lost_count += 1
            if self.lost_count > self.lost_grace_frames:
                self.current = None
            return self.current, self._state()

        # Kasus 1: Belum ada target aktif -> pilih deteksi dengan area terbesar
        if self.current is None:
            self.current = max(detections, key=lambda d: d.get("area", 0.0))
            self.lost_count = 0
            return self.current, self._state()

        # Kasus 2: Sudah ada target aktif -> cari deteksi dengan jarak pusat terdekat
        curr_cx = self.current.get("cx", 0.0)
        curr_cy = self.current.get("cy", 0.0)

        best_match = min(
            detections,
            key=lambda d: math.hypot(d.get("cx", 0.0) - curr_cx, d.get("cy", 0.0) - curr_cy)
        )
        dist = math.hypot(best_match.get("cx", 0.0) - curr_cx, best_match.get("cy", 0.0) - curr_cy)

        if dist <= self.max_center_jump:
            # Kandidat terdekat berada dalam batas toleransi lompatan pusat
            largest = max(detections, key=lambda d: d.get("area", 0.0))
            curr_area = self.current.get("area", 1.0)

            # Cek apakah ada target baru yang jauh lebih besar secara signifikan
            if largest.get("area", 0.0) > curr_area * self.switch_area_ratio:
                self.current = largest
            else:
                self.current = best_match
            self.lost_count = 0
        else:
            # Target lama tidak ditemukan di sekitar koordinat sebelumnya
            self.lost_count += 1
            if self.lost_count > self.lost_grace_frames:
                # Masa tenggang habis -> pilih target terbesar yang tersedia
                self.current = max(detections, key=lambda d: d.get("area", 0.0))
                self.lost_count = 0

        return self.current, self._state()
