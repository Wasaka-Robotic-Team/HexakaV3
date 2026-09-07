"""
WASAKA HEXAPOD - TIKTOK LIVE CLIENT LISTENER
=============================================
Menyambungkan robot ke siaran langsung TikTok secara real-time via library `TikTokLive`.
Menangkap event:
- GiftEvent    : Hadiah masuk -> mapping ke animasi robot (wave, spin, dance, tiptoe, crouch, bow).
- FollowEvent  : Penonton baru follow -> animasi hormat (bow).
- LikeEvent    : Akumulasi like mencapai kelipatan 100 -> animasi twist badan.
- CommentEvent : Kata kunci khusus (misal !joget, !dadah) -> trigger animasi langsung.

AUTHOR: Wasaka Robotic Team
"""

import asyncio
import threading
from .config import (
    GIFT_MAP,
    DEFAULT_UNMAPPED_GIFT_ANIM,
    FOLLOW_ANIMATION,
    LIKE_STEP_THRESHOLD,
    LIKE_ANIMATION,
    COMMENT_TRIGGERS,
)

try:
    from TikTokLive import TikTokLiveClient
    from TikTokLive.types.events import GiftEvent, LikeEvent, FollowEvent, CommentEvent
    HAS_TIKTOK_LIB = True
except ImportError:
    HAS_TIKTOK_LIB = False
    TikTokLiveClient = None
    GiftEvent = None
    LikeEvent = None
    FollowEvent = None
    CommentEvent = None


class TikTokLiveListener:
    """
    Listener asinkronus untuk event siaran langsung TikTok.
    """

    def __init__(self, username, queue_manager):
        self.username = username.strip()
        if not self.username.startswith("@"):
            self.username = "@" + self.username

        self.queue = queue_manager
        self.client = None
        self._thread = None
        self._loop = None
        self._running = False
        self._like_counter = 0

    def is_available(self):
        return HAS_TIKTOK_LIB

    def start(self):
        if not HAS_TIKTOK_LIB:
            print("\n" + "=" * 60)
            print("[WARN] Library 'TikTokLive' belum terinstall di environment Python ini.")
            print("Untuk menghubungkan ke live streaming TikTok asli, jalankan:")
            print("    pip install TikTokLive")
            print("Saat ini Anda dapat menggunakan mode uji simulasi:")
            print("    python main.py --mode tiktok --test")
            print("=" * 60 + "\n")
            return False

        self._running = True
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()
        return True

    def _run_async_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._connect_and_listen())
        except Exception as e:
            print(f"[TIKTOK] Koneksi berakhir: {e}")
        finally:
            self._loop.close()

    async def _connect_and_listen(self):
        print(f"[TIKTOK] Menghubungkan ke siaran langsung {self.username}...")
        self.client = TikTokLiveClient(unique_id=self.username)

        # 1. Event Hadiah (Gift)
        @self.client.on(GiftEvent)
        async def on_gift(event: GiftEvent):
            # Hanya proses streak yang selesai atau gift tunggal
            # (menghindari duplikasi event pada streak combo)
            if event.gift.gift_type == 1 and event.gift.repeat_end != 1:
                return

            gift_name = getattr(event.gift.gift_info, "name", "Gift").lower()
            sender = getattr(event.user, "nickname", getattr(event.user, "uniqueId", "Penonton"))
            count = getattr(event.gift, "repeat_count", 1)

            # Cari pemetaan animasi
            anim_name = GIFT_MAP.get(gift_name, DEFAULT_UNMAPPED_GIFT_ANIM)
            print(f"[TIKTOK-GIFT] {sender} mengirim {event.gift.gift_info.name} (x{count}) -> Trigger [{anim_name.upper()}]")
            self.queue.enqueue(anim_name, sender=sender, gift_name=event.gift.gift_info.name, count=count)

        # 2. Event Follower Baru
        @self.client.on(FollowEvent)
        async def on_follow(event: FollowEvent):
            sender = getattr(event.user, "nickname", "Sahabat")
            print(f"[TIKTOK-FOLLOW] {sender} mulai mengikuti! -> Trigger [{FOLLOW_ANIMATION.upper()}]")
            self.queue.enqueue(FOLLOW_ANIMATION, sender=sender, gift_name="New Follower", count=1)

        # 3. Event Likes
        @self.client.on(LikeEvent)
        async def on_like(event: LikeEvent):
            add_likes = getattr(event, "likes", 1)
            self._like_counter += add_likes
            if self._like_counter >= LIKE_STEP_THRESHOLD:
                self._like_counter = 0
                sender = getattr(event.user, "nickname", "Penonton")
                print(f"[TIKTOK-LIKE] Milestone Likes tercapai! -> Trigger [{LIKE_ANIMATION.upper()}]")
                self.queue.enqueue(LIKE_ANIMATION, sender=sender, gift_name="100+ Likes", count=1)

        # 4. Event Komentar (Chat Trigger)
        @self.client.on(CommentEvent)
        async def on_comment(event: CommentEvent):
            comment_text = getattr(event, "comment", "").strip().lower()
            if comment_text in COMMENT_TRIGGERS:
                sender = getattr(event.user, "nickname", "Penonton")
                anim_name = COMMENT_TRIGGERS[comment_text]
                print(f"[TIKTOK-CHAT] {sender} mengetik '{comment_text}' -> Trigger [{anim_name.upper()}]")
                self.queue.enqueue(anim_name, sender=sender, gift_name=comment_text, count=1)

        try:
            await self.client.start()
        except Exception as e:
            print(f"[ERROR] Gagal tersambung ke TikTok Live ({e}). Pastikan akun sedang live!")

    def stop(self):
        self._running = False
        if self.client and hasattr(self.client, "disconnect"):
            try:
                self.client.disconnect()
            except Exception:
                pass
