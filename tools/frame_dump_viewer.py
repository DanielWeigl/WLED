#!/usr/bin/env python3
"""
Minimal desktop viewer for WLED WLED_ENABLE_FRAME_DUMP UDP broadcasts.

Firmware (compile with -D WLED_ENABLE_FRAME_DUMP) sends every show() frame as
fragmented UDP packets to port 4210 (or WLED_FRAME_DUMP_PORT).

Usage:
  python3 tools/frame_dump_viewer.py
  python3 tools/frame_dump_viewer.py --port 4210

Controls:
  Space / Pause button  — pause (incoming packets are discarded) / resume
  Gamma slider          — display gamma (WLED GV: 0.1–3.0, default 2.2); overlay RGBW stays raw
  Mouse over a row      — overlay with frame header + RGBW of the pixel under the cursor
"""

from __future__ import annotations

import argparse
import socket
import struct
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

MAGIC = b"WF"
VERSION = 1
# magic, ver, flags, seq, bri, reserved, total, width, height, frag_i, frag_n, led_off, led_n
HEADER_FMT = "<2sBBIBBHHHHHHH"  # 24 bytes; see wled00/frame_dump.cpp
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 24, HEADER_SIZE
HISTORY_SEC = 5.0
DEFAULT_PORT = 4210
PIXEL_H = 14
MAX_CANVAS_W = 1600
IMG_ORIGIN_X = 4
IMG_ORIGIN_Y = 4
# Match WLED LED settings GV (settings_leds.htm / gammaCorrectVal).
GAMMA_MIN = 0.1
GAMMA_MAX = 3.0
GAMMA_STEP = 0.1
GAMMA_DEFAULT = 2.2


def build_gamma_lut(gamma: float) -> List[int]:
    """Same mapping as NeoGammaWLEDMethod::calcGammaTable() in colors.cpp."""
    if gamma <= 0.0:
        gamma = 1.0
    lut = [0] * 256
    for i in range(1, 256):
        lut[i] = int(pow(i / 255.0, gamma) * 255.0 + 0.5)
    return lut


@dataclass
class Frame:
    seq: int
    bri: int
    width: int
    height: int
    rgbw: bytes  # len = total_leds * 4
    t: float

    @property
    def led_count(self) -> int:
        return len(self.rgbw) // 4

    def pixel_rgbw(self, index: int) -> Optional[tuple]:
        if index < 0 or index >= self.led_count:
            return None
        o = index * 4
        return self.rgbw[o], self.rgbw[o + 1], self.rgbw[o + 2], self.rgbw[o + 3]


@dataclass
class ViewLayout:
    frames: List[Frame]
    leds: int
    zx: int
    zy: int


class FrameAssembler:
    """Reassemble fragmented WF packets into complete frames."""

    def __init__(self) -> None:
        self._partial: Dict[int, Dict] = {}

    def feed(self, data: bytes) -> Optional[Frame]:
        if len(data) < HEADER_SIZE:
            return None
        magic, ver, flags, seq, bri, _res, total, width, height, frag_i, frag_n, led_off, led_n = struct.unpack_from(
            HEADER_FMT, data, 0
        )
        if magic != MAGIC or ver != VERSION or led_n == 0:
            return None
        if not (flags & 0x01):
            return None
        need = HEADER_SIZE + led_n * 4
        if len(data) < need:
            return None
        payload = data[HEADER_SIZE:need]

        if frag_n <= 1:
            return Frame(seq, bri, width, height, payload, time.monotonic())

        slot = self._partial.get(seq)
        if slot is None or slot["frag_n"] != frag_n or slot["total"] != total:
            slot = {
                "frag_n": frag_n,
                "total": total,
                "bri": bri,
                "width": width,
                "height": height,
                "parts": {},
                "t": time.monotonic(),
            }
            self._partial[seq] = slot
            cutoff = time.monotonic() - 2.0
            for k in [k for k, v in self._partial.items() if v["t"] < cutoff and k != seq]:
                del self._partial[k]

        slot["parts"][frag_i] = (led_off, payload)
        if len(slot["parts"]) < frag_n:
            return None

        buf = bytearray(total * 4)
        for led_off_i, part in slot["parts"].values():
            start = led_off_i * 4
            buf[start : start + len(part)] = part
        del self._partial[seq]
        return Frame(seq, slot["bri"], slot["width"], slot["height"], bytes(buf), time.monotonic())


class Viewer:
    def __init__(self, port: int) -> None:
        self.port = port
        self.paused = False
        self._lock = threading.Lock()
        self._frames: Deque[Frame] = deque()
        self._assembler = FrameAssembler()
        self._stop = threading.Event()
        self._dirty = True
        self._layout: Optional[ViewLayout] = None
        self._hovering = False
        self._last_hover: Optional[tuple] = None  # (row, col)
        self._gamma = GAMMA_DEFAULT
        self._gamma_lut = build_gamma_lut(self._gamma)

        self.root = tk.Tk()
        self.root.title(f"WLED frame dump — UDP :{port}")
        self.root.geometry("900x600")

        top = tk.Frame(self.root)
        top.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)

        self.pause_btn = tk.Button(top, text="Pause", width=10, command=self.toggle_pause)
        self.pause_btn.pack(side=tk.LEFT)

        gamma_box = tk.Frame(top)
        gamma_box.pack(side=tk.LEFT, padx=(12, 4))
        tk.Label(gamma_box, text="Gamma").pack(side=tk.LEFT)
        self.gamma_var = tk.DoubleVar(value=GAMMA_DEFAULT)
        self.gamma_scale = tk.Scale(
            gamma_box,
            from_=GAMMA_MIN,
            to=GAMMA_MAX,
            resolution=GAMMA_STEP,
            orient=tk.HORIZONTAL,
            length=160,
            showvalue=False,
            variable=self.gamma_var,
            command=self._on_gamma,
        )
        self.gamma_scale.pack(side=tk.LEFT, padx=(4, 4))
        self.gamma_label = tk.Label(gamma_box, text=f"{GAMMA_DEFAULT:.1f}", width=4, anchor="w")
        self.gamma_label.pack(side=tk.LEFT)

        self.status = tk.Label(top, text="listening…", anchor="w")
        self.status.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

        body = tk.Frame(self.root)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(body, bg="#111111", highlightthickness=0)
        self.vsb = tk.Scrollbar(body, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vsb.set)
        self.vsb.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Floating overlay (hidden until hover hits a valid pixel row).
        self.overlay = tk.Frame(
            body,
            bg="#1e1e1e",
            highlightbackground="#666666",
            highlightthickness=1,
            padx=10,
            pady=8,
        )
        self.overlay_title = tk.Label(
            self.overlay, text="Frame", bg="#1e1e1e", fg="#ffffff", font=("TkDefaultFont", 10, "bold"), anchor="w"
        )
        self.overlay_title.pack(fill=tk.X)
        self.overlay_header = tk.Label(
            self.overlay, text="", bg="#1e1e1e", fg="#dddddd", justify=tk.LEFT, anchor="nw", font=("TkFixedFont", 9)
        )
        self.overlay_header.pack(fill=tk.X, pady=(4, 6))

        pix_row = tk.Frame(self.overlay, bg="#1e1e1e")
        pix_row.pack(fill=tk.X)
        self.swatch = tk.Canvas(pix_row, width=28, height=28, bg="#000000", highlightthickness=1, highlightbackground="#888888")
        self.swatch.pack(side=tk.LEFT, padx=(0, 8))
        self.overlay_pixel = tk.Label(
            pix_row, text="", bg="#1e1e1e", fg="#dddddd", justify=tk.LEFT, anchor="w", font=("TkFixedFont", 9)
        )
        self.overlay_pixel.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.overlay.place_forget()

        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<Leave>", self._on_leave)
        self.root.bind("<space>", lambda _e: self.toggle_pause())
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._rx = threading.Thread(target=self._rx_loop, name="frame-dump-rx", daemon=True)
        self._rx.start()
        self.root.after(33, self._tick)

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.pause_btn.configure(text="Resume" if self.paused else "Pause")
        self._update_status()

    def _on_gamma(self, _value: str) -> None:
        gamma = float(self.gamma_var.get())
        # Mirror WLED clamp behaviour (set.cpp): out-of-range → 1.0 (unity).
        if gamma < GAMMA_MIN or gamma > GAMMA_MAX:
            gamma = 1.0
            self.gamma_var.set(gamma)
        self._gamma = gamma
        self._gamma_lut = build_gamma_lut(gamma)
        self.gamma_label.configure(text=f"{gamma:.1f}")
        with self._lock:
            self._dirty = True

    def _apply_gamma(self, r: int, g: int, b: int) -> tuple:
        lut = self._gamma_lut
        return lut[r], lut[g], lut[b]

    def on_close(self) -> None:
        self._stop.set()
        self.root.destroy()

    def _rx_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self.port))
        sock.settimeout(0.25)
        while not self._stop.is_set():
            try:
                data, _addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if self.paused:
                continue
            frame = self._assembler.feed(data)
            if frame is None:
                continue
            with self._lock:
                self._frames.append(frame)
                self._prune_locked()
                self._dirty = True

    def _prune_locked(self) -> None:
        cutoff = time.monotonic() - HISTORY_SEC
        while self._frames and self._frames[0].t < cutoff:
            self._frames.popleft()

    def _update_status(self, extra: str = "") -> None:
        with self._lock:
            n = len(self._frames)
            last = self._frames[-1] if self._frames else None
        parts = ["PAUSED" if self.paused else "LIVE", f"{n} frames / {HISTORY_SEC:.0f}s"]
        if last:
            parts.append(f"seq={last.seq} leds={last.led_count} bri={last.bri} {last.width}x{last.height}")
        if extra:
            parts.append(extra)
        self.status.configure(text="  |  ".join(parts))

    def _tick(self) -> None:
        if self._stop.is_set():
            return
        with self._lock:
            dirty = self._dirty
            frames: List[Frame] = list(self._frames) if dirty else []
            if dirty:
                self._dirty = False
        if dirty:
            self._redraw(frames)
            if self._hovering and self._last_hover is not None:
                self._show_hover(*self._last_hover)
        self._update_status()
        self.root.after(33, self._tick)

    def _redraw(self, frames: List[Frame]) -> None:
        self.canvas.delete("all")
        if not frames:
            self._img = None
            self._layout = None
            self.canvas.configure(scrollregion=(0, 0, 100, 100))
            return

        leds = max(1, frames[-1].led_count)
        avail_w = max(self.canvas.winfo_width(), 200)
        zx = max(2, min(24, (min(avail_w, MAX_CANVAS_W) - 8) // leds))
        zy = PIXEL_H

        img = tk.PhotoImage(width=leds, height=len(frames))
        for y, fr in enumerate(frames):
            n = min(leds, fr.led_count)
            scale = fr.bri / 255.0 if fr.bri else 0.0
            colors: List[str] = []
            for i in range(n):
                r, g, b, w = fr.pixel_rgbw(i)  # type: ignore[misc]
                # Preview order: fold W → apply bri → display gamma (WLED Correct()).
                rr = min(255, int(min(255, r + w) * scale))
                gg = min(255, int(min(255, g + w) * scale))
                bb = min(255, int(min(255, b + w) * scale))
                rr, gg, bb = self._apply_gamma(rr, gg, bb)
                colors.append(f"#{rr:02x}{gg:02x}{bb:02x}")
            if n < leds:
                colors.extend(["#000000"] * (leds - n))
            img.put("{" + " ".join(colors) + "}", to=(0, y))

        scaled = img.zoom(zx, zy)
        self._img = scaled
        total_w = leds * zx + 8
        total_h = len(frames) * zy + 8
        self.canvas.create_image(IMG_ORIGIN_X, IMG_ORIGIN_Y, anchor=tk.NW, image=scaled)
        self.canvas.configure(scrollregion=(0, 0, total_w, total_h))
        self._layout = ViewLayout(frames=frames, leds=leds, zx=zx, zy=zy)
        # Keep newest in view unless the user is inspecting a row.
        if not self._hovering:
            self.canvas.yview_moveto(1.0)

    def _hit_test(self, event_x: int, event_y: int) -> Optional[tuple]:
        layout = self._layout
        if layout is None or not layout.frames:
            return None
        cx = self.canvas.canvasx(event_x)
        cy = self.canvas.canvasy(event_y)
        col = int((cx - IMG_ORIGIN_X) // layout.zx)
        row = int((cy - IMG_ORIGIN_Y) // layout.zy)
        if row < 0 or row >= len(layout.frames):
            return None
        if col < 0 or col >= layout.leds:
            return None
        if col >= layout.frames[row].led_count:
            return None
        return row, col

    def _on_motion(self, event: tk.Event) -> None:
        hit = self._hit_test(event.x, event.y)
        self._hovering = hit is not None
        self._last_hover = hit
        if hit is None:
            self.overlay.place_forget()
            return
        self._show_hover(*hit)
        # Keep overlay near the pointer, clamped inside the body.
        body_w = max(self.overlay.master.winfo_width(), 1)
        body_h = max(self.overlay.master.winfo_height(), 1)
        self.overlay.update_idletasks()
        ow = self.overlay.winfo_reqwidth()
        oh = self.overlay.winfo_reqheight()
        # event coords are canvas-relative; overlay is placed on the body (canvas is left-packed).
        x = event.x + 18
        y = event.y + 18
        if x + ow > body_w - 8:
            x = max(8, event.x - ow - 12)
        if y + oh > body_h - 8:
            y = max(8, event.y - oh - 12)
        self.overlay.place(x=x, y=y)
        self.overlay.lift()

    def _on_leave(self, _event: tk.Event) -> None:
        self._hovering = False
        self._last_hover = None
        self.overlay.place_forget()

    def _show_hover(self, row: int, col: int) -> None:
        layout = self._layout
        if layout is None or row < 0 or row >= len(layout.frames):
            self.overlay.place_forget()
            return
        fr = layout.frames[row]
        rgbw = fr.pixel_rgbw(col)
        if rgbw is None:
            self.overlay.place_forget()
            return
        r, g, b, w = rgbw
        age_ms = max(0, int((time.monotonic() - fr.t) * 1000))
        header = (
            f"seq        {fr.seq}\n"
            f"brightness {fr.bri}\n"
            f"geometry   {fr.width} × {fr.height}\n"
            f"LEDs       {fr.led_count}\n"
            f"age        {age_ms} ms\n"
            f"row        {row} / {len(layout.frames) - 1}  (0 = oldest)"
        )
        self.overlay_title.configure(text=f"Frame #{fr.seq}")
        self.overlay_header.configure(text=header)

        # Overlay numbers stay raw UDP RGBW; swatch matches strip preview (fold W + gamma).
        rr, gg, bb = self._apply_gamma(min(255, r + w), min(255, g + w), min(255, b + w))
        self.swatch.configure(bg=f"#{rr:02x}{gg:02x}{bb:02x}")
        xy = ""
        if fr.width > 1:
            x = col % fr.width
            y = col // fr.width
            xy = f"\nxy         {x}, {y}"
        self.overlay_pixel.configure(
            text=(
                f"pixel      {col}{xy}\n"
                f"R {r:3d}   G {g:3d}   B {b:3d}   W {w:3d}\n"
                f"hex        #{r:02X}{g:02X}{b:02X}{w:02X}"
            )
        )

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    ap = argparse.ArgumentParser(description="WLED UDP frame dump viewer")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"UDP listen port (default {DEFAULT_PORT})")
    args = ap.parse_args()
    Viewer(args.port).run()


if __name__ == "__main__":
    main()
