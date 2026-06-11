import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

import tkinter as tk
from tkinter import Scale, VERTICAL, HORIZONTAL
import threading
import time
import math

try:
    import pygame
    _PYGAME_OK = True
except ImportError:
    _PYGAME_OK = False

# ---------------------------------------------------------------------------
# Steam Deck button/axis indices (SDL2 / hid-steam driver, no Steam Input)
#   Adjust these constants if your system maps the device differently.
# ---------------------------------------------------------------------------
SD_AXIS_LX   = 0   # Left  stick horizontal (-1=left, +1=right)   → turn
SD_AXIS_LY   = 1   # Left  stick vertical   (-1=up/fwd, +1=down)  → forward
SD_AXIS_L2   = 2   # Left  trigger          (-1=rest,   +1=pressed)→ reset
SD_AXIS_RY   = 4   # Right stick vertical   (push up = throttle)
SD_DEADZONE  = 0.12
SD_L2_THRESH = 0.0  # L2 value above this (any real press) triggers full reset

SD_BTN_A     = 0   # South  — retract back-right  leg
SD_BTN_B     = 1   # East   — retract front-right leg
SD_BTN_X     = 2   # West   — retract back-left   leg
SD_BTN_Y     = 3   # North  — retract front-left  leg
SD_BTN_L1    = 4   # L1     — extend   all legs gradually
SD_BTN_R1    = 5   # R1     — retract  all legs gradually

# Back paddles — move individual wheel forward at current speed
SD_BTN_L4    = 11  # upper-left  paddle → FL wheel
SD_BTN_L5    = 13  # lower-left  paddle → BL wheel
SD_BTN_R4    = 12  # upper-right paddle → FR wheel
SD_BTN_R5    = 14  # lower-right paddle → BR wheel

# Degrees to extend/retract per 50 ms poll tick while button is held (~60°/s)
SD_LEG_STEP  = 3

# ---------------------------------------------------------------------------
# Iron Man HUD color palette
# ---------------------------------------------------------------------------
C_BG         = "#0a1a35"   # navy background
C_PANEL      = "#122448"   # vivid dark blue panel
C_DARK       = "#060e20"   # darkest elements / diagram bg
C_CYAN       = "#40dcff"   # bright JARVIS blue
C_CYAN_DIM   = "#205878"   # visible blue border
C_GREEN      = "#00f5bc"   # forward / positive - vivid teal-green
C_GREEN_DIM  = "#005a48"   # dim green glow
C_RED        = "#ff4060"   # reverse / danger - vivid red
C_RED_DIM    = "#550020"   # dim red glow
C_PURPLE     = "#7aabff"   # leg indicators - bright blue-purple
C_PURPLE_DIM = "#1e3888"   # dim blue-purple glow
C_WHITE      = "#e8f8ff"   # primary text - near-white blue-tinted
C_GRAY       = "#4888d0"   # muted / stopped state - vivid medium blue
C_TROUGH     = "#102e55"   # slider trough - visible deep blue

# Movement key → set_* method suffix
_MOVE = {
    'w': 'forward', 'up':    'forward',
    'a': 'left',    'left':  'left',
    's': 'reverse', 'down':  'reverse',
    'd': 'right',   'right': 'right',
}



def _sci_frame(parent, title, **kw):
    """LabelFrame with neon-cyan title and dark panel background."""
    return tk.LabelFrame(
        parent, text=f"  {title}  ",
        bg=C_PANEL, fg=C_CYAN,
        font=("Courier New", 9, "bold"),
        bd=1, relief=tk.SOLID,
        highlightbackground=C_CYAN_DIM,
        highlightthickness=1,
        **kw,
    )


class GuiTeleop(Node):

    def __init__(self):
        super().__init__('gui_teleop')
        self.pub       = self.create_publisher(Int32MultiArray, 'wheel_commands', 10)
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)

        self.wheel_speed    = [0, 0, 0, 0]
        self.leg_angles     = [0, 0, 0, 0]
        self.wheel_currents = [0, 0, 0, 0]
        self.leg_currents   = [0, 0, 0, 0]
        self.lock           = threading.Lock()

        self.create_subscription(
            Int32MultiArray, 'wheel_currents', self._wheel_currents_cb, 10)
        self.create_subscription(
            Int32MultiArray, 'leg_currents', self._leg_currents_cb, 10)

        self.wheel_max  = 50
        self.speed_pct  = 20
        self.dead_man   = True
        self._syncing   = False
        self._held_keys = []        # ordered list of currently-held movement keys
        self._hb_state  = False     # heartbeat blink state

        self._sd_active = False     # Steam Deck mode on/off

        self.wheel_sliders = {}
        self.wheel_labels  = {}
        self.leg_sliders   = {}
        self.leg_labels    = {}

        # Diagram item caches — avoid full redraw every frame
        self._diagram_size  = (0, 0)
        self._last_diag_spd = None
        self._diag_glow  = {}
        self._diag_disc  = {}
        self._diag_dot   = {}
        self._diag_idx   = {}
        self._diag_spd   = {}

        # Slider dirty-check
        self._last_ui_spd = None

        self._build_gui()

    # -----------------------------------------------------------------------
    # GUI construction
    # -----------------------------------------------------------------------

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("WHEEL TELEOP  ◈  ROS2")
        self.root.geometry("1440x920")
        self.root.configure(bg=C_BG)

        self._build_header()
        self._build_control_bar()
        self._build_readings_bar()
        self._build_main_area()

        self.root.bind('<KeyPress>',   self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._poll_status()
        self._blink_heartbeat()
        self._periodic_update()

    def _build_header(self):
        hdr = tk.Frame(self.root, bg="#0a1a35", pady=7)
        hdr.pack(fill=tk.X)

        # Heartbeat blink dot
        self.hb_canvas = tk.Canvas(hdr, width=10, height=10,
                                   bg="#0a1a35", highlightthickness=0)
        self.hb_dot = self.hb_canvas.create_oval(1, 1, 9, 9, fill=C_CYAN_DIM)
        self.hb_canvas.pack(side=tk.LEFT, padx=(14, 0))

        tk.Label(hdr, text="WHEEL TELEOP CONTROL INTERFACE",
                 font=("Courier New", 14, "bold"),
                 fg=C_CYAN, bg="#0a1a35").pack(side=tk.LEFT, padx=8)

        # Status indicator
        self.status_dot_c = tk.Canvas(hdr, width=12, height=12,
                                      bg="#0a1a35", highlightthickness=0)
        self.status_dot   = self.status_dot_c.create_oval(1, 1, 11, 11, fill="#333")
        self.status_dot_c.pack(side=tk.RIGHT, padx=6)
        self.status_lbl = tk.Label(hdr, text="● NO LINK",
                                   fg="#555", bg="#0a1a35",
                                   font=("Courier New", 9, "bold"))
        self.status_lbl.pack(side=tk.RIGHT, padx=4)

    def _build_control_bar(self):
        bar = tk.Frame(self.root, bg=C_PANEL, pady=8)
        bar.pack(fill=tk.X)

        # D-pad buttons
        pad = tk.Frame(bar, bg=C_PANEL)
        pad.pack(side=tk.LEFT, padx=14)

        _btn = dict(font=("Courier New", 9, "bold"), relief=tk.FLAT,
                    bd=0, width=7, height=2, cursor="hand2")
        tk.Button(pad, text="▲  FWD\n(W / ↑)",
                  bg="#073a22", fg=C_GREEN,
                  activebackground="#0a5030", activeforeground=C_GREEN,
                  command=self.set_forward, **_btn).grid(row=0, column=1, padx=2, pady=1)
        tk.Button(pad, text="◄ LEFT\n(A / ←)",
                  bg="#0a2848", fg="#60c4ff",
                  activebackground="#0e3a60", activeforeground="#80d8ff",
                  command=self.set_left, **_btn).grid(row=1, column=0, padx=2, pady=1)
        tk.Button(pad, text="▼  REV\n(S / ↓)",
                  bg="#2e0c1e", fg=C_RED,
                  activebackground="#401228", activeforeground=C_RED,
                  command=self.set_reverse, **_btn).grid(row=1, column=1, padx=2, pady=1)
        tk.Button(pad, text="RIGHT ►\n(D / →)",
                  bg="#0a2848", fg="#60c4ff",
                  activebackground="#0e3a60", activeforeground="#80d8ff",
                  command=self.set_right, **_btn).grid(row=1, column=2, padx=2, pady=1)
        tk.Button(pad, text="■  STOP  [ SPACE ]",
                  bg="#0c1a30", fg=C_WHITE,
                  activebackground="#101e3a", activeforeground=C_WHITE,
                  command=self.set_stop,
                  font=("Courier New", 9, "bold"), relief=tk.FLAT, bd=0,
                  width=22, height=1).grid(row=2, column=0, columnspan=3, padx=2, pady=2)

        # Speed slider
        spf = tk.Frame(bar, bg=C_PANEL)
        spf.pack(side=tk.LEFT, padx=18)
        tk.Label(spf, text="SPEED %", bg=C_PANEL, fg=C_CYAN,
                 font=("Courier New", 9, "bold")).pack()
        self.speed_scale = Scale(
            spf, from_=0, to=100, orient=HORIZONTAL, length=130, width=16,
            bg=C_PANEL, fg=C_WHITE, troughcolor=C_TROUGH,
            activebackground=C_CYAN, highlightthickness=0, bd=0,
            font=("Courier New", 8),
            command=lambda v: setattr(self, 'speed_pct', int(float(v))),
        )
        self.speed_scale.set(20)
        self.speed_scale.pack()

        # Checkboxes
        opt = tk.Frame(bar, bg=C_PANEL)
        opt.pack(side=tk.LEFT, padx=18)
        self.dead_man_var = tk.BooleanVar(value=True)
        tk.Checkbutton(opt, text="DEAD MAN  (key-release stops)",
                       variable=self.dead_man_var,
                       bg=C_PANEL, fg=C_CYAN, selectcolor=C_DARK,
                       activebackground=C_PANEL, activeforeground=C_CYAN,
                       font=("Courier New", 9),
                       command=lambda: setattr(self, 'dead_man',
                                               self.dead_man_var.get())).pack(anchor="w")

        # Steam Deck toggle
        sd_frame = tk.Frame(bar, bg=C_PANEL)
        sd_frame.pack(side=tk.RIGHT, padx=10)
        sd_unavail = not _PYGAME_OK
        self._sd_btn = tk.Button(
            sd_frame,
            text="STEAM DECK\n[OFF]",
            font=("Courier New", 9, "bold"),
            bg="#0c1a30", fg=C_GRAY,
            activebackground="#0e3a60", activeforeground=C_CYAN,
            relief=tk.FLAT, bd=0, width=11, height=3,
            cursor="hand2" if not sd_unavail else "arrow",
            state=tk.NORMAL if not sd_unavail else tk.DISABLED,
            command=self._toggle_steamdeck,
        )
        self._sd_btn.pack()
        if sd_unavail:
            tk.Label(sd_frame, text="(pygame missing)", bg=C_PANEL,
                     fg=C_GRAY, font=("Courier New", 7)).pack()

        # E-STOP
        tk.Button(bar, text="⚠\nE-STOP",
                  font=("Courier New", 16, "bold"),
                  bg="#2a0010", fg="#ff0040",
                  activebackground="#ff0040", activeforeground="#ffffff",
                  relief=tk.RAISED, bd=3, width=8, height=3,
                  cursor="hand2",
                  command=self.emergency_stop).pack(side=tk.RIGHT, padx=20)

    def _build_readings_bar(self):
        bar = tk.Frame(self.root, bg=C_DARK, pady=5)
        bar.pack(fill=tk.X, padx=4)

        tk.Label(bar, text="CURRENT DRAW", bg=C_DARK, fg=C_CYAN_DIM,
                 font=("Courier New", 8, "bold")).pack(side=tk.LEFT, padx=(10, 18))

        # motor 0=FL, 1=BL, 2=FR, 3=BR after FL/BL fix
        _corner = ["FL", "BL", "FR", "BR"]

        self._reading_wheel_labels = {}
        wf = tk.Frame(bar, bg=C_DARK)
        wf.pack(side=tk.LEFT, padx=10)
        tk.Label(wf, text="WHEELS (mA)", bg=C_DARK, fg=C_CYAN,
                 font=("Courier New", 8, "bold")).grid(row=0, column=0, columnspan=4, pady=(0, 2))
        for i in range(4):
            tk.Label(wf, text=f"ID:{i}", bg=C_DARK, fg=C_GRAY,
                     font=("Courier New", 7)).grid(row=1, column=i, padx=12)
            tk.Label(wf, text=_corner[i], bg=C_DARK, fg=C_CYAN_DIM,
                     font=("Courier New", 8, "bold")).grid(row=2, column=i, padx=12)
            lbl = tk.Label(wf, text="0", bg=C_DARK, fg=C_GRAY,
                           font=("Courier New", 11, "bold"), width=6)
            lbl.grid(row=3, column=i, padx=12)
            self._reading_wheel_labels[i] = lbl

        tk.Frame(bar, bg=C_CYAN_DIM, width=1).pack(side=tk.LEFT, fill=tk.Y, padx=14, pady=2)

        self._reading_leg_labels = {}
        lf = tk.Frame(bar, bg=C_DARK)
        lf.pack(side=tk.LEFT, padx=10)
        tk.Label(lf, text="LEGS (mA)", bg=C_DARK, fg=C_CYAN,
                 font=("Courier New", 8, "bold")).grid(row=0, column=0, columnspan=4, pady=(0, 2))
        for i in range(4):
            tk.Label(lf, text=f"ID:{i}", bg=C_DARK, fg=C_GRAY,
                     font=("Courier New", 7)).grid(row=1, column=i, padx=12)
            tk.Label(lf, text=_corner[i], bg=C_DARK, fg=C_CYAN_DIM,
                     font=("Courier New", 8, "bold")).grid(row=2, column=i, padx=12)
            lbl = tk.Label(lf, text="0", bg=C_DARK, fg=C_PURPLE_DIM,
                           font=("Courier New", 11, "bold"), width=6)
            lbl.grid(row=3, column=i, padx=12)
            self._reading_leg_labels[i] = lbl

        tk.Frame(bar, bg=C_CYAN_DIM, width=1).pack(side=tk.LEFT, fill=tk.Y, padx=14, pady=2)

        tf = tk.Frame(bar, bg=C_DARK)
        tf.pack(side=tk.LEFT, padx=14)
        tk.Label(tf, text="TOTAL", bg=C_DARK, fg=C_CYAN,
                 font=("Courier New", 8, "bold")).pack()
        self._total_current_lbl = tk.Label(tf, text="0 mA", bg=C_DARK, fg=C_WHITE,
                                           font=("Courier New", 13, "bold"), width=9)
        self._total_current_lbl.pack(pady=4)

    def _build_main_area(self):
        main = tk.Frame(self.root, bg=C_BG)
        main.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # 3-column layout: corner panels | diagram | corner panels
        main.grid_columnconfigure(0, weight=0, minsize=280)
        main.grid_columnconfigure(1, weight=1)
        main.grid_columnconfigure(2, weight=0, minsize=280)
        main.grid_rowconfigure(0, weight=1)
        main.grid_rowconfigure(1, weight=1)

        for wid, lid, row, col, title in (
            (0, 0, 0, 0, "FL — FRONT LEFT  (ID:0)"),
            (1, 1, 1, 0, "BL — BACK LEFT   (ID:1)"),
            (2, 2, 0, 2, "FR — FRONT RIGHT (ID:2)"),
            (3, 3, 1, 2, "BR — BACK RIGHT  (ID:3)"),
        ):
            self._build_corner_panel(main, wid, lid, row, col, title)

        # Diagram — spans both rows in center column
        diag = _sci_frame(main, "ROBOT DIAGRAM")
        diag.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=4, pady=4)
        self.canvas = tk.Canvas(diag, bg=C_DARK, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.canvas.bind('<Configure>', lambda _e: self.draw_robot_diagram())

    def _build_corner_panel(self, parent, wid, lid, row, col, title):
        outer = _sci_frame(parent, title)
        outer.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
        outer.grid_columnconfigure(0, weight=1)
        outer.grid_columnconfigure(1, weight=1)
        outer.grid_rowconfigure(0, weight=1)

        # Wheel slider (left half)
        wf = tk.Frame(outer, bg=C_PANEL, padx=5, pady=4)
        wf.grid(row=0, column=0, sticky="nsew")
        tk.Label(wf, text="WHEEL", bg=C_PANEL, fg=C_CYAN,
                 font=("Courier New", 8, "bold")).pack()
        ws_border = tk.Frame(wf, bg=C_CYAN_DIM)
        ws_border.pack(expand=True, fill=tk.Y, pady=2)
        ws = Scale(ws_border, from_=50, to=-50, orient=VERTICAL, length=300, width=28,
                   bg=C_PANEL, fg=C_WHITE, troughcolor=C_TROUGH,
                   activebackground=C_GREEN, highlightthickness=0, bd=0,
                   showvalue=0,
                   command=lambda v, i=wid: self.update_wheel(i, v))
        ws.set(0)
        ws.pack(fill=tk.Y, expand=True, padx=2, pady=2)
        wl = tk.Label(wf, text="0", bg=C_PANEL, fg=C_GREEN,
                      font=("Courier New", 11, "bold"))
        wl.pack()
        self.wheel_sliders[wid] = ws
        self.wheel_labels[wid]  = wl

        # Leg slider (right half)
        lf_frame = tk.Frame(outer, bg=C_PANEL, padx=5, pady=4)
        lf_frame.grid(row=0, column=1, sticky="nsew")
        tk.Label(lf_frame, text="LEG °", bg=C_PANEL, fg=C_CYAN,
                 font=("Courier New", 8, "bold")).pack()
        ls_border = tk.Frame(lf_frame, bg=C_CYAN_DIM)
        ls_border.pack(expand=True, fill=tk.Y, pady=2)
        ls = Scale(ls_border, from_=180, to=0, orient=VERTICAL, length=300, width=28,
                   bg=C_PANEL, fg=C_WHITE, troughcolor=C_TROUGH,
                   activebackground=C_PURPLE, highlightthickness=0, bd=0,
                   showvalue=0,
                   command=lambda v, i=lid: self.update_leg(i, v))
        ls.set(0)
        ls.pack(fill=tk.Y, expand=True, padx=2, pady=2)
        ll = tk.Label(lf_frame, text="0°", bg=C_PANEL, fg=C_PURPLE,
                      font=("Courier New", 11, "bold"))
        ll.pack()
        self.leg_sliders[lid] = ls
        self.leg_labels[lid]  = ll

        return outer

    # -----------------------------------------------------------------------
    # Diagram rendering
    # -----------------------------------------------------------------------

    def _rebuild_diagram(self, w, h):
        """Full canvas rebuild — called only on first draw or window resize."""
        c = self.canvas
        c.delete("all")
        cx, cy = w // 2, h // 2

        # Background grid
        for x in range(0, w, 40):
            c.create_line(x, 0, x, h, fill="#102840", width=1)
        for y in range(0, h, 40):
            c.create_line(0, y, w, y, fill="#102840", width=1)

        # HUD corner brackets
        for bx, by, sx, sy in ((8,8,1,1),(w-8,8,-1,1),(8,h-8,1,-1),(w-8,h-8,-1,-1)):
            c.create_line(bx, by, bx+sx*20, by, fill=C_CYAN, width=2)
            c.create_line(bx, by, bx, by+sy*20, fill=C_CYAN, width=2)

        # Forward label
        c.create_text(cx, 14, text="▲  FORWARD", fill=C_CYAN_DIM,
                      font=("Courier New", 8))

        bw, bh = 50, 70

        # Robot body (static)
        c.create_rectangle(cx-bw-3, cy-bh-3, cx+bw+3, cy+bh+3,
                            outline=C_CYAN_DIM, fill="", width=1)
        c.create_rectangle(cx-bw, cy-bh, cx+bw, cy+bh,
                            fill="#0a2240", outline=C_CYAN, width=2)
        c.create_line(cx, cy-bh, cx, cy+bh, fill=C_CYAN_DIM, width=1)
        c.create_line(cx-bw, cy, cx+bw, cy, fill=C_CYAN_DIM, width=1)
        c.create_text(cx, cy - 8, text="ROBOT", fill=C_CYAN,
                      font=("Courier New", 9, "bold"))
        c.create_text(cx, cy + 8, text="◊", fill=C_CYAN_DIM,
                      font=("Courier New", 8))

        # Wheel positions: 0=FL(top-left), 1=BL(bottom-left), 2=FR(top-right), 3=BR(bottom-right)
        wheel_pos = {
            0: (cx - 90, cy - 85),
            1: (cx - 90, cy + 85),
            2: (cx + 90, cy - 85),
            3: (cx + 90, cy + 85),
        }

        # Pre-create mutable wheel items
        for i, (wx, wy) in wheel_pos.items():
            self._diag_glow[i] = c.create_oval(
                wx-22, wy-22, wx+22, wy+22, outline=C_GRAY, fill="", width=5)
            self._diag_disc[i] = c.create_oval(
                wx-14, wy-14, wx+14, wy+14, fill=C_DARK, outline=C_GRAY, width=2)
            self._diag_dot[i]  = c.create_oval(
                wx-3, wy-3, wx+3, wy+3, fill=C_GRAY, outline="")
            self._diag_idx[i]  = c.create_text(
                wx, wy-22, text=str(i), fill=C_GRAY,
                font=("Courier New", 8, "bold"))
            self._diag_spd[i]  = c.create_text(
                wx, wy+22, text="0", fill=C_WHITE,
                font=("Courier New", 7))

        self._last_diag_spd = None   # force color update on next pass

    def draw_robot_diagram(self):
        c = self.canvas
        w, h = c.winfo_width(), c.winfo_height()
        if w < 20 or h < 20:
            return

        if (w, h) != self._diagram_size:
            self._diagram_size = (w, h)
            self._rebuild_diagram(w, h)

        with self.lock:
            speeds = list(self.wheel_speed)

        if speeds == self._last_diag_spd:
            return
        self._last_diag_spd = speeds[:]

        for i in range(4):
            spd = speeds[i]
            if spd > 0:
                col, dim = C_GREEN, C_GREEN_DIM
            elif spd < 0:
                col, dim = C_RED, C_RED_DIM
            else:
                col, dim = C_GRAY, "#152e48"
            c.itemconfig(self._diag_glow[i], outline=dim)
            c.itemconfig(self._diag_disc[i], outline=col)
            c.itemconfig(self._diag_dot[i],  fill=col)
            c.itemconfig(self._diag_idx[i],  fill=col)
            c.itemconfig(self._diag_spd[i],  text=str(spd))

    # -----------------------------------------------------------------------
    # Slider callbacks (user interaction)
    # -----------------------------------------------------------------------

    def update_wheel(self, i, v):
        if self._syncing:
            return
        val = int(float(v))
        with self.lock:
            self.wheel_speed[i] = val
        color = C_GREEN if val > 0 else (C_RED if val < 0 else C_GRAY)
        self.wheel_labels[i].config(text=str(val), fg=color)

    def update_leg(self, i, v):
        val = int(float(v))
        with self.lock:
            self.leg_angles[i] = val
        self.leg_labels[i].config(text=f"{val}°")

    # -----------------------------------------------------------------------
    # Periodic loops
    # -----------------------------------------------------------------------

    def _periodic_update(self):
        """Sync wheel sliders + diagram at 20 Hz, decoupled from key-repeat."""
        with self.lock:
            speeds = list(self.wheel_speed)
            wcurr  = list(self.wheel_currents)
            lcurr  = list(self.leg_currents)

        if speeds != self._last_ui_spd:
            self._last_ui_spd = speeds[:]
            self._syncing = True
            for i in range(4):
                self.wheel_sliders[i].set(speeds[i])
                color = C_GREEN if speeds[i] > 0 else (C_RED if speeds[i] < 0 else C_GRAY)
                self.wheel_labels[i].config(text=str(speeds[i]), fg=color)
            self._syncing = False

        total = 0
        for i in range(4):
            w = wcurr[i]
            wc = C_GREEN if w > 20 else (C_RED if w < -20 else C_GRAY)
            self._reading_wheel_labels[i].config(text=str(w), fg=wc)
            total += abs(w)

            l = lcurr[i]
            lc = C_PURPLE if abs(l) > 20 else C_PURPLE_DIM
            self._reading_leg_labels[i].config(text=str(l), fg=lc)
            total += abs(l)

        total_txt = f"{total / 1000:.2f} A" if total >= 1000 else f"{total} mA"
        total_color = C_RED if total > 5000 else (C_CYAN if total > 500 else C_WHITE)
        self._total_current_lbl.config(text=total_txt, fg=total_color)

        self.draw_robot_diagram()
        self.root.after(50, self._periodic_update)

    def _poll_status(self):
        count = self.pub.get_subscription_count()
        if count > 0:
            self.status_dot_c.itemconfig(self.status_dot, fill="#00ff44")
            self.status_lbl.config(text=f"● LINKED  [{count}]", fg="#00ff44")
        else:
            self.status_dot_c.itemconfig(self.status_dot, fill=C_RED)
            self.status_lbl.config(text="● NO LINK", fg=C_RED)
        self.root.after(1000, self._poll_status)

    def _blink_heartbeat(self):
        self._hb_state = not self._hb_state
        self.hb_canvas.itemconfig(self.hb_dot,
                                  fill=C_CYAN if self._hb_state else C_CYAN_DIM)
        self.root.after(600, self._blink_heartbeat)

    # -----------------------------------------------------------------------
    # Movement commands (update state only — UI sync happens in _periodic_update)
    # -----------------------------------------------------------------------

    def set_forward(self):
        spd = int(self.wheel_max * self.speed_pct / 100)
        with self.lock:
            self.wheel_speed = [spd] * 4

    def set_reverse(self):
        spd = int(self.wheel_max * self.speed_pct / 100)
        with self.lock:
            self.wheel_speed = [-spd] * 4

    def set_left(self):
        spd = int(self.wheel_max * self.speed_pct / 100)
        with self.lock:
            self.wheel_speed = [-spd, -spd, spd, spd]

    def set_right(self):
        spd = int(self.wheel_max * self.speed_pct / 100)
        with self.lock:
            self.wheel_speed = [spd, spd, -spd, -spd]

    def set_stop(self):
        with self.lock:
            self.wheel_speed = [0] * 4

    def emergency_stop(self):
        with self.lock:
            self.wheel_speed = [0] * 4
            self.leg_angles  = [0] * 4
        self._held_keys.clear()

        zero = Int32MultiArray()
        zero.data = [0] * 8
        self.pub.publish(zero)

        estop = Bool()
        estop.data = True
        self.estop_pub.publish(estop)

        self._syncing = True
        for i in range(4):
            self.leg_sliders[i].set(0)
            self.leg_labels[i].config(text="0°")
        self._syncing = False

    # -----------------------------------------------------------------------
    # Key handling with held-key tracking
    # -----------------------------------------------------------------------

    def on_key_press(self, e):
        k = e.keysym.lower()
        if k in _MOVE:
            if k not in self._held_keys:
                self._held_keys.append(k)
            getattr(self, f'set_{_MOVE[k]}')()
        elif k == 'space':
            self._held_keys.clear()
            self.set_stop()
        elif k in ('escape', 'x'):
            self._on_close()

    def on_key_release(self, e):
        k = e.keysym.lower()
        if k in self._held_keys:
            self._held_keys.remove(k)

        if not self.dead_man:
            return

        if not self._held_keys:
            self.set_stop()
        else:
            getattr(self, f'set_{_MOVE[self._held_keys[-1]]}')()

    # -----------------------------------------------------------------------
    # Steam Deck mode
    # -----------------------------------------------------------------------

    def _toggle_steamdeck(self):
        if self._sd_active:
            self._sd_active = False
            self._sd_btn.config(text="STEAM DECK\n[OFF]", bg="#0c1a30", fg=C_GRAY)
        else:
            self._sd_active = True
            self._sd_btn.config(text="STEAM DECK\n[ON]", bg="#073a22", fg=C_GREEN)
            t = threading.Thread(target=self._steamdeck_loop, daemon=True)
            t.start()

    def _steamdeck_loop(self):
        """Background thread: poll Steam Deck gamepad at 20 Hz and drive robot."""
        # Use dummy video/audio drivers so pygame doesn't try to open a display
        # window — joystick input works fine without a real video subsystem.
        os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
        os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
        pygame.init()
        pygame.joystick.init()

        joy = None
        if pygame.joystick.get_count() == 0:
            self.get_logger().warn('Steam Deck: no joystick detected — mode disabled')
            self._sd_active = False
            self.root.after(0, lambda: self._sd_btn.config(
                text="STEAM DECK\n[OFF]", bg="#0c1a30", fg=C_GRAY))
            pygame.quit()
            return

        joy = pygame.joystick.Joystick(0)
        joy.init()
        self.get_logger().info(f'Steam Deck: connected to "{joy.get_name()}"')

        while self._sd_active and rclpy.ok():
            pygame.event.pump()

            # ------------------------------------------------------------------
            # L2 (left trigger) → hard reset: zero all wheels and legs
            # Axis value: -1.0 at rest, +1.0 fully pressed
            # ------------------------------------------------------------------
            if joy.get_axis(SD_AXIS_L2) > SD_L2_THRESH:
                with self.lock:
                    self.wheel_speed = [0, 0, 0, 0]
                    self.leg_angles  = [0, 0, 0, 0]
                time.sleep(0.05)
                continue

            # ------------------------------------------------------------------
            # Right stick Y → throttle (push up = faster)
            # -1.0 fully forward, 0 neutral, +1.0 fully back; clamped to [0, 1]
            # ------------------------------------------------------------------
            ry = joy.get_axis(SD_AXIS_RY)
            throttle = max(0.0, -ry)
            if throttle < SD_DEADZONE:
                throttle = 0.0

            # ------------------------------------------------------------------
            # Left stick → arcade drive
            # Y: forward/backward   X: turn left/right
            # ------------------------------------------------------------------
            ly = joy.get_axis(SD_AXIS_LY)
            lx = joy.get_axis(SD_AXIS_LX)
            if abs(ly) < SD_DEADZONE:
                ly = 0.0
            if abs(lx) < SD_DEADZONE:
                lx = 0.0

            effective_spd = throttle * self.wheel_max
            forward = -ly * effective_spd   # stick up (-) → forward (+)
            turn    =  lx * effective_spd   # stick right (+) → right turn

            # FL=0, BL=1, FR=2, BR=3
            def _clamp(v):
                return int(max(-self.wheel_max, min(self.wheel_max, v)))

            ws = [
                _clamp(forward + turn),   # FL
                _clamp(forward + turn),   # BL
                _clamp(forward - turn),   # FR
                _clamp(forward - turn),   # BR
            ]

            # ------------------------------------------------------------------
            # D-pad → extend individual legs (hold = continuous ~60°/s)
            # UP → FL (0)   LEFT → BL (1)   RIGHT → FR (2)   DOWN → BR (3)
            # ------------------------------------------------------------------
            with self.lock:
                angles = list(self.leg_angles)

            hx, hy = joy.get_hat(0)   # (x, y); y=+1=up
            if hy == 1:   angles[0] = min(180, angles[0] + SD_LEG_STEP)
            if hx == -1:  angles[1] = min(180, angles[1] + SD_LEG_STEP)
            if hx == 1:   angles[2] = min(180, angles[2] + SD_LEG_STEP)
            if hy == -1:  angles[3] = min(180, angles[3] + SD_LEG_STEP)

            # ------------------------------------------------------------------
            # L1 → extend all legs   R1 → retract all legs
            # ------------------------------------------------------------------
            if joy.get_button(SD_BTN_L1):
                angles = [min(180, a + SD_LEG_STEP) for a in angles]
            if joy.get_button(SD_BTN_R1):
                angles = [max(0,   a - SD_LEG_STEP) for a in angles]

            # ------------------------------------------------------------------
            # Face buttons → snap individual leg to 0°
            # Y → FL (0)   X → BL (1)   B → FR (2)   A → BR (3)
            # ------------------------------------------------------------------
            if joy.get_button(SD_BTN_Y):  angles[0] = 0
            if joy.get_button(SD_BTN_X):  angles[1] = 0
            if joy.get_button(SD_BTN_B):  angles[2] = 0
            if joy.get_button(SD_BTN_A):  angles[3] = 0

            # ------------------------------------------------------------------
            # Back paddles → drive individual wheel forward at slider speed
            # L4 → FL (0)   L5 → BL (1)   R4 → FR (2)   R5 → BR (3)
            # ------------------------------------------------------------------
            paddle_spd = max(1, int(self.wheel_max * self.speed_pct / 100.0))
            if joy.get_button(SD_BTN_L4):  ws[0] = paddle_spd
            if joy.get_button(SD_BTN_L5):  ws[1] = paddle_spd
            if joy.get_button(SD_BTN_R4):  ws[2] = paddle_spd
            if joy.get_button(SD_BTN_R5):  ws[3] = paddle_spd

            with self.lock:
                self.wheel_speed = ws
                self.leg_angles  = angles

            time.sleep(0.05)   # 20 Hz

        # Zero wheels when mode exits
        with self.lock:
            self.wheel_speed = [0, 0, 0, 0]

        if joy is not None:
            joy.quit()
        pygame.quit()
        self.get_logger().info('Steam Deck: disconnected')

    def _on_close(self):
        """Send estop + zero all actuators, then exit."""
        self._sd_active = False
        self.emergency_stop()
        self.root.after(150, self.root.destroy)

    # -----------------------------------------------------------------------
    # Options
    # -----------------------------------------------------------------------

    # -----------------------------------------------------------------------
    # Current reading callbacks
    # -----------------------------------------------------------------------

    def _wheel_currents_cb(self, msg):
        with self.lock:
            self.wheel_currents = list(msg.data[:4])

    def _leg_currents_cb(self, msg):
        with self.lock:
            self.leg_currents = list(msg.data[:4])

    # -----------------------------------------------------------------------
    # ROS publish loop (background thread)
    # -----------------------------------------------------------------------

    def publish_loop(self):
        dt = 1.0 / 20
        while rclpy.ok():
            msg = Int32MultiArray()
            with self.lock:
                msg.data = self.wheel_speed + self.leg_angles
            self.pub.publish(msg)
            time.sleep(dt)

    def run_gui(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = GuiTeleop()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    t = threading.Thread(target=node.publish_loop, daemon=True)
    t.start()

    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
