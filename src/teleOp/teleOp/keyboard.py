import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

import tkinter as tk
from tkinter import Scale, VERTICAL, HORIZONTAL
import threading
import time
import math

# ---------------------------------------------------------------------------
# Iron Man HUD color palette
# ---------------------------------------------------------------------------
C_BG         = "#081428"   # dark navy (lighter)
C_PANEL      = "#0e2040"   # panel - rich dark blue
C_DARK       = "#040c1c"   # darkest elements / diagram bg
C_CYAN       = "#28d4ff"   # primary accent - JARVIS blue
C_CYAN_DIM   = "#1a5070"   # dim borders / grid
C_GREEN      = "#00eeb0"   # forward / positive - teal-green
C_GREEN_DIM  = "#00503e"   # dim green glow
C_RED        = "#ff3a55"   # reverse / danger
C_RED_DIM    = "#4a001a"   # dim red glow
C_PURPLE     = "#6699ff"   # leg indicators - blue-purple
C_PURPLE_DIM = "#183070"   # dim blue-purple glow
C_WHITE      = "#d8f0ff"   # primary text - bright cool blue-white
C_GRAY       = "#3878c0"   # muted / stopped state - bright medium blue
C_TROUGH     = "#0c2848"   # slider trough - deep blue

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

        self.wheel_speed = [0, 0, 0, 0]
        self.leg_angles  = [0, 0, 0, 0]
        self.lock        = threading.Lock()

        self.wheel_max  = 50
        self.speed_pct  = 100
        self.dead_man   = True
        self._syncing   = False
        self._held_keys = []        # ordered list of currently-held movement keys
        self._compact_panels = []   # corner panels hidden in compact mode
        self._hb_state  = False     # heartbeat blink state

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
        self._build_main_area()

        self.root.bind('<KeyPress>',   self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

        self._poll_status()
        self._blink_heartbeat()
        self._periodic_update()

    def _build_header(self):
        hdr = tk.Frame(self.root, bg="#081428", pady=7)
        hdr.pack(fill=tk.X)

        # Heartbeat blink dot
        self.hb_canvas = tk.Canvas(hdr, width=10, height=10,
                                   bg="#081428", highlightthickness=0)
        self.hb_dot = self.hb_canvas.create_oval(1, 1, 9, 9, fill=C_CYAN_DIM)
        self.hb_canvas.pack(side=tk.LEFT, padx=(14, 0))

        tk.Label(hdr, text="WHEEL TELEOP CONTROL INTERFACE",
                 font=("Courier New", 14, "bold"),
                 fg=C_CYAN, bg="#081428").pack(side=tk.LEFT, padx=8)

        # Status indicator
        self.status_dot_c = tk.Canvas(hdr, width=12, height=12,
                                      bg="#081428", highlightthickness=0)
        self.status_dot   = self.status_dot_c.create_oval(1, 1, 11, 11, fill="#333")
        self.status_dot_c.pack(side=tk.RIGHT, padx=6)
        self.status_lbl = tk.Label(hdr, text="● NO LINK",
                                   fg="#555", bg="#081428",
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
        self.speed_scale.set(100)
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
        self.compact_var = tk.BooleanVar(value=False)
        tk.Checkbutton(opt, text="COMPACT MODE",
                       variable=self.compact_var,
                       bg=C_PANEL, fg=C_CYAN, selectcolor=C_DARK,
                       activebackground=C_PANEL, activeforeground=C_CYAN,
                       font=("Courier New", 9),
                       command=self._toggle_compact).pack(anchor="w")

        # E-STOP
        tk.Button(bar, text="⚠\nE-STOP",
                  font=("Courier New", 16, "bold"),
                  bg="#2a0010", fg="#ff0040",
                  activebackground="#ff0040", activeforeground="#ffffff",
                  relief=tk.RAISED, bd=3, width=8, height=3,
                  cursor="hand2",
                  command=self.emergency_stop).pack(side=tk.RIGHT, padx=20)

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
            (1, 1, 0, 0, "FL — FRONT LEFT"),
            (0, 0, 1, 0, "BL — BACK LEFT"),
            (2, 2, 0, 2, "FR — FRONT RIGHT"),
            (3, 3, 1, 2, "BR — BACK RIGHT"),
        ):
            p = self._build_corner_panel(main, wid, lid, row, col, title)
            self._compact_panels.append(p)

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
                   font=("Courier New", 7),
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
                   font=("Courier New", 7),
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
            c.create_line(x, 0, x, h, fill="#0e2438", width=1)
        for y in range(0, h, 40):
            c.create_line(0, y, w, y, fill="#0e2438", width=1)

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
                            fill="#071a2e", outline=C_CYAN, width=2)
        c.create_line(cx, cy-bh, cx, cy+bh, fill=C_CYAN_DIM, width=1)
        c.create_line(cx-bw, cy, cx+bw, cy, fill=C_CYAN_DIM, width=1)
        c.create_text(cx, cy - 8, text="ROBOT", fill=C_CYAN,
                      font=("Courier New", 9, "bold"))
        c.create_text(cx, cy + 8, text="◊", fill=C_CYAN_DIM,
                      font=("Courier New", 8))

        # Wheel positions
        wheel_pos = {
            0: (cx - 90, cy + 85),
            1: (cx - 90, cy - 85),
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
                col, dim = C_GRAY, "#12283e"
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

        if speeds != self._last_ui_spd:
            self._last_ui_spd = speeds[:]
            self._syncing = True
            for i in range(4):
                self.wheel_sliders[i].set(speeds[i])
                color = C_GREEN if speeds[i] > 0 else (C_RED if speeds[i] < 0 else C_GRAY)
                self.wheel_labels[i].config(text=str(speeds[i]), fg=color)
            self._syncing = False

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
            self.wheel_speed = [-spd, spd, spd, -spd]

    def set_right(self):
        spd = int(self.wheel_max * self.speed_pct / 100)
        with self.lock:
            self.wheel_speed = [spd, -spd, -spd, spd]

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
    # Options
    # -----------------------------------------------------------------------

    def _toggle_compact(self):
        if self.compact_var.get():
            for p in self._compact_panels:
                p.grid_remove()
            self.root.geometry("600x760")
        else:
            for p in self._compact_panels:
                p.grid()
            self.root.geometry("1440x920")

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
