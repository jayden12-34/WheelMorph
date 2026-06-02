import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

import tkinter as tk
from tkinter import Scale, VERTICAL, HORIZONTAL
import threading
import time
import math

# ---------------------------------------------------------------------------
# Sci-fi color palette
# ---------------------------------------------------------------------------
C_BG         = "#080c14"   # root background
C_PANEL      = "#0d1428"   # widget panel background
C_DARK       = "#050810"   # darkest elements / diagram bg
C_CYAN       = "#00d4ff"   # primary neon accent
C_CYAN_DIM   = "#003a4a"   # dimmed cyan (borders, grid)
C_GREEN      = "#00ff7f"   # forward / positive speed
C_GREEN_DIM  = "#003322"   # dim green glow
C_RED        = "#ff1a3c"   # reverse / danger
C_RED_DIM    = "#3a000f"   # dim red glow
C_PURPLE     = "#9060ff"   # leg indicators
C_PURPLE_DIM = "#1e0840"   # dim purple glow
C_WHITE      = "#c8e8ff"   # primary text
C_GRAY       = "#3a5070"   # muted / stopped state
C_TROUGH     = "#0e1928"   # slider trough

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
        hdr = tk.Frame(self.root, bg="#020509", pady=7)
        hdr.pack(fill=tk.X)

        # Heartbeat blink dot
        self.hb_canvas = tk.Canvas(hdr, width=10, height=10,
                                   bg="#020509", highlightthickness=0)
        self.hb_dot = self.hb_canvas.create_oval(1, 1, 9, 9, fill=C_CYAN_DIM)
        self.hb_canvas.pack(side=tk.LEFT, padx=(14, 0))

        tk.Label(hdr, text="WHEEL TELEOP CONTROL INTERFACE",
                 font=("Courier New", 14, "bold"),
                 fg=C_CYAN, bg="#020509").pack(side=tk.LEFT, padx=8)

        # Status indicator
        self.status_dot_c = tk.Canvas(hdr, width=12, height=12,
                                      bg="#020509", highlightthickness=0)
        self.status_dot   = self.status_dot_c.create_oval(1, 1, 11, 11, fill="#333")
        self.status_dot_c.pack(side=tk.RIGHT, padx=6)
        self.status_lbl = tk.Label(hdr, text="● NO LINK",
                                   fg="#555", bg="#020509",
                                   font=("Courier New", 9, "bold"))
        self.status_lbl.pack(side=tk.RIGHT, padx=4)

    def _build_control_bar(self):
        bar = tk.Frame(self.root, bg="#090e1c", pady=8)
        bar.pack(fill=tk.X)

        # D-pad buttons
        pad = tk.Frame(bar, bg="#090e1c")
        pad.pack(side=tk.LEFT, padx=14)

        _btn = dict(font=("Courier New", 9, "bold"), relief=tk.FLAT,
                    bd=0, width=7, height=2, cursor="hand2")
        tk.Button(pad, text="▲  FWD\n(W / ↑)",
                  bg="#0a2010", fg=C_GREEN,
                  activebackground="#163a20", activeforeground=C_GREEN,
                  command=self.set_forward, **_btn).grid(row=0, column=1, padx=2, pady=1)
        tk.Button(pad, text="◄ LEFT\n(A / ←)",
                  bg="#252510", fg="#ffff44",
                  activebackground="#3a3a18", activeforeground="#ffff44",
                  command=self.set_left, **_btn).grid(row=1, column=0, padx=2, pady=1)
        tk.Button(pad, text="▼  REV\n(S / ↓)",
                  bg="#200a10", fg=C_RED,
                  activebackground="#381420", activeforeground=C_RED,
                  command=self.set_reverse, **_btn).grid(row=1, column=1, padx=2, pady=1)
        tk.Button(pad, text="RIGHT ►\n(D / →)",
                  bg="#252510", fg="#ffff44",
                  activebackground="#3a3a18", activeforeground="#ffff44",
                  command=self.set_right, **_btn).grid(row=1, column=2, padx=2, pady=1)
        tk.Button(pad, text="■  STOP  [ SPACE ]",
                  bg="#151515", fg=C_WHITE,
                  activebackground="#222", activeforeground=C_WHITE,
                  command=self.set_stop,
                  font=("Courier New", 9, "bold"), relief=tk.FLAT, bd=0,
                  width=22, height=1).grid(row=2, column=0, columnspan=3, padx=2, pady=2)

        # Speed slider
        spf = tk.Frame(bar, bg="#090e1c")
        spf.pack(side=tk.LEFT, padx=18)
        tk.Label(spf, text="SPEED %", bg="#090e1c", fg=C_CYAN,
                 font=("Courier New", 9, "bold")).pack()
        self.speed_scale = Scale(
            spf, from_=0, to=100, orient=HORIZONTAL, length=130, width=16,
            bg="#090e1c", fg=C_WHITE, troughcolor=C_TROUGH,
            activebackground=C_CYAN, highlightthickness=0, bd=0,
            font=("Courier New", 8),
            command=lambda v: setattr(self, 'speed_pct', int(float(v))),
        )
        self.speed_scale.set(100)
        self.speed_scale.pack()

        # Checkboxes
        opt = tk.Frame(bar, bg="#090e1c")
        opt.pack(side=tk.LEFT, padx=18)
        self.dead_man_var = tk.BooleanVar(value=True)
        tk.Checkbutton(opt, text="DEAD MAN  (key-release stops)",
                       variable=self.dead_man_var,
                       bg="#090e1c", fg=C_CYAN, selectcolor=C_DARK,
                       activebackground="#090e1c", activeforeground=C_CYAN,
                       font=("Courier New", 9),
                       command=lambda: setattr(self, 'dead_man',
                                               self.dead_man_var.get())).pack(anchor="w")
        self.compact_var = tk.BooleanVar(value=False)
        tk.Checkbutton(opt, text="COMPACT MODE",
                       variable=self.compact_var,
                       bg="#090e1c", fg=C_CYAN, selectcolor=C_DARK,
                       activebackground="#090e1c", activeforeground=C_CYAN,
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
        ws_border.pack(fill=tk.BOTH, expand=True, pady=2)
        ws = Scale(ws_border, from_=50, to=-50, orient=VERTICAL, length=300, width=28,
                   bg=C_PANEL, fg=C_WHITE, troughcolor=C_TROUGH,
                   activebackground=C_GREEN, highlightthickness=0, bd=0,
                   font=("Courier New", 7),
                   command=lambda v, i=wid: self.update_wheel(i, v))
        ws.set(0)
        ws.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
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
        ls_border = tk.Frame(lf_frame, bg=C_PURPLE_DIM)
        ls_border.pack(fill=tk.BOTH, expand=True, pady=2)
        ls = Scale(ls_border, from_=180, to=0, orient=VERTICAL, length=300, width=28,
                   bg=C_PANEL, fg=C_WHITE, troughcolor=C_TROUGH,
                   activebackground=C_PURPLE, highlightthickness=0, bd=0,
                   font=("Courier New", 7),
                   command=lambda v, i=lid: self.update_leg(i, v))
        ls.set(0)
        ls.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        ll = tk.Label(lf_frame, text="0°", bg=C_PANEL, fg=C_PURPLE,
                      font=("Courier New", 11, "bold"))
        ll.pack()
        self.leg_sliders[lid] = ls
        self.leg_labels[lid]  = ll

        return outer

    # -----------------------------------------------------------------------
    # Diagram rendering
    # -----------------------------------------------------------------------

    def draw_robot_diagram(self):
        c = self.canvas
        w, h = c.winfo_width(), c.winfo_height()
        if w < 20 or h < 20:
            return

        c.delete("all")
        cx, cy = w // 2, h // 2

        # Background grid
        for x in range(0, w, 40):
            c.create_line(x, 0, x, h, fill="#0c1420", width=1)
        for y in range(0, h, 40):
            c.create_line(0, y, w, y, fill="#0c1420", width=1)

        # HUD corner brackets
        for bx, by, sx, sy in ((8,8,1,1),(w-8,8,-1,1),(8,h-8,1,-1),(w-8,h-8,-1,-1)):
            c.create_line(bx, by, bx+sx*20, by, fill=C_CYAN_DIM, width=2)
            c.create_line(bx, by, bx, by+sy*20, fill=C_CYAN_DIM, width=2)

        # Forward label
        c.create_text(cx, 14, text="▲  FORWARD", fill=C_CYAN_DIM,
                      font=("Courier New", 8))

        with self.lock:
            speeds = list(self.wheel_speed)
            angles = list(self.leg_angles)

        bw, bh = 50, 70   # robot body half-extents

        # Wheel positions
        wheel_pos = {
            0: (cx - 90, cy + 85),   # BL
            1: (cx - 90, cy - 85),   # FL
            2: (cx + 90, cy - 85),   # FR
            3: (cx + 90, cy + 85),   # BR
        }

        # Leg attachment corners on the robot body rectangle
        leg_attach = {
            0: (cx - bw, cy + bh),
            1: (cx - bw, cy - bh),
            2: (cx + bw, cy - bh),
            3: (cx + bw, cy + bh),
        }

        # ---- Draw legs (behind body) ----
        for i in range(4):
            ax, ay = leg_attach[i]
            length = 50
            # Vertical rotation: 0°=pointing down, 90°=horizontal outward, 180°=pointing up
            rad = math.radians(angles[i])
            horiz_dir = -1 if ax < cx else 1
            ex = int(ax + horiz_dir * math.sin(rad) * length)
            ey = int(ay + math.cos(rad) * length)

            # Glow (wide dim)
            c.create_line(ax, ay, ex, ey, fill=C_PURPLE_DIM, width=7,
                          capstyle=tk.ROUND)
            # Core (thin bright)
            c.create_line(ax, ay, ex, ey, fill=C_PURPLE, width=2,
                          capstyle=tk.ROUND)
            # End-effector dot
            c.create_oval(ex-4, ey-4, ex+4, ey+4, fill=C_PURPLE, outline="")
            # Angle readout
            anchor = "w" if ex >= ax else "e"
            ox = 7 if ex >= ax else -7
            c.create_text(ex + ox, ey, text=f"{angles[i]}°",
                          fill="#604a80", font=("Courier New", 7), anchor=anchor)

        # ---- Robot body (drawn over legs) ----
        # Outer glow rect
        c.create_rectangle(cx-bw-3, cy-bh-3, cx+bw+3, cy+bh+3,
                            outline=C_CYAN_DIM, fill="", width=1)
        # Body
        c.create_rectangle(cx-bw, cy-bh, cx+bw, cy+bh,
                            fill="#0a1830", outline=C_CYAN, width=2)
        c.create_text(cx, cy - 8, text="ROBOT", fill=C_CYAN,
                      font=("Courier New", 9, "bold"))
        c.create_text(cx, cy + 8, text="◊", fill=C_CYAN_DIM,
                      font=("Courier New", 8))

        # ---- Wheels ----
        for i, (wx, wy) in wheel_pos.items():
            spd = speeds[i]
            if spd > 0:
                col, dim = C_GREEN,  C_GREEN_DIM
            elif spd < 0:
                col, dim = C_RED,    C_RED_DIM
            else:
                col, dim = C_GRAY,   "#10181e"

            # Outer glow ring
            c.create_oval(wx-24, wy-24, wx+24, wy+24,
                          outline=dim, fill="", width=5)
            # Wheel disc
            c.create_oval(wx-16, wy-16, wx+16, wy+16,
                          fill=C_PANEL, outline=col, width=2)
            # Labels
            c.create_text(wx, wy - 5, text=str(i), fill=col,
                          font=("Courier New", 8, "bold"))
            c.create_text(wx, wy + 6, text=str(spd), fill=C_WHITE,
                          font=("Courier New", 7))

        # ---- Overall direction arrow ----
        # Mecanum kinematics: 0=BL, 1=FL, 2=FR, 3=BR
        forward = (speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4.0
        lateral = (-speeds[0] + speeds[1] + speeds[2] - speeds[3]) / 4.0  # positive=left
        mag = math.sqrt(forward**2 + lateral**2)
        if mag > 0.5:
            norm_scale = min(w, h) * 0.18 / max(self.wheel_max, 1)
            adx = int(-lateral * norm_scale)   # left=negative x on screen
            ady = int(-forward * norm_scale)   # forward=negative y on screen
            if abs(forward) >= abs(lateral):
                arrow_col = C_GREEN if forward > 0 else C_RED
            else:
                arrow_col = "#ffff44"
            c.create_line(cx, cy, cx + adx, cy + ady,
                          fill=arrow_col, width=4,
                          arrow=tk.LAST, arrowshape=(14, 18, 6))

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
