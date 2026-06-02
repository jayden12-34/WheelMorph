import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

import tkinter as tk
from tkinter import ttk, Scale, VERTICAL, HORIZONTAL
import threading
import time


# Maps key names to the set_* method suffix
_MOVE_MAP = {
    'w': 'forward', 'up': 'forward',
    'a': 'left',    'left': 'left',
    's': 'reverse', 'down': 'reverse',
    'd': 'right',   'right': 'right',
}


class GuiTeleop(Node):

    def __init__(self):
        super().__init__('gui_teleop')
        self.pub = self.create_publisher(Int32MultiArray, 'wheel_commands', 10)
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)

        self.wheel_speed = [0, 0, 0, 0]
        self.leg_angles = [0, 0, 0, 0]
        self.lock = threading.Lock()

        self.wheel_max = 50
        self.speed_pct = 100
        self.dead_man = True

        # Ordered list of currently-held movement keys (most recent last)
        self._held_keys = []
        # Suppress slider callbacks during programmatic updates
        self._syncing = False

        self._build_gui()

    # ------------------------------------------------------------------ #
    #  GUI construction                                                    #
    # ------------------------------------------------------------------ #

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("ROS2 Wheel & Leg Teleop")
        self.root.geometry("1400x900")

        self._build_header()
        self._build_control_bar()
        self._build_main_area()

        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

        self._poll_status()
        self._periodic_update()

    def _build_header(self):
        header = tk.Frame(self.root, bg="#2b2b2b", pady=6)
        header.pack(fill=tk.X)

        tk.Label(header, text="ROS2 Wheel & Leg Teleop",
                 font=("Arial", 14, "bold"), fg="white", bg="#2b2b2b").pack(side=tk.LEFT, padx=12)

        self.status_canvas = tk.Canvas(header, width=16, height=16,
                                       bg="#2b2b2b", highlightthickness=0)
        self.status_dot = self.status_canvas.create_oval(2, 2, 14, 14, fill="gray")
        self.status_canvas.pack(side=tk.RIGHT, padx=5)

        self.status_label = tk.Label(header, text="No subscribers",
                                     fg="gray", bg="#2b2b2b", font=("Arial", 10))
        self.status_label.pack(side=tk.RIGHT, padx=4)

    def _build_control_bar(self):
        bar = tk.Frame(self.root, bg="lightgray", pady=10)
        bar.pack(fill=tk.X)

        # WASD d-pad
        bf = tk.Frame(bar, bg="lightgray")
        bf.pack(side=tk.LEFT, padx=12)

        tk.Label(bf, text="W/A/S/D or Arrow Keys",
                 font=("Arial", 10), bg="lightgray").grid(row=0, column=0, columnspan=3, pady=2)

        tk.Button(bf, text="FWD\n(W/↑)", width=8, height=2, bg="lightgreen",
                  command=self.set_forward).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(bf, text="LEFT\n(A/←)", width=8, height=2, bg="lightyellow",
                  command=self.set_left).grid(row=2, column=0, padx=2, pady=2)
        tk.Button(bf, text="REV\n(S/↓)", width=8, height=2, bg="lightcoral",
                  command=self.set_reverse).grid(row=2, column=1, padx=2, pady=2)
        tk.Button(bf, text="RIGHT\n(D/→)", width=8, height=2, bg="lightyellow",
                  command=self.set_right).grid(row=2, column=2, padx=2, pady=2)
        tk.Button(bf, text="STOP (SPACE)", width=26, height=1, bg="gray", fg="white",
                  command=self.set_stop).grid(row=3, column=0, columnspan=3, padx=2, pady=2)

        # Speed %
        sf = tk.Frame(bar, bg="lightgray")
        sf.pack(side=tk.LEFT, padx=16)
        tk.Label(sf, text="Speed %", bg="lightgray", font=("Arial", 10, "bold")).pack()
        self.speed_scale = Scale(sf, from_=0, to=100, orient=HORIZONTAL,
                                 length=140, width=20, font=("Arial", 10),
                                 command=lambda v: setattr(self, 'speed_pct', int(float(v))))
        self.speed_scale.set(100)
        self.speed_scale.pack()

        # Checkboxes
        of = tk.Frame(bar, bg="lightgray")
        of.pack(side=tk.LEFT, padx=16)

        self.dead_man_var = tk.BooleanVar(value=True)
        tk.Checkbutton(of, text="Dead man\n(key-release stops)",
                       variable=self.dead_man_var, bg="lightgray", font=("Arial", 10),
                       command=lambda: setattr(self, 'dead_man', self.dead_man_var.get())
                       ).pack(anchor="w")

        self.compact_var = tk.BooleanVar(value=False)
        tk.Checkbutton(of, text="Compact mode", variable=self.compact_var,
                       bg="lightgray", font=("Arial", 10),
                       command=self._toggle_compact).pack(anchor="w")

        # E-STOP (right-aligned, prominent)
        tk.Button(bar, text="⚠  EMERGENCY\n       STOP",
                  font=("Arial", 15, "bold"), bg="red", fg="white",
                  activebackground="#bb0000", activeforeground="white",
                  width=13, height=3, relief=tk.RAISED, bd=4,
                  command=self.emergency_stop).pack(side=tk.RIGHT, padx=20)

    def _build_main_area(self):
        main = tk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True)

        # --- Diagram (left, fixed-ish width, scales in height) ---
        diag_frame = tk.LabelFrame(main, text="Robot Diagram", font=("Arial", 11, "bold"))
        diag_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=5, pady=5)

        self.canvas = tk.Canvas(diag_frame, width=320, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        # Draw only after the canvas has been sized by the geometry manager
        self.canvas.bind('<Configure>', lambda _e: self.draw_robot_diagram())

        # --- Tabbed sliders (right, fills remaining space) ---
        self.slider_frame = tk.Frame(main)
        self.slider_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        nb = ttk.Notebook(self.slider_frame)
        nb.pack(fill=tk.BOTH, expand=True)

        self.wheel_sliders = {}
        self.wheel_labels = {}
        self.leg_sliders = {}
        self.leg_labels = {}

        # Wheels tab
        wt = tk.Frame(nb)
        nb.add(wt, text="  Wheels  ")
        wt.grid_columnconfigure(0, weight=1)
        wt.grid_columnconfigure(1, weight=1)
        wt.grid_rowconfigure(0, weight=1)
        wt.grid_rowconfigure(1, weight=1)

        for wid, row, col, title in (
            (1, 0, 0, "Front Left (1)"),
            (2, 0, 1, "Front Right (2)"),
            (0, 1, 0, "Back Left (0)"),
            (3, 1, 1, "Back Right (3)"),
        ):
            pf = tk.LabelFrame(wt, text=title, font=("Arial", 11, "bold"))
            pf.grid(row=row, column=col, sticky="nsew", padx=5, pady=5)
            inner = tk.Frame(pf, padx=8, pady=4)
            inner.pack(fill=tk.BOTH, expand=True)
            tk.Label(inner, text=f"Wheel {wid}", font=("Arial", 12, "bold")).pack()
            ws = Scale(inner, from_=50, to=-50, orient=VERTICAL, length=300, width=28,
                       font=("Arial", 10),
                       command=lambda v, i=wid: self.update_wheel(i, v))
            ws.set(0)
            ws.pack(fill=tk.BOTH, expand=True)
            wl = tk.Label(inner, text="0", font=("Arial", 11))
            wl.pack()
            self.wheel_sliders[wid] = ws
            self.wheel_labels[wid] = wl

        # Legs tab
        lt = tk.Frame(nb)
        nb.add(lt, text="  Legs  ")
        lt.grid_columnconfigure(0, weight=1)
        lt.grid_columnconfigure(1, weight=1)
        lt.grid_rowconfigure(0, weight=1)
        lt.grid_rowconfigure(1, weight=1)

        for lid, row, col, title in (
            (1, 0, 0, "Front Left (1)"),
            (2, 0, 1, "Front Right (2)"),
            (0, 1, 0, "Back Left (0)"),
            (3, 1, 1, "Back Right (3)"),
        ):
            pf = tk.LabelFrame(lt, text=title, font=("Arial", 11, "bold"))
            pf.grid(row=row, column=col, sticky="nsew", padx=5, pady=5)
            inner = tk.Frame(pf, padx=8, pady=4)
            inner.pack(fill=tk.BOTH, expand=True)
            tk.Label(inner, text=f"Leg {lid}", font=("Arial", 12, "bold")).pack()
            ls = Scale(inner, from_=180, to=0, orient=VERTICAL, length=300, width=28,
                       font=("Arial", 10),
                       command=lambda v, i=lid: self.update_leg(i, v))
            ls.set(0)
            ls.pack(fill=tk.BOTH, expand=True)
            ll = tk.Label(inner, text="0", font=("Arial", 11))
            ll.pack()
            self.leg_sliders[lid] = ls
            self.leg_labels[lid] = ll

    # ------------------------------------------------------------------ #
    #  Diagram                                                             #
    # ------------------------------------------------------------------ #

    def draw_robot_diagram(self):
        c = self.canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return  # not yet laid out — Configure will call us again

        c.delete("all")
        cx, cy = w // 2, h // 2

        c.create_rectangle(cx - 40, cy - 55, cx + 40, cy + 55,
                            fill="lightblue", outline="black", width=2)
        c.create_text(cx, cy, text="ROBOT", font=("Arial", 10, "bold"))

        wheel_pos = {
            0: (cx - 80, cy + 75),
            1: (cx - 80, cy - 75),
            2: (cx + 80, cy - 75),
            3: (cx + 80, cy + 75),
        }
        with self.lock:
            speeds = list(self.wheel_speed)

        for i, (x, y) in wheel_pos.items():
            speed = speeds[i]
            if speed > 0:
                color = "#90ee90"
            elif speed < 0:
                color = "#f08080"
            else:
                color = "#cccccc"
            c.create_oval(x - 20, y - 20, x + 20, y + 20,
                          fill=color, outline="black", width=2)
            c.create_text(x, y - 7, text=str(i), font=("Arial", 10, "bold"))
            c.create_text(x, y + 7, text=str(speed), font=("Arial", 9))

    # ------------------------------------------------------------------ #
    #  State updates                                                       #
    # ------------------------------------------------------------------ #

    def update_wheel(self, i, v):
        if self._syncing:
            return
        with self.lock:
            self.wheel_speed[i] = int(float(v))
        self.wheel_labels[i].config(text=str(int(float(v))))

    def update_leg(self, i, v):
        with self.lock:
            self.leg_angles[i] = int(float(v))
        self.leg_labels[i].config(text=str(int(float(v))))

    # ------------------------------------------------------------------ #
    #  Periodic UI sync (decoupled from key-repeat rate)                  #
    # ------------------------------------------------------------------ #

    def _periodic_update(self):
        with self.lock:
            speeds = list(self.wheel_speed)

        self._syncing = True
        for i in range(4):
            self.wheel_sliders[i].set(speeds[i])
            self.wheel_labels[i].config(text=str(speeds[i]))
        self._syncing = False

        self.draw_robot_diagram()
        self.root.after(50, self._periodic_update)

    def _poll_status(self):
        count = self.pub.get_subscription_count()
        if count > 0:
            self.status_canvas.itemconfig(self.status_dot, fill="limegreen")
            self.status_label.config(text=f"{count} subscriber(s)", fg="limegreen")
        else:
            self.status_canvas.itemconfig(self.status_dot, fill="red")
            self.status_label.config(text="No subscribers", fg="red")
        self.root.after(1000, self._poll_status)

    # ------------------------------------------------------------------ #
    #  Movement commands (update state only; UI sync happens at 20 Hz)    #
    # ------------------------------------------------------------------ #

    def set_forward(self):
        speed = int(self.wheel_max * self.speed_pct / 100)
        with self.lock:
            self.wheel_speed = [speed] * 4

    def set_reverse(self):
        speed = int(self.wheel_max * self.speed_pct / 100)
        with self.lock:
            self.wheel_speed = [-speed] * 4

    def set_left(self):
        speed = int(self.wheel_max * self.speed_pct / 100)
        with self.lock:
            self.wheel_speed = [-speed, speed, speed, -speed]

    def set_right(self):
        speed = int(self.wheel_max * self.speed_pct / 100)
        with self.lock:
            self.wheel_speed = [speed, -speed, -speed, speed]

    def set_stop(self):
        with self.lock:
            self.wheel_speed = [0] * 4

    def emergency_stop(self):
        with self.lock:
            self.wheel_speed = [0] * 4
            self.leg_angles = [0] * 4
        self._held_keys.clear()

        # Immediately publish zeros so motors react before next publish_loop tick
        zero = Int32MultiArray()
        zero.data = [0] * 8
        self.pub.publish(zero)

        estop = Bool()
        estop.data = True
        self.estop_pub.publish(estop)

        # Reset leg sliders in UI
        self._syncing = True
        for i in range(4):
            self.leg_sliders[i].set(0)
            self.leg_labels[i].config(text="0")
        self._syncing = False

    # ------------------------------------------------------------------ #
    #  Key handling with held-key tracking                                 #
    # ------------------------------------------------------------------ #

    def on_key_press(self, e):
        k = e.keysym.lower()
        if k in _MOVE_MAP:
            if k not in self._held_keys:
                self._held_keys.append(k)
            getattr(self, f'set_{_MOVE_MAP[k]}')()
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
            # Re-apply whichever movement key is still held (most recently pressed)
            getattr(self, f'set_{_MOVE_MAP[self._held_keys[-1]]}')()

    # ------------------------------------------------------------------ #
    #  Options                                                             #
    # ------------------------------------------------------------------ #

    def _toggle_compact(self):
        if self.compact_var.get():
            self.slider_frame.pack_forget()
            self.root.geometry("500x720")
        else:
            self.slider_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
            self.root.geometry("1400x900")

    # ------------------------------------------------------------------ #
    #  ROS publish loop (background thread)                               #
    # ------------------------------------------------------------------ #

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
