import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

CTRL_PORT  = 7700
STATE_PORT = 7701

SD_DEADZONE    = 0.12
SD_LEG_STEP    = 15    # 5× original speed

FRONT_TRACK_CM = 23.0
BACK_TRACK_CM  = 37.0
_TRACK_AVG     = (FRONT_TRACK_CM + BACK_TRACK_CM) / 2.0  # 30.0


class TeleopReceiver(Node):

    def __init__(self):
        super().__init__('teleop_receiver')

        self.declare_parameter('ctrl_port',  CTRL_PORT)
        self.declare_parameter('state_port', STATE_PORT)
        ctrl_port  = self.get_parameter('ctrl_port').value
        state_port = self.get_parameter('state_port').value

        self.pub           = self.create_publisher(Int32MultiArray, 'wheel_commands', 10)
        self.estop_pub     = self.create_publisher(Bool, 'estop', 10)
        self.reset_pub     = self.create_publisher(Bool, 'motor_reset', 10)
        self.compliant_pub = self.create_publisher(Bool, 'compliant_mode', 10)

        self.create_subscription(Int32MultiArray, 'wheel_currents', self._wheel_cb, 10)
        self.create_subscription(Int32MultiArray, 'leg_currents',   self._leg_cb,   10)

        self.wheel_speed    = [0, 0, 0, 0]
        self.leg_angles     = [0, 0, 0, 0]
        self.wheel_currents = [0, 0, 0, 0]
        self.leg_currents   = [0, 0, 0, 0]
        self.speed_pct      = 20
        self.wheel_max      = 50
        self.compliant      = False
        self.lock           = threading.Lock()

        self._sender_addr = None
        self._state_port  = state_port

        self._ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._ctrl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._ctrl_sock.bind(('0.0.0.0', ctrl_port))
        self._ctrl_sock.settimeout(1.0)

        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        threading.Thread(target=self._recv_loop,    daemon=True).start()
        threading.Thread(target=self._publish_loop, daemon=True).start()
        threading.Thread(target=self._state_loop,   daemon=True).start()

        self.get_logger().info(
            f'TeleopReceiver ready  ctrl=UDP:{ctrl_port}  state=UDP:{state_port}')

    # ── ROS subscribers ─────────────────────────────────────────────────────

    def _wheel_cb(self, msg):
        with self.lock:
            self.wheel_currents = list(msg.data[:4])

    def _leg_cb(self, msg):
        with self.lock:
            self.leg_currents = list(msg.data[:4])

    # ── UDP receive loop ─────────────────────────────────────────────────────

    def _recv_loop(self):
        while rclpy.ok():
            try:
                data, addr = self._ctrl_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
            except Exception:
                continue

            self._sender_addr = (addr[0], self._state_port)
            t = msg.get('type')

            if t == 'ctrl':
                self._apply_ctrl(msg)
            elif t == 'estop':
                self._emergency_stop()
            elif t == 'motor_reset':
                threading.Thread(target=self._motor_reset, daemon=True).start()
            elif t == 'compliant':
                val = bool(msg.get('value', False))
                threading.Thread(target=self._set_compliant, args=(val,), daemon=True).start()
            elif t == 'speed_pct':
                with self.lock:
                    self.speed_pct = max(0, min(100, int(msg.get('value', 20))))

    # ── Control processing ───────────────────────────────────────────────────

    def _apply_ctrl(self, ctrl: dict):
        with self.lock:
            angles    = list(self.leg_angles)
            speed_pct = self.speed_pct

        if ctrl.get('speed_pct') is not None:
            speed_pct = max(0, min(100, int(ctrl['speed_pct'])))

        if ctrl.get('l2'):
            with self.lock:
                self.wheel_speed = [0, 0, 0, 0]
                self.leg_angles  = [0, 0, 0, 0]
                self.speed_pct   = speed_pct
            return

        lx = float(ctrl.get('lx', 0))
        ly = float(ctrl.get('ly', 0))
        ry = float(ctrl.get('ry', 0))
        if abs(lx) < SD_DEADZONE: lx = 0.0
        if abs(ly) < SD_DEADZONE: ly = 0.0

        # right stick forward (negative axis) sets throttle
        throttle = max(0.0, -ry)
        if throttle < SD_DEADZONE:
            throttle = 0.0
        else:
            speed_pct = int(throttle * 100)

        effective  = speed_pct / 100.0 * self.wheel_max
        forward    = -ly * effective
        turn       =  lx * effective
        front_turn = turn * (FRONT_TRACK_CM / _TRACK_AVG)
        back_turn  = turn * (BACK_TRACK_CM  / _TRACK_AVG)

        def clamp(v):
            return int(max(-self.wheel_max, min(self.wheel_max, v)))

        ws = [
            clamp(forward + front_turn),  # FL
            clamp(forward + back_turn),   # BL
            clamp(forward - front_turn),  # FR
            clamp(forward - back_turn),   # BR
        ]

        dpad = ctrl.get('dpad', [0, 0])
        dx, dy = int(dpad[0]), int(dpad[1])
        if dy ==  1: angles[0] = min(180, angles[0] + SD_LEG_STEP)
        if dx == -1: angles[1] = min(180, angles[1] + SD_LEG_STEP)
        if dx ==  1: angles[2] = min(180, angles[2] + SD_LEG_STEP)
        if dy == -1: angles[3] = min(180, angles[3] + SD_LEG_STEP)

        if ctrl.get('l1'): angles = [min(180, a + SD_LEG_STEP) for a in angles]
        if ctrl.get('r1'): angles = [max(0,   a - SD_LEG_STEP) for a in angles]

        # face buttons snap leg to 0°  (Y=FL, X=BL, B=FR, A=BR)
        if ctrl.get('btn_y'): angles[0] = 0
        if ctrl.get('btn_x'): angles[1] = 0
        if ctrl.get('btn_b'): angles[2] = 0
        if ctrl.get('btn_a'): angles[3] = 0

        paddle_spd = max(1, int(self.wheel_max * speed_pct / 100))
        if ctrl.get('l4'): ws[0] = paddle_spd
        if ctrl.get('l5'): ws[1] = paddle_spd
        if ctrl.get('r4'): ws[2] = paddle_spd
        if ctrl.get('r5'): ws[3] = paddle_spd

        with self.lock:
            self.wheel_speed = ws
            self.leg_angles  = angles
            self.speed_pct   = speed_pct

    def _emergency_stop(self):
        with self.lock:
            self.wheel_speed = [0, 0, 0, 0]
            self.leg_angles  = [0, 0, 0, 0]
        zero = Int32MultiArray()
        zero.data = [0] * 8
        self.pub.publish(zero)
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)

    def _motor_reset(self):
        """Zero all outputs, signal motor reset, then release."""
        with self.lock:
            self.wheel_speed = [0, 0, 0, 0]
            self.leg_angles  = [0, 0, 0, 0]
        zero = Int32MultiArray()
        zero.data = [0] * 8
        self.pub.publish(zero)

        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)
        time.sleep(0.5)
        msg.data = False
        self.reset_pub.publish(msg)
        self.get_logger().info('Motor reset complete')

    def _set_compliant(self, val: bool):
        with self.lock:
            self.compliant = val
        m = Bool()
        m.data = val
        self.compliant_pub.publish(m)
        self.get_logger().info(f'Compliant mode {"ON" if val else "OFF"}')

    # ── Publish / state loops ────────────────────────────────────────────────

    def _publish_loop(self):
        dt = 1.0 / 20
        while rclpy.ok():
            msg = Int32MultiArray()
            with self.lock:
                msg.data = list(self.wheel_speed) + list(self.leg_angles)
            self.pub.publish(msg)
            time.sleep(dt)

    def _state_loop(self):
        dt = 1.0 / 20
        while rclpy.ok():
            addr = self._sender_addr
            if addr:
                with self.lock:
                    payload = json.dumps({
                        'type':           'state',
                        'wheel_speed':    list(self.wheel_speed),
                        'leg_angles':     list(self.leg_angles),
                        'wheel_currents': list(self.wheel_currents),
                        'leg_currents':   list(self.leg_currents),
                        'speed_pct':      self.speed_pct,
                    }).encode()
                try:
                    self._state_sock.sendto(payload, addr)
                except Exception:
                    pass
            time.sleep(dt)

    def shutdown(self):
        self._emergency_stop()
        try:
            self._ctrl_sock.close()
        except Exception:
            pass
        try:
            self._state_sock.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TeleopReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
