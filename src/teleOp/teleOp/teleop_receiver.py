#!/usr/bin/env python3

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


CTRL_PORT = 7700
STATE_PORT = 7701
DISCOVERY_PORT = 7799

DISCOVERY_REQUEST = 'WHEEL_TELEOP_DISCOVER'
DISCOVERY_RESPONSE = 'WHEEL_TELEOP_HERE'

SD_DEADZONE = 0.12
SD_LEG_STEP = 30


def _clamp(v, lo, hi):
    return int(
        lo if v < lo
        else hi if v > hi
        else v
    )


class TeleopReceiver(Node):

    def __init__(self):

        super().__init__('teleop_receiver')

        # ── ROS publishers ───────────────────────────────────────────────────

        self.pub = self.create_publisher(
            Int32MultiArray,
            'wheel_commands',
            10
        )

        # ── State ────────────────────────────────────────────────────────────

        self.wheel_cmd = [0, 0, 0, 0]

        # Logical angles:
        # [FL, BL, FR, BR]
        self.leg_angles = [0, 0, 0, 0]

        self.speed_pct = 20
        self.wheel_max = 50
        self.drive_mode = 0

        self.lock = threading.Lock()

        # IP address of the most recent teleop sender
        self.client_addr = None

        # ── UDP control socket ───────────────────────────────────────────────

        self.ctrl_sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.ctrl_sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        # IMPORTANT:
        # Listen on every network interface.
        self.ctrl_sock.bind(
            ('0.0.0.0', CTRL_PORT)
        )

        self.ctrl_sock.settimeout(0.5)

        # ── UDP discovery socket ─────────────────────────────────────────────

        self.discovery_sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.discovery_sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.discovery_sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_BROADCAST,
            1
        )

        self.discovery_sock.bind(
            ('0.0.0.0', DISCOVERY_PORT)
        )

        self.discovery_sock.settimeout(0.5)

        # ── Threads ──────────────────────────────────────────────────────────

        threading.Thread(
            target=self._recv_loop,
            daemon=True
        ).start()

        threading.Thread(
            target=self._discovery_loop,
            daemon=True
        ).start()

        # ── Publish state at 20 Hz ───────────────────────────────────────────

        self.create_timer(
            1.0 / 20.0,
            self._publish_loop
        )

        self.get_logger().info(
            f'Teleop receiver listening on '
            f'UDP {CTRL_PORT}'
        )

        self.get_logger().info(
            f'Discovery listening on '
            f'UDP {DISCOVERY_PORT}'
        )

    # ── Discovery ────────────────────────────────────────────────────────────

    def _discovery_loop(self):

        while rclpy.ok():

            try:

                data, addr = self.discovery_sock.recvfrom(
                    1024
                )

                msg = data.decode(
                    errors='ignore'
                )

                if msg == DISCOVERY_REQUEST:

                    # Remember the sender.
                    self.client_addr = addr[0]

                    response = (
                        f'{DISCOVERY_RESPONSE} '
                        f'{CTRL_PORT}'
                    )

                    self.discovery_sock.sendto(
                        response.encode(),
                        addr
                    )

                    self.get_logger().info(
                        f'Discovery request from '
                        f'{addr[0]} — responding'
                    )

            except socket.timeout:
                continue

            except Exception as e:

                if rclpy.ok():
                    self.get_logger().warn(
                        f'Discovery error: {e}'
                    )

    # ── UDP control receiver ─────────────────────────────────────────────────

    def _recv_loop(self):

        while rclpy.ok():

            try:

                data, addr = self.ctrl_sock.recvfrom(
                    4096
                )

                self.client_addr = addr[0]

                msg = json.loads(
                    data.decode()
                )

                msg_type = msg.get('type')

                if msg_type == 'ctrl':

                    self._apply_ctrl(msg)

                elif msg_type == 'estop':

                    self._emergency_stop()

                elif msg_type == 'motor_reset':

                    threading.Thread(
                        target=self._motor_reset,
                        daemon=True
                    ).start()

                elif msg_type == 'speed_pct':

                    with self.lock:

                        self.speed_pct = max(
                            0,
                            min(
                                100,
                                int(
                                    msg.get(
                                        'value',
                                        20
                                    )
                                )
                            )
                        )

            except socket.timeout:
                continue

            except Exception as e:

                if rclpy.ok():
                    self.get_logger().warn(
                        f'UDP receive error: {e}'
                    )

    # ── Control processing ───────────────────────────────────────────────────

    def _apply_ctrl(self, ctrl):

        with self.lock:

            angles = list(
                self.leg_angles
            )

            speed_pct = self.speed_pct

            if 'drive_mode' in ctrl:

                self.drive_mode = _clamp(
                    int(ctrl['drive_mode']),
                    0,
                    1
                )

        if ctrl.get('speed_pct') is not None:

            speed_pct = _clamp(
                int(ctrl['speed_pct']),
                0,
                100
            )

        # L2 = emergency/stop-like control
        if ctrl.get('l2'):

            with self.lock:

                self.wheel_cmd = [
                    0,
                    0,
                    0,
                    0
                ]

                self.leg_angles = [
                    0,
                    0,
                    0,
                    0
                ]

                self.speed_pct = speed_pct

            return

        # ── Wheel control ────────────────────────────────────────────────────

        lx = float(
            ctrl.get('lx', 0)
        )

        ly = float(
            ctrl.get('ly', 0)
        )

        ry = float(
            ctrl.get('ry', 0)
        )

        if abs(lx) < SD_DEADZONE:
            lx = 0.0

        if abs(ly) < SD_DEADZONE:
            ly = 0.0

        throttle = max(
            0.0,
            -ry
        )

        if throttle < SD_DEADZONE:

            throttle = 0.0

        else:

            speed_pct = int(
                throttle * 100
            )

        effective = (
            speed_pct
            / 100.0
            * self.wheel_max
        )

        forward = -ly * effective
        turn = lx * effective

        wm = self.wheel_max

        ws = [
            _clamp(
                forward + turn,
                -wm,
                wm
            ),  # FL

            _clamp(
                forward + turn,
                -wm,
                wm
            ),  # BL

            _clamp(
                forward - turn,
                -wm,
                wm
            ),  # FR

            _clamp(
                forward - turn,
                -wm,
                wm
            ),  # BR
        ]

        # ── Leg controls ─────────────────────────────────────────────────────

        dpad = ctrl.get(
            'dpad',
            [0, 0]
        )

        dx = int(dpad[0])
        dy = int(dpad[1])

        # D-pad retracts individual legs
        if dy == 1:
            angles[0] = max(
                0,
                angles[0] - SD_LEG_STEP
            )  # FL

        if dx == -1:
            angles[1] = max(
                0,
                angles[1] - SD_LEG_STEP
            )  # BL

        if dx == 1:
            angles[2] = max(
                0,
                angles[2] - SD_LEG_STEP
            )  # FR

        if dy == -1:
            angles[3] = max(
                0,
                angles[3] - SD_LEG_STEP
            )  # BR

        # L1 = retract all
        if ctrl.get('l1'):

            angles = [
                max(
                    0,
                    a - SD_LEG_STEP
                )
                for a in angles
            ]

        # R1 = extend all
        if ctrl.get('r1'):

            angles = [
                min(
                    180,
                    a + SD_LEG_STEP
                )
                for a in angles
            ]

        # Face buttons = extend individual legs
        if ctrl.get('btn_y'):

            angles[0] = min(
                180,
                angles[0] + SD_LEG_STEP
            )  # FL

        if ctrl.get('btn_x'):

            angles[1] = min(
                180,
                angles[1] + SD_LEG_STEP
            )  # BL

        if ctrl.get('btn_b'):

            angles[2] = min(
                180,
                angles[2] + SD_LEG_STEP
            )  # FR

        if ctrl.get('btn_a'):

            angles[3] = min(
                180,
                angles[3] + SD_LEG_STEP
            )  # BR

        # ── Paddle wheel commands ────────────────────────────────────────────

        paddle_spd = max(
            1,
            int(
                self.wheel_max
                * speed_pct
                / 100
            )
        )

        if ctrl.get('paddle_reverse'):
            paddle_spd = -paddle_spd

        if ctrl.get('l4'):
            ws[1] = paddle_spd  # BL

        if ctrl.get('l5'):
            ws[0] = paddle_spd  # FL

        if ctrl.get('r4'):
            ws[2] = paddle_spd  # FR

        if ctrl.get('r5'):
            ws[3] = paddle_spd  # BR

        # ── Store state ──────────────────────────────────────────────────────

        with self.lock:

            self.wheel_cmd = ws
            self.leg_angles = angles
            self.speed_pct = speed_pct

    # ── State publishing ─────────────────────────────────────────────────────

    def _publish_loop(self):

        msg = Int32MultiArray()

        while False:
            pass

        # This function is called by a ROS timer, so publish once.
        with self.lock:

            msg.data = (
                list(self.wheel_cmd)
                + list(self.leg_angles)
                + [self.drive_mode]
            )

            client = self.client_addr

            state = {
                'type': 'state',
                'wheel_torque': list(self.wheel_cmd),
                'leg_angles': list(self.leg_angles),
                'wheel_currents': [0, 0, 0, 0],
                'leg_currents': [0, 0, 0, 0],
                'wheel_temps': [0, 0, 0, 0],
                'speed_pct': self.speed_pct,
            }

        self.pub.publish(msg)

        # Send state back to the laptop.
        if client is not None:

            try:

                data = json.dumps(
                    state
                ).encode()

                self.ctrl_sock.sendto(
                    data,
                    (
                        client,
                        STATE_PORT
                    )
                )

            except Exception:
                pass

    # ── E-stop ───────────────────────────────────────────────────────────────

    def _emergency_stop(self):

        self.get_logger().warn(
            'EMERGENCY STOP received'
        )

        with self.lock:

            self.wheel_cmd = [
                0,
                0,
                0,
                0
            ]

            self.leg_angles = [
                0,
                0,
                0,
                0
            ]

    # ── Motor reset ──────────────────────────────────────────────────────────

    def _motor_reset(self):

        self.get_logger().info(
            'Motor reset requested'
        )

        # Keep your existing motor-reset behavior here
        # if this node is also responsible for it.

    # ── Shutdown ─────────────────────────────────────────────────────────────

    def destroy_node(self):

        try:
            self.ctrl_sock.close()
        except Exception:
            pass

        try:
            self.discovery_sock.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = TeleopReceiver()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()