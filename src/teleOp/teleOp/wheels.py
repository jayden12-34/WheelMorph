import select
import signal
import sys
import termios
import threading
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
PORT = '/dev/ttyUSB0'
MOTOR_IDS = [1, 0, 2, 3]  # FL/BL physically swapped — motor 1 = FL, motor 0 = BL

ADDR_TORQUE_ENABLE   = 64
ADDR_GOAL_VELOCITY   = 104
ADDR_PRESENT_CURRENT = 126
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# Keyboard sends -50..50; map to Dynamixel velocity units
VELOCITY_SCALE = 6

# Fraction of normal velocity applied in compliant mode
COMPLIANT_WHEEL_SCALE = 0.5

# mA per unit for Dynamixel X-series (XM/XH); adjust if using XL series (1.0 mA/unit)
CURRENT_UNIT_MA = 2.69

# Motors 0 and 3 are physically mounted in reverse on the chassis
REVERSED_MOTORS = {2, 3}


class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')

        self._sim_counter = 0
        self.port   = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        self._ready     = self._initialize()
        self._compliant = False

        self.subscription = self.create_subscription(
            Int32MultiArray, 'wheel_commands', self.listener_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, 'estop', self._estop_callback, 10)
        self.reset_sub = self.create_subscription(
            Bool, 'motor_reset', self._motor_reset_callback, 10)
        self.compliant_sub = self.create_subscription(
            Bool, 'compliant_mode', self._compliant_callback, 10)

        self.currents_pub = self.create_publisher(Int32MultiArray, 'wheel_currents', 10)
        self.create_timer(0.1, self._publish_currents)

    def _initialize(self):
        try:
            if not self.port.openPort():
                self.get_logger().warn(
                    f'Cannot open {PORT} — wheel node running in SIMULATION mode')
                return False
            if not self.port.setBaudRate(BAUDRATE):
                self.get_logger().warn(
                    'Failed to set baudrate — wheel node running in SIMULATION mode')
                return False

            for mid in MOTOR_IDS:
                result, error = self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                if result != COMM_SUCCESS or error != 0:
                    self.get_logger().warn(
                        f'Torque enable failed for motor {mid} '
                        f'({self.packet.getTxRxResult(result)} | '
                        f'{self.packet.getRxPacketError(error)}) '
                        '— running in SIMULATION mode')
                    return False

            self.get_logger().info(f'Wheel motors ready on {PORT}')
            return True
        except Exception as e:
            self.get_logger().warn(
                f'Motor connection error ({e}) — wheel node running in SIMULATION mode')
            return False

    def _set_velocity(self, motor_id, velocity):
        if motor_id in REVERSED_MOTORS:
            velocity = -velocity
        raw = int(velocity) & 0xFFFFFFFF   # two's complement for negative values
        result, _ = self.packet.write4ByteTxRx(
            self.port, motor_id, ADDR_GOAL_VELOCITY, raw)
        if result != COMM_SUCCESS:
            self.get_logger().warn(f'Velocity write failed for motor {motor_id}')

    def _estop_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().warn('EMERGENCY STOP received — disabling wheel motors')
        if self._ready:
            for mid in MOTOR_IDS:
                self._set_velocity(mid, 0)
                self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._ready = False
        else:
            self.get_logger().warn('[SIM] Emergency stop — all wheel speeds → 0')

    def _motor_reset_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().info('Motor reset: reconnecting wheel motors')
        all_ok = True
        for mid in MOTOR_IDS:
            result, error = self.packet.write1ByteTxRx(
                self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if result != COMM_SUCCESS or error != 0:
                all_ok = False
        if all_ok:
            self._ready = True
            self.get_logger().info('Wheel motors reconnected')
        else:
            self.get_logger().warn('Re-enable failed — retrying full init')
            try:
                self.port.closePort()
            except Exception:
                pass
            self._ready = self._initialize()

    def _compliant_callback(self, msg):
        self._compliant = msg.data
        # Wheels stay torque-enabled in compliant mode; velocity is scaled down
        # so they nominally follow commands but back-drive under moderate force.
        self.get_logger().info(f'Wheel compliant mode {"ON" if msg.data else "OFF"}')

    def listener_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn(f'Expected ≥4 values, got {len(msg.data)}')
            return

        wheel_speeds = list(msg.data[0:4])
        scale = COMPLIANT_WHEEL_SCALE if self._compliant else 1.0

        if self._ready:
            for i, speed in enumerate(wheel_speeds):
                self._set_velocity(MOTOR_IDS[i], speed * VELOCITY_SCALE * scale)
        else:
            # Print sim output at ~1 Hz so the terminal stays readable
            self._sim_counter += 1
            if self._sim_counter % 20 == 0:
                dxl_vals = [
                    -(s * VELOCITY_SCALE * scale) if i in REVERSED_MOTORS
                    else s * VELOCITY_SCALE * scale
                    for i, s in enumerate(wheel_speeds)
                ]
                self.get_logger().info(
                    f'[SIM] wheel_speeds={wheel_speeds}  '
                    f'→ dynamixel_velocities={dxl_vals}')

    def _publish_currents(self):
        currents = []
        for mid in MOTOR_IDS:
            if self._ready:
                data, result, _ = self.packet.read2ByteTxRx(
                    self.port, mid, ADDR_PRESENT_CURRENT)
                if result == COMM_SUCCESS:
                    if data > 32767:
                        data -= 65536
                    currents.append(int(data * CURRENT_UNIT_MA))
                else:
                    currents.append(0)
            else:
                currents.append(0)
        msg = Int32MultiArray()
        msg.data = currents
        self.currents_pub.publish(msg)

    def _disable_motors(self):
        if not self._ready:
            return
        self._ready = False
        for mid in MOTOR_IDS:
            try:
                self.packet.write4ByteTxRx(self.port, mid, ADDR_GOAL_VELOCITY, 0)
                self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            except Exception:
                pass
        try:
            self.port.closePort()
        except Exception:
            pass

    def destroy_node(self):
        self._disable_motors()
        try:
            self.get_logger().info('Wheel motors disabled')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelController()

    def handle_shutdown(*_):
        node._disable_motors()
        sys.exit(0)

    signal.signal(signal.SIGTERM, handle_shutdown)
    signal.signal(signal.SIGHUP, handle_shutdown)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if sys.stdin.isatty():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while spin_thread.is_alive():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if r:
                    ch = sys.stdin.read(1)
                    if ch in ('\x1b', '\x03'):  # ESC or Ctrl+C
                        node.get_logger().warn('ESC pressed — disabling wheel motors and exiting')
                        break
        except (KeyboardInterrupt, SystemExit):
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    else:
        try:
            spin_thread.join()
        except (KeyboardInterrupt, SystemExit):
            pass

    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
