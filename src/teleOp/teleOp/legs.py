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
PORT = '/dev/ttyUSB1'
MOTOR_IDS = [0, 1, 2, 3]

ADDR_TORQUE_ENABLE   = 64
ADDR_GOAL_POSITION   = 116
ADDR_PRESENT_CURRENT = 126
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# 0–180° → 0–2047 Dynamixel ticks  (full 360° = 4095 ticks)
TICKS_PER_DEGREE = 4095.0 / 360.0

# mA per unit for Dynamixel X-series (XM/XH); adjust if using XL series (1.0 mA/unit)
CURRENT_UNIT_MA = 2.69

# Motors 0 and 1 are mounted in the opposite direction, so their
# command angles need to be mirrored (180 - angle).
FLIPPED_MOTORS = {0, 1}


class LegController(Node):
    def __init__(self):
        super().__init__('leg_controller')

        self._sim_counter = 0
        self.port   = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        self._ready = self._initialize()

        self.subscription = self.create_subscription(
            Int32MultiArray, 'wheel_commands', self.listener_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, 'estop', self._estop_callback, 10)

        self.currents_pub = self.create_publisher(Int32MultiArray, 'leg_currents', 10)
        self.create_timer(0.1, self._publish_currents)

    def _initialize(self):
        try:
            if not self.port.openPort():
                self.get_logger().warn(
                    f'Cannot open {PORT} — leg node running in SIMULATION mode')
                return False
            if not self.port.setBaudRate(BAUDRATE):
                self.get_logger().warn(
                    'Failed to set baudrate — leg node running in SIMULATION mode')
                return False

            for mid in MOTOR_IDS:
                result, error = self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                if result != COMM_SUCCESS or error != 0:
                    self.get_logger().warn(
                        f'Torque enable failed for leg motor {mid} '
                        f'({self.packet.getTxRxResult(result)} | '
                        f'{self.packet.getRxPacketError(error)}) '
                        '— running in SIMULATION mode')
                    return False

            self.get_logger().info(f'Leg motors ready on {PORT}')
            return True
        except Exception as e:
            self.get_logger().warn(
                f'Motor connection error ({e}) — leg node running in SIMULATION mode')
            return False

    def _normalize_angle(self, motor_id, angle_deg):
        angle_deg = max(0, min(int(angle_deg), 180))
        if motor_id in FLIPPED_MOTORS:
            angle_deg = 180 - angle_deg
        return angle_deg

    def _set_position(self, motor_id, angle_deg):
        angle_deg = self._normalize_angle(motor_id, angle_deg)
        position  = int(angle_deg * TICKS_PER_DEGREE)
        result, _ = self.packet.write4ByteTxRx(
            self.port, motor_id, ADDR_GOAL_POSITION, position)
        if result != COMM_SUCCESS:
            self.get_logger().warn(f'Position write failed for leg motor {motor_id}')

    def _estop_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().warn('EMERGENCY STOP received — disabling leg motors')
        if self._ready:
            for mid in MOTOR_IDS:
                self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._ready = False
        else:
            self.get_logger().warn('[SIM] Emergency stop acknowledged')

    def listener_callback(self, msg):
        if len(msg.data) < 8:
            self.get_logger().warn(f'Expected 8 values, got {len(msg.data)}')
            return

        leg_angles = list(msg.data[4:8])

        if self._ready:
            for i, angle in enumerate(leg_angles):
                self._set_position(MOTOR_IDS[i], angle)
        else:
            self._sim_counter += 1
            if self._sim_counter % 20 == 0:
                ticks = [int(self._normalize_angle(MOTOR_IDS[i], a) * TICKS_PER_DEGREE)
                         for i, a in enumerate(leg_angles)]
                self.get_logger().info(
                    f'[SIM] leg_angles={leg_angles}°  '
                    f'→ dynamixel_positions={ticks}')

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
            self.get_logger().info('Leg motors disabled')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LegController()

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
                        node.get_logger().warn('ESC pressed — disabling leg motors and exiting')
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
