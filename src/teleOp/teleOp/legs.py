import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    _SDK_AVAILABLE = True
except ImportError:
    _SDK_AVAILABLE = False

PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
PORT = '/dev/ttyUSB1'
MOTOR_IDS = [0, 1, 2, 3]

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# 0–180° → 0–2047 Dynamixel ticks  (full 360° = 4095 ticks)
TICKS_PER_DEGREE = 4095.0 / 360.0


class LegController(Node):
    def __init__(self):
        super().__init__('leg_controller')

        self._sim_counter = 0

        if _SDK_AVAILABLE:
            self.port   = PortHandler(PORT)
            self.packet = PacketHandler(PROTOCOL_VERSION)
        else:
            self.port   = None
            self.packet = None

        self._ready = self._initialize()

        self.subscription = self.create_subscription(
            Int32MultiArray, 'wheel_commands', self.listener_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, 'estop', self._estop_callback, 10)

    def _initialize(self):
        if not _SDK_AVAILABLE:
            self.get_logger().warn(
                'dynamixel_sdk not installed — leg node running in SIMULATION mode')
            return False

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

    def _set_position(self, motor_id, angle_deg):
        angle_deg = max(0, min(int(angle_deg), 180))
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
                ticks = [int(max(0, min(a, 180)) * TICKS_PER_DEGREE)
                         for a in leg_angles]
                self.get_logger().info(
                    f'[SIM] leg_angles={leg_angles}°  '
                    f'→ dynamixel_positions={ticks}')

    def destroy_node(self):
        if self._ready:
            for mid in MOTOR_IDS:
                self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self.port.closePort()
            self.get_logger().info('Leg motors disabled')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LegController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
