import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
PORT = '/dev/ttyUSB1'
MOTOR_IDS = [0, 1, 2, 3]

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Leg angle range: 0-180 degrees → Dynamixel position 0-4095
# (full 360° = 4095 ticks, so 180° = 2048 ticks)
TICKS_PER_DEGREE = 4095.0 / 360.0


class LegController(Node):
    def __init__(self):
        super().__init__('leg_controller')

        self.port = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self._ready = self._initialize()

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'wheel_commands',
            self.listener_callback,
            10)
        self.estop_sub = self.create_subscription(
            Bool,
            'estop',
            self._estop_callback,
            10)

    def _initialize(self):
        if not self.port.openPort():
            self.get_logger().error(f'Failed to open port {PORT}')
            return False
        if not self.port.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to set baudrate')
            return False

        for mid in MOTOR_IDS:
            result, error = self.packet.write1ByteTxRx(
                self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if result != COMM_SUCCESS or error != 0:
                self.get_logger().error(
                    f'Torque enable failed for leg motor {mid}: '
                    f'{self.packet.getTxRxResult(result)} | '
                    f'{self.packet.getRxPacketError(error)}')
                return False

        self.get_logger().info(f'Leg motors ready on {PORT}')
        return True

    def _set_position(self, motor_id, angle_deg):
        angle_deg = max(0, min(int(angle_deg), 180))
        position = int(angle_deg * TICKS_PER_DEGREE)
        result, _ = self.packet.write4ByteTxRx(
            self.port, motor_id, ADDR_GOAL_POSITION, position)
        if result != COMM_SUCCESS:
            self.get_logger().warn(f'Position write failed for leg motor {motor_id}')

    def _estop_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().warn('EMERGENCY STOP: disabling leg motors')
        if self._ready:
            for mid in MOTOR_IDS:
                self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._ready = False

    def listener_callback(self, msg):
        if len(msg.data) < 8:
            self.get_logger().warn(f'Expected 8 values, got {len(msg.data)}')
            return

        leg_angles = list(msg.data[4:8])

        if self._ready:
            for i, angle in enumerate(leg_angles):
                self._set_position(MOTOR_IDS[i], angle)
        self.get_logger().debug(f'Legs: {leg_angles}')

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
