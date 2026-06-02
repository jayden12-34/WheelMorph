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
PORT = '/dev/ttyUSB0'
MOTOR_IDS = [0, 1, 2, 3]

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# Keyboard sends -50..50; map to Dynamixel velocity units
VELOCITY_SCALE = 4

# Motors 0 and 3 are physically mounted in reverse on the chassis
REVERSED_MOTORS = {0, 3}


class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')

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
        if not _git AVAILABLE:
            self.get_logger().warn(
                'dynamixel_sdk not installed — wheel node running in SIMULATION mode')
            return False

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

    def listener_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn(f'Expected ≥4 values, got {len(msg.data)}')
            return

        wheel_speeds = list(msg.data[0:4])

        if self._ready:
            for i, speed in enumerate(wheel_speeds):
                self._set_velocity(MOTOR_IDS[i], speed * VELOCITY_SCALE)
        else:
            # Print sim output at ~1 Hz so the terminal stays readable
            self._sim_counter += 1
            if self._sim_counter % 20 == 0:
                dxl_vals = [
                    -(s * VELOCITY_SCALE) if i in REVERSED_MOTORS
                    else s * VELOCITY_SCALE
                    for i, s in enumerate(wheel_speeds)
                ]
                self.get_logger().info(
                    f'[SIM] wheel_speeds={wheel_speeds}  '
                    f'→ dynamixel_velocities={dxl_vals}')

    def destroy_node(self):
        if self._ready:
            for mid in MOTOR_IDS:
                self._set_velocity(mid, 0)
                self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self.port.closePort()
            self.get_logger().info('Wheel motors disabled')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
