import select
import signal
import sys
import termios
import threading
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
from dynamixel_sdk import (
    PortHandler, PacketHandler, COMM_SUCCESS,
    GroupSyncRead, GroupSyncWrite,
)

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
PORT = '/dev/ttyUSB0'
MOTOR_IDS = [0, 1, 2, 3]

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_CURRENT     = 102
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT  = 126
TORQUE_ENABLE         = 1
TORQUE_DISABLE        = 0
CURRENT_BASED_POSITION_CONTROL_MODE = 5

# Goal Current in CBPCP mode — caps the PID controller's current draw (2.69 mA/unit)
# 500 units ≈ 1345 mA: enough torque to move the legs without straining under load
LEG_GOAL_CURRENT_UNITS = 500

# 0–180° → 0–2047 Dynamixel ticks  (full 360° = 4095 ticks)
TICKS_PER_DEGREE = 4095.0 / 360.0

# mA per unit for Dynamixel X-series (XM/XH); adjust if using XL series (1.0 mA/unit)
CURRENT_UNIT_MA = 2.69

# Compliant mode: hold position (don't advance toward target) above this current draw
COMPLIANT_CURRENT_THRESHOLD_MA = 400

# Motors 0 and 1 are mounted in the opposite direction, so their
# command angles need to be mirrored (180 - angle).
FLIPPED_MOTORS = {0, 1}


class LegController(Node):
    def __init__(self):
        super().__init__('leg_controller')

        self._sim_counter = 0
        self.port   = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        # Sync objects created once; reused every cycle to minimise serial traffic
        self._gsr_current   = GroupSyncRead(self.port, self.packet, ADDR_PRESENT_CURRENT, 2)
        self._gsw_position  = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 4)
        for mid in MOTOR_IDS:
            self._gsr_current.addParam(mid)

        self._ready          = self._initialize()
        self._compliant      = False
        self._leg_targets    = [90] * len(MOTOR_IDS)
        self._last_leg_angles = None  # None forces the first position write

        self.create_subscription(Int32MultiArray, 'wheel_commands', self.listener_callback, 10)
        self.create_subscription(Bool, 'estop',          self._estop_callback,       10)
        self.create_subscription(Bool, 'motor_reset',    self._motor_reset_callback, 10)
        self.create_subscription(Bool, 'compliant_mode', self._compliant_callback,   10)

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
                # Torque must be off to change operating mode
                self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

                result, error = self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_CONTROL_MODE)
                if result != COMM_SUCCESS or error != 0:
                    self.get_logger().warn(
                        f'Operating mode set failed for leg motor {mid} '
                        f'({self.packet.getTxRxResult(result)} | '
                        f'{self.packet.getRxPacketError(error)}) '
                        '— running in SIMULATION mode')
                    return False

                result, error = self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                if result != COMM_SUCCESS or error != 0:
                    self.get_logger().warn(
                        f'Torque enable failed for leg motor {mid} '
                        f'({self.packet.getTxRxResult(result)} | '
                        f'{self.packet.getRxPacketError(error)}) '
                        '— running in SIMULATION mode')
                    return False

                self.packet.write2ByteTxRx(
                    self.port, mid, ADDR_GOAL_CURRENT, LEG_GOAL_CURRENT_UNITS)

            self.get_logger().info(f'Leg motors ready on {PORT} (current-based position control mode)')
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

    # ── Individual helper — used for compliant-mode hold and init ─────────────

    def _set_position(self, motor_id, angle_deg):
        angle_deg = self._normalize_angle(motor_id, angle_deg)
        position  = int(angle_deg * TICKS_PER_DEGREE)
        result, _ = self.packet.write4ByteTxRx(
            self.port, motor_id, ADDR_GOAL_POSITION, position)
        if result != COMM_SUCCESS:
            self.get_logger().warn(f'Position write failed for leg motor {motor_id}')

    # ── Batch helper — one serial packet for all four motors ──────────────────

    def _sync_set_positions(self, angles):
        self._gsw_position.clearParam()
        for i, mid in enumerate(MOTOR_IDS):
            position = int(self._normalize_angle(mid, angles[i]) * TICKS_PER_DEGREE)
            raw = position & 0xFFFFFFFF
            self._gsw_position.addParam(mid, [
                raw & 0xFF, (raw >> 8) & 0xFF,
                (raw >> 16) & 0xFF, (raw >> 24) & 0xFF,
            ])
        if self._gsw_position.txPacket() != COMM_SUCCESS:
            self.get_logger().warn('SyncWrite position failed')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _estop_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().warn('EMERGENCY STOP received — disabling leg motors')
        if self._ready:
            for mid in MOTOR_IDS:
                self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._ready = False
        else:
            self.get_logger().warn('[SIM] Emergency stop acknowledged')

    def _motor_reset_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().info('Motor reset: reconnecting leg motors')
        self._last_leg_angles = None
        all_ok = True
        for mid in MOTOR_IDS:
            self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            result, error = self.packet.write1ByteTxRx(
                self.port, mid, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_CONTROL_MODE)
            if result != COMM_SUCCESS or error != 0:
                all_ok = False
                continue
            result, error = self.packet.write1ByteTxRx(
                self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if result != COMM_SUCCESS or error != 0:
                all_ok = False
                continue
            self.packet.write2ByteTxRx(
                self.port, mid, ADDR_GOAL_CURRENT, LEG_GOAL_CURRENT_UNITS)
        if all_ok:
            self._ready = True
            self.get_logger().info('Leg motors reconnected (current-based position control mode)')
        else:
            self.get_logger().warn('Re-enable failed — retrying full init')
            try:
                self.port.closePort()
            except Exception:
                pass
            self._ready = self._initialize()

    def _compliant_callback(self, msg):
        self._compliant = msg.data
        self.get_logger().info(f'Leg compliant mode {"ON" if msg.data else "OFF"}')

    def listener_callback(self, msg):
        if len(msg.data) < 8:
            self.get_logger().warn(f'Expected 8 values, got {len(msg.data)}')
            return

        leg_angles = list(msg.data[4:8])
        self._leg_targets = leg_angles

        if self._ready and not self._compliant:
            if leg_angles != self._last_leg_angles:
                self._last_leg_angles = leg_angles
                self._sync_set_positions(leg_angles)
        elif not self._ready:
            self._sim_counter += 1
            if self._sim_counter % 20 == 0:
                ticks = [
                    int(self._normalize_angle(MOTOR_IDS[i], a) * TICKS_PER_DEGREE)
                    for i, a in enumerate(leg_angles)
                ]
                self.get_logger().info(
                    f'[SIM] leg_angles={leg_angles}°  → dynamixel_positions={ticks}')
        # In compliant mode with ready hardware, _publish_currents drives toward
        # _leg_targets and backs off under high resistance.

    def _read_present_position(self, motor_id):
        data, result, _ = self.packet.read4ByteTxRx(
            self.port, motor_id, ADDR_PRESENT_POSITION)
        return data if result == COMM_SUCCESS else None

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish_currents(self):
        msg = Int32MultiArray()
        if not self._ready:
            msg.data = [0] * len(MOTOR_IDS)
            self.currents_pub.publish(msg)
            return

        result   = self._gsr_current.txRxPacket()
        currents = []
        for i, mid in enumerate(MOTOR_IDS):
            if result == COMM_SUCCESS and \
                    self._gsr_current.isAvailable(mid, ADDR_PRESENT_CURRENT, 2):
                raw = self._gsr_current.getData(mid, ADDR_PRESENT_CURRENT, 2)
                if raw > 32767:
                    raw -= 65536
                current_ma = int(raw * CURRENT_UNIT_MA)
                currents.append(current_ma)

                if self._compliant:
                    if abs(current_ma) > COMPLIANT_CURRENT_THRESHOLD_MA:
                        # Strong resistance — freeze at current position
                        present = self._read_present_position(mid)
                        if present is not None:
                            self.packet.write4ByteTxRx(
                                self.port, mid, ADDR_GOAL_POSITION, present)
                    else:
                        # No resistance — drive toward commanded target
                        self._set_position(mid, self._leg_targets[i])
            else:
                currents.append(0)

        msg.data = currents
        self.currents_pub.publish(msg)

    # ── Shutdown ──────────────────────────────────────────────────────────────

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
                    if ch in ('\x1b', '\x03'):
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
