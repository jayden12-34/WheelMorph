import select
import signal
import sys
import termios
import threading
import time
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
PORT = '/dev/ttyUSB1'
MOTOR_IDS = [1, 0, 2, 3]  # FL/BL physically swapped — motor 1 = FL, motor 0 = BL

ADDR_OPERATING_MODE      = 11
ADDR_CURRENT_LIMIT       = 38
ADDR_TORQUE_ENABLE       = 64
ADDR_GOAL_CURRENT        = 102
ADDR_GOAL_VELOCITY       = 104
ADDR_PRESENT_CURRENT     = 126
ADDR_PRESENT_TEMPERATURE = 146
TORQUE_ENABLE            = 1
TORQUE_DISABLE           = 0
CURRENT_CONTROL_MODE     = 0
VELOCITY_CONTROL_MODE    = 1

# Teleop sends -50..50; map to Dynamixel current units (2.69 mA/unit on XM/XH series)
# Scale of 15 → max ±750 units ≈ ±2017 mA at 100% torque on XM430-W350
CURRENT_SCALE       = 15
CURRENT_UNIT_MA     = 2.69
CURRENT_LIMIT_UNITS = 750  # = CURRENT_SCALE × 50; caps velocity mode to torque-mode max
VELOCITY_SCALE      = 7    # 50 × 7 = 350 units ≈ 80 RPM on XM430-W350

REVERSED_MOTORS = {2, 3}


class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')

        self._sim_counter = 0
        self.port   = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        # Sync objects created once; reused every cycle to minimise serial traffic
        self._gsr_current  = GroupSyncRead(self.port, self.packet, ADDR_PRESENT_CURRENT,     2)
        self._gsr_temp     = GroupSyncRead(self.port, self.packet, ADDR_PRESENT_TEMPERATURE, 1)
        self._gsw_current  = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_CURRENT,  2)
        self._gsw_velocity = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_VELOCITY, 4)
        for mid in MOTOR_IDS:
            self._gsr_current.addParam(mid)
            self._gsr_temp.addParam(mid)

        self._ready           = self._initialize()
        self._last_cmds       = None   # None forces the first write
        self._drive_mode      = CURRENT_CONTROL_MODE
        self._user_drive_mode = CURRENT_CONTROL_MODE
        self._torque_disabled = False

        self.create_subscription(Int32MultiArray, 'wheel_commands', self.listener_callback, 10)
        self.create_subscription(Bool, 'estop',        self._estop_callback,       10)
        self.create_subscription(Bool, 'motor_reset',  self._motor_reset_callback, 10)

        self.currents_pub = self.create_publisher(Int32MultiArray, 'wheel_currents', 10)
        self.temps_pub    = self.create_publisher(Int32MultiArray, 'wheel_temps',    10)
        self.create_timer(0.1, self._publish_currents)
        self.create_timer(2.0, self._publish_temps)

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
                # Torque must be disabled before changing operating mode or EEPROM registers
                self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

                cl_result, cl_error = self.packet.write2ByteTxRx(
                    self.port, mid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_UNITS)
                if cl_result != COMM_SUCCESS or cl_error != 0:
                    self.get_logger().warn(
                        f'Current Limit write failed for motor {mid} '
                        f'({self.packet.getTxRxResult(cl_result)} | '
                        f'{self.packet.getRxPacketError(cl_error)})')
                time.sleep(0.05)  # EEPROM writes need ~50 ms to settle

                result, error = self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
                if result != COMM_SUCCESS or error != 0:
                    self.get_logger().warn(
                        f'Operating mode set failed for motor {mid} '
                        f'({self.packet.getTxRxResult(result)} | '
                        f'{self.packet.getRxPacketError(error)}) '
                        '— running in SIMULATION mode')
                    return False

                result, error = self.packet.write1ByteTxRx(
                    self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                if result != COMM_SUCCESS or error != 0:
                    self.get_logger().warn(
                        f'Torque enable failed for motor {mid} — running in SIMULATION mode')
                    return False

            self.get_logger().info(f'Wheel motors ready on {PORT} (current control mode)')
            return True
        except Exception as e:
            self.get_logger().warn(
                f'Motor connection error ({e}) — wheel node running in SIMULATION mode')
            return False

    # ── Individual helpers — used only for mode-switch / estop / disable ──────

    def _set_current(self, motor_id, current):
        if motor_id in REVERSED_MOTORS:
            current = -current
        raw = int(current) & 0xFFFF
        result, _ = self.packet.write2ByteTxRx(self.port, motor_id, ADDR_GOAL_CURRENT, raw)
        if result != COMM_SUCCESS:
            self.get_logger().warn(f'Current write failed for motor {motor_id}')

    def _set_velocity(self, motor_id, velocity):
        if motor_id in REVERSED_MOTORS:
            velocity = -velocity
        raw = int(velocity) & 0xFFFFFFFF
        result, _ = self.packet.write4ByteTxRx(self.port, motor_id, ADDR_GOAL_VELOCITY, raw)
        if result != COMM_SUCCESS:
            self.get_logger().warn(f'Velocity write failed for motor {motor_id}')

    # ── Batch helpers — one serial packet for all four motors ─────────────────

    def _sync_apply_currents(self, cmds):
        self._gsw_current.clearParam()
        for i, mid in enumerate(MOTOR_IDS):
            raw = int(cmds[i] * CURRENT_SCALE * (-1 if mid in REVERSED_MOTORS else 1)) & 0xFFFF
            self._gsw_current.addParam(mid, [raw & 0xFF, (raw >> 8) & 0xFF])
        if self._gsw_current.txPacket() != COMM_SUCCESS:
            self.get_logger().warn('SyncWrite current failed')

    def _sync_apply_velocities(self, cmds):
        self._gsw_velocity.clearParam()
        for i, mid in enumerate(MOTOR_IDS):
            raw = int(cmds[i] * VELOCITY_SCALE * (-1 if mid in REVERSED_MOTORS else 1)) & 0xFFFFFFFF
            self._gsw_velocity.addParam(mid, [
                raw & 0xFF, (raw >> 8) & 0xFF,
                (raw >> 16) & 0xFF, (raw >> 24) & 0xFF,
            ])
        if self._gsw_velocity.txPacket() != COMM_SUCCESS:
            self.get_logger().warn('SyncWrite velocity failed')

    # ── Mode management ───────────────────────────────────────────────────────

    def _switch_mode(self, new_mode):
        for mid in MOTOR_IDS:
            if self._ready:
                if self._drive_mode == CURRENT_CONTROL_MODE:
                    self._set_current(mid, 0)
                else:
                    self._set_velocity(mid, 0)
                self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                self.packet.write1ByteTxRx(self.port, mid, ADDR_OPERATING_MODE, new_mode)
                self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self._drive_mode      = new_mode
        self._torque_disabled = False
        self._last_cmds       = None  # force resend in new mode
        self.get_logger().info(
            f'Wheel drive mode → {"VELOCITY" if new_mode == VELOCITY_CONTROL_MODE else "TORQUE/CURRENT"}')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _estop_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().warn('EMERGENCY STOP received — disabling wheel motors')
        if self._ready:
            for mid in MOTOR_IDS:
                self._set_current(mid, 0)
                self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._ready = False
        else:
            self.get_logger().warn('[SIM] Emergency stop — all wheel currents → 0')

    def _motor_reset_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().info('Motor reset: reconnecting wheel motors')
        self._user_drive_mode = CURRENT_CONTROL_MODE
        self._drive_mode      = CURRENT_CONTROL_MODE
        self._torque_disabled = False
        self._last_cmds       = None
        all_ok = True
        for mid in MOTOR_IDS:
            self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            cl_result, cl_error = self.packet.write2ByteTxRx(
                self.port, mid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_UNITS)
            if cl_result != COMM_SUCCESS or cl_error != 0:
                self.get_logger().warn(
                    f'Current Limit write failed for motor {mid} '
                    f'({self.packet.getTxRxResult(cl_result)} | '
                    f'{self.packet.getRxPacketError(cl_error)})')
            time.sleep(0.05)
            result, error = self.packet.write1ByteTxRx(
                self.port, mid, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
            if result != COMM_SUCCESS or error != 0:
                all_ok = False
                continue
            result, error = self.packet.write1ByteTxRx(
                self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if result != COMM_SUCCESS or error != 0:
                all_ok = False
        if all_ok:
            self._ready = True
            self.get_logger().info('Wheel motors reconnected (current control mode)')
        else:
            self.get_logger().warn('Re-enable failed — retrying full init')
            try:
                self.port.closePort()
            except Exception:
                pass
            self._ready = self._initialize()

    def listener_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn(f'Expected ≥4 values, got {len(msg.data)}')
            return

        wheel_cmds = list(msg.data[0:4])
        user_mode  = int(msg.data[8]) if len(msg.data) > 8 else CURRENT_CONTROL_MODE

        if user_mode != self._user_drive_mode:
            self._user_drive_mode = user_mode
            if self._drive_mode != user_mode:
                self._switch_mode(user_mode)

        all_zero = all(c == 0 for c in wheel_cmds)

        if self._user_drive_mode == CURRENT_CONTROL_MODE:
            if all_zero:
                if not self._torque_disabled:
                    self._torque_disabled = True
                    if self._ready:
                        for mid in MOTOR_IDS:
                            self.packet.write1ByteTxRx(
                                self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            else:
                if self._torque_disabled:
                    self._torque_disabled = False
                    self._last_cmds = None  # force resend after re-enable
                    if self._ready:
                        for mid in MOTOR_IDS:
                            self.packet.write1ByteTxRx(
                                self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                if self._ready and wheel_cmds != self._last_cmds:
                    self._last_cmds = wheel_cmds
                    self._sync_apply_currents(wheel_cmds)
        else:
            if self._ready and wheel_cmds != self._last_cmds:
                self._last_cmds = wheel_cmds
                self._sync_apply_velocities(wheel_cmds)

        if not self._ready:
            self._sim_counter += 1
            if self._sim_counter % 20 == 0:
                if self._user_drive_mode == CURRENT_CONTROL_MODE and all_zero:
                    return
                if self._user_drive_mode == CURRENT_CONTROL_MODE:
                    scale, kind = CURRENT_SCALE, 'current_units'
                else:
                    scale, kind = VELOCITY_SCALE, 'velocity_units'
                dxl = [
                    -(c * scale) if MOTOR_IDS[i] in REVERSED_MOTORS else c * scale
                    for i, c in enumerate(wheel_cmds)
                ]
                self.get_logger().info(
                    f'[SIM] wheel_cmds={wheel_cmds}  → dynamixel_{kind}={dxl}')

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_currents(self):
        msg = Int32MultiArray()
        if not self._ready:
            msg.data = [0] * len(MOTOR_IDS)
            self.currents_pub.publish(msg)
            return
        result   = self._gsr_current.txRxPacket()
        currents = []
        for mid in MOTOR_IDS:
            if result == COMM_SUCCESS and \
                    self._gsr_current.isAvailable(mid, ADDR_PRESENT_CURRENT, 2):
                raw = self._gsr_current.getData(mid, ADDR_PRESENT_CURRENT, 2)
                if raw > 32767:
                    raw -= 65536
                currents.append(int(raw * CURRENT_UNIT_MA))
            else:
                currents.append(0)
        msg.data = currents
        self.currents_pub.publish(msg)

    def _publish_temps(self):
        msg = Int32MultiArray()
        if not self._ready:
            msg.data = [0] * len(MOTOR_IDS)
            self.temps_pub.publish(msg)
            return
        result = self._gsr_temp.txRxPacket()
        temps  = []
        for mid in MOTOR_IDS:
            if result == COMM_SUCCESS and \
                    self._gsr_temp.isAvailable(mid, ADDR_PRESENT_TEMPERATURE, 1):
                temps.append(self._gsr_temp.getData(mid, ADDR_PRESENT_TEMPERATURE, 1))
            else:
                temps.append(0)
        msg.data = temps
        self.temps_pub.publish(msg)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def _disable_motors(self):
        if not self._ready:
            return
        self._ready = False
        for mid in MOTOR_IDS:
            try:
                self.packet.write2ByteTxRx(self.port, mid, ADDR_GOAL_CURRENT, 0)
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
                    if ch in ('\x1b', '\x03'):
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
