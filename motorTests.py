import os
import sys
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import keyboard
import serial.tools.list_ports

# Control table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 111
ADDR_LED = 65  # LED address (model-dependent; change if different)

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
BAUDRATE = 57600
VELOCITY_PORT = '/dev/ttyUSB0'  # Velocity-based U2D2
POSITION_PORT = '/dev/ttyUSB1'  # Position-based U2D2
MOTOR_IDS = [0, 1, 2, 3]   # Put all motor IDs you want to control here
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Additional control table addresses for position mode
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Initialize PortHandler and PacketHandler
velocity_port_handler = PortHandler(VELOCITY_PORT)
position_port_handler = PortHandler(POSITION_PORT)
velocity_packet_handler = PacketHandler(PROTOCOL_VERSION)
position_packet_handler = PacketHandler(PROTOCOL_VERSION)

class MotorController:
    def __init__(self, port, packet, motor_ids):
        self.port = port
        self.packet = packet
        self.ids = list(motor_ids)

    def initialize(self):
        try:
            if not self.port.openPort():
                print("Failed to open port. Check COM name and whether another app is using it.")
                print("Available serial ports:", [p.device for p in serial.tools.list_ports.comports()])
                return False
        except PermissionError as e:
            port_name = getattr(self.port, 'getPortName', lambda: str(self.port))()
            print(f"Permission denied when opening port {port_name}: {e}. Try closing other apps that may use the port or run the script as administrator.")
            print("Available serial ports:", [p.device for p in serial.tools.list_ports.comports()])
            return False
        except Exception as e:
            print(f"Exception opening port: {e}")
            print("Available serial ports:", [p.device for p in serial.tools.list_ports.comports()])
            return False

        if not self.port.setBaudRate(BAUDRATE):
            print("Failed to set baudrate")
            return False

        for mid in self.ids:
            # Ping motor first to verify presence
            try:
                model_num, dxl_comm_result, dxl_error = self.packet.ping(self.port, mid)
            except Exception as e:
                print(f"Ping exception for motor {mid}: {e}")
                return False
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print(f"Motor {mid} ping failed: {self.packet.getTxRxResult(dxl_comm_result)} | {self.packet.getRxPacketError(dxl_error)}")
                return False

            # Try enabling torque with retries and verbose diagnostics
            success = False
            for attempt in range(3):
                dxl_comm_result, dxl_error = self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                    success = True
                    break
                print(f"Attempt {attempt+1} failed for motor {mid}: {self.packet.getTxRxResult(dxl_comm_result)} | {self.packet.getRxPacketError(dxl_error)}")
                time.sleep(0.1)
            if not success:
                print(f"Failed to enable torque for motor {mid} after retries")
                return False

        print(f"Initialized motors: {self.ids}")

        value, dxl_comm_result, dxl_error = self.packet.read1ByteTxRx(self.port, 2, ADDR_TORQUE_ENABLE)
        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0 and value == TORQUE_ENABLE:
            print("Motor 2 torque is ENABLED")
        else:
            print("Motor 2 torque NOT enabled or read failed:",
                self.packet.getTxRxResult(dxl_comm_result),
                self.packet.getRxPacketError(dxl_error), "value=", value)

        return True

    def set_velocity(self, motor_id, velocity):
        dxl_comm_result, dxl_error = self.packet.write4ByteTxRx(self.port, motor_id, ADDR_GOAL_VELOCITY, int(velocity))
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to set velocity for {motor_id}: {self.packet.getRxPacketError(dxl_error)}")

    def set_position(self, motor_id, position):
        position = int(max(0, min(position, 4095)))
        dxl_comm_result, dxl_error = self.packet.write4ByteTxRx(self.port, motor_id, ADDR_GOAL_POSITION, position)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to set position for {motor_id}: {self.packet.getRxPacketError(dxl_error)}")

    def set_velocity_all(self, velocity):
        for mid in self.ids:
            self.set_velocity(mid, velocity)

    def set_position_all(self, position):
        for mid in self.ids:
            self.set_position(mid, position)

    def set_led(self, motor_id, value):
        dxl_comm_result, dxl_error = self.packet.write1ByteTxRx(self.port, motor_id, ADDR_LED, int(value))
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f"Failed to set LED for {motor_id}: {self.packet.getTxRxResult(dxl_comm_result)} | {self.packet.getRxPacketError(dxl_error)}")

    def set_led_all(self, value):
        for mid in self.ids:
            self.set_led(mid, value)

    def disable_all(self):
        for mid in self.ids:
            self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.port.closePort()
        print("Motors disabled and port closed")


def control_loop(velocity_controller: MotorController, position_controller: MotorController):
    velocities = {mid: 0 for mid in velocity_controller.ids}
    directions = {mid: 1 for mid in velocity_controller.ids}
    leds = {mid: 0 for mid in velocity_controller.ids}  # 0=Off, 1=On
    position_targets = {mid: 2048 for mid in position_controller.ids}
    speed_step = 10
    pos_step = 50
    selected_index = 0
    broadcast = True  # When True, changes apply to all motors; toggle with 'b'

    print("Controls:")
    print("  TAB    : cycle selected motor")
    print("  b      : toggle broadcast (all motors) / single motor")
    print("  UP/DOWN: set velocity direction for selected (or all if broadcast)")
    print("  LEFT/RIGHT: decrease/increase velocity")
    print("  SPACE  : stop selected/all velocity motors")
    print("  p      : toggle LED for selected (or all if broadcast)")
    print("  z/x    : COM11 id0 retract/extend")
    print("  a/s    : COM11 id1 retract/extend")
    print("  k/l    : COM11 id2 retract/extend")
    print("  n/m    : COM11 id3 retract/extend")
    print("  ESC    : exit")

    last_print = ""
    try:
        while True:
            if keyboard.is_pressed('tab'):
                selected_index = (selected_index + 1) % len(velocity_controller.ids)
                time.sleep(0.2)

            if keyboard.is_pressed('b'):
                broadcast = not broadcast
                print(f"Broadcast mode: {broadcast}")
                time.sleep(0.2)

            selected_id = velocity_controller.ids[selected_index]

            if keyboard.is_pressed('up'):
                if broadcast:
                    for mid in velocity_controller.ids:
                        directions[mid] = 1
                else:
                    directions[selected_id] = 1
                print(f"Velocity direction set to Forward (selected: {selected_id})")
                time.sleep(0.05)

            if keyboard.is_pressed('down'):
                if broadcast:
                    for mid in velocity_controller.ids:
                        directions[mid] = -1
                else:
                    directions[selected_id] = -1
                print(f"Velocity direction set to Backward (selected: {selected_id})")
                time.sleep(0.05)

            if keyboard.is_pressed('right'):
                if broadcast:
                    for mid in velocity_controller.ids:
                        velocities[mid] = min(velocities[mid] + speed_step, 1023)
                else:
                    velocities[selected_id] = min(velocities[selected_id] + speed_step, 1023)
                time.sleep(0.05)

            if keyboard.is_pressed('left'):
                if broadcast:
                    for mid in velocity_controller.ids:
                        velocities[mid] = max(velocities[mid] - speed_step, 0)
                else:
                    velocities[selected_id] = max(velocities[selected_id] - speed_step, 0)
                time.sleep(0.05)

            if keyboard.is_pressed('space'):
                if broadcast:
                    for mid in velocity_controller.ids:
                        velocities[mid] = 0
                else:
                    velocities[selected_id] = 0
                print("Stopped velocity motors")
                time.sleep(0.1)

            if keyboard.is_pressed('p'):
                if broadcast:
                    for mid in velocity_controller.ids:
                        leds[mid] = 0 if leds.get(mid, 0) else 1
                        velocity_controller.set_led(mid, leds[mid])
                else:
                    leds[selected_id] = 0 if leds.get(selected_id, 0) else 1
                    velocity_controller.set_led(selected_id, leds[selected_id])
                print("LEDs: " + ", ".join([f"M{m}:{'On' if leds[m] else 'Off'}" for m in velocity_controller.ids]))
                time.sleep(0.2)

            if keyboard.is_pressed('z'):
                position_targets[0] = max(0, position_targets[0] - pos_step)
                position_controller.set_position(0, position_targets[0])
                time.sleep(0.05)
            if keyboard.is_pressed('x'):
                position_targets[0] = min(4095, position_targets[0] + pos_step)
                position_controller.set_position(0, position_targets[0])
                time.sleep(0.05)
            if keyboard.is_pressed('a'):
                position_targets[1] = max(0, position_targets[1] - pos_step)
                position_controller.set_position(1, position_targets[1])
                time.sleep(0.05)
            if keyboard.is_pressed('s'):
                position_targets[1] = min(4095, position_targets[1] + pos_step)
                position_controller.set_position(1, position_targets[1])
                time.sleep(0.05)
            if keyboard.is_pressed('k'):
                position_targets[2] = max(0, position_targets[2] - pos_step)
                position_controller.set_position(2, position_targets[2])
                time.sleep(0.05)
            if keyboard.is_pressed('l'):
                position_targets[2] = min(4095, position_targets[2] + pos_step)
                position_controller.set_position(2, position_targets[2])
                time.sleep(0.05)
            if keyboard.is_pressed('n'):
                position_targets[3] = max(0, position_targets[3] - pos_step)
                position_controller.set_position(3, position_targets[3])
                time.sleep(0.05)
            if keyboard.is_pressed('m'):
                position_targets[3] = min(4095, position_targets[3] + pos_step)
                position_controller.set_position(3, position_targets[3])
                time.sleep(0.05)

            if keyboard.is_pressed('esc'):
                break

            for mid in velocity_controller.ids:
                if mid == 0 or mid == 3:
                    goal = velocities[mid] * directions[mid] * -1
                else:
                    goal = velocities[mid] * directions[mid]
                velocity_controller.set_velocity(mid, goal)

            status = (
                f"Selected: {selected_id} | Broadcast: {broadcast} | "
                + ", ".join([f"M{m}:{velocities[m] * directions[m]}" for m in velocity_controller.ids])
                + " | POS: "
                + ", ".join([f"M{m}:{position_targets[m]}" for m in position_controller.ids])
            )
            if status != last_print:
                print(status)
                last_print = status

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    velocity_controller = MotorController(velocity_port_handler, velocity_packet_handler, MOTOR_IDS)
    position_controller = MotorController(position_port_handler, position_packet_handler, MOTOR_IDS)
    velocity_ready = velocity_controller.initialize()
    position_ready = position_controller.initialize()

    if not velocity_ready or not position_ready:
        if velocity_ready:
            velocity_controller.disable_all()
        if position_ready:
            position_controller.disable_all()
        sys.exit(1)

    try:
        control_loop(velocity_controller, position_controller)
    finally:
        velocity_controller.disable_all()
        position_controller.disable_all()