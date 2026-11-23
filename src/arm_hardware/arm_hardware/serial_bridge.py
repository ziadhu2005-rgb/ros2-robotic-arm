"""Hardware bridge (assignment section 5).

Subscribes to /arm_controller/joint_trajectory, converts the final point of
each trajectory from radians to servo degrees (0-180, clamped) and sends it
to the microcontroller over USB serial.

Message format (plain text, one line per command):

    <a1,a2,a3,a4,ms>\n

    a1..a4  servo angles in whole degrees, 0-180, in the order joint1..joint4
    ms      time in milliseconds the servos should take to get there

Example:  <147,119,136,107,2000>

Joint angle 0 rad (arm straight up) maps to servo 90 degrees, so
servo_deg = 90 + degrees(radians), clamped to 0-180.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

try:
    import serial
except ImportError:
    serial = None

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']


def rad_to_servo(rad):
    deg = 90.0 + math.degrees(rad)
    return int(round(max(0.0, min(180.0, deg))))


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        # dry_run: print the messages instead of opening the serial port
        self.declare_parameter('dry_run', False)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.dry_run = self.get_parameter('dry_run').value
        self.ser = None

        # Last known servo angles, so a trajectory that names only some joints
        # keeps the others where they are
        self.servo_deg = {name: 90 for name in JOINT_NAMES}

        if self.dry_run:
            self.get_logger().info('dry_run=true, printing messages instead of sending')
        elif serial is None:
            self.get_logger().error(
                'pyserial is missing, install it with: sudo apt install python3-serial')
        else:
            self.open_port()
            self.create_timer(2.0, self.reconnect)

        self.create_subscription(
            JointTrajectory, '/arm_controller/joint_trajectory',
            self.on_trajectory, 10)

    def open_port(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info(f'Opened {self.port} at {self.baud} baud')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f'Cannot open {self.port} ({e}), retrying...',
                                   throttle_duration_sec=10.0)

    def reconnect(self):
        if self.ser is None:
            self.open_port()
            return
        # Print anything the board sends back (its "OK ..." replies)
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            for line in data.decode(errors='ignore').splitlines():
                if line.strip():
                    self.get_logger().info(f'board: {line.strip()}')
        except serial.SerialException:
            self.get_logger().warn('Serial port lost')
            self.ser = None

    def on_trajectory(self, msg):
        if not msg.points:
            return
        point = msg.points[-1]
        if len(point.positions) != len(msg.joint_names):
            self.get_logger().error('positions and joint_names have different lengths')
            return

        for name, rad in zip(msg.joint_names, point.positions):
            if name in self.servo_deg:
                self.servo_deg[name] = rad_to_servo(rad)

        t = point.time_from_start
        ms = int(t.sec * 1000 + t.nanosec / 1e6)
        angles = ','.join(str(self.servo_deg[n]) for n in JOINT_NAMES)
        line = f'<{angles},{ms}>\n'

        if self.dry_run:
            self.get_logger().info(f'would send: {line.strip()}')
        elif self.ser is not None:
            try:
                self.ser.write(line.encode())
            except serial.SerialException as e:
                self.get_logger().warn(f'Write failed ({e})')
                self.ser = None


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
