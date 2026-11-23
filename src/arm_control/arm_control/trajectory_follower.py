"""Follows /arm_controller/joint_trajectory in Gazebo.

Subscribes to trajectory_msgs/JointTrajectory, linearly interpolates between
the trajectory points over their time_from_start, and publishes the result as
std_msgs/Float64 position commands on /arm/<joint>/cmd_pos, which
ros_gz_bridge forwards to the Gazebo JointPositionController plugins.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']

# Same limits as in arm.urdf (radians)
JOINT_LIMITS = {
    'joint1': (-3.14, 3.14),
    'joint2': (-1.57, 1.57),
    'joint3': (-1.57, 1.57),
    'joint4': (-1.57, 1.57),
}

RATE_HZ = 50.0


def clamp(value, low, high):
    return max(low, min(high, value))


class TrajectoryFollower(Node):

    def __init__(self):
        super().__init__('trajectory_follower')

        self.cmd_pubs = {
            name: self.create_publisher(Float64, f'/arm/{name}/cmd_pos', 10)
            for name in JOINT_NAMES
        }
        self.create_subscription(
            JointTrajectory, '/arm_controller/joint_trajectory',
            self.on_trajectory, 10)

        # Last commanded position of every joint
        self.current = {name: 0.0 for name in JOINT_NAMES}

        # Active trajectory as a list of (time_in_seconds, {joint: position})
        self.waypoints = []
        self.start_time = None

        self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info('Waiting for /arm_controller/joint_trajectory ...')

    def on_trajectory(self, msg):
        unknown = [n for n in msg.joint_names if n not in self.current]
        if unknown:
            self.get_logger().warn(f'Ignoring unknown joints: {unknown}')

        # Start from where the arm is now, then add every point of the message
        waypoints = [(0.0, dict(self.current))]
        for point in msg.points:
            if len(point.positions) != len(msg.joint_names):
                self.get_logger().error(
                    'positions and joint_names have different lengths, ignoring message')
                return
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            target = dict(waypoints[-1][1])
            for name, pos in zip(msg.joint_names, point.positions):
                if name in target:
                    low, high = JOINT_LIMITS[name]
                    target[name] = clamp(pos, low, high)
            waypoints.append((max(t, waypoints[-1][0]), target))

        self.waypoints = waypoints
        self.start_time = self.get_clock().now()
        self.get_logger().info(
            f'New trajectory with {len(msg.points)} point(s), '
            f'final target {self.waypoints[-1][1]}')

    def on_timer(self):
        if self.waypoints:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            self.current = self.interpolate(elapsed)
            if elapsed >= self.waypoints[-1][0]:
                self.waypoints = []

        for name, pub in self.cmd_pubs.items():
            pub.publish(Float64(data=self.current[name]))

    def interpolate(self, t):
        if t >= self.waypoints[-1][0]:
            return dict(self.waypoints[-1][1])
        for (t0, p0), (t1, p1) in zip(self.waypoints, self.waypoints[1:]):
            if t0 <= t < t1:
                a = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return {n: p0[n] + a * (p1[n] - p0[n]) for n in p0}
        return dict(self.waypoints[0][1])


def main():
    rclpy.init()
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
