"""Publisher node (assignment section 4.2).

Publishes trajectory_msgs/JointTrajectory messages to
/arm_controller/joint_trajectory, cycling through a list of demo poses.
"""

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']

# Demo poses in radians: [joint1, joint2, joint3, joint4]
POSES = [
    [0.0, 0.0, 0.0, 0.0],      # straight up
    [1.0, 0.5, 0.8, 0.3],      # turn left and reach forward
    [-1.0, 0.5, 0.8, 0.3],     # turn right and reach forward
    [0.0, -0.6, -0.9, -0.5],   # lean backwards
    [0.0, 0.9, 1.2, 0.6],      # reach down in front
]


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        self.declare_parameter('move_time', 2.0)  # seconds to reach each pose
        self.declare_parameter('period', 3.0)     # seconds between poses

        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.index = 0
        period = self.get_parameter('period').value
        self.create_timer(period, self.publish_next)

    def publish_next(self):
        move_time = self.get_parameter('move_time').value
        positions = POSES[self.index]

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(move_time), nanosec=int((move_time % 1) * 1e9))

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        msg.points = [point]

        self.pub.publish(msg)
        self.get_logger().info(f'Pose {self.index}: {positions}')
        self.index = (self.index + 1) % len(POSES)


def main():
    rclpy.init()
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
