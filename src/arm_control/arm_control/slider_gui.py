"""Slider GUI (assignment section 5.5).

One slider per joint (in degrees). Moving a slider publishes a
JointTrajectory with all four joints to /arm_controller/joint_trajectory,
which moves Gazebo, RViz2 and the real servos at the same time.
"""

import math
import tkinter as tk

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# (joint name, min degrees, max degrees) - same limits as arm.urdf
JOINTS = [
    ('joint1', -180, 180),
    ('joint2', -90, 90),
    ('joint3', -90, 90),
    ('joint4', -90, 90),
]

MOVE_TIME = 0.5     # seconds the arm takes to reach a slider value
SEND_EVERY_MS = 100  # at most 10 messages per second while dragging


class SliderGui(Node):

    def __init__(self):
        super().__init__('slider_gui')
        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.root = tk.Tk()
        self.root.title('Arm joint control')
        self.vars = {}
        self.dirty = False

        for row, (name, low, high) in enumerate(JOINTS):
            tk.Label(self.root, text=name, width=8).grid(row=row, column=0, padx=6)
            var = tk.DoubleVar(value=0.0)
            tk.Scale(self.root, variable=var, from_=low, to=high, resolution=1,
                     orient=tk.HORIZONTAL, length=320, label='degrees',
                     command=lambda _value: self.mark_dirty()
                     ).grid(row=row, column=1, padx=6, pady=4)
            self.vars[name] = var

        tk.Button(self.root, text='Home (all 0)', command=self.home
                  ).grid(row=len(JOINTS), column=0, columnspan=2, pady=8)

        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.root.after(SEND_EVERY_MS, self.tick)

    def mark_dirty(self):
        self.dirty = True

    def home(self):
        for var in self.vars.values():
            var.set(0.0)
        self.dirty = True

    def tick(self):
        if self.dirty:
            self.dirty = False
            self.publish()
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(SEND_EVERY_MS, self.tick)

    def publish(self):
        point = JointTrajectoryPoint()
        point.positions = [math.radians(self.vars[n].get()) for n, _, _ in JOINTS]
        point.time_from_start = Duration(
            sec=int(MOVE_TIME), nanosec=int((MOVE_TIME % 1) * 1e9))

        msg = JointTrajectory()
        msg.joint_names = [n for n, _, _ in JOINTS]
        msg.points = [point]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SliderGui()
    try:
        node.root.mainloop()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
