import time

from scipy.spatial.transform import Rotation

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
from numpy import cos, sin, pi
from geometry_msgs.msg import PoseStamped, TwistStamped

from arena_2d.viewer import qos_poses, qos_shutdown
from arena_2d.simulator import sawtooth, qos_efforts
from std_msgs.msg import Empty


class GroundTruth(Node):
    def __init__(self):
        super().__init__('ground_truth')

        self.declare_parameter("mass", 20,
                               ParameterDescriptor(
                                   description="Mass (kg) of the simulated robot."))
        self.declare_parameter("inertia", 20,
                               ParameterDescriptor(
                                   description="Inertia (kg⋅m) of the simulated robot."))
        self.declare_parameter("Fx", 1,
                               ParameterDescriptor(
                                   description="Longitudinal force applied to the robot during launching."))
        self.declare_parameter("Mz", 1,
                               ParameterDescriptor(
                                   description="Turning torque applied to the robot during launching."))
        self.declare_parameter("launch_duration", 1,
                               ParameterDescriptor(
                                   description="Duration in seconds during which force and torque are applied before "
                                               "going to zero."))
        self.declare_parameter("dt", 0.1,
                               ParameterDescriptor(
                                   description="Time step between two consecutive robot positions."))
        self.declare_parameter("start_pose", [0.0, 0.0, 0.0],
                               ParameterDescriptor(
                                   description="""Starting pose (x, y, θ) of the simulated robot."""))
        self._mass = self.get_parameter("mass").value
        self._inertia = self.get_parameter("inertia").value
        self._F = self.get_parameter("Fx").value
        self._M = self.get_parameter("Mz").value
        self._T = self.get_parameter("launch_duration").value
        self._mu = np.sqrt(self._mass / pi * self._inertia)

        # x, y, θ in global frame, v, vθ
        self.starting_pose = self.get_parameter('start_pose').value
        self._X = np.hstack((self.starting_pose, np.zeros(2)))

        self._time_init = None
        self.trajectory = []

        self.publisher_poses_ = self.create_publisher(PoseStamped,
                                                'robots_poses',
                                                      qos_poses)

        self.publisher_commands_ = self.create_publisher(TwistStamped,
                                                         'robots_efforts',
                                                         qos_efforts
                                                    )

        self.subscription_shutdown_ = self.create_subscription(
            Empty,
            '/shutdown',
            self.callback_shutdown,
            qos_shutdown
        )

        self.timer_main = self.create_timer(self.get_parameter('dt').value, self._spin_once)
        self.timer_command = self.create_timer(self.get_parameter("launch_duration").value,
                                               lambda:self.publish_command(0, 0))

        time.sleep(0.5)
        self.publish_command(self._F, self._M)


    def _spin_once(self):
        self._move()

    def _move(self):
        if self._time_init is None:
            self._time_init = self.get_clock().now()
            self.timer_command.reset()
        t = (self.get_clock().now() - self._time_init).nanoseconds * 1e-9

        self._X[:2] = self.position(t)
        self._X[2] = self.heading(t)
        self._X[3] = self.linvel(t)
        self._X[4] = self.rotvel(t)

        self.publish_pose()

    def heading(self, t):
        if t <= self._T:
            theta0 = self.starting_pose[2]
            res = theta0 + self._M / (2 * self._inertia) * t ** 2
        else:
            res = self.heading(self._T) + (self._M * self._T) / self._inertia * (t - self._T)
        return sawtooth(res)

    def position(self, t):
        x0 = self.starting_pose[0]
        y0 = self.starting_pose[1]
        theta0 = self.starting_pose[2]
        theta = self.heading(t)
        k = self._F * self._inertia / (self._mass * self._M)
        x = x0 + k * (sin(theta) - sin(theta0))
        y = y0 + k * (cos(theta0) - cos(theta))
        return x, y

    def linvel(self, t):
        return self._F / self._mass * min(t, self._T)

    def rotvel(self, t):
        return self._M / self._inertia * min(t, self._T)


    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_name()
        msg.pose.position.x, msg.pose.position.y = self._X[0:2]
        quat = Rotation.from_euler("Z", self._X[2]).as_quat()
        quat_msg = msg.pose.orientation
        quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w = quat
        self.publisher_poses_.publish(msg)

    def publish_command(self, fx, mz):
        if fx == 0 and mz == 0:
            self.timer_command.cancel()
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(fx)
        msg.twist.angular.z = float(mz)
        self.publisher_commands_.publish(msg)

    def callback_shutdown(self, msg):
        # If this doesn't work correctly, just set a flag here and close in the main loop
        self.get_logger().info('Shutdown signal received')
        self.executor.shutdown(timeout_sec=0)  # signals executor.spin() to return

def main(args=None):
    rclpy.init(args=args)
    viewer = GroundTruth()
    executor = SingleThreadedExecutor()
    executor.add_node(viewer)
    try:
        executor.spin()  # blocks until executor.shutdown() is called
    except KeyboardInterrupt:
        pass
    finally:  # single, guaranteed cleanup point
        viewer.destroy_node()
        rclpy.shutdown()