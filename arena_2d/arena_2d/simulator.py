from scipy.spatial.transform import Rotation

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import numpy as np
from numpy import cos, sin, pi
from geometry_msgs.msg import TwistStamped, PoseStamped

from arena_2d.viewer import qos_poses, qos_shutdown
from std_msgs.msg import Empty


def sawtooth(x, limit_low=-np.pi, limit_high=np.pi):
    return (x - limit_low) % (limit_high - limit_low) + limit_low

qos_efforts = QoSProfile(
                # reliability=ReliabilityPolicy.RELIABLE,  # TCP-like
                reliability=ReliabilityPolicy.BEST_EFFORT,  # UDP-like
                durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Keep messages for late joiners
                history=HistoryPolicy.KEEP_LAST,  # Keep recent messages history
                depth=1  # Keep only the latest message
            )

class PlanarRobot(Node):
    def __init__(self):
        super().__init__('simulator_planar')

        self.declare_parameter('water_flow', [0.0, -3 * pi / 4],
                               ParameterDescriptor(
                                   description="""Simulate drift of a robot submerged in water.
                                   (m/s, rad), speed and direction in local reference frame 
                                   (initialised at starting position, positive CCW))"""))
        self.declare_parameter("mass", 20,
                               ParameterDescriptor(
                                   description="""Mass (kg) of the simulated robot."""))
        self.declare_parameter("inertia", 20,
                               ParameterDescriptor(
                                   description="""Inertia (kg⋅m) of the simulated robot."""))
        self.declare_parameter("drag_coefficients", [0.0, 0.0, 0.0],
                               ParameterDescriptor(
                                   description="""Drag coefficients of the simulated robot."""))
        self.declare_parameter("start_pose", [0.0, 0.0, 0.0],
                               ParameterDescriptor(
                                   description="""Starting pose (x, y, θ) of the simulated robot."""))
        self.declare_parameter("dt", 0.1,
                               ParameterDescriptor(
                                   description="Time step between two consecutive robot positions."))
        self._mass = self.get_parameter("mass").value
        self._inertia = self.get_parameter("inertia").value
        self._drag_coefficients = self.get_parameter("drag_coefficients").value

        # x, y, θ in global frame, vx, vy, vθ, ax, ay, aθ in robot frame
        self._X = np.hstack((np.array(self.get_parameter('start_pose').value), np.zeros(6)))
        self._command = np.zeros(9)

        self._time_last_move = None
        self._A = None

        self.trajectory = []

        self.publisher_ = self.create_publisher(PoseStamped,
                                                'robots_poses',
                                                qos_poses)

        self.subscription_shutdown_ = self.create_subscription(
            Empty,
            '/shutdown',
            self.callback_shutdown,
            qos_shutdown
        )

        self.subscription_commands_ = self.create_subscription(
            TwistStamped,
            'robots_efforts',
            self.callback_command,
            # 1
            qos_efforts
        )

        self.timer = self.create_timer(self.get_parameter('dt').value,
                                       self._spin_once)  # Update every 10 ms

    def _spin_once(self):
        self._move()

    def _compute_evolution_matrix(self, dt):
        θ = self._X[2]
        self._A = np.array(
            [
                [1, 0, 0, cos(θ) * dt, -sin(θ) * dt, 0, 0, 0, 0],
                [0, 1, 0, sin(θ) * dt, cos(θ) * dt, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, dt, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, dt, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, dt, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, dt],
                [0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0],
            ]
        )

    def _compute_efforts_drag(self, dt):
        Cx, Cy, Cθ = self.get_parameter('drag_coefficients').value
        surface_current = self.get_parameter('water_flow').value
        θ = self._X[2]
        vx = self._X[3]
        vy = self._X[4]
        vθ = self._X[5]
        vel_current = (Rotation.from_euler("Z", θ).inv() * Rotation.from_euler("Z", surface_current[1])).apply(
            [1, 0, 0]
        ) * surface_current[0]
        drag = np.array(
            [
                0,
                0,
                0,
                0,
                0,
                0,
                -Cx * abs(vx - vel_current[0]) * (vx - vel_current[0]) / self._mass,
                -Cy * abs(vy - vel_current[1]) * (vy - vel_current[1]) / self._mass,
                -Cθ * abs(vθ) * vθ / self._inertia,
            ]
        )
        return drag

    def callback_command(self, msg: TwistStamped):
        # Forces in robot's frame
        fx = msg.twist.linear.x
        mz = msg.twist.angular.z
        self.get_logger().info(f'Command received {fx=} {mz=}')
        self._command = np.array([0, 0, 0, 0, 0, 0, fx / self._mass, 0, mz / self._inertia])
        self._spin_once() # To avoid delay, we don't want to wait for the timer to fire

    def _move(self):
        now = self.get_clock().now()
        if self._time_last_move is None:
            self._time_last_move = now
        dt = (now - self._time_last_move).nanoseconds * 1e-9
        self._time_last_move = now
        self._compute_evolution_matrix(dt)
        drag = self._compute_efforts_drag(dt)
        self._X = self._A @ self._X + self._command + drag
        self._X[2] = sawtooth(self._X[2])
        self.publish_pose()

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_name()
        msg.pose.position.x, msg.pose.position.y = self._X[0:2]
        quat = Rotation.from_euler("Z", self._X[2]).as_quat()
        quat_msg = msg.pose.orientation
        quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w = quat
        self.publisher_.publish(msg)

    def callback_shutdown(self, msg):
        # If this doesn't work correctly, just set a flag here and close in the main loop
        self.get_logger().info('Shutdown signal received')
        self.executor.shutdown(timeout_sec=0)  # signals executor.spin() to return

def main(args=None):
    rclpy.init(args=args)
    viewer = PlanarRobot()
    executor = SingleThreadedExecutor()
    executor.add_node(viewer)
    try:
        executor.spin()  # blocks until executor.shutdown() is called
    except KeyboardInterrupt:
        pass
    finally:  # single, guaranteed cleanup point
        viewer.destroy_node()
        rclpy.shutdown()