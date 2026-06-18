from threading import Lock

from scipy.spatial.transform import Rotation

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import numpy as np
from PyQt6.QtCore import Qt
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import matplotlib

matplotlib.use("QtAgg")
plt.style.use("dark_background")

from geometry_msgs.msg import PoseStamped

# ros2 topic pub /robots_poses geometry_msgs/msg/PoseStamped "{header: {stamp: now, frame_id: robot_1}, pose: {position: {x: -1, y: 1, z: 1}, orientation: {x: 1, y: 0, z: 0, w: 1}}}"

class Viewer2D(Node):
    def __init__(self):
        super().__init__('viewer2d')
        self.auto_follow = True
        self.record_trajectory = True
        self.trajectory = []
        self.texts = {}
        self._robot_pos = {}
        self._lock_pos = Lock()
        self._time_init = self.get_clock().now()

        self.x_lims = (-10, 10)
        self.y_lims = (-10, 10)

        self.fig, tmp_ax = plt.subplots(figsize=(7, 5))
        self.ax = self.fig.add_axes((0.04, 0.04, 0.96, 0.96))
        tmp_ax.set_axis_off()

        mngr = plt.get_current_fig_manager()
        window = mngr.window
        window.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint, False)
        self.fig.canvas.mpl_connect("close_event", lambda event: print("Bye!"))

        ax_button_trajectory = self.fig.add_axes((0.7, 0.05, 0.2, 0.025))
        self.button_trajectory = Button(ax_button_trajectory, "Show/Hide trajectory")
        self.button_trajectory.on_clicked(self.toggle_trajectory)
        ax_button_follow = self.fig.add_axes((0.1, 0.05, 0.2, 0.025))
        self.button_follow = Button(ax_button_follow, "Auto follow")
        self.button_follow.on_clicked(lambda event: setattr(self, "auto_follow", not self.auto_follow))

        self.fig.show()
        self.timer = self.create_timer(0.01, self._spin_once)  # Update every 10 ms

        self.subscription = self.create_subscription(
            PoseStamped,
            'robots_poses',
            self.callback_robot_pos,
            10)

    def _spin_once(self):
        self._draw()

    def _draw(self):
        if not plt.fignum_exists(self.fig.number):
            self.executor.shutdown(timeout_sec=0)  # signals executor.spin() to return

        if not self.record_trajectory:
            self.ax.clear()

        with self._lock_pos:
            robot_pos_snapshot = dict(self._robot_pos)

        for robot_id, pos in robot_pos_snapshot.items():
            if self.record_trajectory and robot_id in self.texts.keys():
                self.texts[robot_id].remove()
            self.texts[robot_id] = self.ax.text(pos[0], pos[1] + 0.3, robot_id)
            self.ax.arrow(
                pos[0],
                pos[1],
                float(0.05 * np.cos(pos[2])),
                float(0.05 * np.sin(pos[2])),
                head_width=0.3,
                head_length=1,
                linewidth=0,
                fc=matplotlib.colors.hsv_to_rgb(
                    (((self.get_clock().now() - self._time_init).nanoseconds * 1e-9 % 60) / 60, 1, 1)
                ),
            )

        if robot_pos_snapshot:
            cam_pos = robot_pos_snapshot[sorted(robot_pos_snapshot.keys())[0]]
            if self.auto_follow:
                self.ax.set(
                    xlim=(cam_pos[0] + self.x_lims[0], cam_pos[0] + self.x_lims[1]),
                    ylim=(cam_pos[1] + self.y_lims[0], cam_pos[1] + self.y_lims[1]),
                )
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


    def callback_robot_pos(self, msg: PoseStamped):
        header = msg.header
        robot_id = header.frame_id
        x, y = msg.pose.position.x, msg.pose.position.y
        quat = msg.pose.orientation
        theta = Rotation.from_quat((quat.x, quat.y, quat.z, quat.w)).as_euler('ZYX', degrees=False)[0]
        with self._lock_pos:
            self._robot_pos[robot_id] = np.array([x, y, theta], dtype=float)

    def toggle_trajectory(self, event):
        self.record_trajectory = not self.record_trajectory
        if not self.record_trajectory:
            self.trajectory = []


def main(args=None):
    rclpy.init(args=args)
    viewer = Viewer2D()
    executor = SingleThreadedExecutor()
    executor.add_node(viewer)
    try:
        executor.spin()          # blocks until executor.shutdown() is called
    except KeyboardInterrupt:
        pass
    finally:                     # single, guaranteed cleanup point
        viewer.destroy_node()
        rclpy.shutdown()