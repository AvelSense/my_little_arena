import sys
import time
import math
import signal
from threading import Lock, Thread

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import pyqtgraph as pg
# from pyqtgraph.Qt import QtGui
from pyqtgraph import functions as fn
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QGraphicsItem
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QColor, QTransform

qos_poses = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=2
)

qos_shutdown = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


class Viewer2DNode(Node):
    """ROS 2 Node that runs in a background thread, purely handling incoming messages."""

    def __init__(self):
        super().__init__('viewer2d_ros')
        self.robot_pos = {}
        self.lock_pos = Lock()
        self.sub = self.create_subscription(PoseStamped, 'robots_poses', self.callback_robot_pos, qos_poses)
        self.pub_shutdown = self.create_publisher(Empty, '/shutdown', qos_shutdown)

    def callback_robot_pos(self, msg: PoseStamped):
        robot_id = msg.header.frame_id
        with self.lock_pos:
            self.robot_pos[robot_id] = msg

    def system_shutdown(self):
        self.pub_shutdown.publish(Empty())
        time.sleep(0.2)


class ViewerApp(QMainWindow):
    """PyQt6 GUI that runs in the main thread and polls the ROS node state."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self._time_init = time.time()

        self.auto_follow = True
        self.record_trajectory = False

        self.x_lims = (-10, 10)
        self.y_lims = (-10, 10)

        # Plot state variables
        self.color_offsets = {}
        self.robot_items = {}  # Store PyQtGraph graphical items per robot
        self.trajectories = {}  # Store trajectory paths

        self.init_ui()

        # Update the GUI at 100 Hz (10 ms)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)

    def init_ui(self):
        self.setWindowTitle('Fast ROS2 2D Viewer (PyQtGraph)')
        self.resize(800, 600)

        # Make paths and arrows render smoothly
        pg.setConfigOptions(antialias=True)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 1. Setup the high-performance Plot Widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setAspectLocked(True)  # Equivalent to ax.set_aspect("equal")
        self.plot_widget.setLabel('bottom', "X")
        self.plot_widget.setLabel('left', "Y")
        layout.addWidget(self.plot_widget)

        # 2. Setup the interactive buttons
        btn_layout = QHBoxLayout()
        self.btn_follow = QPushButton("Auto follow: ON")
        self.btn_follow.clicked.connect(self.toggle_follow)

        self.btn_traj = QPushButton("Show/Hide trajectory: OFF")
        self.btn_traj.clicked.connect(self.toggle_traj)

        btn_layout.addWidget(self.btn_follow)
        btn_layout.addWidget(self.btn_traj)
        layout.addLayout(btn_layout)

    def toggle_follow(self):
        self.auto_follow = not self.auto_follow
        self.btn_follow.setText(f"Auto follow: {'ON' if self.auto_follow else 'OFF'}")
        if not self.auto_follow:
            # Unlock the view limit and let the user pan/zoom manually
            self.plot_widget.enableAutoRange(axis=pg.ViewBox.XYAxes)

    def toggle_traj(self):
        self.record_trajectory = not self.record_trajectory
        self.btn_traj.setText(f"Show/Hide trajectory: {'ON' if self.record_trajectory else 'OFF'}")

        # Instantly clear trajectories when toggled off
        if not self.record_trajectory:
            for r_id, items in self.robot_items.items():
                self.trajectories[r_id] = ([], [])
                items['path'].setData([], [])

    def closeEvent(self, event):
        """Trigger ROS shutdown when closing the GUI window."""
        self.ros_node.system_shutdown()
        self.ros_node.executor.shutdown(timeout_sec=0)
        event.accept()

    def update_plot(self):
        # Safely copy the latest poses from the ROS node
        with self.ros_node.lock_pos:
            snapshot = dict(self.ros_node.robot_pos)

        for robot_id, msg in snapshot.items():
            # Initialize newly discovered robots
            if robot_id not in self.color_offsets:
                self.color_offsets[robot_id] = np.random.rand()
                color = QColor.fromHsvF(self.color_offsets[robot_id], 1.0, 1.0)

                # Create graphical items once (Line, Arrow, Text)
                path = pg.PlotDataItem(pen=pg.mkPen(color, width=2))
                arrow = ArrowFromBaseItem(brush=color, pen='w', tailLen=10, tailWidth=3, headLen=15)
                text = pg.TextItem(robot_id, color='w', anchor=(0.5, 1.5))

                self.plot_widget.addItem(path)
                self.plot_widget.addItem(arrow)
                self.plot_widget.addItem(text)

                self.robot_items[robot_id] = {'arrow': arrow, 'text': text, 'path': path}
                self.trajectories[robot_id] = ([], [])

            # Extract positions
            x, y = msg.pose.position.x, msg.pose.position.y
            quat = msg.pose.orientation
            theta = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('ZYX')[0]

            # 1. Update Trajectory (Highly efficient update of an array)
            if self.record_trajectory:
                self.trajectories[robot_id][0].append(x)
                self.trajectories[robot_id][1].append(y)

                # Cap the trajectory length at 5000 points so memory doesn't leak over hours
                if len(self.trajectories[robot_id][0]) > 500:
                    self.trajectories[robot_id][0].pop(0)
                    self.trajectories[robot_id][1].pop(0)

                # Update the line path rendering on the canvas
                self.robot_items[robot_id]['path'].setData(
                    self.trajectories[robot_id][0],
                    self.trajectories[robot_id][1]
                )

            # 2. Update Arrow Rotation & Position
            # PyQtGraph native angles: 0 degrees points left. To orient correctly, we offset by 180.
            angle_deg = -math.degrees(theta) + 180
            self.robot_items[robot_id]['arrow'].setPos(x, y)
            self.robot_items[robot_id]['arrow'].setStyle(angle=angle_deg)

            # 3. Update Text Position
            self.robot_items[robot_id]['text'].setPos(x, y)

            # 4. Handle HSV Color shifting animation based on time
            c_offset = self.color_offsets[robot_id]
            hue = ((60 * c_offset + (time.time() - self._time_init)) % 60) / 60
            new_color = QColor.fromHsvF(hue, 1.0, 1.0)
            self.robot_items[robot_id]['arrow'].setStyle(brush=new_color)
            self.robot_items[robot_id]['path'].setPen(pg.mkPen(new_color, width=2))

        # Handle Auto-follow mechanics based on the first alphabetical ID
        if snapshot and self.auto_follow:
            first_id = sorted(snapshot.keys())[0]
            first_msg = snapshot[first_id]
            cx, cy = first_msg.pose.position.x, first_msg.pose.position.y

            # Instantly update viewport limits
            self.plot_widget.setXRange(cx + self.x_lims[0], cx + self.x_lims[1], padding=0)
            self.plot_widget.setYRange(cy + self.y_lims[0], cy + self.y_lims[1], padding=0)

# Source - https://stackoverflow.com/a/49223444
# Posted by eyllanesc, modified by community. See post 'Timeline' for change history
# Retrieved 2026-07-02, License - CC BY-SA 3.0


class ArrowFromBaseItem(pg.ArrowItem):
    def setStyle(self, **opts):
        self.opts.update(opts)

        opt = dict([(k, self.opts[k]) for k in ['headLen', 'tipAngle', 'baseAngle', 'tailLen', 'tailWidth']])
        tr = QTransform()
        path = fn.makeArrowPath(**opt)

        # QTransform applies operations in reverse order of the calls:
        # Translation happens first, then rotation.
        tr.rotate(self.opts['angle'])

        # Shift the base of the tail (right-most edge) to X=0, and keep Y centered at 0
        base_x = path.boundingRect().right()
        tr.translate(-base_x, 0)

        self.path = tr.map(path)
        self.setPath(self.path)

        self.setPen(fn.mkPen(self.opts['pen']))
        self.setBrush(fn.mkBrush(self.opts['brush']))

        if self.opts['pxMode']:
            self.setFlags(self.flags() | QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        else:
            self.setFlags(self.flags() & ~QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)


def main(args=None):
    rclpy.init(args=args)

    # 1. Allow Python to catch Ctrl+C instantly in the terminal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # 2. Setup ROS Node and spin it dynamically in a Background Thread
    ros_node = Viewer2DNode()
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    ros_thread = Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # 3. Setup GUI inside the Main Thread
    app = QApplication(sys.argv)
    viewer = ViewerApp(ros_node)
    viewer.show()

    # Run event loop until window is closed
    try:
        sys.exit(app.exec())
    finally:
        # Cleanup
        ros_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()