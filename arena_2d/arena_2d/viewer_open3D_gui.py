import time
from threading import Lock

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped

# ros2 topic pub /robots_poses geometry_msgs/msg/PoseStamped "{header: {stamp: now, frame_id: robot_1}, pose: {position: {x: -1, y: 1, z: 1}, orientation: {x: 1, y: 0, z: 0, w: 1}}}"

qos_poses = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # UDP-like
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=2
)

qos_shutdown = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # TCP-like
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


def make_robot_arrow_mesh(color=(1.0, 0.0, 0.0)):
    """Create a small cone+cylinder arrow mesh to represent a robot pose."""
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.08,
        cone_radius=0.16,
        cylinder_height=0.5,
        cone_height=0.3,
    )
    # Arrow is created pointing along +Z by default; rotate so it points along +X
    R = Rotation.from_euler('y', 90, degrees=True).as_matrix()
    arrow.rotate(R, center=(0, 0, 0))
    arrow.paint_uniform_color(color)
    arrow.compute_vertex_normals()
    return arrow


def _hsv_to_rgb(h, s, v):
    """Minimal HSV->RGB conversion (replacement for matplotlib.colors.hsv_to_rgb)."""
    i = int(h * 6.0)
    f = h * 6.0 - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    i %= 6
    return [
        (v, t, p),
        (q, v, p),
        (p, v, t),
        (p, q, v),
        (t, p, v),
        (v, p, q),
    ][i]


def make_trajectory_lineset(points, color):
    """Build a LineSet for a trajectory with an explicit per-line color.

    Every call rebuilds the colors array to match the current number of
    line segments -- LineSet.paint_uniform_color() only stamps colors onto
    the lines that exist *at the time it's called*; if you add more lines
    afterwards without recoloring, those new lines have no color entry and
    render black.
    """
    ls = o3d.geometry.LineSet()
    if len(points) < 2:
        return ls
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector([color for _ in lines])
    return ls


def make_ground_grid(size=50, step=1):
    points = []
    lines = []
    idx = 0
    for i in range(-size, size + 1, step):
        points.append([i, -size, 0])
        points.append([i, size, 0])
        lines.append([idx, idx + 1])
        idx += 2
        points.append([-size, i, 0])
        points.append([size, i, 0])
        lines.append([idx, idx + 1])
        idx += 2
    grid = o3d.geometry.LineSet()
    grid.points = o3d.utility.Vector3dVector(points)
    grid.lines = o3d.utility.Vector2iVector(lines)
    grid.paint_uniform_color([0.25, 0.25, 0.25])
    return grid


class Viewer3D(Node):
    def __init__(self, app: gui.Application):
        super().__init__('viewer3d')
        self.auto_follow = True
        self.record_trajectory = False

        self._robot_pos = {}
        self._lock_pos = Lock()
        self._time_init = self.get_clock().now()

        self.color_offsets = {}
        self.robot_mesh_added = set()       # robot_ids currently added to the scene
        self.trajectory_points = {}         # robot_id -> list of [x, y, z]
        self.trajectory_added = set()       # robot_ids whose trajectory LineSet is in the scene

        self.x_lims = (-10, 10)
        self.y_lims = (-10, 10)

        # --- Open3D GUI / O3DVisualizer setup ---
        self.app = app
        self.vis = o3d.visualization.O3DVisualizer("Robot Viewer 3D", 1000, 700)
        self.vis.show_settings = True
        self.vis.set_background((0.05, 0.05, 0.05, 1.0), None)
        self.app.add_window(self.vis)

        self.vis.add_geometry("ground_grid", make_ground_grid())
        self.vis.add_geometry(
            "world_axis", o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        )

        # Real UI buttons (shown in the "Actions" panel), replacing the old
        # matplotlib Button widgets.
        self.vis.add_action("Toggle trajectory", self._on_toggle_trajectory)
        self.vis.add_action("Toggle auto-follow", self._on_toggle_follow)

        self.vis.reset_camera_to_default()
        self.vis.setup_camera(60, [0, 0, 0], [0, 0, 30], [0, 1, 0])

        self._closed = False
        self.vis.set_on_close(self._on_window_close)

        self.timer = self.create_timer(0.01, self._spin_once)  # Update every 10 ms

        self.subscription_ = self.create_subscription(
            PoseStamped,
            'robots_poses',
            self.callback_robot_pos,
            qos_poses
        )
        self.publisher_shutdown_ = self.create_publisher(
            Empty,
            '/shutdown',
            qos_shutdown
        )

    def _on_toggle_follow(self, vis):
        self.auto_follow = not self.auto_follow

    def _on_toggle_trajectory(self, vis):
        self.record_trajectory = not self.record_trajectory
        if not self.record_trajectory:
            for robot_id in list(self.trajectory_added):
                self.vis.remove_geometry(f"traj_{robot_id}")
            self.trajectory_added = set()
            self.trajectory_points = {}

    def _on_window_close(self):
        self._closed = True
        self.system_shutdown()
        self.executor.shutdown(timeout_sec=0)
        return True  # allow the close to proceed

    def _spin_once(self):
        if self._closed:
            return
        self._draw()
        # Drives the Open3D GUI event loop; returns False once the window
        # has been closed (in addition to the explicit close callback above).
        still_running = self.app.run_one_tick()
        if not still_running and not self._closed:
            self._closed = True
            self.system_shutdown()
            self.executor.shutdown(timeout_sec=0)

    def _draw(self):
        with self._lock_pos:
            robot_pos_snapshot = dict(self._robot_pos)

        for robot_id, msg in robot_pos_snapshot.items():
            if robot_id not in self.color_offsets:
                self.color_offsets[robot_id] = np.random.rand()
            c_offset = self.color_offsets[robot_id]

            x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            quat = msg.pose.orientation
            rot = Rotation.from_quat((quat.x, quat.y, quat.z, quat.w))

            hue = ((60 * c_offset + (self.get_clock().now() - self._time_init).nanoseconds * 1e-9) % 60) / 60
            color = tuple(float(v) for v in _hsv_to_rgb(hue, 1.0, 1.0))

            arrow = make_robot_arrow_mesh(color=color)
            arrow.rotate(rot.as_matrix(), center=(0, 0, 0))
            arrow.translate((x, y, z))
            arrow.compute_vertex_normals()

            # O3DVisualizer.update_geometry() only supports tensor-based
            # point clouds, not legacy TriangleMesh, so moving/recoloring
            # the arrow each frame requires remove + re-add by name.
            name = f"robot_{robot_id}"
            if name in self.robot_mesh_added:
                self.vis.remove_geometry(name)
            self.vis.add_geometry(name, arrow)
            self.robot_mesh_added.add(name)

            if self.record_trajectory:
                pts = self.trajectory_points.setdefault(robot_id, [])
                pts.append([x, y, z])
                traj_name = f"traj_{robot_id}"
                if traj_name in self.trajectory_added:
                    self.vis.remove_geometry(traj_name)
                ls = make_trajectory_lineset(pts, color)
                if len(pts) >= 2:
                    self.vis.add_geometry(traj_name, ls)
                    self.trajectory_added.add(traj_name)

        # Text labels: O3DVisualizer only supports clearing *all* labels
        # and re-adding, there's no per-label update.
        self.vis.clear_3d_labels()
        for robot_id, msg in robot_pos_snapshot.items():
            p = msg.pose.position
            self.vis.add_3d_label([p.x, p.y, p.z + 0.3], robot_id)

        if robot_pos_snapshot and self.auto_follow:
            first_id = sorted(robot_pos_snapshot.keys())[0]
            cam_pos = robot_pos_snapshot[first_id].pose.position
            self.vis.setup_camera(
                60,
                [cam_pos.x, cam_pos.y, cam_pos.z],
                [cam_pos.x, cam_pos.y, cam_pos.z + 10],
                [0, 1, 0],
            )

        self.vis.post_redraw()

    def callback_robot_pos(self, msg: PoseStamped):
        robot_id = msg.header.frame_id
        with self._lock_pos:
            self._robot_pos[robot_id] = msg

    def system_shutdown(self):
        self.publisher_shutdown_.publish(Empty())
        time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)

    gui.Application.instance.initialize()
    viewer = Viewer3D(gui.Application.instance)

    executor = SingleThreadedExecutor()
    executor.add_node(viewer)
    try:
        executor.spin()  # blocks until executor.shutdown() is called
    except KeyboardInterrupt:
        pass
    finally:  # single, guaranteed cleanup point
        viewer.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()