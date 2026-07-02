import time
from threading import Lock

import numpy as np
import open3d as o3d
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


def make_robot_arrow(color=(1.0, 0.0, 0.0)):
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


class Viewer3D(Node):
    def __init__(self):
        super().__init__('viewer3d')
        self.auto_follow = True
        self.record_trajectory = False

        self._robot_pos = {}
        self._lock_pos = Lock()
        self._time_init = self.get_clock().now()

        self.color_offsets = {}
        self.robot_meshes = {}          # robot_id -> TriangleMesh currently in the scene
        self.robot_base_meshes = {}     # robot_id -> pristine (unrotated/untranslated) mesh
        self.trajectory_points = {}     # robot_id -> list of (x, y, z)
        self.trajectory_lineset = {}    # robot_id -> LineSet currently in the scene

        self.x_lims = (-10, 10)
        self.y_lims = (-10, 10)

        # --- Open3D visualizer setup ---
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="Robot Viewer 3D", width=1000, height=700)

        render_opt = self.vis.get_render_option()
        render_opt.background_color = np.array([0.05, 0.05, 0.05])
        render_opt.mesh_show_back_face = True

        grid = self._make_ground_grid()
        self.vis.add_geometry(grid)

        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        self.vis.add_geometry(axis)

        # Keyboard shortcuts replace the matplotlib buttons:
        #   F -> toggle auto-follow
        #   T -> toggle trajectory recording
        #   Q / Esc -> quit (also triggers shutdown publish)
        self.vis.register_key_callback(ord("F"), self._on_toggle_follow)
        self.vis.register_key_callback(ord("T"), self._on_toggle_trajectory)
        self.vis.register_key_callback(ord("Q"), self._on_quit)

        self._closed = False

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

    @staticmethod
    def _make_ground_grid(size=50, step=1):
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

    def _on_toggle_follow(self, vis):
        self.auto_follow = not self.auto_follow
        return False

    def _on_toggle_trajectory(self, vis):
        self.record_trajectory = not self.record_trajectory
        if not self.record_trajectory:
            for robot_id, ls in list(self.trajectory_lineset.items()):
                self.vis.remove_geometry(ls, reset_bounding_box=False)
            self.trajectory_lineset = {}
            self.trajectory_points = {}
        return False

    def _on_quit(self, vis):
        self.system_shutdown()
        self.executor.shutdown(timeout_sec=0)
        return False

    def _spin_once(self):
        self._draw()

    def _draw(self):
        if self._closed:
            return

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

            if robot_id not in self.robot_base_meshes:
                base_mesh = make_robot_arrow(color=color)
                self.robot_base_meshes[robot_id] = base_mesh
                live_mesh = o3d.geometry.TriangleMesh(base_mesh)
                self.robot_meshes[robot_id] = live_mesh
                self.vis.add_geometry(live_mesh, reset_bounding_box=False)

            live_mesh = self.robot_meshes[robot_id]
            base_mesh = self.robot_base_meshes[robot_id]

            live_mesh.vertices = o3d.utility.Vector3dVector(
                np.asarray(base_mesh.vertices)
            )
            live_mesh.vertex_colors = base_mesh.vertex_colors
            live_mesh.rotate(rot.as_matrix(), center=(0, 0, 0))
            live_mesh.translate((x, y, z))
            live_mesh.compute_vertex_normals()
            self.vis.update_geometry(live_mesh)

            if self.record_trajectory:
                pts = self.trajectory_points.setdefault(robot_id, [])
                pts.append([x, y, z])
                if len(pts) >= 2:
                    ls = self.trajectory_lineset.get(robot_id)
                    if ls is None:
                        ls = o3d.geometry.LineSet()
                        ls.paint_uniform_color(color)
                        self.trajectory_lineset[robot_id] = ls
                        self.vis.add_geometry(ls, reset_bounding_box=False)
                    ls.points = o3d.utility.Vector3dVector(pts)
                    lines = [[i, i + 1] for i in range(len(pts) - 1)]
                    ls.lines = o3d.utility.Vector2iVector(lines)
                    self.vis.update_geometry(ls)

        if robot_pos_snapshot and self.auto_follow:
            first_id = sorted(robot_pos_snapshot.keys())[0]
            cam_pos = robot_pos_snapshot[first_id].pose.position
            view_ctl = self.vis.get_view_control()
            # camera_params = view_ctl.convert_to_pinhole_camera_parameters()
            # extrinsic = camera_params.extrinsic.copy()
            # Keep current orientation, re-center the look-at point on the tracked robot
            # view_ctl.set_lookat([cam_pos.x, cam_pos.y, cam_pos.z])
            view_ctl.set_lookat([cam_pos.x, cam_pos.y, cam_pos.z])
            view_ctl.set_front([0, 0, 1])   # camera is placed in the +Z direction, looking toward Z-
            view_ctl.set_up([0, 1, 0])       # Y is up
            view_ctl.set_zoom(3)
            # print(f"{cam_pos.x=} {cam_pos.y=} {cam_pos.z=}")

        self.vis.poll_events()
        self.vis.update_renderer()

        if not self.vis.poll_events():
            # Window was closed by the user
            self._closed = True
            self.system_shutdown()
            self.executor.shutdown(timeout_sec=0)

    def callback_robot_pos(self, msg: PoseStamped):
        robot_id = msg.header.frame_id
        with self._lock_pos:
            self._robot_pos[robot_id] = msg

    def system_shutdown(self):
        self.publisher_shutdown_.publish(Empty())
        time.sleep(0.2)


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


def main(args=None):
    rclpy.init(args=args)
    viewer = Viewer3D()
    executor = SingleThreadedExecutor()
    executor.add_node(viewer)
    try:
        executor.spin()  # blocks until executor.shutdown() is called
    except KeyboardInterrupt:
        pass
    finally:  # single, guaranteed cleanup point
        viewer.vis.destroy_window()
        viewer.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()