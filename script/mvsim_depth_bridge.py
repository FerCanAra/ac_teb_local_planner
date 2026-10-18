#!/usr/bin/env python3
# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | MVSim RGB-D point cloud -> RealSense-like depth image + camera_info     |
# +-------------------------------------------------------------------------+
"""
MVSim publishes its `rgbd_camera` sensor only as an *unorganized* point cloud
(`/<sensor>_points`, frame `<sensor>_points`, MRPT axes: +X forward, +Y left,
+Z up). The Intel ground-floor segmentation expects, like with a real
RealSense, a depth image + camera_info:

    /<camera_name>/depth/image_rect_raw   sensor_msgs/Image (16UC1, mm)
    /<camera_name>/depth/camera_info      sensor_msgs/CameraInfo

This node re-projects the cloud with the intrinsics of the MVSim sensor
(definitions/sensors/rgbd_camera.sensor.xml: 640x480, fx=fy=400, cx=320,
cy=240) into an organized depth image in an optical frame (+Z forward,
+X right, +Y down), and publishes the static TF
`<sensor>_points -> <camera_name>_depth_optical_frame`.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

_NP_TYPES = {
    PointField.INT8: "i1", PointField.UINT8: "u1", PointField.INT16: "<i2",
    PointField.UINT16: "<u2", PointField.INT32: "<i4", PointField.UINT32: "<u4",
    PointField.FLOAT32: "<f4", PointField.FLOAT64: "<f8",
}


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """(N,3) float64 array with the x, y, z fields of a PointCloud2."""
    fields = {f.name: f for f in msg.fields}
    n = msg.width * msg.height
    if n == 0 or not all(k in fields for k in ("x", "y", "z")):
        return np.zeros((0, 3))
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [_NP_TYPES[fields[k].datatype] for k in ("x", "y", "z")],
        "offsets": [fields[k].offset for k in ("x", "y", "z")],
        "itemsize": msg.point_step,
    })
    if msg.height > 1 and msg.row_step != msg.width * msg.point_step:
        rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.row_step)
        buf = rows[:, : msg.width * msg.point_step].tobytes()
    else:
        buf = bytes(msg.data)[: n * msg.point_step]
    arr = np.frombuffer(buf, dtype=dtype, count=n)
    return np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64)


def project_to_depth(pts_sensor: np.ndarray, width: int, height: int,
                     fx: float, fy: float, cx: float, cy: float,
                     min_depth: float, max_depth: float) -> np.ndarray:
    """Z-buffer projection of points given in the MRPT sensor frame
    (+X forward, +Y left, +Z up). Returns a (height, width) float32 depth
    image in metres (0 = no measurement)."""
    depth = np.zeros((height, width), dtype=np.float32)
    if pts_sensor.shape[0] == 0:
        return depth
    # MRPT sensor frame -> optical frame
    zo = pts_sensor[:, 0]
    xo = -pts_sensor[:, 1]
    yo = -pts_sensor[:, 2]
    ok = np.isfinite(zo) & np.isfinite(xo) & np.isfinite(yo) & (zo >= min_depth) & (zo <= max_depth)
    zo, xo, yo = zo[ok], xo[ok], yo[ok]
    u = np.rint(fx * xo / zo + cx).astype(np.int64)
    v = np.rint(fy * yo / zo + cy).astype(np.int64)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, zo = u[inside], v[inside], zo[inside]
    if zo.size == 0:
        return depth
    # keep the closest point per pixel: write in descending depth order
    order = np.argsort(-zo)
    depth[v[order], u[order]] = zo[order].astype(np.float32)
    return depth


class MvsimDepthBridge(Node):

    def __init__(self):
        super().__init__("mvsim_depth_bridge")
        self.declare_parameter("input_topic", "/camera1_points")
        self.declare_parameter("camera_name", "camera1")
        # intrinsics of definitions/sensors/rgbd_camera.sensor.xml
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fx", 400.0)
        self.declare_parameter("fy", 400.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("min_depth", 0.3)
        self.declare_parameter("max_depth", 3.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.w, self.h = int(g("width")), int(g("height"))
        self.fx, self.fy, self.cx, self.cy = (float(g(k)) for k in ("fx", "fy", "cx", "cy"))
        self.min_d, self.max_d = float(g("min_depth")), float(g("max_depth"))
        cam = str(g("camera_name"))
        self.optical_frame = f"{cam}_depth_optical_frame"
        self.static_tf_sent = False
        self.tf_static = StaticTransformBroadcaster(self)

        self.pub_depth = self.create_publisher(Image, f"/{cam}/depth/image_rect_raw", 10)
        self.pub_info = self.create_publisher(CameraInfo, f"/{cam}/depth/camera_info", 10)
        self.create_subscription(PointCloud2, str(g("input_topic")), self.on_cloud, 10)
        self.get_logger().info(
            f"MVSim depth bridge: {g('input_topic')} -> /{cam}/depth/image_rect_raw "
            f"({self.w}x{self.h}, f={self.fx:.0f}, frame {self.optical_frame})")

    def send_static_tf(self, parent_frame: str):
        # MRPT sensor frame (x fwd, y left, z up) -> optical (z fwd, x right, y down)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = self.optical_frame
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        self.tf_static.sendTransform(t)
        self.static_tf_sent = True

    def on_cloud(self, msg: PointCloud2):
        if not self.static_tf_sent:
            self.send_static_tf(msg.header.frame_id)

        depth_m = project_to_depth(cloud_to_xyz(msg), self.w, self.h, self.fx, self.fy,
                                   self.cx, self.cy, self.min_d, self.max_d)
        depth_mm = np.round(depth_m * 1000.0).astype(np.uint16)

        img = Image()
        img.header.stamp = msg.header.stamp
        img.header.frame_id = self.optical_frame
        img.height, img.width = self.h, self.w
        img.encoding = "16UC1"
        img.is_bigendian = 0
        img.step = self.w * 2
        img.data = depth_mm.tobytes()

        info = CameraInfo()
        info.header = img.header
        info.height, info.width = self.h, self.w
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.pub_info.publish(info)
        self.pub_depth.publish(img)


def main(args=None):
    rclpy.init(args=args)
    node = MvsimDepthBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()