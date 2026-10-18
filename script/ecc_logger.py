#!/usr/bin/env python3
# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | AC-TEB (ECC 2027): records one run of category 2                        |
# +-------------------------------------------------------------------------+
"""
Records one category-2 run and writes

    <results_dir>/<YYYY-MM-DD_HH-MM-SS>_<controller>/
        timeseries.csv   one row per sample (rate, default 20 Hz), written
                         while the robot drives (usable even if the run is
                         interrupted). Input of the figures.
        summary.csv      one row with the metrics of the paper table.

timeseries.csv columns
    controller        algorithm under test (launch arg controller:=)
    t                 time since the start of the run [s]
    x, y, yaw         robot pose (MVSim odometry = ground truth) [m, m, rad]
    v, w              linear / angular velocity [m/s, rad/s]
    in_aisle          1 if the robot is inside a row (both rows closer than
                      aisle_max_halfwidth), 0 in headlands and turns
    e_y               true lateral offset of the centreline seen from the
                      robot, as in the paper (> 0: centreline to the left) [m]
    W_true            true free width of the aisle [m]
    aisle_yaw         true direction of the aisle [rad]
    clearance         distance from the robot circle to the nearest row [m]
    compute_time_ms   last computation time of the controller (AC-TEB/TEB
                      plugin, topic /FollowPath/compute_time_ms) [ms]
    est_valid, est_e_y, est_psi, est_W, gamma_c, est_e_std
                      aisle estimate of AC-TEB (/FollowPath/aisle_state)

summary.csv columns
    controller, date, status, success_pct, e_y_rms_cm, e_y_max_cm,
    clearance_min_cm, time_s, compute_time_mean_ms, compute_time_max_ms,
    aisle_samples, path_length_m
  e_y and clearance are evaluated only along the straight sections of the
  narrow rows: in_aisle = 1, W_true <= row_max_width, |yaw - aisle_yaw| <=
  row_max_heading and at least row_entry_margin inside the row.
  est_*: AC-TEB estimate vs ground truth in the same samples (valid %, RMSE
  and correlation of e_y, RMSE of W).
  success_pct is 100 or 0 for one run; the success rate is the mean over runs.

Ground truth (ground_truth:=plants, default) is rebuilt from the plant layout
of the world file, which is deterministic (hash jitter instead of rand()):
inner edges of both rows from the plant collision squares (+-plant_half)
within +-0.5 m along the row, centreline halfway, clearance to the nearest
plant square. The static /map is NOT used: it was built from another random
layout and does not match the simulated plants.
With ground_truth:=map (legacy) it comes from the occupancy map: at every sample,
rays are cast to both sides of the robot over a fan of directions around its
heading, and the direction with the smallest wall-to-wall distance gives the
aisle normal, the width W and the offset. The run starts when route_cat2
publishes /goal_markers and ends when the robot reaches the last goal
(success), does not move for stuck_time (stuck) or after timeout.
"""

import csv
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, Float64MultiArray
from visualization_msgs.msg import MarkerArray

TS_COLS = ["controller", "t", "x", "y", "yaw", "v", "w", "in_aisle", "e_y", "W_true",
           "aisle_yaw", "clearance", "compute_time_ms", "est_valid", "est_e_y", "est_psi",
           "est_W", "gamma_c", "est_e_std"]
SUMMARY_COLS = ["controller", "tag", "date", "status", "success_pct", "e_y_rms_cm", "e_y_max_cm",
                "clearance_min_cm", "time_s", "compute_time_mean_ms", "compute_time_max_ms",
                "aisle_samples", "path_length_m", "est_valid_pct", "est_e_y_rmse_cm",
                "est_e_y_corr", "est_W_rmse_cm"]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GridMap:
    """Occupancy grid with vectorised ray casting."""

    def __init__(self, msg: OccupancyGrid, occ_thresh: int = 65):
        info = msg.info
        self.res = info.resolution
        self.w, self.h = info.width, info.height
        self.ox, self.oy = info.origin.position.x, info.origin.position.y
        if abs(yaw_from_quat(info.origin.orientation)) > 1e-6:
            raise ValueError("rotated map origins are not supported")
        data = np.asarray(msg.data, dtype=np.int16).reshape(self.h, self.w)
        self.occ = data >= occ_thresh          # unknown (-1) counts as free

    def occupied(self, x, y):
        i = np.floor((x - self.ox) / self.res).astype(np.int64)
        j = np.floor((y - self.oy) / self.res).astype(np.int64)
        inside = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        out = np.zeros(np.shape(x), dtype=bool)
        out[inside] = self.occ[j[inside], i[inside]]
        return out

    def first_hit(self, px, py, dirs, r):
        """Distance to the first occupied cell along each direction (inf if none)."""
        px = np.reshape(np.asarray(px, dtype=float), (-1, 1))
        py = np.reshape(np.asarray(py, dtype=float), (-1, 1))
        hit = self.occupied(px + dirs[:, 0:1] * r[None, :], py + dirs[:, 1:2] * r[None, :])
        return np.where(hit.any(axis=1), r[np.argmax(hit, axis=1)], np.inf)


def _hash01(iX, iY, k):
    """Deterministic jitter in [0, 1): same expression as the plants of
    definitions/world/greenhouse_control.world.xml (keep both in sync)."""
    return math.modf(abs(math.sin(iX * 12.9898 + iY * 78.233 + k)) * 43758.5453)[0]


def greenhouse_plants(rand_xy=0.35):
    """(N, 2) plant centres of the MVSim greenhouse (5 double rows)."""
    pts = []
    for iX in range(5):
        for x0, kx, ky in ((3.4, 1.1, 2.2), (4.6, 3.3, 4.4)):
            for iY in range(15):   # south block
                pts.append((x0 + 4 * iX + _hash01(iX, iY, kx) * rand_xy,
                            1.5 + iY * (11 / 15) + _hash01(iX, iY, ky) * rand_xy))
        for x0, kx, ky in ((3.4, 5.5, 6.6), (4.6, 7.7, 8.8)):
            for iY in range(10):   # north block
                pts.append((x0 + 4 * iX + _hash01(iX, iY, kx) * rand_xy,
                            15.2 + iY * (6 / 10) + _hash01(iX, iY, ky) * rand_xy))
    return np.asarray(pts)


class PlantTruth:
    """Exact ground truth from the plant layout of the world file. Rows run
    along y; a plant blocks a square of +-half around its centre (collision
    shape of plant_tomato). The aisles are the 2.8 m gaps between double rows."""

    def __init__(self, rand_xy=0.35, half=0.5, window=0.5):
        self.p = greenhouse_plants(rand_xy)
        self.half, self.window = half, window
        self.cols = np.array([3.4, 4.6, 7.4, 8.6, 11.4, 12.6, 15.4, 16.6, 19.4, 20.6])
        self.aisles = [(4.6, 7.4), (8.6, 11.4), (12.6, 15.4), (16.6, 19.4)]

    def aisle(self, x, y, yaw):
        """(in_aisle, e_y, W, aisle_yaw); e_y as in the paper (> 0: centre to the left)."""
        a_yaw = math.pi / 2 if math.cos(yaw - math.pi / 2) >= 0 else -math.pi / 2
        for c0, c1 in self.aisles:
            if not (c0 < x < c1 + 0.35):
                continue
            near = np.abs(self.p[:, 1] - y) <= self.window
            left = near & (np.abs(self.p[:, 0] - (c0 + 0.175)) < 0.4)
            right = near & (np.abs(self.p[:, 0] - (c1 + 0.175)) < 0.4)
            if not left.any() or not right.any():
                return False, float("nan"), float("nan"), a_yaw
            e0 = self.p[left, 0].max() + self.half      # inner edge of the row at smaller x
            e1 = self.p[right, 0].min() - self.half     # inner edge of the row at larger x
            cx = 0.5 * (e0 + e1)
            return True, float(-math.sin(a_yaw) * (cx - x)), float(e1 - e0), a_yaw
        return False, float("nan"), float("nan"), a_yaw

    def clearance(self, x, y, r_robot):
        dx = np.maximum(np.abs(self.p[:, 0] - x) - self.half, 0.0)
        dy = np.maximum(np.abs(self.p[:, 1] - y) - self.half, 0.0)
        return float(np.min(np.hypot(dx, dy))) - r_robot


class EccLogger(Node):

    def __init__(self):
        super().__init__("ecc_logger")
        p = self.declare_parameter
        p("results_dir", os.path.expanduser("~/ECC"))
        p("controller", "ac_teb")
        p("tag", "")                    # free label of the configuration, e.g. g30_vp5
        p("row_max_width", 2.5)         # metrics only in rows narrower than this [m]
        p("row_max_heading", 0.3)       # ... with the robot aligned with the row [rad]
        p("row_entry_margin", 0.5)      # ... and this far inside the row [m]
        p("robot_radius", 0.36)
        p("goal_tolerance", 0.5)
        p("timeout", 600.0)
        p("stuck_time", 45.0)
        p("aisle_max_halfwidth", 1.5)   # both rows closer than this -> inside a row (W <= 3 m)
        p("aisle_fan_deg", 10.0)        # +- search of the aisle direction around the heading
        p("rate", 20.0)
        p("ground_truth", "plants")     # plants: exact layout of the world file | map: /map
        p("plant_half", 0.5)            # half size of the plant collision square [m]
        p("plants_rand_xy", 0.35)       # PLANTS_RAND_XY of the world file [m]
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.results_root = os.path.expanduser(str(g("results_dir")))
        self.controller = str(g("controller"))
        self.tag = str(g("tag")).strip()
        self.row_max_width = float(g("row_max_width"))
        self.row_max_heading = float(g("row_max_heading"))
        self.row_entry_margin = float(g("row_entry_margin"))
        self.r_robot = float(g("robot_radius"))
        self.goal_tol = float(g("goal_tolerance"))
        self.timeout = float(g("timeout"))
        self.stuck_time = float(g("stuck_time"))
        self.aisle_half = float(g("aisle_max_halfwidth"))
        fan = math.radians(float(g("aisle_fan_deg")))
        self.fan = np.linspace(-fan, fan, 21)
        self.s_window = np.linspace(-0.4, 0.4, 5)
        self.dt = 1.0 / float(g("rate"))

        self.r_lat = np.arange(0.02, 2.0 * self.aisle_half + 0.5, 0.01)
        self.r_clear = np.arange(0.0, 2.0, 0.01)
        ang = np.linspace(-math.pi, math.pi, 120, endpoint=False)
        self.clear_dirs = np.stack([np.cos(ang), np.sin(ang)], axis=1)

        self.map = None
        self.gt_mode = str(g("ground_truth")).lower()
        self.plants = PlantTruth(float(g("plants_rand_xy")), float(g("plant_half")))             if self.gt_mode == "plants" else None
        self.odom = None
        self.aisle_state = None
        self.comp_last = float("nan")
        self.comp_all = []                 # every controller cycle, not only the samples
        self.goals = None
        self.t0 = None
        self.date = None
        self.run_dir = None
        self.ts_file = None
        self.ts_writer = None
        self.rows = []
        self.last_flush = 0.0
        self.last_move_t = None
        self.last_move_p = None
        self.done = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Float64MultiArray, "/FollowPath/aisle_state", self.on_aisle, 20)
        self.create_subscription(Float64, "/FollowPath/compute_time_ms", self.on_comp, 50)
        self.create_subscription(MarkerArray, "/goal_markers", self.on_goals, 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f"ECC logger: controller '{self.controller}' -> {self.results_root}")

    # ------------------------------------------------------------------ inputs
    def on_map(self, msg):
        try:
            self.map = GridMap(msg)
            self.get_logger().info(f"map received: {self.map.w}x{self.map.h} @ {self.map.res} m")
        except ValueError as e:
            self.get_logger().error(str(e))

    def on_odom(self, msg):
        self.odom = msg

    def on_aisle(self, msg):
        if len(msg.data) >= 6:
            self.aisle_state = (time.monotonic(), list(msg.data[:6]))

    def on_comp(self, msg):
        self.comp_last = float(msg.data)
        if self.t0 is not None and not self.done:
            self.comp_all.append(self.comp_last)

    def on_goals(self, msg):
        if self.goals is None and msg.markers:
            self.goals = [(m.pose.position.x, m.pose.position.y) for m in msg.markers]
            self.get_logger().info(f"route received ({len(self.goals)} goals) -> run started")

    # ------------------------------------------------------------ ground truth
    def aisle_truth(self, x, y, yaw):
        """(in_aisle, offset, W, aisle_yaw). offset > 0: robot left of the centreline."""
        a = yaw + self.fan
        n = np.stack([-np.sin(a), np.cos(a)], axis=1)       # left normals
        d = np.stack([np.cos(a), np.sin(a)], axis=1)        # aisle directions
        # ragged rows: inner edge = closest hit over +-0.4 m along the aisle
        dl = np.full(len(a), np.inf)
        dr = np.full(len(a), np.inf)
        for sw in self.s_window:
            ox, oy = x + sw * d[:, 0], y + sw * d[:, 1]
            dl = np.minimum(dl, self.map.first_hit(ox, oy, n, self.r_lat))
            dr = np.minimum(dr, self.map.first_hit(ox, oy, -n, self.r_lat))
        s = dl + dr
        if not np.isfinite(s).any():
            return False, float("nan"), float("nan"), float("nan")
        k = int(np.nanargmin(np.where(np.isfinite(s), s, np.nan)))
        in_aisle = dl[k] <= self.aisle_half and dr[k] <= self.aisle_half
        return bool(in_aisle), float(0.5 * (dr[k] - dl[k])), float(s[k]), float(a[k])

    def clearance(self, x, y):
        return float(np.min(self.map.first_hit(x, y, self.clear_dirs, self.r_clear))) - self.r_robot

    # -------------------------------------------------------------------- loop
    def start_run(self, now):
        self.t0 = now
        self.last_move_t = now
        self.date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.run_dir = os.path.join(self.results_root, f"{self.date}_{self.controller}" + (f"_{self.tag}" if self.tag else ""))
        os.makedirs(self.run_dir, exist_ok=True)
        self.ts_file = open(os.path.join(self.run_dir, "timeseries.csv"), "w", newline="")
        self.ts_writer = csv.writer(self.ts_file)
        self.ts_writer.writerow(TS_COLS)
        self.get_logger().info(f"recording -> {self.run_dir}")

    def step(self):
        if self.done or self.odom is None or self.goals is None or                 (self.plants is None and self.map is None):
            return
        now = time.monotonic()
        if self.t0 is None:
            self.start_run(now)
        t = now - self.t0
        o = self.odom
        x, y = o.pose.pose.position.x, o.pose.pose.position.y
        yaw = yaw_from_quat(o.pose.pose.orientation)
        v, w = o.twist.twist.linear.x, o.twist.twist.angular.z

        if self.plants is not None:
            in_aisle, e_y_true, W, aisle_yaw = self.plants.aisle(x, y, yaw)
            off = -e_y_true
            clr = self.plants.clearance(x, y, self.r_robot)
        else:
            in_aisle, off, W, aisle_yaw = self.aisle_truth(x, y, yaw)
            clr = self.clearance(x, y)
        est = [float("nan")] * 6
        if self.aisle_state is not None and now - self.aisle_state[0] < 0.5:
            est = self.aisle_state[1]

        # e_y = rho_c - n_c^T r: centreline seen from the robot = -offset
        row = [t, x, y, yaw, v, w, int(in_aisle), -off, W, aisle_yaw, clr, self.comp_last] + est
        self.rows.append(row)
        self.ts_writer.writerow([self.controller] + [f"{z:.5g}" for z in row])
        if now - self.last_flush > 1.0:
            self.ts_file.flush()
            self.last_flush = now

        gx, gy = self.goals[-1]
        if self.last_move_p is None or math.hypot(x - self.last_move_p[0], y - self.last_move_p[1]) > 0.1:
            self.last_move_p = (x, y)
            self.last_move_t = now
        status = None
        if math.hypot(x - gx, y - gy) <= self.goal_tol:
            status = "success"
        elif now - self.last_move_t > self.stuck_time:
            status = "stuck"
        elif t > self.timeout:
            status = "timeout"
        if status:
            self.finish(status)

    # ------------------------------------------------------------------ output
    def finish(self, status):
        self.done = True
        if self.ts_file:
            self.ts_file.close()
        if not self.rows:
            return
        a = np.asarray(self.rows, dtype=float)
        t, x, y = a[:, 0], a[:, 1], a[:, 2]
        ina = a[:, 6] > 0.5
        e_y, W, clr = a[:, 7], a[:, 8], a[:, 10]
        # metric mask: straight sections of narrow rows, robot aligned, away
        # from the row ends (entering a row after a turn is not centring)
        head = np.abs(np.angle(np.exp(1j * (a[:, 3] - a[:, 9]))))
        row = ina & (W <= self.row_max_width) & (head <= self.row_max_heading)
        step = np.concatenate([[0.0], np.hypot(np.diff(x), np.diff(y))])
        inside = np.zeros_like(step)
        for i in range(1, len(step)):
            inside[i] = inside[i - 1] + step[i] if ina[i] and ina[i - 1] else 0.0
        row &= inside >= self.row_entry_margin
        ina = row

        def stat(fn, arr):
            arr = arr[np.isfinite(arr)]
            return float(fn(arr)) if arr.size else float("nan")

        comp = np.asarray(self.comp_all, dtype=float)
        s = {
            "controller": self.controller,
            "date": self.date,
            "status": status,
            "success_pct": 100 if status == "success" else 0,
            "e_y_rms_cm": 100 * stat(lambda z: np.sqrt(np.mean(z ** 2)), e_y[ina]),
            "e_y_max_cm": 100 * stat(np.max, np.abs(e_y[ina])),
            "clearance_min_cm": 100 * stat(np.min, clr[ina]),
            "time_s": float(t[-1]),
            "compute_time_mean_ms": stat(np.mean, comp),
            "compute_time_max_ms": stat(np.max, comp),
            "aisle_samples": int(ina.sum()),
            "path_length_m": float(np.sum(np.hypot(np.diff(x), np.diff(y)))),
            "tag": self.tag,
        }
        # aisle estimator of AC-TEB vs ground truth, in the same row samples
        ev, ee, eW = a[:, 12], a[:, 13], a[:, 15]
        m = ina & np.isfinite(ev)
        if m.any():
            mv = m & (ev > 0.5)
            s["est_valid_pct"] = 100.0 * mv.sum() / m.sum()
            if mv.sum() > 2:
                s["est_e_y_rmse_cm"] = 100 * float(np.sqrt(np.mean((ee[mv] - e_y[mv]) ** 2)))
                s["est_e_y_corr"] = float(np.corrcoef(ee[mv], e_y[mv])[0, 1])
                s["est_W_rmse_cm"] = 100 * float(np.sqrt(np.mean((eW[mv] - W[mv]) ** 2)))
        with open(os.path.join(self.run_dir, "summary.csv"), "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=SUMMARY_COLS)
            wr.writeheader()
            wr.writerow({k: (f"{v:.4g}" if isinstance(v, float) else v) for k, v in s.items()})
        self.get_logger().info(
            f"RUN DONE [{status}] {self.controller}: t={s['time_s']:.1f} s, "
            f"e_y RMS={s['e_y_rms_cm']:.1f} cm, max={s['e_y_max_cm']:.1f} cm, "
            f"min clearance={s['clearance_min_cm']:.1f} cm, "
            f"compute={s['compute_time_mean_ms']:.2f} ms, "
            f"estimate valid={s.get('est_valid_pct', float('nan')):.0f} %, "
            f"est e_y RMSE={s.get('est_e_y_rmse_cm', float('nan')):.1f} cm, "
            f"corr={s.get('est_e_y_corr', float('nan')):.2f} -> {self.run_dir}")


def main(args=None):
    rclpy.init(args=args)
    node = EccLogger()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, Exception):  # Ctrl+C or launch shutdown
        pass
    finally:
        if not node.done and node.rows:
            node.finish("interrupted")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
