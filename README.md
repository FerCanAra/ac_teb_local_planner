# AC-TEB: Aisle-Centring Timed Elastic Band for greenhouse robots

**AC-TEB** is a ROS 2 / Nav2 local planner that keeps a mobile robot on the
centreline of narrow crop aisles using only a depth camera. It extends the
[Timed Elastic Band (TEB)](https://github.com/rst-tu-dortmund/teb_local_planner)
model predictive controller with an online aisle estimator and a width-adaptive
centring cost. The centring acts only inside the crop rows; in wide corridors,
or whenever the aisle is not perceived, the controller reduces exactly to the
original TEB.

It was developed for Mediterranean (*raspa y amagado*) greenhouses and is
evaluated in the open-source
[ROS 2 greenhouse benchmark](https://github.com/FerCanAra/robotics_benchmark_greenhouse)
against TEB, MPPI, DWB and RPP, without a prior map and with camera-only
perception.

> Paper: *Camera-Only Aisle Centring for Timed-Elastic-Band MPC of Mobile Robots in
> Unmapped Mediterranean Greenhouses*, F. Cañadas-Aránega, J. L. Guzmán, J. C. Moreno.
> Submitted to the European Control Conference (ECC 2027).

---

## How it works

```
 depth camera ──► ground segmentation ──► aisle estimator ──► AC-TEB (Nav2 controller) ──► cmd_vel
                        │                  (ê_y, ψ̂, Ŵ, c)            ▲
                        └──► online costmaps (no prior map) ──► global planner (path)
```

1. **Ground segmentation.** The depth cloud is labelled as *ground* or
   *obstacle* (Intel `pointcloud_groundfloor_segmentation`).
2. **Aisle estimator.** The *obstacle* points ahead of the robot are divided
   into slices along the aisle. The inner edge of each row is a low quantile of
   the lateral distance in every slice. Both edges are fitted jointly as two
   parallel lines by Huber-weighted least squares, which gives the lateral
   offset `e_y`, the relative heading `ψ` and the free width `W`. A linear
   Kalman filter with a χ² gate fuses them with the odometry. A confidence
   `c ∈ [0,1]` holds the estimate through short occlusions and fades it out at
   the row ends and during turns. A new aisle is only initialised when both
   rows are visible and `W < W̄`, so nothing is estimated in the central
   corridor.
3. **Centring cost.** A term
   `γ_c(Ŵ) Σ k_i ( ê_{y,i}² + γ_ψ/γ_c^max (θ_i − φ̂)² )` is added to the TEB
   optimisation. Its weight
   `γ_c = c · γ_c^max / (1 + exp(α_W (Ŵ − W̄)))` is large in narrow rows,
   vanishes in wide corridors and fades out with the confidence. The local goal
   is also projected onto the centreline.

The new terms are unary g2o edges with constant Jacobians: they keep the
problem sparse and add less than 1 ms per control cycle.

---

## Results (simulation, 5 runs per controller)

Route through two aisles of the benchmark greenhouse, no prior map, depth camera
only. Mean ± std over the successful runs; `e_y` is the lateral error with
respect to the true aisle centre, evaluated along the straight sections of the
rows.

| | DWB | RPP | TEB | MPPI | **AC-TEB** |
|---|---|---|---|---|---|
| Success [%] | 20 | 100 | 80 | 100 | **100** |
| Time [s] | 76.8 | 66.9 ± 2.2 | 65.6 ± 10.5 | 65.8 ± 0.4 | **65.3 ± 0.8** |
| `e_y` RMS [cm] | 25.3 | 24.3 ± 1.5 | 26.3 ± 5.6 | 18.1 ± 2.7 | **14.2 ± 1.5** |
| max \|`e_y`\| [cm] | 57.4 | 46.6 ± 1.4 | 49.2 ± 1.8 | 39.7 ± 2.3 | **35.5 ± 4.7** |
| Min. clearance [cm] | 9.4 | 8.4 ± 4.1 | 3.5 ± 5.3 | **15.4 ± 4.8** | 5.9 ± 5.4 |
| Comp. time [ms] | – | – | 2.9 ± 0.2 | – | 3.8 ± 0.1 |

AC-TEB reduces the RMS lateral error by 46 % with respect to TEB, which shares
all its parameters except the centring term, and by 22 % with respect to MPPI,
with the same route time. MPPI keeps a larger corner clearance because it drives
straighter; reducing the residual heading oscillation of AC-TEB is future work.

---

## Repository structure

```
ac_teb_local_planner/
├── src/, include/                 AC-TEB plugin (TEB + aisle estimator + centring edges)
├── ac_teb_local_planner_plugin.xml
├── params/
│   ├── c2_mpc_ac_teb_params.yaml  AC-TEB / TEB parameters used in the paper
│   ├── c2_mppi_params.yaml        MPPI baseline
│   ├── c2_dwb_params.yaml         DWB baseline
│   └── c2_rpp_params.yaml         RPP baseline
├── config/c2_nav2_config.yaml     Nav2 stack (camera-only costmaps)
├── definitions/                   MVSim greenhouse world, robot and sensors
├── launch/mpc_benchmark_bringup.launch.py
├── script/
│   ├── route_cat2.py              goal sequence of the evaluation route
│   ├── ecc_logger.py              per-run logger (CSV) with ground-truth metrics
│   ├── ecc_plots.py               table and figures from the logged runs
│   └── mvsim_depth_bridge.py      MVSim point cloud → RealSense-like depth image
└── rviz/c2_greenhouse.rviz
```

---

## Requirements

- Ubuntu 22.04, **ROS 2 Humble**
- [Nav2](https://github.com/ros-navigation/navigation2) (`nav2_bringup`, MPPI, DWB, RPP)
- [MVSim](https://github.com/MRPT/mvsim)
- [`benchmark_bringup`](https://github.com/FerCanAra/robotics_benchmark_greenhouse)
  (greenhouse benchmark, `benchmark_msg`, low-level controller)
- `costmap_converter`, `teb_msgs`, `libg2o`
- Intel `pointcloud_groundfloor_segmentation` (ground segmentation of the depth cloud)
- Python 3 with `numpy` and `matplotlib` (only for the plotting scripts)

---

## Installation

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/FerCanAra/robotics_benchmark_greenhouse.git
git clone https://github.com/FerCanAra/ac_teb_local_planner.git
# + pointcloud_groundfloor_segmentation (Intel Robotics AI Suite)

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Usage

Launch the greenhouse, Nav2, the evaluation route and the logger with the
selected controller (category 2 of the benchmark):

```bash
ros2 launch ac_teb_local_planner mpc_benchmark_bringup.launch.py \
    category:=2 controller:=ac_teb
```

| Argument | Values | Description |
|---|---|---|
| `controller` | `ac_teb`, `teb`, `mppi`, `dwb`, `rpp` | Local controller. `teb` is AC-TEB with the centring disabled (`aisle.enabled: False`). |
| `ecc_logger` | `True` / `False` | Record the run in `results_dir`. |
| `results_dir` | path (default `~/ECC`) | Output folder of the logger. |
| `tag` | string | Label appended to the run folder. |
| `use_rviz` | `True` / `False` | Start RViz. |

All controllers share the same global planner, camera-only costmaps and
velocity limits (`v ≤ 0.5 m/s`, `ω ≤ 1.0 rad/s`); only the `FollowPath` block of
the controller server changes.

In RViz, AC-TEB draws the estimated centreline (red) and row edges (orange); they
turn grey when the estimate is held but not used.

---

## Main AC-TEB parameters

In `params/c2_mpc_ac_teb_params.yaml`, block `FollowPath.aisle`:

| Parameter | Paper | Value | Description |
|---|---|---|---|
| `weight` | γ_c^max | 35 | maximum centring weight |
| `width_ref` | W̄ | 3.0 m | width separating crop rows from corridors |
| `width_steepness` | α_W | 6 m⁻¹ | steepness of the width transition |
| `horizon` | L_c | 2.0 m | centring horizon along the aisle |
| `ramp_distance` | L_r | 1.5 m | weight ramp from the robot |
| `heading_weight` | γ_ψ | 15 | heading-alignment weight |
| `ref_time_constant` | τ | 1.0 s | low-pass filter of the centreline |
| `max_heading_error` | ψ_max | 0.5 rad | heading beyond which the centring fades out |
| `min_width`, `max_width` | [W_min, W_max] | 0.3, 3.0 m | plausible width range |
| `hold_distance` | – | 3.0 m | estimate held without measurements |
| `project_goal` | – | `True` | project the local goal onto the centreline |

TEB parameters shared by TEB and AC-TEB: `dt_ref 0.3`, `weight_viapoint 5`
(γ_vp), `min_obstacle_dist 0.10` (d_obst), `penalty_epsilon 0.1`, circular
footprint of radius 0.40 m.

`min_obstacle_dist` should satisfy the compatibility condition
`d_obst ≤ W/2 − r_robot` in the narrowest row, and `weight` the tuning rule
`γ_c^max ≥ γ_vp (d_max/ε_c − 1)` for the largest expected offset `d_max` of the
global path and the admissible residual offset `ε_c` (see the paper).

---

## Reproducing the experiments

1. Run each controller several times (the paper uses 5 runs per controller):
   ```bash
   for c in ac_teb teb mppi dwb rpp; do
     ros2 launch ac_teb_local_planner mpc_benchmark_bringup.launch.py \
         category:=2 controller:=$c tag:=final use_rviz:=False
   done
   ```
   Each run ends automatically when the last goal is reached, the robot gets
   stuck (45 s) or the timeout (600 s) expires.
2. Every run is written to `~/ECC/<date>_<controller>[_<tag>]/`:
   - `timeseries.csv`: pose, true lateral error `e_y`, true width, clearance,
     computation time and, for AC-TEB, the aisle estimate and `γ_c`, at 20 Hz;
   - `summary.csv`: success, time, `e_y` RMS / max, minimum clearance,
     computation time and estimation errors.

   The ground truth is computed from the known plant layout of the world file
   (centre between the inner edges of the plant collision boxes), never from
   the robot's own estimate.
3. Build the table and figures (no ROS needed):
   ```bash
   python3 script/ecc_plots.py --results ~/ECC
   ```

> Note: the robot pose is taken from the simulator odometry (no localisation
> noise), so that the comparison isolates perception and control.

---

## Citation

If you use AC-TEB, please cite:

```bibtex
@inproceedings{canadas2027acteb,
  author    = {Ca{\~n}adas-Ar{\'a}nega, Fernando and Guzm{\'a}n, Jos{\'e} L. and Moreno, Jos{\'e} C.},
  title     = {Camera-Only Aisle Centring for Timed-Elastic-Band {MPC} of Mobile Robots
               in Unmapped {Mediterranean} Greenhouses},
  booktitle = {European Control Conference (ECC)},
  year      = {2027},
  note      = {Submitted}
}
```

AC-TEB builds on the TEB local planner:

```bibtex
@article{rosmann2017integrated,
  author    = {R{\"o}smann, Christoph and Hoffmann, Frank and Bertram, Torsten},
  title     = {Integrated online trajectory planning and optimization in distinctive topologies},
  journal   = {Robotics and Autonomous Systems},
  volume    = {88},
  pages     = {142--153},
  year      = {2017}
}
```

---

## License

BSD 3-Clause. AC-TEB is derived from the
[`teb_local_planner`](https://github.com/rst-tu-dortmund/teb_local_planner)
(© 2016 TU Dortmund, Lehrstuhl RST), whose license and copyright notice are
kept in [`LICENSE`](LICENSE).

## Acknowledgements

This work has been carried out within the framework of the LIFE-ACCLIMATE
project (LIFE23-CCAES-LIFE-ACCLIMATE/101157315) and an Erasmus+ mobility grant
from CeiA3. F. Cañadas-Aránega holds an FPI grant (PRE2022-102415) from the
Spanish Ministry of Science, Innovation and Universities.

University of Almería · CIESOL · ceiA3 — contact: fernando.ca@ual.es
