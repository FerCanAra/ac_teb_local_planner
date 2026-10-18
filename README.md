# Vision-Based Aisle Estimation and Width-Adaptive Centring for Nonlinear Timed-Elastic-Band MPC of Mobile Robots in Unmapped Mediterranean Greenhouses

[![Last Updated](https://img.shields.io/badge/last%20updated-2026--11-blue)](.)
[![DOI](https://zenodo.org/badge/1424278083.svg)](https://doi.org/10.5281/zenodo.23451492)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)

**AC-TEB** is a ROS 2 / Nav2 local planner that keeps a mobile robot on the centreline of narrow crop aisles using only a depth camera. It extends the
[Timed Elastic Band (TEB)](https://github.com/rst-tu-dortmund/teb_local_planner) model predictive controller with an online aisle estimator and a width-adaptive centring cost. The centring acts only inside the crop rows; in wide corridors, or whenever the aisle is not perceived, the controller reduces exactly to the original TEB.

It was developed for Mediterranean (*raspa y amagado*) greenhouses and is evaluated in the open-source [ROS 2 greenhouse benchmark](https://github.com/FerCanAra/robotics_benchmark_greenhouse) against TEB, MPPI, DWB and RPP, without a prior map and with camera-only perception.



You can watch the full video at [YouTube](). The algotihm has been tested on ROS2 Humble, Ubuntu 22.04 LTS.

*[Automatic, Robotics and Mechatronics](https://arm.ual.es/arm-group/) group (ARM - TEP197), Departments of Informatics, University of Almeria, Almería, Spain*

*Gmail: fernando.ca@ual.es*

*Website: https://linktr.ee/FerCanAra*

> **Note:** Process tested with on 18th October 2026.

🗃️ Repository structure
------------------
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

📑 Citation
------------------
*Paper citation*
------------------
```
@article{canadas2026acteb,
AUTHOR = {Fernando Cañadas-Aránega, José L. Guzmán, and José C. Moreno},
TITLE = {Vision-Based Aisle Estimation and Width-Adaptive Centring for Nonlinear Timed-Elastic-Band MPC of Mobile Robots in Unmapped Mediterranean Greenhouses},
}
```
*Software citation*
------------------
```
@software{softwareacteb,
  author  = {Ca{\~n}adas-Ar{\'a}nega, Fernando},
  title   = {Vision-Based Aisle Estimation and Width-Adaptive Centring for Nonlinear Timed-Elastic-Band MPC of Mobile Robots in Unmapped Mediterranean Greenhouses},
  version = {0.0.1},
  year    = {2026},
  doi     = {https://doi.org/10.5281/zenodo.23451492},
  url     = {https://github.com/FerCanAra/ac_teb_local_planner}
}
```
📜 License
--------------------
This project is distributed under the **BSD 3-Clause License**.

Copyright © 2026, Individual contributors  
Project owner: Fernando Cañadas Aránega <fernando.ca@ual.es> (University of Almeria) and collaborators

See the [LICENSE](LICENSE) file for full license text.

⚙️ Prerequisites
--------------------
In order to use the algotihm, you must have the following packages installed:

1. [Robot Operating System (ROS 2) Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). It is recommended to install the full desktop.
2. [Benchmark Robotics for greenhouses](https://github.com/FerCanAra/robotics_benchmark_greenhouse.git). Please follow the readme repository file for installation instructions. To install it in the correct location, please read the ‘Install and build’ section.
3. Realsense Library. It is easily installed with the following command:
```
cd ~/ros2_ws/src # Create this path if you didn't have it available.
git clone https://github.com/realsenseai/realsense-ros.git
git clone https://github.com/realsenseai/librealsense.git
```
4. [GreenSeg Groundfloor Segmentation](https://github.com/FerCanAra/ros2_greenseg.git):Please follow the readme file for installation instructions.

🛠️ Install and build
--------------------
In order to run the greenseg, the official project repository must be installed. This can be done easily using the following commands.

````
mkdir -p ros2_ws/src && cd ros2_ws/src
git clone https://github.com/FerCanAra/robotics_benchmark_greenhouse.git
cd robotics_benchmark_greenhouse
git submodule update --init --recursive
git clone https://github.com/FerCanAra/ac_teb_local_planner.git
cd ../..
sudo apt install python3-scipy ros-$ROS_DISTRO-cv-bridge
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
````

🚀 Usage
--------------------

Run the command using benchmark category 2. The authors recommend using an NVIDIA graphics card to ensure correct operation. For the purposes of this article, a local workstation was used, equipped with an NVIDIA RTX 4060 GPU (8~GB), an Intel Core i7-13400 CPU, 32~GB of Kingston DDR4 RAM (3200~MHz), and a 1~TB Kingston NV2 NVMe SSD.

```bash
ros2 launch ac_teb_local_planner mpc_benchmark_bringup.launch.py  category:=2 controller:=ac_teb
```
| Argument | Values | Description |
|---|---|---|
| `controller` | `ac_teb`, `teb`, `mppi`, `dwb`, `rpp` | Local controller. `teb` is AC-TEB with the centring disabled (`aisle.enabled: False`). |
| `ecc_logger` | `True` / `False` | Record the run in `results_dir`. |
| `results_dir` | path (default `~/ECC`) | Output folder of the logger. |
| `use_rviz` | `True` / `False` | Start RViz. |


📊 Results 
--------------------

Route through two aisles of the benchmark greenhouse, no prior map, depth camera only. Mean ± std over the successful runs; `e_y` is the lateral error with
respect to the true aisle centre, evaluated along the straight sections of the rows.

| | DWB | RPP | TEB | MPPI | **AC-TEB** |
|---|---|---|---|---|---|
| Success [%] | 20 | 100 | 80 | 100 | **100** |
| Time [s] | 76.8 | 66.9 ± 2.2 | 65.6 ± 10.5 | 65.8 ± 0.4 | **65.3 ± 0.8** |
| `e_y` RMS [cm] | 25.3 | 24.3 ± 1.5 | 26.3 ± 5.6 | 18.1 ± 2.7 | **14.2 ± 1.5** |
| max \|`e_y`\| [cm] | 57.4 | 46.6 ± 1.4 | 49.2 ± 1.8 | 39.7 ± 2.3 | **35.5 ± 4.7** |
| Min. clearance [cm] | 9.4 | 8.4 ± 4.1 | 3.5 ± 5.3 | **15.4 ± 4.8** | 5.9 ± 5.4 |
| Comp. time [ms] | – | – | 2.9 ± 0.2 | – | 3.8 ± 0.1 |

AC-TEB reduces the RMS lateral error by 46 % with respect to TEB, which shares all its parameters except the centring term, and by 22 % with respect to MPPI,
with the same route time. MPPI keeps a larger corner clearance because it drives straighter; reducing the residual heading oscillation of AC-TEB is future work.

## Rerformance Analysis and Data Logging

Every run is written to `~/ECC/<date>_<controller>[_<tag>]/`:

   - `timeseries.csv`: pose, true lateral error `e_y`, true width, clearance, computation time and, for AC-TEB, the aisle estimate and `γ_c`, at 20 Hz;
   - `summary.csv`: success, time, `e_y` RMS / max, minimum clearance, computation time and estimation errors.

The ground truth is computed from the known plant layout of the world file
   (centre between the inner edges of the plant collision boxes), never from
   the robot's own estimate.
3. Build the table and figures (no ROS needed):
   ```bash
   python3 script/ecc_plots.py --results ~/ECC
   ```
