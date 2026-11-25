# PGE Usine 4.0 collaborative: MFJA Room 315

Team repository of the 2026–2027 UPSSITECH SRI PGE (Projet de Grande Envergure): a reconfigurable,
collaborative production cell in Room 315 of the MFJA, built on the existing digital twin and robot
software of the platform rather than from scratch.

## Repositories

| Repository | Role | Version used | In the workspace |
|---|---|---|---|
| [mbedeniel/PGE](https://github.com/mbedeniel/PGE) (this one) | Team packages and documents | `main` | `src/PGE` |
| [aip-primeca-occitanie/mfja_3rd_floor_gz](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz) | Digital twin of the 3rd floor: worlds, Montrac rail simulation, PlanSys2 supervisor | branch `INTERNSHIP-ALI-2026` | `src/mfja_3rd_floor_gz` |
| [aip-primeca-occitanie/mfja_ros2_yaskawa](https://github.com/aip-primeca-occitanie/mfja_ros2_yaskawa) | Real Yaskawa arms through MotoROS2: scripts, MoveIt configuration | `3e158ba` | `src/mfja_ros2_yaskawa` |
| [Yaskawa-Global/motoman_ros2_support_packages](https://github.com/Yaskawa-Global/motoman_ros2_support_packages) | Official Yaskawa robot descriptions | `a77f186` | `src/motoman_ros2_support_packages` |
| [yaskawa-global/motoros2_client_interface_dependencies](https://github.com/yaskawa-global/motoros2_client_interface_dependencies), [motoros2_interfaces](https://github.com/yaskawa-global/motoros2_interfaces), [industrial_core](https://github.com/ros-industrial/industrial_core) (messages only) | MotoROS2 message, service and action definitions | `873b5aa`, `0.2.0`, `d547cdc` | `src/motoros2_client_interface_dependencies`, `src/yaskawa-global/`, `src/ros-industrial/` |
| [b-robotized/motoros2_hw_interfaces](https://github.com/b-robotized/motoros2_hw_interfaces) | ros2_control hardware interface for MotoROS2: real arms on joint_trajectory_controller | `dd1827b` + local patch | `src/motoros2_hw_interfaces` |
| [aip-primeca-occitanie/ros_yaskawa_hc10](https://github.com/aip-primeca-occitanie/ros_yaskawa_hc10) | Former ROS 1 driver setup of the Yaskawa arms | — | Not used (reference only) |

The pinned versions are in [`pge.repos`](pge.repos) (development workspace) and [`pge_real_robots.repos`](pge_real_robots.repos) (PC that drives the robots).

## Workspace setup

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, `python3-vcstool`.

```bash
mkdir -p ~/mfja_ws/src && cd ~/mfja_ws/src
git clone git@github.com:mbedeniel/PGE.git
cd ~/mfja_ws
vcs import --skip-existing --input src/PGE/pge.repos src/
git -C src/motoros2_hw_interfaces switch -c pge/local-fixes
git -C src/motoros2_hw_interfaces apply ../PGE/patches/motoros2_hw_interfaces/0001-declare-realtime-tools-and-trajectory-msgs-dependencies.patch
git -C src/motoros2_hw_interfaces commit -am "build: declare realtime_tools and trajectory_msgs dependencies"
rosdep install --from-paths src/PGE src/motoros2_client_interface_dependencies src/yaskawa-global src/ros-industrial src/motoros2_hw_interfaces --ignore-src -y
colcon build --symlink-install --packages-select motoros2_hw_interfaces --cmake-args -DBUILD_TESTING=OFF
colcon build --symlink-install --packages-up-to mfja_yaskawa
colcon build --packages-select mfja_motoman_hc10
source install/setup.bash
```

`motoros2_hw_interfaces` is patched (see [`patches/motoros2_hw_interfaces`](patches/motoros2_hw_interfaces/README.md))
and built without its unit tests. `mfja_motoman_hc10` is built without `--symlink-install`: its scripts are not marked executable in
its repository, so only a copy made at install time can be run with `ros2 run`.

The digital twin has its own build and launch instructions in its README. On our machines it runs
with the local patches of [`patches/mfja_3rd_floor_gz`](patches/mfja_3rd_floor_gz/README.md),
applied on a local branch that is never pushed.

## Workspace on the PC that drives the robots

The PC connected to the robots (MFJA, room 315) only needs the real-arm packages: `pge_real_robots.repos`
lists them, MotoROS2 message definitions included. Workspaces already on that PC are left untouched: ours is
a separate workspace, built and used in a shell that loads ROS only (the PC's `~/.bashrc` loads another
workspace, which would otherwise mix with this one).

Open that shell first, on its own (`bash --norc`), then run the rest in it:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/pge_ws/src && cd ~/pge_ws/src
git clone https://github.com/mbedeniel/PGE.git
cd ~/pge_ws
vcs import --input src/PGE/pge_real_robots.repos src/
git -C src/motoros2_hw_interfaces apply ../PGE/patches/motoros2_hw_interfaces/0001-declare-realtime-tools-and-trajectory-msgs-dependencies.patch
rosdep check --from-paths src --ignore-src
colcon build --symlink-install --packages-up-to mfja_yaskawa_bringup --cmake-args -DBUILD_TESTING=OFF
```

`rosdep check` only reports missing system packages. Those of `mfja_yaskawa_gazebo` (Gazebo, `ros_gz_*`) can be
missing: that package is not built on this PC. Every terminal used with the robots then starts with
`bash --norc` on its own, followed by:

```bash
source /opt/ros/jazzy/setup.bash
source ~/pge_ws/install/setup.bash
```

The micro-ROS agent keeps running as set up on that PC. Use of the real arms: `yaskawa/mfja_yaskawa/README.md`,
section "Real robots on ros2_control".

## Contents

| Path | Content |
|---|---|
| [`yaskawa/`](yaskawa/mfja_yaskawa/README.md) | Yaskawa HC10 / HC10DT: description, controllers, Gazebo simulation with the same ROS 2 interface as the real robots (MotoROS2) |
| [`patches/mfja_3rd_floor_gz/`](patches/mfja_3rd_floor_gz/README.md) | Local fixes for the twin, kept as patches (Gazebo GUI started after the robots are spawned) |
| [`patches/motoros2_hw_interfaces/`](patches/motoros2_hw_interfaces/README.md) | Local build fix for the MotoROS2 hardware interface |
| [`docs/montrac/questions.md`](docs/montrac/questions.md) | Questions on the real Montrac (controller, link, devices, safety) and leads collected so far |
| [`docs/yaskawa/real_robot_checks.md`](docs/yaskawa/real_robot_checks.md) | Checks to run on the real Yaskawa arms |
| [`docs/yaskawa/real_robot_ros2_control_tests.md`](docs/yaskawa/real_robot_ros2_control_tests.md) | Real Yaskawa arms on ros2_control: robot PC setup, test results, next steps |

## Conventions

- Packages are grouped by function under a metapackage (description, bringup, Gazebo simulation).
- Each robot runs in its own namespace; simulation and real robots expose the same ROS 2 interface.
- No hard-coded IP address, local path or personal identifier in code: values live in configuration files or launch arguments.
- External repositories are pinned in `pge.repos`; releases are tagged at each milestone.
