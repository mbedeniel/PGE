# mfja_yaskawa: Yaskawa HC10 / HC10DT of Room 315

Description, controllers and Gazebo simulation of the two Yaskawa arms of the MFJA Room 315 cell.
The simulated arms offer **the same ROS 2 interface as the real ones**, which run the vendor driver
MotoROS2 ([aip-primeca-occitanie/mfja_ros2_yaskawa](https://github.com/aip-primeca-occitanie/mfja_ros2_yaskawa)):
same namespaces, joint names, action, services and status topic. Client code (scripts, MoveIt)
written for the real robots therefore runs unchanged against the simulation.

They replace the Yaskawa models of the `mfja_3rd_floor_gz` twin (branch `INTERNSHIP-ALI-2026`),
following the organisation of the MOTKIN example (course notes, ch. 22–23).

| Package | Content |
|---|---|
| `mfja_yaskawa` | Metapackage, this documentation |
| `mfja_yaskawa_description` | Root files `robots/hc10_robot.urdf.xacro` and `robots/hc10dt_robot.urdf.xacro`, arm macros and meshes from Yaskawa's ROS 2 support package, `ros2_control/` (mock hardware), `gazebo/` (gz_ros2_control), `show.launch.py` |
| `mfja_yaskawa_bringup` | Controller configuration, launch with mock hardware (no simulator) |
| `mfja_yaskawa_gazebo` | Gazebo launches (empty world or the running twin), cell file |
| `mfja_yaskawa_motoros2_sim` | Imitation of the MotoROS2 interface in front of the simulated controllers |

Real cell: left = HC10DT, white covers, namespace `yaskawa_LEFT`;
right = HC10, blue covers, namespace `yaskawa_RIGHT`.

## Interface (identical in simulation and on the real robots)

In the namespace of each robot (`/yaskawa_LEFT/...`, `/yaskawa_RIGHT/...`):

| Name | Type | Role |
|---|---|---|
| `follow_joint_trajectory` | action `control_msgs/action/FollowJointTrajectory` | Execute a trajectory |
| `joint_states` | topic `sensor_msgs/msg/JointState` | Joints `joint_1` … `joint_6` |
| `robot_status` | topic `industrial_msgs/msg/RobotStatus` | Mode, servos, motion, errors |
| `start_traj_mode` | service `motoros2_interfaces/srv/StartTrajMode` | Servos on, trajectory mode (needed before any motion) |
| `stop_traj_mode` | service `std_srvs/srv/Trigger` | Leave trajectory mode |
| `reset_error` | service `motoros2_interfaces/srv/ResetError` | Clear errors |
| `read_single_io`, `write_single_io` | services `motoros2_interfaces/srv/ReadSingleIO`, `WriteSingleIO` | Controller IO (gripper requests) |
| `tf`, `tf_static` | topics | Transforms of the arm |

Rules applied in simulation as MotoROS2 does: `start_traj_mode` first; 2 to 200 points;
all six joints; positions and velocities in every point; strictly increasing times; zero final
velocity; first point equal to the current position. A refused goal returns the MotoROS2 code
(for example `-100202`: too many points; `-100106`: servos off, call `start_traj_mode`).

Gripper: on the real robots a background job opens, closes or initialises the gripper when
OT#101, OT#102 or OT#100 is set; these outputs can be written with `write_single_io`
(probably addresses 10134, 10135, 10133, to confirm on the pendant). The simulation stores the
values and logs the request; the simulated gripper is not modelled yet.

## Install and build

The MotoROS2 message and service definitions are not in apt; they are built from source, as on
the PC that drives the real robots:

```bash
cd ~/mfja_ws
git clone -b master https://github.com/yaskawa-global/motoros2_client_interface_dependencies.git src/motoros2_client_interface_dependencies
vcs import --input src/motoros2_client_interface_dependencies/source_deps.repos src/
rosdep install --from-paths src/PGE/yaskawa src/motoros2_client_interface_dependencies src/yaskawa-global src/ros-industrial --ignore-src -y
colcon build --symlink-install --packages-up-to mfja_yaskawa
source install/setup.bash
```

Sourcing the workspace adds the description to `GZ_SIM_RESOURCE_PATH` so that Gazebo finds the meshes.

## Test step by step

1. **Model only (RViz, joint sliders):**
   ```bash
   ros2 launch mfja_yaskawa_description show.launch.py robot_model:=hc10dt
   ```
2. **Mock hardware (no simulator, wall-clock time):**
   ```bash
   ros2 launch mfja_yaskawa_bringup yaskawa_mock.launch.py robot_model:=hc10
   ```
3. **Gazebo, empty world, both arms:**
   ```bash
   ros2 launch mfja_yaskawa_gazebo yaskawa_gazebo.launch.py
   ```
4. **Inside the twin** (started without its own Yaskawa models, same Gazebo partition):
   ```bash
   ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py robots:=none gz_partition:=pge
   ros2 launch mfja_yaskawa_gazebo yaskawa_gazebo.launch.py start_gazebo:=false gz_partition:=pge
   ```
   Start the simulation (play button) if Gazebo opens paused. Full floor: add `world_name:=mfja_3rd_floor`.

Same commands as on the real robots (replace `yaskawa_RIGHT` by `yaskawa_LEFT` for the other arm):

```bash
ros2 topic echo --once /yaskawa_RIGHT/robot_status
ros2 service call /yaskawa_RIGHT/start_traj_mode motoros2_interfaces/srv/StartTrajMode {}
ros2 action send_goal /yaskawa_RIGHT/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [
    {positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0}},
    {positions: [0.5, 0.3, 0.4, 0.0, -0.5, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 4}}]}}"
ros2 service call /yaskawa_RIGHT/write_single_io motoros2_interfaces/srv/WriteSingleIO "{address: 10134, value: 1}"
ros2 service call /yaskawa_RIGHT/stop_traj_mode std_srvs/srv/Trigger {}
```

The first point must be the current position (all zeros right after launch).

## Using the real-robot tools against the simulation

With `mfja_ros2_yaskawa` in the same workspace:

- **Trajectory script, real-robot mode** (no `--sim`): works against steps 2 to 4.
  ```bash
  ros2 run mfja_motoman_hc10 execute_trajectory.py $TRAJ_PATH/trajectory.csv --robot right
  ```
- **MoveIt, real-robot mode**: use the mock launch (MoveIt there runs on wall-clock time) and let
  MoveIt publish the TF:
  ```bash
  ros2 launch mfja_yaskawa_bringup yaskawa_mock.launch.py robot_model:=hc10 publish_robot_tf:=false
  ros2 launch motoman_hc10_moveit_config demo_mfja.launch.py robot:=right hardware:=real
  ```

## Still to check on the real robots

See `docs/yaskawa/verifications_yaskawa_reel.md`: MotoROS2 configuration of each controller
(namespace, joint names, TF), gripper IO addresses and behaviour, base placement in the room,
speeds. Inertias are placeholders (position control only).
