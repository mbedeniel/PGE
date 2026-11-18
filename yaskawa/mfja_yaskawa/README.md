# mfja_yaskawa: Yaskawa HC10 / HC10DT of Room 315

Description, controllers and Gazebo simulation of the two Yaskawa arms of the MFJA Room 315 cell.
They replace the Yaskawa models of the `mfja_3rd_floor_gz` twin (branch `INTERNSHIP-ALI-2026`),
following the organisation of the MOTKIN example (course notes, ch. 22–23):
packages grouped by function under this metapackage, one `gz_sim` argument to switch the
ros2_control part, the same controllers with mock hardware and in Gazebo.

| Package | Content |
|---|---|
| `mfja_yaskawa` | Metapackage, this documentation |
| `mfja_yaskawa_description` | `robots/hc10_robot.urdf.xacro`, `robots/hc10dt_robot.urdf.xacro` (root files), arm macros, meshes, `ros2_control/` (mock hardware), `gazebo/` (gz_ros2_control), `show.launch.py` |
| `mfja_yaskawa_bringup` | Controller configuration, launch with mock hardware. The real-robot launch will go here |
| `mfja_yaskawa_gazebo` | Gazebo launches (empty world or the running twin), cell file |

Real cell (README of `aip-primeca-occitanie/ros_yaskawa_hc10`): left = HC10DT, white covers;
right = HC10, blue covers.

## What changes compared with the twin

| | Twin (`INTERNSHIP-ALI-2026`) | These packages |
|---|---|---|
| HC10DT kinematics | Copy of the HC10 kinematics | `hc10dt_b10` description used on the real left robot (different joint frames, joint 3 range −95°…265°) |
| Control | Custom Gazebo plugin, topic `/<robot>/joint_trajectory` | `gz_ros2_control` + `joint_trajectory_controller` + `joint_state_broadcaster` |
| Command interface | Topic only | Action `<ns>/arm_controller/follow_joint_trajectory`, topic `<ns>/arm_controller/joint_trajectory` |
| Names | `yaskawa_hc10_1`, `yaskawa_hc10dt_1` | `yaskawa_RIGHT` (HC10), `yaskawa_LEFT` (HC10DT) |
| Joint names | `joint_1_s` … `joint_6_t` | Unchanged |
| Gripper | Simulated | Not yet (real gripper interface unknown) |

## Install and build

```bash
cd ~/mfja_ws
rosdep install --from-paths src/PGE/yaskawa --ignore-src -y
colcon build --symlink-install --packages-up-to mfja_yaskawa
source install/setup.bash
```

Sourcing the workspace adds the description to `GZ_SIM_RESOURCE_PATH` so that Gazebo finds the meshes.

## Test step by step

1. **Model only (RViz, joint sliders):**
   ```bash
   ros2 launch mfja_yaskawa_description show.launch.py robot_model:=hc10dt
   ```
2. **Controllers with mock hardware (no simulator):**
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
   For the full floor, add `world_name:=mfja_3rd_floor` to the second command.

Checks for steps 2 to 4 (replace `yaskawa_RIGHT` by `yaskawa_LEFT` for the other arm):

```bash
ros2 control list_controllers -c /yaskawa_RIGHT/controller_manager
ros2 topic echo --once /yaskawa_RIGHT/joint_states
ros2 action send_goal /yaskawa_RIGHT/arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t],
    points: [{positions: [0.5, 0.3, 0.4, 0.0, -0.5, 0.0], time_from_start: {sec: 4}}]}}"
```

Expected: both controllers `active`, joint states published, the action returns `SUCCESSFUL`.

## Interfaces: simulation vs real robots

The real arms are driven through the Yaskawa controller (no access to the low level), so the
simulation must expose the same interface level, not the same ros2_control hardware.

| | Simulation | Real robots, ROS 1 driver (`ros_yaskawa_hc10`) | Real robots, ROS 2 |
|---|---|---|---|
| Trajectory action | `<ns>/arm_controller/follow_joint_trajectory` | `<ns>/joint_trajectory_action` | To confirm |
| Trajectory topic | `<ns>/arm_controller/joint_trajectory` | `<ns>/joint_path_command` | To confirm |
| Joint states | `<ns>/joint_states` | `<ns>/joint_states` | To confirm |
| Enable / status | None yet | `robot_enable`, `robot_disable`, `robot_status` | To confirm |

Once the ROS 2 interface of the real robots is known, a small adapter will expose the same names in
simulation, so that MoveIt and the application code are identical on both sides.

## Not verified on the real robots

Only the kinematic parameters, joint limits, joint names and the left/right models come from the
files used to drive the real robots. Still to check (see `docs/yaskawa/verifications_yaskawa_reel.md`):
the +90° yaw on the HC10DT `joint_1_s` origin, the base placement, the ROS 2 interface names,
the real timing and speed limits, the grippers. Inertias are placeholders (position control only).
