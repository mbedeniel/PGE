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
| `mfja_yaskawa_description` | Root files `robots/hc10_robot.urdf.xacro` and `robots/hc10dt_robot.urdf.xacro`, arm macros and meshes from Yaskawa's ROS 2 support package, `ros2_control/` (joints, real arms, mock hardware, gripper), `gazebo/` (gz_ros2_control), `show.launch.py` |
| `mfja_yaskawa_bringup` | Controller configuration shared by all modes, launch with mock hardware (no simulator), launch of the real arms on ros2_control, scripts `move_joint.py` and `gripper.py` |
| `mfja_yaskawa_gazebo` | Gazebo launches (empty world or the running twin), cell file |
| `mfja_yaskawa_gz_hardware` | Gazebo hardware system with the ros2_control interfaces of the real arms (C++ plugin loaded by gz_ros2_control) |
| `mfja_yaskawa_motoros2_sim` | Imitation of the MotoROS2 interface in front of the simulated controllers |

Real cell: left = HC10DT, white covers, namespace `yaskawa_LEFT`;
right = HC10, blue covers, namespace `yaskawa_RIGHT`.

## Same controllers in simulation and on the real arms

The client asked for a simulation as close as possible to the real robots, with the same controllers.
Following the MOTKIN example of the course notes (ch. 22–23), one xacro argument chooses the hardware,
and everything above it is shared:

```
                                 one controller configuration          hardware chosen in the xacro
client code ──▶ arm_controller      (joint_trajectory_controller) ──▶  gz_sim:=true             Gazebo (mfja_yaskawa_gz_hardware)
(move_joint.py, gripper_controller  (forward command, gripper)        use_mock_hardware:=true  mock (rehearses the real arm)
 gripper.py,    joint_state_broadcaster                               use_real_hardware:=true  MotoROS2 hardware interface
 MoveIt)                                                                                        + GPIOSystem (gripper)
```

**Identical everywhere:** the ros2_control interfaces of the hardware (per joint: commands position and
velocity, states position and velocity; gripper GPIO `init`, `open`, `close`), the controllers and their
settings (one file, `config/yaskawa_controllers.yaml`: 50 Hz, tolerances, gripper requests), the topics
and actions of the controllers
(`<ns>/arm_controller/follow_joint_trajectory`, `<ns>/gripper_controller/commands`,
`<ns>/joint_state_broadcaster/joint_states`), and the scripts. `<ns>/joint_states` is published by
MotoROS2 on the real arms and by its imitation (`mfja_yaskawa_motoros2_sim`) in simulation.

**What changes with the hardware:**

| | Gazebo | Mock | Real arm |
|---|---|---|---|
| Hardware system | `mfja_yaskawa_gz_hardware/YaskawaGazeboSystem` | `mock_components/GenericSystem` | `motoros2_hw_interfaces/MotoROS2HardwareInterface` |
| Joint motion | velocity command + correction of the position error, in the physics engine | the state is the command | the robot controller follows the points queued in MotoROS2 |
| Gripper GPIO | commands copied to the states | commands copied to the states | `GPIOSystem` (MotoROS2 IO services) |
| Clock | simulated (forced by the Gazebo plugin) | wall clock | wall clock |

**Why our own Gazebo hardware system.** The MotoROS2 hardware interface requires a position and a velocity
command per joint: each point it queues in MotoROS2 carries both, and MotoROS2 uses the velocities to move
smoothly between points. The default Gazebo system of gz_ros2_control (`gz_ros2_control/GazeboSimSystem`)
cannot take both: when a joint receives both, it applies the velocity alone and ignores the position.
`mfja_yaskawa_gz_hardware` is a custom simulation plugin for gz_ros2_control (the mechanism described in
the gz_ros2_control documentation; the MOTKIN example of the course does the same with its own Gazebo
system). It takes exactly the interfaces of the real arms and checks them when it starts. For each joint it
applies the commanded velocity plus a correction of the position error (`position_gain`, 10 1/s, set in
`gazebo/yaskawa_gazebo.urdf.xacro`), as the robot controller follows the points it receives. Joints whose
commands are not claimed by a controller are held still, as the brakes hold the real arms. The gripper
GPIO copies each command to its state, like the mock: `gripper.py` works in Gazebo, but the fingers
themselves are not simulated.

**Same start-up.** In every mode `arm_controller` is loaded inactive (launch argument
`activate_arm_controller`, default `false`) and the arm does not move until a MotoROS2 motion mode is
started. MotoROS2 has two modes, and only one can be active at a time:

| | Trajectory mode | Point-queue mode |
|---|---|---|
| Started by | `start_traj_mode` | `start_point_queue_mode` |
| Used by | MotoROS2's own `<ns>/follow_joint_trajectory` (scripts and MoveIt of `mfja_ros2_yaskawa`) | `arm_controller` through ros2_control (`move_joint.py`, our launches) |
| Then | send the trajectory | activate `arm_controller` by hand |
| Refused in this mode | `start_point_queue_mode` | `start_traj_mode`, and goals sent to `<ns>/follow_joint_trajectory` |
| Stop | `stop_traj_mode` | deactivate `arm_controller`, then `stop_traj_mode` |

The MotoROS2 imitation (`mfja_yaskawa_motoros2_sim`) applies these rules with the MotoROS2 result codes.
Two differences remain, both due to the simulation:

- In trajectory mode the real MotoROS2 executes the trajectory itself, without ros2_control. In
  simulation the trajectory goes through `arm_controller`, so `start_traj_mode` activates it and
  `stop_traj_mode` deactivates it again. The tools of `mfja_ros2_yaskawa` therefore run unchanged.
- A simulated arm cannot refuse a command: if `arm_controller` is activated without
  `start_point_queue_mode`, the simulated arm still moves, whereas the real arm refuses every point.
  On the real arm, the hardware interface also keeps sending the last point after `arm_controller` is
  deactivated, so its launch must be stopped (Ctrl+C) before `stop_traj_mode` is accepted.

While `arm_controller` is inactive, the joints are held where they are: by the brakes on the real arms,
by the Gazebo plugin (`hold_joints`, on by default) and by the mock hardware in simulation.

## Interface (identical in simulation and on the real robots)

In the namespace of each robot (`/yaskawa_LEFT/...`, `/yaskawa_RIGHT/...`):

| Name | Type | Role |
|---|---|---|
| `follow_joint_trajectory` | action `control_msgs/action/FollowJointTrajectory` | Execute a trajectory |
| `joint_states` | topic `sensor_msgs/msg/JointState` | Joints `joint_1` … `joint_6` |
| `robot_status` | topic `industrial_msgs/msg/RobotStatus` | Mode, servos, motion, errors |
| `start_traj_mode` | service `motoros2_interfaces/srv/StartTrajMode` | Servos on, trajectory mode (needed before `follow_joint_trajectory`) |
| `start_point_queue_mode` | service `motoros2_interfaces/srv/StartPointQueueMode` | Servos on, point-queue mode (needed before activating `arm_controller`) |
| `stop_traj_mode` | service `std_srvs/srv/Trigger` | Leave the active mode |
| `reset_error` | service `motoros2_interfaces/srv/ResetError` | Clear errors |
| `read_single_io`, `write_single_io` | services `motoros2_interfaces/srv/ReadSingleIO`, `WriteSingleIO` | Controller IO (gripper requests) |
| `tf`, `tf_static` | topics | Transforms of the arm |

Rules applied in simulation as MotoROS2 does: one motion mode at a time (see "Same start-up" above);
`follow_joint_trajectory` only in trajectory mode, so `start_traj_mode` first; 2 to 200 points;
all six joints; positions and velocities in every point; strictly increasing times; zero final
velocity; first point equal to the current position. As on the real robots, a trajectory starts as
soon as it is received (`header.stamp` is ignored). A refused goal returns the MotoROS2 code
(for example `-100202`: too many points; `-100106`: servos off, call `start_traj_mode`;
`-100209`: wrong mode, point-queue mode is active).

Gripper: on the real robots a background job opens, closes or initialises the gripper when
OT#101, OT#102 or OT#100 goes from 0 to 1; these outputs are written with `write_single_io` at
addresses 10134, 10135, 10133 (open and close checked on the right arm). The MotoROS2 imitation stores
the values and logs the request; see also the section "Gripper" below.

## Install and build

The workspace is described in the [root README](../../README.md): the external repositories,
including the MotoROS2 message definitions (not in apt), are pinned in `pge.repos`.

```bash
cd ~/mfja_ws
vcs import --skip-existing --input src/PGE/pge.repos src/
rosdep install --from-paths src/PGE src/motoros2_client_interface_dependencies src/yaskawa-global src/ros-industrial src/motoros2_hw_interfaces --ignore-src -y
colcon build --symlink-install --packages-select motoros2_hw_interfaces --cmake-args -DBUILD_TESTING=OFF
colcon build --symlink-install --packages-up-to mfja_yaskawa
source install/setup.bash
```

Before the first build of `motoros2_hw_interfaces`, apply its local patch
([`patches/motoros2_hw_interfaces`](../../patches/motoros2_hw_interfaces/README.md)).

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
   Run the second command once the Gazebo window shows the room: a model spawned while the GUI is
   still starting is not drawn. Start the simulation (play button) if Gazebo opens paused. Full floor:
   add `world_name:=mfja_3rd_floor`.

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

Through ros2_control, as on the real arms in [ros2_control mode](#real-robots-on-ros2_control-motoros2_hw_interfaces):

```bash
ros2 service call /yaskawa_RIGHT/start_point_queue_mode motoros2_interfaces/srv/StartPointQueueMode {}
ros2 control switch_controllers -c /yaskawa_RIGHT/controller_manager --activate arm_controller
ros2 run mfja_yaskawa_bringup move_joint.py --robot right --joint 6 --delta 0.1 --back
ros2 control switch_controllers -c /yaskawa_RIGHT/controller_manager --deactivate arm_controller
ros2 service call /yaskawa_RIGHT/stop_traj_mode std_srvs/srv/Trigger {}
```

For quick tests in simulation only, `activate_arm_controller:=true` on the launch starts `arm_controller`
active.

## Using the real-robot tools against the simulation

The tools of [mfja_ros2_yaskawa](https://github.com/aip-primeca-occitanie/mfja_ros2_yaskawa), used on the
real robots, run unchanged against steps 2 to 4. This is the acceptance test of the simulation.

**Trajectory script, real-robot mode** (without `--sim`), with the simulation of step 3 running.
Build `mfja_motoman_hc10` without `--symlink-install`: its scripts are not marked executable in its
repository, so only the copy made at install time can be run.

```bash
colcon build --packages-select mfja_motoman_hc10
source install/setup.bash
export TRAJ_PATH=~/mfja_ws/src/mfja_ros2_yaskawa/mfja_motoman_hc10/trajectories
ros2 run mfja_motoman_hc10 execute_trajectory.py $TRAJ_PATH/test_right_1.csv --robot right
ros2 run mfja_motoman_hc10 execute_trajectory.py $TRAJ_PATH/trajectory.csv --robot left
```

Expected, for each arm: "Trajectory Executor for REAL hardware", trajectory mode enabled,
"Lead-In execution SUCCESSFUL", "Main CSV Trajectory execution SUCCESSFUL", then `stop_traj_mode`.
`test_right_1.csv` is meant for the HC10 (right) and `trajectory.csv` for the HC10DT (left): their
joint 3 values only fit the range of the matching arm.

**MoveIt, real-robot mode** (needs MoveIt 2: `sudo apt install ros-jazzy-moveit`): use the mock
launch, because this MoveIt launch runs on wall-clock time, and let MoveIt publish the TF:

```bash
colcon build --symlink-install --packages-up-to motoman_hc10_moveit_config motoman_hc10_support
ros2 launch mfja_yaskawa_bringup yaskawa_mock.launch.py robot_model:=hc10 publish_robot_tf:=false
ros2 launch motoman_hc10_moveit_config demo_mfja.launch.py robot:=right hardware:=real
```

## Real robots on ros2_control (motoros2_hw_interfaces)

The real arms can run on the same `joint_trajectory_controller` as the simulation. The hardware
plugin [`motoros2_hw_interfaces`](https://github.com/b-robotized/motoros2_hw_interfaces) runs on the
PC: it reads `<ns>/joint_states` from MotoROS2 and sends the command of every control cycle to the
MotoROS2 service `<ns>/queue_traj_point` (point-queue mode). Nothing is installed on the robot
controller. Trajectories are no longer limited to 200 points.

| | Simulation | Real arm, ros2_control |
|---|---|---|
| Launch | `yaskawa_gazebo.launch.py`, `yaskawa_mock.launch.py` | `yaskawa_real.launch.py` |
| Controllers file | `config/yaskawa_controllers.yaml` | the same |
| Command interfaces | position + velocity | the same (required by the plugin) |
| Trajectory action | `<ns>/arm_controller/follow_joint_trajectory` | the same |
| Gripper | `gripper_controller`; commands copied to the states | `gripper_controller` + `GPIOSystem` |
| `<ns>/joint_states` | MotoROS2 imitation (from `<ns>/joint_state_broadcaster/joint_states`) | MotoROS2 (ros2_control republishes it on `<ns>/joint_state_broadcaster/joint_states`) |

On the PC that drives the robots, these packages live in their own workspace, built and used in a
shell that loads ROS only: see "Workspace on the PC that drives the robots" in the [root README](../../README.md).

In this mode MotoROS2's own `follow_joint_trajectory` refuses goals: do not run the scripts of
`mfja_ros2_yaskawa` or its MoveIt launch at the same time. `move_joint.py` works in both cases.

### Procedure, one arm at a time

Before starting: the platform manager has agreed to the test, the teach pendant is in hand with its
emergency stop, the key is on REMOTE, nobody is in the arm's reach, and the micro-ROS agent runs.
Commands for the right arm; for the left one use `robot_model:=hc10dt`, `yaskawa_LEFT` and `--robot left`.

0. **Rehearsal in simulation first** (no robot): the mock follows the same start-up as the real arm.
   On a PC connected to the robots, use another ROS domain in both terminals, so that the mock arm and
   the real one, which share the same names, never meet:
   ```bash
   export ROS_DOMAIN_ID=42
   ros2 launch mfja_yaskawa_bringup yaskawa_mock.launch.py robot_model:=hc10
   ```
   ```bash
   export ROS_DOMAIN_ID=42
   ros2 service call /yaskawa_RIGHT/start_point_queue_mode motoros2_interfaces/srv/StartPointQueueMode {}
   ros2 control switch_controllers -c /yaskawa_RIGHT/controller_manager --activate arm_controller
   ros2 run mfja_yaskawa_bringup move_joint.py --robot right --joint 6 --delta 0.1 --back
   ```
   Then close both terminals: the next steps run in the robots' domain.
1. **MotoROS2 answers** (joints named `joint_1` … `joint_6`, both services listed):
   ```bash
   ros2 topic echo --once /yaskawa_RIGHT/joint_states
   ros2 service list | grep -E "yaskawa_RIGHT/(start_point_queue_mode|queue_traj_point)"
   ```
2. **ros2_control without motion** (`arm_controller` is loaded inactive, nothing is sent):
   ```bash
   ros2 launch mfja_yaskawa_bringup yaskawa_real.launch.py robot_model:=hc10
   ros2 control list_hardware_components -c /yaskawa_RIGHT/controller_manager
   ros2 control list_controllers -c /yaskawa_RIGHT/controller_manager
   ros2 topic echo --once /yaskawa_RIGHT/joint_state_broadcaster/joint_states
   ```
   The hardware component is `active`, `joint_state_broadcaster` active, `arm_controller` inactive,
   and the broadcaster's positions are those of `/yaskawa_RIGHT/joint_states` (not all zeros).
3. **Point-queue mode** (servos on, still no motion):
   ```bash
   ros2 service call /yaskawa_RIGHT/start_point_queue_mode motoros2_interfaces/srv/StartPointQueueMode {}
   ```
   Expected `result_code` value 1 (READY). Otherwise read the message and `robot_status`
   (`reset_error` clears alarms).
4. **Activate the controller**: the arm holds its position and must not move.
   ```bash
   ros2 control switch_controllers -c /yaskawa_RIGHT/controller_manager --activate arm_controller
   ros2 topic list | grep write_stats
   ros2 topic echo <write_stats topic listed above>
   ```
   `write_stats` counts the points sent and refused; failures should stay close to 0 %.
5. **Small motion** of the last joint, then back:
   ```bash
   ros2 run mfja_yaskawa_bringup move_joint.py --robot right --joint 6 --delta 0.1 --back
   ```
   Then other joints with small angles. Ctrl+C cancels the goal and the arm stops.
   More than 200 points: add `--points 300`.
6. **Stop**:
   ```bash
   ros2 control switch_controllers -c /yaskawa_RIGHT/controller_manager --deactivate arm_controller
   # Ctrl+C on the launch, then:
   ros2 service call /yaskawa_RIGHT/stop_traj_mode std_srvs/srv/Trigger {}
   ```

After an emergency stop or a MotoROS2 alarm: deactivate `arm_controller`, call `reset_error`, then
`start_point_queue_mode`, then activate `arm_controller` again. MotoROS2 refuses a first point that is
not the current position, so the arm cannot jump to an old command.

The control loop runs at 50 Hz (one MotoROS2 service call per cycle). If `write_stats` shows almost no
failures, 100 Hz can be tried (`update_rate` in `config/yaskawa_controllers.yaml`, which also sets the
simulation rate).

### Plan and execute in RViz (MoveIt)

`launch/yaskawa_moveit.launch.py` starts MoveIt and RViz on our `arm_controller`: a motion planned in RViz is
executed through `<ns>/arm_controller/follow_joint_trajectory`, as any other trajectory of this setup. The
planning model (URDF, SRDF, kinematics, joint limits, RViz layout) is the one of `motoman_hc10_moveit_config`
(repository `mfja_ros2_yaskawa`), reused unchanged; `config/moveit_controllers.yaml` replaces only its
execution, which uses MotoROS2's own action. MoveIt and that package are in the robot PC's own workspace, so
this launch runs by its path in a normal terminal, while the arm runs in a `pge_shell.sh` terminal:

1. Bring-up as above: `yaskawa_real.launch.py`, `start_point_queue_mode`, activation of `arm_controller`.
2. Normal terminal:
   ```bash
   ros2 launch ~/pge_ws/src/PGE/yaskawa/mfja_yaskawa_bringup/launch/yaskawa_moveit.launch.py robot:=right
   ```
3. In the MotionPlanning panel: velocity and acceleration scaling at 0.10; a small change of one joint in the
   Joints tab; Plan, check the preview, Execute.
4. Stop as above (deactivate `arm_controller`, Ctrl+C the arm launch, `stop_traj_mode`).

### Gripper

On each arm, a background job of the robot controller drives the gripper from three general-purpose
outputs: OT#100 init, OT#101 open, OT#102 close (MotoROS2 IO addresses 10133, 10134, 10135). The
gripper reacts when an output goes from 0 to 1; going back to 0 does nothing, and the job never resets
the outputs. A command is therefore a pulse (checked on the right arm on 25/11/2026 for open and close).

In ros2_control the gripper is a GPIO named `gripper` with interfaces `init`, `open`, `close`:
`motoros2_hw_interfaces/GPIOSystem` on the real arms (outputs read and written through
`read_single_io` / `write_single_io`, 10 Hz); in the mock and in Gazebo, each command is copied to its
state.
`gripper_controller` (`forward_command_controller/MultiInterfaceForwardCommandController`, data
`[init, open, close]`) writes the requests; it writes nothing until a command is published.
`gripper.py` sends one request as a pulse and checks each step on the states read back:

```bash
ros2 run mfja_yaskawa_bringup gripper.py --robot right close
ros2 run mfja_yaskawa_bringup gripper.py --robot right open
```

The robot group must be selected on the teach pendant (Shift + Robot) for the gripper to work. There is
no feedback of the gripper position yet: the states are the requests read back from the outputs. In
Gazebo the gripper requests go through the same interfaces, but the fingers are not simulated.

## Still to check on the real robots

See [`docs/yaskawa/real_robot_checks.md`](../../docs/yaskawa/real_robot_checks.md): MotoROS2 configuration of each controller
(namespace, joint names, TF), gripper IO addresses and behaviour, base placement in the room,
speeds. Inertias are placeholders (position control only).
