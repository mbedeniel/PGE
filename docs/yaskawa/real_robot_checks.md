# Real Yaskawa arms: checks

- **Purpose:** confirm that the simulation (`yaskawa/`, packages `mfja_yaskawa_*`) matches the two real Yaskawa
  arms of room 315, and complete what is missing (grippers).
- **With:** the MFJA platform manager. Reference: the platform wiki on Moodle and the repository
  [mfja_ros2_yaskawa](https://github.com/aip-primeca-occitanie/mfja_ros2_yaskawa).

## Known so far

- Driver: MotoROS2 0.2.1 (commit `41ac074`) on the controllers, micro-ROS agent (docker) on the room PC.
  Teach-pendant key on REMOTE, safety doors closed.
- Interface, in `/yaskawa_LEFT/` (HC10DT) and `/yaskawa_RIGHT/` (HC10): action `follow_joint_trajectory`,
  `joint_states`, `robot_status`, services `start_traj_mode`, `start_point_queue_mode`, `queue_traj_point`,
  `stop_traj_mode`, `reset_error`, `read_single_io`, `write_single_io`. The `follow_joint_trajectory`
  action accepts at most 200 points; the ros2_control path (`real_robot_ros2_control_tests.md`) has no such limit.
- Arm descriptions: Yaskawa's `motoman_hc10_support` package
  ([motoman_ros2_support_packages](https://github.com/Yaskawa-Global/motoman_ros2_support_packages)); joints
  `joint_1` … `joint_6`; no 90° offset on joint 1 of the HC10DT. The simulation uses exactly these models.
  Joint names `joint_1` … `joint_6` confirmed on the right arm on 24/11/2026.
- Grippers: not yet commanded from ROS 2. A background job on each controller runs init, open or close when
  OT#100, OT#101 or OT#102 is set (teach-pendant keys). The robot group must be selected (Shift + Robot).

## Checks

| No. | Item | How to check | Status | Result |
|---|---|---|---|---|
| 1 | MotoROS2 configuration of each controller | Get `motoros2_config.yaml` of both robots: namespace, joint names, `publish_tf`, `namespace_tf`, `tf_frame_prefix`. | Right arm: namespace and joint names confirmed; files not seen | |
| 2 | Gripper: addresses | On the teach pendant, logical address of OT#100, OT#101, OT#102. | Computed: 10133, 10134, 10135 (Yaskawa numbering) | |
| 3 | Gripper: command from ROS 2 | Robot stopped, gripper empty, one person at the emergency stop. Read then write OT#101 with `write_single_io` (commands below) and observe. Pulse or level? Who resets the output? Execution delay? | To test | |
| 4 | Gripper: state | Is there an input telling gripper open, closed, initialised, in fault? Address? | Unknown | |
| 5 | Gripper: reference path | The student script `supervision_IHM.py` drives a Zimmer gripper through a Turck IO-Link gateway (Modbus TCP). Which path to keep: controller outputs or gateway? | To decide | |
| 6 | HC10DT joint 1 zero | Joint 1 at 0: note the arm direction relative to the table, compare with `show.launch.py robot_model:=hc10dt`. | The ROS 2 model has no offset; visual check | |
| 7 | Geometry (3 or 4 poses per robot) | On the teach pendant: angles of the 6 joints (°) and flange position in ROBOT coordinates, tool 0. Same angles in simulation, read `base` → `flange`. Expected difference < 5 mm. | To do | |
| 8 | Limits and speeds | Controller soft limits and maximum speeds, especially in collaborative mode. | Model: HC10 U −5°…355°, HC10DT U −95°…265°, others ±180°; 130, 130, 180, 180, 250, 250 °/s | |
| 9 | Same trajectory, real and simulated | Run the same trajectory on the robot and in simulation; record `joint_states` (`ros2 bag record`); compare duration and differences. | The simulation accepts the same commands | |
| 10 | Placement | Position and orientation of each base relative to its rail and table, table height. | Poses taken from the twin | |
| 11 | Tool and load | Tool number, TCP and mass declared in the controller. | Unknown | |
| 12 | End-of-trajectory errors | Codes -500302 and -100303 sometimes returned although the motion is correct (MotoROS2 action): known cause? | Reported in the `mfja_ros2_yaskawa` README | |

## Commands

Display a robot with one slider per joint (checks 6 and 7):

```bash
ros2 launch mfja_yaskawa_description show.launch.py robot_model:=hc10dt
ros2 run tf2_ros tf2_echo base flange
```

In the Yaskawa models, `base` is the controller's ROBOT frame and `flange` the flange (tool 0).

Gripper (check 3), on the left robot; addresses to confirm in check 2:

```bash
ros2 service call /yaskawa_LEFT/read_single_io motoros2_interfaces/srv/ReadSingleIO "{address: 10134}"
ros2 service call /yaskawa_LEFT/write_single_io motoros2_interfaces/srv/WriteSingleIO "{address: 10134, value: 1}"
ros2 service call /yaskawa_LEFT/write_single_io motoros2_interfaces/srv/WriteSingleIO "{address: 10134, value: 0}"
```

Record a trajectory (check 9):

```bash
ros2 bag record -o real_yaskawa_test /yaskawa_LEFT/joint_states /yaskawa_LEFT/robot_status
```
