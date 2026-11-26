# Tests: real Yaskawa arms on ros2_control (motoros2_hw_interfaces)

Goal: drive the real arms with the same `joint_trajectory_controller` as in simulation, through the hardware
interface [`motoros2_hw_interfaces`](https://github.com/b-robotized/motoros2_hw_interfaces), which uses the
MotoROS2 "point queue" mode (nothing to install on the controllers). Choice agreed with the platform manager
and the client on 20/11/2026; it also lifts the 200-point limit per trajectory.

Full procedure: `yaskawa/mfja_yaskawa/README.md`, section "Real robots on ros2_control".

## Setup on the PC that drives the robots (MFJA, room 315)

- The PC must keep the fixed address that MotoROS2 on both controllers uses for the micro-ROS agent
  (addresses on the platform wiki). Its room-network connection must use manual addressing only.
- Separate workspace `~/pge_ws`, built from `pge_real_robots.repos`; the existing workspace of the PC is not
  modified. The tests of 24–25/11 were run in `bash --norc` shells, which skip `~/.bashrc` but keep what the
  terminal had already loaded: the PC's workspace was therefore still loaded, under ours. Since then the
  workspace is built and used in `pge_shell.sh`, which starts from an empty environment (root README).
- micro-ROS agent: the one already set up on the PC, unchanged.

Network issue fixed on 24/11/2026: the room-network connection of the PC also requested an address by DHCP.
With no DHCP server on that network, the activation failed every 45 s and the fixed address disappeared, so
the robots lost the agent intermittently. The connection now uses manual addressing.

## Results of 24/11/2026, right arm (HC10, `yaskawa_RIGHT`)

| Step | Result |
|---|---|
| a. `move_joint.py` on the simulated arm (`ROS_DOMAIN_ID=42`) | Passed |
| b. Real launch, `arm_controller` inactive | Hardware interface active (asynchronous, 50 Hz), positions read identical to MotoROS2's, services `start_point_queue_mode` and `queue_traj_point` present |
| c. `start_point_queue_mode` | READY (1), servos on |
| d. `arm_controller` activated | The arm does not move; command/measurement error ≈ 1e-5 rad |
| e. First motion (joint 6, 0.1 rad and back) | Passed on 25/11/2026 (see below) |

Points sent to MotoROS2 in 61 s of holding: 3050, of which 87 failed (2.85 %). All failures are replies that
arrived after the 30 ms timeout; MotoROS2 refused no point. Mean round trip of one call: 16.9 ms. 50 Hz is
therefore suitable; 100 Hz would fail most of the time.

Left arm (HC10DT): controller not reachable that day.

## Results of 25/11/2026, right arm

| Test | Result |
|---|---|
| Joint 6, 0.1 rad and back (`move_joint.py`, 100 points, peak 0.031 rad/s) | Passed, three times |
| Joints 1 to 5, 0.1 rad and back each | Passed |
| 600-point trajectory (joint 6, `--points 300 --back`) | Passed: the 200-point limit no longer applies |
| Joint 6, 0.3 rad in 10 s, and a 0.26 rad return at 0.082 rad/s | Passed |
| Cancel during motion (Ctrl+C) | Passed: the arm stops where it is, no jump back |
| Recovery sequence (deactivate, `reset_error`, `start_point_queue_mode`, activate) | Works; to repeat after a real emergency stop |
| `stop_traj_mode` while the launch still runs | Refused ("increment queue not empty"): stop the launch first |

Final positions read right after each result are within 0.0007 rad of the target; the arm settles on the
target shortly after. Points sent after about 4 min 40 s: 14104, of which 561 failed (3.98 %): 560 late
replies and 1 point refused by MotoROS2 (cause to read in the launch output).

Gripper, through `write_single_io` directly: open (10134) and close (10135) act on the change 0 → 1;
putting the output back to 0 does nothing; the robot controller never resets the outputs.

Gripper through ros2_control (`GPIOSystem` + `gripper_controller`, `gripper.py --robot right close`, then
`open`): passed. The outputs read 0 before the command, the pulse is sent and the gripper closes, then opens.
The first attempt failed only because the terminals still had `ROS_DOMAIN_ID=42` from the mock test.

## Same controllers in simulation, 25/11/2026 (development laptop)

After the alignment described in the Yaskawa README ("Same controllers in simulation and on the real arms"):

| Check | Mock | Gazebo |
|---|---|---|
| `joint_state_broadcaster` and `arm_controller` active | Yes | Yes (`arm_controller` commands: position) |
| `<ns>/joint_states`: one publisher (MotoROS2 imitation, as on the real arms) | Yes | Yes |
| `move_joint.py --robot right --joint 6 --delta 0.1 --back` | Passed | Passed |
| `gripper.py --robot right close` | Passed | Not simulated yet |
| `execute_trajectory.py` of `mfja_ros2_yaskawa`, real-robot mode, both arms | – | Passed (lead-in and main trajectory) |

The regression on the real right arm with this configuration is the next step at the platform.

## Same start-up in simulation, 25/11/2026 (development laptop, Gazebo)

After the alignment described in the Yaskawa README ("Same start-up"), right arm unless stated:

| Check | Result |
|---|---|
| `arm_controller` at launch | Inactive; both arms held still |
| `move_joint.py` before any motion mode | Refused (controller not running) |
| `start_point_queue_mode` | READY (1) |
| `start_traj_mode` while point-queue mode is active | Refused, code 114 ("Another trajectory mode is already active") |
| Activation of `arm_controller`, then `move_joint.py` (joint 6, 0.1 rad and back) | Passed |
| `stop_traj_mode` with `arm_controller` active | Refused ("increment queue not empty"), as on the real arm |
| `stop_traj_mode` after deactivating `arm_controller` | Accepted |
| `execute_trajectory.py` of `mfja_ros2_yaskawa` (trajectory mode), both arms | Passed; `arm_controller` inactive again afterwards |

## Next

1. Real right arm: regression with the shared configuration, a real emergency stop followed by the recovery
   sequence, an emergency stop during motion, a motion of several joints at once, an endurance run.
2. The same tests on the left arm.
3. Watch the call failure rate during motion; read the cause of the point refused by MotoROS2.
4. Simulation: the gripper in Gazebo. The start-up is now the same as on the real arms (Yaskawa README,
   "Same start-up"); the real procedure is unchanged.
5. Gripper: effect of the "init" output (OT#100).
