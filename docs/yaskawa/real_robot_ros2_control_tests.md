# Tests: real Yaskawa arms on ros2_control (motoros2_hw_interfaces)

Goal: drive the real arms with the same `joint_trajectory_controller` as in simulation, through the hardware
interface [`motoros2_hw_interfaces`](https://github.com/b-robotized/motoros2_hw_interfaces), which uses the
MotoROS2 "point queue" mode (nothing to install on the controllers). Choice agreed with the platform manager
and the client on 20/11/2026; it also lifts the 200-point limit per trajectory.

Full procedure: `yaskawa/mfja_yaskawa/README.md`, section "Real robots on ros2_control".

## Setup on the PC that drives the robots (MFJA, room 315)

- The PC must keep the fixed address that MotoROS2 on both controllers uses for the micro-ROS agent
  (addresses on the platform wiki). Its room-network connection must use manual addressing only.
- Separate workspace `~/pge_ws`, built from `pge_real_robots.repos` in a `bash --norc` shell that loads
  `/opt/ros/jazzy` only: the existing workspace of the PC is neither modified nor used.
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
| e. First motion (joint 6, 0.1 rad and back) | To do |

Points sent to MotoROS2 in 61 s of holding: 3050, of which 87 failed (2.85 %). All failures are replies that
arrived after the 30 ms timeout; MotoROS2 refused no point. Mean round trip of one call: 16.9 ms. 50 Hz is
therefore suitable; 100 Hz would fail most of the time.

Left arm (HC10DT): controller not reachable that day.

## Next

1. Step e, then the other joints with small angles, a trajectory of more than 200 points (`--points 300`) and
   the recovery after an emergency stop.
2. The same tests on the left arm.
3. Watch the call failure rate during motion.
4. Simulation as close as possible to the real arm: same command interfaces (position + velocity) as on the
   real arm, and imitation of the "point queue" mode in `mfja_yaskawa_motoros2_sim`.
5. Gripper: `GPIOSystem` interface of the same package (outputs OT#100 to OT#102).
