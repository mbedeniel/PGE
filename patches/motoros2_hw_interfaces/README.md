# Local patches for motoros2_hw_interfaces

[b-robotized/motoros2_hw_interfaces](https://github.com/b-robotized/motoros2_hw_interfaces) runs the real
Yaskawa arms on ros2_control through MotoROS2. It is pinned in `pge.repos` at `dd1827b` (December 2024, the
latest commit). Fixes we need are kept here, applied on a local branch and never pushed.

| Patch | Base | Files |
|---|---|---|
| `0001-declare-realtime-tools-and-trajectory-msgs-dependencies.patch` | `dd1827b` | `CMakeLists.txt`, `package.xml` |

## 0001: declare the realtime_tools and trajectory_msgs dependencies

The MotoROS2 hardware interface includes `realtime_tools` and `trajectory_msgs` headers, but the package does
not declare them. The build then depends on another package exporting them. The patch adds both to
`CMakeLists.txt` and `package.xml`; the code is unchanged.

## Apply and build

```bash
cd ~/mfja_ws/src/motoros2_hw_interfaces
git switch -c pge/local-fixes
git apply ~/mfja_ws/src/PGE/patches/motoros2_hw_interfaces/0001-declare-realtime-tools-and-trajectory-msgs-dependencies.patch
git commit -am "build: declare realtime_tools and trajectory_msgs dependencies"
cd ~/mfja_ws
colcon build --packages-select motoros2_hw_interfaces --cmake-args -DBUILD_TESTING=OFF
```

`-DBUILD_TESTING=OFF` skips the package's unit tests, which need `ros2_control_test_assets`.
On the PC that drives the robots, the patch is applied without a commit (`git apply` only): `vcs status`
then shows the two modified files.

## Known limits of this version

- Do not set the hardware parameter `throttle_duration_ms`: it is only read when absent, and a value
  given leaves the log throttle uninitialised.
- `on_deactivate` does nothing: after the arm controller is deactivated, the last command keeps being
  sent and the arm holds that position until the launch is stopped.
- Before the first `joint_states` message from MotoROS2, the plugin reports all joints at 0. Activate
  `arm_controller` only once the states match the robot (procedure in `yaskawa/mfja_yaskawa/README.md`).
