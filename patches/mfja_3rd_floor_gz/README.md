# Local patches for mfja_3rd_floor_gz

We do not change the `INTERNSHIP-ALI-2026` branch of the twin. Fixes we need on our machines are kept
here as patches, applied on a local branch, and never pushed to the twin's repository. They are proposed
to the twin's maintainers separately.

| Patch | Base | Files |
|---|---|---|
| `0001-start-gazebo-gui-after-spawn.patch` | `INTERNSHIP-ALI-2026` @ `2321def` | `mfja_robot_control_config/launch/multi_robot_sim.launch.py`, `mfja_robot_control_config/launch/isolated_industrial_robot.launch.py` |

## 0001: start the Gazebo GUI after the robots are spawned

**Problem.** The Gazebo GUI ignores world updates until it has processed its first snapshot of the
world. A robot spawned while the GUI is still starting is never drawn, although it exists in the
simulation (it moves, its topics work). On a slow machine some robots of the twin are missing from the
window, and which ones changes from one launch to the next.

**Fix.** The launch files start the GUI only after the `ros_gz_sim create` processes of the robots have
exited, so every robot is in the first snapshot:

- `multi_robot_sim.launch.py` (used by `full_floor` and `room_315_only`): waits for every robot spawn,
  then 3 s more so the Room 315 rail nodes, started 4 s after launch, can add their device markers.
  With `robots:=none` there is nothing to wait for and the GUI opens at once.
- `isolated_industrial_robot.launch.py` (used by `single_industrial_robot`): waits for the table and
  the robot spawns.
- If a spawn never ends (for example the server cannot be reached), the GUI opens anyway after 90 s
  (60 s for the isolated robot), so the problem stays visible.

The simulation server, the controllers and the ROS interfaces are unchanged; only the moment the GUI
window opens moves.

## Apply

```bash
cd ~/mfja_ws/src/mfja_3rd_floor_gz
git fetch origin
git switch INTERNSHIP-ALI-2026
git pull --ff-only
git switch -c pge/local-fixes
git apply --check ~/mfja_ws/src/PGE/patches/mfja_3rd_floor_gz/0001-start-gazebo-gui-after-spawn.patch
git apply ~/mfja_ws/src/PGE/patches/mfja_3rd_floor_gz/0001-start-gazebo-gui-after-spawn.patch
git commit -am "fix(launch): start the Gazebo GUI after the robots are spawned"
```

The `pge/local-fixes` branch stays on the local machine: do not push it.

## Follow the twin's updates

```bash
cd ~/mfja_ws/src/mfja_3rd_floor_gz
git fetch origin
git rebase origin/INTERNSHIP-ALI-2026 pge/local-fixes
```

If the rebase stops on a conflict, the upstream launch files have changed: run `git rebase --abort`,
then rebuild the patch on the new base and update the table above.
