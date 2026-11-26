#!/usr/bin/env bash
# Opens a shell that loads ROS 2 Jazzy and this workspace only.
#
# The new shell starts from an empty environment: nothing loaded by ~/.bashrc or by the current
# terminal is kept (other workspaces, ROS_DOMAIN_ID, ...), only the session variables needed by
# terminals and graphical tools. Nothing on the computer is modified. Leave the shell with "exit".
#
# Usage, from any terminal: <workspace>/src/PGE/pge_shell.sh   (for example ~/pge_ws/src/PGE/pge_shell.sh)

ros_setup=/opt/ros/jazzy/setup.bash
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [ ! -f "$ros_setup" ]; then
  echo "$ros_setup not found: ROS 2 Jazzy is not installed on this computer." >&2
  exit 1
fi

rcfile="$(mktemp)"
cat > "$rcfile" <<EOF
source "$ros_setup"
if [ -f "$workspace/install/setup.bash" ]; then
  source "$workspace/install/setup.bash"
  echo "ROS 2 Jazzy + $workspace loaded."
else
  echo "ROS 2 Jazzy loaded; $workspace is not built yet."
fi
cd "$workspace"
PS1='(pge) \w\\$ '
rm -f "$rcfile"
EOF

keep=(HOME USER LOGNAME TERM LANG DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR XAUTHORITY
      DBUS_SESSION_BUS_ADDRESS)
vars=()
for name in "${keep[@]}"; do
  if [ -n "${!name:-}" ]; then
    vars+=("$name=${!name}")
  fi
done

exec env -i "${vars[@]}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --rcfile "$rcfile" -i
