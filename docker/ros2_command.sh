#!/bin/bash

# This script is used to start the perception container.
# It's purpose is to ensure the container shuts down cleanly in case of a SIGTERM or if a process ends unexpectedly.

# The main ros2 launch command is passed as an argument to this script.
# Since the command can be complex, we execute it as it is passed in.
ARGUMENT="$@"
for arg in "$@"; do
    echo "Argument: $arg"
done

IN_FOREGROUND="${IN_FOREGROUND:-0}"  #
source /workspace/install/setup.bash


# By enabling job control with set -m, we are able to use the shell's job management features within this script,
# including sending SIGINT to background processes.
set -m

if [ $IN_FOREGROUND -eq 1 ]; then
  # Run the command in the foreground
  /bin/bash -c "$ARGUMENT"
  exit $?
fi

# This is a simple check to prevent the container from running the wrong command.
if [[ $ARGUMENT != *"ros2 "* ]]; then
  echo "Invalid arguments. Please provide 'ros2 launch ...'"
  exit 1
fi

# Important, start the ros2 daemon. See also https://answers.ros.org/question/327348/what-is-ros2-daemon/
# Node discovery is not working otherwise when using cyclonedds shared memory
ros2 daemon start
# Run the command in the background and collect it's Process ID

/bin/bash -c "$ARGUMENT" &
ROS_PID=$!
echo "ROS_PID: $ROS_PID"

function cleanup() {
  echo "[INFO] [container] Container recevied SIGTERM. Shutting down..."
  # Ros2 requires the SIGINT signal!
  # @see  https://github.com/ros2/launch/issues/666
  kill -s SIGINT $ROS_PID
  wait $ROS_PID

  echo "[INFO] [container] Container exited cleanly."
  exit 0
}

# Trap signals so we can handle them
trap cleanup SIGTERM SIGINT

# Loop to monitor the ror2 launch process. In case it ends unexpectedly, we can abort the container.
while true; do
  # Check if the process is still running using the PID
  # Because of the trap above, this will only happen if the process ends unexpectedly.
  if ! kill -0 $ROS_PID 2>/dev/null; then
    echo "[FATAL] [container] Container Ros2 launch process ended unenxpectedly! Aborting..."
    kill -s SIGTERM $IOX_PID
    exit 1
  fi
  sleep 1
done
