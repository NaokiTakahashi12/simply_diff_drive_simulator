#!/bin/bash
set -e
source "/opt/ros/foxy/setup.bash"
source "/ign_ws/install/setup.bash"
source "/preinstall/install/setup.bash"
exec "$@"
