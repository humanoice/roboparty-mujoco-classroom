#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /opt/roboparty_deploy/install/setup.bash
source /opt/ros2_ws/install/setup.bash
exec "$@"
