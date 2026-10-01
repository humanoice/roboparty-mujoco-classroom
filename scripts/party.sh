#!/bin/bash
# Wrapper so students can type "party ..." in any shell.
source /opt/roboparty_deploy/install/setup.bash
source /opt/ros2_ws/install/setup.bash
exec /opt/ros2_ws/install/roboparty_sim/lib/roboparty_sim/party "$@"
