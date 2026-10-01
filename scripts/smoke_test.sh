#!/bin/bash
# Smoke test: start the robot, walk forward, and check that it moved and did not fall.
# Run inside the container:  /opt/ros2_ws/scripts/smoke_test.sh
# Or from the host:          docker compose exec sim /opt/ros2_ws/scripts/smoke_test.sh
set -eo pipefail
# The ROS setup scripts read unset variables, so turn on "set -u" after them.
source /opt/ros/humble/setup.bash
source /opt/roboparty_deploy/install/setup.bash
source /opt/ros2_ws/install/setup.bash
set -u

fail() { echo "FAIL: $*"; exit 1; }

echo "== Wait for inference_node and the simulator"
for _ in $(seq 1 60); do
    if ros2 service list 2>/dev/null | grep -q '^/start_inference$' \
        && ros2 service list 2>/dev/null | grep -q '^/sim/reset$'; then
        break
    fi
    sleep 1
done
ros2 service list | grep -q '^/start_inference$' || fail "inference_node is not running"

echo "== Unit tests"
python3 -m pytest -q /opt/ros2_ws/src/roboparty_sim/test

echo "== Real-time factor"
rtf=$(timeout 10 ros2 topic echo --once /sim/real_time_factor | awk '/data:/ {print $2}')
echo "real-time factor: $rtf"
python3 -c "import sys; sys.exit(0 if float('$rtf') > 0.9 else 1)" \
    || fail "simulator is slower than real time ($rtf). The robot can fall on this computer."

echo "== Start and walk"
party reset
start_x=$(party status | sed -n 's/.*x=\([-0-9.]*\) m.*/\1/p')
party walk 0.4 5
sleep 1
status=$(party status)
echo "$status"
end_x=$(echo "$status" | sed -n 's/.*x=\([-0-9.]*\) m.*/\1/p')
echo "$status" | grep -q 'The robot is up' || fail "the robot fell"
python3 -c "import sys; sys.exit(0 if $end_x - $start_x > 1.0 else 1)" \
    || fail "the robot did not walk far enough (from x=$start_x to x=$end_x)"

echo "PASS: the robot walked $(python3 -c "print(round($end_x - $start_x, 2))") m and is still up"
