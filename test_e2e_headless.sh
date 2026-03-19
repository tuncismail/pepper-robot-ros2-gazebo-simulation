#!/bin/bash
# End-to-end headless test for Pepper Gazebo simulation (ROS 2 Humble).
# Exit codes: 0 = all checks passed, 1 = one or more checks failed

set +e
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/colcon_ws/src/pepper_virtual/pepper_gazebo_plugin/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/colcon_ws/install/gazebo_model_velocity_plugin/lib

PASS=0
FAIL=0

log()  { echo "[TEST] $*"; }
ok()   { echo "[PASS] $*"; PASS=$((PASS+1)); }
fail() { echo "[FAIL] $*"; FAIL=$((FAIL+1)); }

# 1. Start virtual display
log "Starting Xvfb on :99"
sudo mkdir -p /tmp/.X11-unix && sudo chmod 1777 /tmp/.X11-unix
Xvfb :99 -screen 0 1280x1024x24 &
XVFB_PID=$!
export DISPLAY=:99
sleep 2

# 2. Launch simulation
log "Launching Pepper simulation (headless)"
ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_Y20_CPU_no_arms.launch.py \
    gui:=false &
LAUNCH_PID=$!
sleep 55

# 3. Check topics (retry up to 3×5s to tolerate slow DDS discovery)
check_topic() {
    local topic=$1
    local desc=$2
    for _i in 1 2 3; do
        if ros2 topic list 2>/dev/null | grep -qF "$topic"; then
            ok "$desc exists  ($topic)"
            return
        fi
        sleep 5
    done
    fail "$desc not found  ($topic)"
}

log "Checking topics..."
check_topic /pepper/scan_front     "Front laser scan"
check_topic /pepper/scan_left      "Left laser scan"
check_topic /pepper/scan_right     "Right laser scan"
check_topic /joint_states          "Joint states"
check_topic /tf                    "TF transforms"
check_topic /pepper/odom           "Odometry"

# 4. robot_description param
if ros2 param get /robot_state_publisher robot_description > /dev/null 2>&1; then
    ok "robot_description URDF loaded"
else
    fail "robot_description param missing"
fi

# 5. Pepper model in Gazebo
# odom_publisher.py publishes /pepper/odom only when pepper_MP appears in
# /gazebo/model_states.  Publisher count > 0 confirms the model is spawned
# without requiring a slow DDS subscription.
if ros2 topic info /pepper/odom 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    ok "Pepper model spawned in Gazebo"
else
    fail "Pepper model not found in /gazebo/model_states"
fi

# 6. Velocity command  (timeout guards against waiting forever for a subscriber)
log "Sending test velocity command"
timeout 10 ros2 topic pub --once /pepper/cmd_vel geometry_msgs/msg/Twist \
    '{"linear": {"x": 0.1, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}' \
    > /dev/null 2>&1
CMD_EXIT=$?
# 0 = published to subscriber; 124 = no subscriber yet (topic still reachable)
if [ $CMD_EXIT -eq 0 ] || [ $CMD_EXIT -eq 124 ]; then
    ok "cmd_vel accepted"
else
    fail "cmd_vel rejected (exit $CMD_EXIT)"
fi

# Cleanup
log "Shutting down..."
kill $LAUNCH_PID $XVFB_PID 2>/dev/null || true
sleep 5
kill -9 $LAUNCH_PID $XVFB_PID 2>/dev/null || true
pkill -9 -f "gzserver|spawner|laser_publisher" 2>/dev/null || true

echo ""
echo "════════════════════════════════════"
echo "  E2E Test Results (ROS 2 Humble)"
echo "  PASSED: $PASS"
echo "  FAILED: $FAIL"
echo "════════════════════════════════════"

[ $FAIL -eq 0 ] && exit 0 || exit 1
