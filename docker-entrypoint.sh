#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/colcon_ws/src/pepper_virtual/pepper_gazebo_plugin/models

wait_for_port() {
    local port=$1; local label=$2; local timeout=30; local elapsed=0
    until nc -z localhost "$port" 2>/dev/null; do
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "[ERROR] Timed out waiting for $label on port $port"; exit 1
        fi
        sleep 1; elapsed=$((elapsed + 1))
    done
    echo "[OK] $label ready on port $port"
}

MODE=${1:-gui}
case "$MODE" in
  gui)
    Xvnc :1 -geometry 1280x1024 -depth 24 -localhost -SecurityTypes None &
    export DISPLAY=:1
    sleep 1
    openbox &
    wait_for_port 5901 "Xvnc"
    websockify --web /usr/share/novnc 6080 localhost:5901 &
    wait_for_port 6080 "noVNC"
    ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_Y20_CPU_no_arms.launch.py gui:=true
    ;;
  headless)
    Xvfb :99 -screen 0 1280x1024x24 &
    export DISPLAY=:99
    sleep 2
    ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_Y20_CPU_no_arms.launch.py gui:=false
    ;;
  bash|*)
    exec "${@:-bash}"
    ;;
esac
