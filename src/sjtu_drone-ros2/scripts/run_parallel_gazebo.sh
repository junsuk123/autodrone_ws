#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${WS_ROOT:-/home/j/INCSL/IICC26_ws}"
INSTANCE_COUNT="${1:-4}"
DOMAIN_BASE="${DOMAIN_BASE:-40}"
GAZEBO_PORT_BASE="${GAZEBO_PORT_BASE:-12045}"
USE_RVIZ="${USE_RVIZ:-false}"
USE_APRILTAG="${USE_APRILTAG:-true}"
USE_GUI_FIRST_ONLY="${USE_GUI_FIRST_ONLY:-false}"
STARTUP_STAGGER_SEC="${STARTUP_STAGGER_SEC:-2}"

STATE_DIR="$WS_ROOT/.parallel_sim"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$STATE_DIR/logs/$RUN_ID"
PID_FILE="$STATE_DIR/parallel_instances_$RUN_ID.tsv"
LATEST_LINK="$STATE_DIR/latest.tsv"

if ! [[ "$INSTANCE_COUNT" =~ ^[0-9]+$ ]] || [[ "$INSTANCE_COUNT" -lt 1 ]]; then
  echo "[parallel] INSTANCE_COUNT must be a positive integer" >&2
  exit 1
fi

is_port_busy() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn "( sport = :${port} )" | tail -n +2 | grep -q .
  else
    netstat -ltn 2>/dev/null | awk '{print $4}' | grep -qE ":${port}$"
  fi
}

mkdir -p "$LOG_DIR"
mkdir -p "$STATE_DIR"

echo -e "pid\tdomain_id\tgazebo_port\tlog" > "$PID_FILE"

for ((i = 0; i < INSTANCE_COUNT; i++)); do
  domain_id=$((DOMAIN_BASE + i))
  gazebo_port=$((GAZEBO_PORT_BASE + i))
  if is_port_busy "$gazebo_port"; then
    echo "[parallel] gazebo port already in use: $gazebo_port (set GAZEBO_PORT_BASE to another value)" >&2
    exit 1
  fi
  log_file="$LOG_DIR/instance_${i}.log"
  use_gui="false"
  if [[ "$USE_GUI_FIRST_ONLY" == "true" && "$i" -eq 0 ]]; then
    use_gui="true"
  fi

  cmd="source /opt/ros/humble/setup.bash && \
source '$WS_ROOT/install/setup.bash' && \
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py \
use_gui:=$use_gui use_rviz:=$USE_RVIZ use_teleop:=false use_apriltag:=$USE_APRILTAG"

  (
    export ROS_DOMAIN_ID="$domain_id"
    export GAZEBO_MASTER_URI="http://127.0.0.1:${gazebo_port}"
    nohup bash -lc "$cmd" > "$log_file" 2>&1 &
    echo $! > "$LOG_DIR/instance_${i}.pid"
  )

  pid="$(cat "$LOG_DIR/instance_${i}.pid")"
  echo -e "${pid}\t${domain_id}\t${gazebo_port}\t${log_file}" >> "$PID_FILE"
  printf '[parallel] started instance=%d pid=%s domain=%d gazebo_port=%d\n' "$i" "$pid" "$domain_id" "$gazebo_port"

  if [[ "$i" -lt $((INSTANCE_COUNT - 1)) ]]; then
    sleep "$STARTUP_STAGGER_SEC"
  fi
done

ln -sfn "$PID_FILE" "$LATEST_LINK"
printf '[parallel] pid table: %s\n' "$PID_FILE"
printf '[parallel] latest link: %s\n' "$LATEST_LINK"
