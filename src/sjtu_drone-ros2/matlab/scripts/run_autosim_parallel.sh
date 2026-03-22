#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MATLAB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$(cd "$MATLAB_DIR/../.." && pwd)"

MATLAB_CMD="${MATLAB_CMD:-matlab}"
WORKERS_ARG="${1:-auto}"
SCENARIO_COUNT="${SCENARIO_COUNT:-}"
DOMAIN_BASE="${DOMAIN_BASE:-60}"
GAZEBO_PORT_BASE="${GAZEBO_PORT_BASE:-13045}"
CPU_RESERVE="${CPU_RESERVE:-2}"
MEM_RESERVE_GB="${MEM_RESERVE_GB:-4}"
MEM_PER_WORKER_GB="${MEM_PER_WORKER_GB:-6}"
AUTOSIM_ENABLE_GPU="${AUTOSIM_ENABLE_GPU:-auto}"
WORKERS_PER_GPU="${WORKERS_PER_GPU:-1}"
AUTOSIM_MAX_WORKERS="${AUTOSIM_MAX_WORKERS:-3}"
AUTOSIM_ENABLE_PROGRESS_PLOT="${AUTOSIM_ENABLE_PROGRESS_PLOT:-false}"
AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ="${AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ:-false}"
AUTOSIM_ROS_LOCALHOST_ONLY="${AUTOSIM_ROS_LOCALHOST_ONLY:-0}"
AUTOSIM_ENABLE_DOMAIN_BRIDGE="${AUTOSIM_ENABLE_DOMAIN_BRIDGE:-false}"
OBSERVE_DOMAIN="${OBSERVE_DOMAIN:-90}"
AUTOSIM_MULTI_DRONE_COUNT="${AUTOSIM_MULTI_DRONE_COUNT:-4}"
AUTOSIM_MULTI_DRONE_SPACING_M="${AUTOSIM_MULTI_DRONE_SPACING_M:-3.0}"
AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX="${AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX:-drone_w}"
AUTOSIM_MULTI_DRONE_SPAWN_TAGS="${AUTOSIM_MULTI_DRONE_SPAWN_TAGS:-true}"
AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST="${AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST:-false}"
AUTOSIM_PRIMARY_DRONE_INDEX="${AUTOSIM_PRIMARY_DRONE_INDEX:-1}"

gpu_count=0
if command -v nvidia-smi >/dev/null 2>&1; then
  gpu_count="$(nvidia-smi --query-gpu=index --format=csv,noheader 2>/dev/null | wc -l | tr -d '[:space:]')"
  if ! [[ "$gpu_count" =~ ^[0-9]+$ ]]; then
    gpu_count=0
  fi
fi

if [[ "$AUTOSIM_ENABLE_GPU" == "auto" ]]; then
  if (( gpu_count > 0 )); then
    AUTOSIM_ENABLE_GPU="true"
  else
    AUTOSIM_ENABLE_GPU="false"
  fi
fi

if ! command -v "$MATLAB_CMD" >/dev/null 2>&1; then
  echo "[AUTOSIM] MATLAB command not found: $MATLAB_CMD"
  echo "[AUTOSIM] Set MATLAB_CMD=<path-to-matlab> and retry."
  exit 1
fi

cpu_total="$(nproc --all 2>/dev/null || echo 1)"
mem_avail_kb="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
mem_avail_gb="$((mem_avail_kb / 1024 / 1024))"

cpu_limit="$((cpu_total - CPU_RESERVE))"
if (( cpu_limit < 1 )); then cpu_limit=1; fi

if (( MEM_PER_WORKER_GB <= 0 )); then
  MEM_PER_WORKER_GB=1
fi
mem_budget_gb="$((mem_avail_gb - MEM_RESERVE_GB))"
if (( mem_budget_gb < MEM_PER_WORKER_GB )); then
  mem_limit=1
else
  mem_limit="$((mem_budget_gb / MEM_PER_WORKER_GB))"
  if (( mem_limit < 1 )); then mem_limit=1; fi
fi

auto_workers="$cpu_limit"
if (( mem_limit < auto_workers )); then
  auto_workers="$mem_limit"
fi
if [[ "$AUTOSIM_ENABLE_GPU" == "true" ]] && (( gpu_count > 0 )); then
  if ! [[ "$WORKERS_PER_GPU" =~ ^[0-9]+$ ]] || (( WORKERS_PER_GPU < 1 )); then
    WORKERS_PER_GPU=1
  fi
  gpu_worker_limit="$((gpu_count * WORKERS_PER_GPU))"
  if (( gpu_worker_limit < auto_workers )); then
    auto_workers="$gpu_worker_limit"
  fi
fi
if (( auto_workers < 1 )); then auto_workers=1; fi

if [[ "$AUTOSIM_MAX_WORKERS" =~ ^[0-9]+$ ]] && (( AUTOSIM_MAX_WORKERS >= 1 )); then
  if (( auto_workers > AUTOSIM_MAX_WORKERS )); then
    auto_workers="$AUTOSIM_MAX_WORKERS"
  fi
fi

if [[ "$WORKERS_ARG" == "auto" ]]; then
  WORKERS="$auto_workers"
else
  WORKERS="$WORKERS_ARG"
fi

if ! [[ "$WORKERS" =~ ^[0-9]+$ ]] || (( WORKERS < 1 )); then
  echo "[AUTOSIM] Invalid worker count: $WORKERS"
  exit 1
fi

if [[ "$AUTOSIM_MAX_WORKERS" =~ ^[0-9]+$ ]] && (( AUTOSIM_MAX_WORKERS >= 1 )) && (( WORKERS > AUTOSIM_MAX_WORKERS )); then
  echo "[AUTOSIM] Requested workers=$WORKERS exceeds AUTOSIM_MAX_WORKERS=$AUTOSIM_MAX_WORKERS. Clamping."
  WORKERS="$AUTOSIM_MAX_WORKERS"
fi

collect_matches() {
  local pattern="$1"
  pgrep -af "$pattern" 2>/dev/null || true
}

kill_pattern_now() {
  local pattern="$1"
  local pids=""
  pids="$(pgrep -f "$pattern" || true)"
  if [[ -z "$pids" ]]; then
    return 0
  fi

  while read -r pid; do
    [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  done <<< "$pids"

  sleep 0.5
  pids="$(pgrep -f "$pattern" || true)"
  if [[ -n "$pids" ]]; then
    while read -r pid; do
      [[ -n "$pid" ]] && kill -9 "$pid" 2>/dev/null || true
    done <<< "$pids"
  fi
}

is_port_listening() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn "( sport = :$port )" 2>/dev/null | tail -n +2 | grep -q .
    return $?
  fi

  if command -v lsof >/dev/null 2>&1; then
    lsof -iTCP:"$port" -sTCP:LISTEN >/dev/null 2>&1
    return $?
  fi

  return 1
}

preflight_cleanup_and_verify() {
  local strict="${AUTOSIM_STRICT_PRESTART_CLEANUP:-1}"
  local blockers=""
  local port_conflicts=""

  echo "[AUTOSIM] Preflight: stopping previous AutoSim sessions..."
  bash "$SCRIPT_DIR/stop_autosim_parallel.sh" >/dev/null 2>&1 || true

  for _ in {1..3}; do
    kill_pattern_now "(^|/)gzserver([[:space:]]|$)"
    kill_pattern_now "(^|/)gzclient([[:space:]]|$)"
    kill_pattern_now "(^|/)gz([[:space:]]+sim|$)"
    kill_pattern_now "ign[[:space:]]+gazebo"
    kill_pattern_now "(^|/)rviz2([[:space:]]|$)"
    kill_pattern_now "(^|/)domain_bridge([[:space:]]|$)"
    kill_pattern_now "[a]priltag_detector_node"
    kill_pattern_now "[a]priltag_state_bridge"
    kill_pattern_now "[s]pawn_drone"
    kill_pattern_now "[s]pawn_apriltag"
    kill_pattern_now "[r]obot_state_publisher"
    kill_pattern_now "[j]oint_state_publisher"
    kill_pattern_now "[s]tatic_transform_publisher"
    kill_pattern_now "[r]os2 launch sjtu_drone_bringup"

    blockers=""
    for pattern in \
      "(^|/)gzserver([[:space:]]|$)" \
      "(^|/)gzclient([[:space:]]|$)" \
      "(^|/)gz([[:space:]]+sim|$)" \
      "ign[[:space:]]+gazebo" \
      "(^|/)rviz2([[:space:]]|$)" \
      "(^|/)domain_bridge([[:space:]]|$)" \
      "[a]priltag_detector_node" \
      "[a]priltag_state_bridge" \
      "[s]pawn_drone" \
      "[s]pawn_apriltag" \
      "[r]obot_state_publisher" \
      "[j]oint_state_publisher" \
      "[s]tatic_transform_publisher" \
      "[r]os2 launch sjtu_drone_bringup"; do
      matches="$(collect_matches "$pattern")"
      if [[ -n "$matches" ]]; then
        blockers+="$matches"$'\n'
      fi
    done

    port_conflicts=""
    if is_port_listening 11345; then
      port_conflicts+="11345 "
    fi
    for ((p=GAZEBO_PORT_BASE; p<GAZEBO_PORT_BASE+WORKERS; p++)); do
      if is_port_listening "$p"; then
        port_conflicts+="$p "
      fi
    done

    if [[ -z "$blockers" && -z "$port_conflicts" ]]; then
      echo "[AUTOSIM] Preflight: clean startup state confirmed."
      return 0
    fi

    sleep 1
  done

  echo "[AUTOSIM] Preflight failed: lingering Gazebo/RViz/bridge state detected."
  if [[ -n "$blockers" ]]; then
    echo "[AUTOSIM] Residual processes:"
    printf '%s' "$blockers"
  fi
  if [[ -n "$port_conflicts" ]]; then
    echo "[AUTOSIM] Occupied Gazebo-related ports: $port_conflicts"
  fi

  if [[ "$strict" == "1" || "$strict" == "true" || "$strict" == "yes" ]]; then
    echo "[AUTOSIM] Aborting launch. Run stop script manually and retry after cleanup."
    exit 1
  fi

  echo "[AUTOSIM] AUTOSIM_STRICT_PRESTART_CLEANUP=$strict, continuing despite warnings."
}

preflight_cleanup_and_verify

timestamp="$(date +%Y%m%d_%H%M%S)"
SESSION_ROOT="$MATLAB_DIR/parallel_runs/$timestamp"
OUTPUT_ROOT="$SESSION_ROOT/output"
LOG_ROOT="$SESSION_ROOT/logs"
PID_TABLE="$SESSION_ROOT/workers.tsv"
mkdir -p "$OUTPUT_ROOT" "$LOG_ROOT"

cat > "$SESSION_ROOT/session_info.txt" <<EOF
workspace=$WS_DIR
matlab_dir=$MATLAB_DIR
timestamp=$timestamp
workers=$WORKERS
cpu_total=$cpu_total
cpu_limit=$cpu_limit
mem_avail_gb=$mem_avail_gb
mem_limit=$mem_limit
auto_workers=$auto_workers
max_workers=$AUTOSIM_MAX_WORKERS
domain_base=$DOMAIN_BASE
gazebo_port_base=$GAZEBO_PORT_BASE
EOF

printf "pid\tworker_id\tdomain_id\tgazebo_port\tlog_file\n" > "$PID_TABLE"

echo "[AUTOSIM] Session root: $SESSION_ROOT"
echo "[AUTOSIM] Worker auto-tune: cpu_limit=$cpu_limit mem_limit=$mem_limit -> auto=$auto_workers"
echo "[AUTOSIM] GPU mode: enable=$AUTOSIM_ENABLE_GPU gpu_count=$gpu_count"
echo "[AUTOSIM] Worker ROS domains: $DOMAIN_BASE..$((DOMAIN_BASE + WORKERS - 1))"
echo "[AUTOSIM] CLI inspect tip: export ROS_DOMAIN_ID=$DOMAIN_BASE"
echo "[AUTOSIM] Launching workers: $WORKERS"

for ((i=1; i<=WORKERS; i++)); do
  domain_id="$((DOMAIN_BASE + i - 1))"
  gazebo_port="$((GAZEBO_PORT_BASE + i - 1))"
  log_file="$LOG_ROOT/worker_${i}.log"

  run_cmd="run('$MATLAB_DIR/AutoSim.m')"
  if [[ -n "$SCENARIO_COUNT" ]]; then
    export AUTOSIM_SCENARIO_COUNT="$SCENARIO_COUNT"
  fi

  (
    gpu_device=""
    if [[ "$AUTOSIM_ENABLE_GPU" == "true" ]] && (( gpu_count > 0 )); then
      gpu_device="$(( (i - 1) % gpu_count ))"
      export CUDA_VISIBLE_DEVICES="$gpu_device"
      export AUTOSIM_GPU_DEVICE=1
    else
      export AUTOSIM_GPU_DEVICE=""
    fi

    export AUTOSIM_WORKER_ID="$i"
    export AUTOSIM_WORKER_COUNT="$WORKERS"
    export AUTOSIM_DOMAIN_ID="$domain_id"
    export AUTOSIM_GAZEBO_PORT="$gazebo_port"
    # Keep MATLAB ROS nodes and launched ROS/Gazebo nodes in the same worker domain.
    export ROS_DOMAIN_ID="$domain_id"
    export ROS_LOCALHOST_ONLY="$AUTOSIM_ROS_LOCALHOST_ONLY"
    export GAZEBO_MASTER_URI="http://127.0.0.1:$gazebo_port"
    export GAZEBO_IP="127.0.0.1"
    export AUTOSIM_OUTPUT_ROOT="$OUTPUT_ROOT"
    export AUTOSIM_CLEANUP_SCOPE="instance"
    export AUTOSIM_ENABLE_GPU="$AUTOSIM_ENABLE_GPU"
    export AUTOSIM_ENABLE_PROGRESS_PLOT="$AUTOSIM_ENABLE_PROGRESS_PLOT"
    export AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ="$AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ"
    export AUTOSIM_MULTI_DRONE_COUNT="$AUTOSIM_MULTI_DRONE_COUNT"
    export AUTOSIM_MULTI_DRONE_SPACING_M="$AUTOSIM_MULTI_DRONE_SPACING_M"
    export AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX="$AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX"
    export AUTOSIM_MULTI_DRONE_SPAWN_TAGS="$AUTOSIM_MULTI_DRONE_SPAWN_TAGS"
    export AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST="$AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST"
    export AUTOSIM_PRIMARY_DRONE_INDEX="$AUTOSIM_PRIMARY_DRONE_INDEX"

    exec "$MATLAB_CMD" -batch "$run_cmd"
  ) >"$log_file" 2>&1 &

  pid="$!"
  printf "%s\t%s\t%s\t%s\t%s\n" "$pid" "$i" "$domain_id" "$gazebo_port" "$log_file" >> "$PID_TABLE"
  echo "[AUTOSIM] Worker $i started pid=$pid domain=$domain_id gazebo_port=$gazebo_port gpu=${gpu_device:-none}"
  sleep 1

done

echo "[AUTOSIM] PID table: $PID_TABLE"
echo "[AUTOSIM] Stop command: $SCRIPT_DIR/stop_autosim_parallel.sh $SESSION_ROOT"
echo "[AUTOSIM] Unified worker logs: $SCRIPT_DIR/tail_autosim_parallel_logs.sh $SESSION_ROOT"
echo "[AUTOSIM] Worker progress: $SCRIPT_DIR/show_autosim_worker_progress.sh $SESSION_ROOT -w 2"
echo "[AUTOSIM] Domain bridge (optional): OBSERVE_DOMAIN=90 $SCRIPT_DIR/run_autosim_domain_bridge.sh $SESSION_ROOT"

if [[ "$AUTOSIM_ENABLE_DOMAIN_BRIDGE" == "1" || "$AUTOSIM_ENABLE_DOMAIN_BRIDGE" == "true" || "$AUTOSIM_ENABLE_DOMAIN_BRIDGE" == "yes" ]]; then
  mkdir -p "$SESSION_ROOT/bridge"
  bridge_log="$SESSION_ROOT/bridge/autostart.log"
  (
    export OBSERVE_DOMAIN="$OBSERVE_DOMAIN"
    export AUTOSIM_ROS_LOCALHOST_ONLY="$AUTOSIM_ROS_LOCALHOST_ONLY"
    "$SCRIPT_DIR/run_autosim_domain_bridge.sh" "$SESSION_ROOT"
  ) >"$bridge_log" 2>&1 &
  echo "[AUTOSIM] Domain bridge autostart enabled: observe_domain=$OBSERVE_DOMAIN (log: $bridge_log)"
  echo "[AUTOSIM] Observer CLI tip: export ROS_DOMAIN_ID=$OBSERVE_DOMAIN"
fi
