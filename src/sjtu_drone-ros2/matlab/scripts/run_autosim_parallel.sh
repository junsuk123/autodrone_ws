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
AUTOSIM_MAX_WORKERS="${AUTOSIM_MAX_WORKERS:-}"
AUTOSIM_ENABLE_PROGRESS_PLOT="${AUTOSIM_ENABLE_PROGRESS_PLOT:-false}"
AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ="${AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ:-false}"
AUTOSIM_USE_GUI="${AUTOSIM_USE_GUI:-auto}"
AUTOSIM_USE_RVIZ="${AUTOSIM_USE_RVIZ:-auto}"
AUTOSIM_ROS_LOCALHOST_ONLY="${AUTOSIM_ROS_LOCALHOST_ONLY:-0}"
AUTOSIM_ENABLE_DOMAIN_BRIDGE="${AUTOSIM_ENABLE_DOMAIN_BRIDGE:-false}"
OBSERVE_DOMAIN="${OBSERVE_DOMAIN:-90}"
AUTOSIM_MULTI_DRONE_COUNT="${AUTOSIM_MULTI_DRONE_COUNT:-1}"
AUTOSIM_MULTI_DRONE_SPACING_M="${AUTOSIM_MULTI_DRONE_SPACING_M:-10.0}"
AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX="${AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX:-drone_w}"
AUTOSIM_MULTI_DRONE_SPAWN_TAGS="${AUTOSIM_MULTI_DRONE_SPAWN_TAGS:-true}"
AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST="${AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST:-false}"
AUTOSIM_PRIMARY_DRONE_INDEX="${AUTOSIM_PRIMARY_DRONE_INDEX:-1}"
AUTOSIM_WORKER_LAUNCH_STAGGER_SEC="${AUTOSIM_WORKER_LAUNCH_STAGGER_SEC:-4}"
AUTOSIM_FORCE_SOFTWARE_GL="${AUTOSIM_FORCE_SOFTWARE_GL:-auto}"
AUTOSIM_ALLOW_PARALLEL_RVIZ="${AUTOSIM_ALLOW_PARALLEL_RVIZ:-false}"
AUTOSIM_PARALLEL_RVIZ_MODE="${AUTOSIM_PARALLEL_RVIZ_MODE:-single}"
AUTOSIM_DYNAMIC_WORKER_SCALE="${AUTOSIM_DYNAMIC_WORKER_SCALE:-true}"
AUTOSIM_ALLOW_SCALE_ABOVE_REQUESTED="${AUTOSIM_ALLOW_SCALE_ABOVE_REQUESTED:-false}"
AUTOSIM_SPLIT_SCENARIOS_ACROSS_WORKERS="${AUTOSIM_SPLIT_SCENARIOS_ACROSS_WORKERS:-true}"
AUTOSIM_MEMORY_PROBE_WAIT_SEC="${AUTOSIM_MEMORY_PROBE_WAIT_SEC:-8}"
AUTOSIM_MEMORY_MONITOR_INTERVAL_SEC="${AUTOSIM_MEMORY_MONITOR_INTERVAL_SEC:-10}"
AUTOSIM_SCALE_STEP="${AUTOSIM_SCALE_STEP:-1}"
AUTOSIM_CPU_TARGET_UTIL_PCT="${AUTOSIM_CPU_TARGET_UTIL_PCT:-75}"
AUTOSIM_CPU_HARD_LIMIT_PCT="${AUTOSIM_CPU_HARD_LIMIT_PCT:-90}"

if ! [[ "$AUTOSIM_WORKER_LAUNCH_STAGGER_SEC" =~ ^[0-9]+$ ]]; then
  AUTOSIM_WORKER_LAUNCH_STAGGER_SEC=4
fi
if ! [[ "$AUTOSIM_MEMORY_MONITOR_INTERVAL_SEC" =~ ^[0-9]+$ ]]; then
  AUTOSIM_MEMORY_MONITOR_INTERVAL_SEC=10
fi
if (( AUTOSIM_MEMORY_MONITOR_INTERVAL_SEC < 1 )); then
  AUTOSIM_MEMORY_MONITOR_INTERVAL_SEC=1
fi
if ! [[ "$AUTOSIM_SCALE_STEP" =~ ^[0-9]+$ ]]; then
  AUTOSIM_SCALE_STEP=1
fi
if (( AUTOSIM_SCALE_STEP < 1 )); then
  AUTOSIM_SCALE_STEP=1
fi
if ! [[ "$AUTOSIM_CPU_TARGET_UTIL_PCT" =~ ^[0-9]+$ ]]; then
  AUTOSIM_CPU_TARGET_UTIL_PCT=75
fi
if ! [[ "$AUTOSIM_CPU_HARD_LIMIT_PCT" =~ ^[0-9]+$ ]]; then
  AUTOSIM_CPU_HARD_LIMIT_PCT=90
fi
if (( AUTOSIM_CPU_TARGET_UTIL_PCT < 1 )); then
  AUTOSIM_CPU_TARGET_UTIL_PCT=1
fi
if (( AUTOSIM_CPU_TARGET_UTIL_PCT > 95 )); then
  AUTOSIM_CPU_TARGET_UTIL_PCT=95
fi
if (( AUTOSIM_CPU_HARD_LIMIT_PCT <= AUTOSIM_CPU_TARGET_UTIL_PCT )); then
  AUTOSIM_CPU_HARD_LIMIT_PCT="$((AUTOSIM_CPU_TARGET_UTIL_PCT + 10))"
fi
if (( AUTOSIM_CPU_HARD_LIMIT_PCT > 99 )); then
  AUTOSIM_CPU_HARD_LIMIT_PCT=99
fi

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

auto_workers_probe="$cpu_limit"
if (( mem_limit < auto_workers_probe )); then
  auto_workers_probe="$mem_limit"
fi

auto_workers="$auto_workers_probe"
if [[ "$AUTOSIM_ENABLE_GPU" == "true" ]] && (( gpu_count > 0 )); then
  if ! [[ "$WORKERS_PER_GPU" =~ ^[0-9]+$ ]] || (( WORKERS_PER_GPU < 1 )); then
    WORKERS_PER_GPU=1
  fi
  gpu_worker_limit="$((gpu_count * WORKERS_PER_GPU))"
  if (( gpu_worker_limit < auto_workers )); then
    auto_workers="$gpu_worker_limit"
  fi
fi
if (( auto_workers_probe < 1 )); then auto_workers_probe=1; fi
if (( auto_workers < 1 )); then auto_workers=1; fi

if [[ -n "$AUTOSIM_MAX_WORKERS" ]] && [[ "$AUTOSIM_MAX_WORKERS" =~ ^[0-9]+$ ]] && (( AUTOSIM_MAX_WORKERS >= 1 )); then
  if (( auto_workers_probe > AUTOSIM_MAX_WORKERS )); then
    auto_workers_probe="$AUTOSIM_MAX_WORKERS"
  fi
  if (( auto_workers > AUTOSIM_MAX_WORKERS )); then
    auto_workers="$AUTOSIM_MAX_WORKERS"
  fi
fi

if [[ "$WORKERS_ARG" == "auto" ]]; then
  if [[ "$AUTOSIM_DYNAMIC_WORKER_SCALE" == "1" || "$AUTOSIM_DYNAMIC_WORKER_SCALE" == "true" || "$AUTOSIM_DYNAMIC_WORKER_SCALE" == "yes" ]]; then
    REQUESTED_WORKERS="$auto_workers_probe"
  else
    REQUESTED_WORKERS="$auto_workers"
  fi
else
  REQUESTED_WORKERS="$WORKERS_ARG"
fi

if ! [[ "$REQUESTED_WORKERS" =~ ^[0-9]+$ ]] || (( REQUESTED_WORKERS < 1 )); then
  echo "[AUTOSIM] Invalid worker count: $REQUESTED_WORKERS"
  exit 1
fi

if [[ -n "$SCENARIO_COUNT" ]] && ! [[ "$SCENARIO_COUNT" =~ ^[0-9]+$ ]]; then
  echo "[AUTOSIM] Invalid SCENARIO_COUNT: $SCENARIO_COUNT"
  exit 1
fi

if [[ -n "$SCENARIO_COUNT" ]]; then
  split_enabled="false"
  case "${AUTOSIM_SPLIT_SCENARIOS_ACROSS_WORKERS,,}" in
    1|true|yes|y|on) split_enabled="true" ;;
  esac

  if [[ "$split_enabled" == "true" ]]; then
    if (( SCENARIO_COUNT < REQUESTED_WORKERS )); then
      echo "[AUTOSIM] Scenario split: total scenarios($SCENARIO_COUNT) < requested workers($REQUESTED_WORKERS)."
      echo "[AUTOSIM] Clamping workers to $SCENARIO_COUNT so each worker runs at least one scenario."
      REQUESTED_WORKERS="$SCENARIO_COUNT"
      if (( REQUESTED_WORKERS < 1 )); then
        REQUESTED_WORKERS=1
      fi
    fi

    dyn_enabled="false"
    case "${AUTOSIM_DYNAMIC_WORKER_SCALE,,}" in
      1|true|yes|y|on) dyn_enabled="true" ;;
    esac
    if [[ "$dyn_enabled" == "true" ]]; then
      echo "[AUTOSIM] Scenario split mode active: forcing AUTOSIM_DYNAMIC_WORKER_SCALE=false (no worker replenishment)."
      AUTOSIM_DYNAMIC_WORKER_SCALE="false"
    fi
  fi
fi

if [[ "$WORKERS_ARG" != "auto" ]] && (( REQUESTED_WORKERS > auto_workers_probe )); then
  echo "[AUTOSIM] Requested workers=$REQUESTED_WORKERS exceeds initial resource-safe estimate(auto_workers_probe=$auto_workers_probe)."
  echo "[AUTOSIM] Dynamic probe will launch one worker first, then scale up safely."
fi

if [[ -n "$AUTOSIM_MAX_WORKERS" ]] && [[ "$AUTOSIM_MAX_WORKERS" =~ ^[0-9]+$ ]] && (( AUTOSIM_MAX_WORKERS >= 1 )) && (( REQUESTED_WORKERS > AUTOSIM_MAX_WORKERS )); then
  echo "[AUTOSIM] Requested workers=$REQUESTED_WORKERS exceeds AUTOSIM_MAX_WORKERS=$AUTOSIM_MAX_WORKERS. Clamping."
  REQUESTED_WORKERS="$AUTOSIM_MAX_WORKERS"
fi

PARALLEL_SINGLE_RVIZ_ACTIVE="false"
if (( REQUESTED_WORKERS > 1 )) && [[ "$AUTOSIM_USE_RVIZ" == "1" || "$AUTOSIM_USE_RVIZ" == "true" || "$AUTOSIM_USE_RVIZ" == "yes" ]]; then
  if [[ "$AUTOSIM_ALLOW_PARALLEL_RVIZ" == "1" || "$AUTOSIM_ALLOW_PARALLEL_RVIZ" == "true" || "$AUTOSIM_ALLOW_PARALLEL_RVIZ" == "yes" ]]; then
    echo "[AUTOSIM] Parallel RViz override enabled (AUTOSIM_ALLOW_PARALLEL_RVIZ=$AUTOSIM_ALLOW_PARALLEL_RVIZ)."
  else
    case "${AUTOSIM_PARALLEL_RVIZ_MODE,,}" in
      single)
        PARALLEL_SINGLE_RVIZ_ACTIVE="true"
        echo "[AUTOSIM] Parallel RViz mode=single: only worker 1 will launch RViz."
        ;;
      off|none|false|0)
        echo "[AUTOSIM] Parallel RViz mode=off: forcing AUTOSIM_USE_RVIZ=false in parallel runs."
        AUTOSIM_USE_RVIZ="false"
        ;;
      *)
        PARALLEL_SINGLE_RVIZ_ACTIVE="true"
        echo "[AUTOSIM] Unknown AUTOSIM_PARALLEL_RVIZ_MODE=$AUTOSIM_PARALLEL_RVIZ_MODE, fallback to single-worker RViz."
        ;;
    esac
  fi
fi

if [[ "$AUTOSIM_FORCE_SOFTWARE_GL" == "auto" ]]; then
  AUTOSIM_FORCE_SOFTWARE_GL="false"
fi

if [[ "$AUTOSIM_USE_GUI" == "auto" ]]; then
  if (( REQUESTED_WORKERS > 1 )); then
    AUTOSIM_USE_GUI="false"
  else
    AUTOSIM_USE_GUI="true"
  fi
fi

if [[ "$AUTOSIM_USE_RVIZ" == "auto" ]]; then
  if (( REQUESTED_WORKERS > 1 )); then
    AUTOSIM_USE_RVIZ="false"
  else
    AUTOSIM_USE_RVIZ="true"
  fi
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

WORKERS="$REQUESTED_WORKERS"
SCENARIO_PER_WORKER_BASE=""
SCENARIO_PER_WORKER_REMAINDER=""
TOTAL_SCENARIOS=""

if [[ -n "$SCENARIO_COUNT" ]]; then
  TOTAL_SCENARIOS="$SCENARIO_COUNT"
  SCENARIO_PER_WORKER_BASE=$((SCENARIO_COUNT / WORKERS))
  SCENARIO_PER_WORKER_REMAINDER=$((SCENARIO_COUNT % WORKERS))
fi

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
workers_requested=$REQUESTED_WORKERS
cpu_total=$cpu_total
cpu_limit=$cpu_limit
cpu_target_util_pct=$AUTOSIM_CPU_TARGET_UTIL_PCT
cpu_hard_limit_pct=$AUTOSIM_CPU_HARD_LIMIT_PCT
mem_avail_gb=$mem_avail_gb
mem_limit=$mem_limit
auto_workers=$auto_workers
max_workers=$AUTOSIM_MAX_WORKERS
domain_base=$DOMAIN_BASE
gazebo_port_base=$GAZEBO_PORT_BASE
EOF

printf "pid\tworker_id\tdomain_id\tgazebo_port\tlog_file\n" > "$PID_TABLE"

echo "[AUTOSIM] Session root: $SESSION_ROOT"
echo "[AUTOSIM] Worker auto-tune: cpu_limit=$cpu_limit mem_limit=$mem_limit -> probe_auto=$auto_workers_probe gpu_auto=$auto_workers"
echo "[AUTOSIM] GPU mode: enable=$AUTOSIM_ENABLE_GPU gpu_count=$gpu_count"
echo "[AUTOSIM] Requested workers: $REQUESTED_WORKERS"
if [[ -n "$TOTAL_SCENARIOS" ]]; then
  echo "[AUTOSIM] Scenario distribution: total=$TOTAL_SCENARIOS workers=$WORKERS base=$SCENARIO_PER_WORKER_BASE remainder=$SCENARIO_PER_WORKER_REMAINDER"
fi
echo "[AUTOSIM] Allow scale above requested: $AUTOSIM_ALLOW_SCALE_ABOVE_REQUESTED"
echo "[AUTOSIM] CPU hybrid policy: target=${AUTOSIM_CPU_TARGET_UTIL_PCT}% hard=${AUTOSIM_CPU_HARD_LIMIT_PCT}%"
echo "[AUTOSIM] Visualization defaults: use_gui=$AUTOSIM_USE_GUI use_rviz=$AUTOSIM_USE_RVIZ"
echo "[AUTOSIM] Worker launch stagger: ${AUTOSIM_WORKER_LAUNCH_STAGGER_SEC}s"
echo "[AUTOSIM] Force software GL: $AUTOSIM_FORCE_SOFTWARE_GL"
echo "[AUTOSIM] CLI inspect tip: export ROS_DOMAIN_ID=$DOMAIN_BASE"
echo "[AUTOSIM] Dynamic worker scaling: $AUTOSIM_DYNAMIC_WORKER_SCALE"

if ! [[ "$AUTOSIM_MEMORY_PROBE_WAIT_SEC" =~ ^[0-9]+$ ]]; then
  AUTOSIM_MEMORY_PROBE_WAIT_SEC=8
fi

launch_worker() {
  local i="$1"
  local total_workers="$2"
  local domain_id="$((DOMAIN_BASE + i - 1))"
  local gazebo_port="$((GAZEBO_PORT_BASE + i - 1))"
  local log_file="$LOG_ROOT/worker_${i}.log"
  local gpu_device=""
  local worker_scenarios=""
  local worker_use_rviz="$AUTOSIM_USE_RVIZ"

  if [[ -n "$TOTAL_SCENARIOS" ]]; then
    worker_scenarios="$SCENARIO_PER_WORKER_BASE"
    if (( i <= SCENARIO_PER_WORKER_REMAINDER )); then
      worker_scenarios="$((worker_scenarios + 1))"
    fi
  fi

  if [[ "$PARALLEL_SINGLE_RVIZ_ACTIVE" == "true" ]] && (( i > 1 )); then
    worker_use_rviz="false"
  fi

  (
    if [[ "$AUTOSIM_ENABLE_GPU" == "true" ]] && (( gpu_count > 0 )); then
      gpu_device="$(( (i - 1) % gpu_count ))"
      export CUDA_VISIBLE_DEVICES="$gpu_device"
      export AUTOSIM_GPU_DEVICE=1
    else
      export AUTOSIM_GPU_DEVICE=""
    fi

    export AUTOSIM_WORKER_ID="$i"
    export AUTOSIM_WORKER_COUNT="$total_workers"
    export AUTOSIM_DOMAIN_ID="$domain_id"
    export AUTOSIM_GAZEBO_PORT="$gazebo_port"
    export ROS_DOMAIN_ID="$domain_id"
    export ROS_LOCALHOST_ONLY="$AUTOSIM_ROS_LOCALHOST_ONLY"
    export GAZEBO_MASTER_URI="http://127.0.0.1:$gazebo_port"
    export GAZEBO_IP="127.0.0.1"
    export AUTOSIM_OUTPUT_ROOT="$OUTPUT_ROOT"
    export AUTOSIM_CLEANUP_SCOPE="instance"
    export AUTOSIM_ENABLE_GPU="$AUTOSIM_ENABLE_GPU"
    export AUTOSIM_ENABLE_PROGRESS_PLOT="$AUTOSIM_ENABLE_PROGRESS_PLOT"
    export AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ="$AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ"
    export AUTOSIM_USE_GUI="$AUTOSIM_USE_GUI"
    export AUTOSIM_USE_RVIZ="$worker_use_rviz"
    export AUTOSIM_MULTI_DRONE_COUNT="$AUTOSIM_MULTI_DRONE_COUNT"
    export AUTOSIM_MULTI_DRONE_SPACING_M="$AUTOSIM_MULTI_DRONE_SPACING_M"
    export AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX="$AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX"
    export AUTOSIM_MULTI_DRONE_SPAWN_TAGS="$AUTOSIM_MULTI_DRONE_SPAWN_TAGS"
    export AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST="$AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST"
    export AUTOSIM_PRIMARY_DRONE_INDEX="$AUTOSIM_PRIMARY_DRONE_INDEX"
    if [[ -n "$worker_scenarios" ]]; then
      export AUTOSIM_SCENARIO_COUNT="$worker_scenarios"
    fi

    if [[ "$AUTOSIM_FORCE_SOFTWARE_GL" == "1" || "$AUTOSIM_FORCE_SOFTWARE_GL" == "true" || "$AUTOSIM_FORCE_SOFTWARE_GL" == "yes" ]]; then
      export LIBGL_ALWAYS_SOFTWARE=1
      export QT_QUICK_BACKEND=software
    fi

    exec "$MATLAB_CMD" -batch "$run_cmd"
  ) >"$log_file" 2>&1 &

  local pid="$!"
  printf "%s\t%s\t%s\t%s\t%s\n" "$pid" "$i" "$domain_id" "$gazebo_port" "$log_file" >> "$PID_TABLE"
  LAST_LAUNCHED_PID="$pid"
  LAST_LAUNCHED_WORKER_ID="$i"
  if [[ -n "$worker_scenarios" ]]; then
    echo "[AUTOSIM] Worker $i started pid=$pid domain=$domain_id gazebo_port=$gazebo_port gpu=${gpu_device:-none} rviz=$worker_use_rviz scenarios=$worker_scenarios"
  else
    echo "[AUTOSIM] Worker $i started pid=$pid domain=$domain_id gazebo_port=$gazebo_port gpu=${gpu_device:-none} rviz=$worker_use_rviz"
  fi
}

run_cmd="run('$MATLAB_DIR/AutoSim.m')"
if [[ -z "$TOTAL_SCENARIOS" && -n "$SCENARIO_COUNT" ]]; then
  export AUTOSIM_SCENARIO_COUNT="$SCENARIO_COUNT"
fi

LAST_LAUNCHED_PID=""
LAST_LAUNCHED_WORKER_ID=""

kill_pid_graceful() {
  local pid="$1"
  if [[ -z "$pid" ]]; then
    return
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    return
  fi

  kill "$pid" 2>/dev/null || true
  for _ in {1..10}; do
    if ! kill -0 "$pid" 2>/dev/null; then
      return
    fi
    sleep 0.2
  done
  kill -9 "$pid" 2>/dev/null || true
}

reap_workers() {
  local -a new_pids=()
  local -a new_ids=()
  local idx pid wid
  for idx in "${!ACTIVE_PIDS[@]}"; do
    pid="${ACTIVE_PIDS[$idx]}"
    wid="${ACTIVE_WORKER_IDS[$idx]}"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      new_pids+=("$pid")
      new_ids+=("$wid")
    fi
  done
  ACTIVE_PIDS=("${new_pids[@]}")
  ACTIVE_WORKER_IDS=("${new_ids[@]}")
}

read_cpu_counters() {
  awk '/^cpu / {for(i=2;i<=11;i++) s+=$i; idle=$5+$6; printf "%s %s\n", s, idle; exit}' /proc/stat 2>/dev/null
}

update_cpu_util_pct() {
  local now_total now_idle
  read -r now_total now_idle < <(read_cpu_counters)
  if [[ -z "${now_total:-}" || -z "${now_idle:-}" ]]; then
    CURRENT_CPU_UTIL_PCT=0
    return
  fi

  local delta_total="$((now_total - prev_cpu_total))"
  local delta_idle="$((now_idle - prev_cpu_idle))"
  prev_cpu_total="$now_total"
  prev_cpu_idle="$now_idle"

  if (( delta_total <= 0 )); then
    CURRENT_CPU_UTIL_PCT=0
    return
  fi

  local raw
  raw="$(awk -v dt="$delta_total" -v di="$delta_idle" 'BEGIN{u=((dt-di)*100.0)/dt; if(u<0)u=0; if(u>100)u=100; printf "%.2f", u}')"
  smoothed_cpu_util="$(awk -v s="$smoothed_cpu_util" -v r="$raw" 'BEGIN{printf "%.2f", (3.0*s + r)/4.0}')"
  CURRENT_CPU_UTIL_PCT="$smoothed_cpu_util"
}

compute_target_workers() {
  local active_count="$1"
  local mem_now_kb="$2"
  local est_kb="$fallback_worker_kb"
  local used_kb=0
  local observed_kb=0
  local target="$active_count"
  local mem_target="$active_count"
  local cpu_target="$active_count"

  if (( active_count > 0 )); then
    used_kb="$((mem_start_kb - mem_now_kb))"
    if (( used_kb < 0 )); then
      used_kb=0
    fi
    observed_kb="$((used_kb / active_count))"
    if (( observed_kb < fallback_worker_kb / 2 )); then
      observed_kb="$fallback_worker_kb"
    fi
    if (( dynamic_worker_kb < 1 )); then
      dynamic_worker_kb="$observed_kb"
    else
      dynamic_worker_kb="$(((3 * dynamic_worker_kb + observed_kb) / 4))"
    fi
    if (( dynamic_worker_kb < 1 )); then
      dynamic_worker_kb=1
    fi
    est_kb="$dynamic_worker_kb"
  else
    dynamic_worker_kb="$fallback_worker_kb"
    est_kb="$dynamic_worker_kb"
  fi

  if (( mem_now_kb <= reserve_kb )); then
    local deficit_kb="$((reserve_kb - mem_now_kb))"
    local need_drop="$(((deficit_kb + est_kb - 1) / est_kb))"
    mem_target="$((active_count - need_drop))"
  else
    local addable="$(((mem_now_kb - reserve_kb) / est_kb))"
    mem_target="$((active_count + addable))"
  fi

  local cpu_util_int
  cpu_util_int="$(awk -v u="$CURRENT_CPU_UTIL_PCT" 'BEGIN{printf "%d", int(u+0.5)}')"
  if (( cpu_util_int >= AUTOSIM_CPU_HARD_LIMIT_PCT )); then
    local over="$((cpu_util_int - AUTOSIM_CPU_HARD_LIMIT_PCT))"
    local need_drop="$(((over + 9) / 10))"
    cpu_target="$((active_count - need_drop))"
  elif (( cpu_util_int >= AUTOSIM_CPU_TARGET_UTIL_PCT )); then
    cpu_target="$active_count"
  else
    local headroom="$((AUTOSIM_CPU_TARGET_UTIL_PCT - cpu_util_int))"
    local addable="$((headroom / 10))"
    cpu_target="$((active_count + addable))"
  fi

  target="$mem_target"
  if (( cpu_target < target )); then
    target="$cpu_target"
  fi

  if (( target < 1 )); then
    target=1
  fi
  if [[ "$AUTOSIM_ALLOW_SCALE_ABOVE_REQUESTED" != "1" && "$AUTOSIM_ALLOW_SCALE_ABOVE_REQUESTED" != "true" && "$AUTOSIM_ALLOW_SCALE_ABOVE_REQUESTED" != "yes" ]]; then
    if (( target > REQUESTED_WORKERS )); then
      target="$REQUESTED_WORKERS"
    fi
  fi
  if [[ -n "$AUTOSIM_MAX_WORKERS" ]] && [[ "$AUTOSIM_MAX_WORKERS" =~ ^[0-9]+$ ]] && (( AUTOSIM_MAX_WORKERS >= 1 )) && (( target > AUTOSIM_MAX_WORKERS )); then
    target="$AUTOSIM_MAX_WORKERS"
  fi
  printf '%s' "$target"
}

start_worker_with_new_id() {
  local total_hint="$1"
  local worker_id="$NEXT_WORKER_ID"
  NEXT_WORKER_ID="$((NEXT_WORKER_ID + 1))"
  launch_worker "$worker_id" "$total_hint"
  ACTIVE_PIDS+=("$LAST_LAUNCHED_PID")
  ACTIVE_WORKER_IDS+=("$worker_id")
  TOTAL_LAUNCHED="$((TOTAL_LAUNCHED + 1))"
}

stop_latest_worker() {
  local active_count="${#ACTIVE_PIDS[@]}"
  if (( active_count <= 1 )); then
    return
  fi
  local idx="$((active_count - 1))"
  local pid="${ACTIVE_PIDS[$idx]}"
  local wid="${ACTIVE_WORKER_IDS[$idx]}"
  echo "[AUTOSIM] Scaling down: stopping worker_id=$wid pid=$pid"
  kill_pid_graceful "$pid"
  unset 'ACTIVE_PIDS[idx]'
  unset 'ACTIVE_WORKER_IDS[idx]'
  ACTIVE_PIDS=("${ACTIVE_PIDS[@]}")
  ACTIVE_WORKER_IDS=("${ACTIVE_WORKER_IDS[@]}")
}

reserve_kb="$((MEM_RESERVE_GB * 1024 * 1024))"
fallback_worker_kb="$((MEM_PER_WORKER_GB * 1024 * 1024))"
if (( fallback_worker_kb < 1 )); then
  fallback_worker_kb=1
fi

ACTIVE_PIDS=()
ACTIVE_WORKER_IDS=()
NEXT_WORKER_ID=1
TOTAL_LAUNCHED=0
PEAK_WORKERS=0
dynamic_worker_kb="$fallback_worker_kb"
mem_start_kb="$mem_avail_kb"
CURRENT_CPU_UTIL_PCT=0
smoothed_cpu_util=0
read -r prev_cpu_total prev_cpu_idle < <(read_cpu_counters)
if [[ -z "${prev_cpu_total:-}" || -z "${prev_cpu_idle:-}" ]]; then
  prev_cpu_total=0
  prev_cpu_idle=0
fi

if [[ "$AUTOSIM_DYNAMIC_WORKER_SCALE" == "1" || "$AUTOSIM_DYNAMIC_WORKER_SCALE" == "true" || "$AUTOSIM_DYNAMIC_WORKER_SCALE" == "yes" ]]; then
  echo "[AUTOSIM] Dynamic hybrid scaler enabled: interval=${AUTOSIM_MEMORY_MONITOR_INTERVAL_SEC}s reserve=${MEM_RESERVE_GB}GB step=${AUTOSIM_SCALE_STEP} cpu_target=${AUTOSIM_CPU_TARGET_UTIL_PCT}% cpu_hard=${AUTOSIM_CPU_HARD_LIMIT_PCT}%"
  if [[ "$AUTOSIM_ALLOW_SCALE_ABOVE_REQUESTED" != "1" && "$AUTOSIM_ALLOW_SCALE_ABOVE_REQUESTED" != "true" && "$AUTOSIM_ALLOW_SCALE_ABOVE_REQUESTED" != "yes" ]]; then
    echo "[AUTOSIM] Dynamic scaler cap: requested_workers=${REQUESTED_WORKERS} (no growth above requested)"
  fi
  start_worker_with_new_id 1
  PEAK_WORKERS=1

  if (( AUTOSIM_MEMORY_PROBE_WAIT_SEC > 0 )); then
    sleep "$AUTOSIM_MEMORY_PROBE_WAIT_SEC"
  fi

  while true; do
    reap_workers
    active_count="${#ACTIVE_PIDS[@]}"
    if (( active_count == 0 )); then
      break
    fi

    if (( active_count > PEAK_WORKERS )); then
      PEAK_WORKERS="$active_count"
    fi

    update_cpu_util_pct
    mem_now_kb="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
    target_raw="$(compute_target_workers "$active_count" "$mem_now_kb")"
    target="$target_raw"
    if (( target > active_count + AUTOSIM_SCALE_STEP )); then
      target="$((active_count + AUTOSIM_SCALE_STEP))"
    fi
    if (( target < active_count - AUTOSIM_SCALE_STEP )); then
      target="$((active_count - AUTOSIM_SCALE_STEP))"
    fi

    echo "[AUTOSIM] scaler tick: cpu=${CURRENT_CPU_UTIL_PCT}% mem_avail=${mem_now_kb}KB active=${active_count} est_per_worker=${dynamic_worker_kb}KB target=${target_raw} apply=${target} next_worker_id=${NEXT_WORKER_ID}"

    if (( target > active_count )); then
      to_add="$((target - active_count))"
      for ((k=1; k<=to_add; k++)); do
        start_worker_with_new_id "$target"
        if (( AUTOSIM_WORKER_LAUNCH_STAGGER_SEC > 0 )) && (( k < to_add )); then
          sleep "$AUTOSIM_WORKER_LAUNCH_STAGGER_SEC"
        fi
      done
    elif (( target < active_count )); then
      to_remove="$((active_count - target))"
      for ((k=1; k<=to_remove; k++)); do
        stop_latest_worker
      done
    fi

    sleep "$AUTOSIM_MEMORY_MONITOR_INTERVAL_SEC"
  done

  WORKERS=0
else
  WORKERS="$REQUESTED_WORKERS"
  echo "[AUTOSIM] Launching workers: $WORKERS"
  for ((i=1; i<=WORKERS; i++)); do
    launch_worker "$i" "$WORKERS"
    if (( i < WORKERS )) && (( AUTOSIM_WORKER_LAUNCH_STAGGER_SEC > 0 )); then
      sleep "$AUTOSIM_WORKER_LAUNCH_STAGGER_SEC"
    fi
  done
  TOTAL_LAUNCHED="$WORKERS"
  PEAK_WORKERS="$WORKERS"
fi

echo "workers=$WORKERS" >> "$SESSION_ROOT/session_info.txt"
echo "workers_launched_total=$TOTAL_LAUNCHED" >> "$SESSION_ROOT/session_info.txt"
echo "workers_peak=$PEAK_WORKERS" >> "$SESSION_ROOT/session_info.txt"
if [[ -n "$TOTAL_SCENARIOS" ]]; then
  echo "scenario_total=$TOTAL_SCENARIOS" >> "$SESSION_ROOT/session_info.txt"
  echo "scenario_per_worker_base=$SCENARIO_PER_WORKER_BASE" >> "$SESSION_ROOT/session_info.txt"
  echo "scenario_per_worker_remainder=$SCENARIO_PER_WORKER_REMAINDER" >> "$SESSION_ROOT/session_info.txt"
fi
echo "worker_id_policy=monotonic_no_reuse" >> "$SESSION_ROOT/session_info.txt"
if (( TOTAL_LAUNCHED > 0 )); then
  echo "[AUTOSIM] Worker ROS domains (monotonic IDs): $DOMAIN_BASE..$((DOMAIN_BASE + TOTAL_LAUNCHED - 1))"
fi

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
