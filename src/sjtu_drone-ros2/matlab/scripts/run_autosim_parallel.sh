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
AUTOSIM_ENABLE_PROGRESS_PLOT="${AUTOSIM_ENABLE_PROGRESS_PLOT:-false}"
AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ="${AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ:-false}"

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

if [[ "$WORKERS_ARG" == "auto" ]]; then
  WORKERS="$auto_workers"
else
  WORKERS="$WORKERS_ARG"
fi

if ! [[ "$WORKERS" =~ ^[0-9]+$ ]] || (( WORKERS < 1 )); then
  echo "[AUTOSIM] Invalid worker count: $WORKERS"
  exit 1
fi

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
domain_base=$DOMAIN_BASE
gazebo_port_base=$GAZEBO_PORT_BASE
EOF

printf "pid\tworker_id\tdomain_id\tgazebo_port\tlog_file\n" > "$PID_TABLE"

echo "[AUTOSIM] Session root: $SESSION_ROOT"
echo "[AUTOSIM] Worker auto-tune: cpu_limit=$cpu_limit mem_limit=$mem_limit -> auto=$auto_workers"
echo "[AUTOSIM] GPU mode: enable=$AUTOSIM_ENABLE_GPU gpu_count=$gpu_count"
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
    export GAZEBO_MASTER_URI="http://127.0.0.1:$gazebo_port"
    export GAZEBO_IP="127.0.0.1"
    export AUTOSIM_OUTPUT_ROOT="$OUTPUT_ROOT"
    export AUTOSIM_CLEANUP_SCOPE="instance"
    export AUTOSIM_ENABLE_GPU="$AUTOSIM_ENABLE_GPU"
    export AUTOSIM_ENABLE_PROGRESS_PLOT="$AUTOSIM_ENABLE_PROGRESS_PLOT"
    export AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ="$AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ"

    exec "$MATLAB_CMD" -batch "$run_cmd"
  ) >"$log_file" 2>&1 &

  pid="$!"
  printf "%s\t%s\t%s\t%s\t%s\n" "$pid" "$i" "$domain_id" "$gazebo_port" "$log_file" >> "$PID_TABLE"
  echo "[AUTOSIM] Worker $i started pid=$pid domain=$domain_id gazebo_port=$gazebo_port gpu=${gpu_device:-none}"
  sleep 1

done

echo "[AUTOSIM] PID table: $PID_TABLE"
echo "[AUTOSIM] Stop command: $SCRIPT_DIR/stop_autosim_parallel.sh $SESSION_ROOT"
