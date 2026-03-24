#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MATLAB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SESSION_ROOT="${1:-}"
PID_TABLE=""
PID_TABLES=()
GAZEBO_PORTS=()

sanitize_arg() {
  local x="$1"
  x="${x//$'\r'/}"
  x="${x//$'\n'/}"
  x="${x#\"}"
  x="${x%\"}"
  printf '%s' "$x"
}

if [[ -n "$SESSION_ROOT" ]]; then
  SESSION_ROOT="$(sanitize_arg "$SESSION_ROOT")"
fi

if [[ -z "$SESSION_ROOT" ]]; then
  while IFS= read -r path; do
    [[ -n "$path" ]] && PID_TABLES+=("$path")
  done < <(find "$MATLAB_DIR/parallel_runs" -maxdepth 2 -type f -name workers.tsv 2>/dev/null | sort)

  if [[ ${#PID_TABLES[@]} -eq 0 ]]; then
    echo "[AUTOSIM] No parallel session found."
    exit 0
  fi
fi

if [[ -f "$SESSION_ROOT" && "$(basename "$SESSION_ROOT")" == "workers.tsv" ]]; then
  PID_TABLE="$SESSION_ROOT"
  SESSION_ROOT="$(dirname "$SESSION_ROOT")"
fi

if [[ -z "$PID_TABLE" ]]; then
  if [[ -n "$SESSION_ROOT" ]]; then
    PID_TABLE="$SESSION_ROOT/workers.tsv"
  fi
fi

if [[ -n "$PID_TABLE" ]]; then
  PID_TABLES+=("$PID_TABLE")
fi

if [[ ${#PID_TABLES[@]} -eq 0 ]]; then
  echo "[AUTOSIM] PID table not found: $PID_TABLE"
  exit 1
fi

kill_tree() {
  local parent="$1"
  local children=""
  children="$(pgrep -P "$parent" || true)"

  if [[ -n "$children" ]]; then
    while read -r child; do
      [[ -n "$child" ]] && kill_tree "$child"
    done <<< "$children"
  fi

  if kill -0 "$parent" 2>/dev/null; then
    kill "$parent" 2>/dev/null || true
    sleep 0.2
    kill -9 "$parent" 2>/dev/null || true
  fi
}

kill_pattern_graceful() {
  local pattern="$1"
  local label="$2"
  local pids=""
  local remain=""

  pids="$(pgrep -f "$pattern" || true)"
  if [[ -z "$pids" ]]; then
    return 0
  fi

  echo "[AUTOSIM] Stopping $label..."
  while read -r pid; do
    [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  done <<< "$pids"

  for _ in {1..10}; do
    sleep 0.2
    remain="$(pgrep -f "$pattern" || true)"
    if [[ -z "$remain" ]]; then
      return 0
    fi
  done

  while read -r pid; do
    [[ -n "$pid" ]] && kill -9 "$pid" 2>/dev/null || true
  done <<< "$remain"
}

collect_pattern_matches() {
  local pattern="$1"
  pgrep -af "$pattern" 2>/dev/null || true
}

pid_listen_on_port() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltnp "( sport = :$port )" 2>/dev/null \
      | awk -F'pid=' 'NF>1 {split($2,a,","); print a[1]}' \
      | tr -d '[:space:]' \
      | grep -E '^[0-9]+$' \
      | sort -u || true
    return 0
  fi

  if command -v lsof >/dev/null 2>&1; then
    lsof -nP -iTCP:"$port" -sTCP:LISTEN -t 2>/dev/null | sort -u || true
    return 0
  fi

  return 1
}

resolve_worker_namespace() {
  local worker_id="$1"
  local ns_prefix="$2"
  local ns_id=""

  if [[ "$worker_id" =~ ^[0-9]+$ ]]; then
    ns_id="$(printf '%02d' "$worker_id")"
  else
    ns_id="$worker_id"
  fi

  printf '/%s%s' "$ns_prefix" "$ns_id"
}

kill_worker_scoped_processes() {
  local worker_id="$1"
  local domain_id="$2"
  local gazebo_port="$3"
  local ns_prefix="$4"
  local worker_ns=""

  worker_ns="$(resolve_worker_namespace "$worker_id" "$ns_prefix")"
  echo "[AUTOSIM] worker=$worker_id scoped cleanup (ns=$worker_ns domain=$domain_id port=$gazebo_port)"

  kill_pattern_graceful "[r]os2 launch sjtu_drone_bringup.*${worker_ns}" "worker=$worker_id ros2 launch"
  kill_pattern_graceful "[a]priltag.*${worker_ns}" "worker=$worker_id apriltag"
  kill_pattern_graceful "[s]pawn_drone.*${worker_ns}" "worker=$worker_id spawn_drone"
  kill_pattern_graceful "[s]pawn_apriltag.*${worker_ns}" "worker=$worker_id spawn_apriltag"
  kill_pattern_graceful "[r]obot_state_publisher.*${worker_ns}" "worker=$worker_id robot_state_publisher"
  kill_pattern_graceful "[j]oint_state_publisher.*${worker_ns}" "worker=$worker_id joint_state_publisher"
  kill_pattern_graceful "[s]tatic_transform_publisher.*${worker_ns}" "worker=$worker_id static_transform_publisher"

  if [[ -n "$domain_id" && "$domain_id" =~ ^[0-9]+$ ]]; then
    kill_pattern_graceful "ROS_DOMAIN_ID=${domain_id}.*ros2 launch sjtu_drone_bringup" "worker=$worker_id domain wrapper"
  fi
  if [[ -n "$gazebo_port" && "$gazebo_port" =~ ^[0-9]+$ ]]; then
    kill_pattern_graceful "GAZEBO_MASTER_URI=http://127.0.0.1:${gazebo_port}" "worker=$worker_id gazebo wrapper"
    while IFS= read -r owner_pid; do
      if [[ -n "$owner_pid" ]] && kill -0 "$owner_pid" 2>/dev/null; then
        kill "$owner_pid" 2>/dev/null || true
        sleep 0.2
        kill -9 "$owner_pid" 2>/dev/null || true
      fi
    done < <(pid_listen_on_port "$gazebo_port")
  fi
}

is_port_listening() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn "( sport = :$port )" 2>/dev/null | tail -n +2 | grep -q .
    return $?
  fi

  if command -v lsof >/dev/null 2>&1; then
    lsof -nP -iTCP:"$port" -sTCP:LISTEN >/dev/null 2>&1
    return $?
  fi

  return 1
}

for tbl in "${PID_TABLES[@]}"; do
  if [[ ! -f "$tbl" ]]; then
    echo "[AUTOSIM] PID table not found: $tbl"
    continue
  fi

  echo "[AUTOSIM] Stopping workers from: $tbl"
  ns_prefix="drone_w"
  session_root_local="$(dirname "$tbl")"
  session_info_local="$session_root_local/session_info.txt"
  if [[ -f "$session_info_local" ]]; then
    parsed_prefix="$(awk -F'=' '/^multi_drone_namespace_prefix=/{print $2; exit}' "$session_info_local" | tr -d '[:space:]')"
    if [[ -n "$parsed_prefix" ]]; then
      ns_prefix="$parsed_prefix"
    fi
  fi

  while IFS=$'\t' read -r pid worker_id domain_id gazebo_port log_file; do
    if [[ -z "$pid" ]]; then
      continue
    fi

    kill_worker_scoped_processes "$worker_id" "$domain_id" "$gazebo_port" "$ns_prefix"

    if kill -0 "$pid" 2>/dev/null; then
      kill_tree "$pid"
      echo "[AUTOSIM] worker=$worker_id pid=$pid stopped"
    else
      echo "[AUTOSIM] worker=$worker_id pid=$pid already exited"
    fi

    # Post-pass in case children were re-parented before worker PID exit.
    kill_worker_scoped_processes "$worker_id" "$domain_id" "$gazebo_port" "$ns_prefix"

    if [[ -n "$gazebo_port" && "$gazebo_port" =~ ^[0-9]+$ ]]; then
      GAZEBO_PORTS+=("$gazebo_port")
    fi
  done < <(tail -n +2 "$tbl")
done

# de-duplicate tracked gazebo ports
if [[ ${#GAZEBO_PORTS[@]} -gt 0 ]]; then
  mapfile -t GAZEBO_PORTS < <(printf '%s\n' "${GAZEBO_PORTS[@]}" | sort -u)
fi

# Phase 2: Kill stray ROS visualization and simulation processes
echo "[AUTOSIM] Cleaning up RViz, Gazebo, and bridge/control processes..."
kill_pattern_graceful "(^|/)rviz2([[:space:]]|$)" "rviz2"
kill_pattern_graceful "(^|/)gzserver([[:space:]]|$)" "gzserver"
kill_pattern_graceful "(^|/)gzclient([[:space:]]|$)" "gzclient"
kill_pattern_graceful "(^|/)gz([[:space:]]+sim|$)" "gz sim"
kill_pattern_graceful "ign[[:space:]]+gazebo" "ign gazebo"
kill_pattern_graceful "(^|/)domain_bridge([[:space:]]|$)" "domain_bridge"
kill_pattern_graceful "[r]os2 launch sjtu_drone_bringup" "ros2 launch(sjtu_drone_bringup)"
kill_pattern_graceful "MATLAB[[:space:]].*-batch.*AutoSimMain" "MATLAB AutoSim batch"
kill_pattern_graceful "[a]priltag_detector_node" "apriltag_detector_node"
kill_pattern_graceful "[a]priltag_state_bridge" "apriltag_state_bridge"
kill_pattern_graceful "[s]pawn_drone" "spawn_drone"
kill_pattern_graceful "[s]pawn_apriltag" "spawn_apriltag"
kill_pattern_graceful "robot_state_publisher.*drone" "robot_state_publisher(drone)"
kill_pattern_graceful "(^|/)joy_node([[:space:]]|$)" "joy_node"
kill_pattern_graceful "teleop" "teleop"

# Phase 2.5: enforce cleanup on tracked gazebo ports.
if [[ ${#GAZEBO_PORTS[@]} -gt 0 ]]; then
  for port in "${GAZEBO_PORTS[@]}"; do
    if is_port_listening "$port"; then
      echo "[AUTOSIM] Forcing shutdown on occupied Gazebo port $port"
      while IFS= read -r owner_pid; do
        if [[ -n "$owner_pid" ]] && kill -0 "$owner_pid" 2>/dev/null; then
          kill "$owner_pid" 2>/dev/null || true
          sleep 0.2
          kill -9 "$owner_pid" 2>/dev/null || true
        fi
      done < <(pid_listen_on_port "$port")
    fi
  done
fi

# Phase 3: Final verification
sleep 1
remaining=""

for _ in {1..3}; do
  remaining=""
  for pattern in \
    "(^|/)rviz2([[:space:]]|$)" \
    "(^|/)gzserver([[:space:]]|$)" \
    "(^|/)gzclient([[:space:]]|$)" \
    "(^|/)gz([[:space:]]+sim|$)" \
    "ign[[:space:]]+gazebo" \
    "(^|/)domain_bridge([[:space:]]|$)" \
    "[r]os2 launch sjtu_drone_bringup" \
    "MATLAB[[:space:]].*-batch.*AutoSimMain" \
    "[a]priltag_detector_node" \
    "[a]priltag_state_bridge" \
    "[s]pawn_drone" \
    "[s]pawn_apriltag"; do
    matches="$(collect_pattern_matches "$pattern")"
    if [[ -n "$matches" ]]; then
      remaining+="$matches"$'\n'
    fi
  done

  if [[ -z "$remaining" ]]; then
    break
  fi

  # Retry targeted hard-stop once more before final warning.
  kill_pattern_graceful "(^|/)rviz2([[:space:]]|$)" "rviz2(retry)"
  kill_pattern_graceful "(^|/)gzserver([[:space:]]|$)" "gzserver(retry)"
  sleep 0.5
done

occupied_ports=""
if [[ ${#GAZEBO_PORTS[@]} -gt 0 ]]; then
  for port in "${GAZEBO_PORTS[@]}"; do
    if is_port_listening "$port"; then
      occupied_ports+="$port "
    fi
  done
fi

if [[ -n "$remaining" || -n "$occupied_ports" ]]; then
  echo "[AUTOSIM] Warning: residual simulation/visualization processes detected:"
  if [[ -n "$remaining" ]]; then
    printf '%s' "$remaining"
  fi
  if [[ -n "$occupied_ports" ]]; then
    echo "[AUTOSIM] Warning: occupied Gazebo ports after cleanup: $occupied_ports"
  fi
else
  echo "[AUTOSIM] Cleanup complete: no residual Gazebo/RViz/bridge processes detected."
fi
