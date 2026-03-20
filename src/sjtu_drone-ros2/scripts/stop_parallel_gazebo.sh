#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${WS_ROOT:-/home/j/INCSL/IICC26_ws}"
STATE_DIR="$WS_ROOT/.parallel_sim"
PID_FILE_INPUT="${1:-$STATE_DIR/latest.tsv}"

resolve_pid_file() {
  local in="$1"
  if [[ -L "$in" ]]; then
    readlink -f "$in"
  else
    echo "$in"
  fi
}

kill_tree() {
  local parent="$1"
  local children
  children="$(pgrep -P "$parent" || true)"
  if [[ -n "$children" ]]; then
    while read -r c; do
      [[ -n "$c" ]] && kill_tree "$c"
    done <<< "$children"
  fi

  if kill -0 "$parent" 2>/dev/null; then
    kill "$parent" 2>/dev/null || true
    sleep 0.3
    kill -9 "$parent" 2>/dev/null || true
  fi
}

PID_FILE="$(resolve_pid_file "$PID_FILE_INPUT")"
if [[ ! -f "$PID_FILE" ]]; then
  echo "[parallel] pid file not found: $PID_FILE" >&2
  exit 1
fi

tail -n +2 "$PID_FILE" | while IFS=$'\t' read -r pid domain_id gazebo_port log_file; do
  if [[ -z "$pid" ]]; then
    continue
  fi
  printf '[parallel] stopping pid=%s domain=%s gazebo_port=%s\n' "$pid" "$domain_id" "$gazebo_port"
  kill_tree "$pid"
done

echo "[parallel] stop sequence completed"
