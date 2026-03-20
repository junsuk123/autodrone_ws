#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MATLAB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SESSION_ROOT="${1:-}"
PID_TABLE=""
PID_TABLES=()

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

for tbl in "${PID_TABLES[@]}"; do
  if [[ ! -f "$tbl" ]]; then
    echo "[AUTOSIM] PID table not found: $tbl"
    continue
  fi

  echo "[AUTOSIM] Stopping workers from: $tbl"
  tail -n +2 "$tbl" | while IFS=$'\t' read -r pid worker_id domain_id gazebo_port log_file; do
    if [[ -z "$pid" ]]; then
      continue
    fi

    if kill -0 "$pid" 2>/dev/null; then
      kill_tree "$pid"
      echo "[AUTOSIM] worker=$worker_id pid=$pid stopped"
    else
      echo "[AUTOSIM] worker=$worker_id pid=$pid already exited"
    fi
  done
done
