#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MATLAB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  show_autosim_worker_progress.sh [session_root|workers.tsv] [-w [sec]]

Examples:
  show_autosim_worker_progress.sh
  show_autosim_worker_progress.sh /path/to/parallel_runs/20260323_010101
  show_autosim_worker_progress.sh -w
  show_autosim_worker_progress.sh /path/to/session -w 2

Notes:
  - Read-only monitor (workers.tsv + worker logs); no ROS topic or domain access required.
  - Safe to run while simulation is active.
EOF
}

sanitize_arg() {
  local x="$1"
  x="${x//$'\r'/}"
  x="${x//$'\n'/}"
  x="${x#\"}"
  x="${x%\"}"
  printf '%s' "$x"
}

resolve_session_root() {
  local arg="${1:-}"
  if [[ -z "$arg" ]]; then
    local latest
    latest="$(find "$MATLAB_DIR/parallel_runs" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -n 1 || true)"
    printf '%s' "$latest"
    return
  fi

  if [[ -f "$arg" && "$(basename "$arg")" == "workers.tsv" ]]; then
    printf '%s' "$(dirname "$arg")"
    return
  fi

  printf '%s' "$arg"
}

extract_progress() {
  local log_file="$1"
  if [[ ! -f "$log_file" ]]; then
    printf '0 0 -'
    return
  fi

  local last_line
  last_line="$(tail -n 600 "$log_file" 2>/dev/null | grep -E '\[AUTOSIM\][[:space:]]+Scenario[[:space:]]+[0-9]+/[0-9]+' | tail -n 1 || true)"
  if [[ -z "$last_line" ]]; then
    printf '0 0 -'
    return
  fi

  local now total
  now="$(printf '%s' "$last_line" | sed -nE 's/.*\[AUTOSIM\][[:space:]]+Scenario[[:space:]]+([0-9]+)\/([0-9]+).*/\1/p')"
  total="$(printf '%s' "$last_line" | sed -nE 's/.*\[AUTOSIM\][[:space:]]+Scenario[[:space:]]+([0-9]+)\/([0-9]+).*/\2/p')"
  if [[ -z "$now" || -z "$total" ]]; then
    printf '0 0 -'
    return
  fi

  printf '%s %s %s' "$now" "$total" "$last_line"
}

print_once() {
  local session_root="$1"
  local pid_table="$session_root/workers.tsv"

  if [[ ! -f "$pid_table" ]]; then
    echo "[AUTOSIM-PROGRESS] workers.tsv not found: $pid_table"
    return 1
  fi

  echo "[AUTOSIM-PROGRESS] Session: $session_root"
  echo "[AUTOSIM-PROGRESS] Time: $(date '+%F %T')"
  printf '%-8s %-8s %-9s %-8s %-14s %-20s\n' "worker" "domain" "gazebo" "pid" "scenario" "status"

  while IFS=$'\t' read -r pid worker_id domain_id gazebo_port log_file; do
    if [[ "$pid" == "pid" || -z "$worker_id" ]]; then
      continue
    fi

    local status="exited"
    if kill -0 "$pid" 2>/dev/null; then
      status="running"
    fi

    local now total raw
    read -r now total raw < <(extract_progress "$log_file")
    local prog="-"
    if [[ "$total" != "0" ]]; then
      prog="$now/$total"
    fi

    printf '%-8s %-8s %-9s %-8s %-14s %-20s\n' \
      "$worker_id" "${domain_id:-n/a}" "${gazebo_port:-n/a}" "$pid" "$prog" "$status"
  done < "$pid_table"
}

watch_mode=0
watch_sec=2
session_arg=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -w|--watch)
      watch_mode=1
      if [[ $# -gt 1 && "$2" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        watch_sec="$2"
        shift
      fi
      ;;
    *)
      if [[ -z "$session_arg" ]]; then
        session_arg="$(sanitize_arg "$1")"
      else
        echo "[AUTOSIM-PROGRESS] Unknown argument: $1"
        usage
        exit 1
      fi
      ;;
  esac
  shift
done

session_root="$(resolve_session_root "$session_arg")"
if [[ -z "$session_root" ]]; then
  echo "[AUTOSIM-PROGRESS] No session found under $MATLAB_DIR/parallel_runs"
  exit 1
fi

if [[ "$watch_mode" -eq 0 ]]; then
  print_once "$session_root"
  exit $?
fi

while true; do
  clear
  print_once "$session_root" || true
  sleep "$watch_sec"
done
