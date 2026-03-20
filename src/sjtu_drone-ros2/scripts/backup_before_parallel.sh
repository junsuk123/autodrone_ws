#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${1:-/home/j/INCSL/IICC26_ws}"
PROJECT_DIR="$WS_ROOT/src/sjtu_drone-ros2"
BACKUP_DIR="$WS_ROOT/backups"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_TAR="$BACKUP_DIR/iicc26_before_parallel_${STAMP}.tar.gz"
META_FILE="$BACKUP_DIR/iicc26_before_parallel_${STAMP}.meta.txt"

if [[ ! -d "$PROJECT_DIR" ]]; then
  echo "[backup] project directory not found: $PROJECT_DIR" >&2
  exit 1
fi

mkdir -p "$BACKUP_DIR"

{
  echo "timestamp=$STAMP"
  echo "workspace=$WS_ROOT"
  echo "project_dir=$PROJECT_DIR"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -srmo)"
  echo ""
  echo "[git status]"
  git -C "$PROJECT_DIR" status --short || true
  echo ""
  echo "[git rev-parse HEAD]"
  git -C "$PROJECT_DIR" rev-parse HEAD || true
} > "$META_FILE"

tar \
  --exclude='.git' \
  --exclude='build' \
  --exclude='install' \
  --exclude='log' \
  --exclude='matlab/data' \
  --exclude='matlab/logs' \
  --exclude='matlab/plots' \
  --exclude='matlab/models' \
  -czf "$OUT_TAR" \
  -C "$WS_ROOT/src" "sjtu_drone-ros2"

printf '[backup] archive: %s\n' "$OUT_TAR"
printf '[backup] metadata: %s\n' "$META_FILE"
