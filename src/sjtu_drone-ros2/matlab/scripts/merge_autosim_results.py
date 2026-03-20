#!/usr/bin/env python3
import argparse
import csv
import glob
import os
from typing import Dict, List, Tuple


def find_worker_files(base_dir: str, pattern: str) -> List[str]:
    query = os.path.join(base_dir, "output", "data", "worker_*", "*", pattern)
    return sorted(glob.glob(query))


def parse_worker_run(path: str) -> Tuple[str, str]:
    norm = path.replace("\\", "/")
    parts = norm.split("/")
    worker_id = ""
    run_id = ""
    for i, part in enumerate(parts):
        if part.startswith("worker_"):
            worker_id = part.split("_", 1)[1]
            if i + 1 < len(parts):
                run_id = parts[i + 1]
            break
    return worker_id, run_id


def read_rows(csv_path: str) -> Tuple[List[str], List[Dict[str, str]]]:
    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = [dict(r) for r in reader]
        fields = list(reader.fieldnames or [])
    return fields, rows


def merge_group(files: List[str], output_csv: str) -> int:
    if not files:
        return 0

    merged_rows: List[Dict[str, str]] = []
    all_fields: List[str] = []

    def add_field(field: str) -> None:
        if field not in all_fields:
            all_fields.append(field)

    for path in files:
        fields, rows = read_rows(path)
        worker_id, run_id = parse_worker_run(path)

        for field in fields:
            add_field(field)

        add_field("worker_id")
        add_field("run_id")
        add_field("source_file")

        for row in rows:
            row["worker_id"] = worker_id
            row["run_id"] = run_id
            row["source_file"] = path
            merged_rows.append(row)

    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=all_fields)
        writer.writeheader()
        for row in merged_rows:
            writer.writerow({k: row.get(k, "") for k in all_fields})

    return len(merged_rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Merge parallel AutoSim worker outputs into unified CSV files.")
    parser.add_argument("session_root", help="Path to matlab/parallel_runs/<timestamp>")
    args = parser.parse_args()

    session_root = os.path.abspath(args.session_root)
    out_dir = os.path.join(session_root, "merged")

    groups = [
        ("autosim_dataset_*.csv", "autosim_dataset_merged.csv"),
        ("autosim_trace_*.csv", "autosim_trace_merged.csv"),
        ("autosim_learning_*.csv", "autosim_learning_merged.csv"),
    ]

    for pattern, out_name in groups:
        files = find_worker_files(session_root, pattern)
        out_csv = os.path.join(out_dir, out_name)
        count = merge_group(files, out_csv)
        print(f"[AUTOSIM] {pattern}: files={len(files)} rows={count} -> {out_csv}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
