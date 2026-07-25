#!/usr/bin/env python3
"""Export showcase acceptance log metrics to CSV, JSON, and PNG."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import re
from typing import Dict, List

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot  # noqa: E402


def _metrics(path: Path) -> Dict[str, object]:
    text = path.read_text(encoding="utf-8", errors="replace")
    matches = re.findall(r"SHOWCASE_METRICS_JSON=(\{.*\})", text)
    if not matches:
        raise ValueError(f"no metrics record in {path}")
    payload = json.loads(matches[-1])
    payload["log_file"] = str(path)
    return payload


def main() -> int:
    """Parse arguments and write the three experiment artifacts."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("logs", nargs="+", type=Path)
    arguments = parser.parse_args()
    rows: List[Dict[str, object]] = [
        _metrics(path) for path in arguments.logs
    ]
    arguments.output.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row})
    with (arguments.output / "showcase_metrics.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (arguments.output / "showcase_metrics.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    labels = [str(row["scenario"]) for row in rows]
    elapsed = [float(row["elapsed_sec"]) for row in rows]
    planning = [float(row["planning_time_sec"]) for row in rows]
    figure, axes = pyplot.subplots(1, 2, figsize=(10, 4))
    axes[0].bar(labels, elapsed, color="#2ca02c")
    axes[0].set_ylabel("mission time (s)")
    axes[0].tick_params(axis="x", rotation=25)
    axes[1].bar(labels, planning, color="#1f77b4")
    axes[1].set_ylabel("planning time (s)")
    axes[1].tick_params(axis="x", rotation=25)
    figure.tight_layout()
    figure.savefig(arguments.output / "showcase_metrics.png", dpi=180)
    pyplot.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
