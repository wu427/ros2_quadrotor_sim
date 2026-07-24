"""JSON persistence for user-selected ground-station profiles."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict


def save_profile(path: Path | str, profile: Dict[str, object]) -> None:
    """Write a user profile without touching repository YAML defaults."""
    target = Path(path)
    target.write_text(
        json.dumps(profile, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )


def load_profile(path: Path | str) -> Dict[str, object]:
    """Load and validate a JSON user profile."""
    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid profile '{source}': {error}") from error
    if not isinstance(payload, dict):
        raise ValueError("profile root must be a JSON object")
    return payload
