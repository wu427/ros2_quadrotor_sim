"""Ground-station showcase profile loading."""

from __future__ import annotations

from pathlib import Path
from typing import Dict

from ament_index_python.packages import get_package_share_directory
import yaml


def load_profiles() -> Dict[str, Dict[str, object]]:
    """Load installed profile definitions and resolve map paths."""
    share = Path(get_package_share_directory("drone_ground_station"))
    map_share = Path(get_package_share_directory("drone_map"))
    document = yaml.safe_load(
        (share / "config" / "showcase_profiles.yaml").read_text(
            encoding="utf-8"
        )
    )
    profiles = document.get("profiles", {})
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError("showcase profiles must be a non-empty mapping")
    result = {}
    for name, raw in profiles.items():
        if not isinstance(raw, dict):
            raise ValueError(f"profile '{name}' must be a mapping")
        profile = dict(raw)
        profile["map_path"] = str(map_share / "config" / raw["map_file"])
        result[str(name)] = profile
    return result
