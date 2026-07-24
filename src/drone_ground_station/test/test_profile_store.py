import json

import pytest

from drone_ground_station.profile_store import load_profile
from drone_ground_station.profile_store import save_profile


def test_profile_json_round_trip(tmp_path):
    path = tmp_path / "profile.json"
    expected = {"map": "narrow", "lookahead_distance": 0.55}
    save_profile(path, expected)
    assert load_profile(path) == expected


def test_profile_rejects_non_object(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps([1, 2, 3]), encoding="utf-8")
    with pytest.raises(ValueError):
        load_profile(path)
