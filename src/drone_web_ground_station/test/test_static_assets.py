"""Ensure the offline single-page dashboard ships all local assets."""

from pathlib import Path


def test_static_assets_are_offline() -> None:
    root = Path(__file__).parents[1] / "static"
    html = (root / "index.html").read_text(encoding="utf-8")
    javascript = (root / "app.js").read_text(encoding="utf-8")
    assert "mapCanvas" in html
    assert "巡航模式" in html
    assert "EventSource" in javascript
    assert "https://" not in html
    assert "https://" not in javascript
