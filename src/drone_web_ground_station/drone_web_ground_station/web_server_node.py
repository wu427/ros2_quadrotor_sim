"""Standard-library HTTP/SSE server for the quadrotor web dashboard."""

from __future__ import annotations

from functools import partial
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import threading
import time
from typing import Any
from urllib.parse import urlparse
import webbrowser

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException

from drone_web_ground_station.api_models import parse_goal
from drone_web_ground_station.api_models import parse_waypoints
from drone_web_ground_station.ros_bridge import WebRosBridge


class DashboardServer(ThreadingHTTPServer):
    """Threading HTTP server carrying references to ROS and static assets."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        bridge: WebRosBridge,
        static_root: Path,
        output_root: Path,
    ) -> None:
        super().__init__(address, DashboardHandler)
        self.bridge = bridge
        self.static_root = static_root
        self.output_root = output_root
        self.stopping = threading.Event()


class DashboardHandler(BaseHTTPRequestHandler):
    """Serve static files, JSON APIs and one SSE event stream."""

    server: DashboardServer

    def log_message(self, format_string: str, *args: object) -> None:
        """Forward HTTP access messages to the ROS debug logger."""
        self.server.bridge.get_logger().debug(format_string % args)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler contract.
        """Serve JSON, SSE, or one local static asset."""
        parsed = urlparse(self.path)
        if parsed.path == "/api/health":
            self._json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "message": "healthy",
                    "data": {"time": time.time()},
                },
            )
        elif parsed.path in {"/api/status", "/api/config"}:
            self._json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "message": "snapshot",
                    "data": self.server.bridge.snapshot(),
                },
            )
        elif parsed.path == "/api/maps":
            self._json(HTTPStatus.OK, {"ok": True, "message": "maps", "data": self._maps()})
        elif parsed.path == "/api/waypoints":
            snapshot = self.server.bridge.snapshot()
            self._json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "message": "waypoints",
                    "data": snapshot["paths"]["waypoints"],
                },
            )
        elif parsed.path == "/api/latest-run":
            self._json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "message": "recording",
                    "data": self.server.bridge.snapshot()["recording"],
                },
            )
        elif parsed.path == "/api/sse":
            self._sse()
        else:
            self._static(parsed.path)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler contract.
        """Validate and execute one JSON API command."""
        parsed = urlparse(self.path)
        try:
            payload = self._body_json()
            data: Any = {}
            message = "ok"
            if parsed.path == "/api/goal/preview":
                goal = parse_goal(payload.get("goal", payload))
                valid, reason = self.server.bridge.validate_goal(goal)
                data = {"goal": list(goal), "valid": valid, "reason": reason}
                if not valid:
                    self._json(
                        HTTPStatus.UNPROCESSABLE_ENTITY,
                        {"ok": False, "message": reason, "data": data},
                    )
                    return
                message = "goal valid"
            elif parsed.path == "/api/goal/send":
                goal = parse_goal(payload.get("goal", payload))
                self.server.bridge.send_goal(goal)
                data = {"goal": list(goal)}
                message = "goal sent"
            elif parsed.path == "/api/waypoints":
                points = parse_waypoints(payload.get("waypoints", payload))
                self.server.bridge.set_waypoints(points)
                data = {"waypoints": [list(point) for point in points]}
                message = "waypoints updated"
            elif parsed.path == "/api/patrol/start":
                points = parse_waypoints(payload.get("waypoints", []))
                self.server.bridge.set_waypoints(points)
                config = payload.get("config", {})
                if not isinstance(config, dict):
                    raise ValueError("config must be an object")
                self.server.bridge.start_patrol(config)
                data = {"waypoints": [list(point) for point in points], "config": config}
                message = "patrol started"
            elif parsed.path.startswith("/api/patrol/"):
                command = parsed.path.rsplit("/", 1)[-1].upper()
                mapping = {
                    "PAUSE": "PAUSE",
                    "RESUME": "RESUME",
                    "SKIP": "SKIP",
                    "STOP": "STOP",
                }
                if command not in mapping:
                    raise ValueError("unsupported patrol endpoint")
                self.server.bridge.command(mapping[command])
                message = f"{command.lower()} requested"
            elif parsed.path == "/api/return-home":
                self.server.bridge.command("RETURN_HOME")
                message = "return home requested"
            elif parsed.path == "/api/hold":
                self.server.bridge.command("PAUSE")
                message = "hold requested"
            elif parsed.path == "/api/clear":
                self.server.bridge.command("CLEAR")
                message = "mission cleared"
            elif parsed.path == "/api/simulation/start":
                map_file = payload.get("map_file")
                rviz = bool(payload.get("rviz", False))
                pid = self.server.bridge.start_simulation(
                    str(map_file) if map_file else None,
                    rviz,
                )
                data = {"pid": pid}
                message = "simulation started"
            elif parsed.path == "/api/simulation/stop":
                self.server.bridge.stop_simulation()
                message = "owned simulation stopped"
            elif parsed.path == "/api/rviz/altitude":
                self.server.bridge.set_rviz_altitude(
                    float(payload.get("z", 1.5))
                )
                message = "RViz target altitude updated"
            elif parsed.path == "/api/rviz/start":
                data = {"pid": self.server.bridge.start_rviz()}
                message = "RViz started"
            elif parsed.path == "/api/rviz/stop":
                self.server.bridge.stop_rviz()
                message = "owned RViz stopped"
            elif parsed.path == "/api/export":
                target = self.server.bridge.export(self.server.output_root)
                data = {"directory": str(target)}
                message = "task exported"
            else:
                self._json(
                    HTTPStatus.NOT_FOUND,
                    {"ok": False, "message": "unknown API", "data": {}},
                )
                return
            self._json(HTTPStatus.OK, {"ok": True, "message": message, "data": data})
        except (ValueError, RuntimeError) as error:
            self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error), "data": {}})
        except json.JSONDecodeError:
            self._json(
                HTTPStatus.BAD_REQUEST,
                {"ok": False, "message": "invalid JSON", "data": {}},
            )
        except Exception as error:  # HTTP boundary: log details, return safe message.
            self.server.bridge.get_logger().error(f"Web API error on {parsed.path}: {error}")
            self._json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {
                    "ok": False,
                    "message": "internal server error",
                    "data": {},
                },
            )

    def _body_json(self) -> dict[str, object]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("request JSON must be an object")
        return value

    def _json(self, status: HTTPStatus, payload: object) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(int(status))
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _sse(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        while not self.server.stopping.is_set():
            try:
                payload = json.dumps(
                    self.server.bridge.snapshot(),
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
                self.wfile.write(f"event: snapshot\ndata: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                time.sleep(0.2)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                break

    def _static(self, request_path: str) -> None:
        relative = (
            "index.html"
            if request_path in {"", "/"}
            else request_path.lstrip("/")
        )
        relative_path = Path(relative)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        candidate = self.server.static_root / relative_path
        if not candidate.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        body = candidate.read_bytes()
        content_type, _encoding = mimetypes.guess_type(str(candidate))
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    @staticmethod
    def _maps() -> list[dict[str, str]]:
        share = Path(get_package_share_directory("drone_map")) / "config"
        names = {
            "open_space.yaml": "Open Space",
            "single_obstacle.yaml": "Single Obstacle",
            "static_map.yaml": "Six Obstacles",
            "narrow_passage.yaml": "Narrow Passage",
            "random_seed_42.yaml": "Fixed Seed 42",
            "no_path.yaml": "No Path (Test)",
        }
        return [
            {"label": label, "path": str(share / filename)}
            for filename, label in names.items()
            if (share / filename).is_file()
        ]


def _workspace_root() -> Path:
    share = Path(get_package_share_directory("drone_web_ground_station")).resolve()
    for ancestor in share.parents:
        if ancestor.name == "install":
            return ancestor.parent
    return Path.cwd()


def main(args=None) -> None:
    """Run the ROS bridge and dashboard HTTP server."""
    rclpy.init(args=args)
    parameter_node = WebRosBridge()
    host = str(parameter_node.declare_parameter("host", "127.0.0.1").value)
    port = int(parameter_node.declare_parameter("port", 8765).value)
    open_browser = bool(parameter_node.declare_parameter("open_browser", True).value)
    static_root = Path(get_package_share_directory("drone_web_ground_station")) / "static"
    output_root = _workspace_root() / "output" / "web_runs"
    server = DashboardServer((host, port), parameter_node, static_root, output_root)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    url = f"http://{host}:{port}"
    parameter_node.get_logger().info(f"Web ground station ready at {url}")
    if open_browser:
        threading.Timer(0.8, partial(webbrowser.open, url)).start()
    try:
        rclpy.spin(parameter_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.stopping.set()
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=3.0)
        parameter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
