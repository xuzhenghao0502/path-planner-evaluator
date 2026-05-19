#!/usr/bin/env python3
"""
Web Bridge Node — FastAPI + rclpy bridge for browser-based planning.

Architecture:
  Main thread: rclpy.spin(node)
  Background thread (daemon): uvicorn.run(app)

Cross-thread communication via queue.Queue:
  to_ros_queue:   FastAPI → ROS2   (publish polygons, set start/goal, trigger plan)
  from_ros_queue: ROS2    → FastAPI (plan results → WebSocket broadcast)
"""

import json
import os
import math
import queue
import threading
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Point
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
from std_srvs.srv import Trigger

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

# ---------------------------------------------------------------------------
# Token auth helpers (simple HMAC-signed token, no DB)
# ---------------------------------------------------------------------------
try:
    from jose import jwt, JWTError
except ImportError:
    jwt = None
    JWTError = Exception

SECRET_KEY = os.environ.get("JWT_SECRET", "openspace-web-secret-key-change-me")
TOKEN_EXPIRE_SECONDS = 3600

security = HTTPBearer(auto_error=False)


def create_token(username: str) -> str:
    if jwt is None:
        return f"token:{username}:{uuid.uuid4().hex}"
    return jwt.encode(
        {"sub": username, "exp": time.time() + TOKEN_EXPIRE_SECONDS},
        SECRET_KEY,
        algorithm="HS256",
    )


def verify_token(token_str: str) -> str | None:
    if jwt is None:
        parts = token_str.split(":")
        if len(parts) >= 2 and parts[0] == "token":
            return parts[1]
        return None
    try:
        payload = jwt.decode(token_str, SECRET_KEY, algorithms=["HS256"])
        return payload.get("sub")
    except JWTError:
        return None


def require_auth(credentials: HTTPAuthorizationCredentials = Depends(security)):
    if credentials is None or not verify_token(credentials.credentials):
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid or missing token")
    return credentials


# ---------------------------------------------------------------------------
# Global state (shared between ROS2 node and FastAPI endpoints)
# ---------------------------------------------------------------------------
class SharedState:
    def __init__(self):
        self.polygons: list[dict] = []
        self.start_pose: dict | None = None
        self.goal_pose: dict | None = None
        self.vehicle_config: dict = {}
        self.presets: list[dict] = []
        self.active_preset: str = "9m6"
        self.scene_dir: str = ""
        self.latest_path: list[dict] | None = None
        self.latest_boxes: list[dict] | None = None
        self.plan_success: bool | None = None
        self.plan_message: str = ""
        self.ws_clients: list[WebSocket] = []
        self.lock = threading.Lock()


state = SharedState()
to_ros_queue = queue.Queue()   # FastAPI → ROS2
from_ros_queue = queue.Queue() # ROS2 → FastAPI (for WS broadcast)


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------
app = FastAPI(title="Openspace Planner Web")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---- WebSocket broadcast helpers ----
async def broadcast_to_clients(msg: dict):
    dead = []
    for ws in state.ws_clients:
        try:
            await ws.send_json(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        try:
            state.ws_clients.remove(ws)
        except ValueError:
            pass


def enqueue_ros_task(task_type: str, data: dict | None = None):
    """Schedule a task to be executed on the ROS2 thread."""
    to_ros_queue.put((task_type, data or {}))


# ---- REST Endpoints ----
@app.get("/api/health")
async def health():
    return {"status": "ok"}


@app.get("/api/health/ros2")
async def health_ros2():
    return {"ros2_ready": True, "queue_size": to_ros_queue.qsize()}


@app.post("/api/login")
async def login(body: dict):
    username = body.get("username", "")
    password = body.get("password", "")
    web_user = os.environ.get("WEB_USER", "admin")
    web_pass = os.environ.get("WEB_PASS", "admin123")
    if username == web_user and password == web_pass:
        token = create_token(username)
        return {"token": token, "expires_in": TOKEN_EXPIRE_SECONDS}
    raise HTTPException(status_code=401, detail="Invalid credentials")


@app.get("/api/scenes")
async def list_scenes(_=Depends(require_auth)):
    scene_dir = Path(state.scene_dir)
    if not scene_dir.exists():
        return {"scenes": []}
    scenes = []
    for f in sorted(scene_dir.glob("*.json")):
        try:
            data = json.loads(f.read_text())
            n = len(data.get("polygons", []))
        except Exception:
            n = 0
        scenes.append({"name": f.name, "polygon_count": n})
    return {"scenes": scenes}


@app.get("/api/scenes/{name}")
async def get_scene(name: str, _=Depends(require_auth)):
    path = Path(state.scene_dir) / name
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"Scene not found: {name}")
    data = json.loads(path.read_text())
    return {"name": name, "polygons": data.get("polygons", [])}


@app.post("/api/scenes/save")
async def save_scene(body: dict, _=Depends(require_auth)):
    name = body.get("name", "scene.json")
    polygons = body.get("polygons", [])
    path = Path(state.scene_dir) / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"polygons": polygons}, indent=2))
    with state.lock:
        state.polygons = polygons
    enqueue_ros_task("publish_polygons", {"polygons": polygons})
    return {"success": True, "name": name, "polygon_count": len(polygons)}


@app.delete("/api/scenes/{name}")
async def delete_scene(name: str, _=Depends(require_auth)):
    path = Path(state.scene_dir) / name
    if path.exists():
        path.unlink()
    return {"success": True}


@app.get("/api/polygons")
async def get_polygons(_=Depends(require_auth)):
    with state.lock:
        return {"polygons": state.polygons}


@app.post("/api/polygons")
async def set_polygons(body: dict, _=Depends(require_auth)):
    polygons = body.get("polygons", [])
    with state.lock:
        state.polygons = polygons
    enqueue_ros_task("publish_polygons", {"polygons": polygons})
    return {"success": True, "polygon_count": len(polygons)}


@app.post("/api/plan")
async def trigger_plan(body: dict, _=Depends(require_auth)):
    start = body.get("start")
    goal = body.get("goal")
    if not start or not goal:
        raise HTTPException(status_code=400, detail="start and goal are required")

    polygons = body.get("polygons")
    if polygons is not None:
        with state.lock:
            state.polygons = polygons

    with state.lock:
        state.start_pose = start
        state.goal_pose = goal

    enqueue_ros_task("plan", {
        "start": start,
        "goal": goal,
        "polygons": polygons if polygons is not None else state.polygons,
    })
    return {"status": "accepted"}


@app.get("/api/vehicle-config")
async def get_vehicle_config(_=Depends(require_auth)):
    with state.lock:
        return {
            "preset": state.active_preset,
            "presets": state.presets,
            "params": state.vehicle_config,
        }


@app.post("/api/vehicle-config")
async def set_vehicle_config(body: dict, _=Depends(require_auth)):
    preset_name = body.get("preset")
    custom_params = body.get("params")

    with state.lock:
        if preset_name and preset_name != "custom":
            for p in state.presets:
                if p["name"] == preset_name:
                    state.vehicle_config = dict(p["params"])
                    state.active_preset = preset_name
                    break
            else:
                raise HTTPException(status_code=404, detail=f"Preset not found: {preset_name}")
        elif custom_params:
            state.vehicle_config = dict(custom_params)
            state.active_preset = "custom"

    note = ""
    if preset_name and preset_name != state.active_preset:
        note = "Vehicle config applied to web bridge. Restart planner_bridge_node with updated params to take effect."

    return {"success": True, "preset": state.active_preset, "params": state.vehicle_config, "note": note}


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    token = ws.query_params.get("token", "")
    if not verify_token(token):
        await ws.close(code=4001)
        return
    await ws.accept()
    state.ws_clients.append(ws)
    try:
        while True:
            data = await ws.receive_json()
            if data.get("type") == "ping":
                await ws.send_json({"type": "pong"})
    except (WebSocketDisconnect, Exception):
        pass
    finally:
        try:
            state.ws_clients.remove(ws)
        except ValueError:
            pass


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------
class WebBridgeNode(Node):
    def __init__(self):
        super().__init__("web_bridge_node")

        # Declare ROS2 params
        self.declare_parameter("scene_dir", "/root/ros2_ws/src/openspace_ros2_bridge/scenes")
        self.declare_parameter("scene_name", "customer_map.json")
        self.declare_parameter("web_port", 8000)
        self.declare_parameter("serve_static", True)
        self.declare_parameter("static_dir", "/root/ros2_ws/src/openspace_ros2_bridge/static")

        # Set scene dir in shared state
        state.scene_dir = self.get_parameter("scene_dir").value

        # ---- Publishers ----
        self.polygon_pub = self.create_publisher(MarkerArray, "/obstacle_polygons", 10)
        self.start_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # ---- Subscribers (planning results) ----
        self.path_sub = self.create_subscription(NavPath, "/trajectory_path", self._on_path, 10)
        self.box_sub = self.create_subscription(MarkerArray, "/vehicle_boxes", self._on_boxes, 10)
        self.result_sub = self.create_subscription(String, "/planning_result", self._on_result, 10)

        # ---- Service client ----
        self.plan_client = self.create_client(Trigger, "/planner_bridge_node/plan_path")

        # ---- Timer to process incoming queue ----
        self._pending_plan = None
        self._plan_future = None
        self._plan_start_time = 0.0
        self.create_timer(0.05, self._process_queue)

        # ---- Load vehicle presets ----
        self._load_vehicle_presets()

        # Schedule initial polygon publish (empty)
        enqueue_ros_task("publish_polygons", {"polygons": []})

        self.get_logger().info("Web Bridge Node ready")

    # ---- Vehicle presets ----
    def _load_vehicle_presets(self):
        preset_paths = [
            Path(__file__).parent.parent / "config" / "vehicle_presets.json",
            Path(state.scene_dir).parent / "config" / "vehicle_presets.json",
            Path("/root/ros2_ws/src/openspace_ros2_bridge/config/vehicle_presets.json"),
        ]
        for pp in preset_paths:
            if pp.exists():
                try:
                    data = json.loads(pp.read_text())
                    state.presets = data.get("presets", [])
                    # Set default to 9m6
                    for p in state.presets:
                        if p["name"] == "9m6":
                            state.vehicle_config = dict(p["params"])
                            state.active_preset = "9m6"
                            break
                    self.get_logger().info(f"Loaded {len(state.presets)} vehicle presets from {pp}")
                    return
                except Exception as e:
                    self.get_logger().warn(f"Failed to load presets from {pp}: {e}")

        # Fallback defaults
        state.vehicle_config = {
            "vehicle.length": 12.0,
            "vehicle.width": 2.6,
            "vehicle.wheel_base": 7.1,
            "vehicle.front_overhang": 1.46,
            "vehicle.rear_overhang": 3.33,
            "vehicle.rear_edge_to_ego": 3.5,
            "vehicle.width_without_rearview_mirror": 2.6,
        }
        state.presets = [
            {"name": "9m6", "label": "9m6 (12m x 2.6m, 轴距 7.1m)", "params": dict(state.vehicle_config)},
            {"name": "custom", "label": "Custom", "params": {}},
        ]
        self.get_logger().info("Using default vehicle presets (vehicle_presets.json not found)")

    # ---- Subscriber callbacks (run on ROS2 thread) ----
    def _on_path(self, msg: NavPath):
        path_data = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose
            yaw = math.atan2(
                2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                1.0 - 2.0 * (p.orientation.y * p.orientation.y + p.orientation.z * p.orientation.z),
            )
            path_data.append({"x": p.position.x, "y": p.position.y, "yaw_deg": math.degrees(yaw)})
        state.latest_path = path_data
        from_ros_queue.put({"type": "path_update", "data": path_data})

    def _on_boxes(self, msg: MarkerArray):
        boxes = []
        for m in msg.markers:
            if m.points:
                cx = sum(pt.x for pt in m.points) / len(m.points)
                cy = sum(pt.y for pt in m.points) / len(m.points)
                boxes.append({"x": cx, "y": cy})
        state.latest_boxes = boxes
        from_ros_queue.put({"type": "boxes_update", "data": boxes})

    def _on_result(self, msg: String):
        from_ros_queue.put({"type": "planning_result", "data": msg.data})

    # ---- Main queue processor (runs on ROS2 thread via timer) ----
    def _process_queue(self):
        # Process incoming tasks from FastAPI
        try:
            while True:
                task_type, data = to_ros_queue.get_nowait()
                self._execute_task(task_type, data)
        except queue.Empty:
            pass

        # from_ros_queue is handled by the asyncio flush loop on the uvicorn thread

    def _execute_task(self, task_type: str, data: dict):
        if task_type == "publish_polygons":
            self._do_publish_polygons(data.get("polygons", []))
        elif task_type == "plan":
            self._do_plan(data)
        elif task_type == "publish_start":
            self._do_publish_start(data)
        elif task_type == "publish_goal":
            self._do_publish_goal(data)

    def _do_publish_polygons(self, polygons: list):
        arr = MarkerArray()
        marker_id = 1

        for i, poly in enumerate(polygons):
            vertices = poly.get("vertices", [])
            is_occupied = poly.get("is_occupied", True)
            if len(vertices) < 3:
                continue

            # LINE_STRIP outline (same format as map_editor_node)
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "polygon_outline"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.08
            m.color.a = 0.8
            m.color.r = 1.0 if is_occupied else 0.0
            m.color.g = 0.0 if is_occupied else 1.0
            m.color.b = 0.0

            for x, y in vertices:
                m.points.append(Point(x=float(x), y=float(y), z=0.0))
            if vertices:
                m.points.append(Point(x=float(vertices[0][0]), y=float(vertices[0][1]), z=0.0))

            arr.markers.append(m)

        self.polygon_pub.publish(arr)
        self.get_logger().debug(f"Published {len(arr.markers)} polygon outlines")

    def _do_publish_start(self, data: dict):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = _make_pose(data.get("x", 0), data.get("y", 0), data.get("yaw_deg", 0))
        self.start_pub.publish(msg)

    def _do_publish_goal(self, data: dict):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = _make_pose(data.get("x", 0), data.get("y", 0), data.get("yaw_deg", 0))
        self.goal_pub.publish(msg)

    def _do_plan(self, data: dict):
        # Reset results
        state.latest_path = None
        state.latest_boxes = None
        state.plan_success = None
        state.plan_message = ""

        start = data.get("start", {})
        goal = data.get("goal", {})
        polygons = data.get("polygons", state.polygons)

        # 1. Publish polygons
        self._do_publish_polygons(polygons)

        # 2. Publish start
        start_msg = PoseWithCovarianceStamped()
        start_msg.header.frame_id = "map"
        start_msg.header.stamp = self.get_clock().now().to_msg()
        start_msg.pose.pose = _make_pose(start.get("x", 0), start.get("y", 0), start.get("yaw_deg", 0))
        self.start_pub.publish(start_msg)

        # 3. Publish goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = _make_pose(goal.get("x", 0), goal.get("y", 0), goal.get("yaw_deg", 0))
        self.goal_pub.publish(goal_msg)

        # Small delay for subscribers
        time.sleep(0.3)

        # Signal computing status
        from_ros_queue.put({"type": "plan_status", "status": "computing"})

        # 4. Call planning service in a worker thread to avoid deadlock.
        #    The timer callback must return so the executor can process
        #    the service response.
        def _plan_worker():
            if not self.plan_client.wait_for_service(timeout_sec=5.0):
                from_ros_queue.put({
                    "type": "plan_result",
                    "success": False,
                    "message": "Planner service not available",
                })
                state.plan_success = False
                state.plan_message = "Planner service not available"
                return

            req = Trigger.Request()
            resp = self.plan_client.call(req)
            # Brief sleep so executor can dispatch subscriber callbacks
            # (path_update, boxes_update) before we read state.latest_path
            time.sleep(0.3)
            state.plan_success = resp.success
            state.plan_message = resp.message

            if resp.success and state.latest_path:
                total_len = 0.0
                pts = state.latest_path
                for i in range(1, len(pts)):
                    dx = pts[i]["x"] - pts[i - 1]["x"]
                    dy = pts[i]["y"] - pts[i - 1]["y"]
                    total_len += math.sqrt(dx * dx + dy * dy)

                from_ros_queue.put({
                    "type": "plan_result",
                    "success": True,
                    "message": resp.message,
                    "path": state.latest_path,
                    "boxes": state.latest_boxes or [],
                    "path_length": round(total_len, 2),
                    "point_count": len(state.latest_path),
                })
            else:
                from_ros_queue.put({
                    "type": "plan_result",
                    "success": False,
                    "message": resp.message,
                })

        threading.Thread(target=_plan_worker, daemon=True, name="plan-worker").start()



def _make_pose(x: float, y: float, yaw_deg: float) -> Pose:
    yaw = math.radians(yaw_deg)
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = 0.0
    p.orientation.z = math.sin(yaw / 2.0)
    p.orientation.w = math.cos(yaw / 2.0)
    return p


# ---------------------------------------------------------------------------
# Background loop: flush from_ros_queue → WebSocket
# ---------------------------------------------------------------------------
def _run_ws_flush_loop(loop):
    """Run on the asyncio event loop to poll from_ros_queue and broadcast."""
    async def _flush():
        while True:
            try:
                msg = from_ros_queue.get_nowait()
            except queue.Empty:
                pass
            else:
                await broadcast_to_clients(msg)
            await asyncio_sleep(0.1)

    import asyncio
    async def asyncio_sleep(s):
        await asyncio.sleep(s)

    asyncio.ensure_future(_flush(), loop=loop)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = WebBridgeNode()

    web_port = node.get_parameter("web_port").value
    serve_static = node.get_parameter("serve_static").value
    static_dir = node.get_parameter("static_dir").value

    # Mount static files if serving directly (dev mode, no nginx)
    if serve_static and os.path.isdir(static_dir):
        app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")
        node.get_logger().info(f"Serving static files from {static_dir}")

    # Start uvicorn in background daemon thread
    def run_uvicorn():
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # Start the WS flush loop
        async def flush_loop():
            while True:
                try:
                    # Drain all available messages in a burst
                    while True:
                        msg = from_ros_queue.get_nowait()
                        try:
                            await broadcast_to_clients(msg)
                        except Exception as exc:
                            import traceback
                            traceback.print_exc()
                except queue.Empty:
                    pass
                await asyncio.sleep(0.05)

        async def startup():
            asyncio.create_task(flush_loop())

        config = uvicorn.Config(app, host="0.0.0.0", port=web_port, log_level="warning")
        server = uvicorn.Server(config)
        loop.run_until_complete(startup())
        loop.run_until_complete(server.serve())

    thread = threading.Thread(target=run_uvicorn, daemon=True, name="uvicorn-thread")
    thread.start()
    node.get_logger().info(f"Web server started on http://0.0.0.0:{web_port}")

    # Spin ROS2 on main thread (blocks until shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
