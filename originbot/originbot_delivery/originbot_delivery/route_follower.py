#!/usr/bin/env python3
"""轨道路网导航节点（route_follower）。

与自由避障导航（Nav2：DWB 沿代价地图自由绕行）互补的"轨道式"导航：
  - 沿预设拓扑路网（routes.json：节点 + 边，边可带 via 折线拐点）行驶，
    "管到点也管路"——起点投影到最近的路网边/节点，沿边走完整条路；
  - 遇动态障碍不绕行：前向扇区测到障碍立即停车，无限等待，
    持续畅通一段时间后沿原路继续；
  - 到站后原地转到节点/站点朝向（朝向对准）。

规划：Dijkstra（边权=折线长度），支持从路网中部上车（投影上车/就近节点上车）。
跟随：投影式纯跟踪（pure pursuit），20Hz 直接发布 /cmd_vel，带速度斜率限幅。
定位：复用 AMCL 的 map→odom TF，本节点只消费 map→base_footprint，不发布任何 TF。

安全性：
  - /scan 前向扇区 < stop_distance 即停；< slow_distance 减速；
  - /scan 数据缺失/过期同样停车等待（不盲走）；
  - map→base TF 丢失停车等待恢复；
  - 服务被取消或底盘 cmd_vel 有其它写者（手动模式）时由 Web 端负责互斥，
    本节点收到 /route/cancel 立即清零速度退出。
"""

import heapq
import json
import math
import os
import threading
import time

import rclpy
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from originbot_msgs.srv import CancelNavigation, NavigateToStation

from .robot_identity import default_identity_path, load_identity


def wrap_angle(angle):
    """归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PolylinePath:
    """展开后的全局路径：世界系点序列 + 累计弧长，供投影/取点。"""

    def __init__(self, points):
        self.points = [(float(x), float(y)) for x, y in points]
        self.cum = [0.0]
        for i in range(1, len(self.points)):
            step = math.hypot(
                self.points[i][0] - self.points[i - 1][0],
                self.points[i][1] - self.points[i - 1][1],
            )
            self.cum.append(self.cum[-1] + step)
        self.total = self.cum[-1] if len(self.cum) > 1 else 0.0

    def project(self, x, y):
        """返回 (沿路径弧长 s, 到路径的垂距 d)。"""
        best_s, best_d = 0.0, float("inf")
        px, py = self.points[0]
        for i in range(1, len(self.points)):
            qx, qy = self.points[i]
            dx, dy = qx - px, qy - py
            seg_len2 = dx * dx + dy * dy
            if seg_len2 < 1e-12:
                t = 0.0
            else:
                t = clamp(((x - px) * dx + (y - py) * dy) / seg_len2, 0.0, 1.0)
            cx, cy = px + t * dx, py + t * dy
            d = math.hypot(x - cx, y - cy)
            if d < best_d:
                best_d = d
                best_s = self.cum[i - 1] + t * math.sqrt(seg_len2)
            px, py = qx, qy
        return best_s, best_d

    def point_at(self, s):
        """返回弧长 s 处的点；s 超界时取端点。"""
        s = clamp(s, 0.0, self.total)
        for i in range(1, len(self.points)):
            if s <= self.cum[i] or i == len(self.points) - 1:
                seg = self.cum[i] - self.cum[i - 1]
                if seg < 1e-9:
                    return self.points[i]
                t = (s - self.cum[i - 1]) / seg
                ax, ay = self.points[i - 1]
                bx, by = self.points[i]
                return (ax + t * (bx - ax), ay + t * (by - ay))
        return self.points[-1]


class RouteGraphData:
    """routes.json 的内存表示 + 投影/规划所需索引。文件改动自动热加载。"""

    def __init__(self, path):
        self.path = path
        self.last_mtime = None
        self.nodes = {}      # name -> {"x","y","yaw","station"}
        self.edges = []      # [{"a","b","pts":[(x,y)...],"length"}]
        self.adjacency = {}  # name -> [(neighbor, edge_index)]
        self.reload()

    def reload_if_changed(self):
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return False
        if self.last_mtime is not None and mtime == self.last_mtime:
            return False
        self.reload()
        return True

    def reload(self):
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, ValueError):
            return
        nodes = {}
        for node in data.get("nodes", []):
            try:
                name = str(node["name"])
                nodes[name] = {
                    "x": float(node["x"]),
                    "y": float(node["y"]),
                    "yaw": float(node.get("yaw", 0.0)),
                    "station": str(node.get("station") or ""),
                }
            except (KeyError, TypeError, ValueError):
                continue
        edges = []
        for edge in data.get("edges", []):
            a, b = str(edge.get("a", "")), str(edge.get("b", ""))
            if a not in nodes or b not in nodes or a == b:
                continue
            pts = [(nodes[a]["x"], nodes[a]["y"])]
            for via in edge.get("via") or []:
                try:
                    pts.append((float(via[0]), float(via[1])))
                except (TypeError, ValueError, IndexError):
                    continue
            pts.append((nodes[b]["x"], nodes[b]["y"]))
            length = sum(
                math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
                for i in range(1, len(pts))
            )
            if length < 0.05:
                continue
            edges.append({"a": a, "b": b, "pts": pts, "length": length})
        self.nodes = nodes
        self.edges = edges
        self.adjacency = {name: [] for name in nodes}
        for idx, edge in enumerate(edges):
            self.adjacency[edge["a"]].append((edge["b"], idx))
            self.adjacency[edge["b"]].append((edge["a"], idx))
        try:
            self.last_mtime = os.path.getmtime(self.path)
        except OSError:
            self.last_mtime = None

    def find_goal_node(self, station_name):
        """按站点名或节点名找目标节点。"""
        for name, node in self.nodes.items():
            if node["station"] and node["station"] == station_name:
                return name, node
        if station_name in self.nodes:
            return station_name, self.nodes[station_name]
        return None, None

    def project_onto_edges(self, x, y):
        """返回 (edge_index, s_along_edge, 垂距)，取全路网最优。"""
        best = None
        for idx, edge in enumerate(self.edges):
            s, d = _project_polyline(edge["pts"], x, y)
            if best is None or d < best[2]:
                best = (idx, s, d)
        return best

    def entry_path_to(self, edge_index, s_along, toward_b):
        """从投影点沿边走到指定端点的途经点（含投影点，不含起点）。"""
        edge = self.edges[edge_index]
        pts = edge["pts"]
        cum = _polyline_cum(pts)
        segment = []
        if toward_b:
            # 投影点 → ... → b
            segment.append(_point_at(pts, cum, s_along))
            for i in range(len(pts)):
                if cum[i] > s_along + 1e-9:
                    segment.append(pts[i])
        else:
            # 投影点 → ... → a（逆向）
            segment.append(_point_at(pts, cum, s_along))
            for i in range(len(pts) - 2, -1, -1):
                if cum[i] < s_along - 1e-9:
                    segment.append(pts[i])
        return segment


def _polyline_cum(pts):
    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
    return cum


def _point_at(pts, cum, s):
    total = cum[-1] if cum else 0.0
    s = clamp(s, 0.0, total)
    for i in range(1, len(pts)):
        if s <= cum[i] or i == len(pts) - 1:
            seg = cum[i] - cum[i - 1]
            if seg < 1e-9:
                return pts[i]
            t = (s - cum[i - 1]) / seg
            return (pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]),
                    pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]))
    return pts[-1]


def _project_polyline(pts, x, y):
    """返回 (弧长 s, 垂距 d)。"""
    cum = _polyline_cum(pts)
    best_s, best_d = 0.0, float("inf")
    for i in range(1, len(pts)):
        ax, ay = pts[i - 1]
        bx, by = pts[i]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 < 1e-12 else clamp(((x - ax) * dx + (y - ay) * dy) / seg2, 0.0, 1.0)
        cx, cy = ax + t * dx, ay + t * dy
        d = math.hypot(x - cx, y - cy)
        if d < best_d:
            best_d = d
            best_s = cum[i - 1] + t * math.sqrt(seg2)
    return best_s, best_d


def plan_route(graph, pose, station_name, on_ramp_max, nearest_node_max):
    """从位姿 pose 规划到 station_name 对应节点的路网路径。

    返回 (points, start_desc, error, plan)：成功时 points 为世界系途经点序列
    （首点为当前位姿），error 为空，plan 为将依次经过的节点/边标记序列
    （形如 ["n5","n5->n2","n2","n2->n8","n8"]，供中心预占资源）；
    失败时 points 为 None，error 给出原因，plan 为 []。
    模块级纯函数，便于脱离 ROS 做单元测试。
    """
    if not graph.nodes:
        return None, "", "路网为空，请先在 Web 端绘制路网", []
    goal_node_name, _ = graph.find_goal_node(station_name)
    if goal_node_name is None:
        return None, "", f"路网中找不到目标节点 {station_name}", []

    x, y, _ = pose
    proj = graph.project_onto_edges(x, y)
    # (起始节点, 到该节点的代价, 途经点列表[不含当前位姿])
    sources = []
    start_desc = ""
    if proj is not None and proj[2] <= on_ramp_max:
        edge_idx, s_along, dist = proj
        edge = graph.edges[edge_idx]
        to_b = graph.entry_path_to(edge_idx, s_along, toward_b=True)
        to_a = graph.entry_path_to(edge_idx, s_along, toward_b=False)
        sources.append((edge["a"], s_along, to_a))
        sources.append((edge["b"], edge["length"] - s_along, to_b))
        start_desc = f"从边 {edge['a']}-{edge['b']} 上车（垂距 {dist:.2f} 米）"
    else:
        # 边投影太远：找最近节点直线上车
        name, node = min(
            graph.nodes.items(),
            key=lambda kv: math.hypot(kv[1]["x"] - x, kv[1]["y"] - y))
        dist = math.hypot(node["x"] - x, node["y"] - y)
        if dist > nearest_node_max:
            return None, "", (
                f"当前位置距路网过远（最近节点 {name} 距离 {dist:.2f} 米，"
                f"上限 {nearest_node_max:.2f} 米），请先手动把车挪到路网附近"), []
        sources.append((name, dist, [(node["x"], node["y"])]))
        start_desc = f"从最近节点 {name} 直线上车（{dist:.2f} 米）"

    # Dijkstra（无向图，多源）
    dist = {}
    prev_edge = {}   # node -> (prev_node, edge_idx) 用于还原边序列
    heap = []
    for node_name, cost, _ in sources:
        if cost < dist.get(node_name, float("inf")):
            dist[node_name] = cost
            heapq.heappush(heap, (cost, node_name))
    settled = set()
    while heap:
        d, u = heapq.heappop(heap)
        if u in settled:
            continue
        settled.add(u)
        if d > dist.get(u, float("inf")):
            continue
        for v, edge_idx in graph.adjacency.get(u, ()):
            nd = d + graph.edges[edge_idx]["length"]
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev_edge[v] = (u, edge_idx)
                heapq.heappush(heap, (nd, v))

    if goal_node_name not in dist:
        return None, "", f"路网不连通：{station_name} 无法到达", []

    # 还原节点序列对应的边序列
    edge_sequence = []
    cur = goal_node_name
    while cur in prev_edge:
        prev, edge_idx = prev_edge[cur]
        edge_sequence.append((prev, cur, edge_idx))
        cur = prev
    edge_sequence.reverse()

    # 拼接途经点：起点 entry + 各边折线（去掉重复连接点）。
    # edge_sequence 为空说明起点(源节点)就是目标节点，此时用目标节点的 entry。
    points = [(x, y)]
    entry_owner = edge_sequence[0][0] if edge_sequence else goal_node_name
    start_entry = next((pts for name, _, pts in sources if name == entry_owner), [])
    for p in start_entry:
        if math.hypot(p[0] - points[-1][0], p[1] - points[-1][1]) > 1e-6:
            points.append(p)
    for prev, _, edge_idx in edge_sequence:
        pts = graph.edges[edge_idx]["pts"]
        if prev != graph.edges[edge_idx]["a"]:
            pts = list(reversed(pts))
        for p in pts:
            if math.hypot(p[0] - points[-1][0], p[1] - points[-1][1]) > 1e-6:
                points.append(p)
    # 目标点兜底，保证 goal 一定在路径末端
    goal_node = graph.nodes[goal_node_name]
    if math.hypot(goal_node["x"] - points[-1][0], goal_node["y"] - points[-1][1]) > 1e-6:
        points.append((goal_node["x"], goal_node["y"]))

    if len(points) < 2:
        return None, "", "规划结果为空", []

    # 把还原出的边序列展开成 "节点 / a->b" 交替标记，供中心按序预占资源（§5.2 plan）
    plan = []
    if edge_sequence:
        plan.append(edge_sequence[0][0])
        for prev, cur, _idx in edge_sequence:
            plan.append(f"{prev}->{cur}")
            plan.append(cur)
    else:
        # 源节点即目标节点，无中间边
        plan.append(goal_node_name)
    return points, start_desc, "", plan


class RouteFollower(Node):
    def __init__(self):
        super().__init__("route_follower")

        # ---------- 参数 ----------
        self.declare_parameter("routes_file", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("max_vel_x", 0.22)
        self.declare_parameter("max_vel_theta", 0.8)
        self.declare_parameter("v_accel", 0.5)          # m/s^2 速度斜率
        self.declare_parameter("w_accel", 2.0)          # rad/s^2 角速度斜率
        self.declare_parameter("lookahead", 0.35)
        self.declare_parameter("rotate_threshold", 0.9)  # 航向偏差超过该值先原地转向
        self.declare_parameter("goal_xy_tolerance", 0.10)
        self.declare_parameter("goal_yaw_tolerance", 0.10)
        self.declare_parameter("align_yaw_speed", 0.6)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_half_fov_deg", 30.0)
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("slow_distance", 0.80)
        self.declare_parameter("resume_clear_sec", 1.5)   # 持续畅通该时长才恢复行驶
        self.declare_parameter("scan_stale_sec", 3.0)     # 激光数据过期则停车等待
        self.declare_parameter("on_ramp_max", 0.60)       # 距最近路网边的最大上车距离
        self.declare_parameter("nearest_node_max", 1.50)  # 无边上投影时距最近节点上限
        self.declare_parameter("end_decel", 0.5)          # 到点前减速 m/s^2
        # 控制权软门控：authority==debug（人工手动已接管）时本节点让出 /cmd_vel，
        # 保持任务但不运动；回到 controlled 自动续走。缺省按 controlled，兼容无
        # delivery_server/authority 发布者的旧部署（行为与今天一致）。
        self.declare_parameter("authority_topic", "/delivery/authority")
        self.declare_parameter("report_topic", "/route/report")
        self.declare_parameter("report_rate", 2.0)        # 结构化遥测上报频率 Hz

        routes_file = self.get_parameter("routes_file").value
        if not routes_file:
            config_dir = os.path.expanduser("~/.originbot_delivery/config")
            os.makedirs(config_dir, exist_ok=True)
            routes_file = os.path.join(config_dir, "routes.json")
        self.routes_file = routes_file
        self.graph = RouteGraphData(self.routes_file)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.control_period = 1.0 / float(self.get_parameter("control_rate").value)
        self.max_vel_x = float(self.get_parameter("max_vel_x").value)
        self.max_vel_theta = float(self.get_parameter("max_vel_theta").value)
        self.v_accel = float(self.get_parameter("v_accel").value)
        self.w_accel = float(self.get_parameter("w_accel").value)
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.rotate_threshold = float(self.get_parameter("rotate_threshold").value)
        self.goal_xy_tolerance = float(self.get_parameter("goal_xy_tolerance").value)
        self.goal_yaw_tolerance = float(self.get_parameter("goal_yaw_tolerance").value)
        self.align_yaw_speed = float(self.get_parameter("align_yaw_speed").value)
        self.scan_half_fov = math.radians(float(self.get_parameter("scan_half_fov_deg").value))
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.resume_clear_sec = float(self.get_parameter("resume_clear_sec").value)
        self.scan_stale_sec = float(self.get_parameter("scan_stale_sec").value)
        self.on_ramp_max = float(self.get_parameter("on_ramp_max").value)
        self.nearest_node_max = float(self.get_parameter("nearest_node_max").value)
        self.end_decel = float(self.get_parameter("end_decel").value)

        # ---------- 运行状态 ----------
        self.state_lock = threading.Lock()
        self.goal_active = False
        self.cancel_requested = False
        self.state = "IDLE"          # IDLE/ROTATE/DRIVE/ALIGN/DONE/FAILED/CANCELLED
        self.goal_name = ""
        self.goal_node_name = ""
        self.goal_yaw = 0.0
        self.path = None             # PolylinePath
        self.plan_pending = None     # (station_name,) TF 可用后补规划
        self.plan = []               # 本次任务将依次经过的节点/边标记（供中心预占）
        self.cur_v = 0.0
        self.cur_w = 0.0
        self.blocked = False
        self.blocked_reason = ""
        self.blocked_since = 0.0
        self.clear_since = 0.0
        self.authority = "controlled"   # 由 /delivery/authority 更新；debug=人工已接管
        self.yielded = False            # 是否正处于让出 /cmd_vel 状态
        self.last_status_text = ""
        self.last_status_time = 0.0
        self.last_progress_time = 0.0
        self.tf_fail_notified = False
        self.scan_fail_notified = False

        self.latest_scan = None
        self.latest_scan_time = 0.0

        # ---------- ROS 接口 ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.path_pub = self.create_publisher(PathMsg, "/route/path", path_qos)
        self.status_pub = self.create_publisher(String, "/route/status", 10)
        # 结构化上报（JSON），供未来车端 agent 直接转 MQTT 遥测；与 /route/status
        # 的人类可读文本并行，见 docs/fleet-dispatch-design.md §5.2
        self.report_pub = self.create_publisher(String, self.get_parameter("report_topic").value, 10)

        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self.scan_callback, 10)
        self.create_subscription(
            String, self.get_parameter("authority_topic").value,
            self.authority_callback, path_qos)
        self.create_service(
            NavigateToStation, "/route/navigate", self.navigate_callback)
        self.create_service(
            CancelNavigation, "/route/cancel", self.cancel_callback)

        self.timer = self.create_timer(self.control_period, self.tick)
        self.report_timer = self.create_timer(
            1.0 / max(0.2, float(self.get_parameter("report_rate").value)),
            self.publish_report)
        self.robot_id = load_identity(
            os.path.join(os.path.dirname(os.path.abspath(self.routes_file)),
                         "robot_identity.json")).get("robot_id", "")
        self.get_logger().info(
            f"路网跟随器就绪 robot_id={self.robot_id or '（未分配）'} "
            f"路网文件 {self.routes_file}"
            f"（节点 {len(self.graph.nodes)}，边 {len(self.graph.edges)}）")

    # ================= ROS 回调 =================

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = time.monotonic()

    def authority_callback(self, msg):
        value = (msg.data or "").strip().lower()
        if value in ("controlled", "debug") and value != self.authority:
            self.authority = value
            if value == "debug":
                self.get_logger().warning(
                    "控制权切到 debug（人工接管），路网跟随器让出 /cmd_vel")
            else:
                self.get_logger().info("控制权回到 controlled，路网跟随器可继续")

    def publish_report(self):
        """结构化上报（JSON）：mode/blocked/plan/占用，供车端 agent 转 MQTT 遥测。"""
        with self.state_lock:
            state = self.state
            goal_active = self.goal_active
            goal = self.goal_name
            blocked = self.blocked
            blocked_reason = self.blocked_reason
            waited = (time.monotonic() - self.blocked_since) if blocked else 0.0
            plan = list(self.plan)
            remaining = 0.0
            if goal_active and self.path is not None:
                pose = self._lookup_pose()
                if pose is not None:
                    s, _ = self.path.project(pose[0], pose[1])
                    remaining = max(0.0, self.path.total - s)
        report = {
            "robot_id": self.robot_id,
            "authority": self.authority,
            "yielded": self.yielded,
            "goal_active": goal_active,
            "state": state,
            "goal": goal,
            "remaining_m": round(remaining, 3),
            "plan": plan,
            "blocked": {
                "waiting": bool(blocked or self.yielded),
                "reason": "manual_takeover" if self.yielded else blocked_reason,
                "since_sec": round(waited, 1),
            },
        }
        msg = String()
        msg.data = json.dumps(report, ensure_ascii=False)
        self.report_pub.publish(msg)

    def navigate_callback(self, request, response):
        station_name = (request.station_name or "").strip()
        if not station_name:
            response.accepted = False
            response.message = "路网导航：站点名为空"
            return response
        self.graph.reload_if_changed()
        node_name, node = self.graph.find_goal_node(station_name)
        if node is None:
            response.accepted = False
            response.message = (
                f"路网导航：路网中没有站点 {station_name} 对应的节点，"
                "请先在 Web 端路网编辑器中添加并连线")
            return response
        with self.state_lock:
            self.cancel_requested = False
            self.goal_active = True
            self.state = "PLANNING"
            self.goal_name = station_name
            self.goal_node_name = node_name
            self.goal_yaw = float(node.get("yaw", 0.0))
            self.path = None
            self.plan = []
            self.plan_pending = True
            self.blocked = False
            self.blocked_reason = ""
            self.yielded = False
            self.cur_v = 0.0
            self.cur_w = 0.0
            self.tf_fail_notified = False
            self.scan_fail_notified = False
        self._publish_path_viz([])  # 清掉旧路径显示
        self._status(f"已开始前往 {station_name}（轨道路线，经节点 {node_name}）")
        response.accepted = True
        response.message = f"路网导航：已受理前往 {station_name}"
        return response

    def cancel_callback(self, request, response):
        with self.state_lock:
            was_active = self.goal_active
            self.cancel_requested = True
        self._publish_zero()
        if was_active:
            self._set_state("CANCELLED")
            self._status("导航已取消（轨道路线）")
            with self.state_lock:
                self.goal_active = False
                self.cancel_requested = False
            response.success = True
            response.message = "路网导航：已取消"
        else:
            response.success = True
            response.message = "路网导航：当前没有导航任务"
        return response

    # ================= 主循环 =================

    def tick(self):
        if not self.goal_active:
            self.yielded = False
            return
        if self.cancel_requested:
            self._finish("CANCELLED", "导航已取消（轨道路线）")
            return

        # 控制权软门控：人工接管（debug）时让出 /cmd_vel，保持任务但不运动。
        # 回到 controlled 后自动从当前状态续走（不重复起步转向）。
        if self.authority == "debug":
            if not self.yielded:
                self.yielded = True
                self._status("路网导航：人工已接管（debug），暂停沿路网行驶，让出速度控制权")
            self._publish_zero()
            return
        if self.yielded:
            self.yielded = False
            self.blocked = False
            self.clear_since = 0.0
            self._status(f"路网导航：控制权回到受控，继续前往 {self.goal_name}")

        pose = self._lookup_pose()
        if pose is None:
            self._publish_zero()
            if not self.tf_fail_notified:
                self.tf_fail_notified = True
                self._status("路网导航：未获取到定位 TF，停车等待定位恢复")
            return
        self.tf_fail_notified = False

        # 延迟规划：受理时 TF 可能尚未就绪
        if self.plan_pending:
            ok, message = self._plan(pose)
            with self.state_lock:
                self.plan_pending = False
            if not ok:
                self._finish("FAILED", f"导航失败：{message}")
                return
            self._set_state("ROTATE")
            self._publish_path_viz(self.path.points)
            self._status(
                f"路网导航：路径已规划（{self.path.total:.2f} 米，"
                f"{len(self.path.points) - 1} 段），开始沿路网行驶")

        # 激光数据检查：缺失/过期不盲走
        front_distance = self._front_obstacle_distance()
        blocked_now, scan_ok = self._evaluate_block(front_distance)

        if not scan_ok:
            self._publish_zero()
            if not self.scan_fail_notified:
                self.scan_fail_notified = True
                self._status("路网导航：激光数据缺失，停车等待雷达恢复")
            return
        self.scan_fail_notified = False

        now = time.monotonic()
        state = self.state
        # 起步转向（ROTATE）与到站对准（ALIGN）阶段放行原地旋转：
        # 掉头/到站贴墙场景下车头贴近障碍时，停车等待将永远无法完成对准；
        # 原地自转所需净空远小于行驶，对准完成进入 DRIVE 后恢复完整避障逻辑。
        if blocked_now and state not in ("ROTATE", "ALIGN"):
            if not self.blocked:
                self.blocked = True
                self.blocked_since = now
                self.blocked_reason = "obstacle"
                self._status(
                    f"路网导航：前方 {front_distance:.2f} 米处有障碍，停车等待消除（无限等待）")
            elif now - self.last_status_time >= 15.0:
                self._status(
                    f"路网导航：仍在等待障碍消除（已等待 {now - self.blocked_since:.0f} 秒）")
            self.clear_since = 0.0
            self._ramp_and_publish(0.0, 0.0)
            return
        if blocked_now and state == "ROTATE":
            # 起步掉头：忽略前向障碍继续原地转向，仅限频提示
            if now - self.last_status_time >= 15.0:
                self._status(
                    f"路网导航：起步转向阶段前方 {front_distance:.2f} 米处有物体，"
                    "继续原地转向对准路径（不视为阻挡）")

        if self.blocked and state not in ("ROTATE", "ALIGN"):
            if self.clear_since == 0.0:
                self.clear_since = now
            elif now - self.clear_since >= self.resume_clear_sec:
                self.blocked = False
                self.blocked_reason = ""
                self.clear_since = 0.0
                self._status(f"路网导航：障碍已消除，继续前往 {self.goal_name}")
            else:
                # 畅通时间不足，保持暂停防抖
                self._ramp_and_publish(0.0, 0.0)
                return
        else:
            self.clear_since = 0.0

        # ----- 状态机推进 -----
        x, y, yaw = pose

        if state == "ROTATE":
            # 起步前先对准第一段路径方向
            s_robot, _ = self.path.project(x, y)
            target = self.path.point_at(s_robot + min(self.lookahead, self.path.total))
            alpha = wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)
            if abs(alpha) < 0.35 or self.path.total < 0.05:
                self._set_state("DRIVE")
                return
            self._ramp_and_publish(0.0, clamp(2.0 * alpha, -self.max_vel_theta, self.max_vel_theta))
            return

        if state == "DRIVE":
            s_robot, _ = self.path.project(x, y)
            remaining = self.path.total - s_robot
            if remaining <= self.goal_xy_tolerance:
                self._set_state("ALIGN")
                self._status("路网导航：已到点，开始朝向对准")
                return
            # 目标点：沿路径向前 lookahead（到终点时收缩）
            ld = min(self.lookahead, max(0.08, remaining))
            target = self.path.point_at(s_robot + ld)
            alpha = wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)

            if abs(alpha) > self.rotate_threshold and remaining > self.lookahead:
                # 大偏差先原地转向
                self._ramp_and_publish(
                    0.0, clamp(2.0 * alpha, -self.max_vel_theta, self.max_vel_theta))
            else:
                v_target = self.max_vel_x
                # 前方障碍减速区
                if front_distance < self.slow_distance:
                    scale = clamp(
                        (front_distance - self.stop_distance)
                        / max(0.05, self.slow_distance - self.stop_distance),
                        0.0, 1.0)
                    # 减速区速度下限：窄道/贴墙时不得降为"爬行"。
                    # 原公式在 front→stop_distance 时 v→0，实际表现为 2~3cm/s
                    # 蠕动，操作者感受为"一卡一卡"。硬停车由 stop_distance 的
                    # BLOCKED 分支负责，这里保底 35% 巡航速度保证走动连贯。
                    v_target *= max(0.35, scale)
                # 到点前减速
                v_target = min(
                    v_target,
                    math.sqrt(2.0 * self.end_decel * max(0.0, remaining - 0.02)) + 0.02)
                # 转弯降速（保底 0.5：航向微抖不再把速度砍到 30%，
                # 避免直线段速度忽高忽低形成"抽动"感）
                v_target *= max(0.5, math.cos(alpha))
                w_target = v_target * 2.0 * math.sin(alpha) / max(0.05, ld)
                self._ramp_and_publish(
                    clamp(v_target, 0.0, self.max_vel_x),
                    clamp(w_target, -self.max_vel_theta, self.max_vel_theta))

            if now - self.last_progress_time >= 2.0:
                self.last_progress_time = now
                self._status(f"路网导航：剩余 {max(0.0, remaining):.2f} 米")
            return

        if state == "ALIGN":
            dyaw = wrap_angle(self.goal_yaw - yaw)
            if abs(dyaw) <= self.goal_yaw_tolerance:
                self._finish("DONE", f"导航完成：已到达 {self.goal_name} 并完成朝向对准")
                return
            self._ramp_and_publish(
                0.0, clamp(1.5 * dyaw, -self.align_yaw_speed, self.align_yaw_speed))
            return

        # 未知状态兜底：停车
        self._publish_zero()

    # ================= 规划 =================

    def _plan(self, pose):
        """从当前位姿规划到目标节点的路网路径。成功后填充 self.path / self.plan。"""
        self.graph.reload_if_changed()
        points, start_desc, error, plan = plan_route(
            self.graph, pose, self.goal_name,
            self.on_ramp_max, self.nearest_node_max)
        if points is None:
            return False, error
        self.path = PolylinePath(points)
        self.plan = plan
        self.get_logger().info(
            f"路网导航规划完成：{self.goal_name}，{self.path.total:.2f} 米，{start_desc}")
        return True, ""

    # ================= 感知/执行辅助 =================

    def _lookup_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        t = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return (t.x, t.y, yaw)

    def _front_obstacle_distance(self):
        """前向扇区内最近障碍距离；扇区畅通返回 inf；无帧数据返回 None。"""
        scan = self.latest_scan
        if scan is None:
            return None
        best = float("inf")
        ranges = scan.ranges or []
        angle_min = scan.angle_min
        inc = scan.angle_increment or 0.0
        for i, r in enumerate(ranges):
            if not (r > 0.05) or not (r < 30.0) or math.isnan(r):
                continue
            a = wrap_angle(angle_min + i * inc)
            if abs(a) <= self.scan_half_fov:
                if r < best:
                    best = r
        return best

    def _evaluate_block(self, front_distance):
        """返回 (是否被阻挡, 激光数据是否可用)。"""
        now = time.monotonic()
        if front_distance is None or now - self.latest_scan_time > self.scan_stale_sec:
            return True, False
        return front_distance <= self.stop_distance, True

    def _ramp_and_publish(self, v_target, w_target):
        """带加速度斜率的速度发布，避免底盘电流冲击。"""
        dv = clamp(v_target - self.cur_v, -self.v_accel * self.control_period,
                   self.v_accel * self.control_period)
        dw = clamp(w_target - self.cur_w, -self.w_accel * self.control_period,
                   self.w_accel * self.control_period)
        self.cur_v += dv
        self.cur_w += dw
        if abs(self.cur_v) < 1e-4:
            self.cur_v = 0.0
        if abs(self.cur_w) < 1e-4:
            self.cur_w = 0.0
        twist = Twist()
        twist.linear.x = self.cur_v
        twist.angular.z = self.cur_w
        self.cmd_pub.publish(twist)

    def _publish_zero(self):
        self.cur_v = 0.0
        self.cur_w = 0.0
        twist = Twist()
        self.cmd_pub.publish(twist)

    def _set_state(self, state):
        with self.state_lock:
            self.state = state

    def _finish(self, state, message):
        self._publish_zero()
        self._set_state(state)
        self._status(message)
        with self.state_lock:
            self.goal_active = False
            self.cancel_requested = False
            self.path = None
            self.plan = []
            self.blocked = False
            self.blocked_reason = ""
            self.yielded = False

    def _status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.last_status_time = time.monotonic()
        self.get_logger().info(text)

    def _publish_path_viz(self, points):
        path_msg = PathMsg()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
