#!/usr/bin/env python3

import json
import fcntl
import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from originbot_msgs.srv import CancelNavigation
from originbot_msgs.srv import ControlSystem
from originbot_msgs.srv import MapFile
from originbot_msgs.srv import MarkerEntry
from originbot_msgs.srv import NavigateToStation
from originbot_msgs.srv import ParamsEntry
from originbot_msgs.srv import RouteGraph
from originbot_msgs.srv import SaveMap
from originbot_msgs.srv import SetAuthority
from originbot_msgs.srv import SetPose
from originbot_msgs.srv import StationEntry

from .robot_identity import AUTHORITY_VALUES, load_identity
from .robot_params import ParamsStore


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_multiply(a, b):
    q = Quaternion()
    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    return q


def rotate_vector_by_quaternion(q, v):
    cx = q.y * v.z - q.z * v.y
    cy = q.z * v.x - q.x * v.z
    cz = q.x * v.y - q.y * v.x
    dx = q.y * cz - q.z * cy
    dy = q.z * cx - q.x * cz
    dz = q.x * cy - q.y * cx
    return (
        v.x + 2.0 * (q.w * cx + dx),
        v.y + 2.0 * (q.w * cy + dy),
        v.z + 2.0 * (q.w * cz + dz),
    )


class JsonStore:
    def __init__(self, path, default):
        self.path = path
        self.default = default
        self.lock = threading.Lock()
        self.load()

    def load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                self.data = json.load(handle)
        except (OSError, ValueError):
            self.data = json.loads(json.dumps(self.default))
        try:
            self.last_mtime = os.path.getmtime(self.path)
        except OSError:
            self.last_mtime = None

    def reload_if_changed(self):
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return False
        if self.last_mtime is not None and mtime == self.last_mtime:
            return False
        self.load()
        return True

    def save(self):
        tmp_path = self.path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(self.data, handle, indent=2, ensure_ascii=False)
        os.replace(tmp_path, self.path)


class StationStore(JsonStore):
    def list_stations(self):
        names = []
        xs = []
        ys = []
        yaws = []
        for station in self.data.get("stations", []):
            names.append(station["name"])
            xs.append(float(station["x"]))
            ys.append(float(station["y"]))
            yaws.append(float(station.get("yaw", 0.0)))
        return names, xs, ys, yaws

    def find(self, name):
        for station in self.data.get("stations", []):
            if station["name"] == name:
                return station
        return None

    def upsert(self, name, x, y, yaw):
        station = self.find(name)
        if station is None:
            station = {"name": name, "x": x, "y": y, "yaw": yaw}
            self.data.setdefault("stations", []).append(station)
        else:
            station.update({"x": x, "y": y, "yaw": yaw})
        self.save()

    def delete(self, name):
        stations = self.data.get("stations", [])
        self.data["stations"] = [s for s in stations if s["name"] != name]
        self.save()

    def clear(self):
        self.data["stations"] = []
        self.save()


class MarkerStore(JsonStore):
    def list_markers(self):
        self.reload_if_changed()
        ids = []
        sizes = []
        xs = []
        ys = []
        yaws = []
        for marker in self.data.get("markers", []):
            ids.append(int(marker["id"]))
            sizes.append(float(marker.get("size", self.data.get("marker_size", 0.07))))
            xs.append(float(marker["x"]))
            ys.append(float(marker["y"]))
            yaws.append(float(marker.get("yaw", 0.0)))
        return ids, sizes, xs, ys, yaws

    def upsert(self, marker_id, marker_size, x, y, yaw):
        marker = self.find(marker_id)
        if marker is None:
            marker = {
                "id": int(marker_id),
                "size": float(marker_size),
                "x": x,
                "y": y,
                "yaw": yaw,
            }
            self.data.setdefault("markers", []).append(marker)
        else:
            marker.update(
                {"size": float(marker_size), "x": x, "y": y, "yaw": yaw}
            )
        self.save()

    def delete(self, marker_id):
        self.data["markers"] = [
            m for m in self.data.get("markers", []) if int(m["id"]) != int(marker_id)
        ]
        self.save()

    def find(self, marker_id):
        for marker in self.data.get("markers", []):
            if int(marker["id"]) == int(marker_id):
                return marker
        return None

    def clear(self):
        self.data["markers"] = []
        self.save()


class RouteStore(JsonStore):
    """路网（拓扑 Graph）存储：节点 + 边，供 route_follower 与 Web 画布编辑器共用。

    与 StationStore 增量 CRUD 不同，路网编辑在画布上成批进行，
    因此只提供整张图的读/写（save 前做结构与引用校验）。
    """

    def list_graph(self):
        self.reload_if_changed()
        return json.loads(json.dumps(self.data))

    def clear(self):
        self.data = {"nodes": [], "edges": []}
        self.save()

    def save_graph(self, payload):
        """校验并保存整张路网，返回 (是否成功, 消息)。"""
        if not isinstance(payload, dict):
            return False, "路网数据必须是对象"
        raw_nodes = payload.get("nodes")
        raw_edges = payload.get("edges")
        if not isinstance(raw_nodes, list) or not isinstance(raw_edges, list):
            return False, "路网必须包含 nodes 与 edges 数组"
        nodes = []
        names = set()
        for node in raw_nodes:
            if not isinstance(node, dict):
                return False, "节点必须是对象"
            name = str(node.get("name", "")).strip()
            if not name:
                return False, "存在未命名的路网节点"
            if name in names:
                return False, f"节点名重复：{name}"
            try:
                entry = {
                    "name": name,
                    "x": round(float(node.get("x")), 4),
                    "y": round(float(node.get("y")), 4),
                    "yaw": round(float(node.get("yaw", 0.0)), 4),
                }
            except (TypeError, ValueError):
                return False, f"节点 {name} 的坐标不是数字"
            station = str(node.get("station") or "").strip()
            if station:
                entry["station"] = station
            names.add(name)
            nodes.append(entry)
        edges = []
        seen = set()
        for edge in raw_edges:
            if not isinstance(edge, dict):
                return False, "边必须是对象"
            a = str(edge.get("a", ""))
            b = str(edge.get("b", ""))
            if a not in names or b not in names:
                return False, f"边 {a}-{b} 引用了不存在的节点"
            if a == b:
                return False, f"边 {a}-{b} 的两端不能是同一节点"
            key = tuple(sorted((a, b)))
            if key in seen:
                return False, f"节点 {a} 与 {b} 之间重复连线"
            seen.add(key)
            entry = {"a": a, "b": b}
            via = edge.get("via")
            if via:
                clean_via = []
                if not isinstance(via, list):
                    return False, f"边 {a}-{b} 的 via 必须是数组"
                for point in via:
                    try:
                        clean_via.append([round(float(point[0]), 4), round(float(point[1]), 4)])
                    except (TypeError, ValueError, IndexError):
                        return False, f"边 {a}-{b} 的 via 拐点格式错误"
                if clean_via:
                    entry["via"] = clean_via
            edges.append(entry)
        self.data = {"nodes": nodes, "edges": edges}
        self.save()
        return True, f"路网已保存（节点 {len(nodes)}，边 {len(edges)}）"


class SubprocessController:
    SUBSYSTEM_LABELS = {
        "base": "底盘",
        "localization": "定位",
        "aruco": "ArUco 定位",
        "mapping": "建图",
        "navigation": "导航",
        "route": "路网跟随",
        "map_viewer": "地图查看",
    }
    SUBSYSTEM_NODES = {
        "base": ("originbot_base", "STL06N"),
        "localization": ("ekf_filter_node",),
        "aruco": ("aruco_localization",),
        "mapping": (
            "cartographer_node",
            "cartographer_occupancy_grid_node",
            "slam_toolbox",
        ),
        "navigation": (
            "map_server",
            "amcl",
            "planner_server",
            "controller_server",
            "smoother_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "lifecycle_manager_navigation",
        ),
        "route": ("route_follower",),
        "map_viewer": ("map_viewer",),
    }
    # 每个子系统的进程命令行特征，用于清理 delivery_server 重启时留下的孤儿进程
    SUBSYSTEM_PROCESS_PATTERNS = {
        "base": ("originbot_base", "STL06N", "hobot_usb_cam"),
        "localization": ("ekf_filter_node",),
        "aruco": ("aruco_localization",),
        "mapping": (
            "cartographer_node",
            "cartographer_occupancy_grid_node",
            "async_slam_toolbox_node",
        ),
        "navigation": ("nav2_container",),
        "route": ("route_follower",),
        "map_viewer": ("map_viewer",),
    }

    def __init__(self, node, log_dir):
        self.node = node
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.processes = {}
        self.navigation_map = None
        self.map_viewer_file = None
        self.mapping_engine = "cartographer"
        # Web 端参数配置页生成的配置文件路径（slam_toolbox/nav2/cartographer
        # 通过 launch 参数注入；lidar/imu/camera/ekf 由各 launch 文件自行读取）
        self.param_paths = {}
        self.commands = {
            # 2026-09-02: 摄像头驱动是 CPU/内存压力源(rosbridge 推流+相机采集)，
            # 导航卡顿期间暂时关闭；恢复相机把 use_camera:=false 改回 true 即可
            "base": "ros2 launch originbot_bringup originbot.launch.py use_camera:=false",
            "localization": "ros2 launch originbot_navigation odom_ekf.launch.py",
            "aruco": "ros2 launch originbot_delivery aruco.launch.py",
            "mapping": "ros2 launch originbot_navigation cartographer.launch.py",
            "navigation": "ros2 launch originbot_navigation nav_bringup.launch.py",
            "route": "ros2 launch originbot_delivery route.launch.py",
            "map_viewer": "ros2 launch originbot_delivery map_viewer.launch.py",
        }

    def set_param_paths(self, paths):
        self.param_paths = dict(paths or {})

    def set_navigation_map(self, map_yaml_path):
        self.navigation_map = str(map_yaml_path)

    def set_map_viewer_file(self, map_yaml_path):
        self.map_viewer_file = str(map_yaml_path)

    def set_mapping_engine(self, mapping_engine):
        mapping_engine = mapping_engine.strip().lower()
        if mapping_engine == "slam_toolbox":
            self.mapping_engine = mapping_engine
            self.commands["mapping"] = (
                "ros2 launch originbot_navigation slam_toolbox.launch.py"
            )
        else:
            self.mapping_engine = "cartographer"
            self.commands["mapping"] = (
                "ros2 launch originbot_navigation cartographer.launch.py"
            )
        return self.mapping_engine

    def is_running(self, subsystem):
        process = self.processes.get(subsystem)
        return process is not None and process.poll() is None

    def start(self, subsystem):
        if subsystem not in self.commands:
            return False, f"未知子系统：{subsystem}"
        if self.is_running(subsystem):
            label = self.SUBSYSTEM_LABELS.get(subsystem, subsystem)
            return True, f"{label}已在运行"
        stopped, _ = self.stop(subsystem)
        label = self.SUBSYSTEM_LABELS.get(subsystem, subsystem)
        # 仅当本服务没有管理中的旧进程时才做节点冲突检测；
        # 刚被 stop() 停掉的进程其节点注销有 DDS 传播延迟，
        # 若此刻仍检查会把旧节点误判为“已有实例”而拒绝启动。
        if not stopped:
            conflicts = self.wait_for_conflicts(subsystem)
            # 节点名冲突需进程级核实：DDS 图中的残留节点名可能是刚停掉
            # 进程的幽灵（注销传播有延迟），只有进程确实活着才是真冲突
            if conflicts and self.has_live_process(subsystem):
                return False, (
                    f"检测到已有{label}在运行(节点：{', '.join(conflicts)})。"
                    "若由其它终端启动，请先停止该终端，避免重复实例导致数据冲突"
                )
            if conflicts:
                self.node.get_logger().info(
                    f"{label} 的节点 {', '.join(conflicts)} 仍在 DDS 图中，"
                    "但无对应进程，判定为注销延迟的幽灵节点，继续启动")
        command = self.commands[subsystem]
        if subsystem == "navigation" and self.navigation_map:
            command = f"{command} map:={shlex.quote(self.navigation_map)}"
        elif subsystem == "map_viewer" and self.map_viewer_file:
            command = f"{command} map_file:={shlex.quote(self.map_viewer_file)}"
        # 注入 Web 端参数配置页生成的配置文件（保存参数后重启对应子系统即生效，
        # 开机自启动同样经过这里，因此自动加载）
        param_paths = self.param_paths or {}
        if subsystem == "navigation" and param_paths.get("nav2_params_file"):
            command = (
                f"{command} params_file:={shlex.quote(param_paths['nav2_params_file'])}"
            )
        elif subsystem == "mapping":
            if self.mapping_engine == "slam_toolbox" and param_paths.get("slam_params_file"):
                command = (
                    f"{command} slam_params_file:="
                    f"{shlex.quote(param_paths['slam_params_file'])}"
                )
            elif param_paths.get("cartographer_config_dir"):
                command = (
                    f"{command} cartographer_config_dir:="
                    f"{shlex.quote(param_paths['cartographer_config_dir'])}"
                )
        log_path = self.log_dir / f"{subsystem}.log"
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            with open(log_path, "ab") as log_file:
                process = subprocess.Popen(
                    shlex.split(command),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
        except OSError as exc:
            return False, f"{label}启动失败：{exc}"
        self.processes[subsystem] = process
        self.node.get_logger().info(
            f"已启动{label} (pid {process.pid})"
        )
        return True, f"{label}已启动"

    def find_conflicting_nodes(self, subsystem):
        expected = self.SUBSYSTEM_NODES.get(subsystem, ())
        if not expected:
            return []
        try:
            live = {name.lstrip("/") for name in self.node.get_node_names()}
        except Exception:
            return []
        return sorted(set(expected) & live)

    def has_live_process(self, subsystem):
        """节点名仍在 DDS 图中不代表进程还活着：进程死后节点注销有传播
        延迟（可超过 10s），"先停止、马上启动"会把幽灵节点误判为重复
        实例。这里按进程名核实，只有真有活进程才算数。"""
        patterns = self.SUBSYSTEM_PROCESS_PATTERNS.get(subsystem, ())
        for pattern in patterns:
            try:
                result = subprocess.run(
                    ["pgrep", "-f", pattern],
                    capture_output=True, timeout=5,
                )
                if result.returncode == 0:
                    return True
            except Exception:
                continue
        return False

    def wait_for_conflicts(self, subsystem, attempts=4, delay=1.0):
        conflicts = self.find_conflicting_nodes(subsystem)
        for _ in range(attempts - 1):
            if not conflicts:
                break
            time.sleep(delay)
            conflicts = self.find_conflicting_nodes(subsystem)
        return conflicts

    def _kill_orphaned(self, subsystem):
        """按进程名清理该子系统残留的孤儿进程（delivery_server 被强杀时留下）。

        这些孤儿不在 self.processes 里，正常 stop() 停不掉；只有显式按名字 SIGKILL。
        """
        patterns = self.SUBSYSTEM_PROCESS_PATTERNS.get(subsystem, ())
        if not patterns or not self.find_conflicting_nodes(subsystem):
            return False
        killed = False
        for pattern in patterns:
            try:
                result = subprocess.run(
                    ["pkill", "-9", "-f", pattern],
                    capture_output=True, text=True, timeout=5,
                )
                if result.returncode == 0:
                    killed = True
                    self.node.get_logger().warning(
                        f"已清理 {subsystem} 的孤儿进程: {pattern}")
            except Exception:
                pass
        if killed:
            for _ in range(20):
                if not self.find_conflicting_nodes(subsystem):
                    break
                time.sleep(0.5)
        return killed

    def stop(self, subsystem):
        process = self.processes.pop(subsystem, None)
        label = self.SUBSYSTEM_LABELS.get(subsystem, subsystem)
        if process is None or process.poll() is not None:
            # 无托管进程，但可能残留了孤儿（上次服务重启留下的），尝试按名字清理
            if self._kill_orphaned(subsystem):
                return True, f"{label}已停止（已清理残留进程）"
            return False, f"{label}当前未运行"
        label = self.SUBSYSTEM_LABELS.get(subsystem, subsystem)
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=8)
        except (ProcessLookupError, PermissionError):
            pass
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3)
            except Exception:
                pass
        # Nav2 的 component_container 等 C++ 节点对 SIGINT/SIGTERM 不敏感，
        # 若仍未退出则 SIGKILL 兜底，避免留下孤儿进程阻塞下次启动。
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=2)
            except Exception:
                pass
        # 等待该子系统的节点从 ROS 图中注销，避免 start() 把刚停掉的旧节点
        # 误判为“已有实例”而拒绝启动（曾导致定位重启失败、导航不可用）。
        for _ in range(20):
            if not self.find_conflicting_nodes(subsystem):
                break
            time.sleep(0.5)
        return True, f"{label}已停止"

    def stop_all(self):
        for subsystem in list(self.processes):
            self.stop(subsystem)


class DeliveryServer(Node):
    def __init__(self):
        super().__init__("delivery_server")
        self.declare_parameter("stations_file", "")
        self.declare_parameter("markers_file", "")
        self.declare_parameter("map_directory", os.path.expanduser("~/originbot_maps"))

        pkg_share = self.get_package_share_directory()
        config_dir = Path.home() / ".originbot_delivery" / "config"
        config_dir.mkdir(parents=True, exist_ok=True)
        stations_file = self.get_parameter("stations_file").value
        markers_file = self.get_parameter("markers_file").value
        if not stations_file:
            stations_file = str(config_dir / "stations.json")
        if not markers_file:
            markers_file = str(config_dir / "markers.json")
        for filename in ("stations.json", "markers.json"):
            target = config_dir / filename
            source = Path(pkg_share) / "config" / filename
            if not target.exists() and source.exists():
                shutil.copyfile(source, target)
        self.map_directory = Path(
            self.get_parameter("map_directory").value
        ).expanduser()
        self.map_directory.mkdir(parents=True, exist_ok=True)
        self.state_file = config_dir / "state.json"
        self.state = self.load_state()

        self.station_store = StationStore(stations_file, {"stations": []})
        self.marker_store = MarkerStore(
            markers_file,
            {"dictionary": "DICT_6X6_1000", "marker_size": 0.07, "markers": []},
        )
        self.route_store = RouteStore(
            str(config_dir / "routes.json"), {"nodes": [], "edges": []}
        )
        # 车队身份 + 控制权（阶段 0）。per-robot 配置与"全车队共用"的配置束
        # （map/stations/routes）刻意分开：这里放"这辆车是谁、回桩在哪、默认要什么控制权"。
        self.identity = load_identity(config_dir / "robot_identity.json")
        self.robot_id = self.identity.get("robot_id", "")
        self.authority = self.identity.get("default_authority", "debug")
        self.authority_lock = threading.Lock()
        # 当前导航任务使用的模式：free=Nav2 自由避障 / route=轨道路网。
        # 用于取消导航时分发到正确的执行器，完成/取消后置回 None。
        self.active_nav_mode = None
        self.subprocess_controller = SubprocessController(
            self, self.map_directory / "logs"
        )
        self.subprocess_controller.set_mapping_engine(
            self.state.get("mapping_algorithm", "cartographer")
        )
        # 参数配置：加载 robot_params.json 并物化为各子系统的配置文件，
        # 生成失败不阻塞启动（各子系统会退回包内默认配置）
        self.params_store = ParamsStore(config_dir)
        try:
            self.params_store.write_generated()
        except Exception as exc:
            self.get_logger().error(f"生成参数配置文件失败，子系统将使用默认配置: {exc}")
        self.subprocess_controller.set_param_paths(self.params_store.generated_paths())

        self.status_pub = self.create_publisher(String, "/delivery/status", 10)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_cache = None
        self.map_cache_lock = threading.Lock()
        self.map_pub = self.create_publisher(OccupancyGrid, "map", map_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self.map_subscription_callback, map_qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # AMCL 用 TRANSIENT_LOCAL 订阅 /initialpose，发布端必须用相同 durability，
        # 否则报 "incompatible QoS DURABILITY"，初始位姿送不到 AMCL。
        initial_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", initial_pose_qos
        )
        # 控制权单一真值：latched(TRANSIENT_LOCAL) 发布，route_follower 与 Web 端
        # 据此互斥 /cmd_vel 写者；latched 让新连接的订阅端立刻拿到当前值。
        authority_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.authority_pub = self.create_publisher(
            String, "/delivery/authority", authority_qos
        )
        self.nav_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav_goal_handle = None
        self.nav_lock = threading.Lock()
        # 路网跟随器状态转发：route_follower 的提示语已带 "路网导航" 前缀，
        # 直接转发到 /delivery/status 即可在 Web 状态栏显示；
        # 终态关键词与 Nav2 模式一致，复用前端的导航目标清理逻辑。
        self.route_status_sub = self.create_subscription(
            String, "/route/status", self.route_status_callback, 10
        )
        self.route_nav_client = self.create_client(
            NavigateToStation, "/route/navigate"
        )
        self.route_cancel_client = self.create_client(
            CancelNavigation, "/route/cancel"
        )
        self._nav_feedback_lock = threading.Lock()
        self._nav_feedback_last_time = 0.0
        self._nav_feedback_last_distance = 0.0
        self.pending_pose_lock = threading.Lock()
        self.pending_user_pose = None
        self.pending_map_odom = None
        self.pending_boot_pose = None
        self._initial_pose_guard = threading.Lock()
        # ArUco 视觉定位开关：默认关闭（无 ArUco 码时空转占约 2 核 CPU）
        # 需要时通过环境变量 ENABLE_ARUCO=1 开启
        self.enable_aruco = os.environ.get("ENABLE_ARUCO", "0") == "1"
        # 注意：不在这里从 state.json 恢复 pending_map_odom。
        # 建图坐标系(map→odom)只在「同一会话内、底盘未重启」时有效；
        # 一旦 delivery 服务重启，底盘(originbot_base)也会重启、odom 归零，
        # 旧 map→odom 就会失效，误用会导致 AMCL 初始位姿错位、导航失败。
        # 因此服务重启后必须由用户在 Web 上手动设定位姿。

        self.create_service(
            ControlSystem, "/delivery/control_system", self.control_system_callback
        )
        self.create_service(
            StationEntry, "/delivery/station", self.station_callback
        )
        self.create_service(
            MarkerEntry, "/delivery/marker", self.marker_callback
        )
        self.create_service(MapFile, "/delivery/map_file", self.map_file_callback)
        self.create_service(
            NavigateToStation,
            "/delivery/navigate_to_station",
            self.navigate_callback,
        )
        self.create_service(
            CancelNavigation,
            "/delivery/cancel_navigation",
            self.cancel_navigation_callback,
        )
        self.create_service(RouteGraph, "/delivery/route", self.route_callback)
        self.create_service(SaveMap, "/delivery/save_map", self.save_map_callback)
        self.create_service(SetPose, "/delivery/set_pose", self.set_pose_callback)
        self.create_service(
            SetAuthority, "/delivery/set_authority", self.set_authority_callback
        )
        self.create_service(Trigger, "/delivery/shutdown", self.shutdown_callback)
        self.create_service(ParamsEntry, "/delivery/params", self.params_callback)

        # 上电即广播当前控制权（latched，晚起的 route_follower / Web 也会立刻收到）。
        self._publish_authority(self.authority)
        boot_pose = self.identity.get("boot_pose")
        if boot_pose:
            self.pending_boot_pose = dict(boot_pose)
        self.get_logger().info(
            f"送餐任务服务已就绪 robot_id={self.robot_id or '（未分配）'} "
            f"控制权={self.authority} "
            f"boot_pose={'已配置' if boot_pose else '未配置'}")
        if not boot_pose and self.identity.get("robot_id"):
            self.get_logger().warning(
                "本车未配置 boot_pose：冷启动无自动初始位姿，"
                "需在 Web 设定位姿或用 set_robot_id --capture-boot-pose 记录回桩位")

    def load_state(self):
        try:
            with open(self.state_file, "r", encoding="utf-8") as handle:
                return json.load(handle)
        except (OSError, ValueError):
            return {}

    def save_state(self):
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.state_file.with_suffix(".json.tmp")
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(self.state, handle, indent=2, ensure_ascii=False)
        os.replace(tmp_path, self.state_file)

    def get_default_map_yaml(self):
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory("originbot_navigation"),
            "maps",
            "my_map.yaml",
        )

    def get_package_share_directory(self):
        from ament_index_python.packages import get_package_share_directory

        return get_package_share_directory("originbot_delivery")

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _start_navigation_stack(self):
        """确保导航栈（Nav2/AMCL）与路网跟随器已启动，返回 (ok, message)。

        轨道路线模式同样依赖 AMCL 的 map→odom 定位，因此两种模式共用这个栈；
        区别只在目标发给谁：free→bt_navigator，route→/route/navigate。
        Nav2 空闲时不会写 /cmd_vel，路网跟随器发速度是安全的。
        """
        if self.subprocess_controller.is_running("navigation"):
            self._ensure_route_follower()
            return True, "导航已在运行"
        self.subprocess_controller.stop("map_viewer")
        map_odom = None
        if self.subprocess_controller.is_running("mapping"):
            # 先捕获建图引擎的 map→odom，再停止建图，
            # 保证 AMCL 初始位姿与刚建好的地图保持一致
            map_odom = self._capture_map_odom()
            self.subprocess_controller.stop("mapping")
        if self.enable_aruco:
            self.subprocess_controller.start("aruco")
        if map_odom is not None:
            with self.pending_pose_lock:
                self.pending_map_odom = (map_odom, time.time())
        map_yaml = self.state.get("last_map_yaml")
        if not map_yaml or not os.path.exists(map_yaml):
            map_yaml = self.get_default_map_yaml()
        self.subprocess_controller.set_navigation_map(map_yaml)
        success, message = self.subprocess_controller.start("navigation")
        if success:
            # 路网跟随器轻量且空闲时不占 CPU，随导航栈一并拉起；
            # 启动失败不影响自由避障导航，只记日志
            route_ok, route_msg = self._ensure_route_follower()
            if not route_ok:
                self.get_logger().warning(f"路网跟随器启动失败（不影响自由导航）: {route_msg}")
        return success, message

    def _ensure_route_follower(self):
        """确保路网跟随器子系统在运行；未运行则拉起。返回 (ok, message)。"""
        if self.subprocess_controller.is_running("route"):
            return True, "路网跟随器已在运行"
        return self.subprocess_controller.start("route")

    def _capture_map_odom(self):
        """捕获当前 map→odom 变换（建图引擎在建图时发布），失败返回 None。"""
        try:
            return self.tf_buffer.lookup_transform("map", "odom", Time())
        except Exception as exc:
            self.get_logger().warning(f"获取 map→odom 失败，定位将等待手动设定位姿：{exc}")
            return None

    def _pose_from_map_odom(self, map_odom):
        """由冻结的 map→odom 与当前 odom→base 计算当前 map 帧位姿，失败返回 None。"""
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
        except Exception as exc:
            self.get_logger().warning(
                f"获取 odom→base 失败，初始位姿待手动设置：{exc}"
            )
            return None
        map_to_odom = map_odom.transform
        odom_to_base = odom_to_base.transform
        q = quaternion_multiply(map_to_odom.rotation, odom_to_base.rotation)
        rot_t = rotate_vector_by_quaternion(
            map_to_odom.rotation, odom_to_base.translation
        )
        x = map_to_odom.translation.x + rot_t[0]
        y = map_to_odom.translation.y + rot_t[1]
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return x, y, yaw

    def _publish_initial_pose(self, x, y, yaw):
        """向 /initialpose 发布初始位姿（AMCL 收到后重置粒子）。"""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        # 用零时间戳（tf2 视为"最新可用"）。若用 now()，会比最新 odom->base TF
        # 略新 18~20ms，AMCL 查变换时报 "extrapolation into the future"，
        # 导致初始位姿变换失败、粒子滤波器无法收敛、导航起不来。
        pose.header.stamp = Time().to_msg()
        pose.pose.pose.position.x = float(x)
        pose.pose.pose.position.y = float(y)
        pose.pose.pose.orientation = yaw_to_quaternion(float(yaw))
        pose.pose.covariance = [
            0.0625, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0625, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0625,
        ]
        self.initial_pose_pub.publish(pose)

    def _delayed_initial_pose(self, delay_sec=6.0):
        """导航栈启动后延迟下发初始位姿，避免与 AMCL 启动时序竞争。"""
        time.sleep(delay_sec)
        self._apply_pending_initial_pose()

    def _apply_pending_initial_pose(self, max_attempts=10, interval=3.0):
        """导航就绪前把待生效的初始位姿发给 AMCL；未收敛则自动重试。"""
        # 「启动导航」与「开始导航」各会起一个线程调用本方法；若不互斥，两个线程
        # 会分别消费 user_pose / map_odom 并交替重设 AMCL，导致定位反复跳变。
        if not self._initial_pose_guard.acquire(blocking=False):
            self.get_logger().info("初始位姿正在设置中，跳过本次重复设置")
            return
        try:
            # AMCL 已在定位（如用户刚在 Web 端直接设过位姿）时，
            # pending 队列为空不是错误，静默跳过，避免误导性提示
            if self._amcl_has_pose():
                self.get_logger().info("AMCL 已有定位，跳过初始位姿设置")
                return
            pose = None
            with self.pending_pose_lock:
                if self.pending_user_pose is not None:
                    x, y, yaw, ts = self.pending_user_pose
                    self.pending_user_pose = None
                    if time.time() - ts <= 60.0:
                        pose = (x, y, yaw)
                if pose is None and self.pending_map_odom is not None:
                    map_odom, ts = self.pending_map_odom
                    # 与 state.json 恢复窗口(600s)保持一致；过期不消费，等待手动设定位姿
                    if time.time() - ts <= 600.0:
                        pose = self._pose_from_map_odom(map_odom)
                    else:
                        self.get_logger().warning(
                            "建图坐标系已过期(>600s)，请在 Web 端地图上手动设定位姿"
                        )
                if pose is None and self.pending_boot_pose is not None:
                    # 无 ArUco 时的冷启动位姿来源：本车回桩位（per-robot 配置，不设过期，
                    # 因为它是常置的部署参数而非一次性会话数据）。优先级最低：人工设位与
                    # 同会话建图交接都更准确。
                    bp = self.pending_boot_pose
                    pose = (bp["x"], bp["y"], bp["yaw"])
                    self.publish_status(
                        "使用本车回桩位姿 boot_pose 注入 AMCL 初始位姿 "
                        f"(robot_id={self.robot_id or '未分配'})")
            if pose is None:
                self.publish_status(
                    "无可用初始位姿，请在 Web 端地图上手动设定位姿后重试"
                    "（可用 set_robot_id --capture-boot-pose 记录回桩位以自动注入）"
                )
                return
            # 重复发布直到 AMCL 开始输出位姿（收到初始位姿并收敛后才会发布 map→odom）
            for attempt in range(max_attempts):
                self._publish_initial_pose(pose[0], pose[1], pose[2])
                self.publish_status(
                    "已向 AMCL 设置初始位姿 (x={:.3f}, y={:.3f}, yaw={:.3f}) 第{}/{}次".format(
                        pose[0], pose[1], pose[2], attempt + 1, max_attempts
                    )
                )
                if self._amcl_has_pose():
                    return
                time.sleep(interval)
            self.publish_status("初始位姿已发送但 AMCL 未确认收敛，请检查定位/雷达")
        finally:
            self._initial_pose_guard.release()

    def _amcl_has_pose(self, timeout_sec=2.0):
        """检查 AMCL 是否已在定位。

        注意：AMCL 仅在位姿变化超过 update_min_d/update_min_a 时才更新滤波并
        发布 /amcl_pose；小车静止时不会有新消息。若只等新消息，会把"已定位但
        静止"误判为未定位（白等 2 秒并打印误导性提示）。
        因此优先查 map→odom TF：只要 AMCL 定位成功过，该 TF 会持续存在。
        """
        try:
            self.tf_buffer.lookup_transform("map", "odom", Time())
            return True
        except Exception:
            pass
        got = threading.Event()

        def _cb(msg):
            got.set()

        sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", _cb, 10
        )
        try:
            return got.wait(timeout_sec)
        finally:
            self.destroy_subscription(sub)

    def set_pose_callback(self, request, response):
        if self.subprocess_controller.is_running("mapping"):
            response.success = False
            response.message = "建图进行中，不能设定位姿，请先停止建图"
            return response
        x, y, yaw = float(request.x), float(request.y), float(request.yaw)
        if self.subprocess_controller.is_running("navigation"):
            # AMCL 已在运行，立即生效
            self._publish_initial_pose(x, y, yaw)
            self.publish_status(
                f"初始位姿已设置 (x={x:.3f}, y={y:.3f}, yaw={yaw:.3f})"
            )
        else:
            # AMCL 尚未运行，记录并在导航启动时生效
            with self.pending_pose_lock:
                self.pending_user_pose = (x, y, yaw, time.time())
                self.pending_map_odom = None  # 用户手动设位姿优先，清除旧的建图坐标系
            self.publish_status(
                f"已记录初始位姿，导航启动时将自动设置 (x={x:.3f}, y={y:.3f}, yaw={yaw:.3f})"
            )
        response.success = True
        response.message = (
            f"初始位姿已设置 (x={x:.3f}, y={y:.3f}, yaw={yaw:.3f})"
        )
        return response

    def _publish_authority(self, authority):
        msg = String()
        msg.data = authority
        self.authority_pub.publish(msg)

    def set_authority_callback(self, request, response):
        """设置/查询控制权。authority 留空表示只查询当前值（供 Web 主动拉取）。"""
        wanted = (request.authority or "").strip().lower()
        response.authority = self.authority
        if wanted == "":
            response.success = True
            response.message = f"当前控制权：{self.authority}"
            return response
        if wanted not in AUTHORITY_VALUES:
            response.success = False
            response.message = f"未知控制权：{wanted}（支持 controlled / debug）"
            return response
        if wanted == self.authority:
            response.success = True
            response.message = f"控制权已是 {wanted}"
            self._publish_authority(wanted)  # 幂等重发，便于订阅端重新对齐
            return response
        with self.authority_lock:
            self.authority = wanted
        self._publish_authority(wanted)
        self.publish_status(
            "控制权已切换为受控（本地手动控制被禁止）"
            if wanted == "controlled"
            else "控制权已切换为本地接管（中心不再派发任务）")
        response.success = True
        response.authority = wanted
        response.message = f"控制权已切换为 {wanted}"
        return response

    def shutdown_callback(self, request, response):
        """Web 端安全关机：先停子系统再 systemctl poweroff，避免直接断电损坏文件。"""
        self.get_logger().warning("收到 Web 端关机指令，正在执行安全关机流程")
        self.publish_status("收到关机指令：正在停止子系统并安全关机，指示灯熄灭前请不要断电")
        threading.Thread(target=self._safe_poweroff, daemon=True).start()
        response.success = True
        response.message = "关机指令已受理，系统将在数秒内安全关机"
        return response

    def params_callback(self, request, response):
        """Web 端参数配置页的读写入口：list / set / reset。"""
        command = (request.command or "").strip().lower()
        try:
            if command == "list":
                response.success = True
                response.message = "ok"
                response.data_json = json.dumps(
                    self.params_store.describe(), ensure_ascii=False
                )
                return response

            section = (request.section or "").strip()
            if command == "set":
                try:
                    values = json.loads(request.values_json or "{}")
                except ValueError:
                    response.success = False
                    response.message = "参数值不是合法的 JSON"
                    return response
                ok, message = self.params_store.set(section, values)
                if ok:
                    self._after_params_changed(section)
            elif command == "reset":
                ok, message = self.params_store.reset(section)
                if ok:
                    self._after_params_changed(section)
            else:
                ok, message = False, f"未知命令：{command}（支持 list/set/reset）"
            response.success = ok
            response.message = message
            return response
        except Exception as exc:
            self.get_logger().error(f"参数服务处理 {request.command} 失败: {exc}")
            response.success = False
            response.message = f"处理失败：{exc}"
            return response

    def _after_params_changed(self, section):
        """参数落盘后：重新物化各子系统的配置文件；
        camera 组额外把 ArUco 字典/边长同步进 markers.json（识别节点热加载）。"""
        try:
            self.params_store.write_generated()
        except Exception as exc:
            self.get_logger().error(f"生成参数配置文件失败: {exc}")
        if section in ("camera", "all"):
            camera = self.params_store.data["camera"]
            self.marker_store.data["dictionary"] = camera["dictionary"]
            self.marker_store.data["marker_size"] = float(camera["marker_size"])
            self.marker_store.save()
            self.get_logger().info(
                f"ArUco 规格已同步到 markers.json（字典 {camera['dictionary']}，"
                f"边长 {camera['marker_size']}），识别节点将自动热加载"
            )
        self.publish_status("参数已保存并写入配置文件，按页面提示重启对应子系统后生效")

    def _safe_poweroff(self):
        try:
            # 先睡一小段，让服务响应/状态消息送达 Web 端
            time.sleep(1.5)
            try:
                # 逐个 SIGINT→SIGTERM→SIGKILL 干净停掉底盘/建图/导航等子系统
                self.subprocess_controller.stop_all()
            except Exception as exc:
                self.get_logger().warning(f"停止子系统出现异常（继续关机）: {exc}")
            subprocess.run(["sync"], check=False)
            result = subprocess.run(
                ["systemctl", "poweroff", "-i"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if result.returncode != 0:
                # 无 systemd（或 poweroff.target 被抑制）时退回传统 poweroff
                os.system("poweroff")
        except Exception as exc:
            self.get_logger().error(f"安全关机执行失败: {exc}")

    def _persist_map_odom(self, map_odom):
        """把建图坐标系 map→odom 写入 state.json，服务重启后仍可恢复。"""
        try:
            t = map_odom.transform.translation
            q = map_odom.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self.state["pending_map_odom"] = {
                "x": t.x, "y": t.y, "z": t.z, "yaw": yaw,
                "ts": time.time(),
            }
            self.save_state()
            self.get_logger().info(
                f"建图坐标系已持久化 map→odom (x={t.x:.3f}, y={t.y:.3f}, yaw={yaw:.3f})"
            )
        except Exception as exc:
            self.get_logger().warning(f"持久化建图坐标系失败: {exc}")

    def _stop_mapping_handoff(self):
        """停止建图，并记录建图引擎坐标系供导航启动时设置 AMCL 初始位姿。"""
        map_odom = None
        if self.subprocess_controller.is_running("mapping"):
            map_odom = self._capture_map_odom()
        stopped, message = self.subprocess_controller.stop("mapping")
        if self.enable_aruco:
            self.subprocess_controller.start("aruco")
        if map_odom is not None:
            with self.pending_pose_lock:
                self.pending_map_odom = (map_odom, time.time())
            self._persist_map_odom(map_odom)
            self.publish_status(
                "建图已停止，已记录建图坐标系，导航启动时将自动设定位姿"
            )
        return stopped, message

    def control_system_callback(self, request, response):
        subsystem = request.subsystem.strip().lower()
        requested_mapping_engine = None
        if subsystem.startswith("mapping:"):
            requested_mapping_engine = subsystem.split(":", 1)[1]
            subsystem = "mapping"
        if request.enable:
            if subsystem == "mapping":
                mapping_engine = self.subprocess_controller.set_mapping_engine(
                    requested_mapping_engine
                    or self.state.get("mapping_algorithm", "cartographer")
                )
                self.state["mapping_algorithm"] = mapping_engine
                self.save_state()
                self.subprocess_controller.stop("map_viewer")
                self.subprocess_controller.stop("navigation")
                # 建图时由建图引擎发布 map→odom，先停掉里程计定位，
                # 避免两个节点抢同一个 TF 导致定位错乱
                self.subprocess_controller.stop("localization")
                success, message = self.subprocess_controller.start("mapping")
                if success:
                    # 建图期间按开关运行 ArUco 定位节点（默认关闭，避免空转占满 CPU）
                    if self.enable_aruco:
                        self.subprocess_controller.start("aruco")
            elif subsystem == "navigation":
                success, message = self._start_navigation_stack()
                if success:
                    # 导航栈刚启动，延迟数秒后下发初始位姿，
                    # 否则 AMCL 未初始化时不发布 map→odom，导航永远无法就绪
                    threading.Thread(
                        target=self._delayed_initial_pose, daemon=True
                    ).start()
            else:
                success, message = self.subprocess_controller.start(subsystem)
        else:
            if subsystem == "mapping":
                success, message = self._stop_mapping_handoff()
            else:
                success, message = self.subprocess_controller.stop(subsystem)
                if subsystem == "navigation" and success:
                    # 导航栈停了，路网跟随器一并停（它只消费 AMCL 的 TF）
                    self.subprocess_controller.stop("route")
                    self.active_nav_mode = None
        response.success = success
        response.message = message
        self.publish_status(message)
        return response

    def station_callback(self, request, response):
        command = request.command.strip().lower()
        if command == "list":
            names, xs, ys, yaws = self.station_store.list_stations()
            response.names = names
            response.x = xs
            response.y = ys
            response.yaw = yaws
            response.success = True
            response.message = f"共 {len(names)} 个站点"
        elif command == "add" or command == "update":
            if not request.name:
                response.message = "请填写站点名称"
                return response
            self.station_store.upsert(
                request.name, request.x, request.y, request.yaw
            )
            response.success = True
            response.message = f"站点 {request.name} 已保存"
        elif command == "delete":
            self.station_store.delete(request.name)
            response.success = True
            response.message = f"站点 {request.name} 已删除"
        elif command == "clear":
            self.station_store.clear()
            response.success = True
            response.message = "所有站点已删除"
        else:
            response.message = f"未知操作：{command}"
        return response

    def marker_callback(self, request, response):
        command = request.command.strip().lower()
        if command == "list":
            ids, sizes, xs, ys, yaws = self.marker_store.list_markers()
            response.marker_ids = ids
            response.marker_sizes = sizes
            response.x = xs
            response.y = ys
            response.yaw = yaws
            response.success = True
            response.message = f"共 {len(ids)} 个标志位"
        elif command == "add" or command == "update":
            self.marker_store.upsert(
                request.marker_id,
                request.marker_size,
                request.x,
                request.y,
                request.yaw,
            )
            response.success = True
            response.message = f"标志位 {request.marker_id} 已保存"
        elif command == "delete":
            self.marker_store.delete(request.marker_id)
            response.success = True
            response.message = f"标志位 {request.marker_id} 已删除"
        elif command == "clear":
            self.marker_store.clear()
            response.success = True
            response.message = "所有标志位已删除"
        else:
            response.message = f"未知操作：{command}"
        return response

    def map_file_callback(self, request, response):
        command = request.command.strip().lower()
        if command == "list":
            map_names = sorted(
                path.stem
                for path in self.map_directory.glob("*.yaml")
                if (self.map_directory / f"{path.stem}.pgm").exists()
            )
            response.map_names = map_names
            response.success = True
            response.message = f"共 {len(map_names)} 张地图" if map_names else "暂无已保存地图"
            return response

        if command == "open":
            if self.subprocess_controller.is_running("mapping"):
                response.message = "建图进行中，不能加载其它地图，请先停止建图"
                return response
            if self.subprocess_controller.is_running("navigation"):
                response.message = "导航运行中，不能加载地图，请先停止导航"
                return response
            map_name = request.map_name.strip()
            map_yaml = self.map_directory / f"{map_name}.yaml"
            map_pgm = self.map_directory / f"{map_name}.pgm"
            if not map_yaml.exists() or not map_pgm.exists():
                response.message = f"地图不存在：{map_name}"
                return response
            self.subprocess_controller.stop("map_viewer")
            self.subprocess_controller.set_map_viewer_file(str(map_yaml))
            success, message = self.subprocess_controller.start("map_viewer")
            if success:
                self.state["last_map_yaml"] = str(map_yaml)
                self.state["last_map_name"] = map_name
                self.save_state()
                self.publish_status(f"已打开地图 {map_name}")
            response.success = success
            response.message = message
            return response

        if command == "close":
            success, message = self.subprocess_controller.stop("map_viewer")
            response.success = success
            response.message = message
            return response

        response.message = f"未知操作：{command}"
        return response

    def navigate_callback(self, request, response):
        mode = (getattr(request, "mode", "") or "free").strip().lower()
        if mode not in ("free", "route"):
            response.accepted = False
            response.message = f"未知导航方式：{mode}（支持 free / route）"
            return response
        if self.authority == "controlled" and mode == "free":
            # 生产采用轨道路网：受控态只允许 route，自由导航会绕过路网语义
            response.accepted = False
            response.message = (
                "受控模式下只允许轨道路线导航（free 已拒绝）；"
                "确需自由导航请先接管控制权（debug）")
            return response
        station = self.station_store.find(request.station_name)
        if station is None:
            response.accepted = False
            response.message = f"未找到站点：{request.station_name}"
            return response

        if mode == "route":
            # 尽早给出可读的错误：路网里没有该站点的节点时直接拒绝
            self.route_store.reload_if_changed()
            has_node = any(
                node.get("station") == request.station_name
                or node.get("name") == request.station_name
                for node in self.route_store.data.get("nodes", [])
            )
            if not has_node:
                response.accepted = False
                response.message = (
                    f"路网导航失败：路网中没有站点 {request.station_name} 对应的节点，"
                    "请先在路网编辑器中添加并连线")
                return response

        if not self.subprocess_controller.is_running("navigation"):
            self.publish_status("导航未启动，正在自动启动导航…")
            ok, message = self._start_navigation_stack()
            if not ok:
                response.accepted = False
                response.message = message
                self.publish_status(message)
                return response

        self.active_nav_mode = mode
        if mode == "route":
            # route 模式不经过 Nav2 action，目标由 route_follower 执行
            self.publish_status(f"正在以轨道路线方式导航到 {request.station_name}")
            threading.Thread(
                target=self._send_route_goal_when_ready,
                args=(request.station_name,),
                daemon=True,
            ).start()
            response.accepted = True
            response.message = f"正在以轨道路线方式导航到 {request.station_name}"
            return response

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(station["x"])
        pose.pose.position.y = float(station["y"])
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(float(station.get("yaw", 0.0)))

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.publish_status(f"正在导航到 {request.station_name}")
        threading.Thread(
            target=self._send_goal_when_ready,
            args=(goal, station),
            daemon=True,
        ).start()
        response.accepted = True
        response.message = f"正在导航到 {request.station_name}"
        return response

    def _send_route_goal_when_ready(self, station_name, timeout_sec=60.0):
        """轨道路线模式：定位/跟随器已就绪时立即下发目标，只有冷启动才等待。

        原来无条件 time.sleep(6.0) 是为"导航栈刚启动、AMCL 还没起来"设计的；
        日常运行中导航栈早已就绪，这 6 秒纯属空等（表现为点击后 8 秒才起步）。
        路网模式只依赖 AMCL 的 map→odom TF + 跟随器服务，不需要 Nav2 action，
        因此就绪判定后可直接下发。
        """
        if self.route_nav_client.service_is_ready() and self._amcl_has_pose():
            self._apply_pending_initial_pose()   # 已有定位时内部会立即跳过
        else:
            time.sleep(6.0)
            self._apply_pending_initial_pose()
            if not self._wait_navigation_ready(timeout_sec):
                self.publish_status(
                    "路网导航失败：导航栈启动后 60 秒仍未就绪，请查看定位/导航日志")
                self.active_nav_mode = None
                return
        if not self.route_nav_client.wait_for_service(timeout_sec=15.0):
            self.publish_status("路网导航失败：路网跟随器未运行，请重启导航")
            self.active_nav_mode = None
            return
        request = NavigateToStation.Request()
        request.station_name = station_name
        request.mode = "route"
        future = self.route_nav_client.call_async(request)
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.publish_status("路网导航失败：跟随器响应超时")
            self.active_nav_mode = None
            return
        result = future.result()
        if result is None or not result.accepted:
            self.publish_status(result.message if result else "路网导航失败：无返回")
            self.active_nav_mode = None
            return
        self.publish_status(result.message)

    def _send_goal_when_ready(self, goal, station, timeout_sec=60.0):
        # 导航栈刚启动时 AMCL 尚未注册 /initialpose 订阅，且未初始化时不会发布
        # map→odom TF，导致 costmap/bt_navigator 永远无法就绪。
        # 因此冷启动时先等 AMCL 起来，立即下发初始位姿，再等待导航栈激活；
        # 已就绪（action server 可用 + AMCL 已定位）时跳过固定 6 秒空等。
        if not (self.nav_client.server_is_ready() and self._amcl_has_pose()):
            time.sleep(6.0)
        self._apply_pending_initial_pose()
        if not self._wait_navigation_ready(timeout_sec):
            self.publish_status(
                "导航服务不可用：启动后 60 秒仍未就绪，请查看定位/导航日志"
            )
            return
        self._send_goal_with_retry(goal, station)

    def _wait_navigation_ready(self, timeout_sec=60.0):
        """等待 Nav2 导航栈完全激活（action server 可用且 bt_navigator 处于 ACTIVE）。"""
        deadline = time.time() + timeout_sec
        if not self.nav_client.wait_for_server(timeout_sec=min(15.0, timeout_sec)):
            return False
        while time.time() < deadline:
            if self._bt_navigator_is_active():
                return True
            time.sleep(1.0)
        return False

    def _bt_navigator_is_active(self):
        if not self.nav_state_client.service_is_ready():
            return False
        request = GetState.Request()
        future = self.nav_state_client.call_async(request)
        # rclpy 的 Future.result() 不含 timeout 参数，用 result(timeout=...) 会抛
        # TypeError 被 except 吞掉导致本函数永远返回 False，进而让 _wait_navigation_ready
        # 走到 60s 超时而误报“导航服务不可用”。正确做法是轮询 future.done()，
        # 响应由主线程的 rclpy.spin 处理并置位，这里只需限时等待。
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False
        try:
            result = future.result()
        except Exception:
            return False
        if result is None:
            return False
        return result.current_state.id == 3  # PRIMARY_STATE_ACTIVE

    def _send_goal_with_retry(self, goal, station, retries=5, retry_delay=5.0):
        for attempt in range(retries + 1):
            send_goal_future = self.nav_client.send_goal_async(
                goal, feedback_callback=self.navigation_feedback_callback
            )
            # 与 _bt_navigator_is_active 同理：rclpy Future.result() 无 timeout 参数，
            # 用 result(timeout=10.0) 会抛 TypeError。改为轮询 future.done() 限时等待。
            goal_deadline = time.time() + 10.0
            while (
                not send_goal_future.done()
                and time.time() < goal_deadline
            ):
                time.sleep(0.05)
            if send_goal_future.done():
                goal_handle = send_goal_future.result()
            else:
                goal_handle = None
            if goal_handle is not None and goal_handle.accepted:
                with self.nav_lock:
                    self.nav_goal_handle = goal_handle
                self.publish_status(f"已开始前往 {station['name']}")
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.navigation_result_callback)
                return
            if attempt < retries:
                self.publish_status(
                    f"导航服务尚未就绪，{int(retry_delay)} 秒后重试（第 {attempt + 1}/{retries} 次）…"
                )
                time.sleep(retry_delay)
            else:
                self.publish_status(
                    "导航目标被拒绝：导航服务未就绪或已有其它导航任务进行中，请检查定位/导航日志"
                )

    def navigation_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = float(feedback.distance_remaining)
        now = time.time()
        with self._nav_feedback_lock:
            # Nav2 一个控制周期会发多条反馈；1 秒内且距离变化小于 5cm 的
            # 不重复播报，避免状态栏同一秒刷出多条相同内容
            if (
                now - self._nav_feedback_last_time < 1.0
                and abs(distance - self._nav_feedback_last_distance) < 0.05
            ):
                return
            self._nav_feedback_last_time = now
            self._nav_feedback_last_distance = distance
        self.publish_status(f"剩余距离：{distance:.2f} 米")

    def navigation_result_callback(self, future):
        with self.nav_lock:
            self.nav_goal_handle = None
        result = future.result()
        if result.status == 4:
            self.publish_status("导航完成")
        elif result.status == 5:
            self.publish_status("导航已取消")
        else:
            self.publish_status(f"导航失败，状态码 {result.status}")

    def cancel_navigation_callback(self, request, response):
        if self.active_nav_mode == "route":
            if not self.route_cancel_client.wait_for_service(timeout_sec=5.0):
                response.success = False
                response.message = "路网跟随器未运行，无法取消"
                return response
            future = self.route_cancel_client.call_async(CancelNavigation.Request())
            deadline = time.time() + 5.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            result = future.result() if future.done() else None
            if result is not None and result.success:
                self.active_nav_mode = None
            response.success = result.success if result is not None else False
            response.message = result.message if result is not None else "取消请求超时"
            return response
        with self.nav_lock:
            goal_handle = self.nav_goal_handle
        if goal_handle is None:
            response.success = False
            response.message = "当前没有导航任务"
            return response
        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.result()
        response.success = True
        response.message = "已请求取消导航"
        return response

    def route_status_callback(self, msg):
        """转发路网跟随器状态到统一状态流；终态时清除当前导航模式。"""
        self.publish_status(msg.data)
        for keyword in ("导航完成", "已取消", "导航失败", "中止"):
            if keyword in msg.data:
                self.active_nav_mode = None
                break

    def route_callback(self, request, response):
        """Web 路网画布编辑器的存取入口：list / save / clear。"""
        command = (request.command or "").strip().lower()
        if command == "list":
            graph = self.route_store.list_graph()
            response.success = True
            response.message = "ok"
            response.graph_json = json.dumps(graph, ensure_ascii=False)
            return response
        if command == "save":
            try:
                payload = json.loads(request.graph_json or "{}")
            except ValueError:
                response.success = False
                response.message = "路网数据不是合法的 JSON"
                return response
            ok, message = self.route_store.save_graph(payload)
            response.success = ok
            response.message = message
            response.graph_json = (
                json.dumps(self.route_store.list_graph(), ensure_ascii=False)
                if ok else ""
            )
            if ok:
                self.publish_status(message + "，route 模式导航时自动加载")
            return response
        if command == "clear":
            self.route_store.clear()
            response.success = True
            response.message = "路网已清空"
            response.graph_json = "{\"nodes\": [], \"edges\": []}"
            return response
        response.success = False
        response.message = f"未知操作：{command}（支持 list/save/clear）"
        return response

    def map_subscription_callback(self, msg):
        with self.map_cache_lock:
            self.map_cache = msg

    def save_map_callback(self, request, response):
        map_name = request.map_name.strip() or "delivery_map"
        mapping_running = self.subprocess_controller.is_running("mapping")
        republish_thread = None
        stop_event = threading.Event()
        if not mapping_running:
            with self.map_cache_lock:
                cached = self.map_cache
            if cached is None:
                response.message = (
                    "保存地图失败：没有可用的地图数据。"
                    "请先启动建图生成地图后再保存（保存需在建图运行期间或刚停止建图后进行）"
                )
                self.publish_status(response.message)
                return response
            self.publish_status("建图已停止，使用最近生成的地图数据保存…")

            # 先同步发布一次，让 transient_local 的 latch 提前就绪；map_saver_cli
            # 内部默认 save_map_timeout 仅 2s，系统繁忙时 DDS 发现可能来不及，
            # 从而报 "Failed to spin map subscription"。
            try:
                self.map_pub.publish(cached)
            except Exception as exc:
                self.get_logger().warning(f"预发布缓存地图失败: {exc}")

            def republish_cached_map():
                while not stop_event.is_set():
                    with self.map_cache_lock:
                        msg = self.map_cache
                    if msg is not None:
                        try:
                            self.map_pub.publish(msg)
                        except Exception as exc:
                            self.get_logger().warning(f"重发缓存地图失败: {exc}")
                    time.sleep(0.2)

            republish_thread = threading.Thread(
                target=republish_cached_map, daemon=True
            )
            republish_thread.start()
            # 让 republish 线程先跑起来、DDS 端点先完成注册，再启动 map_saver_cli
            time.sleep(0.5)

        command = (
            "ros2 run nav2_map_server map_saver_cli -f "
            f"{map_name} --ros-args -p map_subscribe_transient_local:=true "
            "-p save_map_timeout:=15.0"
        )
        try:
            result = subprocess.run(
                shlex.split(command),
                cwd=str(self.map_directory),
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            response.message = f"保存地图超时：{exc}"
            return response
        finally:
            if republish_thread is not None:
                stop_event.set()
                republish_thread.join(timeout=1.0)
        if result.returncode != 0:
            response.message = result.stderr or result.stdout
            return response
        response.success = True
        map_yaml_path = self.map_directory / f"{map_name}.yaml"
        response.map_yaml_path = str(map_yaml_path)
        self.state["last_map_yaml"] = str(map_yaml_path)
        self.state["last_map_name"] = map_name
        self.save_state()
        response.message = f"地图已保存到 {response.map_yaml_path}"
        self.publish_status(response.message)
        return response

    def shutdown(self):
        self.subprocess_controller.stop_all()


def acquire_singleton_lock():
    lock_dir = Path.home() / ".originbot_delivery"
    lock_dir.mkdir(parents=True, exist_ok=True)
    handle = open(lock_dir / "delivery_server.lock", "w")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        return None
    handle.write(str(os.getpid()))
    handle.flush()
    return handle


def main(args=None):
    rclpy.init(args=args)
    lock_handle = acquire_singleton_lock()
    if lock_handle is None:
        print(
            "检测到另一个 delivery_server 已在运行，为避免重复启动冲突，本进程退出。",
            file=sys.stderr,
        )
        rclpy.shutdown()
        return
    node = DeliveryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
