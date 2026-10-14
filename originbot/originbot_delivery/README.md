# OriginBot Delivery

这个包在现有 OriginBot ROS 2 工程上补全送餐机器人需要的两层能力：

1. 用 ArUco 码给重复度较高的地图提供全局定位校正。
2. 通过 rosbridge 暴露一套 Web 可调用的任务服务，完成启动底盘、建图、站点管理、标志位管理和站点导航。

## 总体数据流

```text
base / lidar / camera
        |
        +-- /odom (odom -> base_footprint)
        +-- /imu
        +-- /scan
        +-- /image + /camera_info
                        |
              aruco_localization
                        |
                        +-- 建图期：自动登记标志位 map 系坐标 -> markers.json
                        +-- 导航期：/aruco/pose (map frame)，偏差超过阈值时
                                            |
                                            v
                                     /initialpose 重设 AMCL
                                            |
                                            v
                        Nav2 (map_server + AMCL + planner/controller/...)
```

`delivery_server` 的导航栈使用 **AMCL 定位**（`nav_bringup.launch.py`，即 nav2_bringup 自带 AMCL），
ArUco 识别到已登记标志位后，若与 AMCL 位姿偏差超过阈值（默认 0.3 m / 15°，最短 5 s 一次），
会向 `/initialpose` 重新发布位姿来纠正 AMCL。

TF 归属保持单一：

- `originbot_base` 唯一发布 `odom -> base_footprint`。
- 建图期 `map -> odom` 由所选建图算法发布；导航期由 AMCL 发布。
- 仓库同时提供无 AMCL 的全局 EKF 方案（`originbot_navigation` 的 `ekf_map.launch.py` +
  `nav_bringup_no_amcl.launch.py`，见 `launch/ekf.md`），但与本包 Web 流程互斥：
  不要同时运行 AMCL 和全局 EKF，否则会出现两个 `map -> odom`。

## 依赖

- RDK X5，推荐 ROS 2 Humble。
- `slam_toolbox`、`nav2_bringup`、`robot_localization`。
- `rosbridge_server`。
- `python3-opencv` 以及包含 `cv2.aruco` 的 OpenCV contrib。
- 相机内参由 `/camera_info` 提供，通常来自 `hobot_usb_cam` 的标定文件。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select originbot_delivery
source install/setup.bash
```

## Web 启动

```bash
ros2 launch originbot_delivery delivery_bringup.launch.py
```

浏览器打开：

```text
http://<RDK_X5_IP>:8080
```

页面中的 WebSocket 地址填写：

```text
ws://<RDK_X5_IP>:9090
```

## 开机自启动

`delivery_bringup.launch.py` 只启动 Web / rosbridge 服务层，无需图形界面，适合开机自启。包内提供 systemd 服务模板和一键安装脚本（位于 `scripts/`）：

```bash
cd /userdata/dev_ws/src/originbot/originbot_delivery
sudo bash scripts/install_autostart.sh            # 安装并立即启动
sudo bash scripts/install_autostart.sh --no-start # 只安装并设置开机自启，暂不启动
sudo bash scripts/install_autostart.sh --uninstall # 卸载自启动
sudo bash scripts/install_autostart.sh --dry-run  # 预览生成的 systemd 配置，不实际安装
```

脚本会自动探测 ROS 环境（`/opt/ros/humble/setup.bash`）和工作空间（向上查找 `install/setup.bash`），也可通过环境变量覆盖：

```bash
sudo ROS_SETUP=/opt/ros/humble/setup.bash \
     WS_SETUP=/path/to/ws/install/setup.bash \
     RUN_USER=root \
     bash scripts/install_autostart.sh
```

安装后常用命令：

```bash
systemctl status originbot-delivery
journalctl -u originbot-delivery -f
```

注意：

- 服务开启后不要再手动运行 `ros2 launch originbot_delivery delivery_bringup.launch.py`，`delivery_bringup.launch.py` 的单实例保护会拒绝重复启动，导致服务反复重启。
- `delivery_server` 在页面操作时通过 `ros2 launch` 拉起底盘、雷达、相机和导航（见 `delivery_server.py` 的 `commands`），因此服务运行用户需要对相关设备节点有访问权限，RDK X5 默认 `root` 即可。
- 服务运行用户（`RUN_USER`）同时决定状态文件位置 `~/.originbot_delivery`，请保持与手动运行时一致，避免地图、站点等配置读取路径变化。
- 自启动只拉起 Web / rosbridge 服务层，底盘、雷达、相机仍需在页面点击 `Start Base`。

## 操作流程

1. 在 Web 页面点击 `Start Base`，启动底盘、雷达和相机。
2. 在 Web 页面的 `建图算法` 中选择 `SLAM Toolbox` 或 `Cartographer`，点击
   `Start Mapping`。该操作会启动所选建图算法，并同步启动 ArUco 定位用于自动登记标志位。
3. 遥控或手推机器人完成建图。每经过一个 ArUco 码时放慢速度，让相机连续识别到它
   （默认需 ≥5 帧、观测跨度 ≥1 s），节点会自动把该标志位的 map 系坐标写入
   `~/.originbot_delivery/config/markers.json`。
4. 建图完成后点击 `Save Map`。
5. 点击 `Start Navigation`。该操作会停止 Cartographer，并启动带 AMCL 的 Nav2，
   自动加载最近保存的地图，并把建图坐标系换算为 AMCL 初始位姿。
6. 在 `Stations` 面板建立站点（ArUco 标志位一般无需手动录入，建图时已自动登记）。
7. 从导航下拉框选择站点，点击 `Navigate`。导航过程中识别到已登记标志位时，
   会持续纠正 AMCL 定位。

保存地图后，`delivery_server` 会把最近一次保存的 `.yaml` 路径记录在 `~/.originbot_delivery/config/state.json` 中。下次启动导航时，如果没有额外指定地图，会自动加载这张最近保存的地图。

## 键盘遥控

Web 控制台中的“键盘控制”面板用于建图或人工推车。先连接 rosbridge，点击“进入手动模式”，然后用以下按键控制：

```text
W / 上方向键：前进
S / 下方向键：后退
A / 左方向键：左转
D / 右方向键：右转
空格：急停
```

进入手动模式时会先请求取消当前导航任务，避免 Nav2 和键盘同时写入 `/cmd_vel`。退出手动模式或页面失去焦点时会立即发布零速度。

## ArUco 标定

每个 ArUco 标志位包含三个量：

- 物理标记 ID，以及物理标记边长。
- 标记在已保存地图中的全局坐标 `x/y`。
- 标记在 `map` 坐标系中的朝向 `yaw`。

配置会持久化到：

```text
~/.originbot_delivery/config/markers.json
```

### 相机内参标定（让兜底内参可信）

`camera_info_relay` 在收不到真实 `/camera_info` 时会兜底发布一份内参。若从未标定，
兜底只是占位值（`fx=fy=500`），ArUco 的 PnP 解算会产生系统性偏差（越远越偏）且无告警。
请用棋盘格实测一次，把兜底升级为标定值：

```bash
# 标定板：7 x 9 内角点（即 8 x 10 个方格，约 80x100mm）、方格边长 10mm，
# 贴平整硬板上。先在 Web 端"启动底盘"，然后在 RDK 上运行：
# （板面较小，手持棋盘在 0.15~0.35m 近距离内多角度采集，覆盖画面四角；
#   0.35m 处方格仍约 10px 可检出，尽量靠近采集）
ros2 run originbot_bringup calibrate_camera
```

- 结果写入 `~/.originbot_delivery/config/camera_calibration.yaml`，
  重启底盘（`camera_info_relay`）后自动作为兜底内参生效，启动日志会注明来源。
- `RMS 重投影误差 < 0.5px` 为优；`> 1.0px` 建议重标。
- 相机分辨率变化后必须重新标定。
- 无显示环境（SSH）时加 `--no-gui`；台架标定可加 `--device /dev/video0` 脱离 ROS 运行。

### 建图期自动登记（推荐）

`aruco_localization` 在建图期间会自动识别"尚未登记"的标志位：利用当前建图算法发布的
`map -> odom` TF，把相机观测到的标志位换算到 map 系，多次观测收敛后自动写入
`markers.json`。参数（`aruco_localization` 节点）：

```text
auto_register              true   是否开启自动登记
register_min_observations  5      最少观测帧数
register_min_span_sec      1.0    观测时间跨度下限（秒）
register_max_stddev        0.15   观测位置标准差上限（米），防止把抖动数据写进去
register_amcl_timeout      10.0   收到 AMCL 位姿后多少秒内禁止自动登记（导航期）
```

自动登记只在"没有 AMCL 位姿输入"时进行（即建图阶段）；进入导航后节点自动停止登记，
只做定位校正。若某次登记结果不理想，可在 Web 的 `ArUco Markers` 面板删除后重新建图录入。
环境中的标志位间隔建议 2~3 m，标记边长需在 `markers.json` 的 `marker_size` 中与实际一致。

相机安装参数位于 `launch/aruco.launch.py`，默认假设相机在 `base_footprint` 前方。实际安装后必须校准 `camera_x/y/z` 和 `camera_yaw`，并确认 `/camera_info` 内参正确。

Web 页面支持在地图上直接取点。先加载已保存地图，然后在站点或 ArUco 标志位表单中点击“地图取点”，在地图上点击确定 `x/y`，按住并拖拽设置 `yaw`，坐标会自动填入表单。

## 轨道路线导航（route 模式）

除了默认的自由避障导航（Nav2：DWB 在代价地图上自由绕行），本包提供“轨道式”导航：
沿预设拓扑路网（Graph）行驶，**管到点也管路**，遇动态障碍不绕行而是停车等待。
两种模式共用同一套定位（AMCL 的 `map → odom`），区别只在目标发给谁：

| 模式 | 执行器 | 遇动态障碍 | 适用场景 |
|------|--------|-----------|----------|
| `free`（默认） | Nav2 bt_navigator | 自由绕行 | 开阔区域、允许绕行 |
| `route` | `route_follower`（本包） | **停车等待，无限等，消除后续走** | 固定配送路线、走廊、不允许偏离的路径 |

轨道路线模式的行为：

1. 规划：从当前位置投影到最近路网边“上车”（垂距超过 0.6m 则找最近节点直线上车，
   超过 1.5m 拒绝任务），Dijkstra（边权=折线长度）搜索到目标节点的最短边序列。
2. 跟随：投影式纯跟踪（lookahead 0.35m）沿边行驶，起步前先原地转向对准第一段路径；
   到点前自动减速，**到站后原地转到节点朝向（朝向对准，容差 0.1rad）**。
3. 安全：`/scan` 前向 ±30° 扇区 0.45m 内有障碍立即停车，**无限等待**（每 15s 提示一次），
   持续畅通 1.5s 后沿原路继续；激光数据缺失/过期同样停车等待（不盲走）；
   `map → base` TF 丢失停车等待恢复；速度带加速度斜率限幅。

### 路网文件与编辑器

路网保存在 `~/.originbot_delivery/config/routes.json`（route_follower 在每次导航时
自动热加载最新内容）：

```json
{
  "nodes": [{"name": "n1", "x": 1.2, "y": 0.5, "yaw": 0.0, "station": "kitchen"}],
  "edges": [{"a": "n1", "b": "n2", "via": [[2.0, 0.5]]}]
}
```

- 节点 `station` 字段（可选）：绑定到站点；route 模式按站点名导航时匹配它，
  到站朝向取节点 `yaw`。
- 边 `via`（可选）：走廊折线拐点；Web 编辑器 v1 生成直连边，需要拐弯时
  多放一个中间节点即可。

Web 端“路网”面板提供画布编辑器：

- **➕ 添加节点**：在地图上点击放置；自动吸附 0.3m 内的站点（继承位置与朝向）；
- **🔗 连接节点**：依次点击两个节点连线（双向可通行）；
- **🗑 删除**：点击节点删除及其连线，点击连线删边；
- 编辑在本地进行，点**保存路网**才写入后端（结构/引用校验后原子写入）；
- “清空路网”立即生效并保存。

导航面板的“导航方式”下拉框选择 `自由避障（Nav2）` / `轨道路线（沿路网）`，
“开始导航”按所选模式下发；选 route 时若路网为空或目标站点不在路网中会直接拒绝并提示。
“取消导航”对两种模式都有效（delivery_server 根据当前任务模式分发）。

### 与 Nav2 的关系

route 模式**复用**导航栈（`nav_bringup.launch.py`）提供 AMCL 定位与初始位姿交接，
Nav2 空闲时不发布 `/cmd_vel`，route_follower 是唯一的速度写者，无冲突。
“启动导航/停止导航”会一并启停路网跟随器（子系统 `route`，日志在
`<地图目录>/logs/route.log`）。轨道路线不使用 Nav2 规划器/控制器/代价地图，
Nav2 参数调整对 route 模式无效（速度/减停等参数见 route_follower 声明）。

### 新增 ROS 接口

```text
/route/navigate       originbot_delivery/srv/NavigateToStation  # station_name=目标站点
/route/cancel         originbot_delivery/srv/CancelNavigation
/delivery/route       originbot_msgs/srv/RouteGraph              # list/save/clear 路网
/route/status         std_msgs/msg/String                        # 状态（转发到 /delivery/status）
/route/path           geometry_msgs/msg/Path                     # 当前规划路径（RViz 调试）
```

`NavigateToStation.srv` 新增 `mode` 字段（`free`/`route`，留空按 `free`，向后兼容旧前端）。

### 构建（route_follower 首次启用需重新编译两包）

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select originbot_msgs originbot_delivery
source install/setup.bash
sudo bash scripts/install_autostart.sh   # systemd 自启动场景需重装/重启服务
```

## ROS 接口

服务均通过 rosbridge 暴露给 Web：

```text
/delivery/control_system      originbot_delivery/srv/ControlSystem
/delivery/station             originbot_delivery/srv/StationEntry
/delivery/marker              originbot_delivery/srv/MarkerEntry
/delivery/navigate_to_station originbot_delivery/srv/NavigateToStation
/delivery/cancel_navigation   originbot_delivery/srv/CancelNavigation
/delivery/save_map            originbot_delivery/srv/SaveMap
```

ArUco 节点发布：

```text
/aruco/pose         geometry_msgs/msg/PoseWithCovarianceStamped
/aruco/debug_image  sensor_msgs/msg/Image
/aruco/compressed   sensor_msgs/msg/CompressedImage
/aruco/status       std_msgs/msg/String
```

Web 控制台的“摄像头”面板订阅 `/aruco/compressed` 显示实时标注画面，并订阅 `/aruco/status` 显示当前是否识别到 ArUco 码及其编号。这个功能沿用 TogetheROS 的 `hobot_usb_cam` 摄像头链路，把原 `camera_websoket_display.launch.py` 示例中的图像与算法结果展示方式整合进了 rosbridge Web 控制台。

注意：`/aruco/compressed` 必须使用 `compression: "none"`（JSON）订阅，不能使用 `cbor-raw`。当前 rosbridge 的 `cbor-raw` 会把 ROS 消息对象直接交给 CBOR 序列化，`sensor_msgs/CompressedImage` 无法被序列化，会报 `don't know how to cbor serialize object of type CompressedImage`。页面已在 `static/app.js` 中显式指定，并设置了 `throttle_rate: 100`（约 10 fps）控制图像流量。

“定位状态”面板根据 `/odometry/filtered` 的协方差和 `/aruco/status` 判断当前是否定位成功、显示定位置信度、机器人当前位置、最近站点和当前导航目标。

“IMU 状态”面板订阅 `/imu`，显示四元数、欧拉角、角速度和线加速度，并提供按钮调用 `/imu_calibrate` 服务。

任务状态发布：

```text
/delivery/status     std_msgs/msg/String
```
