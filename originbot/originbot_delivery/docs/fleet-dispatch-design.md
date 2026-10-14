# OriginBot 车队调度服务 — 接口与数据模型草案

> 状态：**评审基线 v0.1**（未实现）。用于确定中心调度服务、车端 agent、配置束（地图同源）与路网数据模型的对外契约。评审定稿后再进入编码。
>
> 适用：4–10 台同型号 OriginBot（RDK X5，ROS 2 Humble），生产采用**轨道路网导航**（非自由导航），共用同一地图与站点；脱机/下线时保留每车本地 Web 调试能力。

---

## 0. 已锁定前置决策

| # | 决策 | 结论 | 关键收益 |
|---|---|---|---|
| D1 | 中心↔车通信底座 | **B 方案：非 DDS**（下行 gRPC 或 MQTT-RPC，上行 MQTT 遥测），中心不加入 DDS 域 | 车端 ROS 图**完全无需改造命名空间/`ROS_DOMAIN_ID`**；现有 `/odom /cmd_vel /scan /map` 绝对名与硬编码节点名保持不变 |
| D2 | 地图同源 | **一车建图 → 上传中心 → 中心下发全部车 → 每车 AMCL + ArUco 全局初始位姿对齐** | 各车 `map` 系原点天然一致（同一 `.pgm/.yaml` 字节 + 对同一图做 AMCL 定位） |
| D3 | 避让点语义 | 任意节点可声明为避让点；由调度器负责不死锁，可设多避让点 | 复用 `route_follower` 现成的"投影上车→Dijkstra→纯跟踪"，仅扩展目标选择器 |
| D4 | 调度粒度 | **先粗后细**：中心只做任务分配 + 资源互斥 + 死锁/阻塞避免；工业级时空 A* 预约留后期 | 快速消除 4–10 车的路头死锁 |

**架构总原则**：中心决定"去哪 / 谁先走 / 谁让路"，车端决定"怎么去 / 怎么停"。车端是自治执行体——任务一旦下发，靠本地 `routes.json` + 本地规划即可跑完，中心失联不影响当前任务（正好满足"脱机可用"）。

---

## 1. 术语

- **配置束（config bundle）**：一组在 `map` 坐标系下相互耦合、必须同版本一起下发的配置。见 §3。
- **agent（车端代理）**：每台 X5 上的轻量进程，桥接中心与车端 ROS 服务/话题。不新增导航逻辑。
- **控制权 authority**：`controlled`（受控，中心可派单）/ `debug`（本地 Web 接管）/ `offline`（失联自治）。
- **资源互斥 resource**：路网中的节点与边，作为调度器加锁的对象。
- **blocked 标志**：车端"遇障停车等待"状态的上报，是中心判拆死锁的唯一触发信号。

---

## 2. 总体架构与通信矩阵

```text
                      ┌───────────────────────────────────────────┐
                      │            中心调度服务 (dispatch)          │
                      │  ┌──────────────┐   ┌───────────────────┐  │
                      │  │ 配置束仓库    │   │ 车队状态表          │  │
                      │  │ map/stn/route│   │ pose/占用/authority │  │
                      │  │ /marker 版本 │   │ blocked/task        │  │
                      │  └──────────────┘   └───────────────────┘  │
                      │        ┌──────────────────────────────┐    │
                      │        │ 资源互斥 + 死锁拆解 + 任务分配 │    │
                      │        └──────────────────────────────┘    │
                      └───────────────▲───────────────┬────────────┘
                       gRPC/MQTT-RPC  │(下行指令)      │(上行遥测 MQTT)
                     配置束下载 HTTP   │               │
        ┌─────────────────────────────┼───────────────┼─────────────────┐
        │                     MQTT / RPC Broker (中心侧或独立)            │
        └─────────────────────────────┼───────────────┼─────────────────┘
                                       │               │
        ══════ DDS 边界（车域内，不出车）══════          │
        ┌───────────────▼───────────────────────────────────────────────┐
        │ 车端 agent (每台 X5)                                            │
        │   下行: 调用 /delivery/* srv；上行: 订阅 TF/map、/route/status、  │
        │         /originbot_status 组装遥测                              │
        └───────┬───────────────────────────────────────────────────────┘
                │ 本机 ROS 2 (绝对名，不加 namespace)
   ┌────────────▼────────────────────────────────────────────┐
   │ 车端 ROS 子系统（现有，不改）                              │
   │  originbot_base(/cmd_vel,/odom,/imu,TF odom→base)        │
   │  STL06N(/scan) · AMCL/SLAM(TF map→odom) · Nav2           │
   │  route_follower(/route/navigate,/route/cancel,/cmd_vel)  │
   │  delivery_server(/delivery/*) · web_server:8080 · rosbridge:9090 │
   └─────────────────────────────────────────────────────────┘
        本地 Web 调试端 ── 连本机 rosbridge:9090（不经中心）
```

**通信矩阵**

| 链路 | 协议 | 方向 | 内容 | 频率 |
|---|---|---|---|---|
| 中心 ↔ agent | gRPC 或 MQTT-RPC | 下 | 任务/取消/goto/authority/配置束通知/远程子系统 | 事件 |
| agent → 中心 | MQTT（topic `fleet/<robot_id>/telemetry`） | 上 | 遥测（§5.2） | 2 Hz（可配），blocked 事件即时 |
| agent → 中心 | MQTT `fleet/<robot_id>/lwt` | 上 | 上下线（遗嘱） | 连接断开触发 |
| agent ↔ 中心 | HTTP(S) | 下 | 配置束下载（map.pgm/yaml、bundle.json） | 版本变更时 |
| 浏览器 → 车 | HTTP:8080 + WS:9090 | — | 本地 Web 调试（现有，不变） | — |
| 中心 → 浏览器 | （中心自建页面/接口） | — | 车队总览与人工干预 | — |

**关键约束**：中心**永不**订阅 DDS、**永不**直连车的 `/cmd_vel`。中心只能通过 agent 触发 `/delivery/*` 服务，保证与本地调试互斥可仲裁。

---

## 3. 配置束与地图同源

### 3.1 "共用地图"实为配置束（必须同版本原子下发）

以下四件全部在 `map` 系下表达，缺一即错位，必须捆成一个 bundle 版本一起推：

| 文件 | 内容 | 现位置（每台车本地） |
|---|---|---|
| `map.pgm` + `map.yaml` | 栅格地图 + 原点/分辨率 | `~/originbot_maps/<name>` |
| `stations.json` | 站点 map 系 x/y/yaw | `~/.originbot_delivery/config/` |
| `routes.json` | 路网节点/边（map 系） | `~/.originbot_delivery/config/` |
| `markers.json` | ArUco 标志位 map 系坐标 | `~/.originbot_delivery/config/` |

### 3.2 bundle.json（中心权威清单）

```jsonc
{
  "bundle_version": "b_2026_10_14_001",       // 每次建图发布递增
  "map": {
    "name": "canteen_floor1",
    "yaml_sha256": "...", "pgm_sha256": "...",
    "resolution": 0.05,                        // 全车队钉死（见 3.4）
    "origin": [-5.66, -5.39, 0.0]
  },
  "stations": { "sha256": "..." },
  "routes":   { "sha256": "...", "schema": 2 }, // schema 见 §4
  "markers":  { "sha256": "..." },
  "downloads": {                               // agent 按 sha256 校验后落盘
    "map_yaml": "https://<center>/bundles/b_2026_10_14_001/map.yaml",
    "map_pgm":  "https://<center>/bundles/b_2026_10_14_001/map.pgm",
    "stations": "https://<center>/bundles/b_2026_10_14_001/stations.json",
    "routes":   "https://<center>/bundles/b_2026_10_14_001/routes.json",
    "markers":  "https://<center>/bundles/b_2026_10_14_001/markers.json"
  },
  "published_by": "rb01",                      // 建图车
  "published_at": "2026-10-14T02:00:00Z"
}
```

### 3.3 发布与对齐流程

**建图发布（一次性，一台车做）**
1. Web：Start Base → Start Mapping（Cartographer/slam_toolbox）→ 推车建图，途经 ArUco 自动登记 `markers.json`。
2. Web：Save Map → `~/originbot_maps/<name>.{pgm,yaml}`；建站点、画路网。
3. 中心：`publish_bundle`（车端 agent 上传 map+stations+routes+markers，中心算 sha256、生成新 `bundle_version`）。

**对齐应用（每台车，含建图车）**
1. agent 收 `apply_bundle` → 下载 → 逐件 sha256 校验 → 原子写入 `~/.originbot_delivery/config/`（写临时文件+rename）。
2. 记录 `active_bundle_version`，遥测上报。
3. 冷启动位姿对齐（§3.3.4）→ 上线。

**3.3.4 冷启动初始位姿（这是 D2 的真正难点）**
车开机时物理位姿随机，AMCL 需 `initialpose`。三条路径，按优先级：
1. **ArUco 自动定位（推荐，多车场景必须默认开启）**：`aruco_localization` 识别已登记标志位 → 结合 `markers.json` 反推 map 位姿 → 发 `/initialpose`。
   - ⚠️ 现状**默认关闭**：`delivery_server.py:652` 需 `ENABLE_ARUCO=1` 才拉起，且空跑约占 2 核 CPU。车队方案须默认开，并评估 X5 CPU 余量（相机 640×480@10fps + PnP）。
   - 依赖共享 `markers.json` 与相机内参标定（`README.md:156` 章节）。
2. **SLAM→AMCL 交接**：同车建图后转导航，`delivery_server.py:762-793` 冻结 `map→odom` 换算 initialpose。适合建图车自己，不适合其它冷启车。
3. **人工兜底**：Web "设定位姿" → `/delivery/set_pose`。车队规模下不可扩展，仅作降级。

### 3.4 同源前必须钉死的一致性项（评审确认值）

| 项 | 现状（不一致） | 建议权威值 |
|---|---|---|
| 地图分辨率 | SLAM 0.10 / costmap 0.05 / `my_map.yaml` 0.05 | **0.05**（据 `maps/my_map.yaml`），SLAM 与 costmap 统一 |
| 雷达量程 | 注释称"统一 12m"，代码处处 20.0，`LaserScan.range_max` 硬编码 30 | 全车队**同一 `max_range`**；建图/AMCL/costmap 对齐同值 |
| 底盘串口 | launch 默认 `ttyS1`（`robot.launch.py:158`），代码默认 `ttyS3`，`originbot.launch.py` 不透传 `port_name` | 生成下发命令时**显式 pin `port_name`** |
| AMCL 权重 | `z_hit+z_short+z_max+z_rand=1.10`（注释误标 1.0） | 修正归一，避免多车定位行为不一 |

---

## 4. 路网数据模型扩展（`routes.json` schema v2）

向后兼容：仅加**可选字段**，缺省即等价旧行为（双向、容量 1、type 由是否有 station 推断）。沿用现有 `via`/`station` 的可选字段风格。

```jsonc
{
  "nodes": [
    {
      "name": "n1", "x": 1.2, "y": 0.5, "yaw": 0.0,
      "station": "kitchen",                 // 可选：绑定站点（route 模式按站名导航匹配它）
      "type": "station|avoidance|waypoint", // 新增可选；缺省: 有 station→station，否则 waypoint
      "cap": 1                              // 新增可选：该节点可同时占用车数（默认 1）
    }
  ],
  "edges": [
    {
      "a": "n1", "b": "n2",
      "via": [[2.0, 0.5]],                  // 现有：折线走廊
      "dir": "bi|a_to_b|b_to_a",            // 新增可选：单行道（默认 bi）
      "cap": 1                              // 新增可选：该边同时通行车数（默认 1）
    }
  ]
}
```

**对现实现的影响（最小化）**
- `route_follower.py:169-171` 现构造对称邻接（双向）。`dir=a_to_b/b_to_a` 需在 `RouteGraphData` 构造邻接时按方向裁剪。
- `cap`/`type` 对**车端执行无意义**（车只跟随被规划好的路径）；仅供中心资源互斥读取。故**车端改动可延后**，先让中心能读懂 v2 字段即可。
- 目标选择：`find_goal_node()`（`route_follower.py:177-184`）现按 `node.station` 匹配、回退按 `node.name`。**"去避让点"= 直接传 avoidance 节点名**，此回退已能命中——`goto(node)` 几乎零改造。

---

## 5. 车端 agent 契约

agent 不写导航逻辑，只做协议翻译 + 遥测组装。每条下行/上行用统一 envelope，便于 gRPC 与 MQTT-RPC 复用。

### 5.1 下行指令（中心 → agent）

统一请求：
```jsonc
{
  "cmd_id": "uuid", "robot_id": "rb03", "ts": "...",
  "cmd": "<见下表>", "args": { }
}
```
统一应答（同步/经 reply 通道）：
```jsonc
{ "cmd_id": "uuid", "ok": true, "code": "ACCEPTED|BUSY|REJECTED|UNREACHABLE|ERR", "message": "...", "data": { } }
```

| cmd | args | 映射到车端服务 | 备注 |
|---|---|---|---|
| `assign_task` | `{station, mode:"route"}` | `/delivery/navigate_to_station` | 生产默认 `route`；`free` 仅例外 |
| `cancel_task` | `{}` | `/delivery/cancel_navigation` | 按 `active_nav_mode` 分发（`delivery_server.py:1424`） |
| `goto` | `{kind:"station|node|point", value}` | `/route/navigate`（扩展目标选择器） | 让路/去 avoidance 节点；`node` 走 name 回退 |
| `set_authority` | `{authority:"controlled|debug"}` | 新增 `/delivery/set_authority`（§6） | 接管/交权 |
| `apply_bundle` | `{bundle_version}` | agent 内部：下载+落盘（§3.3） | 完成后回报 `active_bundle_version` |
| `remote` | `{subsystem, enable}` | `/delivery/control_system` | 远程起停子系统（base/mapping/navigation…） |
| `set_pose` | `{x,y,yaw}` | `/delivery/set_pose` | 人工兜底定位 |
| `estop` | `{}` | 见 §9 | 高优先级，agent 直发零速并置 mode |

### 5.2 上行遥测（agent → 中心，`fleet/<robot_id>/telemetry`，2 Hz）

```jsonc
{
  "robot_id": "rb03", "ts": "2026-10-14T06:12:03Z",
  "authority": "controlled",                 // controlled|debug|offline（§6）
  "mode": "executing",                        // idle|rotating|driving|aligning|arrived|blocked|failed|cancelled
  "pose": {                                   // 用 TF map→base_footprint，非裸 /odom（里程计会漂）
    "x": 3.42, "y": -1.08, "yaw": 1.57,
    "frame": "map", "valid": true, "stamp": "..."
  },
  "task": { "kind": "navigate", "station": "kitchen", "mode": "route" },
  "path": { "node": "n5", "edge": "n5->n2", "remaining_m": 1.20, "s_on_edge": 0.4 }, // 中心据此加路权锁
  "blocked": { "waiting": true, "reason": "obstacle", "since_sec": 6.2 },             // ★ 死锁触发信号，事件级即时上报
  "battery_v": 11.8,
  "estop": false,
  "nav_feedback": "剩余距离：1.20 米",
  "active_bundle_version": "b_2026_10_14_001",
  "health": { "base": true, "lidar": true, "amcl": true, "route": true, "tf_ok": true, "aruco": true }
}
```
数据来源：`/route/status`（`route_follower.py:440`）、TF `map→base_footprint`、`/originbot_status`（`delivery_server.py:601`、10 Hz）、`/route/path`、子系统存活用现有 `SUBSYSTEM_NODES` 探测（`delivery_server.py:289-325`）。
**`blocked.waiting` 必须独立成布尔字段**，不要只塞进 `nav_feedback` 文本——现 `route_follower` 遇障"无限等待、每 15s 提示一次"（`route_follower.py:551-599`），若不做死锁拆解，两台对头车必死锁。

### 5.3 在线检测（LWT = 上下线 = authority 联动）

- agent 连 broker 时设遗嘱：`fleet/<robot_id>/lwt` = `{"online":false}`，正常上线发布 `{"online":true}`。
- broker 检测到断连 → 中心将该车 `authority=offline`，**停止派单**，其已持有资源按策略回收（§7）。
- 该信号同时是"下线→本地 Web 调试接管"的触发：offline 时车端本地自治，Web 端不受中心约束可连本机 rosbridge 调试。

### 5.4 身份与 ID（现状缺失，需补）

- 现全工程**无 robot id、无 namespace**（节点名硬编码 `originbot_base`/`delivery_server`/`route_follower`）。
- 方案：**不改 ROS 名**（D1 红利）。仅给 agent 一个身份来源：`~/.originbot_delivery/config/robot_id`（中心分配或出厂烧录），agent 用它拼 MQTT topic 与遥测 `robot_id`。中心车队表以 `robot_id` 为主键。

---

## 6. 控制权与运行模式状态机

### 6.1 状态定义

```
authority ∈ { controlled, debug, offline }   // 谁有权决定这车动什么
execState ∈ { idle, executing, blocked, arrived, failed } // 车端任务进度（来自遥测 mode）
```

| authority | 中心 | 本地 Web | 进入条件 |
|---|---|---|---|
| `controlled` | 可派单/取消/让路 | 只读监控；"接管"按钮 | LWT online 且中心持有控制权 |
| `debug` | 不派单，仅接收遥测 | 完全可手动（键盘/摇杆） | 操作员点"接管"（= `set_authority(debug)`）|
| `offline` | 不派单，标记失联 | 完全可用（连本机 9090） | LWT 断连 / broker 不可达 |

### 6.2 `/cmd_vel` 仲裁（安全前提，现状为软约定）

**问题**：Web 键盘、`route_follower`、Nav2 **都写同一个 `/cmd_vel`**，无 mux；互斥仅靠前端约定（`README.md:140`、`route_follower.py:19-20`）。多车 + 中心介入后必须硬仲裁。

**方案（建议引入 `twist_mux`，优先级自上而下）**：
```
优先级 1: estop（零速）              ← 中心/本地急停
优先级 2: route_follower (controlled/executing)
优先级 3: Nav2 (free 模式，若启用)
优先级 4: Web 键盘 (debug)
```
- **阶段 0 可先用软门控降级实现**：`delivery_server`/`route_follower` 读一个 authority 参数，非本权限来源一律丢弃不写 `/cmd_vel`；`twist_mux` 作为阶段 1 加固。
- 接管切换必须是**原子的**：`debug→controlled` 时先让 Web 停发并零速、`route_follower` 交权，再允许中心派单。反之亦然。
- 新增服务 `/delivery/set_authority {authority} → {ok,message}`，由 agent 调，`delivery_server` 维护单一真值并广播到 `/delivery/authority`（供 Web/前端灰化按钮）。

### 6.3 状态迁移（评审要点）

```
[boot] --LWT online--> controlled
[boot] --LWT offline--> offline
controlled --set_authority(debug)/操作员接管--> debug
debug --操作员交权--> controlled
controlled --broker 断连--> offline （完成当前本地任务，不自派新单）
offline --重连且中心确认--> controlled
任何态 --estop--> blocked/停；解除后回原 authority
```

---

## 7. 调度器核心（粗粒度：任务分配 + 资源互斥 + 死锁拆解）

中心内部，不外露。输入 = 车队状态表 + bundle 路网（含 `dir/cap/type`）。

### 7.1 资源模型
- 资源 = 所有 `node` 与 `edge`；每资源容量取 `cap`（默认 1）。
- 一辆车在某时刻**持有**：所在节点 + 正在通行的边 + 已预约的下一节点（到达后释放前节点）。
- 车端遥测 `path.node/edge` 即"当前持有"，中心据此建占用表。

### 7.2 分配与放行
1. 收到取餐/送餐请求 → 用与车端一致的图模型算候选路径（中心侧规划，仅用于路权预排，真正行驶仍由车端 `route_follower` 沿 `routes.json` 走）。
2. **逐段预约**：只有下一段资源空闲才 `assign_task`/放行；冲突则排队。
3. 让路：当 A 阻塞且 B 需通过，中心对占位车下发 `goto(avoidance 节点)`，把它指挥到最近 `type=avoidance` 节点停靠。

### 7.3 死锁/阻塞拆解（本方案要解决的正是无限等待）
- **触发**：某车 `blocked.waiting=true` 持续 > `blocked_timeout`（建议 20–30s）。
- **检测**：占用图找环（含两车在同一无向边对头）；环上任选非关键车作让步者。
- **动作**：对让步者 `goto(avoidance)`；无空避让点则沿路回退到最近 `cap>1` 或空闲节点。
- **优先级**：载物/任务紧急者优先，其余让路；同优先级 FIFO 排队。

### 7.4 车端配合的改造点（尽量小）
- `route_follower`：无限等待 → 等待超时后把 `blocked` 上报并保持停车，不自行绕行（中心会重排）。可选：新增 `block_report` 事件通道。
- `routes.json` 加载：车端只读跟随，`dir/cap` 可不消费；`plan_route` 需支持 `goto(node name)`（已由 name 回退覆盖）。

---

## 8. 接口清单汇总

**中心对上层（运营/取餐 UI）**：`request_delivery(robot, station)`、`cancel(robot)`、`get_fleet_state()`、`publish_bundle(...)`、`take_control(give|take)`。

**中心 ↔ agent（本草案 §5）**：下行 `assign_task/cancel_task/goto/set_authority/apply_bundle/remote/set_pose/estop`；上行 `telemetry(2Hz)`、`lwt`、`blocked` 事件。

**agent → 车端 ROS（全部现成，除两个新增）**：

| ROS 服务 | 状态 |
|---|---|
| `/delivery/navigate_to_station` (NavigateToStation, `mode`) | ✅ 现成 |
| `/delivery/cancel_navigation` | ✅ 现成 |
| `/route/navigate` (NavigateToStation) | ✅ 现成（`goto node` 需 name 回退，已覆盖） |
| `/delivery/control_system` | ✅ 现成 |
| `/delivery/set_pose` | ✅ 现成 |
| `/delivery/station`、`/delivery/route`(RouteGraph)、`/delivery/save_map`、`/delivery/map_file` | ✅ 现成（建图发布期用） |
| `/delivery/set_authority` + `/delivery/authority` topic | 🆕 新增（§6） |
| `/delivery/estop` | 🆕 新增（§9） |
| `twist_mux`（`/cmd_vel` 优先级仲裁） | 🆕 引入（§6.2，阶段 1） |

---

## 9. 非功能约束与安全

- **失联自治**：`offline` 车跑完当前本地任务，停在最后目标节点，不再自派新单；中心标记失联并回收其预留资源以免阻塞他车。
- **看门狗现状**：底盘已有 0.6s 无有效非零速自动停车（`originbot_base.cpp:739-746`，`auto_stop_on` 默认 true）。注意其"非零才续期"逻辑；estop 走显式零速 + 停 route_follower。
- **`/cmd_vel` 多写者**：`twist_mux` 落地前，authority 软门控是**上线前置条件**（否则中心与本地调试可能同时下发）。
- **单一 `map→odom` 发布者**：AMCL 与 `ekf_map` 互斥（`originbot_delivery/README.md:34-40`），车队配置须锁定用 AMCL 路径，禁混用。
- **estop 语义**：中心/本地均可触发，agent 优先级最高；触发后 `mode=blocked`、清任务、上报，恢复需人工确认。
- **资源回收超时**：中心对 `blocked`/失联车的占用加锁须带 TTL，崩溃恢复时重建车队状态表。

---

## 10. 契约 → 落地阶段映射

| 阶段 | 目标 | 涉及本草案 |
|---|---|---|
| **0 地基（安全）** | 控制权威 + `/cmd_vel` 软门控 + `blocked` 事件上报 | §5.2(blocked)、§6.2(门控)、`/delivery/set_authority` |
| **1 同源+仲裁** | 配置束版本+下发校验、ArUco 默认开、引入 twist_mux | §3、§4、§6.2(mux)、§3.4 一致性钉死 |
| **2 可见** | agent 遥测 + 中心车队状态表 + 同图总览（被动） | §5、§7.1(占用表) |
| **3 防阻塞** | 资源互斥 + 避让点 + 超时让路，车载无限等待降级 | §4(dir/cap/avoidance)、§7.2/7.3 |
| **4 精细化(可选)** | 时空路权预约（time-window A*） | 替换 §7 粗排 |

---

## 11. 评审未决问题

1. **X5 CPU/带宽余量**：默认开 ArUco（约 2 核）+ 相机流 + 中心 agent，单台是否吃得消？是否降帧/降分辨率？
2. **broker 部署位置**：独立服务器 / 复用中心？WiFi 弱网下 MQTT keepalive 与 LWT 判死阈值取值（建议 keepalive 5s、3 次未响应判死）。
3. **中心是否也做规划**：§7.2 中心侧仅"路权预排"，实际路径仍由车端 `route_follower` 生成，二者用**同一份 `routes.json`**——须保证图模型与 Dijkstra 语义严格一致，否则中心放行与车端实走可能错位。备选：中心只按车端上报 `path` 做占用，不自算路径。
4. **`goto(point)` 是否需要**：D3 允许任意节点作避让点，若坚持"只往节点让"，可砍掉 point 目标，`goto` 仅 `kind:station|node`，改动最小。
5. **多地图**：是否只有"单一食堂地图"一份 bundle 在用？若跨楼层/多区，需扩展 bundle 多版本 + 车跨 bundle 切换/重定位流程。
6. **建图期互斥**：受控车队中禁止随手重新建图（会改 map 原点）。`remote(mapping,on)` 是否需中心授权令牌？
7. **`dir` 双向改单向的回溯**：既有 `routes.json` 全 `bi`，加单向是纯运营决策；需现场实测走廊宽度是否支撑对头让路。

---

*本文件为设计草案，不含实现。评审通过后按 §10 阶段推进；阶段 0 接口（`set_authority`/`blocked`/门控）定稿即可并行开工。*
