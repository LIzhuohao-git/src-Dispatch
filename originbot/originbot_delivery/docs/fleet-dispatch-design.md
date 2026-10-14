# OriginBot 车队调度服务 — 接口与数据模型草案

> 状态：**评审基线 v0.3**（未实现）。用于确定中心调度服务、车端 agent、配置束（地图同源）与路网数据模型的对外契约。评审定稿后再进入编码。
>
> v0.3 变更：回填 §11 四项决策（D8 冷启动=boot_pose+人工兜底 / D9 单一地图 / D10 中心不自算路径 / D11 重建图需中心令牌）；§3.3.4 锁定冷启动方案；§7.2 重写为"占用跟踪 + 用上报 plan 预占"，并新增 §5.2 `plan` 字段与 §7.5 权衡说明。
>
> 适用：4–10 台同型号 OriginBot（RDK X5，ROS 2 Humble），生产采用**轨道路网导航**（非自由导航），共用同一地图与站点；脱机/下线时保留每车本地 Web 调试能力。

---

## 0. 已锁定前置决策

| # | 决策 | 结论 | 关键收益 / 代价 |
|---|---|---|---|
| D1 | 中心↔车通信底座 | **B 方案：非 DDS**（下行 gRPC 或 MQTT-RPC，上行 MQTT 遥测），中心不加入 DDS 域 | 车端 ROS 图**完全无需改造命名空间/`ROS_DOMAIN_ID`**；现有 `/odom /cmd_vel /scan /map` 绝对名与硬编码节点名保持不变 |
| D2 | 地图同源 | **一车建图 → 上传中心 → 中心下发全部车 → 每车 AMCL 定位；冷启动初始位姿靠 boot_pose（默认不用 ArUco，见 D5/D8/§3.3.4）** | 各车 `map` 系原点天然一致（同一 `.pgm/.yaml` 字节 + 对同一图做 AMCL 定位） |
| D3 | 避让点语义 | 任意节点可声明为避让点；由调度器负责不死锁，可设多避让点 | 复用 `route_follower` 现成的"投影上车→Dijkstra→纯跟踪"，仅扩展目标选择器 |
| D4 | 调度粒度 | **先粗后细**：中心只做任务分配 + 资源互斥 + 死锁/阻塞避免；工业级时空 A* 预约留后期 | 快速消除 4–10 车的路头死锁 |
| D5 | ArUco 使用 | **默认关闭**（原用于一次性标定地图，现导航靠 AMCL+scan 已达标）；ArUco 保留为**可选**周期校正开关 | 省 ~2 核 CPU + 去掉相机/PnP 依赖；**代价**：放弃持续漂移校正，需靠回桩/人工兜底 |
| D6 | 车身份 | 新增 per-robot `robot_id` 配置（§5.4），**不改 ROS 节点/话题名** | 中心以 `robot_id` 为主键；DDS 仍绝对名（D1 红利） |
| D7 | `/cmd_vel` 仲裁 | **authority 门控为上线前置条件**（§6.2），阶段 0 软门控、阶段 1 引入 `twist_mux` | 消除"中心派单"与"本地调试"同时下发 |
| D8 | 冷启动定位 | **boot_pose 自动注入 + 人工 Web `set_pose` 兜底**（§3.3.4）；不采用 AMCL 全局重定位作默认 | 无 ArUco 也可扩展；前提：车尽量定点待命/回桩 |
| D9 | 地图数量 | **单一地图、单套 bundle 全车队共用** | 同源一次性解决；跨楼层需另议（未来） |
| D10 | 中心是否自算路径 | **否**：中心只按车端上报的 `plan`（完整计划路径）跟踪与预占资源，**不独立跑 Dijkstra**（§7.2/§7.5） | 单一真源=车端 `route_follower`，绝不放行/实走错位；**代价**：中心不会算备选路径，冲突只能串行/让路 |
| D11 | 重建图权限 | **受控态下 `remote(mapping)` 需中心令牌**（§9） | 防某车随手重绘导致全车队 map 原点错位 |

**架构总原则**：中心决定"去哪 / 谁先走 / 谁让路"，车端决定"怎么去 / 怎么走 / 怎么停"。车端是自治执行体——任务一旦下发，靠本地 `routes.json` + 本地规划即可跑完，中心失联不影响当前任务（正好满足"脱机可用"）。

---

## 1. 术语

- **配置束（config bundle）**：一组在 `map` 坐标系下相互耦合、必须同版本一起下发的配置。见 §3。
- **agent（车端代理）**：每台 X5 上的轻量进程，桥接中心与车端 ROS 服务/话题。不新增导航逻辑。
- **控制权 authority**：`controlled`（受控，中心可派单）/ `debug`（本地 Web 接管）/ `offline`（失联自治）。
- **资源互斥 resource**：路网中的节点与边，作为调度器加锁的对象。
- **blocked 标志**：车端"遇障停车等待"状态的上报，是中心判拆死锁的唯一触发信号。
- **boot_pose**：每台车已知的开机/回桩位姿（map 系），无 ArUco 时用于注入 AMCL 初始位姿。
- **plan（上报计划路径）**：车端规划出的、本次任务将要依次经过的节点/边序列；中心据此提前预占资源。

---

## 2. 总体架构与通信矩阵

```text
                      ┌───────────────────────────────────────────┐
                      │            中心调度服务 (dispatch)          │
                      │  ┌──────────────┐   ┌───────────────────┐  │
                      │  │ 配置束仓库    │   │ 车队状态表          │  │
                      │  │ map/stn/route│   │ pose/占用/authority │  │
                      │  │ (/marker) 版本│   │ blocked/task/plan   │  │
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
        │   /route/path、/originbot_status 组装遥测；上电按 boot_pose 注   │
        │   initialpose                                                    │
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
| 中心 ↔ agent | gRPC 或 MQTT-RPC | 下 | 任务/取消/goto/authority/配置束通知/远程子系统/estop/建图令牌 | 事件 |
| agent → 中心 | MQTT（topic `fleet/<robot_id>/telemetry`） | 上 | 遥测（§5.2，含 plan） | 2 Hz（可配），blocked 事件即时 |
| agent → 中心 | MQTT `fleet/<robot_id>/lwt` | 上 | 上下线（遗嘱） | 连接断开触发 |
| agent ↔ 中心 | HTTP(S) | 下 | 配置束下载（map.pgm/yaml、bundle.json） | 版本变更时 |
| 浏览器 → 车 | HTTP:8080 + WS:9090 | — | 本地 Web 调试（现有，不变） | — |
| 中心 → 浏览器 | （中心自建页面/接口） | — | 车队总览与人工干预 | — |

**关键约束**：中心**永不**订阅 DDS、**永不**直连车的 `/cmd_vel`。中心只能通过 agent 触发 `/delivery/*` 服务，保证与本地调试互斥可仲裁。

---

## 3. 配置束与地图同源

### 3.1 "共用地图"实为配置束（必须同版本原子下发）

以下**三件必需**、全部在 `map` 系下表达，缺一即错位，必须捆成一个 bundle 版本一起推；第四件（markers）**可选**——本车队 ArUco 默认关闭（D5），仅在需要时随附：

| 文件 | 内容 | 必需性 | 现位置（每台车本地） |
|---|---|---|---|
| `map.pgm` + `map.yaml` | 栅格地图 + 原点/分辨率 | 必需 | `~/originbot_maps/<name>` |
| `stations.json` | 站点 map 系 x/y/yaw | 必需 | `~/.originbot_delivery/config/` |
| `routes.json` | 路网节点/边（map 系） | 必需 | `~/.originbot_delivery/config/` |
| `markers.json` | ArUco 标志位 map 系坐标 | **可选**（ArUco 关时可不下发） | `~/.originbot_delivery/config/` |

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
  "markers":  { "sha256": "..." },             // 可选
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
1. Web：Start Base → Start Mapping（Cartographer/slam_toolbox）→ 推车建图（可选：建图期临时开 ArUco 登记 `markers.json`，供后续人工/周期校正）。
2. Web：Save Map → `~/originbot_maps/<name>.{pgm,yaml}`；建站点、画路网。
3. 中心：`publish_bundle`（车端 agent 上传 map+stations+routes[+markers]，中心算 sha256、生成新 `bundle_version`）。

**对齐应用（每台车，含建图车）**
1. agent 收 `apply_bundle` → 下载 → 逐件 sha256 校验 → 原子写入 `~/.originbot_delivery/config/`（写临时文件+rename）。
2. 记录 `active_bundle_version`，遥测上报。
3. 冷启动位姿对齐（§3.3.4）→ 上线。

### 3.3.4 冷启动初始位姿（已定：boot_pose + 人工兜底，D8）
车开机时物理位姿随机，AMCL 需 `initialpose`。车队**不用 ArUco 做默认定位**（D5），方案：

1. **boot_pose 自动注入（主路径）**：每台车在自身配置（§3.5/§5.4）存 `boot_pose {x,y,yaw}`——该车充电桩/待命位在 map 系坐标。上电或重连时 agent 自动发 `/initialpose`，AMCL 以该位姿起粒。
   - 前提：**车尽量在已知停位上电**（食堂车队定点待命/回桩）。
2. **人工 Web 兜底**：车被搬动/定位丢失/停放位未知时，操作员本地 Web "设定位姿" → `/delivery/set_pose`（`SetPose.srv` 现成）；中心亦可代下发 `set_pose`（§5.1）。
3. 定位丢失判定与再兜底：`health.tf_ok=false` 或长时间 `blocked.reason=localization` 时，遥测告警，提示就近人工设位或回桩。

**不做**：AMCL `/global_localization` 全图重定位（对称走廊易错层，不设为默认；如未来需要再作可选）。

**无 ArUco 的代价与缓解**：长期漂移仅靠 AMCL+scan——防碰依据车上报的 map 位姿，若漂移则中心占用判断退化。缓解：① 定期回桩（回桩即按 boot_pose 重置）；② 保留 ArUco 作**可选**周期校正开关（默认关，出问题再开）。

### 3.4 同源前必须钉死的一致性项（评审确认值）

| 项 | 现状（不一致） | 建议权威值 |
|---|---|---|
| 地图分辨率 | SLAM 0.10 / costmap 0.05 / `my_map.yaml` 0.05 | **0.05**（据 `maps/my_map.yaml`），SLAM 与 costmap 统一 |
| 雷达量程 | 注释称"统一 12m"，代码处处 20.0，`LaserScan.range_max` 硬编码 30 | 全车队**同一 `max_range`**；建图/AMCL/costmap 对齐同值 |
| 底盘串口 | launch 默认 `ttyS1`（`robot.launch.py:158`），代码默认 `ttyS3`，`originbot.launch.py` 不透传 `port_name` | 生成下发命令时**显式 pin `port_name`** |
| AMCL 权重 | `z_hit+z_short+z_max+z_rand=1.10`（注释误标 1.0） | 修正归一，避免多车定位行为不一 |

### 3.5 公共 bundle（车队共用）vs per-robot 配置（每台独有）

关键区分：**同源的是"地图坐标系相关量"（全车队一份），不共的是"这辆车自己是谁、停在哪"（每台一份）**。

| 归属 | 内容 | 存放 | 下发方 |
|---|---|---|---|
| 公共 bundle | map / stations / routes /（markers） | 中心仓库；车端 `~/.originbot_delivery/config/` | 中心按版本推送 |
| **per-robot** | `robot_id`、`boot_pose`、`default_authority`、broker 凭证 | 车端 `~/.originbot_delivery/config/robot_identity.json`（§5.4） | 出厂烧录 / 首次上线中心分配 |

boot_pose 绝不进 bundle：它是"这台车的桩位在 map 系的坐标"，每车不同；同 map 原点下每车各自标定自己的停位即可。

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
- `cap`/`type` 对**车端执行无意义**（车只跟随被规划好的路径）；仅供中心资源互斥读取与预占（§7.5）。故**车端改动可延后**，先让中心能读懂 v2 字段即可。
- 目标选择：`find_goal_node()`（`route_follower.py:177-184`）现按 `node.station` 匹配、回退按 `node.name`。**"去避让点"= 直接传 avoidance 节点名**，此回退已能命中——`goto(node)` 几乎零改造。

---

## 5. 车端 agent 契约

agent 不写导航逻辑，只做协议翻译 + 遥测组装（含 `/route/path` 转 plan）+ 上电注入 boot_pose。每条下行/上行用统一 envelope，便于 gRPC 与 MQTT-RPC 复用。

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
| `goto` | `{kind:"station|node", value}` | `/route/navigate`（node 走 name 回退） | 让路/去 avoidance 节点（point 目标见 §11 待决） |
| `set_authority` | `{authority:"controlled|debug"}` | 新增 `/delivery/set_authority`（§6） | 接管/交权 |
| `apply_bundle` | `{bundle_version}` | agent 内部：下载+落盘（§3.3） | 完成后回报 `active_bundle_version` |
| `remote` | `{subsystem, enable, token?}` | `/delivery/control_system` | 远程起停子系统；`mapping` 在 controlled 下须带令牌（D11） |
| `grant_remap_token` | `{robot_id}` → `{token}` | agent 暂存，放行一次 `remote(mapping)` | 见 §9；一次性/短时效 |
| `set_pose` | `{x,y,yaw}` | `/delivery/set_pose` | 人工/中心兜底定位（§3.3.4） |
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
  "path": { "node": "n5", "edge": "n5->n2", "remaining_m": 1.20, "s_on_edge": 0.4 }, // 当前占用
  "plan": ["n5","n5->n2","n2","n2->n8","n8"],  // ★ 本次任务将依次经过的节点/边（来自 /route/path），供中心预占（§7.5）
  "blocked": { "waiting": true, "reason": "obstacle", "since_sec": 6.2 },             // ★ 死锁触发信号，事件级即时上报
  "battery_v": 11.8,
  "estop": false,
  "nav_feedback": "剩余距离：1.20 米",
  "active_bundle_version": "b_2026_10_14_001",
  "health": { "base": true, "lidar": true, "amcl": true, "route": true, "tf_ok": true }
}
```
数据来源：`/route/status`（`route_follower.py:440`）、`/route/path`（nav_msgs/Path，`route_follower.py:444`，agent 折成节点/边序列填 `plan`）、TF `map→base_footprint`、`/originbot_status`（`delivery_server.py:601`、10 Hz）、子系统存活用现有 `SUBSYSTEM_NODES` 探测（`delivery_server.py:289-325`）。
**`blocked.waiting` 必须独立成布尔字段**（不要只塞进 `nav_feedback` 文本）——现 `route_follower` 遇障"无限等待、每 15s 提示一次"（`route_follower.py:551-599`），若不做死锁拆解，两台对头车必死锁。

### 5.3 在线检测（LWT = 上下线 = authority 联动）

- agent 连 broker 时设遗嘱：`fleet/<robot_id>/lwt` = `{"online":false}`，正常上线发布 `{"online":true}`。
- broker 检测到断连 → 中心将该车 `authority=offline`，**停止派单**，其已持有资源按策略回收（§7）。
- 该信号同时是"下线→本地 Web 调试接管"的触发：offline 时车端本地自治，Web 端不受中心约束可连本机 rosbridge 调试。

### 5.4 robot_id 身份配置（新增功能，D6）

**现状**：全工程无 robot_id / namespace，节点名硬编码（`originbot_base`/`delivery_server`/`route_follower`）。**D1 红利**：不改 ROS 名即可多车并存（各车在自己板上，中心走非 DDS）。**新增**一个轻量 per-robot 身份配置：

- 文件：`~/.originbot_delivery/config/robot_identity.json`
  ```jsonc
  {
    "robot_id": "rb03",
    "boot_pose": { "x": 0.20, "y": -1.50, "yaw": 1.57 },   // map 系，该车停位（§3.3.4，D8）
    "default_authority": "controlled"
  }
  ```
- 来源：出厂烧录，或首次上线由中心分配（中心保证 `robot_id` 全局唯一）。
- agent 用途：拼 MQTT topic `fleet/<robot_id>/...`、填遥测 `robot_id`、上电时按 `boot_pose` 注 `initialpose`。
- 中心车队表以 `robot_id` 为主键。
- 阶段 0 顺带实现：`set_robot_id` 写入工具（Web 面板或 CLI），中心登记时做唯一性校验。

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

### 6.2 `/cmd_vel` 仲裁（安全前提，上线前置 — D7）

**问题**：Web 键盘、`route_follower`、Nav2 **都写同一个 `/cmd_vel`**，无 mux；互斥仅靠前端约定（`README.md:140`、`route_follower.py:19-20`）。多车 + 中心介入后必须硬仲裁。

**方案（优先级自上而下）**：
```
优先级 1: estop（零速）              ← 中心/本地急停
优先级 2: route_follower (controlled/executing)
优先级 3: Nav2 (free 模式，若启用)
优先级 4: Web 键盘 (debug)
```
- **阶段 0 先用软门控（已确认作为前置）**：`delivery_server`/`route_follower` 读 authority，非本权限来源一律丢弃不写 `/cmd_vel`。
- **阶段 1 引入 `twist_mux`** 做优先级抢占与超时回退，加固为硬仲裁。
- 接管切换必须**原子**：`debug→controlled` 时先让 Web 停发并零速、`route_follower` 交权，再允许中心派单；反之亦然。
- 新增服务 `/delivery/set_authority {authority} → {ok,message}`，由 agent 调；`delivery_server` 维护单一真值并广播 `/delivery/authority`（供前端灰化按钮）。

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

## 7. 调度器核心（粗粒度：任务分配 + 资源互斥 + 死锁拆解，D4/D10）

中心内部，不外露。输入 = 车队状态表（含每车 `plan` + `path`）+ bundle 路网（含 `dir/cap/type`）。**中心不跑 Dijkstra（D10）**，只读车端上报的 `plan` 做占用跟踪与预占。

### 7.1 资源模型
- 资源 = 所有 `node` 与 `edge`；每资源容量取 `cap`（默认 1）。
- 一辆车在某时刻**持有**：`plan` 中已获准、尚未走完的节点/边；当前实际占用见 `path`。
- 到达即释放：车走完某节点/边后从 `plan` 剩余段移除，中心回收。

### 7.2 分配与放行（占用跟踪，非中心规划）
1. 收到取餐/送餐请求 → `assign_task(robot, station)`。
2. 车端 `route_follower` 本地 Dijkstra 规划，产出 `/route/path` → agent 上报 `plan`（节点/边序列）。
3. 中心对 `plan` **逐资源比对占用表**：
   - 全段空闲 → 一次性预占整条 `plan`，放行；
   - 与他车 `plan`/`path` 冲突 → 任务入队等待，或（对头/死锁）转 §7.3 让路。
4. 中心不生成路径，只在"放行/排队/让路/取消"四个动作里选。

### 7.3 死锁/阻塞拆解（本方案要解决的正是无限等待）
- **触发**：某车 `blocked.waiting=true` 持续 > `blocked_timeout`（建议 20–30s），或 `plan` 冲突形成占用环。
- **检测**：占用图找环（含两车在同一无向边对头）。
- **动作**：环上选非关键车作让步者 → `goto(avoidance 节点)`；无空避让点则沿其已走边回退到最近 `cap>1`/空闲节点。
- **优先级**：载物/任务紧急者优先，其余让路；同优先级 FIFO。

### 7.4 车端配合的改造点（尽量小）
- `route_follower`：无限等待 → 等待超时后把 `blocked` 上报并保持停车，不自行绕行（中心会重排）。
- `plan` 上报：agent 把 `/route/path`（`nav_msgs/Path`）映射为节点/边序列；建议在 `routes.json` 已含拓扑的前提下，直接按路径点匹配所属边/节点。
- `routes.json` 加载：车端只读跟随，`dir/cap/type` 车端可不消费，仅供中心。

### 7.5 权衡（"中心不自算"换来什么、失去什么，D10）
- **得到**：单一真源=车端 `route_follower`，杜绝"中心放行路径 ≠ 车实走路径"的错位；中心实现与运维大幅简化。
- **失去**：中心**不会算备选路径**。若某车唯一通路被占且无避让点，中心只能**排队等待**或**取消/改派站点**，不能自动绕行。粗粒度阶段可接受；细粒度（中心侧或车侧多路径规划）留 D4 后期。
- **前提**：预占依赖车端如实、及时上报 `plan`；`plan` 变更后（重规划/让路）须重新申请占用。

---

## 8. 接口清单汇总

**中心对上层（运营/取餐 UI）**：`request_delivery(robot, station)`、`cancel(robot)`、`get_fleet_state()`、`publish_bundle(...)`、`take_control(give|take)`、`grant_remap_token(...)`。

**中心 ↔ agent（本草案 §5）**：下行 `assign_task/cancel_task/goto/set_authority/apply_bundle/remote/set_pose/estop/grant_remap_token`；上行 `telemetry(2Hz，含 plan)`、`lwt`、`blocked` 事件。

**agent → 车端 ROS（全部现成，除新增）**：

| ROS 服务 | 状态 |
|---|---|
| `/delivery/navigate_to_station` (NavigateToStation, `mode`) | ✅ 现成 |
| `/delivery/cancel_navigation` | ✅ 现成 |
| `/route/navigate` (NavigateToStation) | ✅ 现成（`goto node` 走 name 回退已覆盖） |
| `/delivery/control_system` | ✅ 现成 |
| `/delivery/set_pose` | ✅ 现成 |
| `/delivery/station`、`/delivery/route`(RouteGraph)、`/delivery/save_map`、`/delivery/map_file` | ✅ 现成（建图发布期用） |
| `/delivery/set_authority` + `/delivery/authority` topic | 🆕 新增（§6） |
| `/delivery/estop` | 🆕 新增（§9） |
| `robot_identity.json`（robot_id/boot_pose）+ `set_robot_id` 工具 | 🆕 新增（§5.4，D6/D8） |
| `twist_mux`（`/cmd_vel` 优先级仲裁） | 🆕 引入（§6.2，阶段 1） |

---

## 9. 非功能约束与安全

- **失联自治**：`offline` 车跑完当前本地任务，停在最后目标节点，不再自派新单；中心标记失联并回收其预留资源以免阻塞他车。
- **看门狗现状**：底盘已有 0.6s 无有效非零速自动停车（`originbot_base.cpp:739-746`，`auto_stop_on` 默认 true）。注意其"非零才续期"逻辑；estop 走显式零速 + 停 route_follower。
- **`/cmd_vel` 多写者**：`twist_mux` 落地前，authority 软门控是**上线前置条件**（D7）。
- **单一 `map→odom` 发布者**：AMCL 与 `ekf_map` 互斥（`originbot_delivery/README.md:34-40`），车队配置须锁定用 AMCL 路径，禁混用。
- **重建图令牌（D11）**：受控态下 `remote(mapping,on)` 须携带 `grant_remap_token` 下发的一次性短时效令牌；无令牌则车端 `delivery_server` 拒绝启动建图，防止随手重绘改 map 原点、打乱全车队同源。仅 `debug`/`offline` 本地建图无需令牌，但建完须经中心 `publish_bundle` 才影响车队。
- **estop 语义**：中心/本地均可触发，agent 优先级最高；触发后 `mode=blocked`、清任务、上报，恢复需人工确认。
- **资源回收超时**：中心对 `blocked`/失联车的占用加锁须带 TTL，崩溃恢复时重建车队状态表。
- **无 ArUco 的定位退化**：见 §3.3.4；靠 boot_pose 回桩重置 + 人工兜底。

---

## 10. 契约 → 落地阶段映射

| 阶段 | 目标 | 涉及本草案 |
|---|---|---|
| **0 地基（安全，前置）** | robot_id/boot_pose 功能、控制权威 + `/cmd_vel` 软门控、`blocked`+`plan` 上报 | §5.4、§3.3.4、§5.2(blocked/plan)、§6.2(门控)、`/delivery/set_authority` |
| **1 同源+硬仲裁** | 配置束版本+下发校验、引入 twist_mux、重建图令牌、一致性钉死 | §3、§4、§6.2(mux)、§9(令牌)、§3.4 |
| **2 可见** | agent 遥测 + 中心车队状态表 + 同图总览（被动） | §5、§7.1(占用表) |
| **3 防阻塞** | 资源互斥 + 避让点 + 超时让路，车载无限等待降级 | §4(dir/cap/avoidance)、§7.2–7.5 |
| **4 精细化(可选)** | 时空路权预约（time-window A*），或引入中心/车侧多路径规划补 D10 短板 | 扩展 §7 |

---

## 11. 评审事项：已决 / 待决

**已决（v0.2–v0.3 回填）**
- ArUco 默认关，冷启动改用 boot_pose（D5/D8/§3.3.4）。
- 车身份：新增 per-robot `robot_id`/boot_pose 配置，不改 ROS 名（D6/§5.4）。
- `/cmd_vel`：authority 门控为上线前置（D7/§6.2）。
- 地图数量：**单一地图、单套 bundle**（D9）。
- 中心规划：**只跟踪占用 + 用上报 plan 预占，不自算 Dijkstra**（D10/§7.2/§7.5）。
- 重建图：**受控态需中心令牌**（D11/§9）。

**待决（非阻塞，可在阶段推进中定）**
1. **`goto` 是否需要 point 目标**（§5.1）：坚持"只往 avoidance 节点让"则 `goto` 仅 `kind:station|node`，车端零改造；若要"任意坐标让路"需扩 `plan_route` 接受临时点。
2. **broker 部署与阈值**（§5.3/§9）：独立服务器/复用中心；MQTT keepalive 与 LWT 判死阈值（建议 keepalive 5s、3 次未响应判死）。
3. **`dir` 单向化**（§4）：既有 `routes.json` 全 `bi`，改单行是纯运营决策，需现场实测走廊宽度是否支撑对头让路。

---

*本文件为设计草案，不含实现。§0/§3/§7 主干已定；剩余 §11 三项非阻塞，可随阶段 0/1 推进并行敲定。阶段 0 接口（robot_id/boot_pose/`set_authority`/`blocked`/`plan`/门控）即可开工。*
