const state = {
  ros: null,
  connected: false,
  map: null,
  robotPose: null,
  imuData: null,
  stations: [],
  markers: [],
  manualMode: false,
  pressedKeys: new Set(),
  controlMode: "keyboard",              // 手动控制输入方式：keyboard / joystick
  joystick: { active: false, x: 0, y: 0 }, // 遥杆归一化偏移：x 右+，y 下+
  odomPose: null,      // /odom 的最新位姿 {t, x, y, yaw}（odom 系，t=ROS 时间戳秒）
  odomPoseBuf: [],     // /odom 位姿环形缓冲（含 ROS 时间戳），供点云按扫描时刻取位姿
  mapToOdom: null,     // 反解出的 map→odom 变换 {x, y, yaw}，用于位姿外推
  pickTarget: null,
  pickStartCanvas: null,
  pickCurrentCanvas: null,
  navigationTarget: null,
  lastArucoStatusTime: 0,
  lastArucoIds: "",
  lastCameraImageTime: 0,
  scan: null,          // 激光雷达最新一帧 LaserScan
  scanCloud: [],       // 变换到地图(map)坐标系后的点 [ [xWorld, yWorld], ... ]
  scanStamp: 0,        // 最近一次收到 /scan 的时间戳(ms)
  showScan: true,      // 是否显示激光点云
  routeGraph: { nodes: [], edges: [] }, // 路网（拓扑 Graph），轨道路线导航用
  routeEditMode: null, // null / "add" / "link" / "delete"：画布路网编辑模式
  routeLinkSource: null, // 连线模式已选中的第一个节点名
};

const statusLog = document.getElementById("status-log");
const mapCanvas = document.getElementById("map-canvas");
const ctx = mapCanvas.getContext("2d");
// 渲染缩放：把地图 1 格放大为多个 canvas 像素，避免小地图被 CSS 拉伸后模糊。
// 目标是让 canvas 内部分辨率稳定在 ~960px 宽，地图/点云/机器人姿态都清晰。
const RENDER_TARGET_WIDTH = 960;

function renderScale() {
  if (!state.map) return 8;
  return Math.max(1, Math.floor(RENDER_TARGET_WIDTH / state.map.info.width));
}

// ---------------- 地图视图：滚轮缩放 / 拖动平移 ----------------
// 分工：worldToCanvas() 只负责"世界坐标 → 基准坐标"（基准尺度下的一组位图像素），
// 画布上再统一套一层视图变换 (scale, tx, ty) 把基准坐标映射到屏幕 CSS 像素。
// 这样地图、点云、路网、站点、机器人全都会一起缩放，而且文字/线条是矢量绘制，
// 放大后依然清晰 —— 若改成把位图放大再让浏览器插值，字就糊了。
const MAP_ZOOM_MIN = 1;
const MAP_ZOOM_MAX = 8;
const mapView = { zoom: 1, fit: 1, scale: 1, tx: 0, ty: 0 };
let mapRaster = null;        // 静态地图栅格（离屏画布），只在地图变化时重建
let mapRasterSource = null;  // 对应哪一份 state.map
let mapRasterScale = 0;      // 按哪个基准尺度画的

function mapBaseSize() {
  if (!state.map) return { w: 0, h: 0 };
  const s = renderScale();
  return { w: state.map.info.width * s, h: state.map.info.height * s };
}

function mapViewBox() {
  return {
    w: Math.max(1, mapCanvas.clientWidth),
    h: Math.max(1, mapCanvas.clientHeight),
  };
}

// 1 倍缩放 = 整张地图恰好装进取景框；keepZoom 时保留当前放大倍数只重算居中
function fitMapView(keepZoom) {
  if (!state.map) return;
  const box = mapViewBox();
  const { w, h } = mapBaseSize();
  if (!w || !h) return;
  if (box.w <= 1 || box.h <= 1) return; // 布局还没完成，等下次 drawMap 再算
  mapView.fit = Math.min(box.w / w, box.h / h);
  if (!keepZoom) mapView.zoom = MAP_ZOOM_MIN;
  mapView.scale = mapView.fit * mapView.zoom;
  mapView.tx = (box.w - w * mapView.scale) / 2;
  mapView.ty = (box.h - h * mapView.scale) / 2;
}

// 缩放到指定倍数；给了 anchorEvent 就"钉住"光标下的那个地图点不动
function setMapZoom(zoom, anchorEvent) {
  if (!state.map) return;
  const box = mapViewBox();
  const { w, h } = mapBaseSize();
  if (!w || !h) return;
  const next = Math.min(MAP_ZOOM_MAX, Math.max(MAP_ZOOM_MIN, zoom));
  const rect = mapCanvas.getBoundingClientRect();
  const px = anchorEvent ? anchorEvent.clientX - rect.left : box.w / 2;
  const py = anchorEvent ? anchorEvent.clientY - rect.top : box.h / 2;
  const baseX = (px - mapView.tx) / mapView.scale;
  const baseY = (py - mapView.ty) / mapView.scale;
  mapView.zoom = next;
  mapView.scale = mapView.fit * next;
  mapView.tx = px - baseX * mapView.scale;
  mapView.ty = py - baseY * mapView.scale;
  drawMap();
}

// 静态地图栅格：按基准尺度画一次到离屏画布，之后每帧只做一次 drawImage。
// （原实现每帧都要逐格写 s×s 个像素，10Hz 下是白白的开销。）
function ensureMapRaster() {
  if (!state.map) return null;
  const s = renderScale();
  if (mapRaster && mapRasterSource === state.map && mapRasterScale === s) {
    return mapRaster;
  }
  const { width, height } = state.map.info;
  const cw = width * s;
  const ch = height * s;
  const off = document.createElement("canvas");
  off.width = cw;
  off.height = ch;
  const octx = off.getContext("2d");
  const imageData = octx.createImageData(cw, ch);
  for (let i = 0; i < state.map.data.length; i += 1) {
    const value = state.map.data[i];
    // OccupancyGrid.data 是 row-major，data[0] 对应 map 左下角(y 最小)；
    // canvas 的 y 轴向下，与 worldToCanvas 保持一致，需按行上下翻转。
    const col = i % width;
    const row = Math.floor(i / width);
    const canvasRow = height - 1 - row;
    let gray;
    if (value < 0) {
      gray = 170;
    } else if (value === 100) {
      gray = 20;
    } else {
      gray = 240;
    }
    const baseX = col * s;
    const baseY = canvasRow * s;
    for (let dy = 0; dy < s; dy += 1) {
      const rowStart = (baseY + dy) * cw + baseX;
      for (let dx = 0; dx < s; dx += 1) {
        const offset = (rowStart + dx) * 4;
        imageData.data[offset] = gray;
        imageData.data[offset + 1] = gray;
        imageData.data[offset + 2] = gray;
        imageData.data[offset + 3] = 255;
      }
    }
  }
  octx.putImageData(imageData, 0, 0);
  mapRaster = off;
  mapRasterSource = state.map;
  mapRasterScale = s;
  return mapRaster;
}
const rosHostInput = document.getElementById("ros-host");
let cmdVelPublisher = null;
let manualTimer = null;

if (!rosHostInput.value.trim() || rosHostInput.value.trim() === "127.0.0.1") {
  rosHostInput.value = window.location.hostname || "127.0.0.1";
}

function formatRosError(error, host, port) {
  if (error instanceof Error) return error.message;
  if (error && typeof error === "object" && "target" in error) {
    return `WebSocket 连接失败：${host}:${port}，请确认 rosbridge 已启动且端口可访问`;
  }
  return String(error);
}

function log(message) {
  const line = document.createElement("div");
  line.textContent = `${new Date().toLocaleTimeString("zh-CN")} ${message}`;
  statusLog.appendChild(line);
  statusLog.scrollTop = statusLog.scrollHeight;
  while (statusLog.children.length > 200) {
    statusLog.removeChild(statusLog.firstChild);
  }
}

function setConnectionState(connected) {
  state.connected = connected;
  const badge = document.getElementById("connection-state");
  badge.textContent = connected ? "在线" : "离线";
  badge.className = `badge ${connected ? "online" : "offline"}`;
}

function callService(name, type, request) {
  return new Promise((resolve, reject) => {
    const service = new ROSLIB.Service({
      ros: state.ros,
      name,
      serviceType: type,
    });
    service.callService(request, (result) => {
      if (result) resolve(result);
      else reject(new Error(`${name} 无返回结果`));
    }, reject);
  });
}

function connect() {
  const host = document.getElementById("ros-host").value.trim();
  const port = document.getElementById("ros-port").value.trim();
  if (state.ros) state.ros.close();
  state.ros = new ROSLIB.Ros({ url: `ws://${host}:${port}` });
  state.ros.on("connection", () => {
    setConnectionState(true);
    log(`已连接 ${host}:${port}`);
    subscribeToTopics();
    refreshStations();
    refreshMarkers();
    refreshMaps();
    refreshRouteGraph();
  });
  state.ros.on("error", (error) => log(`ROS 错误：${formatRosError(error, host, port)}`));
  state.ros.on("close", () => {
    setConnectionState(false);
    disableManualControl();
  });
}

function subscribeToTopics() {
  const mapTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/map",
    messageType: "nav_msgs/msg/OccupancyGrid",
  });
  mapTopic.subscribe((message) => {
    const prevInfo = state.map && state.map.info;
    // 只有"换了地图"（尺寸/分辨率/原点变了）才复位视图。SLAM 建图期间 /map 会
    // 持续更新，那时若每次都复位，用户的缩放会被一直打断。
    const geometryChanged = !prevInfo
      || prevInfo.width !== message.info.width
      || prevInfo.height !== message.info.height
      || prevInfo.resolution !== message.info.resolution
      || prevInfo.origin.position.x !== message.info.origin.position.x
      || prevInfo.origin.position.y !== message.info.origin.position.y;
    state.map = message;
    if (geometryChanged) fitMapView(false);
    drawMap();
  });

  const poseTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/amcl_pose",
    messageType: "geometry_msgs/msg/PoseWithCovarianceStamped",
  });
  poseTopic.subscribe((message) => {
    state.robotPose = message;
    // AMCL 只在滤波更新时才发布 /amcl_pose 与 map→odom TF（受 update_min_d/a
    // 门限限制），两次更新之间 amcl_pose 是旧值。这里用 amcl_pose 与收到它
    // 时刻的 /odom 反解出 map→odom，之后配合最新 /odom 外推出实时位姿，
    // 效果与 RViz 走 TF 链一致，消除点云/机器人图标的显示滞后。
    if (state.odomPose) {
      const p = message.pose.pose;
      const yawA = yawFromQuaternion(p.orientation);
      const o = state.odomPose;
      const yawMo = yawA - o.yaw;
      const cosMo = Math.cos(yawMo);
      const sinMo = Math.sin(yawMo);
      state.mapToOdom = {
        x: p.position.x - (o.x * cosMo - o.y * sinMo),
        y: p.position.y - (o.x * sinMo + o.y * cosMo),
        yaw: yawMo,
      };
    }
    drawMap();
    updateLocalizationStatus();
  });

  // 轮式里程计：用于在 AMCL 两次更新之间外推实时位姿（仅显示用）
  const odomTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/odom",
    messageType: "nav_msgs/msg/Odometry",
    throttle_rate: 40,
  });
  odomTopic.subscribe((message) => {
    const p = message.pose.pose;
    const st = message.header && message.header.stamp;
    const entry = {
      // ROS 时钟域时间戳：与 /scan 的 stamp 同源，用于点云-位姿时间对齐
      t: st ? st.sec + (st.nanosec || 0) * 1e-9 : null,
      x: p.position.x,
      y: p.position.y,
      yaw: yawFromQuaternion(p.orientation),
    };
    state.odomPose = entry;
    state.odomPoseBuf.push(entry);
    if (state.odomPoseBuf.length > 40) state.odomPoseBuf.shift();
  });

  // 激光雷达点云：订阅 /scan，把 360° 测量点变换到地图坐标系后叠加。
  // 不设 throttle_rate，直接以雷达原生频率(~10Hz)重绘，保证实时性。
  const scanTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/scan",
    messageType: "sensor_msgs/msg/LaserScan",
    throttle_rate: 0,
  });
  scanTopic.subscribe(handleScanMessage);

  const statusTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/delivery/status",
    messageType: "std_msgs/msg/String",
  });
  statusTopic.subscribe((message) => {
    log(message.data);
    updateNavigationTargetFromStatus(message.data);
    updateLocalizationStatus();
    if (message.data.includes("已自动登记")) {
      refreshMarkers();
    }
  });

  cmdVelPublisher = new ROSLIB.Topic({
    ros: state.ros,
    name: "/cmd_vel",
    messageType: "geometry_msgs/msg/Twist",
  });

  const cameraTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/aruco/compressed",
    messageType: "sensor_msgs/msg/CompressedImage",
    compression: "none",
    // 100ms 时 rosbridge 序列化 JPEG 的 CPU 占用接近一整个核，会挤压
    // Nav2 控制循环导致小车走走停停；500ms(约2帧/秒)足够看画面
    throttle_rate: 500,
  });
  cameraTopic.subscribe(handleCameraImage);

  const arucoStatusTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/aruco/status",
    messageType: "std_msgs/msg/String",
  });
  arucoStatusTopic.subscribe(handleArucoStatus);

  const imuTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/imu",
    messageType: "sensor_msgs/msg/Imu",
    // IMU 原生频率高，页面只做状态展示，限到 10Hz 降低 rosbridge 负担
    throttle_rate: 100,
  });
  imuTopic.subscribe(handleImuData);

  const batteryTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/originbot_status",
    messageType: "originbot_msgs/msg/OriginbotStatus",
  });
  batteryTopic.subscribe(handleBatteryStatus);
}

// 激光雷达相对 base_link 的固定安装偏移，以 ldlidar/launch/stl06n.launch.py 发布的
// base_link->laser_link TF 为准: x=0.05 y=0 z=0.15, no rotation（z 对 2D 投影无影响）。
const LASER_TO_BASE_X = 0.05;
const LASER_TO_BASE_Y = 0.0;

// 从 amcl_pose(或四元数)提取绕 Z 轴的 yaw(弧度)
function yawFromQuaternion(q) {
  if (!q) return 0;
  return Math.atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
  );
}

// 用 map→odom 叠加指定的 odom 系位姿（默认传最新值；点云对齐时传历史值）
function composeMapPose(o) {
  if (!state.mapToOdom || !o) return null;
  const m = state.mapToOdom;
  const cosM = Math.cos(m.yaw);
  const sinM = Math.sin(m.yaw);
  return {
    x: m.x + (o.x * cosM - o.y * sinM),
    y: m.y + (o.x * sinM + o.y * cosM),
    yaw: m.yaw + o.yaw,
  };
}

// 从 /odom 缓冲里取与 t（ROS 秒）最接近的位姿；超过 0.3 秒视为不可信
function odomPoseAt(t) {
  const buf = state.odomPoseBuf;
  if (!t || !buf || !buf.length) return null;
  let best = null;
  let bestDt = Infinity;
  for (const entry of buf) {
    if (entry.t == null) continue;
    const dt = Math.abs(entry.t - t);
    if (dt < bestDt) {
      bestDt = dt;
      best = entry;
    }
  }
  return bestDt <= 0.3 ? best : null;
}

// 机器人基座在地图系的实时位姿：map→odom ∘ odom→base 外推；
// 没有 /odom 数据时退回最后一次 amcl_pose。无定位返回 null。
function getRobotMapPose() {
  if (state.mapToOdom && state.odomPose) {
    const m = state.mapToOdom;
    const o = state.odomPose;
    const cosM = Math.cos(m.yaw);
    const sinM = Math.sin(m.yaw);
    return {
      x: m.x + (o.x * cosM - o.y * sinM),
      y: m.y + (o.x * sinM + o.y * cosM),
      yaw: m.yaw + o.yaw,
    };
  }
  if (state.robotPose) {
    const p = state.robotPose.pose.pose;
    return {
      x: p.position.x,
      y: p.position.y,
      yaw: yawFromQuaternion(p.orientation),
    };
  }
  return null;
}

// 把 LaserScan 的每个有效测量点变换到地图(map)坐标系，写入 state.scanCloud。
// 变换 = 机器人 map 位姿(map→odom ∘ odom，AMCL 修正) ∘ laser→base 偏移。
// 该方法保持与雷达帧率一致(~10Hz)，仅在收到新帧时重算，避免额外开销。
function handleScanMessage(message) {
  if (!state.showScan) {
    state.scanCloud = [];
    return;
  }
  state.scan = message;
  state.scanStamp = Date.now();
  const angleMin = message.angle_min || 0;
  const angleInc = message.angle_increment || 0;
  const ranges = message.ranges || [];
  // 点云变换位姿与扫描时间戳对齐：从 /odom 缓冲取与 scan.stamp 最近的
  // 里程计位姿，再叠加最新 map→odom（AMCL 修正慢变，取最新等价于 TF 查询）。
  // 此前用"现在"的位姿变换"过去"的激光，车速 0.5m/s、链路延迟 0.2s 时
  // 点云与地图错位可达 10cm，表现为"地图和点云对不上"。
  const scanStampHeader = message.header && message.header.stamp;
  const scanSec = scanStampHeader
    ? scanStampHeader.sec + (scanStampHeader.nanosec || 0) * 1e-9
    : null;
  const alignedPose = composeMapPose(odomPoseAt(scanSec));
  const robotPose = alignedPose || getRobotMapPose();
  if (!robotPose) {
    state.scanCloud = [];
    return;
  }
  const robotX = robotPose.x;
  const robotY = robotPose.y;
  const robotYaw = robotPose.yaw;

  const cosY = Math.cos(robotYaw);
  const sinY = Math.sin(robotYaw);
  const points = [];
  const N = ranges.length;
  for (let i = 0; i < N; i += 1) {
    const r = ranges[i];
    // 过滤无效值(inf / NaN / 超量程)
    if (!(r > 0.05) || !(r < 30.0)) continue;
    const a = angleMin + i * angleInc;
    // laser 系下：x_laser 向前, y_laser 向左；标准极坐标 x=r*cos, y=r*sin
    let lx = r * Math.cos(a);
    let ly = r * Math.sin(a);
    // laser → base：固定平移
    const bx = lx + LASER_TO_BASE_X;
    const by = ly + LASER_TO_BASE_Y;
    // base → map：旋转(绕Z)+平移
    const wx = robotX + (bx * cosY - by * sinY);
    const wy = robotY + (bx * sinY + by * cosY);
    points.push([wx, wy]);
  }
  state.scanCloud = points;
  drawMap(); // 每帧重绘地图与点云
}

// 3S 锂电实测电压-电量分段（区间内线性插值）：
// 9.84-10.12V → 10-20%，10.12-10.65V → 20-40%，10.65-12V → 40-100%；
// 9.0V 以下视为 0%（约 3.0V/格，接近放空），12V 以上视为 100%。
const BATTERY_SEGMENTS = [
  [9.0, 9.84, 0, 10],
  [9.84, 10.12, 10, 20],
  [10.12, 10.65, 20, 40],
  [10.65, 12.0, 40, 100],
];

function batteryPercent(voltage) {
  if (voltage >= 12.0) return 100;
  for (const [v0, v1, p0, p1] of BATTERY_SEGMENTS) {
    if (voltage <= v1) {
      const percent = p0 + ((voltage - v0) / (v1 - v0)) * (p1 - p0);
      return Math.max(0, Math.min(100, percent));
    }
  }
  return 0;
}

function handleBatteryStatus(message) {
  const element = document.getElementById("battery-voltage");
  if (!element) return;
  const voltage = Number(message.battery_voltage) || 0;
  const percent = voltage > 0 ? batteryPercent(voltage) : 0;
  element.textContent = `${voltage.toFixed(2)} V（${percent.toFixed(0)}%）`;
  element.className = voltage > 0 && voltage < 9.84 ? "battery-low" : "";
}

function uint8ArrayToBase64(bytes) {
  let binary = "";
  const chunkSize = 0x8000;
  for (let i = 0; i < bytes.length; i += chunkSize) {
    const chunk = bytes.subarray(i, i + chunkSize);
    binary += String.fromCharCode.apply(null, chunk);
  }
  return btoa(binary);
}

function handleCameraImage(message) {
  const image = document.getElementById("camera-image");
  const placeholder = document.getElementById("camera-placeholder");
  if (placeholder) placeholder.classList.add("hidden");
  state.lastCameraImageTime = Date.now();
  if (typeof message.data === "string") {
    image.src = `data:image/jpeg;base64,${message.data}`;
  } else {
    image.src = `data:image/jpeg;base64,${uint8ArrayToBase64(message.data)}`;
  }
}

function handleArucoStatus(message) {
  const badge = document.getElementById("aruco-detection-state");
  const detail = document.getElementById("aruco-detection-detail");
  const ids = message.data || "";
  if (!ids || ids === "none") {
    badge.textContent = "未识别";
    badge.className = "badge offline";
    detail.textContent = "等待 ArUco 检测";
    state.lastArucoIds = "";
    updateLocalizationStatus();
    return;
  }
  badge.textContent = "已识别";
  badge.className = "badge online";
  detail.textContent = `已识别，id：${ids}`;
  state.lastArucoStatusTime = Date.now();
  if (ids !== state.lastArucoIds) {
    state.lastArucoIds = ids;
    showArucoToast(`已识别，id：${ids}`);
    log(`已识别 ArUco，id：${ids}`);
  }
  updateLocalizationStatus();
}

function showArucoToast(text) {
  const toast = document.getElementById("aruco-toast");
  if (!toast) return;
  toast.textContent = text;
  toast.classList.remove("hidden");
  clearTimeout(showArucoToast.timer);
  showArucoToast.timer = setTimeout(() => {
    toast.classList.add("hidden");
  }, 3000);
}

function updateCameraStatus() {
  const placeholder = document.getElementById("camera-placeholder");
  if (!placeholder) return;
  const hasFrame = Date.now() - state.lastCameraImageTime < 3000;
  placeholder.classList.toggle("hidden", state.connected && hasFrame);
}

function quaternionToEulerDegrees(q) {
  const sinrCosp = 2.0 * (q.w * q.x + q.y * q.z);
  const cosrCosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = 2.0 * (q.w * q.y - q.z * q.x);
  const pitch = Math.abs(sinp) >= 1.0
    ? Math.sign(sinp) * Math.PI / 2.0
    : Math.asin(sinp);

  const sinyCosp = 2.0 * (q.w * q.z + q.x * q.y);
  const cosyCosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  return {
    roll: (roll * 180.0 / Math.PI).toFixed(1),
    pitch: (pitch * 180.0 / Math.PI).toFixed(1),
    yaw: (yaw * 180.0 / Math.PI).toFixed(1),
  };
}

function handleImuData(message) {
  state.imuData = message;
  const q = message.orientation;
  const euler = quaternionToEulerDegrees(q);
  document.getElementById("imu-quaternion").textContent =
    `${q.x.toFixed(3)}, ${q.y.toFixed(3)}, ${q.z.toFixed(3)}, ${q.w.toFixed(3)}`;
  document.getElementById("imu-euler").textContent =
    `${euler.roll}°, ${euler.pitch}°, ${euler.yaw}°`;

  const angular = message.angular_velocity;
  document.getElementById("imu-angular").textContent =
    `${angular.x.toFixed(3)}, ${angular.y.toFixed(3)}, ${angular.z.toFixed(3)} rad/s`;

  const linear = message.linear_acceleration;
  document.getElementById("imu-linear").textContent =
    `${linear.x.toFixed(3)}, ${linear.y.toFixed(3)}, ${linear.z.toFixed(3)} m/s²`;
}

function yawDegreesFromOdometry(odom) {
  const q = odom.pose.pose.orientation;
  const yaw = Math.atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
  );
  return ((yaw * 180.0) / Math.PI).toFixed(1);
}

function nearestStation() {
  if (!state.robotPose || !state.stations.length) return null;
  const position = state.robotPose.pose.pose.position;
  let nearest = null;
  let nearestDistance = Infinity;
  for (const station of state.stations) {
    const distance = Math.hypot(
      station.x - position.x,
      station.y - position.y,
    );
    if (distance < nearestDistance) {
      nearest = station;
      nearestDistance = distance;
    }
  }
  return { station: nearest, distance: nearestDistance };
}

function updateLocalizationStatus() {
  const stateElement = document.getElementById("localization-state");
  const confidenceElement = document.getElementById("localization-confidence");
  const positionElement = document.getElementById("current-position");
  const stationElement = document.getElementById("current-station");
  const targetElement = document.getElementById("navigation-target");

  if (!state.robotPose) {
    stateElement.textContent = "等待定位数据";
    stateElement.className = "badge offline";
    confidenceElement.textContent = "--";
    positionElement.textContent = "--";
    stationElement.textContent = "--";
    targetElement.textContent = state.navigationTarget || "--";
    return;
  }

  const covariance = state.robotPose.pose.covariance || [];
  const positionVariance = Math.max(covariance[0] || 1, covariance[7] || 1);
  const yawVariance = covariance[35] || 1;
  const positionStd = Math.sqrt(positionVariance);
  const yawStdDeg = Math.sqrt(yawVariance) * 180.0 / Math.PI;
  const hasRecentAruco = Date.now() - state.lastArucoStatusTime < 2000;

  let stateText;
  let stateClass;
  if (hasRecentAruco) {
    stateText = "已定位（ArUco）";
    stateClass = "badge online";
  } else if (positionStd < 0.3) {
    stateText = "已定位（EKF）";
    stateClass = "badge online";
  } else if (positionStd < 0.8) {
    stateText = "定位中";
    stateClass = "badge online";
  } else {
    stateText = "定位置信度低";
    stateClass = "badge offline";
  }
  stateElement.textContent = stateText;
  stateElement.className = stateClass;

  if (positionStd < 0.12 && yawStdDeg < 8.0) {
    confidenceElement.textContent = "高";
  } else if (positionStd < 0.35 && yawStdDeg < 20.0) {
    confidenceElement.textContent = "中";
  } else {
    confidenceElement.textContent = "低";
  }

  const position = state.robotPose.pose.pose.position;
  positionElement.textContent = `x ${position.x.toFixed(2)}，y ${position.y.toFixed(2)}，航向 ${yawDegreesFromOdometry(state.robotPose)}°`;

  const nearest = nearestStation();
  if (nearest) {
    stationElement.textContent = `${nearest.station.name}（${nearest.distance.toFixed(2)} m）`;
  } else {
    stationElement.textContent = "未建立站点";
  }
  targetElement.textContent = state.navigationTarget || "--";
}

function updateNavigationTargetFromStatus(message) {
  if (
    message.includes("导航完成") ||
    message.includes("导航已取消") ||
    message.includes("导航失败") ||
    message.includes("导航目标被拒绝")
  ) {
    state.navigationTarget = null;
  } else if (message.includes("已开始前往 ")) {
    state.navigationTarget = message.replace("已开始前往 ", "");
  }
  updateLocalizationStatus();
}

async function controlSystem(subsystem, enable) {
  let requestSubsystem = subsystem;
  if (subsystem === "mapping" && enable) {
    const mappingEngine = document.getElementById("mapping-algorithm").value;
    requestSubsystem = `mapping:${mappingEngine}`;
  }
  await callService(
    "/delivery/control_system",
    "originbot_msgs/srv/ControlSystem",
    { subsystem: requestSubsystem, enable },
  );
}

function publishCmdVel(linearX, angularZ) {
  if (!state.connected || !cmdVelPublisher) return;
  cmdVelPublisher.publish({
    linear: { x: linearX, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angularZ },
  });
}

function stopManualTimer() {
  if (manualTimer) {
    clearInterval(manualTimer);
    manualTimer = null;
  }
}

function updateManualState() {
  const manualState = document.getElementById("manual-state");
  const button = document.getElementById("manual-mode");
  manualState.textContent = state.manualMode ? "手动模式已开启" : "手动模式已关闭";
  manualState.className = `manual-state${state.manualMode ? " active" : ""}`;
  button.textContent = state.manualMode ? "退出手动模式" : "进入手动模式";
}

function disableManualControl() {
  state.manualMode = false;
  state.pressedKeys.clear();
  resetJoystick();
  stopManualTimer();
  publishCmdVel(0, 0);
  updateManualState();
}

async function enableManualControl() {
  if (!state.connected) {
    log("请先连接 ROS 服务");
    return;
  }
  state.manualMode = true;
  updateManualState();
  try {
    await callService(
      "/delivery/cancel_navigation",
      "originbot_msgs/srv/CancelNavigation",
      {},
    );
  } catch (error) {
    log(`导航取消请求未执行：${error}`);
  }
  stopManualTimer();
  manualTimer = setInterval(publishManualVelocity, 50);
  log(
    state.controlMode === "joystick"
      ? "已进入手动模式，请拖动遥杆圆盘控制"
      : "已进入手动模式，请使用 W/A/S/D 或方向键控制",
  );
}

function publishManualVelocity() {
  const linear = Number(document.getElementById("manual-linear").value || 0);
  const angular = Number(document.getElementById("manual-angular").value || 0);
  const keys = state.pressedKeys;
  let linearX = 0;
  let angularZ = 0;

  if (state.controlMode === "joystick" && state.joystick.active) {
    // 遥杆：y 向下为屏幕正方向，上推(y<0)前进；x 右推为右转，
    // ROS 角速度 z 逆时针为正（左转），所以取负号。
    linearX = -state.joystick.y * linear;
    angularZ = -state.joystick.x * angular;
  } else if (state.controlMode === "keyboard") {
    if (keys.has("w") || keys.has("arrowup")) linearX = linear;
    if (keys.has("s") || keys.has("arrowdown")) linearX = -linear;
    if (keys.has("a") || keys.has("arrowleft")) angularZ = angular;
    if (keys.has("d") || keys.has("arrowright")) angularZ = -angular;
  }

  publishCmdVel(linearX, angularZ);
}

async function setManualMode(enabled) {
  if (enabled) {
    await enableManualControl();
  } else {
    disableManualControl();
  }
}

// ---------- 虚拟遥杆控制 ----------
// 复用手动模式的速度上限（线速度/角速度输入框）与 50ms 定时发布，
// 遥杆只负责给出归一化方向量，实际 cmd_vel 仍由 publishManualVelocity 统一发出。

const JOYSTICK_DEAD_ZONE = 0.08; // 中心死区：避免松手附近微动导致小车缓慢漂移
let joystickPointerId = null;

function joystickElements() {
  return {
    pad: document.getElementById("joystick-pad"),
    knob: document.getElementById("joystick-knob"),
  };
}

function setJoystickKnobPosition(offsetX, offsetY) {
  const { pad, knob } = joystickElements();
  if (!pad || !knob) return;
  const maxRadius = Math.max(1, pad.clientWidth / 2 - knob.offsetWidth / 2);
  const distance = Math.hypot(offsetX, offsetY);
  const factor = distance > maxRadius ? maxRadius / distance : 1;
  knob.style.transform =
    `translate(${offsetX * factor}px, ${offsetY * factor}px)`;
}

function resetJoystick() {
  const { knob } = joystickElements();
  state.joystick.active = false;
  state.joystick.x = 0;
  state.joystick.y = 0;
  joystickPointerId = null;
  if (knob) knob.style.transform = "translate(0px, 0px)";
  const pad = document.getElementById("joystick-pad");
  if (pad) pad.classList.remove("active");
}

function updateJoystickFromEvent(event) {
  const { pad, knob } = joystickElements();
  if (!pad || !knob) return;
  const rect = pad.getBoundingClientRect();
  const centerX = rect.left + rect.width / 2;
  const centerY = rect.top + rect.height / 2;
  const maxRadius = Math.max(1, pad.clientWidth / 2 - knob.offsetWidth / 2);
  let offsetX = event.clientX - centerX;
  let offsetY = event.clientY - centerY;
  // 限制摇杆偏移不出圆盘
  const distance = Math.hypot(offsetX, offsetY);
  if (distance > maxRadius) {
    offsetX = (offsetX / distance) * maxRadius;
    offsetY = (offsetY / distance) * maxRadius;
  }
  setJoystickKnobPosition(offsetX, offsetY);

  let nx = offsetX / maxRadius;
  let ny = offsetY / maxRadius;
  if (Math.hypot(nx, ny) < JOYSTICK_DEAD_ZONE) {
    nx = 0;
    ny = 0;
  }
  state.joystick.x = nx;
  state.joystick.y = ny;
  state.joystick.active = true;
  publishManualVelocity();
}

function handleJoystickStart(event) {
  if (state.controlMode !== "joystick") return;
  event.preventDefault();
  if (!state.manualMode) {
    // 直接推动遥杆即视为想开车：自动进入手动模式（会取消进行中的导航）
    setManualMode(true);
  }
  const { pad } = joystickElements();
  joystickPointerId = event.pointerId;
  try {
    pad.setPointerCapture(event.pointerId);
  } catch (error) {
    // 部分浏览器对已释放的指针会抛错，忽略即可
  }
  pad.classList.add("active");
  updateJoystickFromEvent(event);
}

function handleJoystickMove(event) {
  if (joystickPointerId === null || event.pointerId !== joystickPointerId) return;
  event.preventDefault();
  updateJoystickFromEvent(event);
}

function handleJoystickEnd(event) {
  if (joystickPointerId === null || event.pointerId !== joystickPointerId) return;
  event.preventDefault();
  resetJoystick();
  if (state.manualMode) publishCmdVel(0, 0); // 松手即停
}

function setControlMode(mode) {
  if (state.controlMode === mode) return;
  if (state.manualMode) publishCmdVel(0, 0); // 切换输入方式前先停车
  state.controlMode = mode;
  resetJoystick();
  const keyboardButton = document.getElementById("control-mode-keyboard");
  const joystickButton = document.getElementById("control-mode-joystick");
  keyboardButton.classList.toggle("active", mode === "keyboard");
  joystickButton.classList.toggle("active", mode === "joystick");
  document.getElementById("joystick-pad").classList.toggle("hidden", mode !== "joystick");
  document.getElementById("joystick-hint").classList.toggle("hidden", mode !== "joystick");
  document.getElementById("keyboard-hint").classList.toggle("hidden", mode === "joystick");
}

async function refreshStations() {
  const result = await callService(
    "/delivery/station",
    "originbot_msgs/srv/StationEntry",
    { command: "list", name: "", x: 0, y: 0, yaw: 0 },
  );
  state.stations = [];
  const tbody = document.getElementById("station-list");
  const select = document.getElementById("station-select");
  tbody.innerHTML = "";
  select.innerHTML = "";
  for (let i = 0; i < result.names.length; i += 1) {
    const station = {
      name: result.names[i],
      x: result.x[i],
      y: result.y[i],
      yaw: result.yaw[i],
    };
    state.stations.push(station);
    const option = document.createElement("option");
    option.value = station.name;
    option.textContent = station.name;
    select.appendChild(option);
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>${station.name}</td>
      <td>${station.x.toFixed(2)}</td>
      <td>${station.y.toFixed(2)}</td>
      <td>${station.yaw.toFixed(2)}</td>
      <td><button data-station="${station.name}">删除</button></td>`;
    tbody.appendChild(row);
  }
  drawMap();
  updateLocalizationStatus();
}

async function refreshMarkers() {
  const result = await callService(
    "/delivery/marker",
    "originbot_msgs/srv/MarkerEntry",
    { command: "list", marker_id: 0, marker_size: 0, x: 0, y: 0, yaw: 0 },
  );
  const markers = [];
  for (let i = 0; i < result.marker_ids.length; i += 1) {
    markers.push({
      id: result.marker_ids[i],
      size: result.marker_sizes[i],
      x: result.x[i],
      y: result.y[i],
      yaw: result.yaw[i],
    });
  }
  // 轮询场景下列表没变化就什么都不做，避免每 5 秒重建表格/重绘地图
  if (JSON.stringify(markers) === JSON.stringify(state.markers)) return;
  state.markers = markers;
  const tbody = document.getElementById("marker-list");
  tbody.innerHTML = "";
  for (const marker of markers) {
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>${marker.id}</td>
      <td>${marker.size.toFixed(2)}</td>
      <td>${marker.x.toFixed(2)}</td>
      <td>${marker.y.toFixed(2)}</td>
      <td>${marker.yaw.toFixed(2)}</td>
      <td><button data-marker="${marker.id}">删除</button></td>`;
    tbody.appendChild(row);
  }
  drawMap();
  updateLocalizationStatus();
}

// ---------- 路网（拓扑 Graph）画布编辑器 ----------
// 编辑在本地 state.routeGraph 上进行，点“保存路网”才写入后端 routes.json；
// 轨道路线导航（route 模式）由 route_follower 在导航时自动加载最新路网。

const ROUTE_SNAP_DISTANCE = 0.3; // 添加节点时吸附站点的半径（米）

function routeHitTolerance() {
  // 命中半径：取 12px 对应的世界距离，但至少 0.12m，保证小地图也好点
  if (!state.map) return 0.12;
  return Math.max(0.12, (12 * state.map.info.resolution) / renderScale());
}

function hitRouteNode(worldX, worldY) {
  const tol = routeHitTolerance();
  let best = null;
  let bestDist = Infinity;
  for (const node of state.routeGraph.nodes) {
    const d = Math.hypot(node.x - worldX, node.y - worldY);
    if (d < tol && d < bestDist) {
      best = node;
      bestDist = d;
    }
  }
  return best;
}

function hitStationForRoute(worldX, worldY) {
  // 连接模式下命中蓝色站点：半径略大于节点命中，方便直接点站点接入路网
  const tol = routeHitTolerance() * 1.2;
  let best = null;
  let bestDist = tol;
  for (const station of state.stations) {
    const d = Math.hypot(station.x - worldX, station.y - worldY);
    if (d < bestDist) {
      best = station;
      bestDist = d;
    }
  }
  return best;
}

function distPointToSegment(px, py, ax, ay, bx, by) {
  const dx = bx - ax;
  const dy = by - ay;
  const len2 = dx * dx + dy * dy;
  const t = len2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / len2));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

function hitRouteEdge(worldX, worldY) {
  const tol = routeHitTolerance() * 0.8;
  for (let i = 0; i < state.routeGraph.edges.length; i += 1) {
    const edge = state.routeGraph.edges[i];
    const a = state.routeGraph.nodes.find((n) => n.name === edge.a);
    const b = state.routeGraph.nodes.find((n) => n.name === edge.b);
    if (!a || !b) continue;
    const pts = [a, ...(edge.via || []).map(([vx, vy]) => ({ x: vx, y: vy })), b];
    for (let j = 1; j < pts.length; j += 1) {
      if (distPointToSegment(worldX, worldY, pts[j - 1].x, pts[j - 1].y, pts[j].x, pts[j].y) < tol) {
        return i;
      }
    }
  }
  return null;
}

function nextRouteNodeName() {
  let maxIndex = 0;
  for (const node of state.routeGraph.nodes) {
    const match = /^n(\d+)$/.exec(node.name);
    if (match) maxIndex = Math.max(maxIndex, Number(match[1]));
  }
  return `n${maxIndex + 1}`;
}

function updateRouteHint() {
  const hint = document.getElementById("route-hint");
  if (!hint) return;
  const counts = `节点数 ${state.routeGraph.nodes.length}，边数 ${state.routeGraph.edges.length}`;
  let modeText = "编辑提示：添加节点时自动吸附 0.3 米内的站点（继承站点朝向）；改完点“保存路网”生效。";
  if (state.routeEditMode === "add") {
    modeText = "添加节点模式：在地图上点击放置节点（自动吸附附近站点）；完成后再次点击按钮退出";
  } else if (state.routeEditMode === "link") {
    modeText = state.routeLinkSource
      ? `连接节点模式：已选中 ${state.routeLinkSource}，再点击另一个节点完成连线；点击空白处取消`
      : "连接节点模式：点击第一个节点（紫色节点或蓝色站点均可）";
  } else if (state.routeEditMode === "delete") {
    modeText = "删除模式：点击节点删除该节点及其连线；点击连线删除该边";
  }
  hint.textContent = `${modeText}。${counts}`;
}

function setRouteEditMode(mode) {
  if (state.routeEditMode === mode) {
    state.routeEditMode = null;
  } else {
    state.routeEditMode = mode;
    state.routeLinkSource = null;
  }
  // 与地图取点/设定位姿互斥：进入路网编辑时取消未完成的取点，
  // 否则画布点击被路网优先消费，取点永远完不成
  if (state.routeEditMode && state.pickTarget) {
    state.pickTarget = null;
    state.pickStartCanvas = null;
    state.pickCurrentCanvas = null;
    updatePickButtonState();
  }
  for (const id of ["route-mode-add", "route-mode-link", "route-mode-delete"]) {
    const button = document.getElementById(id);
    if (button) button.classList.toggle("active", state.routeEditMode === id.replace("route-mode-", ""));
  }
  updateRouteHint();
  drawMap();
}

function handleRouteCanvasClick(event) {
  event.preventDefault();
  const canvasPoint = canvasPointFromEvent(event);
  const world = canvasToWorld(canvasPoint);
  if (!world) return;

  if (state.routeEditMode === "add") {
    // 吸附附近站点：继承站点位置、名称与朝向，保证到站逻辑与站点一致
    let snapped = null;
    let snapDist = Infinity;
    for (const station of state.stations) {
      const d = Math.hypot(station.x - world.x, station.y - world.y);
      if (d < ROUTE_SNAP_DISTANCE && d < snapDist) {
        snapped = station;
        snapDist = d;
      }
    }
    // 防重复：同一站点已有绑定节点、或点击位置已有重合节点时不再新建。
    // 此前连点会生成多个坐标完全相同的堆叠节点，导致路网拓扑混乱、
    // 目标站点匹配到孤立节点而无法规划路径。
    let node = null;
    if (snapped) {
      node = state.routeGraph.nodes.find((item) => item.station === snapped.name);
      if (node) log(`站点 ${snapped.name} 已绑定节点 ${node.name}，不重复创建`);
    }
    if (!node) {
      const candidate = snapped
        ? { name: nextRouteNodeName(), x: snapped.x, y: snapped.y, yaw: snapped.yaw, station: snapped.name }
        : { name: nextRouteNodeName(), x: Number(world.x.toFixed(4)), y: Number(world.y.toFixed(4)), yaw: 0 };
      const overlap = state.routeGraph.nodes.find(
        (item) => Math.hypot(item.x - candidate.x, item.y - candidate.y) < 0.05);
      if (overlap) {
        log(`该位置已有节点 ${overlap.name}，不重复创建（如需重画请先在删除模式移除）`);
      } else {
        node = candidate;
        state.routeGraph.nodes.push(node);
        if (snapped) log(`路网节点 ${node.name} 已吸附站点 ${snapped.name}`);
      }
    }
  } else if (state.routeEditMode === "link") {
    // 优先命中紫色节点；没点中节点时再看蓝色站点——直接点站点也能接入路网：
    // 该站点已有绑定节点就复用，否则自动吸附生成一个再连线
    let node = hitRouteNode(world.x, world.y);
    if (!node) {
      const station = hitStationForRoute(world.x, world.y);
      if (station) {
        node = state.routeGraph.nodes.find((item) => item.station === station.name);
        if (!node) {
          node = {
            name: nextRouteNodeName(),
            x: station.x,
            y: station.y,
            yaw: station.yaw,
            station: station.name,
          };
          state.routeGraph.nodes.push(node);
          log(`路网节点 ${node.name} 已吸附站点 ${station.name}`);
        }
      }
    }
    if (!node) {
      state.routeLinkSource = null;
    } else if (!state.routeLinkSource) {
      state.routeLinkSource = node.name;
    } else if (state.routeLinkSource === node.name) {
      state.routeLinkSource = null;
    } else {
      const a = state.routeLinkSource;
      const b = node.name;
      const duplicate = state.routeGraph.edges.some(
        (edge) => (edge.a === a && edge.b === b) || (edge.a === b && edge.b === a),
      );
      if (!duplicate) {
        state.routeGraph.edges.push({ a, b });
        log(`路网连线：${a} ↔ ${b}`);
      } else {
        log(`${a} 与 ${b} 之间已有连线`);
      }
      state.routeLinkSource = null;
    }
  } else if (state.routeEditMode === "delete") {
    const node = hitRouteNode(world.x, world.y);
    if (node) {
      state.routeGraph.edges = state.routeGraph.edges.filter(
        (edge) => edge.a !== node.name && edge.b !== node.name,
      );
      state.routeGraph.nodes = state.routeGraph.nodes.filter(
        (item) => item.name !== node.name,
      );
      if (state.routeLinkSource === node.name) state.routeLinkSource = null;
      log(`已删除节点 ${node.name} 及其所有连线`);
    } else {
      const edgeIndex = hitRouteEdge(world.x, world.y);
      if (edgeIndex !== null) {
        const [removed] = state.routeGraph.edges.splice(edgeIndex, 1);
        log(`已删除连线 ${removed.a} ↔ ${removed.b}`);
      }
    }
  }
  updateRouteHint();
  drawMap();
}

async function refreshRouteGraph() {
  try {
    const result = await callService(
      "/delivery/route",
      "originbot_msgs/srv/RouteGraph",
      { command: "list", graph_json: "" },
    );
    if (!result.success) {
      log(`读取路网失败：${result.message}`);
      return;
    }
    state.routeGraph = JSON.parse(result.graph_json || "{\"nodes\":[],\"edges\":[]}");
    if (!Array.isArray(state.routeGraph.nodes)) state.routeGraph.nodes = [];
    if (!Array.isArray(state.routeGraph.edges)) state.routeGraph.edges = [];
    updateRouteHint();
    drawMap();
  } catch (error) {
    log(`读取路网失败：${error}`);
  }
}

async function saveRouteGraph() {
  try {
    const result = await callService(
      "/delivery/route",
      "originbot_msgs/srv/RouteGraph",
      { command: "save", graph_json: JSON.stringify(state.routeGraph) },
    );
    log(result.message);
    if (result.success && result.graph_json) {
      state.routeGraph = JSON.parse(result.graph_json);
      updateRouteHint();
      drawMap();
    }
  } catch (error) {
    log(`保存路网失败：${error}`);
  }
}

async function clearRouteGraph() {
  if (!window.confirm("确认清空整张路网？此操作会立即保存生效。")) return;
  try {
    const result = await callService(
      "/delivery/route",
      "originbot_msgs/srv/RouteGraph",
      { command: "clear", graph_json: "" },
    );
    log(result.message);
    state.routeGraph = { nodes: [], edges: [] };
    state.routeLinkSource = null;
    updateRouteHint();
    drawMap();
  } catch (error) {
    log(`清空路网失败：${error}`);
  }
}

function worldToCanvas(x, y) {
  if (!state.map) return null;
  const s = renderScale();
  const mx = (x - state.map.info.origin.position.x) / state.map.info.resolution;
  const my = (y - state.map.info.origin.position.y) / state.map.info.resolution;
  return { x: mx * s, y: (state.map.info.height - my) * s };
}

function canvasPointFromEvent(event) {
  const rect = mapCanvas.getBoundingClientRect();
  // 画布现在固定铺满取景框（不再用 object-fit 把位图缩进去），所以"事件坐标 →
  // 基准坐标"就是逆着视图变换算一次。返回的仍是基准坐标 —— 取点、拖拽朝向、
  // 路网编辑这些既有逻辑全都建立在这套坐标上，不用跟着改。
  const px = event.clientX - rect.left;
  const py = event.clientY - rect.top;
  return {
    x: (px - mapView.tx) / mapView.scale,
    y: (py - mapView.ty) / mapView.scale,
  };
}

function canvasToWorld(point) {
  if (!state.map || !point) return null;
  const s = renderScale();
  const { origin, resolution, height } = state.map.info;
  return {
    x: origin.position.x + (point.x / s) * resolution,
    y: origin.position.y + (height - point.y / s) * resolution,
  };
}

function updatePickButtonState() {
  document.querySelectorAll(".pick-button").forEach((button) => {
    const target = button.dataset.pickTarget;
    button.classList.toggle("active", state.pickTarget === target);
  });
  const hint = document.getElementById("pick-hint");
  if (!state.pickTarget) {
    hint.textContent = "点击“地图取点”后，可在地图上点击定位并拖拽设置朝向";
  } else if (state.pickTarget === "station") {
    hint.textContent = "当前正在为站点取点：点击确定位置，按住拖拽设置朝向";
  } else if (state.pickTarget === "pose") {
    hint.textContent = "当前正在设定位姿：点击确定机器人位置，按住拖拽设置朝向";
  } else {
    hint.textContent = "当前正在为 ArUco 标志位取点：点击确定位置，按住拖拽设置朝向";
  }
}

function setPickTarget(target) {
  if (state.pickTarget === target) {
    state.pickTarget = null;
  } else {
    state.pickTarget = target;
  }
  // 与路网编辑模式互斥：两者都消费画布点击，同时激活会互相拦截
  // （此前"路网添加节点 + 设定位姿"同时开启时，取点点击会被路网吞掉）
  if (state.pickTarget && state.routeEditMode) {
    setRouteEditMode(null);
  }
  state.pickStartCanvas = null;
  state.pickCurrentCanvas = null;
  updatePickButtonState();
  drawMap();
}

async function refreshMaps() {
  try {
    const result = await callService(
      "/delivery/map_file",
      "originbot_msgs/srv/MapFile",
      { command: "list", map_name: "" },
    );
    const select = document.getElementById("map-select");
    if (!select) return;
    select.innerHTML = "";
    const names = result.map_names || [];
    if (!names.length) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "暂无已保存地图";
      select.appendChild(option);
      return;
    }
    for (const name of names) {
      const option = document.createElement("option");
      option.value = name;
      option.textContent = name;
      select.appendChild(option);
    }
  } catch (error) {
    log(`获取地图列表失败：${error}`);
  }
}

async function openSelectedMap() {
  const select = document.getElementById("map-select");
  const mapName = select ? select.value : "";
  if (!mapName) {
    log("请先选择地图");
    return;
  }
  try {
    const result = await callService(
      "/delivery/map_file",
      "originbot_msgs/srv/MapFile",
      { command: "open", map_name: mapName },
    );
    log(result.message);
    if (!result.success) {
      log(`打开地图失败：${result.message}`);
    }
  } catch (error) {
    log(`打开地图请求失败：${error}`);
  }
}

async function closeMap() {
  try {
    const result = await callService(
      "/delivery/map_file",
      "originbot_msgs/srv/MapFile",
      { command: "close", map_name: "" },
    );
    log(result.message);
  } catch (error) {
    log(`关闭地图请求失败：${error}`);
  }
}

function fillPickForm(startWorld, endWorld, yaw) {
  if (!startWorld) return;
  const target = state.pickTarget;
  if (target === "station") {
    const form = document.getElementById("station-form");
    form.elements.x.value = startWorld.x.toFixed(3);
    form.elements.y.value = startWorld.y.toFixed(3);
    form.elements.yaw.value = yaw.toFixed(3);
  } else if (target === "marker") {
    const form = document.getElementById("marker-form");
    form.elements.x.value = startWorld.x.toFixed(3);
    form.elements.y.value = startWorld.y.toFixed(3);
    form.elements.yaw.value = yaw.toFixed(3);
  } else if (target === "pose") {
    callService(
      "/delivery/set_pose",
      "originbot_msgs/srv/SetPose",
      { x: startWorld.x, y: startWorld.y, yaw },
    )
      .then((result) => log(result.message))
      .catch((error) => log(`设定位姿失败：${error}`));
  }
  log(
    `已取点：x=${startWorld.x.toFixed(3)}，y=${startWorld.y.toFixed(3)}，yaw=${yaw.toFixed(3)}`,
  );
}

function startMapPick(event) {
  if (!state.pickTarget || !state.map) return;
  event.preventDefault();
  state.pickStartCanvas = canvasPointFromEvent(event);
  state.pickCurrentCanvas = state.pickStartCanvas;
}

function moveMapPick(event) {
  if (!state.pickStartCanvas) return;
  event.preventDefault();
  state.pickCurrentCanvas = canvasPointFromEvent(event);
  drawMap();
}

function endMapPick(event) {
  if (!state.pickStartCanvas || !state.map) return;
  event.preventDefault();
  const endCanvas = canvasPointFromEvent(event);
  const startWorld = canvasToWorld(state.pickStartCanvas);
  const endWorld = canvasToWorld(endCanvas);
  let yaw = 0;
  if (startWorld && endWorld) {
    const dx = endWorld.x - startWorld.x;
    const dy = endWorld.y - startWorld.y;
    if (Math.hypot(dx, dy) > 0.001) {
      yaw = Math.atan2(dy, dx);
    }
  }
  fillPickForm(startWorld, endWorld, yaw);
  state.pickTarget = null;
  state.pickStartCanvas = null;
  state.pickCurrentCanvas = null;
  updatePickButtonState();
  drawMap();
}

// 画一条带箭头的线段（canvas 坐标系），用于表示站点/标志位/机器人的朝向。
// 世界系 yaw 逆时针为正，canvas 的 y 轴向下，调用处需把 dy 取负。
function drawArrow(x0, y0, x1, y1, headSize, color, lineWidth) {
  const angle = Math.atan2(y1 - y0, x1 - x0);
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(
    x1 - headSize * Math.cos(angle - Math.PI / 6),
    y1 - headSize * Math.sin(angle - Math.PI / 6),
  );
  ctx.lineTo(
    x1 - headSize * Math.cos(angle + Math.PI / 6),
    y1 - headSize * Math.sin(angle + Math.PI / 6),
  );
  ctx.closePath();
  ctx.fill();
}

// 画带白色描边的文字标签，保证在地图深浅背景上都可读
function drawLabel(text, x, y, fontSize, color) {
  // 屏幕上的字号保持恒定：视图放大时按比例把基准字号缩回去，否则放到 8 倍时
  // 标签会撑满整屏。写字是矢量绘制，缩小后再被视图变换放大，依然清晰。
  const k = mapView.scale > 0 ? mapView.fit / mapView.scale : 1;
  const px = Math.max(1, fontSize * k);
  ctx.font = `bold ${px}px sans-serif`;
  ctx.textBaseline = "alphabetic";
  ctx.lineJoin = "round";
  ctx.lineWidth = Math.max(2 * k, px * 0.3);
  ctx.strokeStyle = "rgba(255, 255, 255, 0.92)";
  ctx.strokeText(text, x, y);
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
}

function drawMap() {
  if (!state.map) return;
  const s = renderScale();
  const box = mapViewBox();
  // 位图 = 取景框 CSS 尺寸 × dpr；只在尺寸变化时重设（重设 width 会清空画布）
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const bw = Math.round(box.w * dpr);
  const bh = Math.round(box.h * dpr);
  if (mapCanvas.width !== bw || mapCanvas.height !== bh) {
    mapCanvas.width = bw;
    mapCanvas.height = bh;
  }
  // 首帧（或布局刚完成）时把"1 倍"基准算出来
  if (mapView.fit <= 0) fitMapView(false);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, bw, bh);
  // 视图变换：基准坐标 → 屏幕像素（含 dpr）。下面所有绘制都用基准坐标，
  // 于是地图、点云、路网、站点、机器人自动一起缩放。
  ctx.setTransform(
    dpr * mapView.scale, 0,
    0, dpr * mapView.scale,
    dpr * mapView.tx, dpr * mapView.ty,
  );
  const raster = ensureMapRaster();
  if (raster) {
    // 占用栅格放大时保持格子边界清晰，不做插值糊化
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(raster, 0, 0);
  }

  // 激光雷达点云(地图坐标系)：以绿色小方块批量绘制，10Hz 下保持流畅
  if (state.showScan && state.scanCloud && state.scanCloud.length) {
    ctx.fillStyle = "#00e676";
    const cloud = state.scanCloud;
    const dot = Math.max(2, Math.round(s / 2));
    for (let i = 0; i < cloud.length; i += 1) {
      const point = worldToCanvas(cloud[i][0], cloud[i][1]);
      if (point) {
        ctx.fillRect(point.x, point.y, dot, dot);
      }
    }
  }

  // 路网：紫色折线（边）+ 紫色圆点（节点），画在站点之下、点云之上。
  if (state.routeGraph.edges.length || state.routeGraph.nodes.length) {
    ctx.lineCap = "round";
    for (const edge of state.routeGraph.edges) {
      const a = state.routeGraph.nodes.find((n) => n.name === edge.a);
      const b = state.routeGraph.nodes.find((n) => n.name === edge.b);
      if (!a || !b) continue;
      const pts = [a, ...(edge.via || []).map(([vx, vy]) => ({ x: vx, y: vy })), b];
      ctx.strokeStyle = "rgba(147, 51, 234, 0.9)";
      ctx.lineWidth = Math.max(2, s * 0.5);
      ctx.setLineDash([]);
      ctx.beginPath();
      pts.forEach((p, i) => {
        const point = worldToCanvas(p.x, p.y);
        if (!point) return;
        if (i === 0) ctx.moveTo(point.x, point.y);
        else ctx.lineTo(point.x, point.y);
      });
      ctx.stroke();
      // 边中点画一个小箭头表示这段轨道（双向可行，箭头仅示意路径）
      if (pts.length >= 2) {
        const mid = pts[Math.floor(pts.length / 2)];
        const prev = pts[Math.floor(pts.length / 2) - 1];
        const pm = worldToCanvas(mid.x, mid.y);
        const pp = worldToCanvas(prev.x, prev.y);
        if (pm && pp) {
          const angle = Math.atan2(pm.y - pp.y, pm.x - pp.x);
          ctx.fillStyle = "rgba(147, 51, 234, 0.9)";
          ctx.beginPath();
          ctx.moveTo(pm.x, pm.y);
          ctx.lineTo(pm.x - 6 * Math.cos(angle - Math.PI / 6), pm.y - 6 * Math.sin(angle - Math.PI / 6));
          ctx.lineTo(pm.x - 6 * Math.cos(angle + Math.PI / 6), pm.y - 6 * Math.sin(angle + Math.PI / 6));
          ctx.closePath();
          ctx.fill();
        }
      }
    }
    const routeR = Math.max(3, s * 0.8);
    for (const node of state.routeGraph.nodes) {
      const point = worldToCanvas(node.x, node.y);
      if (!point) continue;
      ctx.beginPath();
      ctx.arc(point.x, point.y, routeR, 0, Math.PI * 2);
      ctx.fillStyle = "#9333ea";
      ctx.fill();
      ctx.strokeStyle = "rgba(255, 255, 255, 0.9)";
      ctx.lineWidth = Math.max(1, s * 0.2);
      ctx.stroke();
      // 节点朝向短刻线
      ctx.strokeStyle = "#9333ea";
      ctx.lineWidth = Math.max(1.2, s * 0.3);
      ctx.beginPath();
      ctx.moveTo(point.x, point.y);
      ctx.lineTo(
        point.x + Math.cos(node.yaw || 0) * routeR * 2.0,
        point.y - Math.sin(node.yaw || 0) * routeR * 2.0,
      );
      ctx.stroke();
      if (state.routeLinkSource === node.name) {
        ctx.beginPath();
        ctx.arc(point.x, point.y, routeR + 3, 0, Math.PI * 2);
        ctx.strokeStyle = "#f59e0b";
        ctx.lineWidth = Math.max(1.5, s * 0.4);
        ctx.stroke();
      }
      drawLabel(node.name, point.x + routeR + 2, point.y - routeR - 2, Math.max(9, Math.round(s * 1.4)), "#6b21a8");
    }
  }

  // 站点：蓝色圆点 + 朝向箭头 + 名称标签
  const stationR = Math.max(4, s * 1.1);
  const labelFontSize = Math.max(10, Math.round(s * 1.8));
  for (const station of state.stations) {
    const point = worldToCanvas(station.x, station.y);
    if (!point) continue;
    const arrowLen = stationR * 2.2;
    drawArrow(
      point.x,
      point.y,
      point.x + Math.cos(station.yaw) * arrowLen,
      point.y - Math.sin(station.yaw) * arrowLen,
      Math.max(4, s * 1.2),
      "#1677ff",
      Math.max(1.5, s * 0.4),
    );
    ctx.beginPath();
    ctx.arc(point.x, point.y, stationR, 0, Math.PI * 2);
    ctx.fillStyle = "#1677ff";
    ctx.fill();
    ctx.strokeStyle = "rgba(255, 255, 255, 0.9)";
    ctx.lineWidth = Math.max(1, s * 0.2);
    ctx.stroke();
    drawLabel(
      station.name,
      point.x + stationR + 2,
      point.y - stationR - 2,
      labelFontSize,
      "#0b3ea8",
    );
  }

  // ArUco 标志位：橙色方块 + 存储朝向刻线 + 编号标签（A+id）
  const markerHalf = Math.max(4, s * 1.1);
  for (const marker of state.markers) {
    const point = worldToCanvas(marker.x, marker.y);
    if (!point) continue;
    ctx.save();
    ctx.translate(point.x, point.y);
    // markers.json 的 yaw = 标志位 X 轴在世界系的方向；canvas y 轴向下取负
    ctx.rotate(-marker.yaw);
    ctx.fillStyle = "rgba(250, 140, 22, 0.85)";
    ctx.strokeStyle = "#d46b08";
    ctx.lineWidth = Math.max(1.2, s * 0.25);
    ctx.beginPath();
    ctx.rect(-markerHalf, -markerHalf, markerHalf * 2, markerHalf * 2);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(markerHalf * 1.7, 0);
    ctx.stroke();
    ctx.restore();
    drawLabel(
      `A${marker.id}`,
      point.x + markerHalf + 2,
      point.y + labelFontSize * 0.4,
      labelFontSize,
      "#d46b08",
    );
  }

  const liveRobotPose = getRobotMapPose();
  if (liveRobotPose) {
    const point = worldToCanvas(liveRobotPose.x, liveRobotPose.y);
    if (point) {
      const yaw = liveRobotPose.yaw;
      const robotR = Math.max(4, s);
      ctx.fillStyle = "#e02020";
      ctx.beginPath();
      ctx.arc(point.x, point.y, robotR, 0, Math.PI * 2);
      ctx.fill();

      const arrowLength = Math.max(10, s * 3.5);
      const arrowEndX = point.x + Math.cos(yaw) * arrowLength;
      const arrowEndY = point.y - Math.sin(yaw) * arrowLength;
      ctx.strokeStyle = "#e02020";
      ctx.lineWidth = Math.max(1.5, s * 0.4);
      ctx.beginPath();
      ctx.moveTo(point.x, point.y);
      ctx.lineTo(arrowEndX, arrowEndY);
      ctx.stroke();

      const arrowHeadSize = Math.max(4, s * 1.3);
      const angle = Math.atan2(
        arrowEndY - point.y,
        arrowEndX - point.x,
      );
      ctx.fillStyle = "#d4380d";
      ctx.beginPath();
      ctx.moveTo(arrowEndX, arrowEndY);
      ctx.lineTo(
        arrowEndX - arrowHeadSize * Math.cos(angle - Math.PI / 6),
        arrowEndY - arrowHeadSize * Math.sin(angle - Math.PI / 6),
      );
      ctx.lineTo(
        arrowEndX - arrowHeadSize * Math.cos(angle + Math.PI / 6),
        arrowEndY - arrowHeadSize * Math.sin(angle + Math.PI / 6),
      );
      ctx.closePath();
      ctx.fill();
    }
  }

  if (state.pickTarget && state.pickStartCanvas && state.pickCurrentCanvas) {
    const start = state.pickStartCanvas;
    const end = state.pickCurrentCanvas;
    ctx.strokeStyle = "#ff7a45";
    ctx.lineWidth = Math.max(1.5, s * 0.4);
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(start.x, start.y, Math.max(4, s), 0, Math.PI * 2);
    ctx.fillStyle = "#ff7a45";
    ctx.fill();
  }

  // 收尾：恢复单位变换，避免影响后续在画布上的其它绘制
  ctx.setTransform(1, 0, 0, 1, 0, 0);
}

document.getElementById("connection-form").addEventListener("submit", (event) => {
  event.preventDefault();
  connect();
});

document.querySelectorAll("[data-subsystem]").forEach((button) => {
  button.addEventListener("click", async () => {
    await controlSystem(button.dataset.subsystem, button.dataset.enable === "true");
  });
});

document.getElementById("save-map").addEventListener("click", async () => {
  const mapName = document.getElementById("map-name").value.trim() || "delivery_map";
  const result = await callService(
    "/delivery/save_map",
    "originbot_msgs/srv/SaveMap",
    { map_name: mapName },
  );
  log(result.message);
});

// 安全关机：先弹确认框，确认后调 /delivery/shutdown（服务端会先停子系统再 poweroff）
document.getElementById("shutdown").addEventListener("click", async () => {
  const confirmed = window.confirm(
    "确认安全关机？\n\n"
    + "将停止底盘/建图/导航等子系统，然后关闭系统（等效 systemctl poweroff）。\n"
    + "关机前建议先保存地图；系统指示灯熄灭前请不要直接断电。",
  );
  if (!confirmed) {
    log("已取消关机");
    return;
  }
  try {
    const result = await callService("/delivery/shutdown", "std_srvs/srv/Trigger", {});
    log(result.message);
  } catch (error) {
    log(`关机请求失败：${error}`);
  }
});

document.getElementById("open-map").addEventListener("click", openSelectedMap);
document.getElementById("close-map").addEventListener("click", closeMap);

// 激光点云显示/隐藏开关
{
  const scanToggle = document.getElementById("toggle-scan");
  if (scanToggle) {
    scanToggle.addEventListener("click", () => {
      state.showScan = !state.showScan;
      scanToggle.textContent = `激光点云：${state.showScan ? "开" : "关"}`;
      drawMap();
    });
  }
}

document.getElementById("station-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const form = new FormData(event.target);
  const result = await callService(
    "/delivery/station",
    "originbot_msgs/srv/StationEntry",
    {
      command: "add",
      name: form.get("name"),
      x: Number(form.get("x")),
      y: Number(form.get("y")),
      yaw: Number(form.get("yaw") || 0),
    },
  );
  log(result.message);
  refreshStations();
});

document.getElementById("marker-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const form = new FormData(event.target);
  const result = await callService(
    "/delivery/marker",
    "originbot_msgs/srv/MarkerEntry",
    {
      command: "add",
      marker_id: Number(form.get("id")),
      marker_size: Number(form.get("size")),
      x: Number(form.get("x")),
      y: Number(form.get("y")),
      yaw: Number(form.get("yaw") || 0),
    },
  );
  log(result.message);
  refreshMarkers();
});

document.getElementById("navigate").addEventListener("click", async () => {
  const stationName = document.getElementById("station-select").value;
  if (!stationName) return;
  if (document.getElementById("nav-mode").value === "route" && !state.routeGraph.nodes.length) {
    const proceed = window.confirm(
      "路网为空：轨道路线导航需要先在“路网”面板绘制路网。\n\n"
      + "点击“确定”切换到自由避障导航，点击“取消”去绘制路网。");
    if (!proceed) return;
    document.getElementById("nav-mode").value = "free";
  }
  const button = document.getElementById("navigate");
  button.disabled = true;
  state.navigationTarget = stationName;
  updateLocalizationStatus();
  try {
    // 全局代价地图的障碍层只标不擦（要靠激光射线穿过才擦除），历史上定位跑飞
    // 瞬间误标的“幽灵障碍”会一直残留，导致规划器报 No valid trajectories /
    // failed to generate a valid path。导航前先清空两个代价地图再发目标。
    try {
      await callService(
        "/global_costmap/clear_entirely_global_costmap",
        "nav2_msgs/srv/ClearEntireCostmap",
        {},
      );
      await callService(
        "/local_costmap/clear_entirely_local_costmap",
        "nav2_msgs/srv/ClearEntireCostmap",
        {},
      );
      log("已清空代价地图");
    } catch (clearError) {
      // 导航未启动时服务不存在，跳过即可，不阻塞导航请求
      log(`代价地图清理跳过：${clearError.message || clearError}`);
    }
    const result = await callService(
      "/delivery/navigate_to_station",
      "originbot_msgs/srv/NavigateToStation",
      {
        station_name: stationName,
        mode: document.getElementById("nav-mode").value || "free",
      },
    );
    log(result.message);
    if (
      result.message.includes("不可用") ||
      result.message.includes("被拒绝") ||
      result.message.includes("失败")
    ) {
      state.navigationTarget = null;
      updateLocalizationStatus();
    }
  } catch (error) {
    log(`导航请求失败：${error}`);
    state.navigationTarget = null;
    updateLocalizationStatus();
  } finally {
    button.disabled = false;
  }
});

document.getElementById("cancel-navigation").addEventListener("click", async () => {
  const result = await callService(
    "/delivery/cancel_navigation",
    "originbot_msgs/srv/CancelNavigation",
    {},
  );
  log(result.message);
  state.navigationTarget = null;
  updateLocalizationStatus();
});

document.getElementById("station-list").addEventListener("click", async (event) => {
  const button = event.target.closest("button[data-station]");
  if (!button) return;
  const result = await callService(
    "/delivery/station",
    "originbot_msgs/srv/StationEntry",
    { command: "delete", name: button.dataset.station, x: 0, y: 0, yaw: 0 },
  );
  log(result.message);
  refreshStations();
});

document.getElementById("marker-list").addEventListener("click", async (event) => {
  const button = event.target.closest("button[data-marker]");
  if (!button) return;
  const result = await callService(
    "/delivery/marker",
    "originbot_msgs/srv/MarkerEntry",
    {
      command: "delete",
      marker_id: Number(button.dataset.marker),
      marker_size: 0,
      x: 0,
      y: 0,
      yaw: 0,
    },
  );
  log(result.message);
  refreshMarkers();
});

document.getElementById("manual-mode").addEventListener("click", async () => {
  await setManualMode(!state.manualMode);
});

document.getElementById("control-mode-keyboard").addEventListener("click", () => {
  setControlMode("keyboard");
});

document.getElementById("control-mode-joystick").addEventListener("click", () => {
  setControlMode("joystick");
});

const joystickPad = document.getElementById("joystick-pad");
joystickPad.addEventListener("pointerdown", handleJoystickStart);
joystickPad.addEventListener("pointermove", handleJoystickMove);
joystickPad.addEventListener("pointerup", handleJoystickEnd);
joystickPad.addEventListener("pointercancel", handleJoystickEnd);

document.getElementById("station-pick").addEventListener("click", () => {
  setPickTarget("station");
});

document.getElementById("marker-pick").addEventListener("click", () => {
  setPickTarget("marker");
});

document.getElementById("pose-pick").addEventListener("click", () => {
  setPickTarget("pose");
});

// ---------- 路网编辑器按钮 ----------
document.getElementById("route-mode-add").addEventListener("click", () => setRouteEditMode("add"));
document.getElementById("route-mode-link").addEventListener("click", () => setRouteEditMode("link"));
document.getElementById("route-mode-delete").addEventListener("click", () => setRouteEditMode("delete"));
document.getElementById("route-save").addEventListener("click", saveRouteGraph);
document.getElementById("route-clear").addEventListener("click", clearRouteGraph);

document.getElementById("imu-calibrate").addEventListener("click", async () => {
  const resultElement = document.getElementById("imu-calibrate-result");
  resultElement.textContent = "正在发送标定指令...";
  try {
    const result = await callService(
      "/imu_calibrate",
      "std_srvs/srv/Trigger",
      {},
    );
    resultElement.textContent = result.success
      ? `标定成功：${result.message}`
      : `标定失败：${result.message}`;
  } catch (error) {
    resultElement.textContent = `标定请求失败：${error}`;
  }
});

// ---- 地图缩放 / 平移 ----
let mapPan = null; // 拖动平移的起点（画布内 CSS 坐标 + 当时的位移）

// 滚轮缩放：以光标为锚点，缩放后光标下的那个地图点保持不动
mapCanvas.addEventListener("wheel", (event) => {
  if (!state.map) return;
  event.preventDefault(); // 别让整页跟着滚
  const factor = event.deltaY < 0 ? 1.15 : 1 / 1.15;
  setMapZoom(mapView.zoom * factor, event);
}, { passive: false });

// 双击复位到"整张地图铺满取景框"
mapCanvas.addEventListener("dblclick", () => {
  if (!state.map) return;
  fitMapView(false);
  drawMap();
});

mapCanvas.addEventListener("mousedown", (event) => {
  if (state.routeEditMode) {
    handleRouteCanvasClick(event);
    return;
  }
  if (state.pickTarget) {
    startMapPick(event);
    return;
  }
  // 没有取点/路网编辑任务时，按住左键拖动 = 平移视图
  const rect = mapCanvas.getBoundingClientRect();
  mapPan = {
    x: event.clientX - rect.left,
    y: event.clientY - rect.top,
    tx: mapView.tx,
    ty: mapView.ty,
  };
  mapCanvas.style.cursor = "grabbing";
  event.preventDefault();
});

mapCanvas.addEventListener("mousemove", (event) => {
  if (!mapPan) return;
  const rect = mapCanvas.getBoundingClientRect();
  mapView.tx = mapPan.tx + (event.clientX - rect.left - mapPan.x);
  mapView.ty = mapPan.ty + (event.clientY - rect.top - mapPan.y);
  drawMap();
});

function endMapPan() {
  if (!mapPan) return;
  mapPan = null;
  mapCanvas.style.cursor = "crosshair";
}
mapCanvas.addEventListener("mouseup", endMapPan);
mapCanvas.addEventListener("mouseleave", endMapPan);
window.addEventListener("mouseup", endMapPan);

// 取景框尺寸变了：重算"1 倍"基准并保持当前放大倍数
window.addEventListener("resize", () => {
  if (!state.map) return;
  fitMapView(true);
  drawMap();
});

mapCanvas.addEventListener("mousemove", moveMapPick);
mapCanvas.addEventListener("mouseup", endMapPick);
mapCanvas.addEventListener("mouseleave", () => {
  if (state.pickStartCanvas) {
    state.pickCurrentCanvas = null;
    drawMap();
  }
});

window.addEventListener("keydown", (event) => {
  if (!state.manualMode) return;
  const target = event.target;
  if (
    target instanceof HTMLInputElement ||
    target instanceof HTMLTextAreaElement ||
    target instanceof HTMLSelectElement
  ) {
    return;
  }

  const key = event.key.toLowerCase();
  const controlKeys = ["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright", " "];
  if (!controlKeys.includes(key)) return;
  event.preventDefault();

  if (key === " ") {
    state.pressedKeys.clear();
    publishCmdVel(0, 0);
    log("已急停");
    return;
  }

  state.pressedKeys.add(key);
});

window.addEventListener("keyup", (event) => {
  if (!state.manualMode) return;
  const key = event.key.toLowerCase();
  state.pressedKeys.delete(key);
  if (state.pressedKeys.size === 0) {
    publishCmdVel(0, 0);
  }
});

window.addEventListener("blur", () => {
  if (state.manualMode) {
    state.pressedKeys.clear();
    publishCmdVel(0, 0);
  }
});

updatePickButtonState();
updateManualState();
updateLocalizationStatus();
updateRouteHint();
setInterval(updateLocalizationStatus, 1000);
setInterval(updateCameraStatus, 1000);
// 建图期间 ArUco 可能自动登记新标志位：定时轮询让地图上尽快显示
setInterval(() => {
  if (state.connected) refreshMarkers().catch(() => {});
}, 5000);
setConnectionState(false);
