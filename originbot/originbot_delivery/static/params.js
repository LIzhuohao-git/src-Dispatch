// 小车参数配置页：schema 驱动渲染，通过 /delivery/params 服务读写。
// 保存即落盘并生成各子系统配置文件；"保存并重启"会重启对应子系统使其立即生效。

const state = {
  ros: null,
  connected: false,
  sections: [],          // 服务端下发的参数 schema
  values: {},            // 当前值（含未保存的编辑）
};

const statusLog = document.getElementById("status-log");
const rosHostInput = document.getElementById("ros-host");
const PARAMS_SERVICE = "/delivery/params";
const PARAMS_TYPE = "originbot_msgs/srv/ParamsEntry";

// 参数组 → 受影响的 ROS 子系统（数组 = 需要依次重启的子系统，与控制台"系统"面板一致）
const RESTART_TARGET = {
  lidar: ["base"],
  imu: ["base"],
  camera: ["aruco"],
  slam_toolbox: ["mapping"],
  cartographer: ["mapping"],
  nav2: ["navigation"],
  // EKF 开关牵动 odom→base_footprint TF 的发布方切换，AMCL（导航子系统）也依赖该
  // TF，因此底盘、EKF、导航三者都要重启才能干净交接
  ekf: ["base", "localization", "navigation"],
  route: ["route"],
};
const RESTART_LABEL = {
  base: "底盘（雷达/IMU）",
  aruco: "ArUco 定位",
  mapping: "建图",
  navigation: "导航",
  localization: "定位",
  route: "路网跟随器",
};
const RESTART_WARN = {
  lidar: "将重启底盘，正在进行的建图/导航会中断，确认继续？",
  imu: "将重启底盘，正在进行的建图/导航会中断，确认继续？",
  camera: "将重启 ArUco 定位子系统，确认继续？（ArUco 未启动时仅保存）",
  slam_toolbox: "将重启建图，未保存的建图进度会丢失，请先保存地图。确认继续？",
  cartographer: "将重启建图，未保存的建图进度会丢失，请先保存地图。确认继续？",
  nav2: "将重启导航，重启后需要重新设定位姿，确认继续？",
  ekf: "将重启底盘与定位子系统，正在进行的建图/导航会中断，之后请重新设定位姿。确认继续？",
  route: "将重启路网跟随器，正在进行的轨道路线导航会中断，确认继续？",
};

function restartTargetLabel(sectionId) {
  const targets = RESTART_TARGET[sectionId] || [];
  return targets.map((subsystem) => RESTART_LABEL[subsystem] || "").join("与");
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
    const service = new ROSLIB.Service({ ros: state.ros, name, serviceType: type });
    service.callService(request, (result) => {
      if (result) resolve(result);
      else reject(new Error(`${name} 无返回结果`));
    }, reject);
  });
}

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

// ---------------- 参数读写 ----------------

async function refreshParams() {
  const result = await callService(PARAMS_SERVICE, PARAMS_TYPE, { command: "list" });
  if (!result.success) throw new Error(result.message);
  const payload = JSON.parse(result.data_json);
  state.sections = payload.sections;
  state.values = payload.values;
  renderAll();
}

async function saveSection(sectionId, { restart = false } = {}) {
  const values = state.values[sectionId] || {};
  const result = await callService(PARAMS_SERVICE, PARAMS_TYPE, {
    command: "set",
    section: sectionId,
    values_json: JSON.stringify(values),
  });
  if (!result.success) {
    log(`【${sectionTitle(sectionId)}】保存失败：${result.message}`);
    return false;
  }
  log(`【${sectionTitle(sectionId)}】${result.message}`);
  if (restart) await restartSubsystemFor(sectionId);
  return true;
}

async function resetSection(sectionId) {
  const result = await callService(PARAMS_SERVICE, PARAMS_TYPE, {
    command: "reset",
    section: sectionId,
  });
  log(`${result.success ? "" : "失败："}${result.message}`);
  if (result.success) await refreshParams();
}

async function restartSubsystemFor(sectionId) {
  const targets = RESTART_TARGET[sectionId] || [];
  if (!targets.length) return;
  // 两阶段重启：先按依赖反序全部停止，再按依赖顺序拉起。
  // 这样像 EKF 开关这种"TF 发布方切换"的联动参数不会出现新旧发布方并存。
  const stopped = [];
  for (const subsystem of [...targets].reverse()) {
    // 若本来就没在运行（返回"未运行"），只保存、不强行拉起
    const stopResult = await callService(
      "/delivery/control_system",
      "originbot_msgs/srv/ControlSystem",
      { subsystem, enable: false },
    );
    if ((stopResult.message || "").includes("未运行")) continue;
    stopped.push(subsystem);
    await delay(800);
  }
  if (!stopped.length) {
    log(`${restartTargetLabel(sectionId)}未在运行，参数将在下次启动时生效`);
    return;
  }
  for (const subsystem of stopped) {
    const startResult = await callService(
      "/delivery/control_system",
      "originbot_msgs/srv/ControlSystem",
      { subsystem, enable: true },
    );
    log(`已重启${RESTART_LABEL[subsystem]}：${startResult.message || ""}`);
    await delay(800);
  }
}

function sectionTitle(sectionId) {
  const section = state.sections.find((item) => item.id === sectionId);
  return section ? section.title : sectionId;
}

// ---------------- 渲染 ----------------

function renderAll() {
  const main = document.getElementById("params-main");
  main.textContent = "";
  for (const section of state.sections) {
    main.appendChild(renderSection(section));
  }
}

function renderSection(section) {
  const card = document.createElement("section");
  card.className = "params-panel";

  // 卡片头：标题 + 生效说明
  const header = document.createElement("div");
  header.className = "params-panel-header";
  const title = document.createElement("h2");
  title.textContent = section.title;
  header.appendChild(title);
  const apply = document.createElement("p");
  apply.className = "params-apply";
  apply.textContent = section.apply;
  header.appendChild(apply);
  card.appendChild(header);

  // 卡片体：参数（nav2 等大节按 group 分小节展示）
  const body = document.createElement("div");
  body.className = "params-body";
  const groups = new Map();
  for (const param of section.params) {
    const key = param.group || "";
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(param);
  }
  for (const [groupName, params] of groups) {
    if (groupName) {
      const subtitle = document.createElement("h3");
      subtitle.className = "params-group";
      subtitle.textContent = groupName;
      body.appendChild(subtitle);
    }
    const grid = document.createElement("div");
    grid.className = "params-grid";
    for (const param of params) grid.appendChild(renderParam(section, param));
    body.appendChild(grid);
  }
  card.appendChild(body);

  card.appendChild(buildActions(section));
  return card;
}

function buildActions(section) {
  const actions = document.createElement("div");
  actions.className = "params-actions";

  const restartButton = document.createElement("button");
  restartButton.type = "button";
  restartButton.className = "primary-button";
  restartButton.textContent = `保存并重启${restartTargetLabel(section.id)}`;
  restartButton.addEventListener("click", async () => {
    if (RESTART_TARGET[section.id] && !window.confirm(RESTART_WARN[section.id])) return;
    await saveSection(section.id, { restart: true });
  });
  actions.appendChild(restartButton);

  const saveButton = document.createElement("button");
  saveButton.type = "button";
  saveButton.textContent = "仅保存本节";
  saveButton.addEventListener("click", () => saveSection(section.id));
  actions.appendChild(saveButton);

  const resetButton = document.createElement("button");
  resetButton.type = "button";
  resetButton.className = "danger-soft";
  resetButton.textContent = "恢复本节默认";
  resetButton.addEventListener("click", () => {
    if (!window.confirm(`确定把「${section.title}」恢复为默认值？`)) return;
    resetSection(section.id);
  });
  resetButton.style.marginLeft = "auto";
  actions.appendChild(resetButton);

  return actions;
}

function renderParam(section, param) {
  const row = document.createElement("div");
  row.className = "param-row";

  // 左侧：名称 + 中文说明
  const info = document.createElement("div");
  info.className = "param-info";
  const label = document.createElement("label");
  label.className = "param-label";
  const labelText = document.createElement("span");
  labelText.className = "param-label-text";
  labelText.textContent = param.label;
  label.appendChild(labelText);
  // 原始英文参数名（ROS 参数键），便于与配置文件/官方文档对照
  const keyTag = document.createElement("code");
  keyTag.className = "param-key";
  keyTag.textContent = param.key;
  label.appendChild(keyTag);
  info.appendChild(label);
  const note = document.createElement("div");
  note.className = "param-note";
  note.textContent = param.note;
  info.appendChild(note);
  row.appendChild(info);

  // 右侧：控件
  row.appendChild(buildInput(section.id, param));
  return row;
}

function buildInput(sectionId, param) {
  const value = (state.values[sectionId] || {})[param.key];
  if (param.type === "bool") {
    // 开关样式的复选框
    const wrap = document.createElement("label");
    wrap.className = "param-bool";
    const input = document.createElement("input");
    input.type = "checkbox";
    input.checked = Boolean(value);
    const track = document.createElement("span");
    track.className = "param-switch";
    const text = document.createElement("span");
    text.className = "param-bool-text";
    const sync = () => {
      state.values[sectionId][param.key] = input.checked;
      text.textContent = input.checked ? "开" : "关";
    };
    input.addEventListener("change", sync);
    text.textContent = input.checked ? "开" : "关";
    wrap.appendChild(input);
    wrap.appendChild(track);
    wrap.appendChild(text);
    return wrap;
  }
  if (param.type === "enum") {
    const select = document.createElement("select");
    for (const option of param.options) {
      const item = document.createElement("option");
      item.value = option;
      item.textContent = option;
      select.appendChild(item);
    }
    select.value = String(value);
    select.addEventListener("change", () => {
      state.values[sectionId][param.key] = select.value;
    });
    return select;
  }
  const input = document.createElement("input");
  input.type = "number";
  if (param.min !== undefined) input.min = param.min;
  if (param.max !== undefined) input.max = param.max;
  // 浮点参数用 step="any"：标定系数这类需要精确到小数后 6 位的值，
  // 若按 schema 的 step 生成步进（如 0.01），浏览器会因"步进不匹配"
  // 拒绝提交，用户只能靠上下箭头一点点调。
  if (param.type === "int") {
    if (param.step !== undefined) input.step = param.step;
  } else {
    input.step = "any";
  }
  input.value = String(value);
  input.addEventListener("change", () => {
    let number = Number(input.value);
    if (Number.isNaN(number)) return;
    if (param.type === "int") number = Math.round(number);
    if (param.min !== undefined) number = Math.max(number, param.min);
    if (param.max !== undefined) number = Math.min(number, param.max);
    input.value = String(number);
    state.values[sectionId][param.key] = number;
  });
  return input;
}

// ---------------- 导出 / 导入 ----------------

function exportParams() {
  const payload = {
    version: 1,
    exported_at: new Date().toISOString(),
    values: state.values,
  };
  const blob = new Blob([JSON.stringify(payload, null, 2)], {
    type: "application/json",
  });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = `originbot_params_${new Date().toISOString().slice(0, 10)}.json`;
  link.click();
  URL.revokeObjectURL(url);
  log("参数已导出为 JSON 文件");
}

async function importParams(file) {
  let payload;
  try {
    payload = JSON.parse(await file.text());
  } catch (error) {
    log(`导入失败：文件不是合法的 JSON（${error.message}）`);
    return;
  }
  const values = payload && payload.values ? payload.values : payload;
  if (!values || typeof values !== "object") {
    log("导入失败：文件缺少参数内容（应为导出的 JSON 或 {参数组: {参数: 值}}）");
    return;
  }
  let applied = 0;
  for (const section of state.sections) {
    const sectionValues = values[section.id];
    if (!sectionValues || typeof sectionValues !== "object") continue;
    // 只提交本节存在的键，交由服务端逐项校验
    const filtered = {};
    for (const param of section.params) {
      if (param.key in sectionValues) filtered[param.key] = sectionValues[param.key];
    }
    if (!Object.keys(filtered).length) continue;
    const result = await callService(PARAMS_SERVICE, PARAMS_TYPE, {
      command: "set",
      section: section.id,
      values_json: JSON.stringify(filtered),
    });
    if (result.success) applied += 1;
    else log(`导入「${section.title}」失败：${result.message}`);
  }
  await refreshParams();
  log(`导入完成：${applied} 个参数组已更新，按各节提示重启后生效`);
}

// ---------------- 连接 ----------------

function connect() {
  const host = rosHostInput.value.trim();
  const port = document.getElementById("ros-port").value.trim();
  if (state.ros) state.ros.close();
  state.ros = new ROSLIB.Ros({ url: `ws://${host}:${port}` });
  state.ros.on("connection", () => {
    setConnectionState(true);
    log(`已连接 ${host}:${port}`);
    refreshParams().catch((error) => log(`读取参数失败：${error.message}`));
  });
  state.ros.on("error", () => {});
  state.ros.on("close", () => setConnectionState(false));
}

if (!rosHostInput.value.trim() || rosHostInput.value.trim() === "127.0.0.1") {
  rosHostInput.value = window.location.hostname || "127.0.0.1";
}
document.getElementById("connection-form").addEventListener("submit", (event) => {
  event.preventDefault();
  connect();
});
document.getElementById("export-params").addEventListener("click", exportParams);
document.getElementById("import-params").addEventListener("click", () => {
  document.getElementById("import-file").click();
});
document.getElementById("import-file").addEventListener("change", (event) => {
  const file = event.target.files[0];
  if (file) importParams(file).catch((error) => log(`导入失败：${error.message}`));
  event.target.value = "";
});
document.getElementById("reset-all").addEventListener("click", () => {
  if (!window.confirm("确定恢复全部参数为默认值？当前所有修改都会丢失。")) return;
  callService(PARAMS_SERVICE, PARAMS_TYPE, { command: "reset", section: "all" })
    .then((result) => {
      log(result.message);
      return refreshParams();
    })
    .catch((error) => log(`恢复默认失败：${error.message}`));
});

connect();
