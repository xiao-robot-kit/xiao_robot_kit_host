
"use strict";
/* —— 页壳：模式（仿真/真实）与语言（中/EN）——
   加载时不启动任何数据源（禁隐式仿真/禁自动连 QEMU）；模式由用户点选：
   "sim" → bridge.openSim()（SimTrajSource 双流 + 阈值重建）；
   "real" → bridge.openPort(选中串口)。文案统一走 ui-shell 词典。 */
UI.define({
  zh: {
    "exp5.title": "exp5 · 3D 位移轨迹（惯性推算）",
    "exp5.trailTitle": "轨迹（世界系，Z 上 · 1 格 = 0.1 m）",
    "exp5.hint": "拖拽旋转 · 滚轮缩放 · 双击复位视角 · 亮端 = 当前位置，尾部渐隐 = 历史",
    "exp5.valsTitle": "实时数值",
    "exp5.noteTitle": "口径说明",
    "exp5.note": "四元数姿态 + 机体比力 → 世界系去重力 → 二重积分（<b>ZUPT 零速 + 高通 + 静止零偏学习</b>抑制漂移，算法住 backend/trajectory.py，页面只渲染）。<br>惯性推算误差随时间平方增长（研究文档口径）：本视图是<b>运动示意</b>（方向/形状/量级），不是米制计量；建议运动 &gt;20 cm 且带停顿，「停止放置」时轨迹收敛冻结。<br>算法在 bridge 侧单源运行，「清除」只清画面不清积分状态（断开重连即重置）。",
    "exp5.logTitle": "固件日志（TEXT 帧）",
    "exp5.clear": "清除轨迹",
    "exp5.g6pos": "位置（m，世界系）",
    "exp5.g6quat": "姿态四元数（QUAT 帧）",
    "exp5.g6acc": "机体比力（g，IMU 帧）",
    "exp5.statPos": "QUAT {q} · IMU {imu} · 位置 ({x}, {y}, {z}) m",
    "exp5.statDrop": "QUAT {q} · IMU {imu} · 丢帧 {drop} · 位置 ({x}, {y}, {z}) m",
  },
  en: {
    "exp5.title": "exp5 · 3D displacement trajectory (inertial)",
    "exp5.trailTitle": "Trajectory (world frame, Z up · 1 cell = 0.1 m)",
    "exp5.hint": "Drag to orbit · wheel to zoom · double-click to reset view · bright end = current position, fading tail = history",
    "exp5.valsTitle": "Live values",
    "exp5.noteTitle": "How to read",
    "exp5.note": "Quaternion attitude + body specific force → gravity removal in the world frame → double integration (<b>ZUPT zero-velocity + high-pass + static bias learning</b> limit the drift; the algorithm lives in backend/trajectory.py, this page only renders).<br>Inertial dead reckoning error grows with the square of time (per the research note): this view is a <b>motion illustration</b> (direction/shape/magnitude), not a metrology instrument. Prefer motion &gt;20 cm with pauses; when the board rests, the track freezes as it converges.<br>The algorithm runs once in the bridge; Clear only wipes the drawing, not the integration state (disconnect/reconnect resets it).",
    "exp5.logTitle": "Firmware log (TEXT frames)",
    "exp5.clear": "Clear trail",
    "exp5.g6pos": "Position (m, world frame)",
    "exp5.g6quat": "Attitude quaternion (QUAT frames)",
    "exp5.g6acc": "Body specific force (g, IMU frames)",
    "exp5.statPos": "QUAT {q} · IMU {imu} · pos ({x}, {y}, {z}) m",
    "exp5.statDrop": "QUAT {q} · IMU {imu} · {drop} dropped · pos ({x}, {y}, {z}) m",
  },
});
UI.init({page: "exp5"});   // 只渲染壳，不启动数据源

/* —— exp5 3D 轨迹渲染：Three.js（r128 本地副本，与 exp1 同引入方式）——
   轨迹线 = 单条 THREE.Line + 自研 ShaderMaterial（逐顶点 alpha）：
   最近 HEAD_N 点不透明、更旧线性渐隐至 0（尾部渐隐）；
   顶点缓冲预分配 MAX_PTS，滑窗用 copyWithin 整体左移（memmove，不重建）。
   数据驱动：bridge.posReady（backend/trajectory.py 重建的位置序列）推轨迹；
   bridge.frameReady 的 QUAT 帧（type=4）推姿态标记姿态角（复用 exp1 桥模式）。 */

const viewEl = document.getElementById("view3d");
const valsEl = document.getElementById("vals");
const logEl  = document.getElementById("log");
const statEl = document.getElementById("stat");
const statusEl = document.getElementById("status");

if (!window.THREE) {
  document.getElementById("hint").textContent = UI.t("common.threeMissing");
  statusEl.textContent = UI.t("common.threeMissing");
  throw new Error("three.js missing");
}

/* 视图参数（纯显示层，非算法参数） */
const TRAIL_MAX = 6000;     // 轨迹顶点容量（100Hz 下 60 s）
const TRAIL_HEAD = 150;     // 最近 N 点不透明（100Hz 下 1.5 s）
const GRID_M = 1.0;         // 网格边长（m）
const GRID_DIV = 10;        // 网格分割（1 格 = 0.1 m）
const AXIS_LEN = 0.28;      // 坐标轴长度（m）
const TRIAD_LEN = 0.09;     // 姿态标记三轴长（m）
const CAM0 = { theta: -0.7, phi: 0.45, dist: 1.5 };   // 初始/复位视角（球坐标）

const COL = { acc: 0x4fc3f7, x: 0xe57373, y: 0x81c784, z: 0x64b5f6,
              grid1: 0x2c3947, grid2: 0x1c2630 };

/* —— 场景骨架 —— */
const renderer = new THREE.WebGLRenderer({ canvas: viewEl, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio || 1);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, 2, 0.01, 50);
camera.up.set(0, 0, 1);                       // 世界 Z 上（与去重力 -z·g 口径一致）

function resize() {
  const w = Math.max(2, Math.floor(viewEl.clientWidth));
  const h = Math.max(2, Math.floor(viewEl.clientHeight));
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);

/* 网格（GridHelper 默认躺在 XZ 面/Y 上——世界 Z 上需绕 x 转 90°） */
const grid = new THREE.GridHelper(GRID_M, GRID_DIV, COL.grid1, COL.grid2);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

/* 坐标轴 + 轴名标签（自研 sprite，零外部字体/贴图资源） */
scene.add(new THREE.AxesHelper(AXIS_LEN));
function axisLabel(text, colorCss, x, y, z) {
  const cv = document.createElement("canvas");
  cv.width = cv.height = 64;
  const c2 = cv.getContext("2d");
  c2.fillStyle = colorCss;
  c2.font = "bold 40px sans-serif";
  c2.textAlign = "center";
  c2.textBaseline = "middle";
  c2.fillText(text, 32, 32);
  const sp = new THREE.Sprite(new THREE.SpriteMaterial(
    { map: new THREE.CanvasTexture(cv), depthTest: false }));
  sp.scale.set(0.07, 0.07, 1);
  sp.position.set(x, y, z);
  scene.add(sp);
}
axisLabel("X", "#e57373", AXIS_LEN + 0.06, 0, 0);
axisLabel("Y", "#81c784", 0, AXIS_LEN + 0.06, 0);
axisLabel("Z", "#64b5f6", 0, 0, AXIS_LEN + 0.06);

/* —— 轨迹线（逐顶点 alpha 渐隐）—— */
const posArr = new Float32Array(TRAIL_MAX * 3);
const alphaArr = new Float32Array(TRAIL_MAX);
let trailCount = 0, trailDirty = false;
const trailGeom = new THREE.BufferGeometry();
const posAttr = new THREE.BufferAttribute(posArr, 3);
posAttr.setUsage(THREE.DynamicDrawUsage);
const alphaAttr = new THREE.BufferAttribute(alphaArr, 1);
alphaAttr.setUsage(THREE.DynamicDrawUsage);
trailGeom.setAttribute("position", posAttr);
trailGeom.setAttribute("aAlpha", alphaAttr);
trailGeom.setDrawRange(0, 0);
const trailMat = new THREE.ShaderMaterial({
  transparent: true, depthWrite: false,
  uniforms: { uColor: { value: new THREE.Color(COL.acc) } },
  vertexShader: [
    "attribute float aAlpha;",
    "varying float vA;",
    "void main() {",
    "  vA = aAlpha;",
    "  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);",
    "}"
  ].join("\n"),
  fragmentShader: [
    "uniform vec3 uColor;",
    "varying float vA;",
    "void main() {",
    "  gl_FragColor = vec4(uColor, vA);",
    "}"
  ].join("\n")
});
scene.add(new THREE.Line(trailGeom, trailMat));

function pushTrail(x, y, z) {
  if (trailCount < TRAIL_MAX) {
    const o = trailCount * 3;
    posArr[o] = x; posArr[o + 1] = y; posArr[o + 2] = z;
    trailCount++;
  } else {                                   // 满窗：整体左移 1 点（memmove）
    posArr.copyWithin(0, 3);
    const o = (TRAIL_MAX - 1) * 3;
    posArr[o] = x; posArr[o + 1] = y; posArr[o + 2] = z;
  }
  trailDirty = true;
}

function updateTrailAlpha() {
  for (let i = 0; i < trailCount; i++) {
    const age = trailCount - 1 - i;           // 0 = 最新
    let a;
    if (age < TRAIL_HEAD) a = 1.0;
    else a = Math.max(0.0, 1.0 - (age - TRAIL_HEAD) /
                      Math.max(trailCount - TRAIL_HEAD, 1));
    alphaArr[i] = a;
  }
  alphaAttr.needsUpdate = true;
  posAttr.needsUpdate = true;
  trailGeom.setDrawRange(0, trailCount);
}

function clearTrail() {
  trailCount = 0;
  trailGeom.setDrawRange(0, 0);
  trailDirty = false;
}

/* —— 实时位置标记：小球 + 姿态三轴（QUAT 帧驱动）—— */
const marker = new THREE.Group();
marker.add(new THREE.Mesh(
  new THREE.SphereGeometry(0.016, 16, 12),
  new THREE.MeshBasicMaterial({ color: COL.acc })));
[[1, 0, 0, COL.x], [0, 1, 0, COL.y], [0, 0, 1, COL.z]].forEach(([dx, dy, dz, col]) => {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(
    new Float32Array([0, 0, 0, dx * TRIAD_LEN, dy * TRIAD_LEN, dz * TRIAD_LEN]), 3));
  marker.add(new THREE.Line(g, new THREE.LineBasicMaterial({ color: col })));
});
scene.add(marker);

/* —— 自研轨道控制（拖拽旋转 / 滚轮缩放 / 双击复位；零第三方）—— */
const cam = { theta: CAM0.theta, phi: CAM0.phi, dist: CAM0.dist };
function applyCam() {
  const cp = Math.cos(cam.phi);
  camera.position.set(cam.dist * cp * Math.cos(cam.theta),
                      cam.dist * cp * Math.sin(cam.theta),
                      cam.dist * Math.sin(cam.phi));
  camera.lookAt(0, 0, 0);
}
let dragging = false, lastX = 0, lastY = 0;
viewEl.addEventListener("mousedown", (e) => {
  dragging = true; lastX = e.clientX; lastY = e.clientY;
});
window.addEventListener("mousemove", (e) => {
  if (!dragging) return;
  cam.theta -= (e.clientX - lastX) * 0.008;
  cam.phi = Math.min(1.5, Math.max(-1.5, cam.phi + (e.clientY - lastY) * 0.008));
  lastX = e.clientX; lastY = e.clientY;
  applyCam();
});
window.addEventListener("mouseup", () => { dragging = false; });
viewEl.addEventListener("wheel", (e) => {
  e.preventDefault();
  cam.dist = Math.min(8.0, Math.max(0.3, cam.dist * (e.deltaY > 0 ? 1.12 : 0.9)));
  applyCam();
}, { passive: false });
viewEl.addEventListener("dblclick", () => {
  cam.theta = CAM0.theta; cam.phi = CAM0.phi; cam.dist = CAM0.dist;
  applyCam();
});

/* —— 数值面板 —— */
let lastPos = [0, 0, 0], lastQuat = null, lastAcc = null;
function renderVals() {
  const f = (v) => (v === null ? "—" : v.toFixed(3));
  const rows = [
    ["pos x", f(lastPos[0]) + " m"], ["pos y", f(lastPos[1]) + " m"],
    ["pos z", f(lastPos[2]) + " m"],
  ];
  const q = lastQuat;
  const qs = q === null ? ["—", "—", "—", "—"] : [
    q[0].toFixed(3), q[1].toFixed(3), q[2].toFixed(3), q[3].toFixed(3)];
  const a = lastAcc;
  const as_ = a === null ? ["—", "—", "—"] : [
    a[0].toFixed(3), a[1].toFixed(3), a[2].toFixed(3)];
  let html = `<div class="g6">${UI.t("exp5.g6pos")}</div>`;
  rows.forEach(([n, v]) => { html += `<div class="v">${n} <b>${v}</b></div>`; });
  html += `<div class="g6">${UI.t("exp5.g6quat")}</div>`;
  html += ["w", "x", "y", "z"].map((n, i) =>
    `<div class="v">q${n} <b>${qs[i]}</b></div>`).join("");
  html += `<div class="g6">${UI.t("exp5.g6acc")}</div>`;
  html += ["ax", "ay", "az"].map((n, i) =>
    `<div class="v">${n} <b>${as_[i]}</b></div>`).join("");
  valsEl.innerHTML = html;
}

/* —— 渲染循环 —— */
let lastDraw = 0;
function tick(t) {
  if (t - lastDraw > 40) {                    // 25fps 上限（与 exp1 同节奏）
    if (trailDirty) { updateTrailAlpha(); trailDirty = false; }
    renderer.render(scene, camera);
    lastDraw = t;
  }
  requestAnimationFrame(tick);
}

function log(msg, warn) {
  const d = document.createElement("div");
  if (warn) d.className = "fw";
  d.textContent = msg;
  logEl.appendChild(d);
  while (logEl.children.length > 200) logEl.removeChild(logEl.firstChild);
  logEl.scrollTop = logEl.scrollHeight;
}

let bridge = null, nQuat = 0, nImu = 0, lastDrop = 0;

/* 等 QWebChannel 桥就绪（既有 kick 契约：200ms 轮询） */
function waitBridge(fn) {
  const wait = setInterval(() => {
    if (bridge) { clearInterval(wait); fn(); }
  }, 200);
}

/* 旧 kick 契约（app.py 位置参数串口）：显式切真实模式 + 选端口 + 连接 */
function autoConnect(port) {
  UI.setMode("real", { silent: true });
  const sel = document.getElementById("port");
  const wait = setInterval(() => {
    if (sel.options.length) {
      clearInterval(wait);
      for (const o of sel.options) if (o.value === port) sel.value = port;
      if (!UI.connected) bridge.openPort(sel.value || port, 115200);
    }
  }, 200);
}

/* 旧 kick 契约（app.py --mode sim / Qt 页测试）：显式切仿真模式再启源。
   注意用词法 bridge（top-level let 不挂 window，window.bridge 恒 undefined） */
function startSim() {
  UI.setMode("sim", { silent: true });
  waitBridge(() => { if (!UI.connected) bridge.openSim(); });
}

/* 显式挂 window：app.py 经 runJavaScript("startSim()"/"autoConnect(...)") kick——
   顶层函数声明的隐式全局改为显式契约（app.py 与页面脚本非同源执行域） */
window.startSim = startSim;
window.autoConnect = autoConnect;

/* 页脚：位置为最新重建值（无 posReady 时显示 0.000） */
function renderStat(drop) {
  const p = lastPos.map((v) => v.toFixed(3));
  statEl.textContent = UI.t("exp5.statPos", { q: nQuat, imu: nImu, x: p[0], y: p[1], z: p[2] });
  if (drop) statEl.textContent = UI.t("exp5.statDrop", {
    q: nQuat, imu: nImu, drop, x: p[0], y: p[1], z: p[2] });
}

bridge = LiveBridge();   // 服务器版：SSE 收帧 + POST 命令（API 与旧 bridge 同形）
{
  /* 防御：Qt6 WebChannel 下 Slot 带返回值的方法在 JS 侧拿到的是 Promise
     （PySide6 6.12 实测，与 exp1 同坑）——Promise.resolve 兼容同步返回。 */
  Promise.resolve(bridge.listPorts()).then((ports) => {
    (Array.isArray(ports) ? ports : Array.from(ports || [])).forEach((p) => {
      const o = document.createElement("option");
      o.value = o.textContent = p;
      document.getElementById("port").appendChild(o);
    });
  });
  bridge.frameReady.connect((type, seq, vals) => {
    if (type === 4) {                         // QUAT 帧：姿态标记 + 读数
      nQuat++;
      lastQuat = vals.slice(0, 4);
      marker.quaternion.set(vals[1], vals[2], vals[3], vals[0]).normalize();
      renderVals();
    } else if (type === 1) {                  // IMU_SAMPLE：比力读数
      nImu++;
      lastAcc = vals.slice(0, 3);
      renderVals();
    }
  });
  bridge.posReady.connect((x, y, z) => {     // 重建位置：轨迹 + 标记
    lastPos = [x, y, z];
    marker.position.set(x, y, z);
    pushTrail(x, y, z);
    renderVals();
    renderStat(0);
  });
  bridge.textReady.connect((m) => log(m));
  bridge.statReady.connect((imu, text, drop) => {
    lastDrop = drop;
    renderStat(drop);
  });
  bridge.portChanged.connect((on) => {
    UI.setConnected(on);                      // 状态/按钮文案归壳（中英一致）
    if (!on) { lastQuat = null; lastAcc = null; renderVals(); }
  });
}

/* 语言切换：动态面板与页脚重新渲染 */
UI.onLangChange(() => { renderVals(); renderStat(lastDrop); });

/* 模式切换：先断开当前流（避免仿真与真实叠加），再由用户点按钮启源 */
UI.onModeChange(() => { if (UI.connected) bridge.closePort(); });

/* 按钮 = 显式动作分发：仿真→合成源，真实→串口；不自动、不回落 */
document.getElementById("btn").onclick = () => {
  if (!bridge) return;
  if (UI.connected) { bridge.closePort(); return; }
  if (UI.mode === "sim") { bridge.openSim(); return; }
  bridge.openPort(document.getElementById("port").value || "COM3", 115200);
};
document.getElementById("btn-clear").onclick = () => clearTrail();

resize();
applyCam();
renderVals();
requestAnimationFrame(tick);
