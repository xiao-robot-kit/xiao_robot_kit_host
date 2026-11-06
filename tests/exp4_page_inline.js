
"use strict";
/* —— 页壳：模式（仿真/真实）与语言（中/EN）——
   加载时不启动任何数据源（禁隐式仿真/禁自动连 QEMU）；模式由用户点选：
   "sim" → bridge.openSim()（SimTrajSource 摆动 QUAT 流）；
   "real" → bridge.openPort(选中串口)。文案统一走 ui-shell 词典。 */
UI.define({
  zh: {
    "exp4.title": "exp4 · 四元数姿态（0x04 · 16B · |q|≈1 主通道）",
    "exp4.cubeTitle": "机体立方（QUAT 帧驱动 · 世界 Z 上 · 网格/世界轴不动）",
    "exp4.hint": "拖拽旋转 · 滚轮缩放 · 双击复位 · 立方面色/体轴（X红 Y绿 Z蓝）随姿态转",
    "exp4.quatTitle": "四元数（QUAT 帧原值，w 在前）",
    "exp4.eulerTitle": "欧拉对照（ZYX · 本页从四元数解算，同固件 quat_to_euler 式）",
    "exp4.thAxis": "轴",
    "exp4.thDeg": "角度 (deg)",
    "exp4.thRange": "显示域",
    "exp4.locknote": "⚠ pitch 逼近 ±90°：万向锁区（roll/yaw 不再独立）",
    "exp4.noteTitle": "教学对照：exp3 ↔ exp4（同一 Mahony，两种姿态表示）",
    "exp4.note": "实验三（<code>exp3_euler</code>）固件发 <code>0x03</code> 欧拉帧——三参数 <b>ZYX</b>，人读直观，但 pitch ±90° 处<b>万向锁</b>丢一自由度；本实验固件发 <code>0x04</code> 四元数帧——4 参数<b>无奇异</b>，可插值可平滑连乘，是 3D/渲染主通道。右表欧拉是<b>本页从四元数现算</b>的对照（公式与固件 <code>quat_to_euler</code> 同式，含 ±90° 钉法）——与 exp3 页直读 0x03 帧对照着看：同一姿态、两条表示通道。",
    "exp4.logTitle": "固件日志（TEXT 帧）",
    "exp4.g6quat": "四分量（w,x,y,z · 帧原值）",
    "exp4.g6norm": "范数（|q|，发送端保持 ≈1）",
    "exp4.stat": "QUAT {q} · IMU {imu} · |q| {norm}",
    "exp4.webglOff": "WebGL 不可用——3D 视图已关闭（数值/欧拉面板照常；3D 旋转需 GPU 开窗验证）",
  },
  en: {
    "exp4.title": "exp4 · Quaternion attitude (0x04 · 16B · |q|≈1 main channel)",
    "exp4.cubeTitle": "Body cube (driven by QUAT frames · world Z up · grid/world axes fixed)",
    "exp4.hint": "Drag to orbit · wheel to zoom · double-click to reset · face colours / body axes (X red, Y green, Z blue) follow the attitude",
    "exp4.quatTitle": "Quaternion (raw QUAT frame values, w first)",
    "exp4.eulerTitle": "Euler reference (ZYX · computed on this page from the quaternion, same formula as firmware quat_to_euler)",
    "exp4.thAxis": "Axis",
    "exp4.thDeg": "Angle (deg)",
    "exp4.thRange": "Display range",
    "exp4.locknote": "⚠ pitch near ±90°: gimbal-lock region (roll/yaw no longer independent)",
    "exp4.noteTitle": "Teaching cross-reference: exp3 ↔ exp4 (same Mahony filter, two attitude representations)",
    "exp4.note": "Experiment 3 (<code>exp3_euler</code>) sends <code>0x03</code> Euler frames — three parameters in <b>ZYX</b>, easy to read, but at pitch ±90° <b>gimbal lock</b> costs one degree of freedom. This experiment sends <code>0x04</code> quaternion frames — four parameters, <b>singularity-free</b>, interpolable and smoothly composable: the main channel for 3D/rendering. The Euler table on the right is <b>computed here from the quaternion</b> (same formula as firmware <code>quat_to_euler</code>, including the ±90° clamp) — read it against the exp3 page's 0x03 frames: one attitude, two representation channels.",
    "exp4.logTitle": "Firmware log (TEXT frames)",
    "exp4.g6quat": "Four components (w,x,y,z · raw frame values)",
    "exp4.g6norm": "Norm (|q|, sender keeps ≈1)",
    "exp4.stat": "QUAT {q} · IMU {imu} · |q| {norm}",
    "exp4.webglOff": "WebGL unavailable — 3D view disabled (numeric/Euler panels still work; verify 3D rotation in a GPU window)",
  },
});
UI.init({page: "exp4"});   // 只渲染壳，不启动数据源

const viewEl = document.getElementById("view3d");
const valsEl = document.getElementById("vals");
const eulerEl = document.getElementById("euler");
const logEl  = document.getElementById("log");
const statEl = document.getElementById("stat");
const statusEl = document.getElementById("status");

if (!window.THREE) {
  statusEl.textContent = UI.t("common.threeMissing");
  throw new Error("three.js missing");
}

/* 视图参数（纯显示层，非算法参数） */
const CUBE = 0.8;             // 立方边长（无量纲，视图比例）
const TRIAD = 0.62;           // 体轴长（体轴随姿态转）
const AXIS_LEN = 0.45;        // 世界轴长（不动，与体轴对照）
const LOCK_DEG = 85;          // 万向锁提示阈值（deg，显示层口径）
const CAM0 = { theta: -0.7, phi: 0.45, dist: 2.4 };   // 初始/复位视角（球坐标）

const COL = { xpos: 0xe57373, xneg: 0x7f3232, ypos: 0x81c784, yneg: 0x395c3f,
              zpos: 0x64b5f6, zneg: 0x32557f, x: 0xe57373, y: 0x81c784, z: 0x64b5f6,
              edge: 0xd8e2ea, grid1: 0x2c3947, grid2: 0x1c2630 };

/* —— 场景骨架（世界 Z 上，与 exp5 同口径）——
   WebGL 降级守卫：无 GPU/offscreen 环境创建 WebGLRenderer 抛错——3D 视图
   关闭（渲染泵跳过），四元数/欧拉数值面板照常跑（数据链路不依赖渲染面）。 */
let renderer = null;
try {
  renderer = new THREE.WebGLRenderer({ canvas: viewEl, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio || 1);
} catch (e) {
  document.getElementById("hint").textContent = UI.t("exp4.webglOff");
}
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, 2, 0.01, 50);
camera.up.set(0, 0, 1);

function resize() {
  const w = Math.max(2, Math.floor(viewEl.clientWidth));
  const h = Math.max(2, Math.floor(viewEl.clientHeight));
  if (renderer) renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);

const grid = new THREE.GridHelper(2.0, 8, COL.grid1, COL.grid2);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

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
  sp.scale.set(0.09, 0.09, 1);
  sp.position.set(x, y, z);
  scene.add(sp);
}
axisLabel("X", "#e57373", AXIS_LEN + 0.08, 0, 0);
axisLabel("Y", "#81c784", 0, AXIS_LEN + 0.08, 0);
axisLabel("Z", "#64b5f6", 0, 0, AXIS_LEN + 0.08);

/* —— 机体立方：面色区分 ± 轴 + 棱线 + 体轴三线（Group 整体随四元数转）—— */
const cube = new THREE.Group();
const faceMats = [COL.xpos, COL.xneg, COL.ypos, COL.yneg, COL.zpos, COL.zneg]
  .map((c) => new THREE.MeshBasicMaterial({ color: c, transparent: true,
                                            opacity: 0.35 }));
cube.add(new THREE.Mesh(new THREE.BoxGeometry(CUBE, CUBE, CUBE), faceMats));
cube.add(new THREE.LineSegments(
  new THREE.EdgesGeometry(new THREE.BoxGeometry(CUBE, CUBE, CUBE)),
  new THREE.LineBasicMaterial({ color: COL.edge })));
[[1, 0, 0, COL.x], [0, 1, 0, COL.y], [0, 0, 1, COL.z]].forEach(([dx, dy, dz, col]) => {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(
    new Float32Array([0, 0, 0, dx * TRIAD, dy * TRIAD, dz * TRIAD]), 3));
  cube.add(new THREE.Line(g, new THREE.LineBasicMaterial({ color: col })));
});
scene.add(cube);

/* —— 自研轨道控制（拖拽旋转 / 滚轮缩放 / 双击复位；零第三方，同 exp5）—— */
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
  cam.dist = Math.min(8.0, Math.max(0.5, cam.dist * (e.deltaY > 0 ? 1.12 : 0.9)));
  applyCam();
}, { passive: false });
viewEl.addEventListener("dblclick", () => {
  cam.theta = CAM0.theta; cam.phi = CAM0.phi; cam.dist = CAM0.dist;
  applyCam();
});

/* —— 数值换算（headless 可测契约，显式挂 window）—— */
/* 四元数 (w,x,y,z) → ZYX 欧拉 (roll,pitch,yaw)（rad）。
   与固件 CoreLib quat.c quat_to_euler 同式：pitch 的 |sinp|>=1
   万向锁区钉 ±90°（copysignf 同义钉法）。 */
function quatToEulerZyx(w, x, y, z) {
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw];
}
function quatNorm(w, x, y, z) {
  return Math.sqrt(w * w + x * x + y * y + z * z);
}
window.quatToEulerZyx = quatToEulerZyx;   // harness 数值契约（镜像 quat.c）
window.quatNorm = quatNorm;

/* —— 数值面板 —— */
let lastQuat = null;
function renderVals() {
  const f = (v) => (v === null ? "—" : v.toFixed(4));
  const q = lastQuat;
  let html = `<div class="g6">${UI.t("exp4.g6quat")}</div>`;
  html += ["w", "x", "y", "z"].map((n, i) =>
    `<div class="v">q${n} <b>${f(q ? q[i] : null)}</b></div>`).join("");
  html += `<div class="g6">${UI.t("exp4.g6norm")}</div>`;
  html += `<div class="v">|q| <b>${q ? quatNorm(q[0], q[1], q[2], q[3]).toFixed(4)
                                     : "—"}</b></div>`;
  valsEl.innerHTML = html;
}

function renderEuler() {
  const q = lastQuat;
  const deg = (v) => (q === null ? "—" : (v * 180 / Math.PI).toFixed(2) + " \u00B0");
  let r = 0, p = 0, y = 0;
  if (q !== null) {
    [r, p, y] = quatToEulerZyx(q[0], q[1], q[2], q[3]);
  }
  document.getElementById("e-roll").textContent = deg(r);
  document.getElementById("e-pitch").textContent = deg(p);
  document.getElementById("e-yaw").textContent = deg(y);
  const locked = q !== null && Math.abs(p * 180 / Math.PI) >= LOCK_DEG;
  document.getElementById("er-pitch").classList.toggle("lock", locked);
  document.getElementById("locknote").classList.toggle("show", locked);
}

/* —— 渲染循环（25fps 上限，同 exp5 节奏；WebGL 降级时只泵帧不渲染）—— */
let lastDraw = 0;
function tick(t) {
  if (t - lastDraw > 40) {
    if (renderer) renderer.render(scene, camera);
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

let bridge = null, nQuat = 0, nImu = 0, lastNorm = null;
let lastStat = { q: 0, imu: 0, norm: "—" };

/* 等 QWebChannel 桥就绪（既有 kick 契约：200ms 轮询） */
function waitBridge(fn) {
  const wait = setInterval(() => {
    if (bridge) { clearInterval(wait); fn(); }
  }, 200);
}

function autoConnect(port) {
  UI.setMode("real", { silent: true });      // 旧 kick 契约：显式选中真实模式
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

/* 数据源桥：服务器版（web/assets/live.js）——SSE 收帧 + POST 下命令，
   API 与旧 QWebChannel bridge 同形（frameReady.connect / openSim / openPort ...）。 */
bridge = LiveBridge();          // 页面加载即连 /api/stream（只连流，不启动任何数据源）
{
  /* 串口列表：服务器 /api/ports（pyserial 可选，缺失时为空列表） */
  Promise.resolve(bridge.listPorts()).then((ports) => {
    (Array.isArray(ports) ? ports : Array.from(ports || [])).forEach((p) => {
      const o = document.createElement("option");
      o.value = o.textContent = p;
      document.getElementById("port").appendChild(o);
    });
  });
  bridge.frameReady.connect((type, seq, vals) => {
    if (type === 4) {                         // QUAT 帧：立方姿态 + 读数
      nQuat++;
      lastQuat = vals.slice(0, 4);
      lastNorm = quatNorm(lastQuat[0], lastQuat[1], lastQuat[2], lastQuat[3]);
      /* THREE.Quaternion.set(x,y,z,w)：参数序换位（帧 w 在前）——与 exp5
         姿态标记同款路径；normalize 仅显示层防漂（帧值已 ≈1） */
      cube.quaternion.set(vals[1], vals[2], vals[3], vals[0]).normalize();
      renderVals();
      renderEuler();
    } else if (type === 1) {                  // IMU_SAMPLE：仅 --sim 双流有（计数）
      nImu++;
    }
  });
  bridge.textReady.connect((m) => log(m));
  bridge.statReady.connect((imu, text, drop) => {
    lastStat = { q: nQuat, imu: nImu,
                 norm: lastNorm === null ? "—" : lastNorm.toFixed(4) };
    statEl.textContent = UI.t("exp4.stat", lastStat);
  });
  bridge.portChanged.connect((on) => {
    UI.setConnected(on);                      // 状态/按钮文案归壳（中英一致）
    if (!on) { lastQuat = null; lastNorm = null; renderVals(); renderEuler(); }
  });
}

/* 语言切换：动态面板（四元数/页脚）重新渲染 */
UI.onLangChange(() => { renderVals(); statEl.textContent = UI.t("exp4.stat", lastStat); });

/* 模式切换：先断开当前流（避免仿真与真实叠加），再由用户点按钮启源 */
UI.onModeChange(() => { if (UI.connected) bridge.closePort(); });

/* 按钮 = 显式动作分发：仿真→合成源，真实→串口；不自动、不回落 */
document.getElementById("btn").onclick = () => {
  if (!bridge) return;
  if (UI.connected) { bridge.closePort(); return; }
  if (UI.mode === "sim") { bridge.openSim(); return; }
  bridge.openPort(document.getElementById("port").value || "COM3", 115200);
};

resize();
applyCam();
renderVals();
renderEuler();
requestAnimationFrame(tick);
