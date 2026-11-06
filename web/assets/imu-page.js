/* imu-page.js — IMU 实验统一页面核心（用户定案 2026-11-06：五实验「逐级继承」）。
 *
 * 继承链：exp1 裸波形 ⊂ exp2（+零点校准）⊂ exp3（+欧拉，独立页保留）⊂
 *         exp4（+四元数姿态）⊂ exp5（+位移轨迹）。
 * 服务器页只剩四份薄壳（examples/imu/exp1_waveform、exp2_bias、exp4_quat_view、exp5_trajectory 的 index.html），壳里只声明：
 *     window.IMU_PAGE = {id:"exp4", panels:["raw","cal","euler","quat"]};
 * 本文件按 panels 渲染面板（渲染顺序固定 = 继承顺序），面板集合由薄壳声明、
 * 核心校验（未知面板名直接报错，不静默跳过）；覆盖关系由
 * tests/imu_page_harness.js 逐壳断言。
 *
 * 五个面板（各自的引擎都是原页面实现的搬入，不是重写）：
 *   raw   0x01 六轴波形：抖动缓冲 60ms + 100Hz 匀速消费 + 亚像素相位（exp1 页引擎，
 *         headless 契约 window.__xrkDebug.pump/__xrkWave 保持）
 *   cal   0x02 raw|deb|bias|temp + 300 帧统计窗 + Δ 行 + **三个命令控件**
 *         （读取当前值 / 一键调零 / 温漂补偿）：CMD 0x7D → ACK 0x7E 命令通道，
 *         算法全在 MCU，页面只发命令/读回执（不减偏、不拟合）；TEXT 里 OK calib 行高亮
 *   euler 0x03 三轴条形（exp3 页画法：中心零点、pitch ±90° 锁边界虚线）
 *   quat  0x04 3D 立方 + 四元数读数 + 从四元数现算的欧拉对照（THREE 本地副本）
 *   traj  SSE pos 渐隐轨迹（exp5 页引擎：顶点缓冲 + 逐顶点 alpha + 自研轨道控制）
 *
 * 页壳纪律（沿用 ui-shell/live.js 契约）：加载只渲染，**不启动任何数据源**；
 * 模式/语言走 UI（xrk.mode/xrk.lang，缺省 real/zh）；启源只在用户点按钮时发生。
 * 文案全部走 ui-shell 词典（本文件只加 imu.* 前缀键，zh/en 成对）。
 */
(function (root) {
  "use strict";

  var UI = root.UI;
  var PAGE = root.IMU_PAGE || null;
  var PAGE_ID = PAGE && PAGE.id;
  var ALL_PANELS = ["raw", "cal", "euler", "quat", "traj"];   // 继承顺序 = 渲染顺序

  /* ── 词典（imu.* 前缀；zh/en 键必须成对，harness 查）──
     页脚模板按页 id 选（每个实验强调的读数不同：exp1 看帧数、exp4 看 QUAT…）。 */
  var DICT = {
    zh: {
      "imu.title.exp1": "exp1 · IMU 裸数据直出（0x01 · 六轴波形）",
      "imu.title.exp2": "exp2 · 裸数据 + 零点校准（0x02 · raw|deb|bias|temp）",
      "imu.title.exp4": "exp4 · 裸数据 + 校准 + 欧拉 + 四元数姿态（逐级继承）",
      "imu.title.exp5": "exp5 · 裸数据 + 校准 + 欧拉 + 四元数 + 位移轨迹（逐级继承）",
      "imu.panel.raw": "原始波形（0x01 IMU_SAMPLE · 加速度 g / 角速度 dps）",
      "imu.panel.cal": "零点校准（0x02 IMU_CAL · raw|deb|bias|temp）",
      "imu.panel.euler": "欧拉角（0x03 EULER · ZYX 弧度 → deg · pitch ±90° 锁边界）",
      "imu.panel.quat": "四元数姿态（0x04 QUAT · w 在前 · 3D 立方）",
      "imu.panel.traj": "位移轨迹（SSE pos · 世界系 Z 上 · 1 格 = 0.1 m）",
      "imu.panel.log": "固件日志（TEXT 帧 · OK calib 行高亮）",
      "imu.raw.vals": "实时数值（帧原值）",
      "imu.target.ph": "tcp:127.0.0.1:5558 或 serial:COM5@115200",
      "imu.target.tip": "真实模式目标串：tcp:<host>:<port>（QEMU UART0 串口桥）或 serial:<COMx>@<baud>（物理串口）；留空则用左例串口名拼 serial:<COMx>@115200",
      "imu.cal.thDelta": "Δ = raw − deb（六轴）",
      "imu.cmd.read": "读取当前值",
      "imu.cmd.readTip": "下发 GET_BIAS(0x02) + GET_STATUS(0x0A) + GET_DRIFT(0x0D)：显示零偏 6 值 / bias_valid / tc_valid / 温漂系数（CMD 0x7D → ACK 0x7E）",
      "imu.cmd.readPending": "读取中…（等 MCU 回执）",
      "imu.cmd.readDone": "已读取：bias_valid={bv} tc_valid={tv}",
      "imu.cmd.cal": "一键调零",
      "imu.cmd.calTip": "下发 CALIBRATE(0x01)：MCU 采 500 样本后回 ACK（异步，约 5s），完成后显示新零偏",
      "imu.cmd.busy": "校准中…",
      "imu.cmd.pending": "已下发 CALIBRATE，MCU 采集中…（标定窗 {n} 样本）",
      "imu.cmd.done": "校准完成：零偏 ax={ax} ay={ay} az={az} mg · gx={gx} gy={gy} gz={gz} dps",
      "imu.cmd.tc": "温漂补偿",
      "imu.cmd.tcTip": "下发 TEMP_CALIB(0x03, 秒数)，再置 TEMPCOMP_EN 位并读回 tc_valid 与 k 系数",
      "imu.cmd.tcPending": "已下发 TEMP_CALIB（{sec}s），采集中…",
      "imu.cmd.tcDone": "温漂标定完成：tc_valid={valid} enabled={enabled}",
      "imu.cmd.sec": "采集秒数",
      "imu.cmd.progress": "标定进度 {done}/{total}（{pct}%）",
      "imu.cmd.progressEst": "标定中… 已 {sec}s（固件未上报进度，估算 {pct}%）",
      "imu.cmd.fail": "命令失败：{msg}",
      "imu.cmd.timeout": "未收到 MCU 回执（固件未响应 CMD / 未连接）",
      "imu.cmd.notConnected": "未连接（先在「仿真」或「真实」模式启动数据源，再下发命令）",
      "imu.bias.title": "零偏（GET_BIAS）",
      "imu.bias.none": "尚未读取（点「读取当前值」或「一键调零」）",
      "imu.bias.valid": "bias_valid={v}",
      "imu.bias.hint": "⚠ 未标定（bias_valid=0）：仿真/真机零偏未补偿，轨迹会漂 —— 点「一键调零」让 MCU 标定",
      "imu.status.title": "状态（GET_STATUS）",
      "imu.status.none": "尚未读取",
      "imu.status.line": "flags=0x{flags} bias_valid={bv} tc_valid={tv} TEMPCOMP_EN={en}",
      "imu.drift.title": "温漂系数 k（GET_DRIFT · 每 °C）",
      "imu.drift.none": "尚未读取",
      "imu.tc.title": "温补状态（GET_TC_STATE）",
      "imu.tc.none": "尚未读取",
      "imu.tc.line": "tc_valid={valid} enabled={enabled} k=[{k}]",
      "imu.cal.thAxis": "轴",
      "imu.cal.thRaw": "原始 raw",
      "imu.cal.thDeb": "补偿后 deb",
      "imu.cal.thBias": "当前零偏 bias",
      "imu.cal.thRef": "参考（静止 Z_UP）",
      "imu.cal.temp": "芯片温度 <b>{now}</b>（窗口 {range}）",
      "imu.cal.statTitle": "零偏统计（窗口均值 ± σ · 窗 {n}/{N} 帧）",
      "imu.cal.thRawStat": "raw 均值 ± σ",
      "imu.cal.thDebStat": "deb 均值 ± σ",
      "imu.cal.wait": "尚未收到 0x02 帧：先在「仿真」或「真实」模式下点启动/连接（真实档点「调零」下发 CAL）",
      "imu.cal.thAxisShort": "轴",
      "imu.euler.thAxis": "轴",
      "imu.euler.thDeg": "角度 (deg)",
      "imu.euler.thRad": "弧度 (rad)",
      "imu.euler.thPeak": "峰值 |max| (deg)",
      "imu.euler.thRange": "显示域",
      "imu.euler.roll": "roll 绕 X",
      "imu.euler.pitch": "pitch 绕 Y",
      "imu.euler.yaw": "yaw 绕 Z",
      "imu.euler.rangeRP": "±180°",
      "imu.euler.rangePitch": "±90°（红虚线 = 万向锁边界）",
      "imu.quat.vals": "四元数（帧原值，w 在前）",
      "imu.quat.eulerTitle": "欧拉对照（ZYX · 本页从四元数现算，同固件 quat_to_euler 式）",
      "imu.quat.thAxis": "轴",
      "imu.quat.thDeg": "角度 (deg)",
      "imu.quat.thRange": "显示域",
      "imu.quat.locknote": "⚠ pitch 逼近 ±90°：万向锁区（roll/yaw 不再独立）",
      "imu.quat.norm": "范数 |q|",
      "imu.traj.values": "位置 / 姿态 / 比力",
      "imu.traj.clear": "清除轨迹",
      "imu.traj.hint": "拖拽旋转 · 滚轮缩放 · 双击复位视角 · 亮端 = 当前位置，尾部渐隐 = 历史",
      "imu.hint.cube": "拖拽旋转 · 滚轮缩放 · 双击复位 · 立方面色/体轴（X红 Y绿 Z蓝）随姿态转",
      "imu.msg.calibSimulated": "仿真档调零：已用预设注入零偏合成 {cmd} 的标定回执（3 帧 0x02 + TEXT），无板也能看校准面板",
      "imu.msg.calibSent": "已向固件下发 {cmd}（明文行）；标定完成后固件回 3 帧 0x02 + TEXT",
      "imu.msg.notConnected": "未连接（先在「仿真」或「真实」模式下点启动/连接，再调零）",
      "imu.msg.webglOff": "WebGL 不可用——3D 视图已关闭（数值/条形面板照常；3D 旋转需 GPU 开窗验证）",
      "imu.stat.exp1": "帧 {imu} · 丢帧 {drop}",
      "imu.stat.exp2": "IMU_CAL {cal} · 窗口 {win}/{N} · 温度 {temp}",
      "imu.stat.exp4": "QUAT {q} · IMU {imu} · |q| {norm}",
      "imu.stat.exp5": "QUAT {q} · IMU {imu} · 位置 ({x}, {y}, {z}) m",
    },
    en: {
      "imu.title.exp1": "exp1 · Raw IMU stream (0x01 · six-axis waveform)",
      "imu.title.exp2": "exp2 · Raw stream + bias calibration (0x02 · raw|deb|bias|temp)",
      "imu.title.exp4": "exp4 · Quaternion attitude on top of raw + calibration + Euler (progressive)",
      "imu.title.exp5": "exp5 · Raw + calibration + Euler + quaternion + trajectory (progressive)",
      "imu.panel.raw": "Raw waveform (0x01 IMU_SAMPLE · accel g / gyro dps)",
      "imu.panel.cal": "Bias calibration (0x02 IMU_CAL · raw|deb|bias|temp)",
      "imu.panel.euler": "Euler angles (0x03 EULER · ZYX radians → deg · pitch ±90° limit)",
      "imu.panel.quat": "Quaternion attitude (0x04 QUAT · w first · 3D cube)",
      "imu.panel.traj": "Displacement trajectory (SSE pos · world Z up · 1 cell = 0.1 m)",
      "imu.panel.log": "Firmware log (TEXT frames · OK calib lines highlighted)",
      "imu.raw.vals": "Live values (raw frame values)",
      "imu.target.ph": "tcp:127.0.0.1:5558 or serial:COM5@115200",
      "imu.target.tip": "Real-mode target: tcp:<host>:<port> (QEMU UART0 serial bridge) or serial:<COMx>@<baud> (physical port); leave empty to build serial:<COMx>@115200 from the port above",
      "imu.cal.thDelta": "Δ = raw − deb (six axes)",
      "imu.cmd.read": "Read values",
      "imu.cmd.readTip": "Send GET_BIAS(0x02) + GET_STATUS(0x0A) + GET_DRIFT(0x0D): show 6-axis bias / bias_valid / tc_valid / drift coefficients (CMD 0x7D → ACK 0x7E)",
      "imu.cmd.readPending": "Reading… (waiting for the MCU ack)",
      "imu.cmd.readDone": "Read: bias_valid={bv} tc_valid={tv}",
      "imu.cmd.cal": "Calibrate",
      "imu.cmd.calTip": "Send CALIBRATE(0x01): the MCU replies after 500 samples (asynchronous, ≈5 s); the new bias is shown on completion",
      "imu.cmd.busy": "Calibrating…",
      "imu.cmd.pending": "CALIBRATE sent — MCU sampling… (window {n} samples)",
      "imu.cmd.done": "Calibration done: bias ax={ax} ay={ay} az={az} mg · gx={gx} gy={gy} gz={gz} dps",
      "imu.cmd.tc": "Temp-comp calib",
      "imu.cmd.tcTip": "Send TEMP_CALIB(0x03, seconds), then set the TEMPCOMP_EN bit and read back tc_valid + k coefficients",
      "imu.cmd.tcPending": "TEMP_CALIB sent ({sec}s) — sampling…",
      "imu.cmd.tcDone": "Temp-comp calib done: tc_valid={valid} enabled={enabled}",
      "imu.cmd.sec": "Seconds",
      "imu.cmd.progress": "Calibration progress {done}/{total} ({pct}%)",
      "imu.cmd.progressEst": "Calibrating… {sec}s elapsed (firmware reports no progress; ≈{pct}%)",
      "imu.cmd.fail": "Command failed: {msg}",
      "imu.cmd.timeout": "No MCU ack (firmware did not answer CMD / not connected)",
      "imu.cmd.notConnected": "Not connected (start Simulation or Real first, then send commands)",
      "imu.bias.title": "Bias (GET_BIAS)",
      "imu.bias.none": "Not read yet (press Read values or Calibrate)",
      "imu.bias.valid": "bias_valid={v}",
      "imu.bias.hint": "⚠ Not calibrated (bias_valid=0): bias is not compensated yet, trajectory will drift — press One-tap zero and let the MCU calibrate",
      "imu.status.title": "Status (GET_STATUS)",
      "imu.status.none": "Not read yet",
      "imu.status.line": "flags=0x{flags} bias_valid={bv} tc_valid={tv} TEMPCOMP_EN={en}",
      "imu.drift.title": "Drift coefficients k (GET_DRIFT · per °C)",
      "imu.drift.none": "Not read yet",
      "imu.tc.title": "Temp-comp state (GET_TC_STATE)",
      "imu.tc.none": "Not read yet",
      "imu.tc.line": "tc_valid={valid} enabled={enabled} k=[{k}]",
      "imu.cal.thAxis": "Axis",
      "imu.cal.thRaw": "raw",
      "imu.cal.thDeb": "deb",
      "imu.cal.thBias": "bias now",
      "imu.cal.thRef": "Reference (static Z_UP)",
      "imu.cal.temp": "Chip temp <b>{now}</b> (window {range})",
      "imu.cal.statTitle": "Bias stats (window mean ± σ · {n}/{N} frames)",
      "imu.cal.thRawStat": "raw mean ± σ",
      "imu.cal.thDebStat": "deb mean ± σ",
      "imu.cal.wait": "No 0x02 frame yet: pick Simulation or Real and press start/connect (in real mode Calibrate sends CAL)",
      "imu.cal.thAxisShort": "Axis",
      "imu.euler.thAxis": "Axis",
      "imu.euler.thDeg": "Angle (deg)",
      "imu.euler.thRad": "Radians (rad)",
      "imu.euler.thPeak": "Peak |max| (deg)",
      "imu.euler.thRange": "Display range",
      "imu.euler.roll": "roll about X",
      "imu.euler.pitch": "pitch about Y",
      "imu.euler.yaw": "yaw about Z",
      "imu.euler.rangeRP": "±180°",
      "imu.euler.rangePitch": "±90° (red dashed = gimbal-lock limit)",
      "imu.quat.vals": "Quaternion (raw frame values, w first)",
      "imu.quat.eulerTitle": "Euler reference (ZYX · computed here from the quaternion, same formula as firmware quat_to_euler)",
      "imu.quat.thAxis": "Axis",
      "imu.quat.thDeg": "Angle (deg)",
      "imu.quat.thRange": "Display range",
      "imu.quat.locknote": "⚠ pitch near ±90°: gimbal-lock region (roll/yaw no longer independent)",
      "imu.quat.norm": "Norm |q|",
      "imu.traj.values": "Position / attitude / specific force",
      "imu.traj.clear": "Clear trail",
      "imu.traj.hint": "Drag to orbit · wheel to zoom · double-click to reset view · bright end = current position, fading tail = history",
      "imu.hint.cube": "Drag to orbit · wheel to zoom · double-click to reset · face colours / body axes (X red, Y green, Z blue) follow the attitude",
      "imu.msg.calibSimulated": "Simulation calibration: synthesised the {cmd} acknowledgement from the preset injected bias (3× 0x02 frames + TEXT) — the calibration panel works without a board",
      "imu.msg.calibSent": "Sent {cmd} to firmware (plain line); on completion the firmware replies with 3× 0x02 frames + TEXT",
      "imu.msg.notConnected": "Not connected (pick Simulation or Real, press start/connect, then calibrate)",
      "imu.msg.webglOff": "WebGL unavailable — 3D view disabled (numeric/bar panels still work; verify 3D rotation in a GPU window)",
      "imu.stat.exp1": "{imu} frames · {drop} dropped",
      "imu.stat.exp2": "IMU_CAL {cal} · window {win}/{N} · temp {temp}",
      "imu.stat.exp4": "QUAT {q} · IMU {imu} · |q| {norm}",
      "imu.stat.exp5": "QUAT {q} · IMU {imu} · pos ({x}, {y}, {z}) m",
    },
  };

  /* 页脚模板按页 id 选（缺登记即配置错——显式报错，不静默回退到别的页口径） */
  var STAT_KEYS = {
    exp1: "imu.stat.exp1",
    exp2: "imu.stat.exp2",
    exp4: "imu.stat.exp4",
    exp5: "imu.stat.exp5",
  };

  /* ── 显示层参数（非算法参数）── */
  var FRAME_HZ = 100;          // 协议帧率（固件 IMU_SAMPLE 100Hz）
  var JITTER_BUF_S = 0.06;     // 波形抖动缓冲深度（s）
  var WIN_N = 300;             // 零偏统计窗（100Hz 下 3 s）
  var AXES6 = ["ax", "ay", "az", "gx", "gy", "gz"];     // 采样序（与 0x02 块一致）
  var AXES3 = ["roll", "pitch", "yaw"];                 // 0x03 分量序
  var RANGES = { roll: 180, pitch: 90, yaw: 180 };      // 条形显示域（deg）
  var COLORS = { roll: "#4fc3f7", pitch: "#ffb74d", yaw: "#81c784" };
  var LOCK_DEG = 85;           // 万向锁提示阈值（deg，显示层口径）
  var CH_COLORS = [0xe57373, 0x81c784, 0x64b5f6];       // 波形 x/y/z 三色
  var CUBE = 0.8, TRIAD = 0.62, AXIS_LEN = 0.45;        // 立方视图比例（无量纲）
  var CUBE_CAM0 = { theta: -0.7, phi: 0.45, dist: 2.4 };
  var TRAIL_MAX = 6000, TRAIL_HEAD = 150;               // 轨迹顶点容量 / 不透明段
  var GRID_M = 1.0, GRID_DIV = 10, TRAIL_AXIS = 0.28, TRAIL_TRIAD = 0.09;
  var TRAIL_CAM0 = { theta: -0.7, phi: 0.45, dist: 1.5 };
  var COL3D = { xpos: 0xe57373, xneg: 0x7f3232, ypos: 0x81c784, yneg: 0x395c3f,
                zpos: 0x64b5f6, zneg: 0x32557f, x: 0xe57373, y: 0x81c784, z: 0x64b5f6,
                edge: 0xd8e2ea, acc: 0x4fc3f7, grid1: 0x2c3947, grid2: 0x1c2630 };

  function FMT(v) { return v === null || v === undefined ? "—" : v.toFixed(4); }
  function signed(v, dig) { return (v >= 0 ? "+" : "") + v.toFixed(dig); }
  function deg(r) { return r * 180 / Math.PI; }
  /* 窗口级监听：浏览器里 root === window；node harness 的 root 无 addEventListener。 */
  function onRoot(ev, fn) {
    if (typeof root.addEventListener === "function") root.addEventListener(ev, fn);
  }

  /* ══════════════════════════ 页壳：配置校验 + 布局 ══════════════════════════ */

  var doc = root.document;
  var ENABLED = {};

  function byId(id) { return doc.getElementById(id); }

  function readConfig() {
    if (!PAGE_ID) throw new Error("imu-page: 薄壳缺 window.IMU_PAGE={id,panels}");
    if (!STAT_KEYS[PAGE_ID]) throw new Error("imu-page: 未登记的页 id " + PAGE_ID);
    var list = (PAGE && PAGE.panels) || [];
    if (!list.length) throw new Error("imu-page: 页面 " + PAGE_ID + " 的 panels 为空");
    list.forEach(function (name) {
      if (ALL_PANELS.indexOf(name) < 0) {
        throw new Error("imu-page: 未知面板 " + name + "（合法：" + ALL_PANELS.join("/") + "）");
      }
      ENABLED[name] = true;
    });
    return ALL_PANELS.filter(function (name) { return ENABLED[name]; });
  }

  var PANELS_ON = readConfig();       // 渲染顺序 = 继承顺序（与壳里数组顺序无关）

  var STYLE = [
    ":root{--bg:#101418;--panel:#1a2129;--line:#2c3947;--fg:#d8e2ea;--dim:#7c8b98;",
    "--acc:#4fc3f7;--ok:#66bb6a;--warn:#ffb74d;--cal:#81c784;--lock:#e57373}",
    "*{box-sizing:border-box;margin:0;padding:0}",
    "body{background:var(--bg);color:var(--fg);font:13px/1.5 'Cascadia Mono',Consolas,monospace;padding:14px}",
    "header{display:flex;gap:10px;align-items:center;margin-bottom:12px;flex-wrap:wrap}",
    "h1{font-size:15px;color:var(--acc);font-weight:600}",
    "select,button,input{background:var(--panel);color:var(--fg);border:1px solid var(--line);",
    "border-radius:4px;padding:5px 12px;font:inherit}",
    "select,button{cursor:pointer}",
    "input{min-width:240px;cursor:text}",
    "#target{color:var(--dim)}",
    "button:hover{border-color:var(--acc)}",
    "#btn-cal{color:var(--cal);border-color:#395c3f}#btn-cal:hover{border-color:var(--cal)}",
    "#btn-cal[disabled]{opacity:.55;cursor:progress}",
    "#cal-banner{font-size:12px;margin-bottom:8px;min-height:1.4em}",
    "#cal-banner.pending{color:var(--warn)}",
    "#cal-banner.done{color:var(--cal)}",
    "#cal-banner.fail{color:var(--lock)}",
    "#cal-delta{font-size:12px;color:var(--dim);margin-top:8px}",
    ".cmdbar{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:2px 0 8px}",
    ".cmdbar input[type=number]{width:80px;min-width:0}",
    ".cmdbar label{color:var(--dim);font-size:12px}",
    "#cal-progress{font-size:12px;color:var(--dim);min-height:1.2em}",
    ".cmdout{font-size:12px;color:var(--dim);margin-top:6px;line-height:1.6}",
    ".cmdout b{color:var(--acc);font-weight:normal}",
    "#status{color:var(--dim)}",
    ".grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}",
    ".panel{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:10px;position:relative}",
    ".panel h2,.panel h3{font-size:12px;color:var(--dim);font-weight:normal;margin-bottom:8px;letter-spacing:.08em}",
    ".span2{grid-column:1/-1}",
    "canvas{width:100%;background:#0c1014;border-radius:4px;display:block}",
    "#cAcc,#cGyro{height:170px}",
    "#bars{height:230px}",
    "#view3d{height:380px;cursor:grab}#view3d:active{cursor:grabbing}",
    "#trail3d{height:420px;cursor:grab}#trail3d:active{cursor:grabbing}",
    ".wavecell{position:relative}",
    ".rng{position:absolute;top:6px;left:10px;color:var(--dim);font-size:11px;pointer-events:none}",
    ".hint{color:var(--dim);font-size:11px;margin-top:6px}",
    ".vals{display:grid;grid-template-columns:repeat(3,1fr);gap:6px 14px;font-size:14px;margin-top:8px}",
    ".vals .g6{grid-column:1/-1;color:var(--dim);font-size:11px;margin-top:2px;letter-spacing:.08em}",
    ".v b{color:var(--acc);font-weight:normal}",
    "table{border-collapse:collapse;width:100%;font-size:12px}",
    "th,td{border:1px solid var(--line);padding:4px 7px;text-align:right;white-space:nowrap}",
    "th{color:var(--dim);font-weight:normal;text-align:left}",
    "td.ax{text-align:left}td.raw{color:var(--warn)}td.deb{color:var(--cal)}",
    "td.bias{color:var(--acc)}td.ref{color:var(--dim)}",
    "#temp{font-size:13px;margin-top:8px}#temp b{color:var(--acc);font-weight:normal}",
    "#wait{color:var(--warn);font-size:12px;margin-top:6px}",
    "#euler tr.lock td{color:var(--lock);border-color:var(--lock)}",
    "#locknote{color:var(--lock);font-size:11px;margin-top:6px;display:none}#locknote.show{display:block}",
    "#log{height:150px;overflow-y:auto;font-size:12px;color:var(--dim);white-space:pre-wrap}",
    "#log .fw{color:var(--warn)}#log .calok{color:var(--cal)}",
    "footer{margin-top:10px;color:var(--dim);font-size:12px}",
  ].join("");

  function markup() {
    var h = [];
    h.push('<header>');
    h.push('<h1 id="title"></h1>');
    h.push('<span id="modebar"></span>');
    h.push('<select id="port"></select>');
    h.push('<input id="target" type="text" data-i18n-ph="imu.target.ph"' +
           ' data-i18n-title="imu.target.tip">');
    h.push('<button id="btn"></button>');
    if (ENABLED.cal) h.push('<button id="btn-cal"></button>');
    if (ENABLED.traj) h.push('<button id="btn-clear"></button>');
    h.push('<button id="btn-lang"></button>');
    h.push('<span id="status"></span>');
    h.push('</header>');
    h.push('<div class="grid">');
    if (ENABLED.raw) {
      h.push('<div class="panel span2" data-panel="raw">');
      h.push('<h2 id="cap-raw" data-i18n="imu.panel.raw"></h2>');
      h.push('<div class="grid">');
      h.push('<div class="wavecell"><canvas id="cAcc" width="520" height="170"></canvas>');
      h.push('<span class="rng" id="rng-acc"></span></div>');
      h.push('<div class="wavecell"><canvas id="cGyro" width="520" height="170"></canvas>');
      h.push('<span class="rng" id="rng-gyr"></span></div>');
      h.push('</div>');
      h.push('<div id="vals" class="vals"></div>');
      h.push('</div>');
    }
    if (ENABLED.cal) {
      h.push('<div class="panel" data-panel="cal">');
      h.push('<h2 id="cap-cal" data-i18n="imu.panel.cal"></h2>');
      h.push('<div id="cal-banner"></div>');
      h.push('<div id="cal-progress"></div>');
      h.push('<div class="cmdbar">');
      h.push('<button id="btn-imu-read"></button>');
      h.push('<button id="btn-cal"></button>');
      h.push('<button id="btn-tc"></button>');
      h.push('<label id="tc-sec-lbl" for="tc-sec"></label>');
      h.push('<input id="tc-sec" type="number" min="1" max="255" step="1" value="30">');
      h.push('</div>');
      h.push('<div id="imu-bias" class="cmdout"></div>');
      h.push('<div id="imu-status" class="cmdout"></div>');
      h.push('<div id="imu-drift" class="cmdout"></div>');
      h.push('<div id="imu-tc" class="cmdout"></div>');
      h.push('<table id="cal-table"></table>');
      h.push('<div id="cal-delta"></div>');
      h.push('<div id="temp"></div>');
      h.push('<div id="wait"></div>');
      h.push('<h3 id="stat-title"></h3>');
      h.push('<table id="cal-stats-table"></table>');
      h.push('</div>');
    }
    if (ENABLED.euler) {
      h.push('<div class="panel" data-panel="euler">');
      h.push('<h2 id="cap-euler" data-i18n="imu.panel.euler"></h2>');
      h.push('<table id="euler"></table>');
      h.push('<canvas id="bars" width="748" height="230"></canvas>');
      h.push('</div>');
    }
    if (ENABLED.quat) {
      h.push('<div class="panel" data-panel="quat">');
      h.push('<h2 id="cap-quat" data-i18n="imu.panel.quat"></h2>');
      h.push('<canvas id="view3d"></canvas>');
      h.push('<div class="hint" id="cube-hint"></div>');
      h.push('<div id="q-vals" class="vals"></div>');
      h.push('<h3 id="cap-quat-euler"></h3>');
      h.push('<table id="quat-euler"></table>');
      h.push('<div id="locknote"></div>');
      h.push('</div>');
    }
    if (ENABLED.traj) {
      h.push('<div class="panel span2" data-panel="traj">');
      h.push('<h2 id="cap-traj" data-i18n="imu.panel.traj"></h2>');
      h.push('<canvas id="trail3d"></canvas>');
      h.push('<div class="hint" id="traj-hint"></div>');
      h.push('<div id="t-vals" class="vals"></div>');
      h.push('</div>');
    }
    h.push('<div class="panel span2" data-panel="log">');
    h.push('<h2 id="cap-log" data-i18n="imu.panel.log"></h2>');
    h.push('<div id="log"></div>');
    h.push('</div>');
    h.push('</div>');
    h.push('<footer id="stat"></footer>');
    return h.join("");
  }

  function injectStyle() {
    if (!doc.head || typeof doc.createElement !== "function") return;
    if (byId("imu-page-style")) return;
    var st = doc.createElement("style");
    st.id = "imu-page-style";
    st.textContent = STYLE;
    doc.head.appendChild(st);
  }

  /* ══════════════════════════ raw 面板：波形引擎（exp1 搬入） ══════════════════ */

  /* 六通道滚动波形 = 两组（加速度/角速度）× 三色 THREE.Line；顶点缓冲预分配，
     滚动 = 数据环左移 + setDrawRange(N-count, count)；量程自适应 = 窗口峰值 × 1.2。 */
  function makeScope3(canvas, colors, rngEl) {
    var W = Math.max(2, Math.floor(canvas.clientWidth || +canvas.width));
    var H = Math.max(2, Math.floor(canvas.clientHeight || +canvas.height));
    var N = W;                                  // 每像素列一样本
    var renderer = new root.THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setPixelRatio(root.devicePixelRatio || 1);
    renderer.setSize(W, H, false);
    renderer.setClearColor(0x0c1014);
    var scene = new root.THREE.Scene();
    var cam = new root.THREE.OrthographicCamera(0, W, 1, -1, -1, 1);
    var zero = new root.THREE.BufferGeometry();
    zero.setAttribute("position",
      new root.THREE.BufferAttribute(new Float32Array([0, 0, 0, W, 0, 0]), 3));
    scene.add(new root.THREE.Line(zero,
      new root.THREE.LineBasicMaterial({ color: 0x22303c })));

    var chans = colors.map(function (col) {
      var pos = new Float32Array(N * 3);
      var attr = new root.THREE.BufferAttribute(pos, 3);
      attr.setUsage(root.THREE.DynamicDrawUsage);
      var geom = new root.THREE.BufferGeometry();
      geom.setAttribute("position", attr);
      geom.setDrawRange(N, 0);
      scene.add(new root.THREE.Line(geom,
        new root.THREE.LineBasicMaterial({ color: col })));
      return { pos: pos, attr: attr, geom: geom, yr: new Float32Array(N), pend: [] };
    });
    var count = 0;

    return {
      push: function (vals) {
        for (var k = 0; k < chans.length; k++) chans[k].pend.push(vals[k]);
      },
      qlen: function () { return chans[0].pend.length; },
      count: function () { return count; },
      trim: function (cap) {
        var over = chans[0].pend.length - cap;
        if (over <= 0) return 0;
        for (var c = 0; c < chans.length; c++) chans[c].pend.splice(0, over);
        return over;
      },
      clear: function () {
        for (var c = 0; c < chans.length; c++) { chans[c].pend.length = 0; chans[c].yr.fill(0); }
        count = 0;
      },
      consume: function (n) {
        var take = Math.min(n, chans[0].pend.length, N);
        if (take <= 0) return 0;
        var overflow = Math.max(0, count + take - N);
        for (var c = 0; c < chans.length; c++) {
          if (overflow) chans[c].yr.copyWithin(0, overflow);
          var taken = chans[c].pend.splice(0, take);
          for (var j = 0; j < taken.length; j++) chans[c].yr[count - overflow + j] = taken[j];
        }
        count = Math.min(count + take, N);
        return take;
      },
      draw: function (phase) {
        var ph = phase || 0;
        var peak = 1e-6;
        for (var c = 0; c < chans.length; c++) {
          for (var i = 0; i < count; i++) {
            var col = N - count + i, o = col * 3;
            chans[c].pos[o] = col;
            chans[c].pos[o + 1] = chans[c].yr[i];
            var a = Math.abs(chans[c].yr[i]);
            if (a > peak) peak = a;
          }
          chans[c].attr.needsUpdate = true;
          chans[c].geom.setDrawRange(N - count, count);
        }
        peak *= 1.2;
        cam.left = ph; cam.right = W + ph;
        cam.top = peak; cam.bottom = -peak;
        cam.updateProjectionMatrix();
        renderer.render(scene, cam);
        if (rngEl) rngEl.textContent = "±" + peak.toFixed(2);
      },
    };
  }

  /* ══════════════════════════ cal 面板：统计窗（exp2 搬入） ═════════════════════ */

  /* 滑动窗统计：push 喂 19 值帧；mean/std 取第 i 分量窗口均值/样本标准差。 */
  function makeWinStats(winN) {
    var buf = [];
    var api = {
      push: function (vals) { buf.push(vals); if (buf.length > winN) buf.shift(); },
      reset: function () { buf.length = 0; },
      n: function () { return buf.length; },
      mean: function (i) {
        if (!buf.length) return null;
        var s = 0;
        for (var k = 0; k < buf.length; k++) s += buf[k][i];
        return s / buf.length;
      },
      std: function (i) {
        if (buf.length < 2) return null;
        var m = api.mean(i), s = 0;
        for (var k = 0; k < buf.length; k++) s += (buf[k][i] - m) * (buf[k][i] - m);
        return Math.sqrt(s / (buf.length - 1));
      },
      min: function (i) { return buf.length ? Math.min.apply(null, buf.map(function (v) { return v[i]; })) : null; },
      max: function (i) { return buf.length ? Math.max.apply(null, buf.map(function (v) { return v[i]; })) : null; },
    };
    return api;
  }

  /* ══════════════════════════ 四元数 ↔ 欧拉（同固件 quat.c） ══════════════════ */

  /* 四元数 (w,x,y,z) → ZYX 欧拉 (roll,pitch,yaw) rad；pitch 的 |sinp| >= 1 处钉 ±90°。 */
  function quatToEulerZyx(qw, qx, qy, qz) {
    var roll = Math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    var sinp = 2 * (qw * qy - qz * qx);
    var pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
    var yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
    return [roll, pitch, yaw];
  }
  function quatNorm(qw, qx, qy, qz) {
    return Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  }

  /* ══════════════════════════ 3D 视图：立方（exp4） / 轨迹（exp5） ═════════════ */

  /* 自研轨道控制（拖拽旋转 / 滚轮缩放 / 双击复位；零第三方）——两个视图共用。 */
  function makeOrbit(canvas, camera, cam0) {
    var cam = { theta: cam0.theta, phi: cam0.phi, dist: cam0.dist };
    function apply() {
      var cp = Math.cos(cam.phi);
      camera.position.set(cam.dist * cp * Math.cos(cam.theta),
                          cam.dist * cp * Math.sin(cam.theta),
                          cam.dist * Math.sin(cam.phi));
      camera.lookAt(0, 0, 0);
    }
    var dragging = false, lastX = 0, lastY = 0;
    canvas.addEventListener("mousedown", function (e) {
      dragging = true; lastX = e.clientX; lastY = e.clientY;
    });
    onRoot("mousemove", function (e) {
      if (!dragging) return;
      cam.theta -= (e.clientX - lastX) * 0.008;
      cam.phi = Math.min(1.5, Math.max(-1.5, cam.phi + (e.clientY - lastY) * 0.008));
      lastX = e.clientX; lastY = e.clientY;
      apply();
    });
    onRoot("mouseup", function () { dragging = false; });
    canvas.addEventListener("wheel", function (e) {
      if (e.preventDefault) e.preventDefault();
      cam.dist = Math.min(8.0, Math.max(0.3, cam.dist * (e.deltaY > 0 ? 1.12 : 0.9)));
      apply();
    }, { passive: false });
    canvas.addEventListener("dblclick", function () {
      cam.theta = cam0.theta; cam.phi = cam0.phi; cam.dist = cam0.dist;
      apply();
    });
    apply();
    return cam;
  }

  /* 轴名标签（自研 sprite，零外部字体/贴图资源） */
  function addAxisLabels(scene, len, size) {
    [["X", "#e57373", len + 0.08, 0, 0], ["Y", "#81c784", 0, len + 0.08, 0],
     ["Z", "#64b5f6", 0, 0, len + 0.08]].forEach(function (a) {
      var cv = doc.createElement("canvas");
      cv.width = cv.height = 64;
      var c2 = cv.getContext("2d");
      c2.fillStyle = a[1];
      c2.font = "bold 40px sans-serif";
      c2.textAlign = "center";
      c2.textBaseline = "middle";
      c2.fillText(a[0], 32, 32);
      var sp = new root.THREE.Sprite(new root.THREE.SpriteMaterial(
        { map: new root.THREE.CanvasTexture(cv), depthTest: false }));
      sp.scale.set(size, size, 1);
      sp.position.set(a[2], a[3], a[4]);
      scene.add(sp);
    });
  }

  /* 立方视图：机体立方随四元数转，网格/世界轴不动（世界 Z 上）。 */
  function makeCubeView(canvas) {
    var renderer = new root.THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setPixelRatio(root.devicePixelRatio || 1);
    var scene = new root.THREE.Scene();
    var camera = new root.THREE.PerspectiveCamera(50, 2, 0.01, 50);
    camera.up.set(0, 0, 1);

    var grid = new root.THREE.GridHelper(2.0, 8, COL3D.grid1, COL3D.grid2);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    scene.add(new root.THREE.AxesHelper(AXIS_LEN));
    addAxisLabels(scene, AXIS_LEN, 0.09);

    var cube = new root.THREE.Group();
    var faceMats = [COL3D.xpos, COL3D.xneg, COL3D.ypos, COL3D.yneg, COL3D.zpos, COL3D.zneg]
      .map(function (c) {
        return new root.THREE.MeshBasicMaterial({ color: c, transparent: true, opacity: 0.35 });
      });
    cube.add(new root.THREE.Mesh(
      new root.THREE.BoxGeometry(CUBE, CUBE, CUBE), faceMats));
    cube.add(new root.THREE.LineSegments(
      new root.THREE.EdgesGeometry(new root.THREE.BoxGeometry(CUBE, CUBE, CUBE)),
      new root.THREE.LineBasicMaterial({ color: COL3D.edge })));
    [[1, 0, 0, COL3D.x], [0, 1, 0, COL3D.y], [0, 0, 1, COL3D.z]].forEach(function (a) {
      var g = new root.THREE.BufferGeometry();
      g.setAttribute("position", new root.THREE.BufferAttribute(
        new Float32Array([0, 0, 0, a[0] * TRIAD, a[1] * TRIAD, a[2] * TRIAD]), 3));
      cube.add(new root.THREE.Line(g,
        new root.THREE.LineBasicMaterial({ color: a[3] })));
    });
    scene.add(cube);
    var cam = makeOrbit(canvas, camera, CUBE_CAM0);

    return {
      cube: cube,
      render: function () { renderer.render(scene, camera); },
      resize: function () {
        var w = Math.max(2, Math.floor(canvas.clientWidth || +canvas.width));
        var h = Math.max(2, Math.floor(canvas.clientHeight || +canvas.height));
        renderer.setSize(w, h, false);
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
      },
      cam: cam,
    };
  }

  /* 轨迹视图：单条 Line + 逐顶点 alpha（最近 TRAIL_HEAD 点不透明，尾部渐隐）。 */
  function makeTrailView(canvas) {
    var renderer = new root.THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setPixelRatio(root.devicePixelRatio || 1);
    var scene = new root.THREE.Scene();
    var camera = new root.THREE.PerspectiveCamera(50, 2, 0.01, 50);
    camera.up.set(0, 0, 1);

    var grid = new root.THREE.GridHelper(GRID_M, GRID_DIV, COL3D.grid1, COL3D.grid2);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    scene.add(new root.THREE.AxesHelper(TRAIL_AXIS));
    addAxisLabels(scene, TRAIL_AXIS, 0.07);

    var posArr = new Float32Array(TRAIL_MAX * 3);
    var alphaArr = new Float32Array(TRAIL_MAX);
    var trailCount = 0, trailDirty = false;
    var trailGeom = new root.THREE.BufferGeometry();
    var posAttr = new root.THREE.BufferAttribute(posArr, 3);
    posAttr.setUsage(root.THREE.DynamicDrawUsage);
    var alphaAttr = new root.THREE.BufferAttribute(alphaArr, 1);
    alphaAttr.setUsage(root.THREE.DynamicDrawUsage);
    trailGeom.setAttribute("position", posAttr);
    trailGeom.setAttribute("aAlpha", alphaAttr);
    trailGeom.setDrawRange(0, 0);
    var trailMat = new root.THREE.ShaderMaterial({
      transparent: true, depthWrite: false,
      uniforms: { uColor: { value: new root.THREE.Color(COL3D.acc) } },
      vertexShader: [
        "attribute float aAlpha;", "varying float vA;", "void main() {",
        "  vA = aAlpha;",
        "  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);",
        "}"].join("\n"),
      fragmentShader: [
        "uniform vec3 uColor;", "varying float vA;", "void main() {",
        "  gl_FragColor = vec4(uColor, vA);", "}"].join("\n"),
    });
    scene.add(new root.THREE.Line(trailGeom, trailMat));

    var marker = new root.THREE.Group();
    marker.add(new root.THREE.Mesh(
      new root.THREE.SphereGeometry(0.016, 16, 12),
      new root.THREE.MeshBasicMaterial({ color: COL3D.acc })));
    [[1, 0, 0, COL3D.x], [0, 1, 0, COL3D.y], [0, 0, 1, COL3D.z]].forEach(function (a) {
      var g = new root.THREE.BufferGeometry();
      g.setAttribute("position", new root.THREE.BufferAttribute(
        new Float32Array([0, 0, 0, a[0] * TRAIL_TRIAD, a[1] * TRAIL_TRIAD,
                          a[2] * TRAIL_TRIAD]), 3));
      marker.add(new root.THREE.Line(g,
        new root.THREE.LineBasicMaterial({ color: a[3] })));
    });
    scene.add(marker);
    var cam = makeOrbit(canvas, camera, TRAIL_CAM0);

    function updateTrailAlpha() {
      for (var i = 0; i < trailCount; i++) {
        var age = trailCount - 1 - i;
        alphaArr[i] = age < TRAIL_HEAD ? 1.0
          : Math.max(0.0, 1.0 - (age - TRAIL_HEAD) / Math.max(trailCount - TRAIL_HEAD, 1));
      }
      alphaAttr.needsUpdate = true;
      posAttr.needsUpdate = true;
      trailGeom.setDrawRange(0, trailCount);
    }

    return {
      marker: marker,
      geom: trailGeom,
      cam: cam,
      push: function (x, y, z) {
        if (trailCount < TRAIL_MAX) {
          var o = trailCount * 3;
          posArr[o] = x; posArr[o + 1] = y; posArr[o + 2] = z;
          trailCount++;
        } else {
          posArr.copyWithin(0, 3);
          var e = (TRAIL_MAX - 1) * 3;
          posArr[e] = x; posArr[e + 1] = y; posArr[e + 2] = z;
        }
        trailDirty = true;
      },
      clear: function () {
        trailCount = 0;
        trailGeom.setDrawRange(0, 0);
        trailDirty = false;
      },
      count: function () { return trailCount; },
      render: function () {
        if (trailDirty) { updateTrailAlpha(); trailDirty = false; }
        renderer.render(scene, camera);
      },
      resize: function () {
        var w = Math.max(2, Math.floor(canvas.clientWidth || +canvas.width));
        var h = Math.max(2, Math.floor(canvas.clientHeight || +canvas.height));
        renderer.setSize(w, h, false);
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
      },
    };
  }

  /* ══════════════════════════ euler 面板：三轴条形（exp3 画法） ═════════════════ */

  function makeEulerBars(canvas) {
    var ctx = canvas.getContext("2d");
    if (!ctx) return null;
    var W = canvas.width, H = canvas.height;
    var ROW_H = 66, PAD_L = 64, PAD_R = 24;

    function drawBar(axis, row, angle) {
      var y0 = 14 + row * ROW_H;
      var x0 = PAD_L, x1 = W - PAD_R;
      var mid = (x0 + x1) / 2, half = (x1 - x0) / 2;
      ctx.strokeStyle = "#2c3947";
      ctx.beginPath(); ctx.moveTo(mid, y0); ctx.lineTo(mid, y0 + 34); ctx.stroke();
      ctx.fillStyle = "#546e7a"; ctx.font = "10px Consolas"; ctx.textAlign = "center";
      var step = RANGES[axis] / 3;
      for (var k = -3; k <= 3; k++) {
        if (!k) continue;
        var gx = mid + half * k / 3;
        ctx.strokeStyle = "#232b32";
        ctx.beginPath(); ctx.moveTo(gx, y0 + 34); ctx.lineTo(gx, y0 + 38); ctx.stroke();
        ctx.fillText((k > 0 ? "+" : "") + Math.round(step * k) + "\u00B0", gx, y0 + 49);
      }
      if (axis === "pitch") {                 // pitch 锁边界 ±90°（红虚线）
        ctx.strokeStyle = "#e57373"; ctx.setLineDash([4, 3]);
        [x0, x1].forEach(function (gx) {
          ctx.beginPath(); ctx.moveTo(gx, y0 - 2); ctx.lineTo(gx, y0 + 36); ctx.stroke();
        });
        ctx.setLineDash([]);
      }
      var frac = Math.max(-1, Math.min(1, angle / RANGES[axis]));
      var bx = frac >= 0 ? mid : mid + half * frac;
      var bw = Math.abs(half * frac);
      ctx.fillStyle = COLORS[axis];
      ctx.fillRect(bx, y0 + 8, Math.max(bw, frac === 0 ? 0 : 1), 18);
      ctx.fillStyle = COLORS[axis]; ctx.font = "13px Consolas"; ctx.textAlign = "left";
      ctx.fillText(axis, 8, y0 + 22);
      ctx.textAlign = "right";
      ctx.fillText(signed(angle, 2) + "\u00B0", x1, y0 + 4);
    }

    return {
      render: function (d) {
        ctx.clearRect(0, 0, W, H);
        drawBar("roll", 0, d[0]);
        drawBar("pitch", 1, d[1]);
        drawBar("yaw", 2, d[2]);
      },
      clear: function () { ctx.clearRect(0, 0, W, H); },
    };
  }

  /* ══════════════════════════ 页面装配 ══════════════════════════ */

  injectStyle();
  doc.body.innerHTML = markup();
  UI.define(DICT);
  UI.init({ page: PAGE_ID });          // 只渲染壳，不启动数据源

  var titleEl = byId("title");
  var statEl = byId("stat");        // 页脚计数行（每页模板不同，见 STAT_KEYS）
  var logEl = byId("log");
  var portEl = byId("port");
  var targetEl = byId("target");       // 真实档目标串（tcp:/serial:）输入框
  var logT0 = Date.now();               // 日志时间戳基准（收流开始时重置）

  var THREE_OK = !!root.THREE;
  var accS = null, gyrS = null, cubeView = null, trailView = null, barsView = null;
  var cube3dOff = false, traj3dOff = false;    // WebGL 不可用（无 GPU）时的降级标记
  var stats = makeWinStats(WIN_N);

  if (ENABLED.raw && THREE_OK) {
    try {
      accS = makeScope3(byId("cAcc"), CH_COLORS, byId("rng-acc"));
      gyrS = makeScope3(byId("cGyro"), CH_COLORS, byId("rng-gyr"));
    } catch (e) {
      log("[host] " + UI.t("imu.msg.webglOff"), true);
    }
  }
  if (ENABLED.quat && THREE_OK) {
    try {
      cubeView = makeCubeView(byId("view3d"));
    } catch (e) {
      cube3dOff = true;
    }
  }
  if (ENABLED.traj && THREE_OK) {
    try {
      trailView = makeTrailView(byId("trail3d"));
    } catch (e) {
      traj3dOff = true;
    }
  }
  if (ENABLED.euler) barsView = makeEulerBars(byId("bars"));

  /* —— 运行态 —— */
  var counters = { imu: 0, cal: 0, euler: 0, quat: 0, text: 0, drop: 0 };
  var lastCal = null, lastQuat = null, lastNorm = null, lastPos = [0, 0, 0];
  var lastEuler = null, peaks = { roll: 0, pitch: 0, yaw: 0 };
  var lastAcc = null, bridge = null, connected = false;

  /* ══════════════════════ 文案与面板渲染（zh/en 同一路径） ══════════════════════ */

  function renderTexts() {
    titleEl.textContent = UI.t("imu.title." + PAGE_ID);
    doc.title = UI.t("imu.title." + PAGE_ID);      // 浏览器标签页标题随语言切
    if (ENABLED.raw) byId("cap-raw").textContent = UI.t("imu.panel.raw");
    if (ENABLED.cal) {
      byId("cap-cal").textContent = UI.t("imu.panel.cal");
      renderOpBanner();         // 命令态文案随语言重刷
      renderOpButtons();        // 壳的 renderChrome 会无条件启开按钮，这里按会话态收回
      renderOpProgress();
      renderCmdOut();
    }
    if (ENABLED.euler) byId("cap-euler").textContent = UI.t("imu.panel.euler");
    if (ENABLED.quat) {
      byId("cap-quat").textContent = UI.t("imu.panel.quat");
      byId("cap-quat-euler").textContent = UI.t("imu.quat.eulerTitle");
      byId("cube-hint").textContent =
        UI.t(cube3dOff ? "imu.msg.webglOff" : "imu.hint.cube");
    }
    if (ENABLED.traj) {
      byId("cap-traj").textContent = UI.t("imu.panel.traj");
      byId("traj-hint").textContent =
        UI.t(traj3dOff ? "imu.msg.webglOff" : "imu.traj.hint");
      byId("btn-clear").textContent = UI.t("imu.traj.clear");
    }
    byId("cap-log").textContent = UI.t("imu.panel.log");
    renderCalTables();
    renderEulerTables();
    renderQuatEulerTable();
  }

  function renderCalTables() {
    if (!ENABLED.cal) return;
    var rows = AXES6.map(function (a, i) {
      var ref = ["0", "0", "+1", "0", "0", "0"][i];
      return '<tr><td class="ax">' + a + (i < 3 ? " (g)" : " (dps)") + "</td>" +
        '<td class="raw" id="r-' + a + '">—</td>' +
        '<td class="deb" id="d-' + a + '">—</td>' +
        '<td class="bias" id="b-' + a + '">—</td>' +
        '<td class="ref">' + ref + "</td></tr>";
    }).join("");
    byId("cal-table").innerHTML =
      "<tr><th>" + UI.t("imu.cal.thAxis") + "</th><th>" + UI.t("imu.cal.thRaw") +
      "</th><th>" + UI.t("imu.cal.thDeb") + "</th><th>" + UI.t("imu.cal.thBias") +
      "</th><th>" + UI.t("imu.cal.thRef") + "</th></tr>" + rows;
    var srows = AXES6.map(function (a) {
      return '<tr><td class="ax">' + a + "</td>" +
        '<td class="raw" id="m-' + a + '">—</td>' +
        '<td class="deb" id="s-' + a + '">—</td></tr>';
    }).join("");
    byId("cal-stats-table").innerHTML =
      "<tr><th>" + UI.t("imu.cal.thAxisShort") + "</th><th>" +
      UI.t("imu.cal.thRawStat") + "</th><th>" + UI.t("imu.cal.thDebStat") +
      "</th></tr>" + srows;
    renderCal();
    renderCalStats();
  }

  function renderEulerTables() {
    if (!ENABLED.euler) return;
    var rows = AXES3.map(function (a) {
      return '<tr id="er-' + a + '"><td class="ax">' +
        '<span class="sw" style="background:' + COLORS[a] + '"></span>' +
        UI.t("imu.euler." + a) + "</td>" +
        '<td id="v-' + a + '">—</td><td id="r-' + a + '">—</td>' +
        '<td id="p-' + a + '">—</td>' +
        '<td class="ref">' + UI.t(a === "pitch" ? "imu.euler.rangePitch" : "imu.euler.rangeRP") +
        "</td></tr>";
    }).join("");
    byId("euler").innerHTML =
      "<tr><th>" + UI.t("imu.euler.thAxis") + "</th><th>" + UI.t("imu.euler.thDeg") +
      "</th><th>" + UI.t("imu.euler.thRad") + "</th><th>" + UI.t("imu.euler.thPeak") +
      "</th><th>" + UI.t("imu.euler.thRange") + "</th></tr>" + rows;
    renderEulerTable();
  }

  function renderQuatEulerTable() {
    if (!ENABLED.quat) return;
    var rows = AXES3.map(function (a) {
      return '<tr id="qr-' + a + '"><td class="ax">' +
        '<span class="sw" style="background:' + COLORS[a] + '"></span>' +
        UI.t("imu.euler." + a) + "</td>" +
        '<td id="e-' + a + '">—</td>' +
        '<td class="ref">' + UI.t(a === "pitch" ? "imu.euler.rangePitch" : "imu.euler.rangeRP") +
        "</td></tr>";
    }).join("");
    byId("quat-euler").innerHTML =
      "<tr><th>" + UI.t("imu.quat.thAxis") + "</th><th>" + UI.t("imu.quat.thDeg") +
      "</th><th>" + UI.t("imu.quat.thRange") + "</th></tr>" + rows;
    byId("locknote").textContent = UI.t("imu.quat.locknote");
    renderQuatEuler();
  }

  function renderRawVals() {
    if (!ENABLED.raw || !lastAcc) return;
    var rows = [["ax", lastAcc[0]], ["ay", lastAcc[1]], ["az", lastAcc[2]],
                ["gx", lastAcc[3]], ["gy", lastAcc[4]], ["gz", lastAcc[5]]];
    byId("vals").innerHTML = rows.map(function (r) {
      return '<div class="v">' + r[0] + " <b>" + r[1].toFixed(4) + "</b></div>";
    }).join("");
  }

  function renderCal() {
    if (!ENABLED.cal) return;
    var v = lastCal;
    var cell = function (idx) { return v === null ? "—" : FMT(v[idx]); };
    AXES6.forEach(function (a, i) {
      byId("r-" + a).textContent = cell(i);         // raw（补偿前）
      byId("d-" + a).textContent = cell(6 + i);     // deb（补偿后）
      byId("b-" + a).textContent = cell(12 + i);    // bias（当前零偏）
    });
    var nowTxt = v === null ? "—" : FMT(v[18]) + " \u00B0C";
    var rangeTxt = stats.n() > 0
      ? FMT(stats.min(18)) + " ~ " + FMT(stats.max(18)) + " \u00B0C" : "—";
    byId("temp").innerHTML = UI.t("imu.cal.temp", { now: nowTxt, range: rangeTxt });
    byId("wait").textContent = v === null ? UI.t("imu.cal.wait") : "";
    renderCalDelta();
  }

  /* Δ 行（单独占一行，六轴）：Δ = raw − deb，即主机/固件实际减掉的零偏。
     调零前固件与主机都未减偏 → Δ = 0；调零后立刻跳到 ≈ bias（sim 档 = 注入零偏）。 */
  function renderCalDelta() {
    if (!ENABLED.cal) return;
    var el = byId("cal-delta");
    if (!el) return;
    var cells = AXES6.map(function (a, i) {
      var d = lastCal === null ? 0 : lastCal[i] - lastCal[6 + i];
      return a + " " + d.toFixed(4);
    });
    el.textContent = UI.t("imu.cal.thDelta") + "  " + cells.join(" · ");
  }

  function renderCalStats() {
    if (!ENABLED.cal) return;
    var pair = function (x, y) {
      return (x === null ? "—" : FMT(x)) + " \u00B1 " + (y === null ? "—" : FMT(y));
    };
    AXES6.forEach(function (a, i) {
      byId("m-" + a).textContent = pair(stats.mean(i), stats.std(i));
      byId("s-" + a).textContent = pair(stats.mean(6 + i), stats.std(6 + i));
    });
    byId("stat-title").textContent =
      UI.t("imu.cal.statTitle", { n: stats.n(), N: WIN_N });
  }

  /* ═══════════════ 命令通道：三个控件（读取当前值 / 一键调零 / 温漂补偿） ═══════════════
     **算法全在 MCU**（用户定案 2026-11-07）：本页只发 CMD 帧、只读 ACK 回执，
     不做任何减偏/拟合。协议（与 backend/hub.py 的 CMD/ACK 单点表一致）：
       CMD 0x7D payload = [MOD_IMU=1][FUNC][params…]
       ACK 0x7E payload = [MOD_IMU][FUNC][status][data…]
     func 用法：read      = get_bias 0x02 + get_status 0x0A + get_drift 0x0D
                calibrate = 0x01（异步：固件采 500 样本后回 ACK，回 bias6）
                tc        = temp_calib 0x03 → get_status 0x0A → set_flags 0x09
                            → get_tc_state 0x10 → get_status（回读生效位）
     服务器入口 POST /api/imu/cmd {"func","args"} → {"func","status","payload"}；失败 409 {"error"}。
     一键调零走旧入口 /api/calib（服务器内部即 func=calibrate）：既有页测试固定了
     bridge.sendCalib 这条路径，下行命令与 /api/imu/cmd 完全相同，回执同样带回 payload=bias6。 */
  var CMD_TIMEOUT_MS = 8000;        // 一般命令回执兜底（服务器 5s 更早收口）
  var CAL_TIMEOUT_MS = 22000;       // CALIBRATE 异步（固件 500 样本 ≈5s；服务器 20s）
  var CAL_SAMPLES = 500;            // 固件标定窗（进度/文案口径）
  var CAL_WINDOW_EST_S = CAL_SAMPLES / 100;   // 估算进度用（100Hz 采样率）
  var FLAG_TEMPCOMP_EN = 0x0001;    // SYS_FLAGS bit0（= hub.SYSF_TEMPCOMP_EN）

  var CMD_BTN_OP = { "btn-imu-read": "read", "btn-cal": "calibrate", "btn-tc": "tc" };
  var CMD_BTN_LABEL = { read: "imu.cmd.read", calibrate: "imu.cmd.cal", tc: "imu.cmd.tc" };
  var CMD_BTN_TIP = { read: "imu.cmd.readTip", calibrate: "imu.cmd.calTip", tc: "imu.cmd.tcTip" };

  /* 一次一个操作在飞：pending（按钮禁用/「校准中…」/进度）→ done | failed（明报原因） */
  var op = { name: "", phase: "idle", timer: 0, prog: 0, t0: 0, sec: 0,
             exact: null, err: null,
             bias: null, status: null, drift: null, tc: null };

  function opBusy() { return op.phase === "pending"; }

  function opTimeoutMs(name) {
    return name === "calibrate" ? CAL_TIMEOUT_MS : CMD_TIMEOUT_MS;
  }

  function clearOpTimers() {
    if (op.timer) { clearTimeout(op.timer); op.timer = 0; }
    if (op.prog) { clearInterval(op.prog); op.prog = 0; }
  }

  function fmtSigned(v, dig) { return (v >= 0 ? "+" : "") + v.toFixed(dig); }
  function fmt3(v) { return v === null || v === undefined ? "—" : v.toFixed(3); }
  function fmtK(values) {
    if (!values) return "—";
    return values.map(function (v) { return v.toFixed(4); }).join(",");
  }

  function biasVars() {
    var b = op.bias;
    if (!b) return { ax: "—", ay: "—", az: "—", gx: "—", gy: "—", gz: "—" };
    return { ax: Math.round(b[0] * 1000), ay: Math.round(b[1] * 1000),
             az: Math.round(b[2] * 1000),
             gx: fmt3(b[3]), gy: fmt3(b[4]), gz: fmt3(b[5]) };
  }

  function readVars() {
    var s = op.status;
    return { bv: s ? s[1] : "—", tv: s ? s[2] : "—" };
  }

  function tcVars() {
    var tc = op.tc;
    return { valid: tc ? tc[0] : "—", enabled: tc ? tc[1] : "—",
             k: tc ? fmtK(tc.slice(2)) : "—" };
  }

  function renderOpBanner() {
    var el = byId("cal-banner");
    if (!el) return;
    var cls = "", text = "";
    if (op.phase === "pending") {
      cls = "pending";
      text = op.name === "calibrate" ? UI.t("imu.cmd.pending", { n: CAL_SAMPLES })
        : op.name === "tc" ? UI.t("imu.cmd.tcPending", { sec: op.sec })
        : UI.t("imu.cmd.readPending");
    } else if (op.phase === "done") {
      cls = "done";
      text = op.name === "calibrate" ? UI.t("imu.cmd.done", biasVars())
        : op.name === "tc" ? UI.t("imu.cmd.tcDone", tcVars())
        : UI.t("imu.cmd.readDone", readVars());
    } else if (op.phase === "failed") {
      cls = "fail";
      text = op.err ? UI.t("imu.cmd.fail", { msg: op.err }) : UI.t("imu.cmd.timeout");
    }
    el.className = cls;
    el.textContent = text;
  }

  /* 进度：有固件 TEXT `calib progress k/N` 就用实数，否则按标定窗估算（并明说是估算） */
  function renderOpProgress() {
    var el = byId("cal-progress");
    if (!el) return;
    if (!(opBusy() && op.name === "calibrate")) { el.textContent = ""; return; }
    if (op.exact) {
      var pct = Math.min(100, Math.round(100 * op.exact.done / Math.max(op.exact.total, 1)));
      el.textContent = UI.t("imu.cmd.progress",
                            { done: op.exact.done, total: op.exact.total, pct: pct });
      return;
    }
    var sec = (Date.now() - op.t0) / 1000;
    var est = Math.min(99, Math.round(100 * sec / CAL_WINDOW_EST_S));
    el.textContent = UI.t("imu.cmd.progressEst", { sec: sec.toFixed(1), pct: est });
  }

  function renderOpButtons() {
    Object.keys(CMD_BTN_OP).forEach(function (id) {
      var el = byId(id);
      if (!el) return;
      var name = CMD_BTN_OP[id];
      el.disabled = opBusy();
      el.textContent = UI.t(opBusy() && op.name === name ? "imu.cmd.busy"
                                                         : CMD_BTN_LABEL[name]);
      el.title = UI.t(CMD_BTN_TIP[name]);
    });
    var lbl = byId("tc-sec-lbl");
    if (lbl) lbl.textContent = UI.t("imu.cmd.sec");
  }

  /* ── 回执读数（零偏 / 状态 / 温漂系数 / 温补态）── */
  function renderBias() {
    var el = byId("imu-bias");
    if (!el) return;
    var head = UI.t("imu.bias.title");
    var b = op.bias;
    if (!b) { el.textContent = head + "  " + UI.t("imu.bias.none"); return; }
    var cells = AXES6.map(function (a, i) {
      return a + " " + fmtSigned(b[i], 4) + (i < 3 ? " g" : " dps");
    });
    el.textContent = head + "  " + cells.join(" · ") + "  " +
      UI.t("imu.bias.valid", { v: op.status ? op.status[1] : "—" });
  }

  function renderStatus() {
    var el = byId("imu-status");
    if (!el) return;
    var s = op.status;
    if (!s) {
      el.textContent = UI.t("imu.status.title") + "  " + UI.t("imu.status.none");
      return;
    }
    el.textContent = UI.t("imu.status.title") + "  " + UI.t("imu.status.line", {
      flags: ("0000" + Number(s[0]).toString(16)).slice(-4),
      bv: s[1], tv: s[2], en: (s[0] & FLAG_TEMPCOMP_EN) ? 1 : 0,
    }) + (Number(s[1]) ? "" : "\n" + UI.t("imu.bias.hint"));
  }

  function renderDrift() {
    var el = byId("imu-drift");
    if (!el) return;
    var head = UI.t("imu.drift.title");
    var k = op.drift;
    if (!k) { el.textContent = head + "  " + UI.t("imu.drift.none"); return; }
    el.textContent = head + "  " +
      AXES6.map(function (a, i) { return a + " " + fmtSigned(k[i], 5); }).join(" · ");
  }

  function renderTcState() {
    var el = byId("imu-tc");
    if (!el) return;
    var tc = op.tc;
    if (!tc) { el.textContent = UI.t("imu.tc.title") + "  " + UI.t("imu.tc.none"); return; }
    el.textContent = UI.t("imu.tc.title") + "  " + UI.t("imu.tc.line", tcVars());
  }

  function renderCmdOut() {
    renderBias(); renderStatus(); renderDrift(); renderTcState();
  }

  function opBegin(name, sec) {
    clearOpTimers();
    op.name = name;
    op.phase = "pending";
    op.t0 = Date.now();
    op.sec = sec || 0;
    op.exact = null;
    op.err = null;
    if (name === "calibrate") op.prog = setInterval(renderOpProgress, 100);
    op.timer = setTimeout(function () {
      op.timer = 0;
      if (!opBusy()) return;
      opFail(null);              // 客户端兜底超时（服务器一般先回 409）
    }, opTimeoutMs(name));
    renderOpButtons(); renderOpBanner(); renderOpProgress();
  }

  function opSucceed(name) {
    clearOpTimers();
    op.name = name;
    op.phase = "done";
    op.err = null;
    renderOpButtons(); renderOpBanner(); renderOpProgress(); renderCmdOut();
  }

  function opFail(err) {
    var wasPending = opBusy();
    clearOpTimers();
    op.phase = "failed";
    op.err = err || null;
    renderOpButtons(); renderOpBanner(); renderOpProgress(); renderCmdOut();
    if (wasPending) log("[cmd] " + (err || UI.t("imu.cmd.timeout")), true);
  }

  function opClear() {
    clearOpTimers();
    op.name = "";
    op.phase = "idle";
    op.err = null;
    op.exact = null;
    renderOpButtons(); renderOpBanner(); renderOpProgress();
  }

  function errText(err) {
    if (!err) return null;
    return err.message ? err.message : String(err);
  }

  /* ── 命令传输（页面唯一出口）：POST /api/imu/cmd；409/网络错 → reject ── */
  function imuCmd(func, args) {
    return fetch("/api/imu/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ func: func, args: args || {} }),
    }).then(function (res) {
      return res.json().catch(function () { return {}; }).then(function (data) {
        if (!res.ok) throw new Error((data && data.error) || ("HTTP " + res.status));
        return data;
      });
    });
  }

  /* 一键调零的下行：走旧入口（服务器内部 = func=calibrate），回执同样带 payload=bias6 */
  function sendCalibrateCmd() {
    if (!bridge || typeof bridge.sendCalib !== "function") {
      return Promise.reject(new Error(UI.t("imu.cmd.notConnected")));
    }
    return Promise.resolve(bridge.sendCalib()).then(function (msg) {
      if (msg && msg.code === "notConnected") throw new Error(UI.t("imu.cmd.notConnected"));
      return msg || {};
    });
  }

  /* 控件 1：读取当前值 = GET_BIAS + GET_STATUS + GET_DRIFT */
  function cmdRead() {
    opBegin("read");
    Promise.all([imuCmd("get_bias"), imuCmd("get_status"), imuCmd("get_drift")])
      .then(function (r) {
        op.bias = r[0].payload || null;
        op.status = r[1].payload || null;
        op.drift = r[2].payload || null;
        opSucceed("read");
        log("[cmd] " + UI.t("imu.cmd.readDone", readVars()));
      }, function (err) { opFail(errText(err)); });
  }

  /* 控件 2：一键调零 = CALIBRATE(0x01)（异步，完成回 bias6）→ 顺带 GET_STATUS */
  function cmdCalibrate() {
    opBegin("calibrate");
    stats.reset();
    renderCalStats();
    renderStat();
    sendCalibrateCmd()
      .then(function (msg) {
        if (msg.code) {                 // 旧入口回执码（calibSimulated / calibSent）如实入日志
          var key = "imu.msg." + msg.code;
          log("[cal] " + (UI.has(key) ? UI.t(key, msg) : UI.t("msg." + msg.code, msg)));
        }
        if (msg.payload && msg.payload.length === 6) op.bias = msg.payload;
        if (msg.status !== undefined && msg.status !== 0) {
          throw new Error("CALIBRATE status=" + msg.status);
        }
        return imuCmd("get_status").then(function (st) { op.status = st.payload || null; },
                                        function () { /* 读状态失败不掩盖标定成功 */ });
      })
      .then(function () {
        opSucceed("calibrate");
        log("[cmd] " + UI.t("imu.cmd.done", biasVars()));
      }, function (err) { opFail(errText(err)); });
  }

  /* 控件 3：温漂补偿 = TEMP_CALIB(0x03,秒) → GET_STATUS → SET_FLAGS(置 TEMPCOMP_EN)
     → GET_TC_STATE（tc_valid + k）→ GET_STATUS（回读生效位） */
  function cmdTempComp() {
    var secInput = byId("tc-sec");
    var sec = Math.max(1, Math.min(255,
      parseInt((secInput && secInput.value) || "30", 10) || 30));
    opBegin("tc", sec);
    imuCmd("temp_calib", { seconds: sec })
      .then(function (r) {
        op.drift = r.payload || null;                    // TEMP_CALIB 回 k6
        return imuCmd("get_status").then(function (st) {
          op.status = st.payload || null;
          var flags = ((op.status ? op.status[0] : 0) | FLAG_TEMPCOMP_EN) & 0xFFFF;
          return imuCmd("set_flags", { flags: flags }).then(function (sf) {
            log("[cmd] set_flags 0x" + Number(sf.payload[0]).toString(16));
            return imuCmd("get_tc_state").then(function (tc) {
              op.tc = tc.payload || null;
              return imuCmd("get_status").then(function (st2) {
                op.status = st2.payload || null;
              }, function () { /* 回读失败不掩盖标定成功 */ });
            });
          });
        });
      })
      .then(function () {
        opSucceed("tc");
        log("[cmd] " + UI.t("imu.cmd.tcDone", tcVars()));
      }, function (err) { opFail(errText(err)); });
  }

  function renderEulerTable() {
    if (!ENABLED.euler) return;
    AXES3.forEach(function (a, i) {
      if (lastEuler === null) {
        byId("v-" + a).textContent = "—";
        byId("r-" + a).textContent = "—";
        byId("p-" + a).textContent = "—";
        return;
      }
      var d = deg(lastEuler[i]);
      var av = Math.abs(d);
      if (av > peaks[a]) peaks[a] = av;
      byId("v-" + a).textContent = signed(d, 3) + " \u00B0";
      byId("r-" + a).textContent = signed(lastEuler[i], 4);
      byId("p-" + a).textContent = signed(peaks[a], 3) + " \u00B0";
    });
    if (barsView) {
      if (lastEuler === null) barsView.clear();
      else barsView.render([deg(lastEuler[0]), deg(lastEuler[1]), deg(lastEuler[2])]);
    }
  }

  function renderQuatVals() {
    if (!ENABLED.quat) return;
    var q = lastQuat;
    var f = function (v) { return v === null || v === undefined ? "—" : v.toFixed(4); };
    var html = '<div class="g6">' + UI.t("imu.quat.vals") + "</div>";
    html += ["w", "x", "y", "z"].map(function (n, i) {
      return '<div class="v">q' + n + " <b>" + f(q ? q[i] : null) + "</b></div>";
    }).join("");
    html += '<div class="g6">' + UI.t("imu.quat.norm") + "</div>";
    html += '<div class="v">|q| <b>' + (lastNorm === null ? "—" : lastNorm.toFixed(4)) +
      "</b></div>";
    byId("q-vals").innerHTML = html;
  }

  function renderQuatEuler() {
    if (!ENABLED.quat) return;
    var q = lastQuat;
    var e = q === null ? null : quatToEulerZyx(q[0], q[1], q[2], q[3]);
    AXES3.forEach(function (a, i) {
      byId("e-" + a).textContent = e === null ? "—" : signed(deg(e[i]), 2) + " \u00B0";
      byId("qr-" + a).classList.toggle(
        "lock", a === "pitch" && e !== null && Math.abs(deg(e[1])) >= LOCK_DEG);
    });
    byId("locknote").classList.toggle(
      "show", e !== null && Math.abs(deg(e[1])) >= LOCK_DEG);
  }

  function renderTrajVals() {
    if (!ENABLED.traj) return;
    var f3 = function (v) { return v.toFixed(3); };
    var q = lastQuat, a = lastAcc;
    var html = '<div class="g6">' + UI.t("imu.traj.values") + "</div>";
    html += ["x", "y", "z"].map(function (n, i) {
      return '<div class="v">pos ' + n + " <b>" + f3(lastPos[i]) + " m</b></div>";
    }).join("");
    html += ["w", "x", "y", "z"].map(function (n, i) {
      return '<div class="v">q' + n + " <b>" + (q ? q[i].toFixed(3) : "—") + "</b></div>";
    }).join("");
    html += ["ax", "ay", "az"].map(function (n, i) {
      return '<div class="v">' + n + " <b>" + (a ? a[i].toFixed(3) : "—") + "</b></div>";
    }).join("");
    byId("t-vals").innerHTML = html;
  }

  function statVars() {
    return {
      imu: counters.imu, cal: counters.cal, q: counters.quat, drop: counters.drop,
      win: stats.n(), N: WIN_N,
      temp: lastCal === null ? "—" : FMT(lastCal[18]) + " \u00B0C",
      norm: lastNorm === null ? "—" : lastNorm.toFixed(4),
      x: lastPos[0].toFixed(3), y: lastPos[1].toFixed(3), z: lastPos[2].toFixed(3),
    };
  }

  function renderStat() {
    statEl.textContent = UI.t(STAT_KEYS[PAGE_ID], statVars());
  }

  function renderAll() {
    renderRawVals();
    renderCal();
    renderCalStats();
    renderEulerTable();
    renderQuatVals();
    renderQuatEuler();
    renderTrajVals();
    renderStat();
  }

  function log(msg, warn) {
    var d = doc.createElement("div");
    if (warn) d.className = "fw";
    else if (String(msg).indexOf("OK calib") >= 0) d.className = "calok";
    // 时间戳 = 相对收流起点（未启源时相对页面加载）：标定与帧的先后顺序可对时
    d.textContent = "[+" + ((Date.now() - (logT0 || Date.now())) / 1000).toFixed(3) +
      "s] " + msg;
    logEl.appendChild(d);
    while (logEl.children.length > 200) logEl.removeChild(logEl.firstChild);
    logEl.scrollTop = logEl.scrollHeight;      // 自动滚到底（最新回执可见）
  }

  /* ══════════════════════ 波形泵（抖动缓冲 + 匀速消费 + 亚像素相位） ═══════════════
     抖动根因与修法（exp1 实测）：到达抖动 → 抽帧数抖动 → x 轴按样本序号推进；
     ① 抖动缓冲 60ms 起播；② 虚拟帧时钟按协议 100Hz 匀速消费；③ 亚像素相位。 */
  var lastNow = 0, tickAcc = 0, primed = false;

  function resetView() {
    if (accS) accS.clear();
    if (gyrS) gyrS.clear();
    primed = false; tickAcc = 0; lastNow = 0;
  }

  /* 泵体（headless 契约）：入参 dtMs = 与上次 rAF 的时间差（ms）；返回亚像素相位。 */
  function pump(dtMs) {
    if (!accS) return 0;
    if (!primed && accS.qlen() >= Math.round(FRAME_HZ * JITTER_BUF_S)) primed = true;
    if (primed) {
      var over = accS.qlen() - FRAME_HZ * 5;
      if (over > 0) { accS.trim(FRAME_HZ * 5); gyrS.trim(FRAME_HZ * 5); }
      tickAcc += Math.min(dtMs, 250) / 1000 * FRAME_HZ;
      var n = Math.floor(tickAcc);
      if (n > 0) { tickAcc -= n; accS.consume(n); gyrS.consume(n); }
    }
    var phase = primed ? Math.min(tickAcc, 0.999) : 0;
    root.__xrkWave = { phase: phase, rate: FRAME_HZ, primed: primed,
                       qlen: accS.qlen(), count: accS.count() };
    return phase;
  }

  function tick(t) {
    var dt = lastNow ? t - lastNow : 0;
    lastNow = t;
    var phase = pump(dt);
    if (accS) accS.draw(phase);
    if (gyrS) gyrS.draw(phase);
    if (cubeView) cubeView.render();
    if (trailView) trailView.render();
    root.requestAnimationFrame(tick);
  }

  /* headless 契约（tests/imu_page_harness.js / exp1_wave_harness.js）：
     泵体/窗长/帧率/抖动缓冲深度 + 数值换算 + 统计窗构造。 */
  root.__xrkDebug = {
    pump: pump, frameHz: FRAME_HZ, jitterBufS: JITTER_BUF_S,
    count: function () { return accS ? accS.count() : 0; },
    qlen: function () { return accS ? accS.qlen() : 0; },
    pushSample: function (ax, ay, az, gx) {
      if (!accS) return;
      accS.push([ax, ay, az]); gyrS.push([gx, 0, 0]);
    },
    reset: resetView,
    get phase() { return (root.__xrkWave || {}).phase; },
    panels: PANELS_ON,
    // 命令通道契约（harness 驱动 pending/done/failed 三态；超时可缩短免真等）
    cmdTimeout: function (ms) {
      if (typeof ms === "number" && ms > 0) { CMD_TIMEOUT_MS = ms; CAL_TIMEOUT_MS = ms; }
      return CMD_TIMEOUT_MS;
    },
    opPhase: function () { return op.phase; },
    opName: function () { return op.name; },
  };
  root.quatToEulerZyx = quatToEulerZyx;      // 数值契约（镜像固件 quat.c）
  root.quatNorm = quatNorm;
  root.makeWinStats = makeWinStats;          // 统计窗契约（镜像 exp2 页口径）

  /* ══════════════════════ 数据源：显式启动（禁隐式回落） ══════════════════════ */

  function waitBridge(fn) {
    var wait = setInterval(function () {
      if (bridge) { clearInterval(wait); fn(); }
    }, 200);
  }

  /* 旧 kick 契约（app.py --mode sim / Qt 页测试）：显式切仿真模式再启源。 */
  function startSim() {
    UI.setMode("sim", { silent: true });
    waitBridge(function () { if (!connected) bridge.openSim(); });
  }

  /* 旧 kick 契约（app.py 位置参数串口）：显式切真实模式 + 选端口 + 连接。 */
  function autoConnect(port) {
    UI.setMode("real", { silent: true });
    var sel = byId("port");
    var wait = setInterval(function () {
      if (sel.options.length) {
        clearInterval(wait);
        for (var i = 0; i < sel.options.length; i++) {
          if (sel.options[i].value === port) sel.value = port;
        }
        if (!connected) bridge.openPort(realTarget(sel.value || port), 115200);
      }
    }, 200);
  }
  root.startSim = startSim;
  root.autoConnect = autoConnect;

  /* 真实档目标串（与 server 的 _imu_real_target 同一条规则）：
     输入框优先（tcp:<host>:<port> / serial:<COMx>@<baud> 直通），否则由串口名拼
     serial:<COMx>@115200——页面与服务器只认一条目标串，不各留一套。 */
  function realTarget(device) {
    var typed = String(((targetEl && targetEl.value) || "")).trim();
    if (typed) return typed;
    var dev = String(device || "").trim() ||
      String((portEl && portEl.value) || "").trim() || "COM3";
    if (/^tcp:\S+$/.test(dev) || /^serial:\S+$/.test(dev)) return dev;
    return "serial:" + dev + "@115200";
  }

  function onFrame(type, seq, vals) {
    if (type === 1) {                                  // IMU_SAMPLE：裸波形
      counters.imu++;
      lastAcc = vals.slice(0, 6);
      if (accS) { accS.push(lastAcc.slice(0, 3)); gyrS.push(lastAcc.slice(3, 6)); }
      renderRawVals();
    } else if (type === 2 && vals.length === 19) {      // IMU_CAL：校准面板（固件侧 raw/deb/bias）
      counters.cal++;
      lastCal = vals.slice(0, 19);
      stats.push(lastCal);
      renderCal();
      renderCalStats();
    } else if (type === 3 && vals.length === 3) {       // EULER：欧拉条形
      counters.euler++;
      lastEuler = vals.slice(0, 3);
      renderEulerTable();
    } else if (type === 4 && vals.length === 4) {       // QUAT：立方 + 四元数读数
      counters.quat++;
      lastQuat = vals.slice(0, 4);
      lastNorm = quatNorm(lastQuat[0], lastQuat[1], lastQuat[2], lastQuat[3]);
      if (cubeView) {
        /* THREE.Quaternion.set(x,y,z,w)：帧 w 在前；normalize 仅显示层防漂 */
        cubeView.cube.quaternion.set(lastQuat[1], lastQuat[2], lastQuat[3],
                                     lastQuat[0]).normalize();
      }
      if (trailView) trailView.marker.quaternion.set(lastQuat[1], lastQuat[2],
                                                     lastQuat[3], lastQuat[0]).normalize();
      renderQuatVals();
      renderQuatEuler();
      if (ENABLED.traj) renderTrajVals();
    }
    renderStat();
  }

  bridge = root.LiveBridge();        // 页面加载即连 SSE（只连流，不启动任何数据源）
  {
    Promise.resolve(bridge.listPorts()).then(function (ports) {
      (Array.isArray(ports) ? ports : Array.from(ports || [])).forEach(function (p) {
        var o = doc.createElement("option");
        o.value = o.textContent = p;
        portEl.appendChild(o);
      });
    });
    bridge.frameReady.connect(onFrame);
    bridge.textReady.connect(function (m) {
      counters.text++;
      log(m);
      // 固件若上报标定进度（`OK calib progress k/N`），进度改用实数（否则按窗估算）
      var pm = /calib progress\s+(\d+)\s*\/\s*(\d+)/.exec(String(m));
      if (pm && opBusy() && op.name === "calibrate") {
        op.exact = { done: +pm[1], total: +pm[2] };
        renderOpProgress();
      }
    });
    bridge.statReady.connect(function (imu, text, drop) {
      counters.drop = drop || 0;
      renderStat();
    });
    if (ENABLED.traj) {
      bridge.posReady.connect(function (x, y, z) {
        lastPos = [x, y, z];
        if (trailView) {
          trailView.marker.position.set(x, y, z);
          trailView.push(x, y, z);
        }
        renderTrajVals();
        renderStat();
      });
    }
    bridge.portChanged.connect(function (on) {
      connected = on;
      UI.setConnected(on);           // 壳的 renderChrome 会把按钮启开，下面立即按会话态收回
      opClear();                     // 连接/断开都清「进行中」：不许挂着假进度
      if (on) {
        logT0 = Date.now();          // 日志时间戳基准 = 收流起点
        return;
      }
      {                              // 断开即清窗（不残留上一条流）
        lastCal = null; lastQuat = null; lastNorm = null; lastEuler = null;
        lastAcc = null; lastPos = [0, 0, 0];
        peaks = { roll: 0, pitch: 0, yaw: 0 };
        stats.reset();
        resetView();
        if (trailView) trailView.clear();
        renderAll();
      }
    });
  }

  /* 三个控件（用户定案：算法全在 MCU，上位机只发命令 + 读回执）：
       读取当前值 = GET_BIAS + GET_STATUS + GET_DRIFT
       一键调零   = CALIBRATE（异步，按钮「校准中…」+ 进度，完成显示新零偏）
       温漂补偿   = TEMP_CALIB(秒) → SET_FLAGS(TEMPCOMP_EN) → GET_TC_STATE/GET_STATUS */
  if (ENABLED.cal) {
    byId("btn-imu-read").onclick = function () { if (!opBusy()) cmdRead(); };
    byId("btn-cal").onclick = function () { if (!opBusy()) cmdCalibrate(); };
    byId("btn-tc").onclick = function () { if (!opBusy()) cmdTempComp(); };
  }
  if (ENABLED.traj) {
    byId("btn-clear").onclick = function () { if (trailView) trailView.clear(); };
  }

  /* 语言切换：静态文案 + 动态面板/页脚整轮重建（中英同路径） */
  UI.onLangChange(function () { renderTexts(); renderAll(); });

  /* 模式切换：先断开当前流（避免仿真与真实叠加），再由用户点按钮启源 */
  UI.onModeChange(function () {
    opClear();                       // 换模式即作废在飞的命令会话
    if (UI.connected) bridge.closePort();
  });
  onRoot("pagehide", opClear);
  onRoot("beforeunload", opClear);

  /* 按钮 = 显式动作分发：仿真→合成源，真实→目标串（tcp:… / serial:…）；不自动、不回落 */
  byId("btn").onclick = function () {
    if (!bridge) return;
    if (UI.connected) { bridge.closePort(); return; }
    if (UI.mode === "sim") { bridge.openSim(); return; }
    bridge.openPort(realTarget(), 115200);
  };

  function resizeAll() {
    if (cubeView) cubeView.resize();
    if (trailView) trailView.resize();
  }
  onRoot("resize", resizeAll);

  renderTexts();
  renderAll();
  resizeAll();
  root.requestAnimationFrame(tick);
})(typeof globalThis !== "undefined" ? globalThis : this);
