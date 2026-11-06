
"use strict";
/* —— 页壳：模式（仿真/真实）与语言（中/EN）——
   加载时不启动任何数据源（禁隐式仿真/禁自动连 QEMU）；模式由用户点选：
   "sim" → bridge.openSim()（SimCalSource 镜像固件开机标定时序）
   "real" → bridge.openPort(选中串口)。文案统一走 ui-shell 词典。 */
UI.define({
  zh: {
    "exp2.title": "exp2 · 调零 + 温补（0x02 IMU_CAL · 76B · raw|deb|bias|temp）",
    "exp2.calBtn": "调零",
    "exp2.calTip": "向固件下发 CAL 明文行（下行链路演示）并重开统计窗",
    "exp2.latestTitle": "最新帧 · 温补前后对比（raw = 补偿前 / deb = 补偿后）",
    "exp2.thAxis": "轴",
    "exp2.thRaw": "原始 raw",
    "exp2.thDeb": "补偿后 deb",
    "exp2.thBias": "当前零偏 bias",
    "exp2.thRef": "参考（静止 Z_UP）",
    "exp2.temp": "芯片温度 <b>{now}</b>（窗口 {range}）",
    "exp2.statTitle": "零偏统计（窗口均值 ± 标准差 · 窗 {n}/{N} 帧）",
    "exp2.thRawStat": "raw 均值 ± σ",
    "exp2.thDebStat": "deb 均值 ± σ",
    "exp2.thRead": "读数说明",
    "exp2.readNote": "raw 均值偏移 = 零偏 + 温漂（温升时爬走）；<br>deb 均值贴参考 = 补偿生效。<br>σ 为窗口内抖动（噪声+阶梯残差）。",
    "exp2.noteTitle": "调零按钮流程",
    "exp2.note": "<b>「调零」= 命令下发 + 窗口重开</b>：<br>1. 经串口向固件发明文行 <code>CAL\\n</code>（与 exp_motor 的 <code>SCEN&nbsp;&lt;n&gt;</code> 命令同款下行协议）；<br>2. 重开本页统计窗（重新累计均值）。<br><b>现状（如实）</b>：当前 exp2 固件<b>开机自标定</b>（读 N=500 帧求零偏，见固件日志 <code>OK calib 500 frames…</code>），<b>不解析下行命令</b>——按钮演示的是命令链路；固件接 RX 解析后命令即生效（回执走 0x7E ACK）。仿真模式下无下行通道，回执会如实说明。<br><b>温补口径</b>：bias(T) = bias_ref + k·(T − T_ref)，0.5°C 阶梯重算——看「当前零偏」列随温度爬走、deb 列贴住参考值。",
    "exp2.logTitle": "固件日志（TEXT 帧 · 开机标定时序）",
    "exp2.stat": "IMU_CAL {n} · 窗口 {win}/{N} · 温度 {temp}",
  },
  en: {
    "exp2.title": "exp2 · Bias calibration + temp comp (0x02 IMU_CAL · 76B · raw|deb|bias|temp)",
    "exp2.calBtn": "Calibrate",
    "exp2.calTip": "Send the CAL plain line to firmware (downlink demo) and reset the stats window",
    "exp2.latestTitle": "Latest frame · before/after temp comp (raw = uncompensated / deb = compensated)",
    "exp2.thAxis": "Axis",
    "exp2.thRaw": "raw",
    "exp2.thDeb": "deb",
    "exp2.thBias": "bias now",
    "exp2.thRef": "Reference (static Z_UP)",
    "exp2.temp": "Chip temp <b>{now}</b> (window {range})",
    "exp2.statTitle": "Bias stats (window mean ± std · {n}/{N} frames)",
    "exp2.thRawStat": "raw mean ± σ",
    "exp2.thDebStat": "deb mean ± σ",
    "exp2.thRead": "How to read",
    "exp2.readNote": "raw mean offset = bias + temp drift (creeps up as the chip warms);<br>deb mean sits on the reference = compensation works.<br>σ is in-window jitter (noise + step residual).",
    "exp2.noteTitle": "Calibration button flow",
    "exp2.note": "<b>Calibrate = command downlink + window reset</b>:<br>1. Send the plain line <code>CAL\\n</code> over the serial port (same downlink protocol as exp_motor's <code>SCEN&nbsp;&lt;n&gt;</code>);<br>2. Reset this page's stats window (restart accumulation).<br><b>Status (honest)</b>: the current exp2 firmware <b>self-calibrates on boot</b> (reads N=500 frames, see the <code>OK calib 500 frames…</code> log line) and <b>does not parse downlink commands</b> — the button demonstrates the command path; the command takes effect once firmware parses RX (ACK via 0x7E). Simulation mode has no downlink and says so.<br><b>Temp comp</b>: bias(T) = bias_ref + k·(T − T_ref), recomputed on 0.5°C steps — watch the bias column creep with temperature while deb sticks to the reference.",
    "exp2.logTitle": "Firmware log (TEXT frames · boot calibration sequence)",
    "exp2.stat": "IMU_CAL {n} · window {win}/{N} · temp {temp}",
  },
});
UI.init({page: "exp2"});   // 只渲染壳，不启动数据源

const logEl  = document.getElementById("log");
const statEl = document.getElementById("stat");
const statusEl = document.getElementById("status");

/* 统计窗（显示层参数）：100Hz 下 3 s；块序 = 协议序（raw6|deb6|bias6|temp） */
const WIN_N = 300;
const AXES = ["ax", "ay", "az", "gx", "gy", "gz"];   // 采样序（与 0x02 块一致）
const FMT = (v) => v.toFixed(4);

/* —— 滑动窗统计（headless 可测契约，显式挂 window）——
   push 喂 19 值帧；mean/std 取第 i 分量窗口均值/样本标准差；reset 清窗。 */
function makeWinStats(winN) {
  const buf = [];
  return {
    push(vals) {
      buf.push(vals);
      if (buf.length > winN) buf.shift();
    },
    reset() { buf.length = 0; },
    n() { return buf.length; },
    mean(i) {
      if (!buf.length) return null;
      let s = 0;
      for (const v of buf) s += v[i];
      return s / buf.length;
    },
    std(i) {
      if (buf.length < 2) return null;
      const m = this.mean(i);
      let s = 0;
      for (const v of buf) s += (v[i] - m) * (v[i] - m);
      return Math.sqrt(s / (buf.length - 1));
    },
    min(i) {
      if (!buf.length) return null;
      return Math.min(...buf.map((v) => v[i]));
    },
    max(i) {
      if (!buf.length) return null;
      return Math.max(...buf.map((v) => v[i]));
    },
  };
}
window.makeWinStats = makeWinStats;   // harness 数值契约

const stats = makeWinStats(WIN_N);
let lastVals = null, nCal = 0;

function renderLatest() {
  const v = lastVals;
  const t = (idx) => (v === null ? "—" : FMT(v[idx]));   // 传下标：判空后才索引（防 null[0]）
  AXES.forEach((a, i) => {
    document.getElementById("r-" + a).textContent = t(i);        // raw（补偿前）
    document.getElementById("d-" + a).textContent = t(6 + i);    // deb（补偿后）
    document.getElementById("b-" + a).textContent = t(12 + i);   // bias（当前零偏）
  });
  const nowTxt = v === null ? "—" : FMT(v[18]) + " \u00B0C";
  const rangeTxt = stats.n() > 0
    ? FMT(stats.min(18)) + " ~ " + FMT(stats.max(18)) + " \u00B0C" : "—";
  document.getElementById("temp").innerHTML =
    UI.t("exp2.temp", { now: nowTxt, range: rangeTxt });
}

function renderStats() {
  const fmt = (x) => (x === null ? "—" : FMT(x));
  const pair = (x, y) => fmt(x) + " \u00B1 " + fmt(y);
  AXES.forEach((a, i) => {
    document.getElementById("m-" + a).textContent =
      pair(stats.mean(i), stats.std(i));
    document.getElementById("s-" + a).textContent =
      pair(stats.mean(6 + i), stats.std(6 + i));
  });
  document.getElementById("stat-title").textContent =
    UI.t("exp2.statTitle", { n: stats.n(), N: WIN_N });
  statEl.textContent = UI.t("exp2.stat", {
    n: nCal, win: stats.n(), N: WIN_N,
    temp: lastVals === null ? "—" : FMT(lastVals[18]) + " \u00B0C",
  });
}

function renderAll() { renderLatest(); renderStats(); }

function log(msg, warn) {
  const d = document.createElement("div");
  if (warn) d.className = "fw";
  d.textContent = msg;
  logEl.appendChild(d);
  while (logEl.children.length > 200) logEl.removeChild(logEl.firstChild);
  logEl.scrollTop = logEl.scrollHeight;
}

let bridge = null;

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
   仿真源镜像固件开机时序：先 5 s 标定窗（只见 TEXT 横幅无 0x02 帧），
   窗满后 0x02 流起。 */
function startSim() {
  UI.setMode("sim", { silent: true });
  waitBridge(() => { if (!UI.connected) bridge.openSim(); });
}

window.startSim = startSim;     // app.py kick 显式契约（同 exp1/exp4/exp5）
window.autoConnect = autoConnect;

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
    if (type === 2 && vals.length === 19) {    // IMU_CAL：19 值整块
      nCal++;
      lastVals = vals.slice(0, 19);
      stats.push(lastVals);
      renderAll();
    }
  });
  bridge.textReady.connect((m) => log(m));
  bridge.portChanged.connect((on) => {
    UI.setConnected(on);                        // 状态/按钮文案归壳（中英一致）
    if (!on) {
      lastVals = null; stats.reset(); renderAll();   // 断开即清窗（重连重统计）
    }
  });
}

/* 语言切换：动态面板（温度行/统计标题/页脚）重新渲染 */
UI.onLangChange(() => renderAll());

/* 模式切换：先断开当前流（避免仿真与真实叠加），再由用户点按钮启源 */
UI.onModeChange(() => { if (UI.connected) bridge.closePort(); });

/* 按钮 = 显式动作分发：仿真→合成源，真实→串口；不自动、不回落 */
document.getElementById("btn").onclick = () => {
  if (!bridge) return;
  if (UI.connected) { bridge.closePort(); return; }
  if (UI.mode === "sim") { bridge.openSim(); return; }
  bridge.openPort(document.getElementById("port").value || "COM3", 115200);
};

/* 调零按钮：① CAL 明文行下发（回执按当前语言翻译）② 统计窗重开 */
document.getElementById("btn-cal").onclick = () => {
  if (!bridge) return;
  Promise.resolve(bridge.sendCalib()).then((msg) => {
    /* 后端回执已键值化（{code, cmd}）；旧字符串回执原样打印（兼容期内） */
    trainLog(msg);
  });
  stats.reset();
  renderAll();
};

function trainLog(msg) {
  if (msg && typeof msg === "object" && msg.code) {
    log("[cal] " + UI.t("msg." + msg.code, msg));
  } else {
    log("[cal] " + String(msg));
  }
}

renderAll();
