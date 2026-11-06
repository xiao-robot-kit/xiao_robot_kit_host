/* motor-page.js — 电机实验页面核心（共享资产；7 个电机实例目录页面共用这一份）。
 *
 * 背景（用户定案 2026-11-05）：上位机 = 本地服务器（backend/server.py，端口 8760）
 * + 实例目录页面 + run.bat 一键单实例。页面核心只此一份；每页 index.html 只做薄壳：
 *
 *   <body><div id="app"></div>
 *     <script>window.MOTOR_PAGE = {id:"motor_pwm", scen:1, title:"电机 PWM 阶梯"};</script>
 *     <script src="/assets/ui-shell.js"></script>
 *     <script src="/assets/motor-page.js"></script></body>
 *
 * MOTOR_PAGE 字段（薄壳声明，页面核心只读）：
 *   id    —— 页面注册名 = 服务器 --page / URL 前缀（backend/server.py 的 MOTOR_PAGES：
 *            motor_pwm → examples/motor/exp_motor_pwm，… motor_pidscan → exp_motor_pidscan）
 *   scen  —— 本页期望的 0x06 场景号（1..5）；null = 本页不依赖 SCEN 帧
 *            （motor_scope 的 0x05 四槽 demo、exp_motor_pidscan 的遥测流）
 *   title —— 中文标题（词典 motor.title.<id> 缺失时的兜底文案）
 *   slots —— 槽位数（缺省 4；pidscan 单槽 = 1）
 *
 * 铁律：
 *   1) 加载不启动任何数据源：只做 GET /api/state、GET /api/ports 与订阅 /api/stream，
 *      绝不自动 POST /api/open。不点不连、不自动仿真、不自动回落（禁隐式）。
 *   2) 模式只有 sim/real 两档，由 ui-shell 的 UI.mode 承载（xrk.mode / xrk.lang
 *      持久化）；真实目标串显式输入并存 xrk.motorTarget。
 *   3) 模式切换 = 先 POST /api/close，再等用户点「启动/连接」。
 *   4) 文案全走 zh/en 词典（motor.* 前缀，两侧成对——缺一即 bug）；静态文案用
 *      data-i18n 锚点（ui-shell 统一刷新），动态面板由本文件按 UI.t 重渲染。
 *
 * 服务器契约（与 backend lane 约定，勿改名）：
 *   GET  /api/state  {kind,scenario,source,target,connected,frames,stats,counters}
 *   GET  /api/ports  {ports:[{device,desc}]}
 *   GET  /api/stream SSE 事件 mframe|text|verdict|state|port
 *   POST /api/open   {mode:"sim"} | {mode:"real",target:"tcp:127.0.0.1:5560"|
 *                    "serial:COM5@115200"}   失败 → 409 {error:"…"}
 *   POST /api/close、POST /api/verify（回 verdict 同形）
 *   POST /api/command {line} —— 场景激活归服务器（--page 已知 scen），页面不发下行命令：
 *                    固件 ROM 启动期注入命令会卡死 boot（scope_motor.py 实测结论），
 *                    由服务器在 app 帧就绪后自己接管时序。
 */
(function (root) {
  "use strict";

  var UI = root.UI;
  if (!UI) throw new Error("motor-page.js: 需先加载 /assets/ui-shell.js");

  var CFG = root.MOTOR_PAGE || {};
  var PAGE_ID = typeof CFG.id === "string" ? CFG.id : "";
  var SCEN = typeof CFG.scen === "number" ? CFG.scen : null;
  var SLOT_COUNT = typeof CFG.slots === "number" ? CFG.slots : 4;

  var LS_TARGET = "xrk.motorTarget";
  var DEFAULT_TARGET = "tcp:127.0.0.1:5560";
  var WAVE_MAX = 1200;          // 波形滚动窗口上限（≥600 帧口径）
  var LOG_MAX = 200;            // TEXT 日志行上限
  var CANVAS_H = 240;
  var RENDER_INTERVAL_MS = 33;  // DOM/波形刷新节拍（≈30Hz，与 ≤50Hz 的 mframe 展示流匹配）

  /* ─────────────────────────── 词典（motor.* 前缀） ─────────────────────────── */
  var DICT = {
    zh: {
      "motor.title.motor_pwm": "电机 PWM 阶梯",
      "motor.title.motor_servo": "电机舵机脉宽",
      "motor.title.motor_enc": "电机编码器闭环",
      "motor.title.motor_nested": "电机嵌套定时器",
      "motor.title.motor_dc": "电机直流换向",
      "motor.title.motor_scope": "电机示波器（四槽 demo）",
      "motor.title.motor_pidscan": "电机 PID 扫描（3×3）",

      "motor.target": "真实目标",
      "motor.targetPh": "tcp:127.0.0.1:5560 或 serial:COM5@115200",
      "motor.targetTip": "真实模式目标：tcp:宿主:端口（QEMU 串口桥）或 serial:COMx@波特率",

      "motor.wave.title": "波形（当前槽 setpoint / duty / actual · 等距时间轴 · 自动量程）",
      "motor.leg.sp": "sp 设定",
      "motor.leg.duty": "duty 占空",
      "motor.leg.actual": "actual 实际",
      "motor.legend": "窗口 {n} 帧 · 等距时间轴 · 自动量程",
      "motor.wave.empty": "尚无数据（先在真实/仿真模式下点「启动/连接」）",

      "motor.scen.line": "SCEN 场景 {scen} · 定时器快照 {n} 条",
      "motor.scen.expect": "期望 SCEN {scen} · 尚未收到 0x06 SCEN 帧",
      "motor.scen.wait": "本页用 0x05 遥测帧（无 0x06 场景头）",
      "motor.th.idx": "索引",
      "motor.th.ch": "通道",
      "motor.th.freq": "频率 Hz",

      "motor.slots.title": "槽位实时值（0x05 MOTOR 帧；点行切换波形槽）",
      "motor.th.slot": "槽位",
      "motor.th.sp": "sp",
      "motor.th.duty": "duty",
      "motor.th.actual": "actual",
      "motor.th.phase": "phase",
      "motor.th.fault": "fault",

      "motor.verify.title": "验收（verdict：PASS/FAIL 徽标 + 逐项断言 + 汇总）",
      "motor.th.res": "结果",
      "motor.th.item": "断言",
      "motor.th.detail": "详情",
      "motor.badge.pass": "PASS",
      "motor.badge.fail": "FAIL",
      "motor.verify.none": "尚未验收",
      "motor.btn.verify": "验收",
      "motor.sum": "帧 {frames} · 坏 CRC {bad} · 丢帧 {gaps} · TEXT {text}",

      "motor.log.title": "固件 TEXT 日志（最多 {n} 行）",
      "motor.stat": "源 {source} · 帧 {frames} · SCEN {scen}",
      "motor.source.none": "未启动",
      "motor.source.sim": "仿真",
      "motor.source.real": "真实",

      "motor.log.open": "请求数据源：{mode} {target}",
      "motor.log.openOk": "数据源已就绪：源 {source}，目标 {target}",
      "motor.log.openFail": "打开失败：{msg}",
      "motor.log.close": "已断开数据源",
      "motor.log.closeFail": "断开出错：{msg}",
      "motor.log.modeSwitch": "模式切到「{mode}」：已断开当前数据源，请再点「启动/连接」",
      "motor.log.badTarget": "目标格式不支持：{target}（应形如 tcp:127.0.0.1:5560 或 serial:COM5@115200）",
      "motor.log.verifyFail": "验收请求失败：{msg}",
      "motor.log.stateFail": "状态读取失败：{msg}",
      "motor.log.streamFail": "事件流订阅失败：{msg}",
      "motor.log.portsFail": "串口列表读取失败：{msg}",

      "motor.item.link.frames": "链路 · 帧数充足",
      "motor.item.link.bad_crc": "链路 · 坏 CRC 在限内",
      "motor.item.link.seq_gaps": "链路 · 丢帧率在限内",
      "motor.item.link.fault": "链路 · 无 fault 帧",
      "motor.item.link.layout": "链路 · 0x06 布局合法",
      "motor.item.link.timers": "链路 · 定时器快照与场景表一致",
      "motor.item.scurve.shape": "S 梯度 · 形状断言（单调 / 停驻 / 收敛 / 交替腿）",
      "motor.item.pwm.staircase": "PWM · 台阶形状断言（跟随 / 档内平稳 / 档序）",
      "motor.item.servo.calibration": "舵机 · 五点标定断言（脉宽 + 回读角）",
      "motor.item.enc.settling": "编码器 · 阶跃收敛断言",
      "motor.item.nested.isolation": "嵌套 · 通道隔离 + 双定时器断言",
      "motor.item.dc.direction": "DC · 方向位与正反对称断言",
      "motor.item.pidscan.g": "PID 扫描 · 组 {g}（settle / overshoot / steady）",
      "motor.item.pidscan.round": "PID 扫描 · 整轮跑完（round done）",
      "motor.item.pidscan.table": "PID 扫描 · 固件表 9 行 + 单一最优标记",
      "motor.item.pidscan.best": "PID 扫描 · 主机最优组 = 固件最优",
    },
    en: {
      "motor.title.motor_pwm": "Motor PWM staircase",
      "motor.title.motor_servo": "Motor servo pulse",
      "motor.title.motor_enc": "Motor encoder closed loop",
      "motor.title.motor_nested": "Motor nested timers",
      "motor.title.motor_dc": "Motor DC reversal",
      "motor.title.motor_scope": "Motor scope (four-slot demo)",
      "motor.title.motor_pidscan": "Motor PID scan (3x3)",

      "motor.target": "Real target",
      "motor.targetPh": "tcp:127.0.0.1:5560 or serial:COM5@115200",
      "motor.targetTip": "Real-mode target: tcp:host:port (QEMU serial bridge) or serial:COMx@baud",

      "motor.wave.title": "Waveform (active slot setpoint / duty / actual · evenly spaced time axis · auto range)",
      "motor.leg.sp": "sp setpoint",
      "motor.leg.duty": "duty",
      "motor.leg.actual": "actual",
      "motor.legend": "window {n} frames · evenly spaced time axis · auto range",
      "motor.wave.empty": "No data yet (pick Real/Sim, then press Start/Connect)",

      "motor.scen.line": "SCEN scenario {scen} · {n} timer snapshot(s)",
      "motor.scen.expect": "expects SCEN {scen} · no 0x06 SCEN frame yet",
      "motor.scen.wait": "This page uses 0x05 telemetry frames (no 0x06 scenario header)",
      "motor.th.idx": "idx",
      "motor.th.ch": "channels",
      "motor.th.freq": "freq Hz",

      "motor.slots.title": "Live slot values (0x05 MOTOR frames; click a row to switch the waveform slot)",
      "motor.th.slot": "Slot",
      "motor.th.sp": "sp",
      "motor.th.duty": "duty",
      "motor.th.actual": "actual",
      "motor.th.phase": "phase",
      "motor.th.fault": "fault",

      "motor.verify.title": "Verification (verdict: PASS/FAIL badge + per-item assertions + summary)",
      "motor.th.res": "Result",
      "motor.th.item": "Assertion",
      "motor.th.detail": "Detail",
      "motor.badge.pass": "PASS",
      "motor.badge.fail": "FAIL",
      "motor.verify.none": "not verified yet",
      "motor.btn.verify": "Verify",
      "motor.sum": "frames {frames} · bad CRC {bad} · seq gaps {gaps} · TEXT {text}",

      "motor.log.title": "Firmware TEXT log (up to {n} lines)",
      "motor.stat": "source {source} · frames {frames} · SCEN {scen}",
      "motor.source.none": "not started",
      "motor.source.sim": "sim",
      "motor.source.real": "real",

      "motor.log.open": "requesting data source: {mode} {target}",
      "motor.log.openOk": "data source ready: source {source}, target {target}",
      "motor.log.openFail": "open failed: {msg}",
      "motor.log.close": "data source closed",
      "motor.log.closeFail": "close failed: {msg}",
      "motor.log.modeSwitch": "mode switched to \"{mode}\": current data source closed, press Start/Connect again",
      "motor.log.badTarget": "unsupported target: {target} (expected tcp:127.0.0.1:5560 or serial:COM5@115200)",
      "motor.log.verifyFail": "verify request failed: {msg}",
      "motor.log.stateFail": "state read failed: {msg}",
      "motor.log.streamFail": "event stream subscribe failed: {msg}",
      "motor.log.portsFail": "port list read failed: {msg}",

      "motor.item.link.frames": "link · enough frames",
      "motor.item.link.bad_crc": "link · bad CRC within limit",
      "motor.item.link.seq_gaps": "link · sequence gap rate within limit",
      "motor.item.link.fault": "link · no fault frames",
      "motor.item.link.layout": "link · 0x06 layout valid",
      "motor.item.link.timers": "link · timer snapshot matches the scenario table",
      "motor.item.scurve.shape": "S-curve · shape assertions (monotonic / dwell / converge / alternating legs)",
      "motor.item.pwm.staircase": "PWM · staircase shape assertions (follow / flat / step order)",
      "motor.item.servo.calibration": "servo · five-point calibration assertions (pulse + readback angle)",
      "motor.item.enc.settling": "encoder · step settling assertions",
      "motor.item.nested.isolation": "nested · channel isolation + dual-timer assertions",
      "motor.item.dc.direction": "DC · direction bit and forward/reverse symmetry assertions",
      "motor.item.pidscan.g": "PID scan · group {g} (settle / overshoot / steady)",
      "motor.item.pidscan.round": "PID scan · full round finished (round done)",
      "motor.item.pidscan.table": "PID scan · firmware table 9 rows + single best mark",
      "motor.item.pidscan.best": "PID scan · host best group = firmware best",
    },
  };
  UI.define(DICT);

  var COLORS = { sp: "#4fc3f7", duty: "#ffb74d", actual: "#81c784",
                 grid: "#2c3947", dim: "#7c8b98" };

  var CSS =
    "body{background:#101418;color:#d8e2ea;font:13px/1.5 'Cascadia Mono',Consolas,monospace;" +
    "margin:0;padding:14px}" +
    "header{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:12px}" +
    "h1{font-size:15px;color:#4fc3f7;font-weight:600;margin:0}" +
    "input,button{background:#1a2129;color:#d8e2ea;border:1px solid #2c3947;border-radius:4px;" +
    "padding:5px 10px;font:inherit}" +
    "button{cursor:pointer}button:hover{border-color:#4fc3f7}" +
    "input#target{min-width:260px;cursor:text}" +
    "#status{color:#d8e2ea}#detail{color:#7c8b98}" +
    "main{display:grid;grid-template-columns:1fr 330px;gap:12px;align-items:start}" +
    "section{background:#1a2129;border:1px solid #2c3947;border-radius:6px;padding:10px;" +
    "margin-bottom:12px}" +
    "section h2{font-size:12px;color:#7c8b98;font-weight:normal;margin:0 0 8px;" +
    "letter-spacing:.08em}" +
    "canvas#wave{display:block;width:100%;height:240px;background:#0c1014;border-radius:4px}" +
    "#legend{display:flex;gap:14px;align-items:center;flex-wrap:wrap;font-size:12px;" +
    "color:#7c8b98;margin-top:6px}" +
    "#legend .sw{display:inline-block;width:10px;height:10px;margin-right:4px;" +
    "vertical-align:middle}" +
    "#scen-line{color:#7c8b98;margin:8px 0 4px}" +
    "table{border-collapse:collapse;width:100%;font-size:13px}" +
    "th,td{border:1px solid #2c3947;padding:3px 8px;text-align:right}" +
    "th{color:#7c8b98;font-weight:normal;text-align:left}" +
    "td:first-child,th:first-child{text-align:left}" +
    "tr.row{cursor:pointer}tr.row-on td{color:#4fc3f7}" +
    "#verify-head{display:flex;gap:10px;align-items:center;margin-bottom:8px}" +
    "#badge{font-weight:600;padding:2px 10px;border-radius:4px;background:#2c3947}" +
    "#badge.ok{background:#1b5e20;color:#a5d6a7}#badge.bad{background:#b71c1c;color:#ffcdd2}" +
    "#vsum{color:#7c8b98;font-size:12px}" +
    "#log{height:180px;overflow-y:auto;font-size:12px;color:#7c8b98;white-space:pre-wrap}" +
    "#log .fw{color:#ffb74d}";

  /* ─────────────────────────── DOM 搭建（薄壳只有 #app） ─────────────────────────── */
  var host = root.document && root.document.getElementById
    ? root.document.getElementById("app") : null;
  if (!host) throw new Error("motor-page.js: 薄壳缺宿主容器 <div id=\"app\"></div>");

  if (root.document.head && root.document.createElement) {
    var styleEl = root.document.createElement("style");
    styleEl.id = "motor-page-style";
    styleEl.textContent = CSS;
    root.document.head.appendChild(styleEl);
  }

  var el = {};
  function mk(tag, id, text) {
    var n = root.document.createElement(tag);
    if (id) { n.id = id; el[id] = n; }
    if (text !== undefined) n.textContent = text;
    if (tag === "button") n.type = "button";
    return n;
  }
  function clear(node) {
    while (node.children.length) node.removeChild(node.children[0]);
  }

  /* 头部：标题 + 模式分段（ui-shell 注入）+ 真实目标 + 启动/连接 + EN + 状态行 */
  var header = mk("header");
  var h1 = mk("h1", "title");
  var titleKey = "motor.title." + PAGE_ID;
  if (UI.has(titleKey)) h1.setAttribute("data-i18n", titleKey);
  else h1.textContent = CFG.title || PAGE_ID;
  var modebar = mk("span", "modebar");
  var targetLabel = mk("label", "target-label");
  targetLabel.setAttribute("data-i18n", "motor.target");
  var target = mk("input", "target");
  target.setAttribute("type", "text");
  target.setAttribute("list", "ports");
  target.setAttribute("data-i18n-ph", "motor.targetPh");
  target.setAttribute("data-i18n-title", "motor.targetTip");
  target.value = readTargetPref();
  var datalist = mk("datalist", "ports");
  var btn = mk("button", "btn");
  var btnLang = mk("button", "btn-lang");
  var status = mk("span", "status");
  var detail = mk("span", "detail");
  [h1, modebar, targetLabel, target, datalist, btn, btnLang, status, detail]
    .forEach(function (n) { header.appendChild(n); });

  /* 左列：波形（含 SCEN 面板） */
  var main = mk("main", "main");
  var left = mk("div", "left");
  var waveSec = mk("section", "wave-sec");
  var waveH2 = mk("h2");
  waveH2.setAttribute("data-i18n", "motor.wave.title");
  var wave = mk("canvas", "wave");
  wave.width = 900;
  wave.height = CANVAS_H;
  var legend = mk("div", "legend");
  var scenLine = mk("div", "scen-line");
  var timers = mk("table", "timers");
  [waveH2, wave, legend, scenLine, timers].forEach(function (n) { waveSec.appendChild(n); });
  left.appendChild(waveSec);

  /* 右列：槽位表 / 验收面板 / 日志 */
  var right = mk("div", "right");
  var slotSec = mk("section", "slot-sec");
  var slotH2 = mk("h2");
  slotH2.setAttribute("data-i18n", "motor.slots.title");
  var slotsTable = mk("table", "slots");
  [slotH2, slotsTable].forEach(function (n) { slotSec.appendChild(n); });

  var verifySec = mk("section", "verify-sec");
  var verifyH2 = mk("h2");
  verifyH2.setAttribute("data-i18n", "motor.verify.title");
  var verifyHead = mk("div", "verify-head");
  var badge = mk("span", "badge");
  badge.className = "badge";
  var vsum = mk("span", "vsum");
  var btnVerify = mk("button", "btn-verify");
  btnVerify.setAttribute("data-i18n", "motor.btn.verify");
  [badge, vsum, btnVerify].forEach(function (n) { verifyHead.appendChild(n); });
  var items = mk("table", "items");
  [verifyH2, verifyHead, items].forEach(function (n) { verifySec.appendChild(n); });

  var logSec = mk("section", "log-sec");
  var logH2 = mk("h2");
  /* 日志标题带 {n} 占位：applyStatic 不做插值，故动态渲染（renderLogTitle） */
  var logBox = mk("div", "log", "");
  [logH2, logBox].forEach(function (n) { logSec.appendChild(n); });

  [slotSec, verifySec, logSec].forEach(function (n) { right.appendChild(n); });
  main.appendChild(left);
  main.appendChild(right);
  host.appendChild(header);
  host.appendChild(main);

  /* ─────────────────────────── 状态 ─────────────────────────── */
  var st = {
    source: "none",      // none|sim|real（/api/state）
    target: "",
    connected: false,
    frames: 0,           // 单调：SSE mframe 累计，/api/state 绝对值取大
    scen: null,          // 0x06 帧里的 scen_id
    timers: [],
    slots: [],           // 槽位最新一条记录
    points: 0,           // 累计入窗点数（全部槽）
    verdict: null,
    log: [],
  };
  /* 行号 ↔ 帧 slot 的映射：多槽页 = 一行一槽；单槽页（pidscan）所有组的帧都进同一行
     （帧 slot = 扫描组号 0..8，行内「槽位」列照原值显示当前组）。 */
  function rowOf(slot) { return SLOT_COUNT === 1 ? 0 : slot; }
  var history = [];      // history[row] = [{sp,du,ac}]（滚动窗口 WAVE_MAX）
  var activeSlot = 0;
  var lastTimersKey = null;
  var scheduled = false;
  var dirty = { slots: true, wave: true, scen: true, verdict: true, stat: true };

  /* ─────────────────────────── 小工具 ─────────────────────────── */
  var memTarget = null;
  function readTargetPref() {
    var v = null;
    try { v = root.localStorage ? root.localStorage.getItem(LS_TARGET) : null; } catch (e) { v = null; }
    if (typeof v !== "string" || !v) v = memTarget;
    return typeof v === "string" && v ? v : DEFAULT_TARGET;
  }
  function writeTargetPref(v) {
    memTarget = v;
    try {
      if (root.localStorage && typeof root.localStorage.setItem === "function") {
        root.localStorage.setItem(LS_TARGET, v);
      }
    } catch (e) { /* 隐私模式：只留内存 */ }
  }
  function targetValid(t) {
    return /^tcp:\S+$/.test(t) || /^serial:\S+$/.test(t);
  }
  function num(v) { return (typeof v === "number" && isFinite(v)) ? v : 0; }
  function num0(v) { return (typeof v === "number" && isFinite(v)) ? String(v) : "—"; }
  function fixed(v) { return (typeof v === "number" && isFinite(v)) ? v.toFixed(3) : "—"; }

  function log(msg, warn) {
    st.log.push(msg);
    while (st.log.length > LOG_MAX) st.log.shift();
    var line = root.document.createElement("div");
    line.className = warn ? "ln fw" : "ln";
    line.textContent = msg;
    logBox.appendChild(line);
    while (logBox.children.length > LOG_MAX) logBox.removeChild(logBox.children[0]);
    logBox.scrollTop = logBox.scrollHeight;
  }

  function postJSON(url, body) {
    return root.fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    }).then(function (res) {
      return res.json().catch(function () { return {}; }).then(function (data) {
        if (!res.ok) {
          var err = new Error((data && data.error) || ("HTTP " + res.status));
          err.status = res.status;
          throw err;
        }
        return data;
      });
    });
  }
  function getJSON(url) {
    return root.fetch(url, { cache: "no-store" }).then(function (res) { return res.json(); });
  }

  /* ─────────────────────────── 渲染（脏位合并，≤RENDER_INTERVAL_MS 一次） ───────────────────────────
     用 setTimeout 而不是 requestAnimationFrame：rAF 在不可见页面（offscreen
     QWebEngine / 后台标签页 / 隐藏窗口）被挂起，DOM 会停在首帧——页面在那种
     环境下也要能更新（集成测试就靠它）。50Hz 数据用 33ms 节拍足够。 */
  function schedule() {
    if (scheduled) return;
    scheduled = true;
    root.setTimeout(flush, RENDER_INTERVAL_MS);
  }
  function flush() {
    scheduled = false;
    if (dirty.slots) { renderSlots(); dirty.slots = false; }
    if (dirty.wave) { drawWave(); dirty.wave = false; }
    if (dirty.scen) { renderScen(false); dirty.scen = false; }
    if (dirty.verdict) { renderVerdict(); dirty.verdict = false; }
    if (dirty.stat) { renderStat(); dirty.stat = false; }
  }

  function fillTable(table, headers, rows) {
    clear(table);
    var thead = root.document.createElement("thead");
    var hrow = root.document.createElement("tr");
    headers.forEach(function (txt) {
      var th = root.document.createElement("th");
      th.textContent = txt;
      hrow.appendChild(th);
    });
    thead.appendChild(hrow);
    table.appendChild(thead);
    var tbody = root.document.createElement("tbody");
    rows.forEach(function (cells) {
      var tr = root.document.createElement("tr");
      cells.forEach(function (txt) {
        var td = root.document.createElement("td");
        td.textContent = txt;
        tr.appendChild(td);
      });
      tbody.appendChild(tr);
    });
    table.appendChild(tbody);
  }

  /* 槽位表：行数固定（槽位数来自薄壳），只刷单元格 → 高帧率下不重建 DOM */
  var slotRows = [];
  function buildSlotsTable() {
    clear(slotsTable);
    var thead = root.document.createElement("thead");
    var hrow = root.document.createElement("tr");
    ["motor.th.slot", "motor.th.sp", "motor.th.duty", "motor.th.actual",
     "motor.th.phase", "motor.th.fault"].forEach(function (k) {
      var th = root.document.createElement("th");
      th.textContent = UI.t(k);
      hrow.appendChild(th);
    });
    thead.appendChild(hrow);
    slotsTable.appendChild(thead);
    var tbody = root.document.createElement("tbody");
    slotRows = [];
    for (var i = 0; i < SLOT_COUNT; i++) {
      var tr = root.document.createElement("tr");
      tr.className = "row" + (i === activeSlot ? " row-on" : "");
      var cells = [];
      for (var c = 0; c < 6; c++) {
        var td = root.document.createElement("td");
        td.textContent = c === 0 ? String(i) : "—";
        tr.appendChild(td);
        cells.push(td);
      }
      (function (idx) { tr.addEventListener("click", function () { setActiveSlot(idx); }); })(i);
      tbody.appendChild(tr);
      slotRows.push({ tr: tr, cells: cells });
    }
    slotsTable.appendChild(tbody);
  }
  function renderSlots() {
    for (var i = 0; i < slotRows.length; i++) {
      var rec = st.slots[i];
      var cells = slotRows[i].cells;
      /* 「槽位」列照帧原值（pidscan 单槽页 = 当前扫描组号；多槽页 = 槽序号） */
      cells[0].textContent = rec && typeof rec.slot === "number" ? String(rec.slot) : String(i);
      cells[1].textContent = rec ? fixed(rec.sp) : "—";
      cells[2].textContent = rec ? fixed(rec.duty) : "—";
      cells[3].textContent = rec ? fixed(rec.actual) : "—";
      cells[4].textContent = rec && typeof rec.phase === "number" ? String(rec.phase) : "—";
      cells[5].textContent = rec && typeof rec.fault === "number" ? String(rec.fault) : "—";
      slotRows[i].tr.className = "row" + (i === activeSlot ? " row-on" : "");
    }
  }
  function setActiveSlot(i) {
    if (i < 0 || i >= SLOT_COUNT || i === activeSlot) return;
    activeSlot = i;
    dirty.slots = true;
    dirty.wave = true;
    schedule();
  }

  function renderLegend() {
    clear(legend);
    [["motor.leg.sp", COLORS.sp], ["motor.leg.duty", COLORS.duty],
     ["motor.leg.actual", COLORS.actual]].forEach(function (it) {
      var sw = root.document.createElement("span");
      sw.className = "sw";
      sw.style.background = it[1];
      var tx = root.document.createElement("span");
      tx.textContent = UI.t(it[0]);
      legend.appendChild(sw);
      legend.appendChild(tx);
    });
    var win = root.document.createElement("span");
    win.textContent = UI.t("motor.legend", { n: WAVE_MAX });
    legend.appendChild(win);
  }

  function drawSeries(ctx, hist, key, color, px, py, n) {
    ctx.strokeStyle = color;
    ctx.beginPath();
    for (var i = 0; i < n; i++) {
      var y = py(hist[i][key]);
      if (i === 0) ctx.moveTo(px(0), y);
      else ctx.lineTo(px(i), y);
    }
    ctx.stroke();
  }

  function drawWave() {
    if (!wave || typeof wave.getContext !== "function") return;
    var ctx = wave.getContext("2d");
    if (!ctx) return;
    var w = wave.width || 900;
    var h = wave.height || CANVAS_H;
    ctx.clearRect(0, 0, w, h);
    var hist = history[activeSlot] || [];
    var n = hist.length;
    var padL = 10, padR = 10, padT = 16, padB = 16;
    if (!n) {
      ctx.fillStyle = COLORS.dim;
      ctx.font = "12px monospace";
      ctx.fillText(UI.t("motor.wave.empty"), padL, Math.round(h / 2));
      return;
    }
    var lo = Infinity, hi = -Infinity, i, p;
    for (i = 0; i < n; i++) {
      p = hist[i];
      lo = Math.min(lo, p.sp, p.du, p.ac);
      hi = Math.max(hi, p.sp, p.du, p.ac);
    }
    if (!isFinite(lo) || !isFinite(hi)) return;
    if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
    var span = hi - lo;
    function px(idx) { return n <= 1 ? padL : padL + (w - padL - padR) * idx / (n - 1); }
    function py(v) { return padT + (h - padT - padB) * (hi - v) / span; }

    ctx.strokeStyle = COLORS.grid;
    ctx.beginPath();
    ctx.moveTo(padL, py(lo));
    ctx.lineTo(w - padR, py(lo));
    ctx.stroke();
    ctx.fillStyle = COLORS.dim;
    ctx.font = "11px monospace";
    ctx.fillText(hi.toFixed(3), padL + 2, padT - 4);
    ctx.fillText(lo.toFixed(3), padL + 2, h - 4);
    ctx.fillText(String(n), Math.max(padL, w - padR - 40), padT - 4);

    drawSeries(ctx, hist, "sp", COLORS.sp, px, py, n);
    drawSeries(ctx, hist, "du", COLORS.duty, px, py, n);
    drawSeries(ctx, hist, "ac", COLORS.actual, px, py, n);
  }

  function renderScen(force) {
    if (st.scen !== null) {
      scenLine.textContent = UI.t("motor.scen.line",
        { scen: st.scen, n: st.timers.length });
    } else if (SCEN !== null) {
      scenLine.textContent = UI.t("motor.scen.expect", { scen: SCEN });
    } else {
      scenLine.textContent = UI.t("motor.scen.wait");
    }
    var key = JSON.stringify(st.timers);
    if (!force && key === lastTimersKey) return;
    lastTimersKey = key;
    var rows = st.timers.map(function (t) {
      return [String(t[0]), String(t[1]), String(t[2])];
    });
    fillTable(timers,
      [UI.t("motor.th.idx"), UI.t("motor.th.ch"), UI.t("motor.th.freq")], rows);
  }

  function itemLabel(key) {
    var raw = String(key == null ? "" : key);
    var k = "motor.item." + raw;
    if (UI.has(k)) return UI.t(k);
    /* pidscan 的断言项按组号编号（pidscan.g0..g8）：组号是数据驱动，用模板键 */
    var m = /^pidscan\.g(\d+)$/.exec(raw);
    if (m) return UI.t("motor.item.pidscan.g", { g: m[1] });
    return raw;                      // 未知 key 原样显示（后端新增项不阻塞页面）
  }

  function renderVerdict() {
    var v = st.verdict;
    var headers = [UI.t("motor.th.res"), UI.t("motor.th.item"), UI.t("motor.th.detail")];
    if (!v || typeof v !== "object") {
      badge.textContent = UI.t("motor.verify.none");
      badge.className = "badge";
      vsum.textContent = "";
      fillTable(items, headers, []);
      return;
    }
    var pass = !!v.pass;
    badge.textContent = UI.t(pass ? "motor.badge.pass" : "motor.badge.fail");
    badge.className = "badge " + (pass ? "ok" : "bad");
    var sum = v.summary || {};
    vsum.textContent = UI.t("motor.sum", {
      frames: num0(sum.frames), bad: num0(sum.bad_crc),
      gaps: num0(sum.seq_gaps), text: num0(sum.text),
    });
    var rows = (Array.isArray(v.items) ? v.items : []).map(function (it) {
      return [it && it.ok ? "✓" : "✗",
              itemLabel(it && it.key),
              it && it.detail != null ? String(it.detail) : ""];
    });
    fillTable(items, headers, rows);
  }

  function renderLogTitle() {
    logH2.textContent = UI.t("motor.log.title", { n: LOG_MAX });
  }

  function sourceName(src) {
    return UI.t(src === "sim" || src === "real" ? "motor.source." + src : "motor.source.none");
  }
  function renderStat() {
    detail.textContent = UI.t("motor.stat", {
      source: sourceName(st.source),
      frames: st.frames,
      scen: st.scen === null ? "—" : st.scen,
    });
  }

  function syncTarget() {
    var real = UI.mode === "real";
    target.hidden = !real;
    targetLabel.hidden = !real;
  }

  /* ─────────────────────────── 服务器事件 ─────────────────────────── */
  function parseEvent(ev) {
    try { return JSON.parse(ev && ev.data ? ev.data : "{}"); } catch (e) { return null; }
  }

  function setConnected(on) {
    st.connected = !!on;
    UI.setConnected(st.connected);
    dirty.stat = true;
    schedule();
  }

  function onState(s) {
    if (!s || typeof s !== "object") return;
    if (typeof s.source === "string") st.source = s.source;
    if (typeof s.target === "string") st.target = s.target;
    if (typeof s.frames === "number" && s.frames > st.frames) st.frames = s.frames;
    if (typeof s.connected === "boolean") setConnected(s.connected);
    dirty.stat = true;
    schedule();
  }

  function onMframe(d) {
    if (!d || typeof d !== "object") return;
    st.frames++;
    if (d.type === 6) {
      st.scen = typeof d.scen === "number" ? d.scen : st.scen;
      if (Array.isArray(d.timers)) {
        st.timers = d.timers;
        dirty.scen = true;
      }
    }
    var recs = Array.isArray(d.recs) ? d.recs : [];
    for (var i = 0; i < recs.length; i++) {
      var r = recs[i] || {};
      var row = rowOf(typeof r.slot === "number" ? r.slot : i);
      if (row < 0 || row >= SLOT_COUNT) continue;
      st.slots[row] = r;
      var hist = history[row] || (history[row] = []);
      hist.push({ sp: num(r.sp), du: num(r.duty), ac: num(r.actual) });
      if (hist.length > WAVE_MAX) hist.shift();
      st.points++;
    }
    if (recs.length) {
      dirty.slots = true;
      dirty.wave = true;
      dirty.stat = true;
      schedule();
    }
  }

  function onVerdict(v) {
    if (!v || typeof v !== "object") return;
    st.verdict = v;
    dirty.verdict = true;
    schedule();
  }

  /* ─────────────────────────── 显式动作（用户点了才发 POST） ─────────────────────────── */
  function openSource(body) {
    log(UI.t("motor.log.open", { mode: body.mode, target: body.target || "—" }));
    return postJSON("/api/open", body).then(function (state) {
      onState(state);
      log(UI.t("motor.log.openOk", {
        source: sourceName(st.source),
        target: (state && state.target) || body.target || "—",
      }));
    }, function (err) {
      log(UI.t("motor.log.openFail", { msg: err.message }), true);
      st.source = "none";
      setConnected(false);         // 状态行归壳：失败即回「未连接」
    });
  }

  function closeSource() {
    return postJSON("/api/close", {}).then(function (state) {
      onState(state);
      log(UI.t("motor.log.close"));
    }, function (err) {
      log(UI.t("motor.log.closeFail", { msg: err.message }), true);
      setConnected(false);
    });
  }

  function onPrimary() {
    if (st.connected) { closeSource(); return; }
    if (UI.mode === "sim") { openSource({ mode: "sim" }); return; }
    var t = String(target.value || "").trim();
    if (!targetValid(t)) {
      log(UI.t("motor.log.badTarget", { target: t || "—" }), true);
      return;
    }
    writeTargetPref(t);
    openSource({ mode: "real", target: t });
  }

  function onVerify() {
    postJSON("/api/verify", {}).then(onVerdict, function (err) {
      log(UI.t("motor.log.verifyFail", { msg: err.message }), true);
    });
  }

  function refreshState() {
    return getJSON("/api/state").then(onState, function (err) {
      log(UI.t("motor.log.stateFail", { msg: err.message }), true);
    });
  }

  function refreshPorts() {
    return getJSON("/api/ports").then(function (d) {
      var ports = (d && Array.isArray(d.ports)) ? d.ports : [];
      ports.forEach(function (p) {
        var opt = root.document.createElement("option");
        opt.value = p.device;
        opt.textContent = p.desc ? p.device + " — " + p.desc : p.device;
        datalist.appendChild(opt);
      });
    }, function (err) {
      log(UI.t("motor.log.portsFail", { msg: err.message }), true);
    });
  }

  var stream = null;
  function openStream() {
    if (typeof root.EventSource !== "function") return null;
    try {
      stream = new root.EventSource("/api/stream");
    } catch (e) {
      log(UI.t("motor.log.streamFail", { msg: e.message }), true);
      return null;
    }
    stream.addEventListener("mframe", function (ev) { onMframe(parseEvent(ev)); });
    stream.addEventListener("text", function (ev) {
      var d = parseEvent(ev);
      if (d && d.line != null) log(String(d.line));
    });
    stream.addEventListener("verdict", function (ev) { onVerdict(parseEvent(ev)); });
    stream.addEventListener("state", function (ev) { onState(parseEvent(ev)); });
    stream.addEventListener("port", function (ev) {
      /* 中枢内部事件是 bool；HTTP 契约是 {"connected":bool}——两种都收 */
      var d = parseEvent(ev);
      var on = typeof d === "boolean" ? d
        : (d && typeof d.connected === "boolean" ? d.connected : null);
      if (on === null) return;
      setConnected(on);
      if (!on) st.source = "none";
    });
    return stream;
  }

  /* ─────────────────────────── 装配 ─────────────────────────── */
  buildSlotsTable();
  renderLegend();
  renderLogTitle();
  syncTarget();
  UI.init({ page: PAGE_ID });            // 只渲染壳；绝不启动数据源

  btn.onclick = onPrimary;
  btnVerify.onclick = onVerify;
  target.addEventListener("change", function () {
    var t = String(target.value || "").trim();
    if (t) writeTargetPref(t);
  });
  /* 画布后端尺寸跟随布局宽度（CSS 宽度 100%；无布局信息时用 900 宽） */
  function fitWave() {
    var w = wave.clientWidth || 900;
    if (wave.width !== w) { wave.width = w; wave.height = CANVAS_H; }
    dirty.wave = true;
    schedule();
  }
  if (typeof root.addEventListener === "function") root.addEventListener("resize", fitWave);
  fitWave();

  UI.onModeChange(function (mode) {
    if (st.connected) closeSource();
    log(UI.t("motor.log.modeSwitch", { mode: UI.t("mode." + mode) }));
    syncTarget();
  });
  UI.onLangChange(function () {
    buildSlotsTable();
    renderSlots();                 // 重建后立刻回填（不等下一帧）
    renderLegend();
    renderLogTitle();
    renderScen(true);
    renderVerdict();
    renderStat();
    drawWave();
  });

  refreshState();
  refreshPorts();
  openStream();                          // 只订阅流，不启动数据源
  refreshAll();

  function refreshAll() {
    dirty.slots = dirty.wave = dirty.scen = dirty.verdict = dirty.stat = true;
    schedule();
  }

  /* ─────────────────────────── headless 探针（harness 用） ─────────────────────────── */
  var api = {
    cfg: { id: PAGE_ID, scen: SCEN, slots: SLOT_COUNT, title: CFG.title || "" },
    els: el,
    state: st,
    inspect: function () {
      return {
        pid: PAGE_ID, mode: UI.mode, lang: UI.lang, connected: st.connected,
        frames: st.frames, points: st.points, activeSlot: activeSlot,
        wavePoints: (history[activeSlot] || []).length,
        logLines: st.log.length, verdict: st.verdict, target: st.target,
      };
    },
  };

  root.MotorPage = api;
  if (typeof module !== "undefined" && module.exports) module.exports = api;
})(typeof globalThis !== "undefined" ? globalThis : this);
