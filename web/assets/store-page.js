/* store-page.js — store_demo 页（薄壳 #app 自建 DOM；ui-shell.js + live.js 之上）。
 *
 * 数据面（服务器架构）：
 *   · SSE  /api/stream —— TEXT 事件（上电 boot 行 / 命令回执行）
 *   · POST /api/store/cmd —— MOD_STORE=0x20 命令：
 *        get    → ACK data 22B（boot/gain/flags/note/load_state/frame_len/dirty/kb）
 *        set    → 单键落盘（RESET 语义 = set boot=0，本页映射为独立按钮）
 *        status → ACK data 6B（load_state/frame_len/dirty/entries/kb）
 *   协议表唯一对照点 = 固件 proto/include/proto/frame.h 的 PROTO_STORE_FUNC_*。
 *
 * 显式模式纪律：加载不启动任何数据源；「仿真 | 真实 → 启动/连接」由壳按钮触发，
 * 命令按钮未连接时仍可点（回「未连接」诚实反馈，不禁用）。
 */
(function (root) {
  "use strict";

  var UI = root.UI;
  if (!UI || !UI.define || !UI.init) {
    throw new Error("store-page.js: 需要 ui-shell.js 先加载");
  }
  var PAGE = root.STORE_PAGE || {id: "store_demo", title: "store_demo"};

  /* ── 词典（zh/en 成对；ui_shell_harness 会查键对齐）── */
  UI.define({
    zh: {
      "store.intro": "pstore 断电续跑演示：上电自增 boot 计数并落盘；断电再上电，计数在上次基础上续跑（load_state=1 恢复）。ERASE 介质级清空本版未实现（pstore 缺 API），“恢复出厂” = SET boot=0。",
      "store.get": "GET 快照",
      "store.reset": "RESET（SET boot=0）",
      "store.status": "STATUS",
      "store.snapshot": "快照（GET）",
      "store.statusrow": "状态行（STATUS）",
      "store.log": "TEXT 日志（上电行 / 命令回执）",
      "store.nosnap": "（尚未请求 GET）",
      "store.cmdSent": "→ 已下发 {name}",
      "store.cmdOk": "[host] {name} 完成（status=0）",
      "store.cmdFail": "[host] {name} 失败：{msg}",
      "store.boot": "boot 计数",
      "store.gain": "gain",
      "store.flags": "flags",
      "store.note": "note",
      "store.load": "load_state",
      "store.restored": "1=断电恢复",
      "store.fresh": "0=出厂空白",
      "store.len": "frame_len",
      "store.dirty": "dirty",
      "store.kb": "capacity_kb",
      "store.entries": "entries",
      "store.target": "目标",
      "store.targetPh": "tcp:127.0.0.1:5558 或 serial:COMx@115200",
      "store.targetTip": "真实模式连接目标（QEMU / 设备串口桥）；仿真模式无需填写",
    },
    en: {
      "store.intro": "pstore power-cycle resume: boot counter increments and saves on power-on; after power-loss + reboot the counter resumes (load_state=1 = restored). Media-level ERASE is not implemented (pstore lacks the API); \"factory reset\" here = SET boot=0.",
      "store.get": "GET snapshot",
      "store.reset": "RESET (SET boot=0)",
      "store.status": "STATUS",
      "store.snapshot": "Snapshot (GET)",
      "store.statusrow": "Status row (STATUS)",
      "store.log": "TEXT log (boot line / ack lines)",
      "store.nosnap": "(GET not requested yet)",
      "store.cmdSent": "→ sent {name}",
      "store.cmdOk": "[host] {name} done (status=0)",
      "store.cmdFail": "[host] {name} failed: {msg}",
      "store.boot": "boot count",
      "store.gain": "gain",
      "store.flags": "flags",
      "store.note": "note",
      "store.load": "load_state",
      "store.restored": "1=restored after power cycle",
      "store.fresh": "0=factory blank",
      "store.len": "frame_len",
      "store.dirty": "dirty",
      "store.kb": "capacity_kb",
      "store.entries": "entries",
      "store.target": "Target",
      "store.targetPh": "tcp:127.0.0.1:5558 or serial:COMx@115200",
      "store.targetTip": "Real-mode connect target (QEMU / device serial bridge); not needed in sim mode",
    },
  });

  /* ── DOM 搭建（薄壳只有 #app；锚点 id 与 ui-shell 契约一致）── */
  var host = root.document && root.document.getElementById
    ? root.document.getElementById("app") : null;
  if (!host) throw new Error("store-page.js: 薄壳缺宿主容器 <div id=\"app\"></div>");

  if (root.document.head && root.document.createElement) {
    var styleEl = root.document.createElement("style");
    styleEl.id = "store-page-style";
    styleEl.textContent =
      "body{font-family:system-ui,'Segoe UI',sans-serif;background:#0f1926;color:#d8e2ef;" +
      "margin:0}#app{max-width:1080px;margin:0 auto;padding:16px}" +
      "header{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:8px}" +
      "h1{font-size:18px;margin:0 8px 0 0}" +
      "#target{flex:1;min-width:240px;background:#152238;border:1px solid #2c3947;" +
      "border-radius:4px;color:#d8e2ef;padding:5px 8px}" +
      "#btn,#btn-lang,#modebar button,button.cmd{background:#152238;color:#d8e2ef;" +
      "border:1px solid #2c3947;border-radius:4px;padding:5px 12px;cursor:pointer}" +
      "#btn-lang{min-width:52px}button.cmd{background:#103a2f;border-color:#1f6f5a}" +
      "button.cmd:hover{filter:brightness(1.25)}" +
      "#status{color:#8be9a8;font-size:13px}" +
      "#status:not(.on){color:#f38ba8}" +
      ".panel{border:1px solid #2c3947;border-radius:6px;padding:10px 14px;margin:10px 0}" +
      ".panel h2{font-size:14px;margin:0 0 8px;color:#7eb8f0}" +
      "#snap{display:grid;grid-template-columns:max-content 1fr;gap:4px 18px;" +
      "font-size:13px}.snap-k{color:#7eb8f0}#snap .snap-note{color:#ffd28a}" +
      "#row{border:1px solid #2c3947;border-radius:4px;padding:6px 10px;font-size:13px;" +
      "min-height:20px;font-family:ui-monospace,Consolas,monospace}" +
      "#log{height:150px;overflow-y:auto;font-size:12px;color:#9fb4c9;" +
      "white-space:pre-wrap;font-family:ui-monospace,Consolas,monospace}";
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

  /* 头部：标题 + 模式分段 + 真实目标 + 启动/连接 + EN + 状态 */
  var header = mk("header");
  header.appendChild(mk("h1", "title", PAGE.title));
  header.appendChild(mk("span", "modebar"));
  var target = mk("input", "target");
  target.setAttribute("type", "text");
  target.setAttribute("data-i18n-ph", "store.targetPh");
  target.setAttribute("data-i18n-title", "store.targetTip");
  header.appendChild(target);
  header.appendChild(mk("button", "btn"));
  header.appendChild(mk("button", "btn-lang"));
  var status = mk("span", "status");
  header.appendChild(status);

  /* 命令行（三按钮：GET / RESET（SET boot=0）/ STATUS） */
  var cmdRow = mk("div");
  var bGet = mk("button", "btn-get", UI.t ? "" : "");
  bGet.className = "cmd";
  bGet.setAttribute("data-i18n", "store.get");
  var bReset = mk("button", "btn-reset", "");
  bReset.className = "cmd";
  bReset.setAttribute("data-i18n", "store.reset");
  var bStatus = mk("button", "btn-status", "");
  bStatus.className = "cmd";
  bStatus.setAttribute("data-i18n", "store.status");
  [bGet, bReset, bStatus].forEach(function (n) { cmdRow.appendChild(n); });

  /* 快照面板（GET 22B） */
  var snapSec = mk("section", "snap-sec");
  snapSec.className = "panel";
  var snapH2 = mk("h2");
  snapH2.setAttribute("data-i18n", "store.snapshot");
  var snap = mk("div", "snap");
  snap.textContent = "-";   // 初始占位（UI.apply 不动它）
  snapSec.appendChild(snapH2);
  snapSec.appendChild(snap);

  /* 状态行面板（STATUS 6B） */
  var rowSec = mk("section", "row-sec");
  rowSec.className = "panel";
  var rowH2 = mk("h2");
  rowH2.setAttribute("data-i18n", "store.statusrow");
  var row = mk("div", "row");
  row.textContent = "-";
  rowSec.appendChild(rowH2);
  rowSec.appendChild(row);

  /* 日志面板 */
  var logSec = mk("section", "log-sec");
  logSec.className = "panel";
  var logH2 = mk("h2");
  logH2.setAttribute("data-i18n", "store.log");
  var log = mk("div", "log");
  logSec.appendChild(logH2);
  logSec.appendChild(log);

  [header, cmdRow, snapSec, rowSec, logSec].forEach(function (n) { host.appendChild(n); });
  UI.init({page: PAGE.id});
  UI.apply();

  var bridge = LiveBridge();
  bridge.portChanged.connect(function (on) {
    UI.setConnected(on);
    status.className = on ? "on" : "";
  });
  bridge.textReady.connect(function (msg) {
    var pre = root.document.createElement("div");
    pre.textContent = "[txt] " + msg;
    log.appendChild(pre);
    log.scrollTop = log.scrollHeight;
  });
  bridge.statReady.connect(function (imu, text, drop) { /* store 无 IMU 计数，忽略 */ });

  function storeCmd(func, args) {
    return fetch("/api/store/cmd", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({func: func, args: args || {}}),
    }).then(function (res) {
      return res.json().catch(function () { return {}; }).then(function (data) {
        if (!res.ok) throw new Error((data && data.error) || ("HTTP " + res.status));
        return data;
      });
    });
  }

  function snapLine(name) { return UI.t("store.cmdSent", {name: name}); }
  var CMD_FLOW = {
    get: {
      label: "GET",
      send: function () {
        return storeCmd("get");
      },
      onOk: function (p) {
        if (!p || typeof p.boot !== "number") return;
        snap.textContent =
          UI.t("store.boot") + "=" + p.boot + "  ·  " +
          UI.t("store.gain") + "=" + p.gain + "  ·  " +
          UI.t("store.flags") + "=0x" + p.flags.toString(16).padStart(2, "0") + "  ·  " +
          UI.t("store.note") + "=“" + noteStr(p.note) + "”  ·  " +
          UI.t("store.load") + "=" + p.load_state +
          (p.load_state === 1 ? " (" + UI.t("store.restored") + ")" : " (" + UI.t("store.fresh") + ")") +
          "  ·  " + UI.t("store.len") + "=" + p.frame_len +
          "  ·  " + UI.t("store.dirty") + "=" + p.dirty +
          "  ·  " + UI.t("store.kb") + "=" + p.capacity_kb + "KiB";
      },
    },
    reset: {
      label: "RESET",
      send: function () { return storeCmd("set", {kv: {boot: 0}}); },
      onOk: function () { row.textContent = ""; },
    },
    status: {
      label: "STATUS",
      send: function () { return storeCmd("status"); },
      onOk: function (p) {
        if (!p || typeof p.load_state !== "number") return;
        row.textContent =
          UI.t("store.load") + "=" + p.load_state +
          "  ·  " + UI.t("store.len") + "=" + p.frame_len +
          "  ·  " + UI.t("store.dirty") + "=" + p.dirty +
          "  ·  " + UI.t("store.entries") + "=" + p.entries +
          "  ·  " + UI.t("store.kb") + "=" + p.capacity_kb + "KiB";
      },
    },
  };

  function noteStr(raw) {
    if (!raw) return "";
    var s = "";
    for (var i = 0; i < raw.length; i++) {
      if (raw[i] === 0) break;
      s += String.fromCharCode(raw[i]);
    }
    return s;
  }

  function fire(name) {
    var flow = CMD_FLOW[name];
    if (!flow) return;
    logUI(UI.t("store.cmdSent", {name: flow.label}));
    flow.send().then(function (r) {
      if (r.status !== 0) {
        logUI(UI.t("store.cmdFail", {name: flow.label, msg: "status=" + r.status}));
        return;
      }
      if (flow.onOk) flow.onOk(r.payload);
      logUI(UI.t("store.cmdOk", {name: flow.label}));
    }).catch(function (e) {
      logUI(UI.t("store.cmdFail", {name: flow.label, msg: e.message}));
    });
  }

  function logUI(msg) {
    var pre = root.document.createElement("div");
    pre.textContent = msg;
    log.appendChild(pre);
    log.scrollTop = log.scrollHeight;
  }

  bGet.addEventListener("click", function () { fire("get"); });
  bReset.addEventListener("click", function () { fire("reset"); });
  bStatus.addEventListener("click", function () { fire("status"); });
  bGet.textContent = UI.t("store.get");
  bReset.textContent = UI.t("store.reset");
  bStatus.textContent = UI.t("store.status");

  /* 启动/连接按钮（壳渲染，点击处理归页面：按当前模式走显式开档） */
  var btn = byId("btn");
  btn.addEventListener("click", function () {
    if (UI.connected) {
      bridge.closePort().then(function (s) { if (s) UI.setConnected(false); });
      return;
    }
    if (UI.mode === "sim") {
      bridge.openSim();
    } else {
      var t = target.value.trim();
      if (!t) {
        logUI(UI.t("store.cmdFail", {name: "open", msg: "真实模式需要目标" +
               "（tcp:host:port 或 serial:COMx@115200）"}));
        return;
      }
      bridge.openPort(t);
    }
  });

  function byId(id) {
    return root.document.getElementById(id);
  }

  UI.onModeChange(function (mode) {
    // 切模式时已连接的数据源由页面负责关掉（壳契约）；store 页就地关闭
    if (bridge.connected) bridge.closePort();
  });

  /* 兼容旧 kick（app.py --mode 复用）：显式走同一条路径 */
  root.startSim = function () {
    UI.setMode("sim");
    bridge.openSim();
  };
  root.autoConnect = function (port) {
    UI.setMode("real");
    target.value = port;
    bridge.openPort(port);
  };

  var api = {
    _ui: UI,
    _bridge: bridge,
    _fire: fire,
    _el: el,
    _snapLine: snapLine,
  };
  root.STORE_PAGE_API = api;
  if (typeof module !== "undefined" && module.exports) module.exports = api;
})(typeof globalThis !== "undefined" ? globalThis : this);