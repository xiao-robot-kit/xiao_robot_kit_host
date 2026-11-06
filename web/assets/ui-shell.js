/* ui-shell.js — 上位机统一外壳：数据源模式开关（仿真/真实）+ 中英双语（zh/en）。
 * 零第三方、零构建：页面在自身内联脚本之前 <script src="../assets/ui-shell.js">。
 *
 * 设计约束（用户定案，2026-11-05）：
 *   1) 数据源只有两个**显式**档：sim = 仿真（本机合成源，不碰设备/QEMU）、
 *      real = 真实（设备串口 / 外部桥，需用户先就绪并点「连接」）。
 *      **禁止隐式回落**——页壳 init() 绝不启动任何数据源：没插设备不会自动
 *      仿真、不会自动连 QEMU；必须先由用户点模式按钮 + 连接按钮。
 *   2) 语言只有 zh/en 两档，切换即时生效（静态文案 + 状态栏 + 按钮 + 分段
 *      开关），选择持久化在 localStorage：xrk.lang / xrk.mode。
 *   3) 页面契约（显式 ID，壳只认这几个锚点，其余页面自便）：
 *        <span id="modebar"></span>   // 壳注入「仿真 | 真实」分段开关
 *        <button id="btn-lang"></button>  // 壳接管：切换 zh/en（文案自更新）
 *        <select id="port"></select>  // 真实模式可选串口；仿真模式壳置灰
 *        <button id="btn"></button>   // 连接/断开/启动仿真/停止仿真（点击处理归页面）
 *        <span id="status"></span>    // 壳接管状态文案（UI.setConnected 驱动）
 *        [data-i18n="key"] / [data-i18n-html="key"] / [data-i18n-ph="key"]
 *        [data-i18n-title="key"]      // 静态文案锚点，UI.apply() 统一刷新
 *   4) 页面脚本契约：UI.define({zh:{...},en:{...}}) → UI.init({page}) →
 *      bridge.portChanged.connect(on => UI.setConnected(on)) →
 *      按钮点击里按 UI.mode 决定 openSim() / openPort()。
 *   5) 兼容旧 kick：window.startSim() / window.autoConnect(port) 由页面实现，
 *      内部先 UI.setMode(...) 再走同一条显式路径（app.py --mode 亦复用）。
 *
 * headless 可测：node 下 require 本文件即得 UI（无 document 时所有 DOM 操作
 * 静默跳过）；tests/ui_shell_harness.js 校验词典键对齐与禁自动启动契约。
 */
(function (root) {
  "use strict";

  var LS_MODE = "xrk.mode";
  var LS_LANG = "xrk.lang";
  var MODES = ["sim", "real"];
  var LANGS = ["zh", "en"];

  /* —— 持久化（无 localStorage 时退化为内存字典：node harness / 隐私模式）—— */
  var mem = {};
  var store = (function () {
    try {
      var ls = root.localStorage;
      if (ls && typeof ls.getItem === "function") {
        ls.getItem(LS_MODE);            // 探测可读写（部分环境抛 SecurityError）
        return ls;
      }
    } catch (e) { /* 落到内存字典 */ }
    return {
      getItem: function (k) { return Object.prototype.hasOwnProperty.call(mem, k) ? mem[k] : null; },
      setItem: function (k, v) { mem[k] = String(v); },
    };
  })();

  function doc() {
    return root.document || null;
  }
  function byId(id) {
    var d = doc();
    return d && typeof d.getElementById === "function" ? d.getElementById(id) : null;
  }

  /* —— 壳层通用词典（页面词典经 UI.define 合并进来）——
     注意：zh 侧状态文案「已连接 / 未连接」是既有测试锚点（Qt 页测试断言），勿改。 */
  var COMMON = {
    zh: {
      "mode.sim": "仿真",
      "mode.real": "真实",
      "mode.simTip": "仿真模式：本机合成数据源（不连设备、不连 QEMU）",
      "mode.realTip": "真实模式：连接设备串口或外部桥（需先就绪）",
      "btn.connect": "连接",
      "btn.disconnect": "断开",
      "btn.startSim": "启动仿真",
      "btn.stopSim": "停止仿真",
      "btn.lang": "EN",
      "status.connected": "已连接",
      "status.disconnected": "未连接",
      "common.threeMissing": "three.js 加载失败（web/assets/vendor/three.min.js）",
      "common.port": "串口",
      "msg.calibSent": "已向固件下发 {cmd}（明文行，SCEN 同款；当前 exp2 固件开机自标定不解析下行）",
      "msg.calibSimNoDownlink": "仿真源无下行通道（调零命令仅真实串口生效）；统计窗已重置——仿真演示的调零效果看窗口重统计",
      "msg.notConnected": "未连接（先在「仿真」或「真实」模式下点启动/连接）",
    },
    en: {
      "mode.sim": "Sim",
      "mode.real": "Real",
      "mode.simTip": "Simulation mode: local synthetic source (no device, no QEMU)",
      "mode.realTip": "Real mode: device serial port or external bridge (start it first)",
      "btn.connect": "Connect",
      "btn.disconnect": "Disconnect",
      "btn.startSim": "Start sim",
      "btn.stopSim": "Stop sim",
      "btn.lang": "中文",
      "status.connected": "Connected",
      "status.disconnected": "Disconnected",
      "common.threeMissing": "three.js failed to load (web/assets/vendor/three.min.js)",
      "common.port": "Port",
      "msg.calibSent": "Sent {cmd} to firmware (plain line, same as SCEN; current exp2 firmware self-calibrates on boot and ignores downlink)",
      "msg.calibSimNoDownlink": "Simulation source has no downlink (CAL only works over a real port); stats window reset — watch the window re-accumulate",
      "msg.notConnected": "Not connected (pick Simulation or Real, then press start/connect)",
    },
  };

  var DICT = { zh: {}, en: {} };
  LANGS.forEach(function (lang) {
    Object.keys(COMMON[lang]).forEach(function (k) { DICT[lang][k] = COMMON[lang][k]; });
  });

  var state = {
    mode: readPref(LS_MODE, MODES, "real"),   // 缺省真实模式：不点不启动（禁隐式仿真）
    lang: readPref(LS_LANG, LANGS, "zh"),     // 缺省中文（既有测试锚点）
    connected: false,
    page: "",
    inited: false,
  };

  function readPref(key, allowed, fallback) {
    var v = store.getItem(key);
    return allowed.indexOf(v) >= 0 ? v : fallback;
  }

  var listeners = { mode: [], lang: [], connected: [] };

  function emit(kind, value) {
    for (var i = 0; i < listeners[kind].length; i++) {
      try { listeners[kind][i](value); } catch (e) { /* 页面回调异常不阻断壳 */ }
    }
  }

  /* —— 文案 —— */
  function translate(lang, key, vars) {
    var table = DICT[lang] || {};
    var s = Object.prototype.hasOwnProperty.call(table, key) ? table[key] : key;
    if (vars) {
      Object.keys(vars).forEach(function (name) {
        s = s.split("{" + name + "}").join(String(vars[name]));
      });
    }
    return s;
  }

  /* —— DOM 刷新（缺失元素一律跳过：harness/无壳页面安全）—— */
  function applyStatic() {
    var d = doc();
    if (!d || typeof d.querySelectorAll !== "function") return;
    var nodes = d.querySelectorAll("[data-i18n],[data-i18n-html],[data-i18n-ph],[data-i18n-title]");
    for (var i = 0; i < nodes.length; i++) {
      var el = nodes[i];
      var attrs = [
        ["data-i18n", function (v) { el.textContent = v; }],
        ["data-i18n-html", function (v) { el.innerHTML = v; }],
        ["data-i18n-ph", function (v) { el.setAttribute("placeholder", v); }],
        ["data-i18n-title", function (v) { el.setAttribute("title", v); }],
      ];
      for (var a = 0; a < attrs.length; a++) {
        var key = el.getAttribute && el.getAttribute(attrs[a][0]);
        if (key) attrs[a][1](translate(state.lang, key));
      }
    }
    if (d.documentElement && d.documentElement.setAttribute) {
      d.documentElement.setAttribute("lang", state.lang === "en" ? "en" : "zh-CN");
    }
  }

  function renderModeBar() {
    var bar = byId("modebar");
    if (!bar || !doc() || typeof doc().createElement !== "function") return;
    if (typeof bar.appendChild !== "function") return;
    bar.textContent = "";
    MODES.forEach(function (mode) {
      var btn = doc().createElement("button");
      btn.type = "button";
      btn.className = "ui-seg" + (state.mode === mode ? " ui-seg-on" : "");
      btn.textContent = translate(state.lang, "mode." + mode);
      btn.title = translate(state.lang, "mode." + mode + "Tip");
      if (typeof btn.setAttribute === "function") btn.setAttribute("data-mode", mode);
      btn.onclick = function () { api.setMode(mode); };
      bar.appendChild(btn);
    });
  }

  function btnLabel() {
    if (state.connected) {
      return translate(state.lang, state.mode === "sim" ? "btn.stopSim" : "btn.disconnect");
    }
    return translate(state.lang, state.mode === "sim" ? "btn.startSim" : "btn.connect");
  }

  function renderChrome() {
    var btn = byId("btn");
    if (btn) btn.textContent = btnLabel();
    var langBtn = byId("btn-lang");
    if (langBtn) {
      langBtn.textContent = translate(state.lang, "btn.lang");
      langBtn.title = state.lang === "zh" ? "Switch to English" : "切换到中文";
    }
    var status = byId("status");
    if (status) {
      status.textContent = translate(state.lang,
        state.connected ? "status.connected" : "status.disconnected");
    }
    var port = byId("port");
    if (port) {
      if ("disabled" in port) port.disabled = state.mode === "sim";
      if (port.style) port.style.opacity = state.mode === "sim" ? "0.45" : "1";
    }    var cal = byId("btn-cal");
    if (cal && "disabled" in cal) cal.disabled = false;   // 调零按钮仍有回执语义，不禁用
    renderModeBar();
  }

  var api = {
    LS_MODE: LS_MODE,
    LS_LANG: LS_LANG,
    MODES: MODES,
    LANGS: LANGS,

    /* 页面词典合并（重复键以页面为准；两侧键必须成对，harness 会查） */
    define: function (dict) {
      LANGS.forEach(function (lang) {
        var part = (dict && dict[lang]) || {};
        Object.keys(part).forEach(function (k) { DICT[lang][k] = part[k]; });
      });
      if (state.inited) applyStatic();
      return api;
    },

    /* 取词：vars 用 {name} 占位（缺键返回键名——便于 harness 发现漏词） */
    t: function (key, vars) { return translate(state.lang, key, vars); },
    has: function (key) { return Object.prototype.hasOwnProperty.call(DICT.zh, key); },
    dict: function (lang) { return DICT[lang === "en" ? "en" : "zh"]; },

    /* 静态文案刷新（语言切换后由 setLang 自动调用；页面动态重建时可手动调） */
    apply: function () { applyStatic(); return api; },

    get mode() { return state.mode; },
    get lang() { return state.lang; },
    get connected() { return state.connected; },

    /* 模式：持久化 + 刷新壳 → 通知页面（页面负责关掉当前连接） */
    setMode: function (mode, opts) {
      if (MODES.indexOf(mode) < 0) throw new Error("UI.setMode: 非法模式 " + mode);
      var silent = !!(opts && opts.silent);
      if (state.mode === mode) { renderChrome(); return api; }
      state.mode = mode;
      store.setItem(LS_MODE, mode);
      renderChrome();
      if (!silent) emit("mode", mode);
      return api;
    },

    setLang: function (lang) {
      if (LANGS.indexOf(lang) < 0) throw new Error("UI.setLang: 非法语言 " + lang);
      state.lang = lang;
      store.setItem(LS_LANG, lang);
      applyStatic();
      renderChrome();
      emit("lang", lang);
      return api;
    },
    toggleLang: function () { return api.setLang(state.lang === "zh" ? "en" : "zh"); },

    /* 连接状态（唯一事实源在页面：portChanged 驱动） */
    setConnected: function (on) {
      state.connected = !!on;
      renderChrome();           // 状态文案与按钮都走同一渲染路径（语言切换亦复用）
      emit("connected", state.connected);
      return api;
    },

    onModeChange: function (fn) { listeners.mode.push(fn); return api; },
    onLangChange: function (fn) { listeners.lang.push(fn); return api; },
    onConnectedChange: function (fn) { listeners.connected.push(fn); return api; },

    /* 初始化：只渲染壳，**绝不启动数据源**（模式/语言均需用户显式点选） */
    init: function (opts) {
      state.page = (opts && opts.page) || state.page;
      var d = doc();
      if (d && d.head && typeof d.createElement === "function"
          && typeof d.getElementById === "function") {
        var styleId = "ui-shell-style";
        if (!d.getElementById(styleId)) {
          var st = d.createElement("style");
          st.id = styleId;
          st.textContent =
            "#modebar{display:inline-flex;border:1px solid var(--line,#2c3947);" +
            "border-radius:4px;overflow:hidden}" +
            "#modebar .ui-seg{background:transparent;border:0;border-radius:0;" +
            "padding:5px 12px;opacity:.65}" +
            "#modebar .ui-seg-on{background:rgba(79,195,247,.18);opacity:1;" +
            "color:var(--acc,#4fc3f7);box-shadow:inset 0 0 0 1px rgba(79,195,247,.45)}" +
            "#btn-lang{min-width:52px}";
          d.head.appendChild(st);
        }
      }
      var langBtn = byId("btn-lang");
      if (langBtn) langBtn.onclick = function () { api.toggleLang(); };
      applyStatic();
      renderChrome();
      state.inited = true;
      return api;
    },

    /* harness 用：重置到默认（zh / real / 未连接），不持久化 */
    _reset: function () {
      state.mode = "real";
      state.lang = "zh";
      state.connected = false;
      listeners = { mode: [], lang: [], connected: [] };
      return api;
    },
  };

  /* 端口边界：浏览器挂 window.UI；node harness 走 module.exports */
  if (root) root.UI = api;
  if (typeof module !== "undefined" && module.exports) module.exports = api;
})(typeof globalThis !== "undefined" ? globalThis : this);
