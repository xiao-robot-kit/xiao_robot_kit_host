/* ui_shell_harness.js — 上位机统一外壳（web/assets/ui-shell.js）契约测试（node，无浏览器）。
 *
 * 目的：把「仿真/真实 双模式」与「中英双语」从口头约定变成可执行断言。
 *   0) 缺省状态：real + zh + 未连接（禁隐式回落：不点不启动任何数据源）
 *   1) 词典：zh/en 键集合必须一一对应（每个页面内联 UI.define 的词典都查）
 *   2) 页面锚点闭合：页面里出现的每个 data-i18n[-html|-ph|-title] 键都必须能取到词
 *   3) 模式语义：setMode 持久化 + 按钮文案随模式/连接状态变化；非法值抛错
 *   4) 语言语义：setLang 即时生效（status/btn 文案与 t() 同步切换）
 *   5) 禁自动启动：页面内联脚本里 UI.init 之外不得有顶层 openSim/openPort 调用
 *
 * 跑法：node tests/ui_shell_harness.js（exit 0 = 全过）
 */

"use strict";
const fs = require("fs");
const path = require("path");

const fails = [];
function check(name, ok, detail) {
    if (!ok) fails.push(`${name}${detail ? "：" + detail : ""}`);
}

/* —— DOM 桩（与各页 harness 同款最小面）—— */
function stubEl() {
    const el = {
        textContent: "", innerHTML: "", value: "", title: "", disabled: false,
        children: [], style: {},
        appendChild(c) { el.children.push(c); },
        removeChild(c) { const i = el.children.indexOf(c); if (i >= 0) el.children.splice(i, 1); },
        getAttribute() { return null; },
        setAttribute() {},
    };
    el.options = el.children;
    return el;
}
const els = {};
globalThis.window = { addEventListener() {} };
globalThis.document = {
    getElementById(id) { return (els[id] = els[id] || stubEl()); },
    createElement() { return stubEl(); },
    // 无 querySelectorAll / head：applyStatic 与样式注入应安全跳过
};

const SHELL = path.join(__dirname, "..", "web", "assets", "ui-shell.js");
const UI = require(SHELL);
globalThis.UI = UI;

/* —— 0) 缺省状态 —— */
check("缺省模式=real", UI.mode === "real", `mode=${UI.mode}`);
check("缺省语言=zh", UI.lang === "zh", `lang=${UI.lang}`);
check("缺省未连接", UI.connected === false);
UI.init({ page: "harness" });
check("init后可取词", UI.t("mode.sim") === "仿真" && UI.t("mode.real") === "真实");
check("init后仍未连接（禁自动启动）", UI.connected === false);
check("缺省按钮=连接", els["btn"].textContent === "连接", `btn=${els["btn"].textContent}`);
check("缺省状态=未连接", els["status"].textContent === "未连接",
      `status=${els["status"].textContent}`);
check("modebar 两档", els["modebar"].children.length === 2,
      `children=${els["modebar"].children.length}`);

/* —— 1) 模式语义 —— */
UI.setMode("sim");
check("setMode(sim) 生效", UI.mode === "sim");
check("sim 按钮=启动仿真", els["btn"].textContent === "启动仿真",
      `btn=${els["btn"].textContent}`);
check("sim 模式禁用串口选择", els["port"].disabled === true);
UI.setConnected(true);
check("sim 已连接按钮=停止仿真", els["btn"].textContent === "停止仿真",
      `btn=${els["btn"].textContent}`);
check("已连接状态文案", els["status"].textContent === "已连接",
      `status=${els["status"].textContent}`);
let modeEvents = 0, lastMode = null;
UI.onModeChange((m) => { modeEvents++; lastMode = m; });
UI.setMode("real");
check("setMode 通知监听", modeEvents === 1 && lastMode === "real");
check("real 模式放开串口选择", els["port"].disabled === false);
check("real 已连接按钮=断开", els["btn"].textContent === "断开",
      `btn=${els["btn"].textContent}`);
let threw = false;
try { UI.setMode("qemu"); } catch (e) { threw = true; }
check("非法模式抛错", threw);
UI.setConnected(false);

/* —— 2) 语言语义 —— */
UI.setLang("en");
check("en: mode.sim", UI.t("mode.sim") === "Sim");
check("en: btn.connect", els["btn"].textContent === "Connect",
      `btn=${els["btn"].textContent}`);
check("en: status", els["status"].textContent === "Disconnected",
      `status=${els["status"].textContent}`);
check("en: 模板插值", UI.t("msg.calibSent", { cmd: "CAL" }).includes("CAL"));
UI.toggleLang();
check("toggleLang 回 zh", UI.lang === "zh" && UI.t("mode.sim") === "仿真");
let langEvents = 0;
UI.onLangChange(() => { langEvents++; });
UI.setLang("en");
check("setLang 通知监听", langEvents === 1);
UI.setLang("zh");


/* 去注释（保留字符串内容）：避免「注释里写了 bridge.openSim()」造成误判。
   同时把字符串字面量内容抹成空串——字符串里的调用不是真调用。 */
function stripComments(src) {
    let out = "", i = 0, inStr = null;
    while (i < src.length) {
        const c = src[i], n = src[i + 1];
        if (inStr) {
            if (c === "\\") { out += c + (n || ""); i += 2; continue; }
            if (c === inStr) { inStr = null; out += c; i++; continue; }
            out += c; i++; continue;
        }
        if (c === "/" && n === "*") {
            const end = src.indexOf("*/", i + 2);
            i = end < 0 ? src.length : end + 2;
            continue;
        }
        if (c === "/" && n === "/") {
            const end = src.indexOf("\n", i + 2);
            i = end < 0 ? src.length : end;
            continue;
        }
        if (c === '"' || c === "'" || c === "`") { inStr = c; out += c; i++; continue; }
        out += c; i++;
    }
    return out;
}

/* —— 3) 薄壳声明 + 核心词典/锚点/禁自动启动 ——
   IMU 实验页已是「薄壳 + 统一核心」（用户定案 2026-11-06：逐级继承）：
   薄壳（examples/imu/.../index.html）只声明 window.IMU_PAGE={id,panels}，
   词典 / i18n 锚点 / 启动路径全在 web/assets/imu-page.js。
   故此段对薄壳查「声明了什么」，对核心查「词典与锚点闭合」。 */
const PAGES = {
    exp1: path.join(__dirname, "..", "examples", "imu", "exp1_waveform", "index.html"),
    exp2: path.join(__dirname, "..", "examples", "imu", "exp2_bias", "index.html"),
    exp4: path.join(__dirname, "..", "examples", "imu", "exp4_quat_view", "index.html"),
    exp5: path.join(__dirname, "..", "examples", "imu", "exp5_trajectory", "index.html"),
};
const PANELS = {                       // 逐级继承的规定集合（缺一即 bug）
    exp1: ["raw"],
    exp2: ["raw", "cal"],
    exp4: ["raw", "cal", "euler", "quat"],
    exp5: ["raw", "cal", "euler", "quat", "traj"],
};

/* 抠出对象字面量（大括号配平扫描，支持嵌套与字符串）：from = 起始搜索下标 */
function extractObjectLiteral(src, from) {
    if (from < 0) return null;
    const i = src.indexOf("{", from);
    if (i < 0) return null;
    let depth = 0, inStr = null, esc = false;
    for (let j = i; j < src.length; j++) {
        const c = src[j];
        if (inStr) {
            if (esc) esc = false;
            else if (c === "\\") esc = true;
            else if (c === inStr) inStr = null;
            continue;
        }
        if (c === '"' || c === "'" || c === "`") { inStr = c; continue; }
        if (c === "{") depth++;
        else if (c === "}") {
            depth--;
            if (depth === 0) {
                try {
                    return new Function("return " + src.slice(i, j + 1))();
                } catch (e) {
                    check("对象字面量可求值", false, e.message);
                    return null;
                }
            }
        }
    }
    return null;
}

for (const [name, file] of Object.entries(PAGES)) {
    const src = fs.readFileSync(file, "utf8");
    const m = src.match(/window\.IMU_PAGE\s*=\s*(\{[\s\S]*?\})\s*;/);
    let cfg = null;
    if (m) {
        try { cfg = new Function("return " + m[1])(); }
        catch (e) { check(`${name} 薄壳配置可求值`, false, e.message); }
    }
    check(`${name} 薄壳声明 IMU_PAGE`, !!cfg);
    if (cfg) {
        check(`${name} 薄壳 id 自洽`, cfg.id === name, `id=${cfg.id}`);
        check(`${name} 面板集合逐级继承`,
              JSON.stringify((cfg.panels || []).slice().sort()) ===
              JSON.stringify(PANELS[name].slice().sort()),
              `got=${JSON.stringify(cfg.panels)}`);
    }
    check(`${name} 薄壳载入统一核心`, src.includes("/assets/imu-page.js"));
    check(`${name} 薄壳不含实现（词典/逻辑在核心）`,
          !src.includes("UI.define(") && !/function\s+\w+\(/.test(src));
}

/* 核心：词典 zh/en 键对齐（imu.* 前缀）+ 锚点闭合 + 顶层启源守卫 */
const CORE_SRC = fs.readFileSync(
    path.join(__dirname, "..", "web", "assets", "imu-page.js"), "utf8");
const CORE_DICT = extractObjectLiteral(CORE_SRC, CORE_SRC.indexOf("var DICT = "));
check("核心有词典对象", !!CORE_DICT);
if (CORE_DICT) {
    const zhKeys = Object.keys(CORE_DICT.zh || {}).sort();
    const enKeys = Object.keys(CORE_DICT.en || {}).sort();
    check("核心词典 zh/en 键对齐",
          zhKeys.length > 0 && zhKeys.join("|") === enKeys.join("|"),
          `zh=${zhKeys.length} en=${enKeys.length} 差集=` +
          JSON.stringify(zhKeys.filter((k) => enKeys.indexOf(k) < 0)
            .concat(enKeys.filter((k) => zhKeys.indexOf(k) < 0))));
    check("核心词典键全部 imu.* 前缀",
          zhKeys.every((k) => k.indexOf("imu.") === 0),
          JSON.stringify(zhKeys.filter((k) => k.indexOf("imu.") !== 0)));
    UI.define(CORE_DICT);                     // 合并进壳词典（后续取词检查用）
    /* 锚点闭合 A：核心 markup 里的 data-i18n 字面量键 */
    const attrs = [...CORE_SRC.matchAll(/data-i18n(?:-html|-ph|-title)?="([^"]+)"/g)]
        .map((mm) => mm[1]);
    const missAttr = attrs.filter((k) => !UI.has(k));
    check("核心 i18n 锚点闭合", attrs.length > 0 && missAttr.length === 0,
          `attrs=${attrs.length} missing=${JSON.stringify([...new Set(missAttr)])}`);
    /* 锚点闭合 B：核心代码里 UI.t("imu.…") 的字面量键（防拼错） */
    const calls = [...CORE_SRC.matchAll(/UI\.t\("(imu\.[^"]+)"\)/g)].map((mm) => mm[1]);
    const missCall = calls.filter((k) => !UI.has(k));
    check("核心取词键闭合", calls.length > 0 && missCall.length === 0,
          `calls=${calls.length} missing=${JSON.stringify([...new Set(missCall)])}`);
}

check("核心调 UI.init({page})", /UI\.init\(\{\s*page:\s*PAGE_ID\s*\}\)/.test(CORE_SRC));

/* 禁自动启动：实现里 openSim(/openPort( 不得在顶层裸调
   （必须包在按钮处理或 startSim/autoConnect 等 kick 函数内）。
   判据：出现处前 400 字符内必须有 "function <name>(" 或 "onclick =" ——
   粗粒度但足以挡住「加载即连」回归。 */
const coreCode = stripComments(CORE_SRC);
for (const call of ["openSim(", "openPort("]) {
    let idx = coreCode.indexOf(call);
    while (idx >= 0) {
        const head = coreCode.slice(Math.max(0, idx - 400), idx);
        const guarded = /function\s+\w+\s*\([^)]*\)\s*\{[^]*$/.test(head) ||
                        /onclick\s*=\s*(\(|function)/.test(head);
        check(`核心 ${call} 不在顶层裸调`, guarded,
              `位置 ${idx} 前文=${JSON.stringify(head.slice(-60))}`);
        idx = coreCode.indexOf(call, idx + call.length);
    }
}

/* —— 3.5) 后端回执码 ↔ 前端词典闭合 ——
   bridge.sendCalib 之类的 Slot 返回 {code,...}，页面取 UI.t("msg."+code)
   （旧桌面壳 ui-shell 词典）或 UI.t("imu.msg."+code)（服务器版 IMU 页核心词典，
   含仿真档 calibSimulated）。后端新增一个码而前端没词 = 界面直接显示键名
   （用户看不懂）——用源码扫描对齐。 */
{
    const sources = [
        path.join(__dirname, "..", "backend", "bridge.py"),
        path.join(__dirname, "..", "backend", "hub.py"),
    ];
    let codes = [];
    for (const file of sources) {
        const src = fs.readFileSync(file, "utf8");
        codes = codes.concat([...src.matchAll(/"code":\s*"([A-Za-z0-9_.]+)"/g)].map((m) => m[1]));
    }
    codes = [...new Set(codes)];
    check("后端有回执码", codes.length > 0, `codes=${JSON.stringify(codes)}`);
    const missing = codes.filter((c) => !UI.has("msg." + c) && !UI.has("imu.msg." + c));
    check("回执码词典闭合", missing.length === 0,
          `缺词=${JSON.stringify(missing)}` +
          `（bridge.py/hub.py 的 code 必须在前端词典里成对存在）`);
}

/* —— 4) 汇总 —— */
if (fails.length) {
    fails.forEach((f) => console.log("  [FAIL]", f));
    console.log(`== ui_shell_harness: ${fails.length} fail ==`);
    process.exit(1);
}
console.log("== ui_shell_harness: 缺省态/模式语义/语言语义/词典键对齐/锚点闭合/禁自动启动, 0 fail ==");
