/* motor_page_harness.js — 电机实例目录页面契约测试（node，无浏览器）。
 *
 * 被测对象：
 *   web/assets/motor-page.js            页面核心（7 个电机实验共用的共享资产）
 *   examples/motor/<dir>/index.html     薄壳（只声明 window.MOTOR_PAGE + 宿主 #app）
 *
 * 做法：从每页 HTML 抽内联薄壳（vm 沙箱求值取 MOTOR_PAGE），再用最小
 * DOM / EventSource / fetch / localStorage 桩 require 页面核心，驱动真实事件与点击。
 *
 * 断言（任务卡口径）：
 *   S) 7 个薄壳结构：单一内联块 + 两个 /assets/ 脚本 + #app；id/scen/slots 与目录契约一致
 *   D) 词典 zh/en 键一一对应；页面 data-i18n 锚点全部可解析（缺一即 bug）
 *   1) 加载不启动任何数据源：零 /api/open；只读 /api/state + /api/ports、订阅 /api/stream
 *   2) 点「启动仿真」→ 恰一次 POST /api/open，body = {"mode":"sim"}；状态行=已连接
 *   3) mframe → 槽位表数值更新、波形点数增加、canvas 已绘制
 *   4) verdict pass=false → FAIL 徽标；逐项 ✓/✗；已知 key 走词典、未知 key 原样显示
 *   5) 切 en → 标题/按钮/表头变英文
 *   6) /api/open 409 → 错误進日志、状态回未连接、real body 带 target 且持久化
 *   P) pidscan 单槽页：一行槽位表，帧 slot=扫描组号照原值显示且入同一行
 *
 * 跑法：node tests/motor_page_harness.js（exit 0 = 全过；需 node>=18）
 */

"use strict";
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const ROOT = path.join(__dirname, "..");
const SHELL = path.join(ROOT, "web", "assets", "ui-shell.js");
const PAGE_JS = path.join(ROOT, "web", "assets", "motor-page.js");

const fails = [];
function check(name, ok, detail) {
    if (!ok) fails.push(`${name}${detail ? "：" + detail : ""}`);
}
/* 等页面渲染泵：页面按 RENDER_INTERVAL_MS(33ms) 的 setTimeout 节拍合并刷新，
   等待两个以上节拍再断言 DOM（不靠内部 flush 后门）。 */
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/* 实例目录 ↔ 页面 id（服务器 --page / URL 前缀）↔ 期望 0x06 场景号 ↔ 槽位数
   —— 后端契约（backend/server.py 的 MOTOR_PAGES 注册），勿随手改 */
const PAGES = [
    { dir: "exp_motor_pwm", id: "motor_pwm", scen: 1, slots: 4 },
    { dir: "exp_motor_servo", id: "motor_servo", scen: 2, slots: 4 },
    { dir: "exp_motor_enc", id: "motor_enc", scen: 3, slots: 4 },
    { dir: "exp_motor_nested", id: "motor_nested", scen: 4, slots: 4 },
    { dir: "exp_motor_dc", id: "motor_dc", scen: 5, slots: 4 },
    { dir: "motor_scope", id: "motor_scope", scen: null, slots: 4 },
    { dir: "exp_motor_pidscan", id: "motor_pidscan", scen: null, slots: 1 },
];

/* ─────────────── S) 薄壳结构（7 页全查） ─────────────── */
function readShell(page) {
    const file = path.join(ROOT, "examples", "motor", page.dir, "index.html");
    const html = fs.readFileSync(file, "utf8");
    const inline = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map((m) => m[1]);
    const srcs = [...html.matchAll(/<script\s+src="([^"]+)"><\/script>/g)].map((m) => m[1]);
    const sandbox = { window: {} };
    inline.forEach((code) => vm.runInNewContext(code, sandbox, { timeout: 1000 }));
    return { html, inline, srcs, cfg: sandbox.window.MOTOR_PAGE };
}

const shells = {};
PAGES.forEach((page) => {
    const sh = readShell(page);
    shells[page.dir] = sh;
    check(`${page.dir}: 页面存在`, !!sh.html);
    check(`${page.dir}: 单一内联薄壳块`, sh.inline.length === 1, `blocks=${sh.inline.length}`);
    check(`${page.dir}: 只加载两个共享资产`,
          JSON.stringify(sh.srcs) === JSON.stringify(["/assets/ui-shell.js", "/assets/motor-page.js"]),
          JSON.stringify(sh.srcs));
    check(`${page.dir}: 宿主容器 #app`, /id="app"/.test(sh.html));
    const cfg = sh.cfg || {};
    check(`${page.dir}: MOTOR_PAGE.id 与目录契约一致`, cfg.id === page.id, `id=${cfg.id}`);
    check(`${page.dir}: MOTOR_PAGE.scen 与场景契约一致`, cfg.scen === page.scen, `scen=${cfg.scen}`);
    check(`${page.dir}: 槽位数（缺省 4）`, (cfg.slots === undefined ? 4 : cfg.slots) === page.slots,
          `slots=${cfg.slots}`);
    check(`${page.dir}: 中文标题兜底文案`, typeof cfg.title === "string" && cfg.title.length > 0);
});

/* ─────────────── 桩环境（每次 load 一套干净的） ─────────────── */
function makeEnv() {
    const els = [];
    const ids = new Map();

    function makeCtx() {
        const ctx = {
            calls: 0, fillStyle: "", strokeStyle: "", font: "",
            textAlign: "", textBaseline: "",
            beginPath() { ctx.calls++; }, moveTo() { ctx.calls++; }, lineTo() { ctx.calls++; },
            stroke() { ctx.calls++; }, clearRect() { ctx.calls++; }, fillText() { ctx.calls++; },
            fillRect() { ctx.calls++; }, arc() { ctx.calls++; }, closePath() { ctx.calls++; },
            save() {}, restore() {}, setLineDash() {},
        };
        return ctx;
    }

    function el(tag) {
        const e = {
            tagName: String(tag).toUpperCase(), id: "", className: "", type: "", value: "",
            textContent: "", innerHTML: "", hidden: false, disabled: false, title: "",
            width: 0, height: 0, clientWidth: 900, clientHeight: 240,
            scrollTop: 0, scrollHeight: 0, attrs: {}, style: {}, handlers: {}, children: [],
            appendChild(c) { e.children.push(c); return c; },
            removeChild(c) {
                const i = e.children.indexOf(c);
                if (i >= 0) e.children.splice(i, 1);
                return c;
            },
            get firstChild() { return e.children.length ? e.children[0] : null; },
            setAttribute(k, v) { e.attrs[k] = String(v); },
            getAttribute(k) {
                return Object.prototype.hasOwnProperty.call(e.attrs, k) ? e.attrs[k] : null;
            },
            addEventListener(t, fn) { (e.handlers[t] = e.handlers[t] || []).push(fn); },
            removeEventListener(t, fn) {
                const a = e.handlers[t] || [];
                const i = a.indexOf(fn);
                if (i >= 0) a.splice(i, 1);
            },
            click() {
                const ev = { target: e, type: "click" };
                if (typeof e.onclick === "function") e.onclick(ev);
                (e.handlers.click || []).forEach((fn) => fn(ev));
            },
            getContext() {
                if (!e._ctx) e._ctx = makeCtx();
                return e._ctx;
            },
            classList: {
                _s: new Set(),
                add(c) { this._s.add(c); },
                remove(c) { this._s.delete(c); },
                contains(c) { return this._s.has(c); },
                toggle(c, on) {
                    if (on === undefined) { this._s.has(c) ? this._s.delete(c) : this._s.add(c); }
                    else if (on) this._s.add(c);
                    else this._s.delete(c);
                },
            },
        };
        els.push(e);
        return e;
    }

    const app = el("div");
    app.id = "app";
    ids.set("app", app);

    function selectorAttrs(sel) {
        return String(sel).split(",").map((s) => s.trim()).filter(Boolean)
            .map((s) => { const m = s.match(/^\[([-\w]+)\]$/); return m ? m[1] : null; })
            .filter(Boolean);
    }

    const document = {
        head: { appendChild() {} },
        documentElement: { attrs: {}, setAttribute(k, v) { this.attrs[k] = v; } },
        getElementById(id) {
            if (ids.has(id)) return ids.get(id);
            for (const e of els) if (e.id === id) { ids.set(id, e); return e; }
            return null;
        },
        createElement(tag) { return el(tag); },
        querySelectorAll(sel) {
            const attrs = selectorAttrs(sel);
            const out = new Set();
            els.forEach((e) => {
                attrs.forEach((a) => { if (e.getAttribute(a)) out.add(e); });
            });
            return Array.from(out);
        },
    };

    /* localStorage（xrk.mode / xrk.lang / xrk.motorTarget 都落这里） */
    const data = new Map();
    const localStorage = {
        getItem(k) { return data.has(k) ? data.get(k) : null; },
        setItem(k, v) { data.set(k, String(v)); },
        removeItem(k) { data.delete(k); },
    };

    /* fetch：只桩页面用到的几条，调用全记录 */
    const net = {
        calls: [],
        open: { status: 200, error: null },
        verify: null,
    };
    function baseState() {
        return { kind: "motor", scenario: "pwm", source: "none", target: "",
                 connected: false, frames: 0, stats: {}, counters: {} };
    }
    function respond(status, data) {
        return { ok: status >= 200 && status < 300, status,
                 json: () => Promise.resolve(data) };
    }
    const fetchStub = (url, opts) => {
        const body = opts && opts.body ? JSON.parse(opts.body) : null;
        const call = { url: String(url), method: (opts && opts.method) || "GET", body };
        net.calls.push(call);
        if (call.method === "GET" && call.url === "/api/state") return Promise.resolve(respond(200, baseState()));
        if (call.method === "GET" && call.url === "/api/ports") {
            return Promise.resolve(respond(200, { ports: [{ device: "COM5", desc: "USB serial" }] }));
        }
        if (call.method === "POST" && call.url === "/api/open") {
            if (net.open.status !== 200) return Promise.resolve(respond(net.open.status, { error: net.open.error }));
            return Promise.resolve(respond(200, Object.assign(baseState(), {
                connected: true, source: body.mode === "sim" ? "sim" : "real",
                target: body.target || "",
            })));
        }
        if (call.method === "POST" && call.url === "/api/close") {
            return Promise.resolve(respond(200, baseState()));
        }
        if (call.method === "POST" && call.url === "/api/verify") {
            return Promise.resolve(respond(200, net.verify || { ok: true, pass: true, items: [], summary: {} }));
        }
        return Promise.resolve(respond(404, { error: "no stub for " + call.url }));
    };

    /* EventSource：记下订阅，再由 harness 主动 emit SSE 事件 */
    const streams = [];
    function EventSource(url) {
        this.url = String(url);
        this.handlers = {};
        this.closed = false;
        streams.push(this);
    }
    EventSource.prototype.addEventListener = function (type, fn) {
        (this.handlers[type] = this.handlers[type] || []).push(fn);
    };
    EventSource.prototype.close = function () { this.closed = true; };
    function emit(type, payload) {
        const inst = streams.filter((s) => !s.closed).pop();
        if (!inst) throw new Error("没有活的 EventSource，无法 emit " + type);
        (inst.handlers[type] || []).forEach((fn) => fn({ data: JSON.stringify(payload) }));
    }

    return { document, localStorage, net, fetchStub, EventSource, streams, emit, els };
}

function loadPage(env, cfg) {
    globalThis.document = env.document;
    globalThis.localStorage = env.localStorage;
    globalThis.EventSource = env.EventSource;
    globalThis.fetch = env.fetchStub;
    globalThis.MOTOR_PAGE = cfg;
    delete require.cache[require.resolve(SHELL)];
    delete require.cache[require.resolve(PAGE_JS)];
    const UI = require(SHELL);
    globalThis.UI = UI;
    const page = require(PAGE_JS);
    return { UI, page };
}

/* DOM 取值助手 */
function byId(env, id) { return env.document.getElementById(id); }
function tableParts(env, id) {
    const t = byId(env, id);
    const thead = t.children[0];
    const tbody = t.children[1];
    return { thead, tbody,
             headers: thead && thead.children[0] ? thead.children[0].children.map((c) => c.textContent) : [],
             rows: tbody ? tbody.children : [] };
}
function logLines(env) {
    return byId(env, "log").children.map((c) => c.textContent);
}

(async function main() {
    /* ════════ 主场景：exp_motor_pwm（4 槽 · SCEN 1） ════════ */
    const env = makeEnv();
    const { UI, page } = loadPage(env, shells.exp_motor_pwm.cfg);
    await sleep(60);                       // 等初始渲染泵（setTimeout 兜底 rAF）

    /* D) 词典对齐 + 锚点闭合 */
    const zhKeys = Object.keys(UI.dict("zh")).sort();
    const enKeys = Object.keys(UI.dict("en")).sort();
    const zhOnly = zhKeys.filter((k) => !enKeys.includes(k));
    const enOnly = enKeys.filter((k) => !zhKeys.includes(k));
    check("词典 zh/en 键一一对应", zhOnly.length === 0 && enOnly.length === 0,
          `zhOnly=${zhOnly.join(",")} enOnly=${enOnly.join(",")}`);
    PAGES.forEach((p) => {
        check(`词典覆盖页面标题 ${p.id}`, UI.has("motor.title." + p.id), "motor.title." + p.id);
    });
    const anchors = env.document.querySelectorAll(
        "[data-i18n],[data-i18n-html],[data-i18n-ph],[data-i18n-title]");
    const unresolved = anchors
        .map((e) => e.getAttribute("data-i18n") || e.getAttribute("data-i18n-ph")
                 || e.getAttribute("data-i18n-title") || e.getAttribute("data-i18n-html"))
        .filter((k) => k && !UI.has(k));
    check("data-i18n 锚点全部可解析", unresolved.length === 0, unresolved.join(","));
    check("锚点非空（头部+四个面板）", anchors.length >= 7, `anchors=${anchors.length}`);

    /* ① 加载不启动数据源 */
    check("加载后零 POST /api/open", env.net.calls.every((c) => c.url !== "/api/open"),
          JSON.stringify(env.net.calls));
    check("加载即读 /api/state",
          env.net.calls.some((c) => c.url === "/api/state" && c.method === "GET"));
    check("加载即读 /api/ports",
          env.net.calls.some((c) => c.url === "/api/ports" && c.method === "GET"));
    check("加载即订阅 /api/stream",
          env.streams.length === 1 && env.streams[0].url === "/api/stream",
          `streams=${env.streams.map((s) => s.url).join(",")}`);
    check("加载状态行=未连接", byId(env, "status").textContent === "未连接",
          byId(env, "status").textContent);
    check("串口 datalist 已填充", byId(env, "ports").children.length === 1,
          `options=${byId(env, "ports").children.length}`);
    check("真实模式缺省显示目标输入", byId(env, "target").hidden === false);

    /* ② 点「仿真」→ 点「启动仿真」→ POST /api/open {"mode":"sim"} */
    const modebar = byId(env, "modebar");
    const simBtn = modebar.children.find((b) => b.textContent === "仿真");
    check("模式分段含「仿真」", !!simBtn);
    simBtn.click();
    await sleep(60);
    check("点「仿真」切到 sim 模式", UI.mode === "sim", `mode=${UI.mode}`);
    check("模式切换不下发 /api/open", env.net.calls.every((c) => c.url !== "/api/open"));
    check("仿真模式隐藏目标输入", byId(env, "target").hidden === true);

    byId(env, "btn").click();
    await sleep(70);
    const opens = env.net.calls.filter((c) => c.url === "/api/open");
    check("点「启动仿真」恰一次 POST /api/open", opens.length === 1, `n=${opens.length}`);
    check("open body 恰为 {\"mode\":\"sim\"}",
          opens.length === 1 && JSON.stringify(opens[0].body) === '{"mode":"sim"}',
          opens.length ? JSON.stringify(opens[0].body) : "no call");
    check("连接后状态行=已连接", byId(env, "status").textContent === "已连接",
          byId(env, "status").textContent);
    check("连接后按钮=停止仿真", byId(env, "btn").textContent === "停止仿真",
          byId(env, "btn").textContent);

    /* ③ mframe → 槽位表 + 波形点 */
    const wavePts0 = page.inspect().wavePoints;
    const frame = (seq, sp) => ({
        type: 5, seq, scen: null, timers: null, recs: [
            { slot: 0, kind: 0, sp, duty: sp, actual: sp - 0.01, phase: 7, fault: 0 },
            { slot: 1, kind: 2, sp: 90, duty: 1500, actual: 90, phase: 3, fault: 0 },
            { slot: 2, kind: 3, sp: 30, duty: 0.19, actual: 29.5, phase: 7, fault: 0 },
            { slot: 3, kind: 1, sp: -0.5, duty: -0.5, actual: -0.49, phase: 7, fault: 0 },
        ],
    });
    env.emit("mframe", frame(1, 0.25));
    await sleep(60);
    const slots0 = tableParts(env, "slots");
    check("槽位表 4 行", slots0.rows.length === 4, `rows=${slots0.rows.length}`);
    const row0 = slots0.rows[0].children.map((c) => c.textContent);
    check("槽位 0 sp 更新", row0[1] === "0.250", JSON.stringify(row0));
    check("槽位 0 duty 更新", row0[2] === "0.250", JSON.stringify(row0));
    check("槽位 0 actual 更新", row0[3] === "0.240", JSON.stringify(row0));
    check("槽位 0 phase/fault 更新", row0[4] === "7" && row0[5] === "0", JSON.stringify(row0));
    const row1 = slots0.rows[1].children.map((c) => c.textContent);
    check("槽位 1 独立列值", row1[1] === "90.000" && row1[2] === "1500.000", JSON.stringify(row1));
    check("波形点数 +1", page.inspect().wavePoints === wavePts0 + 1,
          `${wavePts0} -> ${page.inspect().wavePoints}`);
    check("canvas 已绘制", byId(env, "wave").getContext("2d").calls > 0);

    env.emit("mframe", frame(2, 0.5));
    await sleep(60);
    check("波形点数再 +1", page.inspect().wavePoints === wavePts0 + 2,
          `wavePoints=${page.inspect().wavePoints}`);

    /* 0x06 SCEN 帧：scen_id + 定时器快照表 */
    env.emit("mframe", { type: 6, seq: 3, scen: 1, timers: [[0, 4, 20000]],
                         recs: [{ slot: 0, kind: 0, sp: 0.75, duty: 0.75, actual: 0.74, phase: 7, fault: 0 }] });
    await sleep(60);
    check("SCEN 行显示 scen_id", byId(env, "scen-line").textContent.includes("1"),
          byId(env, "scen-line").textContent);
    const timers = tableParts(env, "timers");
    check("定时器快照表 1 行", timers.rows.length === 1, `rows=${timers.rows.length}`);
    check("定时器快照 [0,4,20000]",
          timers.rows.length === 1
          && JSON.stringify(timers.rows[0].children.map((c) => c.textContent)) === '["0","4","20000"]',
          timers.rows.length ? JSON.stringify(timers.rows[0].children.map((c) => c.textContent)) : "");

    /* text 事件 → 日志 */
    env.emit("text", { line: "TEXT boot ok" });
    await sleep(60);
    check("TEXT 事件進日志", logLines(env).some((l) => l.includes("TEXT boot ok")),
          logLines(env).slice(-2).join(" | "));
    for (let i = 0; i < 240; i++) env.emit("text", { line: "filler " + i });
    await sleep(60);
    check("日志上限 200 行", byId(env, "log").children.length === 200,
          `lines=${byId(env, "log").children.length}`);

    /* ④ verdict pass=false → FAIL 徽标 + 逐项 */
    env.emit("verdict", {
        ok: true, pass: false,
        items: [
            { key: "pwm.staircase", ok: true, detail: "er=0.004" },
            { key: "pidscan.g4", ok: true, detail: "st=0.123s" },
            { key: "not.in.dict", ok: false, detail: "boom" },
        ],
        summary: { frames: 1234, bad_crc: 0, seq_gaps: 2, text: 5 },
    });
    await sleep(60);
    check("verdict pass=false → 徽标 FAIL", byId(env, "badge").textContent === "FAIL",
          byId(env, "badge").textContent);
    const vitems = tableParts(env, "items");
    check("逐项断言 3 行", vitems.rows.length === 3, `rows=${vitems.rows.length}`);
    const vi0 = vitems.rows[0].children.map((c) => c.textContent);
    const vi1 = vitems.rows[1].children.map((c) => c.textContent);
    const vi2 = vitems.rows[2].children.map((c) => c.textContent);
    check("✓/✗ 标记", vi0[0] === "✓" && vi2[0] === "✗", JSON.stringify([vi0[0], vi2[0]]));
    check("已知 key 走前端词典",
          vi0[1] === "PWM · 台阶形状断言（跟随 / 档内平稳 / 档序）", vi0[1]);
    check("pidscan 组号 key 用模板翻译", vi1[1] === "PID 扫描 · 组 4（settle / overshoot / steady）",
          vi1[1]);
    check("未知 key 原样显示", vi2[1] === "not.in.dict", vi2[1]);
    check("detail 原样展示", vi0[2] === "er=0.004", vi0[2]);
    const sum = byId(env, "vsum").textContent;
    check("summary 计数显示", sum.includes("1234") && sum.includes("5") && sum.includes("2"), sum);

    /* ⑤ 切 en → 标题/按钮/表头 */
    byId(env, "btn").click();                       // 断开（已连接态）：只发 /api/close
    await sleep(70);
    check("已连接按钮=停止（POST /api/close）",
          env.net.calls.some((c) => c.url === "/api/close" && c.method === "POST"));
    check("断开后状态行=未连接", byId(env, "status").textContent === "未连接",
          byId(env, "status").textContent);

    byId(env, "btn-lang").click();
    await sleep(60);
    check("点 EN 切到英文", UI.lang === "en", `lang=${UI.lang}`);
    check("切 en：标题变英文", byId(env, "title").textContent === "Motor PWM staircase",
          byId(env, "title").textContent);
    check("切 en：按钮变英文", byId(env, "btn").textContent === "Start sim",
          byId(env, "btn").textContent);
    check("切 en：槽位表头变英文", tableParts(env, "slots").headers[0] === "Slot",
          JSON.stringify(tableParts(env, "slots").headers));
    check("切 en：验收表头变英文", tableParts(env, "items").headers[0] === "Result",
          JSON.stringify(tableParts(env, "items").headers));
    check("切 en：状态行变英文", byId(env, "status").textContent === "Disconnected",
          byId(env, "status").textContent);

    /* ⑥ /api/open 409 → 错误進日志 + 状态=未连接 */
    env.net.open = { status: 409, error: "serial:COM5@115200 unavailable" };
    const realBtn = byId(env, "modebar").children.find((b) => b.textContent === "Real");
    check("切真实模式按钮存在（en）", !!realBtn);
    realBtn.click();
    await sleep(60);
    check("真实模式显示目标输入", byId(env, "target").hidden === false);
    check("模式切换仍不发 /api/open",
          env.net.calls.filter((c) => c.url === "/api/open").length === 1,
          `n=${env.net.calls.filter((c) => c.url === "/api/open").length}`);

    byId(env, "target").value = "serial:COM5@115200";
    byId(env, "btn").click();
    await sleep(70);
    const opens2 = env.net.calls.filter((c) => c.url === "/api/open");
    check("真实模式 POST 带 target",
          opens2.length === 2 && JSON.stringify(opens2[1].body)
            === '{"mode":"real","target":"serial:COM5@115200"}',
          opens2.length > 1 ? JSON.stringify(opens2[1].body) : "no call");
    check("409 错误進日志", logLines(env).some((l) => l.includes("unavailable")),
          logLines(env).slice(-2).join(" | "));
    check("409 后状态=未连接", byId(env, "status").textContent === "Disconnected",
          byId(env, "status").textContent);
    check("真实目标持久化 xrk.motorTarget",
          env.localStorage.getItem("xrk.motorTarget") === "serial:COM5@115200",
          env.localStorage.getItem("xrk.motorTarget"));
    check("非法目标不下发 open", (() => {
        const before = env.net.calls.filter((c) => c.url === "/api/open").length;
        byId(env, "target").value = "udp:nonsense";
        byId(env, "btn").click();
        return env.net.calls.filter((c) => c.url === "/api/open").length === before;
    })());
    await sleep(60);
    check("非法目标错误進日志", logLines(env).some((l) => l.includes("udp:nonsense")),
          logLines(env).slice(-1)[0]);

    /* ════════ P) pidscan 单槽页：slot=扫描组号 入同一行 ════════ */
    const env2 = makeEnv();
    const loaded2 = loadPage(env2, shells.exp_motor_pidscan.cfg);
    await sleep(60);
    check("pidscan 单槽表 1 行", tableParts(env2, "slots").rows.length === 1,
          `rows=${tableParts(env2, "slots").rows.length}`);
    loaded2.page.state.connected = true;
    env2.emit("mframe", { type: 5, seq: 1, scen: null, timers: null, recs: [
        { slot: 5, kind: 2, sp: 90, duty: 0.72, actual: 88.5, phase: 1, fault: 0 }] });
    await sleep(60);
    const prow = tableParts(env2, "slots").rows[0].children.map((c) => c.textContent);
    check("pidscan 槽位列 = 当前扫描组号", prow[0] === "5", JSON.stringify(prow));
    check("pidscan sp/actual 更新", prow[1] === "90.000" && prow[3] === "88.500", JSON.stringify(prow));
    check("pidscan 波形点数 +1", loaded2.page.inspect().wavePoints === 1,
          `wavePoints=${loaded2.page.inspect().wavePoints}`);
    check("pidscan 场景行提示 0x05 遥测（无 SCEN 头）",
          byId(env2, "scen-line").textContent.includes("0x05"),
          byId(env2, "scen-line").textContent);

    /* ════════ 汇总 ════════ */
    if (fails.length) {
        console.log(`== motor_page_harness: ${fails.length} FAIL ==`);
        fails.forEach((f) => console.log("  [FAIL]", f));
        process.exit(1);
    }
    console.log("== motor_page_harness: 0 fail（7 薄壳结构 + 词典对齐 + 显式启动/双子语/验收/409/pidscan）==");
    process.exit(0);
})().catch((err) => {
    console.log("== motor_page_harness: 异常终止 ==");
    console.log(err && err.stack ? err.stack : err);
    process.exit(1);
});
