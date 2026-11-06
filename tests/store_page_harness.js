/* store_page_harness.js — store_demo 实例目录页面契约测试（node，无浏览器）。
 *
 * 被测对象：
 *   web/assets/store-page.js         页面核心（薄壳 #app 自建 DOM）
 *   examples/store_demo/index.html   薄壳（只声明 window.STORE_PAGE + #app + 三个资产）
 *
 * 做法：从 HTML 抽内联薄壳（vm 沙箱取 STORE_PAGE），再桩 DOM/EventSource/fetch/
 * localStorage require ui-shell → live → store-page，驱动 SSE 与按钮点击。
 *
 * 断言（T-018 口径）：
 *   S) 薄壳结构：内联块 + ui-shell/live/store-page 三脚本 + #app；id 与注册表一致
 *   D) 词典 zh/en 键一一对应；data-i18n 锚点全部可解析
 *   1) 加载不启动任何数据源（零 POST /api/open；只读 /api/state、/api/ports；订阅 SSE）
 *   2) sim 档启动 → POST /api/open {"mode":"sim"}；TEXT 行（boot 日志）入页
 *   3) GET 按钮 → POST /api/store/cmd {"func":"get"} → 快照面板填充（boot 等字段）
 *   4) RESET 按钮 → POST /api/store/cmd {"func":"set","args":{"kv":{"boot":0}}}
 *   5) STATUS 按钮 → POST /api/store/cmd {"func":"status"} → 状态行填充
 *   6) 命令 409 → 日志记失败（诚实反馈，不禁用按钮）
 *   7) 切 en → 按钮/标题变英文；模式显式（模拟禁止隐式回落）
 *
 * 跑法：node tests/store_page_harness.js（exit 0 = 全过；需 node>=18）
 */

"use strict";
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const ROOT = path.join(__dirname, "..");
const SHELL = path.join(ROOT, "web", "assets", "ui-shell.js");
const LIVE = path.join(ROOT, "web", "assets", "live.js");
const PAGE_JS = path.join(ROOT, "web", "assets", "store-page.js");
const INDEX = path.join(ROOT, "examples", "store_demo", "index.html");

const fails = [];
function check(name, ok, detail) {
    if (!ok) fails.push(`${name}${detail ? "：" + detail : ""}`);
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/* ─────────────── S) 薄壳结构 ─────────────── */
const html = fs.readFileSync(INDEX, "utf8");
const inline = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map((m) => m[1]);
const srcs = [...html.matchAll(/<script\s+src="([^"]+)"><\/script>/g)].map((m) => m[1]);
const sandbox = { window: {} };
inline.forEach((code) => vm.runInNewContext(code, sandbox, { timeout: 1000 }));
const cfg = sandbox.window.STORE_PAGE || {};

check("store_demo 页面存在", html.length > 0);
check("单一内联薄壳块", inline.length === 1, `blocks=${inline.length}`);
check("加载三个共享资产",
      JSON.stringify(srcs) ===
      JSON.stringify(["/assets/ui-shell.js", "/assets/live.js", "/assets/store-page.js"]),
      JSON.stringify(srcs));
check("宿主容器 #app", /id="app"/.test(html));
check("STORE_PAGE.id 与注册表一致", cfg.id === "store_demo", `id=${cfg.id}`);
check("标题兜底文案", typeof cfg.title === "string" && cfg.title.length > 0);

/* ─────────────── 桩环境 ─────────────── */
function makeEnv() {
    const els = [];
    const ids = new Map();

    function el(tag) {
        const e = {
            tagName: String(tag).toUpperCase(), id: "", className: "", type: "", value: "",
            textContent: "", innerHTML: "", hidden: false, disabled: false, title: "",
            scrollTop: 0, scrollHeight: 0, attrs: {}, style: {}, handlers: {}, children: [],
            appendChild(c) { e.children.push(c); return c; },
            removeChild(c) {
                const i = e.children.indexOf(c);
                if (i >= 0) e.children.splice(i, 1);
                return c;
            },
            setAttribute(k, v) { e.attrs[k] = String(v); },
            getAttribute(k) {
                return Object.prototype.hasOwnProperty.call(e.attrs, k) ? e.attrs[k] : null;
            },
            addEventListener(t, fn) { (e.handlers[t] = e.handlers[t] || []).push(fn); },
            click() {
                const ev = { target: e, type: "click" };
                if (typeof e.onclick === "function") e.onclick(ev);
                (e.handlers.click || []).forEach((fn) => fn(ev));
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

    const data = new Map();
    const localStorage = {
        getItem(k) { return data.has(k) ? data.get(k) : null; },
        setItem(k, v) { data.set(k, String(v)); },
        removeItem(k) { data.delete(k); },
    };

    const net = {
        calls: [],
        open: { status: 200, error: null },
        storeGet: { payload: { boot: 6, gain: 3.25, flags: 0, note: [0, 0, 0, 0, 0, 0, 0, 0],
                               load_state: 1, frame_len: 70, dirty: 0, capacity_kb: 4 } },
        storeStatus: { payload: { load_state: 1, frame_len: 70, dirty: 0, entries: 4,
                                  capacity_kb: 4 } },
        txt: [],
    };
    function baseState(extra) {
        return Object.assign({ kind: "store", source: "none", target: "",
                               connected: false, counters: { text: 0 } }, extra || {});
    }
    function respond(status, data) {
        return { ok: status >= 200 && status < 300, status,
                 json: () => Promise.resolve(data) };
    }
    const fetchStub = (url, opts) => {
        const body = opts && opts.body ? JSON.parse(opts.body) : null;
        const call = { url: String(url), method: (opts && opts.method) || "GET", body };
        net.calls.push(call);
        if (call.method === "GET" && call.url === "/api/state") {
            return Promise.resolve(respond(200, baseState()));
        }
        if (call.method === "GET" && call.url === "/api/ports") {
            return Promise.resolve(respond(200, { ports: [{ device: "COM5", desc: "USB serial" }] }));
        }
        if (call.method === "POST" && call.url === "/api/open") {
            if (net.open.status !== 200) {
                return Promise.resolve(respond(net.open.status, { error: net.open.error }));
            }
            return Promise.resolve(respond(200, baseState({
                connected: true, source: body.mode === "sim" ? "sim" : "real",
                target: body.target || "",
            })));
        }
        if (call.method === "POST" && call.url === "/api/close") {
            return Promise.resolve(respond(200, baseState()));
        }
        if (call.method === "POST" && call.url === "/api/store/cmd") {
            if (net.open.status !== 200) {
                return Promise.resolve(respond(409, { error: net.open.error }));
            }
            if (body.func === "get") {
                return Promise.resolve(respond(200, { func: "get", status: 0,
                                                      payload: net.storeGet.payload }));
            }
            if (body.func === "set") {
                return Promise.resolve(respond(200, { func: "set", status: 0, payload: {} }));
            }
            if (body.func === "status") {
                return Promise.resolve(respond(200, { func: "status", status: 0,
                                                      payload: net.storeStatus.payload }));
            }
            return Promise.resolve(respond(409, { error: "未知 func " + body.func }));
        }
        return Promise.resolve(respond(404, { error: "no stub for " + call.url }));
    };

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

function loadPage(env) {
    globalThis.document = env.document;
    globalThis.localStorage = env.localStorage;
    globalThis.EventSource = env.EventSource;
    globalThis.fetch = env.fetchStub;
    globalThis.STORE_PAGE = cfg;
    delete require.cache[require.resolve(SHELL)];
    delete require.cache[require.resolve(LIVE)];
    delete require.cache[require.resolve(PAGE_JS)];
    const UI = require(SHELL);
    globalThis.UI = UI;
    require(LIVE);
    const page = require(PAGE_JS);
    return { UI, page };
}

function byId(env, id) { return env.document.getElementById(id); }
function logLines(env) {
    return byId(env, "log").children.map((c) => c.textContent);
}
const storeCmds = (env) => env.net.calls.filter((c) => c.url === "/api/store/cmd");

(async function main() {
    const env = makeEnv();
    const { UI } = loadPage(env);
    await sleep(80);

    /* D) 词典 + 锚点闭合 */
    const zhKeys = Object.keys(UI.dict("zh")).sort();
    const enKeys = Object.keys(UI.dict("en")).sort();
    const zhOnly = zhKeys.filter((k) => !enKeys.includes(k));
    const enOnly = enKeys.filter((k) => !zhKeys.includes(k));
    check("词典 zh/en 键一一对应", zhOnly.length === 0 && enOnly.length === 0,
          `zhOnly=${zhOnly.join(",")} enOnly=${enOnly.join(",")}`);
    const anchors = env.document.querySelectorAll(
        "[data-i18n],[data-i18n-html],[data-i18n-ph],[data-i18n-title]");
    const unresolved = anchors
        .map((e) => e.getAttribute("data-i18n") || e.getAttribute("data-i18n-ph")
                 || e.getAttribute("data-i18n-title") || e.getAttribute("data-i18n-html"))
        .filter((k) => k && !UI.has(k));
    check("data-i18n 锚点全部可解析", unresolved.length === 0, unresolved.join(","));
    check("锚点非空（三按钮+两个面板头）", anchors.length >= 5, `anchors=${anchors.length}`);

    /* 1) 加载不启动任意数据源 */
    check("加载后零 POST /api/open",
          env.net.calls.every((c) => c.url !== "/api/open"), JSON.stringify(env.net.calls));
    check("加载即订阅 /api/stream",
          env.streams.length === 1 && env.streams[0].url === "/api/stream",
          `streams=${env.streams.map((s) => s.url).join(",")}`);
    check("加载状态行=未连接", byId(env, "status").textContent === "未连接",
          byId(env, "status").textContent);
    check("三按钮渲染（zh 冒烟）", byId(env, "btn-get").textContent === "GET 快照",
          byId(env, "btn-get").textContent);

    /* 2) sim 档启动 → open{"mode":"sim"} → TEXT boot 行入日志 */
    const simBtn = byId(env, "modebar").children.find((b) => b.textContent === "仿真");
    check("模式分段含「仿真」", !!simBtn);
    simBtn.click();
    await sleep(40);
    check("点「仿真」切到 sim 模式", UI.mode === "sim", `mode=${UI.mode}`);
    byId(env, "btn").click();
    await sleep(70);
    const opens = env.net.calls.filter((c) => c.url === "/api/open");
    check("点「启动仿真」恰一次 POST /api/open", opens.length === 1, `n=${opens.length}`);
    check("open body 恰为 {\"mode\":\"sim\"}",
          opens.length === 1 && JSON.stringify(opens[0].body) === '{"mode":"sim"}',
          opens.length ? JSON.stringify(opens[0].body) : "no call");
    check("SSE state 推连接 → 状态行=已连接",
          (env.emit("state", { connected: true }), byId(env, "status").textContent) === "已连接",
          byId(env, "status").textContent);
    env.emit("text", "OK store boot: count=1 load=0 len=70 kb=4");
    await sleep(40);
    check("TEXT boot 行入日志",
          logLines(env).some((l) => l.includes("count=1") && l.includes("load=0")),
          JSON.stringify(logLines(env)));

    /* 3) GET → /api/store/cmd {"func":"get"} → 快照面板填充 */
    byId(env, "btn-get").click();
    await sleep(70);
    const gets = storeCmds(env).filter((c) => c.body.func === "get");
    check("GET 恰一次 func=get", gets.length === 1, `n=${gets.length}`);
    check("GET 无参（args 可为空对象）",
          gets.length === 1 &&
          JSON.stringify(gets[0].body) === '{"func":"get","args":{}}',
          JSON.stringify(gets[0] && gets[0].body));
    const snap = byId(env, "snap");
    check("快照面板含 boot=6", /boot 计数=6/.test(snap.textContent), snap.textContent);
    check("快照面板含恢复语义", /恢复/.test(snap.textContent), snap.textContent);

    /* 4) RESET → set boot=0 */
    byId(env, "btn-reset").click();
    await sleep(60);
    const sets = storeCmds(env).filter((c) => c.body.func === "set");
    check("RESET 恰一次 func=set", sets.length === 1, `n=${sets.length}`);
    check("RESET body = {\"func\":\"set\",\"args\":{\"kv\":{\"boot\":0}}}",
          sets.length === 1 &&
          JSON.stringify(sets[0].body) === '{"func":"set","args":{"kv":{"boot":0}}}',
          sets.length ? JSON.stringify(sets[0].body) : "no call");

    /* 5) STATUS → 状态行填充 */
    byId(env, "btn-status").click();
    await sleep(60);
    const sts = storeCmds(env).filter((c) => c.body.func === "status");
    check("STATUS 恰一次 func=status", sts.length === 1, `n=${sts.length}`);
    const row = byId(env, "row");
    check("状态行含 entries=4", /entries=4/.test(row.textContent), row.textContent);

    /* 6) 命令 409 → 失败日志（诚实反馈） */
    env.net.open.status = 409;
    env.net.open.error = "unreachable target";
    byId(env, "btn-get").click();
    await sleep(60);
    check("409 → 命令失败日志",
          logLines(env).some((l) => l.includes("失败") && l.includes("unreachable")),
          JSON.stringify(logLines(env)));

    /* 7) 切 en → 按钮英文 */
    UI.setLang("en");
    await sleep(40);
    check("切 en → GET 按钮英文", byId(env, "btn-get").textContent === "GET snapshot",
          byId(env, "btn-get").textContent);
    env.emit("text", "OK store boot: count=2 load=1 len=70 kb=4");
    await sleep(20);
    check("TEXT 日志英文模式仍记录",
          logLines(env).some((l) => l.includes("count=2")), JSON.stringify(logLines(env)));

    console.log(fails.length ? fails.map((f) => "  ✗ " + f).join("\n") : "store_page_harness: ALL PASS");
    process.exit(fails.length ? 1 : 0);
})();