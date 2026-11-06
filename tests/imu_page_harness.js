/* imu_page_harness.js — IMU 统一页面核心契约测试（node，无浏览器/无 GPU）。
 *
 * 背景（用户定案 2026-11-06）：五个 IMU 实验页「逐级继承」——
 *   exp1 裸波形 ⊂ exp2（+零点校准）⊂ exp3（+欧拉，独立页保留）
 *                ⊂ exp4（+四元数姿态）⊂ exp5（+位移轨迹）。
 * 服务器侧只剩四份薄壳（exp1/exp2/exp4/exp5），面板集合写在薄壳的
 * window.IMU_PAGE.panels 里，页面核心 web/assets/imu-page.js 按它渲染。
 *
 * 断言分组：
 *   1) 薄壳 = 薄壳：四份 index.html 只声明 id/panels + 四个共享脚本，无内联实现
 *   2) 逐级包含矩阵：panels 集合必须恰好等于规定集合，且 exp1 ⊂ exp2 ⊂ exp4 ⊂ exp5
 *   3) 核心按声明渲染：声明过的面板有标题（渲染了），没声明的面板一片空白
 *   4) 词典键对齐：imu.* 前缀 zh/en 成对、关键键可取词（缺键会返回键名）
 *   5) 禁自动启动：加载后无 openSim/openPort/sendCalib，模式 real / 语言 zh
 *   6) 调零按钮：POST /api/calib（sendCalib）+ 回执按词典翻译 + 统计窗重开
 *   7) 0x02 帧 → cal 面板 raw/deb/bias 六轴 + 温度 + 统计窗
 *   8) 0x03 帧 → euler 三轴条形（含 pitch 锁边界虚线）+ 数值表
 *   9) 0x04 帧 → quat 数值/范数/立方参数序 + 欧拉对照表
 *  10) pos → traj 渐隐轨迹 + 标记位置（exp5）
 *  11) 双语：切 en 后页面标题与全部面板标题变英文（无残留中文）
 *  12) 波形泵契约（raw 面板）：抖动缓冲 / 匀速消费 / 亚像素相位（exp1 引擎搬入）
 *  13) 断开清窗：portChanged(false) → 面板回 — / 轨迹清空
 *
 * 本文件同时导出 makeEnv()：exp1_wave/exp2/exp4/exp5 四个既有 harness 用同一套桩
 * 加载真实页面（薄壳 + 核心），避免四份桩代码漂移。
 * 跑法：node tests/imu_page_harness.js（exit 0 = 全过；需 node>=18）
 */

"use strict";
const fs = require("fs");
const path = require("path");

const ROOT = path.join(__dirname, "..");
const CORE = path.join(ROOT, "web", "assets", "imu-page.js");

const SHELLS = {
    exp1: path.join("examples", "imu", "exp1_waveform", "index.html"),
    exp2: path.join("examples", "imu", "exp2_bias", "index.html"),
    exp4: path.join("examples", "imu", "exp4_quat_view", "index.html"),
    exp5: path.join("examples", "imu", "exp5_trajectory", "index.html"),
};
const ALL_PANELS = ["raw", "cal", "euler", "quat", "traj"];
const REQUIRED = {                       // 逐级继承的规定集合（缺一即 bug）
    exp1: ["raw"],
    exp2: ["raw", "cal"],
    exp4: ["raw", "cal", "euler", "quat"],
    exp5: ["raw", "cal", "euler", "quat", "traj"],
};

/* ───────────────────────── 薄壳解析 ───────────────────────── */

function readShell(pageId) {
    const file = path.join(ROOT, SHELLS[pageId]);
    const html = fs.readFileSync(file, "utf8");
    const m = html.match(/window\.IMU_PAGE\s*=\s*(\{[\s\S]*?\})\s*;/);
    if (!m) return { file, html, cfg: null, inline: null };
    let cfg = null;
    try {
        cfg = eval("(" + m[1] + ")");      // 薄壳里的字面量配置（测试侧解析）
    } catch (e) {
        return { file, html, cfg: null, inline: m[0], error: e.message };
    }
    return { file, html, cfg, inline: m[0] };
}

/* ───────────────────────── DOM / THREE / 桥 桩 ───────────────────────── */

function makeClassList() {
    const set = new Set();
    return {
        toggle(cls, on) {
            if (on === undefined) { set.has(cls) ? set.delete(cls) : set.add(cls); }
            else if (on) set.add(cls); else set.delete(cls);
        },
        add(cls) { set.add(cls); },
        remove(cls) { set.delete(cls); },
        has(cls) { return set.has(cls); },
    };
}

function makeEnv(pageId, opts) {
    const shell = readShell(pageId);
    const webglThrows = !!(opts && opts.webglThrows);
    const els = {};
    const rec = { fills: [], dashes: [], clears: 0 };

    function ctxStub() {
        return {
            fillStyle: "", strokeStyle: "", font: "", textAlign: "", textBaseline: "",
            beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}, fillRect() {},
            clearRect() { rec.clears++; },
            setLineDash(d) { if (d && d.length) rec.dashes.push(d.slice()); },
            fillText(text) { rec.fills.push(String(text)); },
        };
    }

    function stubEl(id) {
        const el = {
            id: id || "", textContent: "", innerHTML: "", value: "", title: "",
            className: "", disabled: false, scrollTop: 0, scrollHeight: 0,
            width: 520, height: 170, clientWidth: 520, clientHeight: 170,
            children: [], style: {}, classList: makeClassList(),
            addEventListener() {}, click() {},
            appendChild(c) { el.children.push(c); return c; },
            removeChild(c) {
                const i = el.children.indexOf(c);
                if (i >= 0) el.children.splice(i, 1);
            },
            getAttribute() { return null; }, setAttribute() {},
            getContext(kind) { return kind === "2d" ? ctxStub() : null; },
        };
        el.options = el.children;       // <select> 桩：options 即 children
        return el;
    }

    const headEl = stubEl("head");
    const bodyEl = stubEl("body");
    const documentStub = {
        title: "",
        head: headEl,
        body: bodyEl,
        documentElement: { setAttribute() {} },
        getElementById(id) {
            if (id === "head") return headEl;
            if (id === "body") return bodyEl;
            return (els[id] = els[id] || stubEl(id));
        },
        createElement(tag) {
            if (tag === "canvas") return stubEl("canvas");
            return stubEl(tag);
        },
        querySelectorAll() { return []; },
    };

    /* —— THREE 桩：页面用到的表面（波形 / 立方 / 轨迹）—— */
    const reg = { geoms: [], groups: [], quats: [], cams: [] };
    class Vec3 {
        constructor(x, y, z) { this.x = x || 0; this.y = y || 0; this.z = z || 0; }
        set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
    }
    class Quaternion {
        constructor() { reg.quats.push(this); this._args = null; }
        set(x, y, z, w) { this._args = [x, y, z, w]; return this; }
        normalize() { return this; }
    }
    class Obj3 {
        constructor() {
            this.position = new Vec3(0, 0, 0);
            this.scale = new Vec3(1, 1, 1);
            this.quaternion = new Quaternion();
            this.children = [];
            this.rotation = { x: 0, y: 0, z: 0 };
        }
        add(c) { this.children.push(c); return this; }
    }
    class Attr {
        constructor(arr, size) { this.array = arr; this.itemSize = size; this.needsUpdate = false; }
        setUsage() { return this; }
    }
    class Geom {
        constructor() { reg.geoms.push(this); this._attrs = {}; this._range = [0, 0]; }
        setAttribute(n, a) { this._attrs[n] = a; return this; }
        setDrawRange(o, c) { this._range = [o, c]; }
    }
    class Line extends Obj3 {
        constructor(g, m) { super(); this.geometry = g; this.material = m; }
    }
    class LineSegments extends Line {}
    class Mesh extends Line {}
    class Group extends Obj3 { constructor() { super(); reg.groups.push(this); } }
    class Camera extends Obj3 {
        constructor() { super(); reg.cams.push(this); this.aspect = 1; this.up = new Vec3(0, 1, 0); }
        updateProjectionMatrix() {}
        lookAt() {}
    }
    let Renderer = class {
        constructor() { this.renders = 0; }
        setPixelRatio() {} setSize() {} setClearColor() {}
        render() { this.renders++; }
    };
    const noop = class { constructor() {} };
    if (webglThrows) {                 // 无 GPU/offscreen 环境：WebGLRenderer 构造即抛
        Renderer = class { constructor() { throw new Error("no webgl"); } };
    }
    const THREE = {
        Vec3, Quaternion, Group, Line, LineSegments, Mesh,
        BufferGeometry: Geom, BoxGeometry: Geom, EdgesGeometry: Geom, SphereGeometry: Geom,
        BufferAttribute: Attr, DynamicDrawUsage: 35048,
        LineBasicMaterial: noop, MeshBasicMaterial: noop, ShaderMaterial: noop,
        SpriteMaterial: noop, CanvasTexture: noop, Color: noop,
        GridHelper: Obj3, AxesHelper: Obj3, Sprite: Obj3,
        Scene: Obj3, OrthographicCamera: Camera, PerspectiveCamera: Camera,
        WebGLRenderer: Renderer,
    };

    /* —— 桥桩（LiveBridge 同形；信号 handlers 可驱动）—— */
    function stubSignal() {
        const handlers = [];
        return { connect(fn) { handlers.push(fn); }, _handlers: handlers };
    }
    const bridgeCalls = [];
    let calibReply = { code: "calibSimulated", cmd: "CAL" };   // 可改（测回执带 payload / 失败）
    const bridgeStub = {
        listPorts: () => Promise.resolve(["COM3", "COM99"]),
        openPort: (p, b) => { bridgeCalls.push(["openPort", p, b]); return Promise.resolve({}); },
        closePort: () => { bridgeCalls.push(["closePort"]); return Promise.resolve({}); },
        openSim: () => { bridgeCalls.push(["openSim"]); return Promise.resolve({}); },
        sendCalib: () => {
            bridgeCalls.push(["sendCalib"]);
            return Promise.resolve(calibReply);
        },
        frameReady: stubSignal(), textReady: stubSignal(), statReady: stubSignal(),
        portChanged: stubSignal(), posReady: stubSignal(),
    };

    /* —— fetch 桩（页面命令通道 POST /api/imu/cmd 用）——
       env.net.calls 记录请求；env.net.routes[func] 应答 {status, body}（可返回
       Promise — 用于挂起/超时分支；未登记的路由回空对象）。 */
    const net = { calls: [], routes: {} };
    function stubFetch(url, opts) {
        let body = null;
        try { body = opts && opts.body ? JSON.parse(opts.body) : null; } catch (e) { body = null; }
        net.calls.push({ url: String(url), method: (opts && opts.method) || "GET", body });
        const key = (body && body.func) || "*";
        const handler = net.routes[key];
        const resp = handler ? handler(body) : { status: 200, body: {} };
        if (resp && typeof resp.then === "function") return resp;   // 挂起/异步路由
        const status = resp.status || 200;
        return Promise.resolve({
            ok: status < 400, status,
            json: () => Promise.resolve(resp.body || {}),
        });
    }
    globalThis.fetch = stubFetch;

    /* —— 全局装配（页面脚本按浏览器脚本执行：root = globalThis）—— */
    const rafQueue = [];
    globalThis.window = globalThis;          // 浏览器里 window === globalThis
    globalThis.document = documentStub;
    globalThis.THREE = THREE;
    globalThis.devicePixelRatio = 1;
    globalThis.requestAnimationFrame = (fn) => { rafQueue.push(fn); return 0; };
    delete require.cache[require.resolve(path.join(ROOT, "web", "assets", "ui-shell.js"))];
    globalThis.UI = require(path.join(ROOT, "web", "assets", "ui-shell.js"));
    globalThis.UI._reset();
    globalThis.LiveBridge = () => bridgeStub;
    globalThis.IMU_PAGE = shell.cfg || undefined;
    delete require.cache[require.resolve(CORE)];

    let bootError = null;
    try {
        require(CORE);
    } catch (e) {
        bootError = e;
    }

    const env = {
        pageId, shell, els, reg, rec, bridgeCalls, net, bootError,
        UI: globalThis.UI,
        setCalibReply(v) { calibReply = v; return env; },
        /* 桩挂在 globalThis 上：切回本 env 的 document/UI（ui-shell 懒取 root.document） */
        activate() {
            globalThis.document = documentStub;
            globalThis.UI = env.UI;
            globalThis.THREE = THREE;
            globalThis.fetch = stubFetch;
            globalThis.LiveBridge = () => bridgeStub;
            globalThis.IMU_PAGE = shell.cfg || undefined;
            return env;
        },
        get debug() { return globalThis.__xrkDebug || null; },
        sig(name) { return bridgeStub[name]._handlers; },
        emit(name, ...args) { env.sig(name).forEach((fn) => fn(...args)); },
        pumpFrame(t) { rafQueue.splice(0).forEach((fn) => fn(t)); },
    };
    return env;
}

/* ───────────────────────── 断言主体（本文件直接跑时的全集） ───────────────────── */

function createRun() {
    const fails = [];
    return {
        fails,
        check(name, ok, detail) {
            if (!ok) fails.push(`${name}${detail ? "：" + detail : ""}`);
        },
    };
}

function runSuite() {
    const { fails, check } = createRun();
    const near = (a, b, eps) => Math.abs(a - b) <= (eps === undefined ? 1e-9 : eps);
    const hasCJK = (s) => /[\u4e00-\u9fff]/.test(s || "");
    /* 未渲染的面板元素根本不会被页面核心取用（桩按需创建）→ 缺失即「未渲染」 */
    const txt = (env, id) => (env.els[id] ? env.els[id].textContent : "");
    const html = (env, id) => (env.els[id] ? env.els[id].innerHTML : "");

    /* 1) 薄壳 = 薄壳：只声明配置 + 四个共享脚本 */
    const shells = {};
    Object.keys(SHELLS).forEach((id) => {
        const s = readShell(id);
        shells[id] = s;
        check(`${id} 薄壳配置可解析`, !!s.cfg, s.error || "window.IMU_PAGE 缺失");
        if (!s.cfg) return;
        check(`${id} 薄壳 id`, s.cfg.id === id, `cfg.id=${s.cfg.id}`);
        const inline = s.html.match(/<script>([\s\S]*?)<\/script>/g) || [];
        check(`${id} 只有一段内联脚本`, inline.length === 1, `inline=${inline.length}`);
        check(`${id} 内联脚本只声明 IMU_PAGE`,
              inline.length === 1 && /IMU_PAGE/.test(inline[0])
              && !/function/.test(inline[0]),
              `inline=${(inline[0] || "").slice(0, 80)}`);
        ["ui-shell.js", "live.js", "imu-page.js"].forEach((asset) => {
            check(`${id} 载入 ${asset}`, s.html.includes("/assets/" + asset));
        });
        check(`${id} 载入 three 本地副本`,
              s.html.includes("/assets/vendor/three.min.js"));
        check(`${id} 无内联实现残留`, !/function\s+\w+\(/.test(s.html),
              "薄壳里出现了函数定义（实现应全部在 imu-page.js）");
    });

    /* 2) 逐级包含矩阵：集合恰好等于规定，且 exp1 ⊂ exp2 ⊂ exp4 ⊂ exp5 */
    const order = ["exp1", "exp2", "exp4", "exp5"];
    order.forEach((id) => {
        const cfg = shells[id].cfg;
        if (!cfg) return;
        const got = (cfg.panels || []).slice();
        const want = REQUIRED[id];
        check(`${id} 面板集合 = ${want.join("+")}`,
              JSON.stringify(got.slice().sort()) === JSON.stringify(want.slice().sort()),
              `got=${JSON.stringify(got)}`);
        got.forEach((p) => check(`${id} 面板名合法 ${p}`, ALL_PANELS.indexOf(p) >= 0));
    });
    for (let i = 1; i < order.length; i++) {
        const lo = shells[order[i - 1]].cfg, hi = shells[order[i]].cfg;
        const missing = (lo && lo.panels || []).filter((p) => (hi && hi.panels || []).indexOf(p) < 0);
        check(`${order[i]} 继承 ${order[i - 1]} 的全部面板`, missing.length === 0,
              `缺=${JSON.stringify(missing)}`);
        const added = (hi && hi.panels || []).filter((p) => (lo && lo.panels || []).indexOf(p) < 0);
        check(`${order[i]} 比 ${order[i - 1]} 多面板`, added.length >= 1,
              `多出=${JSON.stringify(added)}`);
    }

    /* 3/4/5) 逐页起页面：渲染集合 + 词典 + 禁自动启动 */
    const envs = {};
    order.forEach((id) => {
        const env = makeEnv(id);
        envs[id] = env;
        check(`${id} 页面核心加载不崩`, !env.bootError,
              env.bootError ? env.bootError.message : "");
        if (env.bootError) return;
        REQUIRED[id].forEach((p) => {
            check(`${id} 渲染了面板 ${p}`, txt(env, "cap-" + p).length > 0,
                  `cap-${p}=${JSON.stringify(txt(env, "cap-" + p))}`);
        });
        ALL_PANELS.filter((p) => REQUIRED[id].indexOf(p) < 0).forEach((p) => {
            check(`${id} 不渲染未声明面板 ${p}`, txt(env, "cap-" + p) === "",
                  `cap-${p}=${JSON.stringify(txt(env, "cap-" + p))}`);
        });
        check(`${id} 加载后未启源（禁隐式回落）`,
              env.bridgeCalls.length === 0, JSON.stringify(env.bridgeCalls));
        check(`${id} 缺省真实模式`, env.UI.mode === "real", `mode=${env.UI.mode}`);
        check(`${id} 缺省中文`, env.UI.lang === "zh", `lang=${env.UI.lang}`);
        check(`${id} startSim/autoConnect kick 契约`,
              typeof globalThis.startSim === "function" &&
              typeof globalThis.autoConnect === "function");
        check(`${id} 状态行=未连接`, env.els["status"].textContent === "未连接",
              `status=${env.els["status"].textContent}`);
    });

    /* 4) 词典键对齐（imu.* 前缀；zh/en 成对且可取词） */
    const UI = envs.exp5.UI;
    const zh = UI.dict("zh"), en = UI.dict("en");
    const zhKeys = Object.keys(zh).filter((k) => k.indexOf("imu.") === 0).sort();
    const enKeys = Object.keys(en).filter((k) => k.indexOf("imu.") === 0).sort();
    check("imu.* 词条数量 > 30", zhKeys.length > 30, `zh=${zhKeys.length}`);
    check("imu.* zh/en 键集合一致",
          JSON.stringify(zhKeys) === JSON.stringify(enKeys),
          `zh-only=${zhKeys.filter((k) => enKeys.indexOf(k) < 0)} ` +
          `en-only=${enKeys.filter((k) => zhKeys.indexOf(k) < 0)}`);
    zhKeys.forEach((k) => {
        check(`词条非空 ${k}`, String(zh[k]).length > 0 && String(en[k]).length > 0);
    });
    ["imu.panel.raw", "imu.panel.cal", "imu.panel.euler", "imu.panel.quat",
     "imu.panel.traj", "imu.title.exp1", "imu.title.exp2", "imu.title.exp4",
     "imu.title.exp5", "imu.msg.calibSimulated", "imu.msg.calibSent",
     "imu.msg.notConnected"].forEach((k) => {
        check(`键可取词 ${k}`, UI.t(k) !== k && UI.t(k).length > 0);
    });

    /* 6) 调零按钮：POST /api/calib + 回执翻译 + 统计窗重开（exp2/exp4/exp5 全支持） */
    ["exp2", "exp4", "exp5"].forEach((id) => {
        const env = envs[id];
        if (env.bootError) return;
        check(`${id} 有调零按钮`, typeof env.els["btn-cal"].onclick === "function");
        env.els["btn-cal"].onclick();
        check(`${id} 调零 → sendCalib（POST /api/calib）`,
              env.bridgeCalls.some((c) => c[0] === "sendCalib"),
              JSON.stringify(env.bridgeCalls));
        const statTitle = env.els["stat-title"].textContent;
        check(`${id} 调零重开统计窗`, statTitle.includes("0/300"), statTitle);
    });
    check("exp1 无调零按钮（能力未声明）",
          !envs.exp1.els["btn-cal"] || typeof envs.exp1.els["btn-cal"].onclick !== "function");

    /* 标定回执：TEXT 里 OK calib 行高亮（class calok），其余日志行不误标 */
    {
        const env = envs.exp2;
        env.emit("textReady", "OK sim: static Z_UP 100Hz");
        env.emit("textReady", "OK calib done bias ax=20 ay=-30 az=15 mg temp=25000 mC");
        const rows = env.els["log"].children;
        const ok = rows.filter((r) => (r.className || "") === "calok");
        check("OK calib 日志行高亮", ok.length === 1 && /OK calib/.test(ok[0].textContent),
              JSON.stringify(rows.map((r) => [r.className, r.textContent.slice(0, 20)])));
    }

    /* 7) 0x02 帧 → cal 面板六轴 + 温度 + 统计窗（exp2 起全部页都有） */
    const calVals = [0.02, -0.03, 1.015, 1.25, -0.5, 2.5,        // raw6
                     0.0, 0.0, 1.0, 1.225, -0.5, 2.5,            // deb6
                     0.02, -0.03, 0.015, 0.025, 0.0, 0.0,        // bias6
                     27.5];                                      // temp
    ["exp2", "exp4", "exp5"].forEach((id) => {
        const env = envs[id];
        if (env.bootError) return;
        env.emit("frameReady", 2, 1, calVals);
        check(`${id} cal raw 列`, env.els["r-az"].textContent === "1.0150",
              `r-az=${env.els["r-az"].textContent}`);
        check(`${id} cal deb 列`, env.els["d-az"].textContent === "1.0000",
              `d-az=${env.els["d-az"].textContent}`);
        check(`${id} cal bias 列`, env.els["b-gx"].textContent === "0.0250",
              `b-gx=${env.els["b-gx"].textContent}`);
        check(`${id} cal 温度`, env.els["temp"].innerHTML.includes("27.5000"),
              `temp=${env.els["temp"].innerHTML}`);
        check(`${id} cal 统计窗计数`, env.els["stat-title"].textContent.includes("窗 1/300")
              || env.els["stat-title"].textContent.includes("1/300"),
              env.els["stat-title"].textContent);
        check(`${id} cal 统计表均值`, env.els["m-az"].textContent.startsWith("1.0150"),
              `m-az=${env.els["m-az"].textContent}`);
    });
    check("exp1 缺省不显示 cal 面板（未声明）", txt(envs.exp1, "r-az") === "");

    /* 8) 0x03 帧 → euler 三轴条形（+ pitch 锁边界虚线 + 数值表） */
    const eulerRad = [0.1, -0.2, 0.3];                 // rad
    ["exp4", "exp5"].forEach((id) => {
        const env = envs[id];
        if (env.bootError) return;
        env.rec.fills.length = 0;
        env.rec.dashes.length = 0;
        env.emit("frameReady", 3, 1, eulerRad);
        check(`${id} euler deg 列`, env.els["v-roll"].textContent.startsWith("+5.730"),
              `v-roll=${env.els["v-roll"].textContent}`);
        check(`${id} euler rad 列`, env.els["r-pitch"].textContent === "-0.2000",
              `r-pitch=${env.els["r-pitch"].textContent}`);
        check(`${id} euler 峰值列`, env.els["p-yaw"].textContent.startsWith("+17.189"),
              `p-yaw=${env.els["p-yaw"].textContent}`);
        const fills = env.rec.fills.join("|");
        ["roll", "pitch", "yaw"].forEach((a) => {
            check(`${id} 条形画了 ${a}`, fills.includes(a), fills.slice(0, 80));
        });
        check(`${id} 条形画了当前值`, fills.includes("+5.73"), fills.slice(0, 120));
        check(`${id} pitch ±90° 锁边界虚线`, env.rec.dashes.some((d) => d[0] === 4 && d[1] === 3),
              JSON.stringify(env.rec.dashes));
    });
    check("exp2 缺省不显示 euler 面板（未声明）", txt(envs.exp2, "v-roll") === "");

    /* 9) 0x04 帧 → quat 数值/范数/立方参数序 + 欧拉对照表 */
    ["exp4", "exp5"].forEach((id) => {
        const env = envs[id];
        if (env.bootError) return;
        env.emit("frameReady", 4, 1, [0.9, 0.1, 0.2, 0.3]);
        check(`${id} quat 分量入面板`, env.els["q-vals"].innerHTML.includes("0.9000"),
              env.els["q-vals"].innerHTML.slice(0, 80));
        check(`${id} quat 范数`, env.els["q-vals"].innerHTML.includes("0.9747"),
              env.els["q-vals"].innerHTML.slice(0, 160));
        const cube = env.reg.groups[0];
        check(`${id} 立方参数序 (x,y,z,w)`,
              cube && JSON.stringify(cube.quaternion._args) === JSON.stringify([0.1, 0.2, 0.3, 0.9]),
              cube ? JSON.stringify(cube.quaternion._args) : "no cube group");
        check(`${id} 欧拉对照表更新`, env.els["e-pitch"].textContent !== "—",
              `e-pitch=${env.els["e-pitch"].textContent}`);
        /* 万向锁提示：pitch 91° 姿态（四元数）→ 锁区类 + 提示显示 */
        const a = 91.0 / 2 * Math.PI / 180;
        env.emit("frameReady", 4, 2, [Math.cos(a), 0, Math.sin(a), 0]);
        check(`${id} 锁区类挂上`, env.els["qr-pitch"].classList.has("lock"));
        check(`${id} 锁提示显示`, env.els["locknote"].classList.has("show"));
    });
    check("exp2 缺省不显示 quat 面板（未声明）", html(envs.exp2, "q-vals") === "");

    /* 10) pos → traj 轨迹 + 标记位置（exp5） */
    {
        const env = envs.exp5;
        for (let i = 0; i < 300; i++) env.emit("posReady", i * 0.001, -i * 0.002, i * 0.0005);
        env.pumpFrame(0);
        env.pumpFrame(100);
        const trail = env.reg.geoms.find((g) => g._attrs.aAlpha);
        check("exp5 轨迹几何存在", !!trail);
        if (trail) {
            check("exp5 轨迹追加（drawRange）",
                  trail._range[0] === 0 && trail._range[1] === 300,
                  JSON.stringify(trail._range));
            const al = trail._attrs.aAlpha.array;
            check("exp5 最新点不透明", al[299] === 1, `alpha[299]=${al[299]}`);
            check("exp5 最旧点渐隐≈0", al[0] < 0.01, `alpha[0]=${al[0]}`);
        }
        const marker = env.reg.groups[1];
        check("exp5 标记随 pos 移动",
              marker && near(marker.position.x, 0.299) && near(marker.position.y, -0.598),
              marker ? `pos=(${marker.position.x},${marker.position.y})` : "no marker");
        check("exp5 位置读数", env.els["t-vals"].innerHTML.includes("0.299"),
              env.els["t-vals"].innerHTML.slice(0, 80));
        env.els["btn-clear"].onclick();
        check("exp5 清除轨迹", trail && trail._range[1] === 0, trail ? String(trail._range[1]) : "");
    }
    check("exp4 缺省不显示 traj 面板（未声明）", html(envs.exp4, "t-vals") === "");

    /* 11) 双语：切 en 后页面标题 + 全部面板标题变英文（无残留中文） */
    order.forEach((id) => {
        const env = envs[id];
        if (env.bootError) return;
        env.activate();
        const zhTexts = { title: txt(env, "title"), log: txt(env, "cap-log") };
        REQUIRED[id].forEach((p) => { zhTexts[p] = txt(env, "cap-" + p); });
        env.UI.setLang("en");
        const now = { title: txt(env, "title"), log: txt(env, "cap-log") };
        REQUIRED[id].forEach((p) => { now[p] = txt(env, "cap-" + p); });
        Object.keys(now).forEach((k) => {
            check(`${id} en 文案已切换 ${k}`, now[k] !== zhTexts[k] && now[k].length > 0,
                  `zh=${JSON.stringify(zhTexts[k])} en=${JSON.stringify(now[k])}`);
            check(`${id} en 文案无中文残留 ${k}`, !hasCJK(now[k]), now[k]);
        });
        if (txt(env, "btn-cal")) {
            /* 调零按钮文案两态（步 6 刚点过调零 → pending 态是合法值）：
               空闲 = Calibrate / 标定中 = Calibrating…；两态都由 imu.cal.* 词典出。 */
            check(`${id} en 调零按钮`,
                  txt(env, "btn-cal") === "Calibrate"
                  || txt(env, "btn-cal") === "Calibrating…", txt(env, "btn-cal"));
            check(`${id} en 调零按钮无中文残留`, !hasCJK(txt(env, "btn-cal")),
                  txt(env, "btn-cal"));
        }
        if (txt(env, "btn-clear")) {
            check(`${id} en 清除轨迹按钮`, txt(env, "btn-clear") === "Clear trail",
                  txt(env, "btn-clear"));
        }
        check(`${id} en 状态行`, env.els["status"].textContent === "Disconnected",
              env.els["status"].textContent);
        env.UI.setLang("zh");        // 复原，避免影响后续断言
    });
    envs.exp5.activate();

    /* 12) 波形泵契约（raw 面板 = exp1 引擎搬入）：抖动缓冲 / 匀速消费 / 相位 */
    {
        const env = envs.exp1;
        const D = env.debug;
        check("波形泵契约挂载", !!D && typeof D.pump === "function");
        if (D) {
            const RATE = D.frameHz, BUF = D.jitterBufS;
            const need = Math.round(RATE * BUF);
            check("帧率 100Hz / 抖动缓冲 60ms", RATE === 100 && near(BUF, 0.06),
                  `rate=${RATE} buf=${BUF}`);
            for (let k = 0; k < need - 1; k++) D.pushSample(0, 0, 1, 0);
            D.pump(16.7); D.pump(16.7);
            check("缓冲未满不消费", D.count() === 0 && D.qlen() === need - 1,
                  `count=${D.count()} qlen=${D.qlen()}`);
            check("缓冲未满相位=0", D.phase === 0, `phase=${D.phase}`);
            let pushed = need - 1, arrivalMs = 0, batch = [1, 2, 2, 3, 1, 2, 4, 2], bi = 0;
            let prev = 0, maxStep = 0, minStep = 1;
            for (let i = 0; i < 120; i++) {
                arrivalMs += 16.667;
                while (arrivalMs >= 20) {
                    const n = batch[bi++ % batch.length];
                    for (let k = 0; k < n; k++) { D.pushSample(0.02, -0.03, 1.0, 1.25); pushed++; }
                    arrivalMs -= 20;
                }
                const ph = D.pump(16.667);
                check(`相位∈[0,1) @${i}`, ph >= 0 && ph < 1, `phase=${ph}`);
                const step = (ph - prev + 1) % 1;
                if (i > 10) { maxStep = Math.max(maxStep, step); minStep = Math.min(minStep, step); }
                prev = ph;
            }
            const want = RATE * 120 * 16.667 / 1000;      // ≈200 帧
            check("匀速消费≈100Hz（与到达批大小无关）",
                  Math.abs(D.count() - want) <= Math.max(3, want * 0.03),
                  `count=${D.count()} want≈${want.toFixed(1)}`);
            check("样本守恒", D.count() + D.qlen() === pushed,
                  `count=${D.count()} qlen=${D.qlen()} pushed=${pushed}`);
            check("相位步进恒定", maxStep > 0 && (maxStep - minStep) <= 0.02,
                  `min=${minStep.toFixed(3)} max=${maxStep.toFixed(3)}`);
            check("波形数值入面板",
                  (env.emit("frameReady", 1, 1, [0.02, -0.03, 1.0, 1.25, -0.5, 2.5]),
                   html(env, "vals").includes("1.0000")),
                  html(env, "vals").slice(0, 80));
            D.reset();
            D.pump(16.7);
            check("断开重置清窗", D.count() === 0 && D.qlen() === 0,
                  `count=${D.count()} qlen=${D.qlen()}`);
        }
    }

    /* 13) 断开清窗：portChanged(false) → 面板回 — */
    {
        const env = envs.exp5;
        env.emit("portChanged", true);
        check("连接状态→已连接", env.els["status"].textContent === "已连接",
              env.els["status"].textContent);
        env.emit("frameReady", 2, 3, calVals);
        env.emit("frameReady", 3, 3, eulerRad);
        env.emit("frameReady", 4, 3, [1, 0, 0, 0]);
        env.emit("posReady", 0.1, 0, 0);
        env.emit("portChanged", false);
        check("断开后 cal 回 —", env.els["r-az"].textContent === "—",
              env.els["r-az"].textContent);
        check("断开后温度回 —", env.els["temp"].innerHTML.includes("—"),
              env.els["temp"].innerHTML);
        check("断开后欧拉回 —", env.els["v-roll"].textContent === "—",
              env.els["v-roll"].textContent);
        check("断开后四元数回 —", env.els["q-vals"].innerHTML.includes("—"),
              env.els["q-vals"].innerHTML.slice(0, 80));
        check("断开后状态=未连接", env.els["status"].textContent === "未连接",
              env.els["status"].textContent);
        const trail = env.reg.geoms.find((g) => g._attrs.aAlpha);
        check("断开后轨迹清空", trail && trail._range[1] === 0, trail ? String(trail._range[1]) : "");
    }

    /* 14) 显式启动路径：startSim → openSim；listPorts 填串口下拉 */
    return (async () => {
        const env = envs.exp5.activate();   // window.startSim 属最后加载的页面核心
        await new Promise((r) => setTimeout(r, 50));      // listPorts Promise
        const opts = env.els["port"].options;
        check("串口列表填充", opts.length === 2 && opts[0].value === "COM3",
              `opts=${opts.length}`);
        globalThis.startSim();
        await new Promise((r) => setTimeout(r, 350));
        check("startSim → openSim（显式启动）",
              env.bridgeCalls.some((c) => c[0] === "openSim"),
              JSON.stringify(env.bridgeCalls));
        check("切仿真模式后按钮=启动仿真",
              env.els["btn"].textContent === "启动仿真", env.els["btn"].textContent);

        /* 15) 命令通道三控件（读当前值 / 一键调零 / 温漂补偿）的请求→回执→显示链路
           + 失败与超时路径。用干净的 exp2 env（会话从 idle 起）。 */
        const cal = makeEnv("exp2");
        check("命令通道 env 可用", !cal.bootError, cal.bootError ? cal.bootError.message : "");
        if (!cal.bootError) {
            const T = cal.UI.t;
            const banner = () => cal.els["cal-banner"];
            const phase = () => (cal.debug && cal.debug.opPhase ? cal.debug.opPhase() : "?");
            const cmds = () => cal.net.calls.filter((c) => c.url === "/api/imu/cmd")
                .map((c) => c.body.func);
            const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
            const bias6 = [0.02, -0.03, 0.015, 1.0, -0.5, 2.5];      // 单源：hub.SIM_SENSOR_BIAS
            const k6 = [0.0005, 0.0005, 0.0005, 0.05, 0.05, 0.05];   // 温漂系数（每 °C）

            check("命令通道契约挂载（headless）",
                  typeof cal.debug.cmdTimeout === "function" && phase() === "idle",
                  `phase=${phase()}`);
            check("三个控件都在页面上",
                  ["btn-imu-read", "btn-cal", "btn-tc"].every((id) => !!cal.els[id])
                  && cal.els["btn-tc"].onclick !== undefined,
                  JSON.stringify(Object.keys(cal.els).filter((k) => k.indexOf("btn-") === 0)));
            check("控件初始文案（一键调零）", cal.els["btn-cal"].textContent === T("imu.cmd.cal"),
                  cal.els["btn-cal"].textContent);
            check("读数区初始为未读取",
                  cal.els["imu-bias"].textContent.includes(T("imu.bias.none"))
                  && cal.els["imu-tc"].textContent.includes(T("imu.tc.none")),
                  cal.els["imu-bias"].textContent.slice(0, 40));

            /* 路由表：命令通道应答（页面只发命令、只显示回执，不做任何换算） */
            cal.net.routes.get_bias = () => ({
                status: 200, body: { func: "get_bias", status: 0, payload: bias6 },
            });
            cal.net.routes.get_status = () => ({
                status: 200, body: { func: "get_status", status: 0, payload: [0, 1, 0] },
            });
            cal.net.routes.get_drift = () => ({
                status: 200, body: { func: "get_drift", status: 0, payload: k6 },
            });
            cal.emit("portChanged", true);

            /* 15a) 读取当前值 = GET_BIAS + GET_STATUS + GET_DRIFT */
            cal.els["btn-imu-read"].onclick();
            check("读取 → 三命令下发", JSON.stringify(cmds())
                  === JSON.stringify(["get_bias", "get_status", "get_drift"]),
                  JSON.stringify(cmds()));
            check("读取中 banner=pending", banner().className === "pending",
                  `${banner().className}|${banner().textContent}`);
            check("读取中按钮禁用", cal.els["btn-imu-read"].disabled === true);
            await sleep(20);
            check("读取完成 banner=done", banner().className === "done", banner().textContent);
            check("读取完成 = readDone 文案",
                  banner().textContent === T("imu.cmd.readDone", { bv: 1, tv: 0 }),
                  banner().textContent);
            check("零偏 6 值入面板", cal.els["imu-bias"].textContent.includes("+1.0000")
                  && cal.els["imu-bias"].textContent.includes("-0.5000")
                  && cal.els["imu-bias"].textContent.includes("+0.0150"),
                  cal.els["imu-bias"].textContent);
            check("bias_valid 入面板", cal.els["imu-bias"].textContent.includes("bias_valid=1"),
                  cal.els["imu-bias"].textContent);
            check("状态行（flags/bias_valid/tc_valid）",
                  cal.els["imu-status"].textContent.includes("flags=0x0000")
                  && cal.els["imu-status"].textContent.includes("tc_valid=0"),
                  cal.els["imu-status"].textContent);
            check("温漂系数入面板", cal.els["imu-drift"].textContent.includes("+0.05000"),
                  cal.els["imu-drift"].textContent);
            check("读取后按钮恢复", cal.els["btn-imu-read"].disabled === false
                  && cal.els["btn-imu-read"].textContent === T("imu.cmd.read"),
                  `${cal.els["btn-imu-read"].disabled}/${cal.els["btn-imu-read"].textContent}`);

            /* 15b) 一键调零 = CALIBRATE（异步）：pending + 进度 → ACK 后 done + 新零偏 */
            cal.setCalibReply({ code: "calibSimulated", cmd: "CAL", func: "calibrate",
                                status: 0, payload: bias6 });
            cal.els["btn-cal"].onclick();
            check("调零 → sendCalib（旧入口 func=calibrate）",
                  cal.bridgeCalls.some((c) => c[0] === "sendCalib"),
                  JSON.stringify(cal.bridgeCalls));
            check("调零 pending 文案", banner().className === "pending"
                  && banner().textContent === T("imu.cmd.pending", { n: 500 }),
                  banner().textContent);
            check("调零按钮=校准中", cal.els["btn-cal"].textContent === T("imu.cmd.busy"),
                  cal.els["btn-cal"].textContent);
            check("调零中三个按钮都禁用",
                  ["btn-imu-read", "btn-cal", "btn-tc"].every((id) => cal.els[id].disabled),
                  JSON.stringify(["btn-imu-read", "btn-cal", "btn-tc"]
                      .map((id) => cal.els[id].disabled)));
            check("调零中进度有文案（无固件进度时为估算）",
                  cal.els["cal-progress"].textContent.length > 0,
                  cal.els["cal-progress"].textContent);
            cal.emit("textReady", "OK calib progress 250/500");
            check("固件进度 TEXT → 进度用实数",
                  cal.els["cal-progress"].textContent.includes("250/500"),
                  cal.els["cal-progress"].textContent);
            const logRows = cal.els["log"].children.map((c) => c.textContent);
            check("OK calib 日志行高亮 + 时间戳",
                  cal.els["log"].children.some((c) => c.className === "calok"
                      && /^\[\+\d+\.\d{3}s\]/.test(c.textContent)),
                  JSON.stringify(logRows.slice(-3)));
            await sleep(20);
            check("调零完成 banner=done", banner().className === "done", banner().textContent);
            check("完成文案带新零偏（ax/gx…）",
                  banner().textContent === T("imu.cmd.done",
                      { ax: 20, ay: -30, az: 15, gx: "1.000", gy: "-0.500", gz: "2.500" }),
                  banner().textContent);
            check("调零后零偏面板刷新", cal.els["imu-bias"].textContent.includes("+1.0000"),
                  cal.els["imu-bias"].textContent);
            check("调零后按钮恢复可用",
                  ["btn-imu-read", "btn-cal", "btn-tc"]
                      .every((id) => cal.els[id].disabled === false),
                  JSON.stringify(["btn-imu-read", "btn-cal", "btn-tc"]
                      .map((id) => cal.els[id].disabled)));

            /* 15c) 温漂补偿 = TEMP_CALIB → GET_STATUS → SET_FLAGS(置 bit0) → GET_TC_STATE → GET_STATUS */
            const flagCalls = [];
            let curFlags = 0;              // MCU 侧功能位（set_flags 后生效 → 回读可见）
            cal.net.routes.get_status = () => ({
                status: 200, body: { func: "get_status", status: 0,
                                     payload: [curFlags, 1, 1] },
            });
            cal.net.routes.temp_calib = () => ({
                status: 200, body: { func: "temp_calib", status: 0, payload: k6 },
            });
            cal.net.routes.set_flags = (b) => {
                flagCalls.push(b.args.flags);
                curFlags = b.args.flags;
                return { status: 200, body: { func: "set_flags", status: 0,
                                              payload: [b.args.flags] } };
            };
            cal.net.routes.get_tc_state = () => ({
                status: 200, body: { func: "get_tc_state", status: 0,
                                     payload: [1, 1].concat(k6) },
            });
            const before = cmds().length;
            cal.els["btn-tc"].onclick();
            check("温漂补偿 pending 文案",
                  banner().className === "pending"
                  && banner().textContent === T("imu.cmd.tcPending", { sec: 30 }),
                  banner().textContent);
            await sleep(40);
            check("温漂补偿命令序列",
                  JSON.stringify(cmds().slice(before)) === JSON.stringify(
                      ["temp_calib", "get_status", "set_flags", "get_tc_state", "get_status"]),
                  JSON.stringify(cmds().slice(before)));
            check("set_flags 置 TEMPCOMP_EN(bit0)", flagCalls.length === 1 && flagCalls[0] === 1,
                  JSON.stringify(flagCalls));
            check("温漂系数入面板（k6）", cal.els["imu-drift"].textContent.includes("+0.05000"),
                  cal.els["imu-drift"].textContent);
            check("tc_valid/enabled 入面板",
                  cal.els["imu-tc"].textContent.includes("tc_valid=1")
                  && cal.els["imu-tc"].textContent.includes("enabled=1"),
                  cal.els["imu-tc"].textContent);
            check("TEMPCOMP_EN 生效位入状态行",
                  cal.els["imu-status"].textContent.includes("TEMPCOMP_EN=1"),
                  cal.els["imu-status"].textContent);
            check("温漂补偿完成 banner=done",
                  banner().className === "done"
                  && banner().textContent === T("imu.cmd.tcDone",
                      { valid: 1, enabled: 1 }),
                  banner().textContent);

            /* 15d) 失败路径：409 → 明报错误文案 + 按钮恢复 */
            cal.net.routes.get_bias = () => ({ status: 409, body: { error: "未连接（stub）" } });
            cal.els["btn-imu-read"].onclick();
            await sleep(20);
            check("409 → 失败 banner", banner().className === "fail", banner().textContent);
            check("409 → 失败文案带后端错误",
                  banner().textContent === T("imu.cmd.fail", { msg: "未连接（stub）" }),
                  banner().textContent);
            check("失败后按钮恢复可用", cal.els["btn-imu-read"].disabled === false
                  && cal.els["btn-cal"].disabled === false, "");
            check("失败入日志（warn 行）",
                  cal.els["log"].children.some((c) => c.className === "fw"
                      && c.textContent.includes("未连接（stub）")),
                  JSON.stringify(cal.els["log"].children.slice(-2)
                      .map((c) => c.textContent)));

            /* 15e) 无回执 → 超时分支（hook 缩超时，免真等 8s） */
            cal.net.routes.get_bias = () => new Promise(() => { /* 永不响应 */ });
            cal.debug.cmdTimeout(30);
            cal.els["btn-imu-read"].onclick();
            check("超时前仍 pending", phase() === "pending", `phase=${phase()}`);
            await sleep(80);
            check("无回执 → 超时文案", banner().className === "fail"
                  && banner().textContent === T("imu.cmd.timeout"), banner().textContent);
            check("超时后会话回 failed（按钮已恢复）",
                  phase() === "failed" && cal.els["btn-imu-read"].disabled === false,
                  `phase=${phase()}`);

            /* 15f) 断开清「进行中」状态（不背假进度） */
            cal.net.routes.get_bias = () => new Promise(() => { /* 永不响应 */ });
            cal.debug.cmdTimeout(8000);
            cal.els["btn-imu-read"].onclick();
            check("断开前 pending", phase() === "pending", `phase=${phase()}`);
            cal.emit("portChanged", false);
            check("断开后会话 idle + banner 清空",
                  phase() === "idle" && banner().textContent === "",
                  `${phase()}|${banner().textContent}`);
            check("断开后按钮恢复可用", cal.els["btn-imu-read"].disabled === false);
        }
        return fails;
    })();
}

module.exports = { SHELLS, ALL_PANELS, REQUIRED, readShell, makeEnv, createRun };

if (require.main === module) {
    runSuite().then((fails) => {
        fails.forEach((f) => console.log("  [FAIL]", f));
        console.log(`== imu_page_harness: 薄壳/逐级包含/渲染集合/词典/禁自启/调零/` +
                    `0x02·0x03·0x04 面板/pos 轨迹/双语/波形泵/断开清窗/显式启动/` +
                    `命令通道(三控件·pending·done·失败·超时), ` +
                    `${fails.length} fail ==`);
        process.exit(fails.length ? 1 : 0);
    });
}
