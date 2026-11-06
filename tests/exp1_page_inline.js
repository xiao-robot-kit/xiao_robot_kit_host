
"use strict";
/* —— 页壳：模式（仿真/真实）与语言（中/EN）——
   数据通路不变，模式/语言由 web/assets/ui-shell.js 统一提供：
   * 加载时不启动任何数据源（禁隐式仿真、禁自动连 QEMU）；
   * UI.mode === "sim" 时按钮 = 启动仿真（bridge.openSim），"real" 时 = 连接串口；
   * 语言切换即时重建本页静态文案（data-i18n 锚点）。 */
UI.define({
  zh: {
    "exp1.title": "exp1 · IMU 裸数据直出",
    "exp1.acc": "加速度（g）",
    "exp1.gyr": "角速度（dps）",
    "exp1.vals": "实时数值",
    "exp1.logv": "固件日志（TEXT 帧）",
    "exp1.port": "串口（真实模式）",
    "exp1.stat": "帧 {n} · 丢帧 {drop}",
  },
  en: {
    "exp1.title": "exp1 · Raw IMU stream",
    "exp1.acc": "Acceleration (g)",
    "exp1.gyr": "Angular rate (dps)",
    "exp1.vals": "Live values",
    "exp1.logv": "Firmware log (TEXT frames)",
    "exp1.port": "Serial port (real mode)",
    "exp1.stat": "{n} frames · {drop} dropped",
  },
});
UI.init({page: "exp1"});   // 只渲染壳，不启动数据源

/* —— 波形引擎：Three.js（r128 本地副本）——
   六通道滚动波形 = 两组（加速度/角速度）× 三色 THREE.Line；
   BufferGeometry 预分配（每通道 N 顶点，永不重建/重分配）；
   滚动 = 数据环左移 + setDrawRange(N-count, count) 圈定有效窗口
   （填充期窗口从右向左生长，满窗后 count=N、数据整体左移滚动）；
   量程自适应口径沿用旧自研 scope：窗口内峰值×1.2，量程标注见 .rng 覆层。
   （旧 scope 中 getElementById 的西里尔字母手误死代码已随本次重写消除） */
const accC = document.getElementById("cAcc");
const gyrC = document.getElementById("cGyro");
const valsEl = document.getElementById("vals");
const logEl  = document.getElementById("log");
const statEl = document.getElementById("stat");
const statusEl = document.getElementById("status");

if (!window.THREE) {
  statusEl.textContent = UI.t("common.threeMissing");
  throw new Error("three.js missing");
}

function scope3(canvas, colors) {
  const W = Math.max(2, Math.floor(canvas.clientWidth || +canvas.width));
  const H = Math.max(2, Math.floor(canvas.clientHeight || +canvas.height));
  const N = W;                            // 每像素列一样本（与旧 scope 同密度）
  const renderer = new THREE.WebGLRenderer({canvas: canvas, antialias: true});
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  renderer.setSize(W, H, false);
  renderer.setClearColor(0x0c1014);
  const scene = new THREE.Scene();
  const cam = new THREE.OrthographicCamera(0, W, 1, -1, -1, 1);  // y 域随量程改
  const zero = new THREE.BufferGeometry();                       // 零基线（静态 2 顶点）
  zero.setAttribute("position",
    new THREE.BufferAttribute(new Float32Array([0,0,0,  W,0,0]), 3));
  scene.add(new THREE.Line(zero, new THREE.LineBasicMaterial({color: 0x22303c})));

  const chans = colors.map(col => {
    const pos = new Float32Array(N * 3);  // 预分配顶点：x=列号，y=值（g/dps），z=0
    const attr = new THREE.BufferAttribute(pos, 3);
    attr.setUsage(THREE.DynamicDrawUsage);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", attr);
    geom.setDrawRange(N, 0);              // 初始空窗口（count=0）
    scene.add(new THREE.Line(geom, new THREE.LineBasicMaterial({color: col})));
    return {pos: pos, attr: attr, geom: geom,
            yr: new Float32Array(N), pend: []};
  });
  let count = 0;                          // 有效样本数（≤N，各通道同长）
  const rngEl = canvas.parentElement.querySelector(".rng");

  return {
    push(vals) { for (let k = 0; k < chans.length; k++) chans[k].pend.push(vals[k]); },
    qlen() { return chans[0].pend.length; },   // 待入窗样本数（= 抖动缓冲深度）
    count() { return count; },                 // 窗内样本数（测试观测点）
    /* 队头丢 over 个（慢消费保护：后台标签页 rAF 停摆时不让队列无限长） */
    trim(cap) {
      const over = chans[0].pend.length - cap;
      if (over <= 0) return 0;
      for (const c of chans) c.pend.splice(0, over);
      return over;
    },
    clear() {                                  // 断开/切模式：清窗清队列
      for (const c of chans) { c.pend.length = 0; c.yr.fill(0); }
      count = 0;
    },
    /* 匀速消费 n 个样本入窗（不足则只取现有；从队头 FIFO 取，不丢其余排队样本） */
    consume(n) {
      const take = Math.min(n, chans[0].pend.length, N);
      if (take <= 0) return 0;
      const overflow = Math.max(0, count + take - N);   // 满窗：先整体左移 overflow 列
      for (const c of chans) {
        if (overflow) c.yr.copyWithin(0, overflow);
        const taken = c.pend.splice(0, take);           // 队头取 take 个
        for (let j = 0; j < taken.length; j++) c.yr[count - overflow + j] = taken[j];
      }
      count = Math.min(count + take, N);
      return take;
    },
    /* 渲染：phase ∈ [0,1) 为亚像素滚动相位（相机左移 phase 像素）*/
    draw(phase) {
      const ph = phase || 0;
      let peak = 1e-6;                    // 量程自适应：窗口峰值×1.2
      for (const c of chans) {
        for (let i = 0; i < count; i++) {
          const col = N - count + i, o = col * 3;
          c.pos[o] = col; c.pos[o + 1] = c.yr[i];
          const a = Math.abs(c.yr[i]);
          if (a > peak) peak = a;
        }
        c.attr.needsUpdate = true;
        c.geom.setDrawRange(N - count, count);
      }
      peak *= 1.2;
      cam.left = ph; cam.right = W + ph;  // 亚像素平移：整像素跳动只在相位回绕点发生
      cam.top = peak; cam.bottom = -peak; cam.updateProjectionMatrix();
      renderer.render(scene, cam);
      if (rngEl) rngEl.textContent = "±" + peak.toFixed(2);
    }
  };
}

const CH_COLORS = [0xe57373, 0x81c784, 0x64b5f6];   // x/y/z 三色（沿用旧配色）
const accS = scope3(accC, CH_COLORS);
const gyrS = scope3(gyrC, CH_COLORS);

function renderVals(a, g) {
  const rows = [["ax",a[0]],["ay",a[1]],["az",a[2]],["gx",g[0]],["gy",g[1]],["gz",g[2]]];
  valsEl.innerHTML = rows.map(([n,v]) =>
    `<div class="v">${n} <b>${v.toFixed(4)}</b></div>`).join("");
}

/* —— 渲染泵：抖动缓冲 + 匀速消费 + 亚像素滚动（抖动根因修复）——
   实测（本机 3 s）：_SimWorker 20ms 批轮询每批到达 1~4 帧（49 次/s，共 147 帧）；
   旧实现按 rAF 40ms 节流就抽帧 ⇒ 每次抽走 3~6 帧；而波形 x 轴按样本序号
   排布（1 样本 = 1 px），抽帧数一变横向推进速度就变——这就是肉眼看得到的
   「抖动/一顿一顿」。修法三段：
     ① 抖动缓冲 60ms：起播先攒 ≈6 帧，吸收到达抖动（帧不够就暂停，绝不跳帧）；
     ② 虚拟帧时钟：按协议 100Hz 匀速消费，与到达节奏完全解耦（不追不抢）；
     ③ 亚像素相位：相机左移 phase 像素，使整像素平移只发生在相位回绕点——
        推进速度恒定，无整块跳动。 */
const FRAME_HZ = 100;          // 协议帧率（固件 IMU_SAMPLE 100Hz）
const JITTER_BUF_S = 0.06;     // 抖动缓冲深度（s）
let bridge = null, connected = false;
let lastNow = 0, tickAcc = 0, primed = false, lastStat = { n: 0, drop: 0 };

function resetView() {         // 断开/切模式：清窗 + 重置抖动缓冲相位
  accS.clear(); gyrS.clear();
  primed = false; tickAcc = 0; lastNow = 0;
}

function tick(t) {
  const dt = lastNow ? t - lastNow : 0;
  lastNow = t;
  const phase = pump(dt);
  accS.draw(phase); gyrS.draw(phase);
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);

/* 泵体（从 rAF 回调里抽出来，便于 headless 契约测试）：
   入参 dtMs = 本次 rAF 与上次的时间差（ms）。返回亚像素相位。 */
function pump(dtMs) {
  if (!primed && accS.qlen() >= Math.round(FRAME_HZ * JITTER_BUF_S)) primed = true;
  if (primed) {
    /* 慢消费保护：队列超 5 s 存量就丢最旧（后台标签 rAF 停摆/源率异常时
       宁可滞后有界，不让内存无界长） */
    const over = accS.qlen() - FRAME_HZ * 5;
    if (over > 0) { accS.trim(FRAME_HZ * 5); gyrS.trim(FRAME_HZ * 5); }
    tickAcc += Math.min(dtMs, 250) / 1000 * FRAME_HZ;
    const n = Math.floor(tickAcc);
    if (n > 0) { tickAcc -= n; accS.consume(n); gyrS.consume(n); }
  }
  const phase = primed ? Math.min(tickAcc, 0.999) : 0;
  window.__xrkWave = { phase: phase, rate: FRAME_HZ, primed: primed,
                       qlen: accS.qlen(), count: accS.count() };
  return phase;
}

/* headless 契约（tests/exp1_wave_harness.js）：泵体/窗长/帧率/抖动缓冲深度 */
window.__xrkDebug = {
  pump: pump, frameHz: FRAME_HZ, jitterBufS: JITTER_BUF_S,
  count: () => accS.count(), qlen: () => accS.qlen(),
  pushSample: (ax, ay, az, gx) => { accS.push([ax, ay, az]); gyrS.push([gx, 0, 0]); },
  reset: resetView,
  get phase() { return (window.__xrkWave || {}).phase; },
};

function log(msg, warn) {
  const d=document.createElement("div");
  if (warn) d.className="fw";
  d.textContent=msg; logEl.appendChild(d);
  while (logEl.children.length>200) logEl.removeChild(logEl.firstChild);
  logEl.scrollTop=logEl.scrollHeight;
}

/* 等 QWebChannel 桥就绪（既有 kick 契约：200ms 轮询） */
function waitBridge(fn) {
  const wait = setInterval(() => {
    if (bridge) { clearInterval(wait); fn(); }
  }, 200);
}

/* 旧 kick 契约（app.py --mode sim / Qt 页测试）：显式切仿真模式再启源 */
function startSim() {
  UI.setMode("sim", { silent: true });
  waitBridge(() => { if (!connected) bridge.openSim(); });
}

/* 旧 kick 契约（app.py 位置参数串口）：显式切真实模式 + 选端口 + 连接 */
function autoConnect(port) {
  UI.setMode("real", { silent: true });
  const sel = document.getElementById("port");
  const wait = setInterval(() => {
    if (sel.options.length) {
      clearInterval(wait);
      for (const o of sel.options) if (o.value === port) sel.value = port;
      if (!connected) bridge.openPort(sel.value || port, 115200);
    }
  }, 200);
}

window.startSim = startSim;        // app.py 显式 kick（非同源执行域，必挂 window）
window.autoConnect = autoConnect;

bridge = LiveBridge();   // 服务器版：SSE 收帧 + POST 命令（API 与旧 bridge 同形）
{
  /* 防御：Qt6 WebChannel 下 Slot 带返回值的方法在 JS 侧拿到的是 Promise
     （PySide6 6.12 实测），旧写法 listPorts().forEach 直接抛异常并中断
     本回调后续的信号接线。Promise.resolve 兼容旧版同步 Array 返回。 */
  Promise.resolve(bridge.listPorts()).then(ports => {
    (Array.isArray(ports) ? ports : Array.from(ports || [])).forEach(p => {
      const o=document.createElement("option"); o.value=o.textContent=p;
      document.getElementById("port").appendChild(o);
    });
  });
  bridge.frameReady.connect((type,seq,vals) => {
    if (type !== 1) return;                       // 本页只消费 IMU_SAMPLE（0x01）
    accS.push(vals.slice(0,3)); gyrS.push(vals.slice(3,6));
    renderVals(vals.slice(0,3), vals.slice(3,6));
  });
  bridge.textReady.connect(m => log(m));
  bridge.statReady.connect((imu,text,drop) => {
    lastStat = {n: imu, drop: drop};
    statEl.textContent = UI.t("exp1.stat", lastStat);
  });
  bridge.portChanged.connect(on => {
    connected = on;
    UI.setConnected(on);                          // 状态/按钮文案归壳（中英一致）
    if (!on) resetView();                         // 断开即清窗（不残留上一条流）
  });
}

/* 语言切换：动态状态行也要跟着换（静态文案由 data-i18n 自动刷新） */
UI.onLangChange(() => { statEl.textContent = UI.t("exp1.stat", lastStat); });

/* 模式切换：先断开当前流（避免仿真与真实叠加），再由用户点按钮启源 */
UI.onModeChange(() => { if (connected) bridge.closePort(); });

/* 按钮 = 显式动作分发：仿真→合成源，真实→串口；不自动、不回落 */
document.getElementById("btn").onclick = () => {
  if (!bridge) return;
  if (connected) { bridge.closePort(); return; }
  if (UI.mode === "sim") { bridge.openSim(); return; }
  bridge.openPort(document.getElementById("port").value || "COM3", 115200);
};
