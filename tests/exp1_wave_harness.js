/* exp1_wave_harness.js — exp1 波形泵契约测试（node，无浏览器/无 GPU）。
 *
 * 为什么要这层：用户报的「没有设备时波形在抖」根因是**到达抖动 → 抽帧数量抖动 →
 * x 轴按样本序号排布**，修法是「抖动缓冲 + 虚拟帧时钟匀速消费 + 亚像素相位」。
 * 这条链路没法靠 Qt 页测试断言（rAF 在 offscreen 不跑），所以用真实到达抖动模式
 * 回放泵体（页面核心的 window.__xrkDebug.pump）。
 *
 * 页面形态变更（用户定案 2026-11-06「逐级继承」）：exp1 页已改为薄壳
 * （examples/imu/exp1_waveform/index.html 只声明 panels=["raw"]），波形引擎原样搬进
 * 统一核心 web/assets/imu-page.js。本文件按新页面断言同一组能力，判据不变：
 *
 *   0) 薄壳声明 exp1=[raw]；加载后不启动任何数据源（禁隐式回落）
 *   1) 抖动缓冲：缓冲未攒够时一帧都不消费（primed=false, count=0）
 *   2) 匀速消费：按 60fps rAF（dt=16.7ms）泵 2s，消费量 ≈ 100Hz×时长（±3%），
 *      **与到达批大小无关**（回放实测模式：每 20ms 批 1~4 帧）
 *   3) 不丢样本：count + 队列余量 == 已推帧数
 *   4) 亚像素相位始终 ∈ [0,1)，且不因批大小跳变（相邻两帧相位增量差 ≤ 0.02）
 *   5) 显式启动：startSim() → openSim；双语：切 en 后标题/面板标题变英文
 *
 * 跑法：node tests/exp1_wave_harness.js（exit 0 = 全过；需 node>=18）
 */

"use strict";
const H = require("./imu_page_harness.js");

const { fails, check } = H.createRun();
const near = (a, b, eps) => Math.abs(a - b) <= (eps === undefined ? 1e-9 : eps);

async function main() {
    const env = H.makeEnv("exp1");
    if (env.bootError) {
        fails.push(`页面加载崩溃: ${env.bootError.message}`);
        return;
    }
    const shell = env.shell.cfg;
    check("薄壳声明 exp1 面板集合", JSON.stringify(shell.panels) === JSON.stringify(["raw"]),
          JSON.stringify(shell.panels));
    check("禁隐式回落：加载后未启源", env.bridgeCalls.length === 0,
          JSON.stringify(env.bridgeCalls));
    check("缺省真实模式", env.UI.mode === "real", `mode=${env.UI.mode}`);

    const D = env.debug;
    check("泵体契约挂载", !!D && typeof D.pump === "function");
    if (D) {
        /* 回放测得抖动模式：每 20ms 批到 1~4 帧 */
        const BATCH = [1, 2, 2, 3, 1, 2, 4, 2, 1, 3, 2, 1, 2, 3, 2, 2];
        const RATE = D.frameHz;                  // 100Hz
        const BUF = D.jitterBufS;                // 0.06s
        const need = Math.round(RATE * BUF);     // 起播所需帧数（6）

        /* 1) 缓冲未满：一帧都不消费 */
        for (let k = 0; k < need - 1; k++) D.pushSample(0, 0, 1, 0);
        D.pump(16.7);
        D.pump(16.7);
        check("缓冲未满不消费", D.count() === 0 && D.qlen() === need - 1,
              `count=${D.count()} qlen=${D.qlen()}`);
        check("缓冲未满相位=0", D.phase === 0, `phase=${D.phase}`);

        /* 2) 2 秒回放：rAF 60fps，帧按 20ms 批到达 */
        let pushed = need - 1;
        let arrivalMs = 0, batchIdx = 0, prevPhase = 0, maxStep = 0, minStep = 1;
        const TICKS = 120;                       // 2.0 s @ 60fps
        for (let i = 0; i < TICKS; i++) {
            arrivalMs += 16.667;
            while (arrivalMs >= 20) {
                const n = BATCH[batchIdx++ % BATCH.length];
                for (let k = 0; k < n; k++) { D.pushSample(0, 0, 1, 0); pushed++; }
                arrivalMs -= 20;
            }
            const ph = D.pump(16.667);
            const step = (ph - prevPhase + 1) % 1;
            if (i > 10 && step > maxStep) maxStep = step;
            if (i > 10 && step < minStep) minStep = step;
            prevPhase = ph;
            check(`相位越界@${i}`, ph >= 0 && ph < 1, `phase=${ph}`);
        }

        /* 期望消费量 = 100Hz × 2.0s（容许 1 帧量化 + 起播延迟：±3%） */
        const want = RATE * TICKS * 16.667 / 1000;      // ≈200
        const got = D.count();
        check("匀速消费≈100Hz", Math.abs(got - want) <= Math.max(3, want * 0.03),
              `count=${got} want≈${want.toFixed(1)}（与批大小无关）`);

        /* 不丢样本：窗内 + 队列余量 == 已推帧数 */
        check("样本守恒", got + D.qlen() === pushed,
              `count=${got} qlen=${D.qlen()} pushed=${pushed}`);

        /* 相位推进恒定：批大小怎么跳都只改整数部分，小数相位步进恒定 */
        check("相位步进恒定", maxStep > 0 && (maxStep - minStep) <= 0.02,
              `min=${minStep.toFixed(3)} max=${maxStep.toFixed(3)}`);
        check("相位步进=理论值", near(maxStep, 0.667, 0.05),
              `maxStep=${maxStep.toFixed(3)}（应 ≈0.667）`);

        /* 3) 断开重置：清窗后回到未起播态 */
        D.reset();
        D.pump(16.7);
        check("断开重置清窗", D.count() === 0 && D.qlen() === 0 && D.phase === 0,
              `count=${D.count()} qlen=${D.qlen()} phase=${D.phase}`);
    }

    /* 数值面板：0x01 帧入窗后六轴读数可见 */
    env.emit("frameReady", 1, 1, [0.02, -0.03, 1.0, 1.25, -0.5, 2.5]);
    check("六轴读数入面板", (env.els["vals"] ? env.els["vals"].innerHTML : "").includes("1.0000"),
          (env.els["vals"] ? env.els["vals"].innerHTML : "").slice(0, 80));

    /* 4) 显式启动路径 + 串口列表 */
    await new Promise((r) => setTimeout(r, 50));
    const opts = env.els["port"].options;
    check("串口列表填充", opts.length === 2 && opts[0].value === "COM3", `opts=${opts.length}`);
    globalThis.startSim();
    await new Promise((r) => setTimeout(r, 350));
    check("startSim → openSim", env.bridgeCalls.some((c) => c[0] === "openSim"),
          JSON.stringify(env.bridgeCalls));

    /* 5) 双语：en 后页面标题/面板标题/状态行变英文 */
    env.activate();
    env.UI.setLang("en");
    check("en 页面标题", /Raw IMU stream/.test(env.els["title"].textContent),
          env.els["title"].textContent);
    check("en 面板标题", /Raw waveform/.test(env.els["cap-raw"].textContent),
          env.els["cap-raw"].textContent);
    check("en 状态行", env.els["status"].textContent === "Disconnected",
          env.els["status"].textContent);
}

main().then(() => {
    fails.forEach((f) => console.log("  [FAIL]", f));
    console.log("== exp1_wave_harness: 禁隐式回落/缓冲/匀速消费/样本守恒/相位平滑/" +
                `显式启动/双语, ${fails.length} fail ==`);
    process.exit(fails.length ? 1 : 0);
});
