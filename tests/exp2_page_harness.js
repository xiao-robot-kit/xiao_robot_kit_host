/* exp2_page_harness.js — exp2 页面（裸波形 + 零点校准）逻辑 harness（node，无浏览器）。
 *
 * 页面形态变更（用户定案 2026-11-06「逐级继承」）：exp2 页已改为薄壳
 * （examples/imu/exp2_bias/index.html 只声明 panels=["raw","cal"]），实现搬进统一
 * 核心 web/assets/imu-page.js。本文件按新页面断言同一组能力，覆盖不降：
 *
 *   0) 薄壳声明 exp2 = exp1 的全部面板 + cal（逐级继承）
 *   1) makeWinStats 数值契约：mean/std/min/max 恒等式（已知输入手算基准）、
 *      滑窗封顶 shift、reset 清窗
 *   2) 0x02 帧（19 值）入页：最新帧 raw/deb/bias 三块分列、统计表均值列与温度
 *      随动；19 值以外的帧（长度错）被拒（防御路径）
 *   3) 统计口径：raw 均值含温漂项、deb 均值 = 补偿后残差（手算基准对数）
 *   4) 调零按钮：CAL 命令下发（sendCalib 可观察）+ 统计窗重开（0/300）
 *   5) 断开清窗：portChanged(false) → 面板回 —
 *   6) startSim() → openSim；listPorts Promise 填充串口下拉
 *
 * 表格/按钮布局渲染不在此层逐像素断言——属人工开窗口项。
 * 跑法：node tests/exp2_page_harness.js（exit 0 = 全过；需 node>=18）
 */

"use strict";
const H = require("./imu_page_harness.js");

const { fails, check } = H.createRun();
const near = (a, b, eps) => Math.abs(a - b) <= (eps === undefined ? 1e-9 : eps);

function mk(x) {
    /* 一帧 19 值：第 0 分量 = x、第 6 分量 = 10+x、第 18 = 25+x（统计口径基准） */
    return [x, 2 * x, 0, 0, 0, 0, 10 + x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25 + x];
}

async function main() {
    const env = H.makeEnv("exp2");
    if (env.bootError) {
        fails.push(`页面脚本加载崩溃: ${env.bootError.message}`);
        return;
    }
    const els = env.els;
    const cfg = env.shell.cfg;
    check("薄壳声明 exp2 面板集合",
          JSON.stringify(cfg.panels) === JSON.stringify(["raw", "cal"]),
          JSON.stringify(cfg.panels));
    check("禁隐式回落：加载后未启源", env.bridgeCalls.length === 0,
          JSON.stringify(env.bridgeCalls));

    /* 1) makeWinStats 数值契约（已知输入手算基准） */
    check("契约挂载", typeof globalThis.makeWinStats === "function");
    const ws = globalThis.makeWinStats(4);
    [1, 2, 3, 4, 5, 6].forEach((x) => ws.push(mk(x)));   // 窗 4：留 [3,4,5,6]
    check("滑窗封顶", ws.n() === 4, `n=${ws.n()}`);
    check("均值", near(ws.mean(0), 4.5), `mean0=${ws.mean(0)}（窗 [3,4,5,6] 应 4.5）`);
    check("均值第2块", near(ws.mean(6), 14.5), `mean6=${ws.mean(6)}`);
    check("温度均值", near(ws.mean(18), 29.5), `mean18=${ws.mean(18)}`);
    check("标准差", near(ws.std(0), Math.sqrt(5 / 3)), `std0=${ws.std(0)}`);
    check("min/max", ws.min(0) === 3 && ws.max(0) === 6,
          `min=${ws.min(0)} max=${ws.max(0)}`);
    ws.reset();
    check("reset清窗", ws.n() === 0 && ws.mean(0) === null && ws.std(0) === null);
    const ws1 = globalThis.makeWinStats(4);
    ws1.push(mk(1));
    check("单帧σ为null", ws1.std(0) === null, `std=${ws1.std(0)}（n<2 应 null）`);

    /* 2/3) 0x02 帧入页：手算基准对数（SimCalSource 同构模型） */
    const N = 120;
    for (let k = 1; k <= N; k++) {
        const t = k / 100;
        const temp = 25 + 0.1 * t;
        const rawGx = 1.0 + 0.05 * (temp - 25);
        const biasGx = 1.025;
        const v = [0.02, -0.03, 1.015, rawGx, -0.5, 2.5,          // raw6
                   0, 0, 1, rawGx - biasGx, -0.5, 2.5,            // deb6
                   0.02, -0.03, 0.015, biasGx, -0.5, 2.5,         // bias6
                   temp];                                          // temp
        env.emit("frameReady", 2, k & 0xFF, v);
    }
    check("最新帧raw列", els["r-gx"].textContent === (1.0 + 0.05 * 0.12).toFixed(4),
          `r-gx=${els["r-gx"].textContent}`);
    check("最新帧deb列", els["d-gx"].textContent === (1.0 + 0.05 * 0.12 - 1.025).toFixed(4),
          `d-gx=${els["d-gx"].textContent}`);
    check("最新帧bias列", els["b-gx"].textContent === "1.0250", `b-gx=${els["b-gx"].textContent}`);
    check("温度显示", els["temp"].innerHTML.includes((25 + 0.1 * 1.2).toFixed(4) + " °C"),
          `temp=${els["temp"].innerHTML}`);

    let s = 0;
    for (let k = 1; k <= N; k++) s += 1.0 + 0.05 * (0.1 * k / 100);
    const expMean = s / N;
    check("raw均值列", els["m-gx"].textContent.startsWith(expMean.toFixed(4)),
          `m-gx=${els["m-gx"].textContent}（应 ${expMean.toFixed(4)}…）`);
    /* deb 序列 = raw(k) − 1.025：均值 = expMean − 1.025（同线性性质） */
    check("deb均值列", els["s-gx"].textContent.startsWith((expMean - 1.025).toFixed(4)),
          `s-gx=${els["s-gx"].textContent}（应 ${(expMean - 1.025).toFixed(4)}…）`);
    check("温度范围", els["temp"].innerHTML.includes((25 + 0.1 * 0.01).toFixed(4)),
          `temp=${els["temp"].innerHTML}`);
    check("帧计数", els["stat"].textContent.includes(`IMU_CAL ${N}`),
          `stat=${els["stat"].textContent}`);
    check("窗口计数", els["stat"].textContent.includes(`窗口 ${N}/300`),
          `stat=${els["stat"].textContent}`);

    /* 防御：长度错的 0x02 帧被拒（计数不增）；其他帧型不驱动校准面板 */
    env.emit("frameReady", 2, 1, [1, 2, 3]);
    check("畸形帧拒收", !els["stat"].textContent.includes(`IMU_CAL ${N + 1}`),
          `stat=${els["stat"].textContent}`);
    env.emit("frameReady", 4, 1, [0.9, 0.1, 0.2, 0.3]);
    check("QUAT帧不驱动校准面板",
          els["r-gx"].textContent === (1.0 + 0.05 * 0.12).toFixed(4),
          `r-gx=${els["r-gx"].textContent}`);

    /* 4) 调零按钮：sendCalib 下发 + 统计窗重开 + 回执按词典翻译入日志 */
    els["btn-cal"].onclick();
    await new Promise((r) => setTimeout(r, 50));   // Promise.then 回执入日志
    check("CAL命令下发", env.bridgeCalls.some((c) => c[0] === "sendCalib"),
          `calls=${JSON.stringify(env.bridgeCalls)}`);
    check("回执入日志", els["log"].children.some((c) =>
          c.textContent.includes("[cal]") && c.textContent.includes("仿真档调零")),
          `log=${els["log"].children.map((c) => c.textContent).join("|").slice(0, 120)}`);
    check("窗口重开", els["stat"].textContent.includes("窗口 0/300"),
          `stat=${els["stat"].textContent}`);

    /* 5) 断开清窗 */
    env.emit("portChanged", false);
    check("断开清窗", els["stat"].textContent.includes("窗口 0/300") &&
          els["r-gx"].textContent === "—" && els["temp"].innerHTML.includes("—"),
          `stat=${els["stat"].textContent} r-gx=${els["r-gx"].textContent}`);

    /* 6) startSim() → openSim（真实 node 定时器；app.py kick 同路径）+ 串口列表 */
    globalThis.startSim();
    await new Promise((r) => setTimeout(r, 350));
    check("startSim→openSim", env.bridgeCalls.some((c) => c[0] === "openSim"),
          `calls=${JSON.stringify(env.bridgeCalls)}`);
    await new Promise((r) => setTimeout(r, 50));
    const opts = els["port"].options;
    check("串口列表填充", opts.length === 2 && opts[0].value === "COM3", `opts=${opts.length}`);
}

main().then(() => {
    fails.forEach((f) => console.log("  [FAIL]", f));
    console.log("== exp2_page_harness: 薄壳面板/winStats契约/0x02入页/统计对数/" +
                `畸形拒收/调零按钮/断开清窗/startSim/串口列表, ${fails.length} fail ==`);
    process.exit(fails.length ? 1 : 0);
});
