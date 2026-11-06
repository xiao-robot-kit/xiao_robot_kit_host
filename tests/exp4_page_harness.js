/* exp4_page_harness.js — exp4 页面（+欧拉 +四元数姿态）逻辑 harness（node，无浏览器）。
 *
 * 页面形态变更（用户定案 2026-11-06「逐级继承」）：exp4 页已改为薄壳
 * （examples/imu/exp4_quat_view/index.html 只声明 panels=["raw","cal","euler","quat"]），
 * 实现搬进统一核心 web/assets/imu-page.js。本文件按新页面断言同一组能力，覆盖不降：
 *
 *   0) 薄壳面板集合 = exp2 的全部能力 + euler + quat（逐级继承，缺一即 bug）
 *   1) 数值换算契约 quatToEulerZyx：单位四元数/三轴 90°/复合角
 *      （与固件 quat.c quat_to_euler 同式；弧度→度对照基准值）
 *   2) 万向锁钉法：|sinp|>=1 时 pitch 钉 ±90°；91° 姿态经 ZYX 反解
 *      显示为 (180°, 89°, 180°)——欧拉退化（exp3↔exp4 教学呼应的数值证据）
 *   3) quatNorm：|q| 契约（0.5×4 → 1；3-4-5 → 1）
 *   4) QUAT 帧驱动 cube.quaternion.set(x,y,z,w) 参数序（THREE 约定，帧 w 在前）
 *   5) 锁指示：pitch ≥ LOCK_DEG(85°) 时欧拉对照行加 lock 类、locknote 显示
 *   6) 0x03 帧驱动欧拉条形面板（数值表 + 条形绘制 + pitch 锁边界虚线）
 *   7) startSim() → openSim；listPorts Promise 填充串口下拉
 *   8) WebGL 降级守卫：渲染器构造抛错（无 GPU/offscreen）不得杀数据链路
 *
 * WebGL 像素渲染与鼠标轨道控制不在此层——属人工开窗口项。
 * 跑法：node tests/exp4_page_harness.js（exit 0 = 全过；需 node>=18）
 */

"use strict";
const H = require("./imu_page_harness.js");

const { fails, check } = H.createRun();
const near = (a, b, eps) => Math.abs(a - b) <= (eps === undefined ? 1e-9 : eps);
const deg = (r) => r * 180 / Math.PI;

async function main() {
    const env = H.makeEnv("exp4");
    if (env.bootError) {
        fails.push(`页面脚本加载崩溃: ${env.bootError.message}`);
        return;
    }
    const els = env.els;
    check("薄壳声明 exp4 面板集合",
          JSON.stringify(env.shell.cfg.panels) ===
          JSON.stringify(["raw", "cal", "euler", "quat"]),
          JSON.stringify(env.shell.cfg.panels));
    check("禁隐式回落：加载后未启源", env.bridgeCalls.length === 0,
          JSON.stringify(env.bridgeCalls));

    /* 1) 数值换算契约：quatToEulerZyx（rad；与固件 quat.c 同式） */
    const qe = globalThis.quatToEulerZyx;
    check("换算契约挂载", typeof qe === "function" && typeof globalThis.quatNorm === "function");

    let e = qe(1, 0, 0, 0);
    check("identity→零", e.every((v) => v === 0), `got ${JSON.stringify(e)}`);

    const c = Math.SQRT1_2;
    const w45 = Math.cos(Math.PI / 8), y45 = Math.sin(Math.PI / 8);
    e = qe(w45, 0, y45, 0);                       // pitch +45°
    check("pitch45", near(deg(e[1]), 45, 1e-9) && near(e[0], 0, 1e-12) && near(e[2], 0, 1e-12),
          `got ${JSON.stringify(e.map(deg))}`);
    e = qe(c, 0, c, 0);                           // pitch 恰 +90°
    check("pitch90奇异点", near(deg(e[1]), 90, 1e-9), `got pitch=${deg(e[1])}`);
    e = qe(c, c, 0, 0);                           // roll +90°
    check("roll90", near(e[0], Math.PI / 2) && near(e[1], 0) && near(e[2], 0),
          `got ${JSON.stringify(e.map(deg))}`);
    e = qe(c, 0, 0, c);                           // yaw +90°
    check("yaw90", near(e[2], Math.PI / 2) && near(e[0], 0) && near(e[1], 0),
          `got ${JSON.stringify(e.map(deg))}`);
    e = qe(c, -0, -c, 0);                         // pitch −90°（sinp=−1 → 钉 −π/2）
    check("pitch-90钉", near(e[1], -Math.PI / 2), `got ${deg(e[1])}`);

    /* 2) 万向锁钉法：91° 姿态 → ZYX 反解 (180, 89, 180) */
    e = qe(c, 0, c, 0);
    check("锁钉+90", near(e[1], Math.PI / 2, 1e-12), `got ${deg(e[1])}`);
    const a915 = 91.0 / 2 * Math.PI / 180;
    const w915 = Math.cos(a915), y915 = Math.sin(a915);
    e = qe(w915, 0, y915, 0);
    check("91°退化pitch", near(deg(e[1]), 89.0, 1e-6),
          `pitch=${deg(e[1]).toFixed(6)}（应 89°：asin 值域折叠）`);
    check("91°退化roll/yaw", near(deg(e[0]), 180, 1e-6) && near(deg(e[2]), 180, 1e-6),
          `roll=${deg(e[0]).toFixed(4)} yaw=${deg(e[2]).toFixed(4)}（应双 180°）`);

    /* 3) quatNorm 契约 */
    check("norm .5×4", near(globalThis.quatNorm(0.5, 0.5, 0.5, 0.5), 1));
    check("norm 3-4-5", near(globalThis.quatNorm(0.6, 0, 0.8, 0), 1));
    check("norm 非单位", near(globalThis.quatNorm(1, 1, 0, 0), Math.SQRT2));

    /* 4) QUAT 帧 → cube.quaternion.set(x,y,z,w)（THREE 约定，帧 w 在前） */
    env.emit("frameReady", 4, 1, [0.9, 0.1, 0.2, 0.3]);
    const cube = env.reg.groups[0];
    check("QUAT帧→cube", cube && cube.quaternion._args !== null);
    check("quat参数序(x,y,z,w)",
          cube && JSON.stringify(cube.quaternion._args) === JSON.stringify([0.1, 0.2, 0.3, 0.9]),
          cube ? JSON.stringify(cube.quaternion._args) : "no cube");
    check("四分量入面板", els["q-vals"].innerHTML.includes("0.3000"),
          els["q-vals"].innerHTML.slice(0, 80));
    check("欧拉对照表更新", els["e-pitch"].textContent !== "—",
          `e-pitch=${els["e-pitch"].textContent}`);

    /* 5) 锁指示：91° 姿态 → lock 类 + locknote；回小角度 → 解除 */
    env.emit("frameReady", 4, 2, [w915, 0, y915, 0]);
    check("锁类挂上", els["qr-pitch"].classList.has("lock"),
          `e-pitch=${els["e-pitch"].textContent}`);
    check("锁提示显示", els["locknote"].classList.has("show"));
    env.emit("frameReady", 4, 3, [0.9, 0.1, 0.2, 0.3]);
    check("锁类解除", !els["qr-pitch"].classList.has("lock"));

    /* IMU 帧只计数不驱动姿态（立方参数不变） */
    const before = JSON.stringify(cube.quaternion._args);
    env.emit("frameReady", 1, 1, [0.01, 0.02, 0.99, 0, 0, 0]);
    check("IMU帧不动立方", JSON.stringify(cube.quaternion._args) === before);

    /* 相机 up=Z（世界系 Z 上口径，与 exp5 同）——注：raw 面板的波形相机
       先建，故按 up=Z 认 3D 视图相机，而不是按创建序取 cams[0] */
    const cam = env.reg.cams.find((c) => c.up.z === 1);
    check("相机Z上", cam && cam.up.x === 0 && cam.up.y === 0 && cam.up.z === 1,
          cam ? `up=(${cam.up.x},${cam.up.y},${cam.up.z})` : "no cam");

    /* 6) 0x03 帧驱动欧拉条形面板（本页同时具备 0x03 通道） */
    env.rec.fills.length = 0;
    env.rec.dashes.length = 0;
    env.emit("frameReady", 3, 1, [0.1, -0.2, 0.3]);
    check("欧拉deg列", els["v-roll"].textContent.startsWith("+5.730"),
          `v-roll=${els["v-roll"].textContent}`);
    check("欧拉条形绘制", ["roll", "pitch", "yaw"].every((a) => env.rec.fills.includes(a)),
          env.rec.fills.join("|").slice(0, 100));
    check("pitch锁边界虚线", env.rec.dashes.some((d) => d[0] === 4 && d[1] === 3),
          JSON.stringify(env.rec.dashes));

    /* 7) startSim() → openSim + 串口列表 */
    globalThis.startSim();
    await new Promise((r) => setTimeout(r, 350));
    check("startSim→openSim", env.bridgeCalls.some((c) => c[0] === "openSim"),
          JSON.stringify(env.bridgeCalls));
    await new Promise((r) => setTimeout(r, 50));
    const opts = els["port"].options;
    check("串口列表填充", opts.length === 2 && opts[0].value === "COM3", `opts=${opts.length}`);

    /* 8) WebGL 降级守卫：渲染器构造抛错不得杀数据链路（换抛错桩重载页面核心） */
    try {
        const broken = H.makeEnv("exp4", { webglThrows: true });
        check("WebGL降级-不崩", !broken.bootError,
              broken.bootError ? broken.bootError.message : "");
        if (!broken.bootError) {
            broken.emit("frameReady", 4, 9, [0.9, 0.1, 0.2, 0.3]);
            check("WebGL降级-数据链路活", broken.els["e-pitch"].textContent !== "—",
                  `e-pitch=${broken.els["e-pitch"].textContent}`);
            check("WebGL降级-提示语", broken.els["cube-hint"].textContent.includes("WebGL"),
                  broken.els["cube-hint"].textContent.slice(0, 40));
            check("WebGL降级-换算契约仍在", typeof globalThis.quatToEulerZyx === "function");
        }
    } catch (e) {
        check("WebGL降级-不崩", false, `重载抛错: ${e.message}`);
    }
}

main().then(() => {
    fails.forEach((f) => console.log("  [FAIL]", f));
    console.log("== exp4_page_harness: 面板集合/换算契约/锁钉法/norm/quat序/锁指示/" +
                `0x03条形/startSim/串口列表/WebGL降级, ${fails.length} fail ==`);
    process.exit(fails.length ? 1 : 0);
});
