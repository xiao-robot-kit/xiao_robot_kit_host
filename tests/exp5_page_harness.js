/* exp5_page_harness.js — exp5 页面（+位移轨迹）逻辑 harness（node，无浏览器）。
 *
 * 页面形态变更（用户定案 2026-11-06「逐级继承」）：exp5 页已改为薄壳
 * （examples/imu/exp5_trajectory/index.html 只声明
 * panels=["raw","cal","euler","quat","traj"]），实现搬进统一核心
 * web/assets/imu-page.js。本文件按新页面断言同一组能力，覆盖不降：
 *
 *   0) 薄壳面板集合 = exp4 的全部能力 + traj（逐级继承末端，缺一即 bug）
 *   1) QUAT 帧驱动 marker.quaternion.set(x,y,z,w) 参数序（THREE 约定）
 *   2) posReady 驱动 marker.position.set + 轨迹缓冲追加（经 rAF 泵帧走真实
 *      tick→updateTrailAlpha 路径，drawRange 随动）
 *   3) 尾部渐隐：最新点 alpha=1、头部窗末点 alpha=1、最旧点 alpha≈0
 *   4) 满窗滑窗：超容量后 drawRange 封顶、首点被挤出（copyWithin 左移）
 *   5) 清除轨迹（btn-clear 处理器路径）
 *   6) startSim() → openSim；listPorts Promise 填充串口下拉
 *
 * WebGL 像素渲染与鼠标轨道控制不在此层——属人工开窗口项。
 * 跑法：node tests/exp5_page_harness.js（exit 0 = 全过；需 node>=18）
 */

"use strict";
const H = require("./imu_page_harness.js");

const { fails, check } = H.createRun();
const NEAR = 1e-6;

async function main() {
    const env = H.makeEnv("exp5");
    if (env.bootError) {
        fails.push(`页面脚本加载崩溃: ${env.bootError.message}`);
        return;
    }
    const els = env.els;
    check("薄壳声明 exp5 面板集合",
          JSON.stringify(env.shell.cfg.panels) ===
          JSON.stringify(["raw", "cal", "euler", "quat", "traj"]),
          JSON.stringify(env.shell.cfg.panels));
    check("禁隐式回落：加载后未启源", env.bridgeCalls.length === 0,
          JSON.stringify(env.bridgeCalls));

    const marker = env.reg.groups[1];           // groups[0] = 立方（quat 面板）
    const trailGeom = env.reg.geoms.find((g) => g._attrs.aAlpha);
    const cam = env.reg.cams.find((c) => c.up.z === 1);
    check("轨迹几何存在", !!trailGeom);
    check("姿态标记存在", !!marker);

    /* 1) QUAT 帧参数序：THREE.Quaternion.set(x,y,z,w)；页面收 (w,x,y,z) */
    env.emit("frameReady", 4, 1, [0.9, 0.1, 0.2, 0.3]);
    check("QUAT 帧→marker", marker && marker.quaternion._args !== null);
    check("quat 参数序(x,y,z,w)",
          marker && JSON.stringify(marker.quaternion._args) ===
          JSON.stringify([0.1, 0.2, 0.3, 0.9]),
          marker ? JSON.stringify(marker.quaternion._args) : "no marker");

    /* 2) IMU 帧不崩 + 相机 up=Z（世界系 Z 上口径） */
    env.emit("frameReady", 1, 1, [0.01, 0.02, 0.99, 0, 0, 0]);
    check("相机 Z 上", cam && cam.up.x === 0 && cam.up.y === 0 && cam.up.z === 1,
          cam ? `up=(${cam.up.x},${cam.up.y},${cam.up.z})` : "no cam");

    /* 3) posReady ×300 → 轨迹 + 标记 + drawRange（泵两帧走真实 tick） */
    for (let i = 0; i < 300; i++) {
        env.emit("posReady", i * 0.001, -i * 0.002, i * 0.0005);
    }
    env.pumpFrame(0);
    env.pumpFrame(100);
    check("轨迹追加(drawRange)",
          trailGeom && trailGeom._range[0] === 0 && trailGeom._range[1] === 300,
          trailGeom ? JSON.stringify(trailGeom._range) : "");
    check("标记位置", marker && marker.position.x === 299 * 0.001 &&
          marker.position.y === -299 * 0.002 && marker.position.z === 299 * 0.0005,
          marker ? `pos=(${marker.position.x},${marker.position.y},${marker.position.z})` : "");

    /* 4) 尾部渐隐：最新/头部窗末点不透明，最旧点≈0，单调 */
    if (trailGeom) {
        const al = trailGeom._attrs.aAlpha.array;
        check("最新点不透明", al[299] === 1, `alpha[299]=${al[299]}`);
        check("头部窗末点不透明", al[150] === 1, `alpha[150]=${al[150]}`);
        check("最旧点渐隐≈0", al[0] < 0.01, `alpha[0]=${al[0]}`);
        check("渐隐单调", al[299] >= al[200] && al[200] >= al[100] && al[100] >= al[0],
              `a[100]=${al[100]} a[200]=${al[200]}`);
    }

    /* 5) 满窗滑窗：塞超容量（+50 点），drawRange 封顶、首点被挤出 */
    const MAX = 6000;
    for (let i = 0; i < MAX + 50; i++) env.emit("posReady", i * 0.001, 0, 0);
    env.pumpFrame(200);
    check("滑窗封顶", trailGeom && trailGeom._range[1] === MAX,
          trailGeom ? `count=${trailGeom._range[1]}` : "");
    if (trailGeom) {
        const pa = trailGeom._attrs.position.array;
        check("滑窗左移", Math.abs(pa[0] - 50 * 0.001) < NEAR,
              `first_x=${pa[0]}（应≈第 51 点 0.05）`);
        check("末点最新", Math.abs(pa[(MAX - 1) * 3] - (MAX + 49) * 0.001) < NEAR,
              `last_x=${pa[(MAX - 1) * 3]}`);
    }

    /* 6) 清除轨迹（按钮处理器路径） */
    els["btn-clear"].onclick();
    check("清除轨迹", trailGeom && trailGeom._range[1] === 0,
          trailGeom ? `count=${trailGeom._range[1]}` : "");

    /* 7) startSim() → openSim（真实 node 定时器；app.py kick 同路径） */
    globalThis.startSim();
    await new Promise((r) => setTimeout(r, 350));
    check("startSim→openSim", env.bridgeCalls.some((c) => c[0] === "openSim"),
          `calls=${JSON.stringify(env.bridgeCalls)}`);

    /* 8) listPorts Promise 填充串口下拉 */
    await new Promise((r) => setTimeout(r, 50));
    const opts = els["port"].options;
    check("串口列表填充", opts.length === 2 && opts[0].value === "COM3", `opts=${opts.length}`);
}

main().then(() => {
    fails.forEach((f) => console.log("  [FAIL]", f));
    console.log("== exp5_page_harness: 面板集合/quat序/Z上/轨迹追加/渐隐/滑窗/" +
                `清除/startSim/串口列表, ${fails.length} fail ==`);
    process.exit(fails.length ? 1 : 0);
});
