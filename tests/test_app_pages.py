"""app.py 页面路由测试（路由解析 = headless 可测部分；Qt 不起窗）。

对应矩阵一一对应断言（用户定案）：exp1_raw↔exp1_waveform｜exp2_calib↔exp2_bias｜
exp3_euler↔exp3_euler_view（独立页）｜exp4_quat↔exp4_quat_view｜exp5↔exp5_trajectory。

覆盖：
  1) 路由表全集 = exp1..exp5（exp2/exp3/exp4/exp5 全路由化，缺省 exp1）
  2) _pick_page：缺省 exp1；--page X 与 --page=X 两种写法逐页可解析；
     未知名 SystemExit（Qt 起前拦截）
  3) 页面文件现实一致：QWebChannel 页 html 存在；exp3 独立页三件套存在
  4) 注册表：全部页面走服务器（页面住实例目录 / exp3 独立页），服务器脚本与中枢就位
  5) exp3 串口位置参数拒绝（独立页 sim/tcp 源，不吃 COMx）

跑法：python tests/test_app_pages.py（exit 0 = 全过；需 PySide6——import 级，
不起 QApplication、不开窗口）
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import app  # noqa: E402  （import 即语法级验证：模块加载不崩）


fails = []


def check(name, ok, detail=""):
    if not ok:
        fails.append(f"{name}{('：' + detail) if detail else ''}")


def main() -> int:
    # 1) 路由表全集
    known = set(app.KNOWN_PAGES)
    want = {"exp1", "exp2", "exp3", "exp4", "exp5", "store_demo"}
    check("路由全集", known == want, f"got {sorted(known)}")
    check("缺省exp1", app.DEFAULT_PAGE == "exp1")
    # 3) 页面文件现实一致：服务器页的页面住各自实例目录
    for p in sorted(app.SERVER_PAGES):
        if p == "store_demo":
            rel = ("store_demo",)
        else:
            rel = {"exp1": ("exp1_waveform",), "exp2": ("exp2_bias",),
                   "exp4": ("exp4_quat_view",), "exp5": ("exp5_trajectory",)}[p]
        path = os.path.join(app._EXAMPLES, "imu", *rel, "index.html") \
            if p != "store_demo" else os.path.join(app._EXAMPLES, *rel, "index.html")
        check(f"{p} 页面在实例目录", os.path.isfile(path), path)
    check("exp3 独立页存在",
          os.path.isfile(os.path.join(app.EXP3_DIR, "euler_view.py"))
          and os.path.isfile(os.path.join(app.EXP3_DIR, "index.html"))
          and os.path.isfile(os.path.join(app.EXP3_DIR, "run.bat")),
          app.EXP3_DIR)
    check("服务器脚本存在", os.path.isfile(app.SERVER_SCRIPT), app.SERVER_SCRIPT)
    check("数据源中枢存在",
          os.path.isfile(os.path.join(os.path.dirname(app.SERVER_SCRIPT), "hub.py")))

    # 4) 注册表语义：全部页面走服务器（exp3 独立页除外），不再有 QWebChannel 页
    check("服务器页全集", set(app.SERVER_PAGES) == {"exp1", "exp2", "exp4", "exp5", "store_demo"},
          f"got {sorted(app.SERVER_PAGES)}")
    check("web/ 已无页面（页面迁实例目录）",
          not any(os.path.isfile(os.path.join(app._ROOT, "web", p, "index.html"))
                  for p in ("exp1", "exp2", "exp4", "exp5")))

    # 5) 串口参数提取 + exp3 拒绝路径
    check("串口参数提取", app._serial_arg(["app.py", "COM57"]) == "COM57")
    check("--page值不算串口", app._serial_arg(["app.py", "--page", "exp5", "COM57"])
          == "COM57")
    check("--page=式不算串口", app._serial_arg(["app.py", "--page=exp5"]) is None)
    check("sim无串口", app._serial_arg(["app.py", "--sim"]) is None)

    # 6) 显式模式（禁隐式回落）：缺省 real；--sim 旧别名；--mode 优先；非法值拒绝
    check("缺省模式=real（不自动仿真）", app._pick_mode(["app.py"]) == "real")
    check("缺省模式带串口仍 real",
          app._pick_mode(["app.py", "COM57"]) == "real")
    check("--sim 旧别名=sim", app._pick_mode(["app.py", "--sim"]) == "sim")
    check("--mode sim", app._pick_mode(["app.py", "--mode", "sim"]) == "sim")
    check("--mode=real", app._pick_mode(["app.py", "--mode=real"]) == "real")
    check("显式 --mode 胜 --sim",
          app._pick_mode(["app.py", "--mode", "real", "--sim"]) == "real")
    try:
        app._pick_mode(["app.py", "--mode", "qemu"])
        check("非法模式拒绝", False, "未抛 SystemExit")
    except SystemExit as e:
        check("非法模式拒绝", "qemu" in str(e), f"msg={e}")
    check("--mode 值不算串口",
          app._serial_arg(["app.py", "--mode", "sim", "COM57"]) == "COM57")

    # 7) 语言参数（缺省 zh；非法值拒绝）
    check("缺省语言=zh", app._pick_lang(["app.py"]) == "zh")
    check("--lang en", app._pick_lang(["app.py", "--lang", "en"]) == "en")
    check("--lang=zh", app._pick_lang(["app.py", "--lang=zh"]) == "zh")
    try:
        app._pick_lang(["app.py", "--lang", "fr"])
        check("非法语言拒绝", False, "未抛 SystemExit")
    except SystemExit as e:
        check("非法语言拒绝", "fr" in str(e), f"msg={e}")

    # exp3 拒绝串口参数：SystemExit 在 Qt 起之前抛出——headless 直测 main 分支
    saved_argv = sys.argv
    sys.argv = ["app.py", "--page", "exp3", "COM57"]
    try:
        app.main()
        check("exp3拒绝串口", False, "应抛 SystemExit")
    except SystemExit as e:
        check("exp3拒绝串口", "exp3" in str(e) and "COM57" in str(e), f"msg={e}")
    finally:
        sys.argv = saved_argv

    print(f"== test_app_pages: 路由全集/解析/文件一致/桥选型/参数提取/显式模式/语言, "
          f"{len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
