/* live.js — 服务器版上位机的页面数据客户端（零依赖，EventSource/SSE + fetch）。
 *
 * 背景（用户定案 2026-11-05）：上位机 = 本地服务器 + 各实验实例目录里的页面。
 * 页面不再走 Qt/QWebChannel，而是：
 *   页面 ──GET /api/stream（SSE）──> 服务器（backend/server.py）
 *        ──POST /api/open|close|calib──> 数据源中枢（backend/hub.py）
 *
 * 本文件刻意**保留旧 bridge 的信号式 API**（frameReady.connect(fn) 等），
 * 让页面业务代码零改：只有「怎么拿到 bridge」这一处从 QWebChannel 换成 LiveBridge()。
 *
 *   const bridge = LiveBridge();
 *   bridge.frameReady.connect((type, seq, vals) => {...});
 *   bridge.openSim(); / bridge.openPort("COM5", 115200); / bridge.closePort();
 *
 * 契约（与服务器事件一一对应）：
 *   frameReady(type, seq, vals)   解码值（IMU_SAMPLE=6 / IMU_CAL=19 / QUAT=4）
 *   textReady(str)                TEXT 帧 / 服务器错误文案
 *   statReady(imu, text, drop)    计数
 *   posReady(x, y, z)             轨迹重建位置（exp5）
 *   portChanged(bool)             连接状态
 */
(function (root) {
  "use strict";

  function signal() {
    var handlers = [];
    return {
      connect: function (fn) { if (typeof fn === "function") handlers.push(fn); },
      emit: function () {
        var args = arguments;
        for (var i = 0; i < handlers.length; i++) {
          try { handlers[i].apply(null, args); } catch (e) { /* 单个处理器异常不扩散 */ }
        }
      },
    };
  }

  function postJSON(url, body) {
    return fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    }).then(function (res) {
      return res.json().catch(function () { return {}; }).then(function (data) {
        if (!res.ok) {
          var msg = (data && data.error) || ("HTTP " + res.status);
          throw new Error(msg);
        }
        return data;
      });
    });
  }

  function getJSON(url) {
    return fetch(url, { cache: "no-store" }).then(function (res) { return res.json(); });
  }

  function LiveBridge() {
    var bridge = {
      frameReady: signal(),
      textReady: signal(),
      statReady: signal(),
      posReady: signal(),
      portChanged: signal(),
      connected: false,
      mode: "none",
      state: null,

      listPorts: function () {
        return getJSON("/api/ports").then(function (d) {
          return (d.ports || []).map(function (p) { return p.device; });
        }).catch(function () { return []; });
      },

      openSim: function () {
        return postJSON("/api/open", { mode: "sim" }).then(function (s) {
          bridge._applyState(s);
          return s;
        }).catch(function (e) {
          bridge.textReady.emit("[host] 启动仿真失败：" + e.message);
          return null;
        });
      },

      openPort: function (port, baud) {
        return postJSON("/api/open", { mode: "real", port: port, baud: baud || 115200 })
          .then(function (s) {
            bridge._applyState(s);
            return s;
          }).catch(function (e) {
            bridge.textReady.emit("[host] 打开串口失败：" + e.message);
            bridge.portChanged.emit(false);
            return null;
          });
      },

      closePort: function () {
        return postJSON("/api/close").then(function (s) {
          bridge._applyState(s);
          return s;
        }).catch(function () { return null; });
      },

      sendCalib: function () {
        return postJSON("/api/calib").catch(function (e) {
          return { code: "notConnected", detail: e.message };
        });
      },

      _applyState: function (s) {
        if (!s) return;
        bridge.state = s;
        bridge.mode = s.mode;
        var on = !!s.connected;
        if (on !== bridge.connected) {
          bridge.connected = on;
          bridge.portChanged.emit(on);
        }
        var c = s.counters || {};
        if (typeof c.imu === "number") {
          bridge.statReady.emit(c.imu, c.text || 0, c.drop || 0);
        }
      },

      _stream: function () {
        if (typeof EventSource === "undefined") {
          bridge.textReady.emit("[host] 浏览器不支持 EventSource（需要现代浏览器或 Qt WebEngine）");
          return;
        }
        var es = new EventSource("/api/stream");
        bridge._es = es;
        es.addEventListener("state", function (e) { bridge._applyState(JSON.parse(e.data)); });
        es.addEventListener("port", function (e) {
          var d = JSON.parse(e.data);
          if (!!d.connected !== bridge.connected) {
            bridge.connected = !!d.connected;
            bridge.portChanged.emit(bridge.connected);
          }
        });
        es.addEventListener("frame", function (e) {
          var d = JSON.parse(e.data);
          bridge.frameReady.emit(d.type, d.seq, d.vals);
        });
        es.addEventListener("stat", function (e) {
          var d = JSON.parse(e.data);
          bridge.statReady.emit(d.imu, d.text, d.drop);
        });
        es.addEventListener("text", function (e) {
          bridge.textReady.emit(JSON.parse(e.data));
        });
        es.addEventListener("pos", function (e) {
          var d = JSON.parse(e.data);
          bridge.posReady.emit(d.x, d.y, d.z);
        });
        es.onerror = function () {
          /* EventSource 自带重连；这里只把状态显示为未连接，不触发任何自动重启 */
          if (bridge.connected) {
            bridge.connected = false;
            bridge.portChanged.emit(false);
          }
        };
      },
    };
    bridge._stream();
    return bridge;
  }

  root.LiveBridge = LiveBridge;
  if (typeof module !== "undefined" && module.exports) module.exports = LiveBridge;
})(typeof globalThis !== "undefined" ? globalThis : this);
