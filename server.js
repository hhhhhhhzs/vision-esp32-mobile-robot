/*
  IoT Car Cloud Server (HTTP + WS)
  - HTTP:
      GET  /               -> web_console (静态网页)
      POST /control         -> {device, command} 转发给小车
      GET  /stream?device=  -> MJPEG (multipart) 视频流
      GET  /health          -> 健康检查
  - WebSocket (小车端):
      ws://host:port/ws?device=car1
        * TEXT: 小车 -> 服务器 -> (可选)日志
        * BIN : JPG 帧 -> 推给 /stream 的 MJPEG 客户端
        * 兼容 BIN 帧格式：
            1) 纯 JPEG
            2) "JPG1" + JPEG   （你当前固件就是这个） 见固件注释：必须一次 sendBIN 发完
            3) 0xAA 0x55 lenLE16 + JPEG（旧协议）
      ws://host:port/ws_audio?device=car1
        * BIN : 小车麦克风 PCM16(16k,mono) -> 转发给浏览器 /audio
  - WebSocket (浏览器端):
      ws://host:port/audio?device=car1
        * BIN : 浏览器麦克风 PCM16 -> 转发给小车 ws_audio（喇叭播放）
        * BIN : 小车麦克风 PCM16 -> 服务器广播给所有浏览器音频 WS
*/

const http = require("http");
const path = require("path");
const fs = require("fs");
const express = require("express");
const bodyParser = require("body-parser");
const { WebSocketServer } = require("ws");

const PORT = process.env.PORT ? parseInt(process.env.PORT, 10) : 12060;

function now() {
  return new Date().toISOString().replace("T", " ").split(".")[0];
}

function parseDevice(reqUrl) {
  try {
    const u = new URL(reqUrl, "http://localhost");
    return u.searchParams.get("device") || "car1";
  } catch (e) {
    return "car1";
  }
}

// ---------- web_console 路径自动探测（避免你把 server.js 放到不同目录导致 Cannot GET /） ----------
function pickWebDir() {
  const candidates = [
    path.join(__dirname, "web_console"),          // server.js 与 web_console 同级
    path.join(__dirname, "..", "web_console"),    // server.js 在 server/ 子目录
    "/root/clou/web_console",                     // 常见部署路径（可按你自己的改）
    "/root/web_console",
  ];

  for (const d of candidates) {
    try {
      if (fs.existsSync(path.join(d, "index.html"))) return d;
    } catch (_) {}
  }
  return null;
}

const WEB_DIR = pickWebDir();

const app = express();
app.use(bodyParser.json({ limit: "1mb" }));

if (WEB_DIR) {
  app.use(express.static(WEB_DIR));
  // 明确兜底一下 /，避免某些环境下 static index 行为被改
  app.get("/", (req, res) => res.sendFile(path.join(WEB_DIR, "index.html")));
  console.log(`[${now()}] WEB_DIR=${WEB_DIR}`);
} else {
  console.log(`[${now()}] WEB_DIR NOT FOUND (no web_console/index.html). / will show a hint page.`);
  app.get("/", (req, res) => {
    res.status(200).type("text").send(
      "web_console not found.\n" +
      "Please upload web_console folder (with index.html) next to server.js or in ../web_console.\n"
    );
  });
}

app.get("/health", (req, res) => res.json({ ok: true, ts: Date.now() }));

// ---------- devices ----------
const devices = new Map();
/*
  devices.get(device) = {
    carWs: WebSocket|null,
    carAudioWs: WebSocket|null,
    mjpegClients: Set<res>,
    audioClients: Set<WebSocket>,
    lastFrame: Buffer|null
  }
*/
function getDev(device) {
  if (!devices.has(device)) {
    devices.set(device, {
      carWs: null,
      carAudioWs: null,
      mjpegClients: new Set(),
      audioClients: new Set(),
      lastFrame: null,
    });
  }
  return devices.get(device);
}

// ---------- HTTP: control ----------
app.post("/control", (req, res) => {
  const device = (req.body && req.body.device) ? String(req.body.device) : "car1";
  const command = (req.body && req.body.command) ? String(req.body.command) : "";
  const d = getDev(device);

  if (!command) return res.status(400).json({ ok: false, error: "command required" });
  if (!d.carWs || d.carWs.readyState !== 1) {
    return res.status(503).json({ ok: false, error: "car not connected", device });
  }

  try {
    d.carWs.send(command);
    return res.json({ ok: true, device, command });
  } catch (e) {
    return res.status(500).json({ ok: false, error: String(e) });
  }
});

// ---------- HTTP: stream (MJPEG) ----------
app.get("/stream", (req, res) => {
  const device = req.query.device ? String(req.query.device) : "car1";
  const d = getDev(device);

  res.writeHead(200, {
    "Cache-Control": "no-cache, no-store, must-revalidate",
    "Pragma": "no-cache",
    "Connection": "close",
    "Content-Type": "multipart/x-mixed-replace; boundary=frame",
  });

  d.mjpegClients.add(res);

  // 立刻推一帧（如果有）
  if (d.lastFrame) writeJpegToMjpeg(res, d.lastFrame);

  req.on("close", () => d.mjpegClients.delete(res));
});

function writeJpegToMjpeg(res, jpegBuf) {
  try {
    res.write(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpegBuf.length}\r\n\r\n`);
    res.write(jpegBuf);
    res.write("\r\n");
  } catch (_) {}
}

// ---------- JPEG frame compatibility ----------
function extractJpeg(buf) {
  // 1) "JPG1" + jpeg
  if (buf.length > 4 &&
      buf[0] === 0x4A && buf[1] === 0x50 && buf[2] === 0x47 && buf[3] === 0x31) {
    return buf.slice(4);
  }
  // 2) 0xAA 0x55 lenLE16 + jpeg
  if (buf.length >= 6 && buf[0] === 0xAA && buf[1] === 0x55) {
    const len = buf[2] | (buf[3] << 8);
    if (len > 0 && buf.length >= 4 + len) return buf.slice(4, 4 + len);
  }
  // 3) assume pure jpeg
  return buf;
}

// ---------- WebSocket upgrade routing ----------
const server = http.createServer(app);

const wssCar = new WebSocketServer({ noServer: true });
const wssCarAudio = new WebSocketServer({ noServer: true });
const wssWebAudio = new WebSocketServer({ noServer: true });

server.on("upgrade", (req, socket, head) => {
  const pathname = new URL(req.url, "http://localhost").pathname;

  if (pathname === "/ws") {
    wssCar.handleUpgrade(req, socket, head, (ws) => wssCar.emit("connection", ws, req));
    return;
  }
  if (pathname === "/ws_audio") {
    wssCarAudio.handleUpgrade(req, socket, head, (ws) => wssCarAudio.emit("connection", ws, req));
    return;
  }
  if (pathname === "/audio") {
    wssWebAudio.handleUpgrade(req, socket, head, (ws) => wssWebAudio.emit("connection", ws, req));
    return;
  }

  socket.destroy();
});

// ---------- WS: car control/video ----------
wssCar.on("connection", (ws, req) => {
  const device = parseDevice(req.url);
  const d = getDev(device);

  if (d.carWs && d.carWs.readyState === 1) {
    try { d.carWs.close(1000, "replaced"); } catch (_) {}
  }
  d.carWs = ws;
  console.log(`[${now()}] [CAR] connected device=${device}`);

  ws.on("message", (data, isBinary) => {
    if (isBinary) {
      const raw = Buffer.from(data);
      const jpeg = extractJpeg(raw);
      d.lastFrame = jpeg;
      for (const res of d.mjpegClients) writeJpegToMjpeg(res, jpeg);
      return;
    }
    // 文本：日志/状态（可扩展）
    // const msg = data.toString();
  });

  ws.on("close", () => {
    if (d.carWs === ws) d.carWs = null;
    console.log(`[${now()}] [CAR] disconnected device=${device}`);
  });
  ws.on("error", (e) => console.log(`[${now()}] [CAR] error device=${device}:`, e.message || e));
});

// ---------- WS: car audio (mic -> web, web -> spk) ----------
wssCarAudio.on("connection", (ws, req) => {
  const device = parseDevice(req.url);
  const d = getDev(device);

  if (d.carAudioWs && d.carAudioWs.readyState === 1) {
    try { d.carAudioWs.close(1000, "replaced"); } catch (_) {}
  }
  d.carAudioWs = ws;
  console.log(`[${now()}] [CAR-AUDIO] connected device=${device}`);

  ws.on("message", (data, isBinary) => {
    if (!isBinary) return;
    // 小车麦克风 -> 广播给所有浏览器音频客户端
    for (const client of d.audioClients) {
      if (client.readyState === 1) client.send(data, { binary: true });
    }
  });

  ws.on("close", () => {
    if (d.carAudioWs === ws) d.carAudioWs = null;
    console.log(`[${now()}] [CAR-AUDIO] disconnected device=${device}`);
  });
});

// ---------- WS: web audio ----------
wssWebAudio.on("connection", (ws, req) => {
  const device = parseDevice(req.url);
  const d = getDev(device);

  d.audioClients.add(ws);
  console.log(`[${now()}] [WEB-AUDIO] connected device=${device} clients=${d.audioClients.size}`);

  ws.on("message", (data, isBinary) => {
    if (!isBinary) return;
    // 浏览器麦克风 -> 转发给小车音频 WS（喇叭播放）
    if (d.carAudioWs && d.carAudioWs.readyState === 1) {
      d.carAudioWs.send(data, { binary: true });
    }
  });

  ws.on("close", () => {
    d.audioClients.delete(ws);
    console.log(`[${now()}] [WEB-AUDIO] disconnected device=${device} clients=${d.audioClients.size}`);
  });
});

server.listen(PORT, () => {
  console.log(`[${now()}] Server listening on :${PORT}`);
  console.log(`- Open http://<server_ip>:${PORT}/  (建议部署 HTTPS 才能用浏览器麦克风)`);
});
