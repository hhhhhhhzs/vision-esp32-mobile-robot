/*
  ESP32-S3-CAM (GOOUUU) + OV3660
  - WebSocket 控制 + 二进制视频帧推流
  - PCA9685：4个直流电机（CH0~CH7） + 3个舵机（CH8~CH10）
  - OLED(I2C) 表情
  - 板载补光：WS2812 (GPIO48)
  - 音频：MAX98357A(I2S OUT) + INMP441(I2S MIC)

  迁移要点：
  - I2C: SDA=GPIO41, SCL=GPIO42
  - Flash: WS2812 GPIO48
  - Camera: OV3660 (GOOUUU ESP32-S3-CAM pin map)

  Web command（兼容旧网页）：
    forward/backward/left/right/rotate_left/rotate_right/stop
    camera_on/camera_off
    camera_light_on/camera_light_off
    emoji_happy/emoji_sad/emoji_angry/emoji_surprise/emoji_sleep/emoji_disdain/emoji_cat/emoji_auto/emoji_off
    arm_up/arm_down/arm_stop            -> 舵机CH8（大臂）
    forearm_up/forearm_down/forearm_stop-> 舵机CH9（小臂）
    claw_open/claw_close/claw_stop      -> 舵机CH10（夹爪）
    servo8_<0~180> / servo9_<0~180> / servo10_<0~180>
    sound_beep / sound_meow / sound_on / sound_off
    mic_on / mic_off   (上报 RMS 音量)
*/

#include <WiFi.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>

#include "driver/i2s.h"
#include <esp_heap_caps.h>
#include <math.h>

// FreeRTOS types used across the file
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ---- Forward state vars (used by functions defined earlier in the .ino) ----
// Arduino .ino does NOT auto-forward-declare global variables.
// These are referenced by oledLoop()/standbyApplyOutputsOff() which appear
// above their original definitions.
// standbyMode is declared near the top (forward state vars)
static volatile bool standbyMode = false;
static QueueHandle_t qMic = nullptr;
static QueueHandle_t qSpk = nullptr;

// ================= WiFi / Server =================
static const char* WIFI_SSID = "408";
static const char* WIFI_PASS = "123456789";

static const char* WS_HOST = "112.126.85.212";
static const uint16_t WS_PORT = 12060;
static const char* WS_PATH = "/ws?device=car1";



// ====== 配网/持久化配置（NVS）======
// 说明：
// - 首次烧录：如果 NVS 里没有配置，会使用上面写死的 WIFI_SSID/WIFI_PASS/WS_* 作为默认值
// - 之后改网：连接失败 30s 自动进入配网模式（AP + 网页配置），保存到 NVS 后自动重启生效
static Preferences prefs;
static WebServer  cfgServer(80);
static DNSServer  dnsServer;

static bool cfgMode = false;            // 是否处于配网模式
static uint32_t wifiLostSince = 0;      // 断网开始计时（用于 30s 进入配网）
static uint32_t cfgPortalStartMs = 0;   // 进入配网模式的时间戳
static uint32_t cfgLastActivityMs = 0; // 最近一次“有人在配网”的时间戳（HTTP访问/有设备连AP）
static uint8_t  cfgLastStaNum = 0;     // AP 连接的站点数（用于判断是否有人连上热点）
static const uint32_t CFG_IDLE_TO_RETRY_MS = 60000; // 配网 1min 没动静 -> 退出配网，重试连网
static inline void cfgTouch();
static void stopConfigPortal();
static void configPortalIdleCheckAndRetryWiFi();


static String cfg_ssid;
static String cfg_pass;
static String cfg_ws_host;
static uint16_t cfg_ws_port = WS_PORT;
static String cfg_ws_path;
static String cfg_ws_audio_path;  // 由 cfg_ws_path 自动推导出来的音频 WS 路径（必须是全局，避免临时 String 失效）

// ================= I2C (用户要求 41/42) =================
static const int I2C_SDA = 41;
static const int I2C_SCL = 42;

static const uint8_t PCA_ADDR = 0x40;
static uint8_t OLED_ADDR_7BIT = 0x3C;

// ================= WS2812（氛围彩灯 / 板载补光） =================
// 你现在板子上的“补光灯”变成了 WS2812（GPIO48）。
// 本工程把它当作“氛围彩灯”来用：网页可控制颜色/呼吸/闪烁/关闭。
// 注意：我们不做“联网成功/配网状态/WS连接状态”的自动灯效（按你的要求）——默认一直是关的。

#define WS2812_PIN        48
#define WS2812_COUNT      1
#define WS2812_BRIGHTNESS 255

Adafruit_NeoPixel ws2812(WS2812_COUNT, WS2812_PIN, NEO_GRB + NEO_KHZ800);

enum LedMode : uint8_t { LED_OFF=0, LED_STATIC=1, LED_BREATH=2, LED_FLASH=3 };
static LedMode ledMode = LED_OFF;
static uint8_t ledR=0, ledG=0, ledB=0;
static uint32_t ledStartMs = 0;

// 呼吸周期（毫秒）
static uint16_t ledBreathPeriodMs = 1500;

// flash：点亮持续时间
static uint16_t ledFlashOnMs = 120;

static inline uint8_t clamp8(int v) { return (v<0)?0: (v>255?255:(uint8_t)v); }

static inline uint32_t rgbColor(uint8_t r, uint8_t g, uint8_t b) {
  return ws2812.Color(r, g, b);
}

static inline void ledApply(uint8_t r, uint8_t g, uint8_t b) {
  ws2812.setPixelColor(0, rgbColor(r,g,b));
  ws2812.show();
}

// 初始化：默认关灯
static inline void flashInit() {
  ws2812.begin();
  ws2812.setBrightness(WS2812_BRIGHTNESS);
  ws2812.clear();
  ws2812.show();
  ledMode = LED_OFF;
}

// 兼容旧网页命令：camera_light_on/off
static inline void flashOn() {
  ledMode = LED_STATIC;
  ledR = 255; ledG = 255; ledB = 255;
  ledApply(ledR, ledG, ledB);
}
static inline void flashOff() {
  ledMode = LED_OFF;
  ws2812.clear();
  ws2812.show();
}

// 新：氛围灯控制
static inline void ledOff() {
  ledMode = LED_OFF;
  ws2812.clear();
  ws2812.show();
}

static inline void ledStatic(uint8_t r, uint8_t g, uint8_t b) {
  ledMode = LED_STATIC;
  ledR = r; ledG = g; ledB = b;
  ledApply(ledR, ledG, ledB);
}

static inline void ledBreath(uint8_t r, uint8_t g, uint8_t b, uint16_t periodMs=1500) {
  ledMode = LED_BREATH;
  ledR = r; ledG = g; ledB = b;
  ledBreathPeriodMs = (periodMs < 300) ? 300 : periodMs; // 太短会闪
  ledStartMs = millis();
}

static inline void ledFlashOnce(uint8_t r, uint8_t g, uint8_t b, uint16_t onMs=120) {
  ledMode = LED_FLASH;
  ledR = r; ledG = g; ledB = b;
  ledFlashOnMs = (onMs < 30) ? 30 : onMs;
  ledStartMs = millis();
  ledApply(ledR, ledG, ledB);
}

// 呼吸/闪烁状态机（放在 loop() 里周期调用）
static inline void ledLoop() {
  const uint32_t now = millis();

  if (ledMode == LED_OFF || ledMode == LED_STATIC) return;

  if (ledMode == LED_FLASH) {
    // 点亮 onMs 后熄灭，并自动回到 OFF
    if (now - ledStartMs >= ledFlashOnMs) {
      ledOff();
    }
    return;
  }

  if (ledMode == LED_BREATH) {
    // 三角波 + 简单 gamma（让呼吸更柔）
    uint32_t p = ledBreathPeriodMs;
    uint32_t ph = (now - ledStartMs) % p;      // 0..p-1
    uint32_t half = p / 2;
    uint32_t t = (ph < half) ? ph : (p - ph);  // 0..half
    uint8_t bri = (uint8_t)((t * 255) / (half ? half : 1));
    // gamma-ish：亮度平方（更像“呼吸”）
    uint16_t bri2 = (uint16_t)bri * (uint16_t)bri;
    bri = (uint8_t)(bri2 / 255);

    uint8_t r = (uint8_t)((uint16_t)ledR * bri / 255);
    uint8_t g = (uint8_t)((uint16_t)ledG * bri / 255);
    uint8_t b = (uint8_t)((uint16_t)ledB * bri / 255);
    ledApply(r, g, b);
  }
}

// 解析网页命令：
// 1) led:off
// 2) led:r,g,b
// 3) led:breath,r,g,b            （默认周期 1500ms）
// 4) led:breath,r,g,b,periodMs   （可选）
// 5) led:flash,r,g,b             （默认点亮 120ms）
// 6) led:flash,r,g,b,onMs        （可选）
static bool handleLedCmd(const String& cmd) {
  if (!cmd.startsWith("led:")) return false;

  String p = cmd.substring(4);
  p.trim();

  if (p == "off") { ledOff(); return true; }

  // led:breath,...
  if (p.startsWith("breath")) {
    String rest = p.substring(6);
    if (rest.startsWith(",")) rest = rest.substring(1);
    rest.trim();

    int v[4] = {0,0,0,1500};
    int k=0, pos=0;
    while (k<4) {
      int c = rest.indexOf(',', pos);
      String t = (c>=0)? rest.substring(pos,c) : rest.substring(pos);
      t.trim();
      if (t.length()==0) break;
      v[k++] = t.toInt();
      if (c<0) break;
      pos = c+1;
    }
    if (k>=3) ledBreath(clamp8(v[0]), clamp8(v[1]), clamp8(v[2]), (k>=4)? (uint16_t)v[3] : 1500);
    return true;
  }

  // led:flash,...
  if (p.startsWith("flash")) {
    String rest = p.substring(5);
    if (rest.startsWith(",")) rest = rest.substring(1);
    rest.trim();

    int v[4] = {255,255,255,120};
    int k=0, pos=0;
    while (k<4) {
      int c = rest.indexOf(',', pos);
      String t = (c>=0)? rest.substring(pos,c) : rest.substring(pos);
      t.trim();
      if (t.length()==0) break;
      v[k++] = t.toInt();
      if (c<0) break;
      pos = c+1;
    }
    if (k>=3) ledFlashOnce(clamp8(v[0]), clamp8(v[1]), clamp8(v[2]), (k>=4)? (uint16_t)v[3] : 120);
    return true;
  }

  // led:r,g,b
  int comma1 = p.indexOf(',');
  int comma2 = (comma1>=0)? p.indexOf(',', comma1+1) : -1;
  if (comma1>=0 && comma2>=0) {
    int r = p.substring(0, comma1).toInt();
    int g = p.substring(comma1+1, comma2).toInt();
    int b = p.substring(comma2+1).toInt();
    ledStatic(clamp8(r), clamp8(g), clamp8(b));
    return true;
  }

  // 兜底：无法解析就关灯
  ledOff();
  return true;
}

// ================= Video / Safety =================
static bool cameraEnabled = false;
// ===== Stream tuning（画质/延迟平衡版） =====
// 你反馈“QQVGA 太糊”，这里默认提升到 QVGA（320x240）。
// 如果你觉得仍然卡：优先把 STREAM_FPS 降到 4；或把 JPEG_QUALITY 提高到 14~16。
// 如果你想更清晰：把 FRAME_SIZE 改为 FRAMESIZE_VGA，并把 FPS 调到 2~3。
#define STREAM_FPS          5
#define STREAM_FRAME_SIZE   FRAMESIZE_QVGA
#define STREAM_JPEG_QUALITY 12  // 10~14 画质较好；数值越大压缩越强(画质越差)
#define STREAM_FB_COUNT     1   // 1 + WHEN_EMPTY：最不容易 FB-OVF，延迟也更稳

static const uint32_t FRAME_INTERVAL_MS = 1000 / STREAM_FPS;  // 推流节流

static uint32_t lastFrameMs = 0;

static const uint32_t CMD_TIMEOUT_MS = 450;
static uint32_t lastCmdMs = 0;
static uint32_t lastPingMs = 0;

// 相机状态：
// - cameraEnabled：功能开关（网页 camera_on/off 控制）
// - cameraInited ：驱动是否已初始化（esp_camera_init 成功后为 true）
//   说明：待机想让“摄像头彻底不工作”，需要在待机时 deinit；唤醒再 init。
static bool cameraInited = false;
static bool cameraDeinitPending = false;  // 进入待机后在 loop() 里执行 deinit（避免回调里做重活）

// ================= Objects =================
WebSocketsClient ws;
WebSocketsClient wsAudio;  // 音频对讲 WebSocket（/ws_audio）
Adafruit_PWMServoDriver pca(PCA_ADDR);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ================= Camera pin map (GOOUUU ESP32-S3-CAM) =================
// 图中标注：SIOD=GPIO4, SIOC=GPIO5, VSYNC=GPIO6, HREF=GPIO7, XCLK=GPIO15, PCLK=GPIO13
// Y2=GPIO11, Y3=GPIO9, Y4=GPIO8, Y5=GPIO10, Y6=GPIO12, Y7=GPIO18, Y8=GPIO17, Y9=GPIO16
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;  // 降低XCLK，减少总吞吐，避免FB-OVF
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;  // 只在缓冲空时采集，避免发送阻塞时溢出
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    config.frame_size   = STREAM_FRAME_SIZE;
    config.jpeg_quality = STREAM_JPEG_QUALITY;
    config.fb_count     = STREAM_FB_COUNT;
  } else {
    config.frame_size   = STREAM_FRAME_SIZE;
    config.jpeg_quality = STREAM_JPEG_QUALITY;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init failed: 0x%X\n", err);
    return false;
  }

  // 标记相机已初始化（用于待机时安全停用/恢复）
  // 注意：只“停止推流”不会真正关闭传感器与 XCLK。
  // 如需待机时让摄像头彻底不工作（更省电/更少干扰），需要 esp_camera_deinit()。

  sensor_t* s = esp_camera_sensor_get();
  if (s && s->id.PID == OV3660_PID) {
    // 若画面方向不对：把 1 改成 0 试试；或打开 hmirror
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  Serial.println("[CAM] init OK");
  cameraInited = true;
  return true;
}

// 待机用：彻底关闭摄像头（停止 XCLK/采集），以减少功耗与干扰。
// 注意：这会导致“唤醒后第一次打开相机”需要重新 init（会有 200~800ms 的恢复时间，取决于分辨率/PSRAM）。
static inline void cameraDeinitNow() {
  if (!cameraInited) return;
  esp_camera_deinit();
  cameraInited = false;

  // 保险：把 XCLK 变回输入，确保不会残留输出
  pinMode(XCLK_GPIO_NUM, INPUT);
  Serial.println("[CAM] deinit");
}

// ================= PCA9685 motor + servo =================
namespace motor {
  struct Motor2Pin { uint8_t in1; uint8_t in2; };
  static inline void hi(uint8_t ch) { ::pca.setPWM(ch, 4096, 0); }
  static inline void lo(uint8_t ch) { ::pca.setPWM(ch, 0, 4096); }

  static const Motor2Pin M_FL = {0, 1};
  static const Motor2Pin M_FR = {2, 3};
  static const Motor2Pin M_RL = {4, 5};
  static const Motor2Pin M_RR = {6, 7};

  // 如某个轮方向反了，将对应项设为 true
  static const bool INV_FL = false;
  static const bool INV_FR = false;
  static const bool INV_RL = false;
  static const bool INV_RR = false;

  static inline void set(const Motor2Pin& m, int dir, bool inv) {
    if (inv) dir = -dir;
    if (dir > 0) { hi(m.in1); lo(m.in2); }
    else if (dir < 0) { lo(m.in1); hi(m.in2); }
    else { lo(m.in1); lo(m.in2); }
  }

  static inline void stopAll() {
    set(M_FL, 0, INV_FL); set(M_FR, 0, INV_FR);
    set(M_RL, 0, INV_RL); set(M_RR, 0, INV_RR);
  }

  static inline void forward() {
    set(M_FL,+1,INV_FL); set(M_FR,+1,INV_FR);
    set(M_RL,+1,INV_RL); set(M_RR,+1,INV_RR);
  }
  static inline void backward() {
    set(M_FL,-1,INV_FL); set(M_FR,-1,INV_FR);
    set(M_RL,-1,INV_RL); set(M_RR,-1,INV_RR);
  }
  static inline void left() {
    set(M_FL,-1,INV_FL); set(M_FR,+1,INV_FR);
    set(M_RL,+1,INV_RL); set(M_RR,-1,INV_RR);
  }
  static inline void right() {
    set(M_FL,+1,INV_FL); set(M_FR,-1,INV_FR);
    set(M_RL,-1,INV_RL); set(M_RR,+1,INV_RR);
  }
  static inline void rotateLeft() {
    set(M_FL,-1,INV_FL); set(M_FR,+1,INV_FR);
    set(M_RL,-1,INV_RL); set(M_RR,+1,INV_RR);
  }
  static inline void rotateRight() {
    set(M_FL,+1,INV_FL); set(M_FR,-1,INV_FR);
    set(M_RL,+1,INV_RL); set(M_RR,-1,INV_RR);
  }
}

// Servo channels
static const uint8_t SERVO_CH_ARM     = 8;
static const uint8_t SERVO_CH_FOREARM = 9;
static const uint8_t SERVO_CH_CLAW    = 10;

static int servoArm = 90;
static int servoForearm = 90;
static int servoClaw = 90;

static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2500;

static inline uint16_t usToPCA(uint16_t us) {
  return (uint16_t)((uint32_t)us * 4096 / 20000);
}
static inline void servoWriteAngle(uint8_t ch, int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  uint16_t us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
  pca.setPWM(ch, 0, usToPCA(us));
}

// ================= OLED =================
// 说明：大多数 128x64 I2C OLED（SSD1306/SSD1315/SH1106 等）都是“单色屏”(1-bit)，
// 只能显示“亮/灭”两种像素，并不存在真正的多颜色。
// 为了让表情更有层次感，这里用：
// 1) 眼珠实心 + 高光“挖空”(setDrawColor(0)) 形成层次
// 2) 腮红用点阵抖动（看起来像更浅的灰）
// 3) 用“粗线”(多次平移绘制)让线条更柔和
// 4) 不画边框（你要求的：不要边框）
// 5) “鄙视”不是表情脸，而是一个清晰的“倒拇指”手势
//
// ⚠ 注意：你的小车工程里 I2C 已固定 SDA=41 / SCL=42，OLED 的初始化在 setup() 中完成。
// 本段只负责 OLED 绘制（表情/文字）。

static uint8_t scanOLEDAddr() {
  for (uint8_t addr = 0x3C; addr <= 0x3D; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return addr;
  }
  return 0x3C;
}

static void oledText(const char* l1, const char* l2 = nullptr) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, l1);
  if (l2) u8g2.drawStr(0, 26, l2);
  u8g2.sendBuffer();
}

static uint8_t  emojiMode = 8;               // 默认 AUTO
static const uint32_t OLED_INTERVAL_MS = 120;
static uint32_t lastOledMs = 0;
static uint32_t animFrame  = 0;

// --- 表情ID ---
#define EMOJI_OFF       0
#define EMOJI_HAPPY     1
#define EMOJI_SAD       2
#define EMOJI_ANGRY     3
#define EMOJI_SURPRISE  4
#define EMOJI_SLEEP     5
#define EMOJI_DISDAIN   6   // 鄙视：竖中指
#define EMOJI_CAT       7
#define EMOJI_AUTO      8

// U8g2 的 drawArc() 角度单位是 0..255 (对应 0..360 度)
static inline uint8_t u8g2_deg_to_u8(int deg) {
  deg %= 360;
  if (deg < 0) deg += 360;
  return (uint8_t)((deg * 256) / 360);
}

// 画“粗线”：通过平移多画几条线实现（1-bit OLED 也能显得更柔和）
static inline void drawThickLine(int x0, int y0, int x1, int y1, int t) {
  if (t <= 1) { u8g2.drawLine(x0, y0, x1, y1); return; }
  int r = t / 2;
  for (int dx = -r; dx <= r; dx++) {
    for (int dy = -r; dy <= r; dy++) {
      if (dx*dx + dy*dy <= r*r) u8g2.drawLine(x0 + dx, y0 + dy, x1 + dx, y1 + dy);
    }
  }
}

// 点阵腮红（模拟浅色层次）
static inline void drawCheekDots(int cx, int cy) {
  for (int y = -3; y <= 3; y++) {
    for (int x = -6; x <= 6; x++) {
      if (((x + y) & 1) == 0) {
        if (x*x + y*y <= 18) u8g2.drawPixel(cx + x, cy + y);
      }
    }
  }
}

// 眼睛：睁眼（带高光挖空）
static inline void drawOpenEye(int cx, int cy, bool angry=false) {
  u8g2.drawCircle(cx, cy, 9, U8G2_DRAW_ALL);          // 眼白
  u8g2.drawDisc(cx + 2, cy + 1, 3, U8G2_DRAW_ALL);    // 眼珠

  // 高光：挖空一点像素（让眼睛更“灵动”）
  u8g2.setDrawColor(0);
  u8g2.drawDisc(cx + 4, cy - 2, 1, U8G2_DRAW_ALL);
  u8g2.setDrawColor(1);

  // 上眼皮/眉毛：生气更锋利
  if (angry) drawThickLine(cx - 12, cy - 10, cx + 10, cy - 4, 2);
  else       u8g2.drawArc(cx, cy - 1, 10, u8g2_deg_to_u8(210), u8g2_deg_to_u8(330));
}

// 眼睛：闭眼 / 眨眼 / 睡觉
static inline void drawClosedEye(int cx, int cy, bool angry=false) {
  drawThickLine(cx - 10, cy, cx + 10, cy, 2);
  if (angry) drawThickLine(cx - 12, cy - 7, cx + 8, cy - 3, 2);
}

// 眼睛：惊讶
static inline void drawSurpriseEye(int cx, int cy) {
  u8g2.drawCircle(cx, cy, 10, U8G2_DRAW_ALL);
  u8g2.drawDisc(cx, cy, 3, U8G2_DRAW_ALL);
  u8g2.setDrawColor(0);
  u8g2.drawDisc(cx + 3, cy - 3, 1, U8G2_DRAW_ALL);
  u8g2.setDrawColor(1);
}

// 嘴巴：开心
static inline void drawMouthHappy() {
  // 微笑（向上）：使用圆的下半段弧线（200°~340°）
  // 选较小半径与较高中心，避免越界/被裁切
  u8g2.drawArc(64, 40, 16, u8g2_deg_to_u8(200), u8g2_deg_to_u8(340));
  // 两个小酒窝
  u8g2.drawDisc(48, 48, 1);
  u8g2.drawDisc(80, 48, 1);
}

// 嘴巴：难过
// 嘴巴：难过（注意：原来中心 y=56 半径=20 会越过 64px 高度，导致下半截被裁） 
static inline void drawMouthSad() {
  // 难过（向下）：使用圆的上半段弧线（20°~160°）
  u8g2.drawArc(64, 52, 16, u8g2_deg_to_u8(20), u8g2_deg_to_u8(160));
}

// 嘴巴：生气（咬牙）
static inline void drawMouthAngry() {
  drawThickLine(46, 46, 82, 46, 2);
  u8g2.drawVLine(56, 44, 6);
  u8g2.drawVLine(64, 44, 6);
  u8g2.drawVLine(72, 44, 6);
}

// 嘴巴：惊讶（空心 + 点阵阴影）
static inline void drawMouthSurprise() {
  // “O” 形张嘴
  u8g2.drawCircle(64, 46, 7, U8G2_DRAW_ALL);
  u8g2.drawDisc(64, 46, 2, U8G2_DRAW_ALL);
}

// 嘴巴：睡觉
static inline void drawMouthSleep() {
  drawThickLine(52, 48, 76, 48, 2);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(8, 56, "Z z z");
}

// 猫：耳朵、鼻子、胡须
static inline void drawCatExtras() {
  // 轻微下移 + 收窄，避免贴边裁切
  u8g2.drawTriangle(22, 10, 34, 2, 46, 10);
  u8g2.drawTriangle(106, 10, 94, 2, 82, 10);
  // 小鼻子
  u8g2.drawTriangle(62, 42, 66, 42, 64, 45);
  // 胡须（左右各 3 条）
  u8g2.drawLine(28, 44, 44, 42);
  u8g2.drawLine(28, 48, 44, 48);
  u8g2.drawLine(28, 52, 44, 54);

  u8g2.drawLine(100, 44, 84, 42);
  u8g2.drawLine(100, 48, 84, 48);
  u8g2.drawLine(100, 52, 84, 54);
}

// 鄙视手势：竖中指（middle finger）
static inline void drawDisdainGesture(int cx = 64, int cy = 34) {
  // 竖中指手势：简化像素风（单色 OLED）
  // - 手掌 + 手腕
  // - 中指竖起，其余手指折叠
  int palmW = 22, palmH = 18;
  int palmX = cx - palmW / 2;
  int palmY = cy - palmH / 2 + 4;

  // 手腕
  u8g2.drawRBox(palmX + 6, palmY + palmH, 10, 14, 2);

  // 手掌
  u8g2.drawRBox(palmX, palmY, palmW, palmH, 3);

  // 折叠的四指（上边缘的小凸起）
  u8g2.drawRBox(palmX + 2,  palmY - 6, 4, 6, 1);
  u8g2.drawRBox(palmX + 6,  palmY - 6, 4, 6, 1);
  u8g2.drawRBox(palmX + 16, palmY - 6, 4, 6, 1);

  // 中指（竖起更高）
  u8g2.drawRBox(palmX + 10, palmY - 18, 4, 18, 1);

  // 大拇指（侧面）
  u8g2.drawRBox(palmX - 4, palmY + 6, 6, 8, 2);

  // 镂空分割线：让结构更像“手”
  u8g2.setDrawColor(0);
  u8g2.drawVLine(palmX + 7,  palmY - 1, 10);
  u8g2.drawVLine(palmX + 15, palmY - 1, 10);
  u8g2.setDrawColor(1);
}

// 主绘制：不画边框，只画主体
static void drawFace(uint8_t mode, uint32_t frame) {
  u8g2.clearBuffer();

  // 鄙视：只画手势，不画脸
  if (mode == EMOJI_DISDAIN) {
    drawDisdainGesture(64, 34);
    u8g2.sendBuffer();
    return;
  }

  const int eyeY   = 20;
  const int leftX  = 40;
  const int rightX = 88;

  // 眨眼更自然：连续 2 帧闭眼
  bool blink = ((frame % 36) == 0 || (frame % 36) == 1);

  // 需要“可爱感”的表情加腮红
  if (mode == EMOJI_HAPPY || mode == EMOJI_SAD || mode == EMOJI_SURPRISE || mode == EMOJI_CAT) {
    drawCheekDots(26, 38);
    drawCheekDots(102, 38);
  }

  switch (mode) {
    case EMOJI_HAPPY:
      if (blink) { drawClosedEye(leftX, eyeY); drawClosedEye(rightX, eyeY); }
      else       { drawOpenEye(leftX, eyeY);   drawOpenEye(rightX, eyeY); }
      drawMouthHappy();
      break;

    case EMOJI_SAD:
      u8g2.drawDisc(leftX - 16, eyeY + 16, 2);  // 泪滴
      u8g2.drawDisc(leftX - 16, eyeY + 20, 1);
      if (blink) { drawClosedEye(leftX, eyeY); drawClosedEye(rightX, eyeY); }
      else       { drawOpenEye(leftX, eyeY);   drawOpenEye(rightX, eyeY); }
      drawMouthSad();
      break;

    case EMOJI_ANGRY:
      if (blink) { drawClosedEye(leftX, eyeY, true); drawClosedEye(rightX, eyeY, true); }
      else       { drawOpenEye(leftX, eyeY, true);   drawOpenEye(rightX, eyeY, true); }
      drawMouthAngry();
      drawThickLine(10, 14, 22, 20, 2);        // 怒气线
      drawThickLine(118, 14, 106, 20, 2);
      break;

    case EMOJI_SURPRISE:
      drawSurpriseEye(leftX, eyeY);
      drawSurpriseEye(rightX, eyeY);
      drawMouthSurprise();
      break;

    case EMOJI_SLEEP:
      drawClosedEye(leftX, eyeY);
      drawClosedEye(rightX, eyeY);
      drawMouthSleep();
      break;

    case EMOJI_CAT:
      if (blink) { drawClosedEye(leftX, eyeY); drawClosedEye(rightX, eyeY); }
      else       { drawOpenEye(leftX, eyeY);   drawOpenEye(rightX, eyeY); }
      drawCatExtras();
      u8g2.drawLine(64, 47, 60, 52);           // 小嘴
      u8g2.drawLine(64, 47, 68, 52);
      break;

    default:
      if (blink) { drawClosedEye(leftX, eyeY); drawClosedEye(rightX, eyeY); }
      else       { drawOpenEye(leftX, eyeY);   drawOpenEye(rightX, eyeY); }
      drawThickLine(54, 48, 74, 48, 2);        // 中性直嘴
      break;
  }

  u8g2.sendBuffer();
}

static inline void oledLoop() {
  uint32_t now = millis();
  if (now - lastOledMs < OLED_INTERVAL_MS) return;
  lastOledMs = now;

  if (standbyMode) return;

  if (emojiMode == EMOJI_OFF) return;

  uint8_t m = emojiMode;

  if (m == EMOJI_AUTO) {
    // 自动轮播（含鄙视手势）
    const uint8_t list[7] = {EMOJI_HAPPY, EMOJI_SAD, EMOJI_ANGRY, EMOJI_SURPRISE, EMOJI_SLEEP, EMOJI_DISDAIN, EMOJI_CAT};
    uint8_t idx = (now / 2800) % 7;
    m = list[idx];
  }

  drawFace(m, animFrame++);
}

// ================= I2S Audio (MAX98357A + INMP441) =================
// 你之前的建议中：喇叭和麦克风共用 BCLK/LRC，再分别接 DOUT/DIN
#define I2S_BCLK 47
#define I2S_LRC  21
#define I2S_DOUT 45   // -> MAX98357A DIN
#define I2S_DIN  14   // <- INMP441 SD (DOUT)

// INMP441 L/R -> GND(左声道)；若你接到 3V3 则改成 1
#define MIC_USE_RIGHT_CHANNEL 0

#define I2S_PORT I2S_NUM_1

static bool i2sInited = false;
static bool audioEnabled = true;

static bool micReportEnabled = false;
static uint32_t lastMicReportMs = 0;
static const uint32_t MIC_REPORT_INTERVAL_MS = 120;

// ===== 音频流（云端转发：浏览器<->服务器<->小车）=====
// - 车端：连接 /ws_audio?device=xxx
// - 网页：连接 /audio?device=xxx（服务器转发）
// - 数据：PCM16 mono @16kHz，20ms 一帧（320 samples = 640 bytes）
static bool micStreamEnabled = false;  // 发送麦克风 PCM 到服务器
static bool spkEnabled = true;         // 接收并播放来自服务器的 PCM
static bool wsAudioConnected = false;  // 音频 WS 连接状态

// ================= Standby (软待机：不卸载 Camera/I2S，避免并发导致重启) =================
// 目标：网页点“待机”后，车端不推视频、不收/发音频、不亮 OLED、氛围灯熄灭；
// 仍保持与服务器连接，便于“唤醒”。
// 说明：为了稳定性，这里不调用 esp_camera_deinit() / i2s_driver_uninstall()，
// 只做“功能关闭 + 外设省电”。这样不会像“硬卸载”那样在并发推流/播放时触发重启。

// standbyMode declared near the top (forward state vars)

static LedMode  standbyPrevLedMode = LED_OFF;
static uint8_t  standbyPrevLedR = 0, standbyPrevLedG = 0, standbyPrevLedB = 0;
static uint16_t standbyPrevBreathPeriodMs = 1500;
static uint16_t standbyPrevFlashOnMs = 120;

static bool standbyPrevCameraEnabled = false;
static bool standbyPrevMicStreamEnabled = false;
static bool standbyPrevSpkEnabled = true;
static bool standbyPrevAudioEnabled = true;
static bool standbyPrevMicReportEnabled = false;
static uint8_t standbyPrevEmojiMode = EMOJI_AUTO;

static inline void standbyApplyOutputsOff() {
  // 电机停
  motor::stopAll();

  // 灯关（记住原状态，便于唤醒恢复）
  standbyPrevLedMode = ledMode;
  standbyPrevLedR = ledR; standbyPrevLedG = ledG; standbyPrevLedB = ledB;
  standbyPrevBreathPeriodMs = ledBreathPeriodMs;
  standbyPrevFlashOnMs = ledFlashOnMs;
  ledOff();

  // 视频/音频关（只关功能，不卸载驱动）
  standbyPrevCameraEnabled = cameraEnabled;
  standbyPrevMicStreamEnabled = micStreamEnabled;
  standbyPrevSpkEnabled = spkEnabled;
  standbyPrevAudioEnabled = audioEnabled;
  standbyPrevMicReportEnabled = micReportEnabled;

  cameraEnabled = false;
  // 待机：让摄像头彻底停掉（不仅仅是不推流）
  // 这样网页即使缓存了最后一帧，也不会再采集新帧，功耗也更低。
  cameraDeinitPending = true;
  micStreamEnabled = false;
  spkEnabled = false;
  audioEnabled = false;
  micReportEnabled = false;

  if (qMic) xQueueReset(qMic);
  if (qSpk) xQueueReset(qSpk);

  // OLED 关
  standbyPrevEmojiMode = emojiMode;
  emojiMode = EMOJI_OFF;
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  u8g2.setPowerSave(1);
}

static inline void standbyRestoreOutputs() {
  // OLED 恢复
  u8g2.setPowerSave(0);
  emojiMode = standbyPrevEmojiMode;

  // 视频/音频恢复到待机前状态（网页也可以再手动开关）
  cameraEnabled = standbyPrevCameraEnabled;
  // 唤醒时清掉待机 deinit 标记（实际 init 由 cameraService() 按需触发）
  cameraDeinitPending = false;
  micStreamEnabled = standbyPrevMicStreamEnabled;
  spkEnabled = standbyPrevSpkEnabled;
  audioEnabled = standbyPrevAudioEnabled;
  micReportEnabled = standbyPrevMicReportEnabled;

  // 灯恢复
  if (standbyPrevLedMode == LED_STATIC) {
    ledStatic(standbyPrevLedR, standbyPrevLedG, standbyPrevLedB);
  } else if (standbyPrevLedMode == LED_BREATH) {
    ledBreath(standbyPrevLedR, standbyPrevLedG, standbyPrevLedB, standbyPrevBreathPeriodMs);
  } else if (standbyPrevLedMode == LED_FLASH) {
    ledMode = LED_FLASH;
    ledR = standbyPrevLedR; ledG = standbyPrevLedG; ledB = standbyPrevLedB;
    ledFlashOnMs = standbyPrevFlashOnMs;
    ledStartMs = millis();
  } else {
    ledOff();
  }
}

static inline void setStandby(bool on) {
  if (on) {
    if (standbyMode) return;
    standbyMode = true;
    standbyApplyOutputsOff();
    Serial.println("[STANDBY] ON");
  } else {
    if (!standbyMode) return;
    standbyMode = false;
    standbyRestoreOutputs();
    Serial.println("[STANDBY] OFF");
  }
}

#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_FRAME_MS      20
#define AUDIO_SAMPLES_PER_FRAME (AUDIO_SAMPLE_RATE * AUDIO_FRAME_MS / 1000) // 320
#define AUDIO_BYTES_PER_FRAME   (AUDIO_SAMPLES_PER_FRAME * 2)              // PCM16 mono

typedef struct {
  uint16_t len;
  uint8_t  data[AUDIO_BYTES_PER_FRAME];
} AudioFrame;

// qMic/qSpk are declared near the top (forward state vars)


static void i2sInitOnce() {
  if (i2sInited) return;

  // 先设为上拉输入，避免外设在启动阶段干扰
  pinMode(I2S_BCLK, INPUT_PULLUP);
  pinMode(I2S_LRC,  INPUT_PULLUP);
  pinMode(I2S_DOUT, INPUT_PULLUP);
  pinMode(I2S_DIN,  INPUT_PULLUP);
  delay(5);

  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  cfg.sample_rate = 16000;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S_MSB;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pin = {};
  pin.bck_io_num = I2S_BCLK;
  pin.ws_io_num = I2S_LRC;
  pin.data_out_num = I2S_DOUT;
  pin.data_in_num = I2S_DIN;

  Serial.println("[AUDIO] I2S init (TX+RX, 32bit)");
  i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  i2s_set_pin(I2S_PORT, &pin);
  i2s_zero_dma_buffer(I2S_PORT);

  i2sInited = true;
}

static inline void i2sWriteStereo32(const int32_t* lr, size_t frames) {
  size_t bytes = 0;
  i2s_write(I2S_PORT, (const void*)lr, frames * 2 * sizeof(int32_t), &bytes, portMAX_DELAY);
}

// 把 PCM16(单声道) 的一帧塞进扬声器队列，由 spkTask 统一写 I2S。
// 目的：避免「提示音/喇叭」与「对讲播放」同时写 I2S 导致的卡顿/杂音。
static inline bool spkEnqueuePcm16Frame(const int16_t* pcm, int samples, bool padToFullFrame=true) {
  if (!qSpk) return false;
  if (samples <= 0) return true;

  AudioFrame f;
  int n = samples;
  if (n > (int)AUDIO_SAMPLES_PER_FRAME) n = AUDIO_SAMPLES_PER_FRAME;

  // 默认补齐到 20ms，保证 I2S 连续输出更稳。
  if (padToFullFrame) {
    // 先清零，再拷贝有效采样
    memset(f.data, 0, AUDIO_BYTES_PER_FRAME);
    memcpy(f.data, (const void*)pcm, n * 2);
    f.len = AUDIO_BYTES_PER_FRAME;
  } else {
    memcpy(f.data, (const void*)pcm, n * 2);
    f.len = (uint16_t)(n * 2);
  }

  return (xQueueSend(qSpk, &f, 0) == pdTRUE);
}

static void playBeep(int freq=880, int ms=120, int vol=8000) {
  if (!audioEnabled || !spkEnabled) return;
  i2sInitOnce();

  // 关键修复：不要在这里直接 i2s_write。
  // 因为 spkTask 会持续写静音/语音帧，若同时写 I2S 会导致「喇叭卡顿」。
  if (!qSpk) {
    // 兜底：如果队列还没创建，才走直写（此时通常还没启动 spkTask）。
    const int sr = AUDIO_SAMPLE_RATE;
    int total = sr * ms / 1000;
    static int32_t lr[256*2];
    float phase = 0.0f;
    float step = 2.0f * 3.1415926f * (float)freq / (float)sr;
    while (total > 0) {
      int n = (total > 256) ? 256 : total;
      for (int i=0;i<n;i++) {
        int16_t s16 = (int16_t)(vol * sinf(phase));
        phase += step;
        if (phase > 2.0f*3.1415926f) phase -= 2.0f*3.1415926f;
        int32_t s32 = ((int32_t)s16) << 16;
        lr[2*i]   = s32;
        lr[2*i+1] = s32;
      }
      i2sWriteStereo32(lr, n);
      total -= n;
    }
    return;
  }

  // 正常路径：生成 PCM16 -> 送入 qSpk
  const int sr = AUDIO_SAMPLE_RATE;
  const int totalSamples = sr * ms / 1000;
  int produced = 0;
  float phase = 0.0f;
  const float step = 2.0f * 3.1415926f * (float)freq / (float)sr;
  int16_t frame[AUDIO_SAMPLES_PER_FRAME];

  while (produced < totalSamples) {
    int n = totalSamples - produced;
    if (n > (int)AUDIO_SAMPLES_PER_FRAME) n = AUDIO_SAMPLES_PER_FRAME;
    for (int i=0;i<n;i++) {
      frame[i] = (int16_t)(vol * sinf(phase));
      phase += step;
      if (phase > 2.0f*3.1415926f) phase -= 2.0f*3.1415926f;
    }
    // padToFullFrame=true：尾帧自动补零到 20ms
    if (!spkEnqueuePcm16Frame(frame, n, true)) break;
    produced += n;
  }
}

static void playMeow() {
  if (!audioEnabled || !spkEnabled) return;
  i2sInitOnce();

  for (int k=0;k<3;k++) {
    int f1 = (k==0)? 420 : (k==1? 520: 460);
    int f2 = (k==0)? 780 : (k==1? 680: 360);
    int ms = (k==1)? 180 : 140;

    const int sr = AUDIO_SAMPLE_RATE;
    const int totalSamples = sr * ms / 1000;
    int produced = 0;
    float phase = 0.0f;
    int16_t frame[AUDIO_SAMPLES_PER_FRAME];

    while (produced < totalSamples) {
      int n = totalSamples - produced;
      if (n > (int)AUDIO_SAMPLES_PER_FRAME) n = AUDIO_SAMPLES_PER_FRAME;
      for (int i=0;i<n;i++) {
        const float a = (float)(produced + i) / (float)totalSamples;
        float f = (1.0f-a)*f1 + a*f2;
        if (k==1) f += 20.0f * sinf(2.0f*3.1415926f * (float)(produced + i) / (float)sr * 9.0f);
        const float step = 2.0f * 3.1415926f * f / (float)sr;
        int16_t s16 = (int16_t)(9000 * sinf(phase));
        phase += step;
        if (phase > 2.0f*3.1415926f) phase -= 2.0f*3.1415926f;

        float env = 1.0f;
        if (a < 0.08f) env = a/0.08f;
        if (a > 0.92f) env = (1.0f-a)/0.08f;
        frame[i] = (int16_t)((float)s16 * env);
      }

      if (!spkEnqueuePcm16Frame(frame, n, true)) break;
      produced += n;
    }
  }
}

static float readMicRms() {
  i2sInitOnce();
  // 读 256 帧（每帧 2ch * 32bit）
  static int32_t buf[256*2];
  size_t bytesRead = 0;
  esp_err_t err = i2s_read(I2S_PORT, (void*)buf, sizeof(buf), &bytesRead, 20 / portTICK_PERIOD_MS);
  if (err != ESP_OK || bytesRead == 0) return 0.0f;

  int frames = bytesRead / (2 * (int)sizeof(int32_t));
  double acc = 0.0;
  int count = 0;

  for (int i=0;i<frames;i++) {
    int32_t l = buf[2*i];
    int32_t r = buf[2*i+1];
    int32_t x = MIC_USE_RIGHT_CHANNEL ? r : l;

    // INMP441 通常 24bit 有效，这里取高 24bit（保留符号）
    int32_t s24 = x >> 8;
    double v = (double)s24;
    acc += v * v;
    count++;
  }
  if (count == 0) return 0.0f;
  double mean = acc / (double)count;
  return (float)sqrt(mean);
}

// ================= WebSocket helpers =================
static void wsSendJson(const JsonDocument& doc) {
  String s;
  serializeJson(doc, s);
  ws.sendTXT(s);
}

// 视频二进制包： [0xAA 0x55 lenLE16] + jpeg payload
// ===== Video push (兼容原云端协议): "JPG1" + jpeg bytes，必须一次 sendBIN 发完 =====

// 复用视频包缓存，避免每帧 malloc/free 造成抖动与延迟
static uint8_t* gJpgPacket = nullptr;
static size_t   gJpgCap    = 0;

static void wsSendFrame(camera_fb_t* fb) {
  if (!fb) return;
  if (!ws.isConnected()) return;
  if (fb->format != PIXFORMAT_JPEG) return;

  const size_t total = 4 + fb->len;

  // 确保缓存足够大（尽量用 PSRAM）
  if (total > gJpgCap) {
    if (gJpgPacket) { free(gJpgPacket); gJpgPacket = nullptr; gJpgCap = 0; }
    gJpgPacket = (uint8_t*)heap_caps_malloc(total, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!gJpgPacket) gJpgPacket = (uint8_t*)malloc(total);
    if (!gJpgPacket) return;
    gJpgCap = total;
  }

  gJpgPacket[0] = 'J'; gJpgPacket[1] = 'P'; gJpgPacket[2] = 'G'; gJpgPacket[3] = '1';
  memcpy(gJpgPacket + 4, fb->buf, fb->len);

  // 单包发送，云端 /stream 才能识别
  ws.sendBIN(gJpgPacket, total);
}


static bool parseServoAngleCmd(const String& cmd, uint8_t& chOut, int& angleOut) {
  // servo8_120 / servo9_30 / servo10_90
  if (!cmd.startsWith("servo")) return false;
  int underscore = cmd.indexOf('_');
  if (underscore < 0) return false;
  String left = cmd.substring(5, underscore);
  String right = cmd.substring(underscore+1);
  left.trim(); right.trim();
  int ch = left.toInt();
  int ang = right.toInt();
  if (!(ch==8 || ch==9 || ch==10)) return false;
  if (ang < 0) ang = 0;
  if (ang > 180) ang = 180;
  chOut = (uint8_t)ch;
  angleOut = ang;
  return true;
}

static void handleCmd(const String& cmd) {
  lastCmdMs = millis();

  // --- Standby ---
  // 软待机：不卸载 camera/i2s，只做“功能关闭”，避免并发推流/播放时触发重启
  if (cmd == "standby_on")  { setStandby(true);  return; }
  if (cmd == "standby_off") { setStandby(false); return; }
  if (standbyMode) return; // 待机中：忽略其它指令（包含电机/舵机/灯/表情/音视频）

  // 舵机角度直写
  uint8_t ch; int ang;
  if (parseServoAngleCmd(cmd, ch, ang)) {
    if (ch == 8) servoArm = ang;
    if (ch == 9) servoForearm = ang;
    if (ch == 10) servoClaw = ang;
    servoWriteAngle(ch, ang);
    return;
  }

  // 氛围彩灯（WS2812）：led:off / led:r,g,b / led:breath,r,g,b,(period) / led:flash,r,g,b,(onMs)
  if (handleLedCmd(cmd)) return;


  // 底盘
  if      (cmd == "forward")        motor::forward();
  else if (cmd == "backward")       motor::backward();
  else if (cmd == "left")           motor::left();
  else if (cmd == "right")          motor::right();
  else if (cmd == "rotate_left")    motor::rotateLeft();
  else if (cmd == "rotate_right")   motor::rotateRight();
  else if (cmd == "stop")           motor::stopAll();

  // 摄像头/补光灯
  else if (cmd == "camera_on")        cameraEnabled = true;
  else if (cmd == "camera_off")       cameraEnabled = false;
  else if (cmd == "camera_light_on")  flashOn();
  else if (cmd == "camera_light_off") flashOff();

  // 表情
  else if (cmd == "emoji_happy")      emojiMode = EMOJI_HAPPY;
  else if (cmd == "emoji_sad")        emojiMode = EMOJI_SAD;
  else if (cmd == "emoji_angry")      emojiMode = EMOJI_ANGRY;
  else if (cmd == "emoji_surprise")   emojiMode = EMOJI_SURPRISE;
  else if (cmd == "emoji_sleep")      emojiMode = EMOJI_SLEEP;
  else if (cmd == "emoji_disdain")    emojiMode = EMOJI_DISDAIN;
  else if (cmd == "emoji_cat")        emojiMode = EMOJI_CAT;
  else if (cmd == "emoji_auto")       emojiMode = EMOJI_AUTO;
  else if (cmd == "emoji_off")        emojiMode = EMOJI_OFF;

  // 机械臂（按住动/松开停）
  else if (cmd == "arm_up") {
    servoArm += 3; if (servoArm > 180) servoArm = 180;
    servoWriteAngle(SERVO_CH_ARM, servoArm);
  } else if (cmd == "arm_down") {
    servoArm -= 3; if (servoArm < 0) servoArm = 0;
    servoWriteAngle(SERVO_CH_ARM, servoArm);
  } else if (cmd == "arm_stop") {
    servoWriteAngle(SERVO_CH_ARM, servoArm);
  }

  else if (cmd == "forearm_up") {
    servoForearm += 3; if (servoForearm > 180) servoForearm = 180;
    servoWriteAngle(SERVO_CH_FOREARM, servoForearm);
  } else if (cmd == "forearm_down") {
    servoForearm -= 3; if (servoForearm < 0) servoForearm = 0;
    servoWriteAngle(SERVO_CH_FOREARM, servoForearm);
  } else if (cmd == "forearm_stop") {
    servoWriteAngle(SERVO_CH_FOREARM, servoForearm);
  }

  else if (cmd == "claw_open") {
    servoClaw += 3; if (servoClaw > 180) servoClaw = 180;
    servoWriteAngle(SERVO_CH_CLAW, servoClaw);
  } else if (cmd == "claw_close") {
    servoClaw -= 3; if (servoClaw < 0) servoClaw = 0;
    servoWriteAngle(SERVO_CH_CLAW, servoClaw);
  } else if (cmd == "claw_stop") {
    servoWriteAngle(SERVO_CH_CLAW, servoClaw);
  }

  // 喇叭
  else if (cmd == "sound_beep")      playBeep();
  else if (cmd == "sound_meow")      playMeow();
  else if (cmd == "sound_on")        { audioEnabled = true;  spkEnabled = true; }
  else if (cmd == "sound_off")       { audioEnabled = false; spkEnabled = false; }
  else if (cmd == "talk_on")         playMeow(); // 兼容旧命令
  else if (cmd == "talk_off")        {}

  // 麦克风
  else if (cmd == "mic_on")          micReportEnabled = true;
  else if (cmd == "mic_off")         micReportEnabled = false;
  else if (cmd == "mic_stream_on")  micStreamEnabled = true;
  else if (cmd == "mic_stream_off") { micStreamEnabled = false; if (qMic) xQueueReset(qMic); }
}

// WS 收消息：JSON {"cmd":"forward"} 或纯文本 "forward"
static void onWsEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("[WS] Connected");
      // 不主动发 hello，保持与原云端协议一致
      if (emojiMode == EMOJI_OFF) emojiMode = EMOJI_HAPPY;
      break;

    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected -> stop");
      motor::stopAll();
      emojiMode = EMOJI_SLEEP;
      break;

    case WStype_TEXT: {
      String s;
      s.reserve(length + 1);
      for (size_t i = 0; i < length; i++) s += (char)payload[i];
      s.trim();
      if (s.length() == 0) break;

      String cmd = s;
      if (s.startsWith("{")) {
        StaticJsonDocument<256> doc;
        auto err = deserializeJson(doc, s);
        if (!err) {
          if (doc["command"]) cmd = String((const char*)doc["command"]);
          else if (doc["cmd"]) cmd = String((const char*)doc["cmd"]);
        }
      }

      Serial.print("[CMD] ");
      Serial.println(cmd);
      handleCmd(cmd);
      break;
    }

    default:
      break;
  }
}

// ================= WebSocket Audio（浏览器<->服务器<->小车） =================
static void onWsAudioEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsAudioConnected = true;
      Serial.println("[WS-AUDIO] Connected");
      // 轻提示音（不喜欢可注释）
      if (!standbyMode) playBeep(1200, 60, 6000);
      break;

    case WStype_DISCONNECTED:
      wsAudioConnected = false;
      Serial.println("[WS-AUDIO] Disconnected");
      break;

    case WStype_BIN: {
      // 服务器转发下行 PCM（网页说话->喇叭播放）
      if (standbyMode) break;
      if (!spkEnabled) break;
      if (!qSpk) break;
      if (length < 2) break;

      AudioFrame f;
      f.len = (uint16_t)min((size_t)AUDIO_BYTES_PER_FRAME, length);
      memcpy(f.data, payload, f.len);
      // 队列满就丢，保证实时
      xQueueSend(qSpk, &f, 0);
      break;
    }

    default:
      break;
  }
}

// ================= Loops =================
static inline void cameraService() {
  // 待机：执行一次 deinit（停止 XCLK/采集），并保持不再 init
  if (standbyMode) {
    if (cameraDeinitPending) {
      cameraDeinitPending = false;
      cameraDeinitNow();
    }
    return;
  }

  // 非待机：如果网页/逻辑把 cameraEnabled 打开了，但相机驱动未初始化，则自动 init
  // （这样不会因为待机 deinit 后，用户一唤醒/一开相机就崩）
  if (cameraEnabled && !cameraInited) {
    if (!initCamera()) {
      cameraEnabled = false;
      Serial.println("[CAM] auto-init failed, camera disabled");
    }
  }
}

static void pushFrameOnce() {
  if (standbyMode) return;
  if (!ws.isConnected() || !cameraEnabled || !cameraInited) return;
  uint32_t now = millis();
  if (now - lastFrameMs < FRAME_INTERVAL_MS) return;
  lastFrameMs = now;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;
  wsSendFrame(fb);
  esp_camera_fb_return(fb);
}

static void timeoutSafety() {
  uint32_t now = millis();
  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    motor::stopAll();
  }
}

static void micLoop() {
  if (!micReportEnabled) return;
  uint32_t now = millis();
  if (now - lastMicReportMs < MIC_REPORT_INTERVAL_MS) return;
  lastMicReportMs = now;

  float rms = readMicRms();
  StaticJsonDocument<128> doc;
  doc["type"] = "mic";
  doc["rms"] = rms;
  wsSendJson(doc);
}

// ===== 音频任务：采集麦克风（INMP441）-> qMic =====
static void micTask(void* arg) {
  (void)arg;
  i2sInitOnce();

  // 读入 32bit 立体声：L/R 交错
  static int32_t raw[AUDIO_SAMPLES_PER_FRAME * 2];

  for (;;) {
    if (standbyMode || !micStreamEnabled || !wsAudioConnected || !qMic) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    size_t bytesRead = 0;
    esp_err_t e = i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead, portMAX_DELAY);
    if (e != ESP_OK || bytesRead == 0) continue;

    // bytesRead / (2ch * 4bytes) = frames
    int frames = bytesRead / (2 * (int)sizeof(int32_t));
    if (frames <= 0) continue;

    AudioFrame f;
    int samples = min(frames, (int)AUDIO_SAMPLES_PER_FRAME);
    f.len = samples * 2;

    // 选择左/右声道（INMP441 的 L/R 引脚决定输出哪个声道）
    int ch = MIC_USE_RIGHT_CHANNEL ? 1 : 0;
    int16_t* out = (int16_t*)f.data;
    for (int i = 0; i < samples; i++) {
      int32_t s32 = raw[i*2 + ch];
      // INMP441 常见有效位在高 16bit
      out[i] = (int16_t)(s32 >> 16);
    }

    // 队列满就丢（保证实时）
    xQueueSend(qMic, &f, 0);
  }
}

// ===== 音频任务：qSpk -> MAX98357A 播放 =====
//
// 之前版本：spkTask 会一直阻塞等队列数据（portMAX_DELAY）。
// 现实网络里 Web->云->车 的音频帧会“抖动”到达：
// - 有时短时间成批到达（burst）
// - 有时出现几十 ms 的空洞
// 如果 spkTask 在“空洞”里阻塞，I2S DMA 会被耗尽，从而出现「断断续续 / 句子缺字」。
// 这里改为：
// - 以 20ms 为节拍等待一帧
// - 等不到就写一帧静音（保持 I2S 时钟连续）
// - 队列堆积过多时仍会丢老帧（保持实时）
// 这样说话会更“连贯”，且不会显著增加延迟。
static void spkTask(void* arg) {
  (void)arg;
  if (!qSpk) vTaskDelete(nullptr);

  static int32_t lr[AUDIO_SAMPLES_PER_FRAME * 2];
  // 静音帧（避免每次 underflow 都 memset 大块数组）
  static bool silenceInit = false;
  static int32_t lr_silence[AUDIO_SAMPLES_PER_FRAME * 2];

  if (!silenceInit) {
    memset(lr_silence, 0, sizeof(lr_silence));
    silenceInit = true;
  }

  AudioFrame f;
  for (;;) {
    if (standbyMode || !spkEnabled) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    // 20ms 等待一帧；如果超时则输出 20ms 静音（防止 I2S 断流）
    bool got = (xQueueReceive(qSpk, &f, pdMS_TO_TICKS(AUDIO_FRAME_MS)) == pdTRUE);

    if (!got) {
      i2sWriteStereo32(lr_silence, AUDIO_SAMPLES_PER_FRAME);
      continue;
    }

    if (f.len == 0) continue;

    // PCM16 单声道（little-endian）-> I2S 32bit 双声道
    // 注意：f.data 是 uint8_t[]，必须按 int16_t* 解析，否则会出现“吱吱响/听不清”。
    const int samples = min((int)(f.len / 2), (int)AUDIO_SAMPLES_PER_FRAME);
    const int16_t* pcm = (const int16_t*)f.data;
    for (int i = 0; i < samples; i++) {
      const int16_t s = pcm[i];
      const int32_t s32 = ((int32_t)s) << 16; // MSB 对齐
      lr[i * 2 + 0] = s32; // L
      lr[i * 2 + 1] = s32; // R
    }
    i2sWriteStereo32(lr, samples);
  }
}

// 主循环里把 qMic 的帧发到服务器（避免多线程直接调用 wsAudio）
static inline void audioTxLoop() {
  if (standbyMode) return;
  if (!wsAudioConnected || !micStreamEnabled || !qMic) return;
  // 重要：不要一次性把队列全发完。
  // 在弱网/小带宽下，sendBIN 可能阻塞较久，导致：
  //  1) ws.loop() 得不到及时调用（控制命令延迟/丢失）
  //  2) 摄像头推流帧率骤降甚至“卡死”
  // 所以每次 loop 只发送少量音频帧，并设置时间预算。
  const uint32_t t0 = micros();
  const uint8_t  MAX_FRAMES_PER_TICK = 2;     // 每次 loop 最多发 2 帧（40ms音频）
  const uint32_t MAX_US_PER_TICK     = 3000;  // 每次 loop 最多花 3ms 发音频

  AudioFrame f;
  uint8_t sent = 0;
  while (sent < MAX_FRAMES_PER_TICK && (micros() - t0) < MAX_US_PER_TICK) {
    if (xQueueReceive(qMic, &f, 0) != pdTRUE) break;
    if (f.len > 0) wsAudio.sendBIN(f.data, f.len);
    sent++;
  }
}

// ================= setup/loop =================
// ================= WiFi 配网（30s 失败进入 AP 配置） =================
static inline String macSuffix2() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[8];
  sprintf(buf, "%02X%02X", (uint8_t)(mac>>8), (uint8_t)mac);
  return String(buf);
}

static void cfgLoad() {
  prefs.begin("carcfg", true);
  cfg_ssid    = prefs.getString("ssid", "");
  cfg_pass    = prefs.getString("pass", "");
  cfg_ws_host = prefs.getString("wshost", "");
  cfg_ws_port = prefs.getUShort("wsport", WS_PORT);
  cfg_ws_path = prefs.getString("wspath", "");
  prefs.end();

  // 首次烧录兜底：如果 NVS 为空，用写死常量作为默认
  if (cfg_ssid.length() == 0) cfg_ssid = WIFI_SSID;
  if (cfg_pass.length() == 0) cfg_pass = WIFI_PASS;
  if (cfg_ws_host.length() == 0) cfg_ws_host = WS_HOST;
  if (cfg_ws_path.length() == 0) cfg_ws_path = WS_PATH;
}


// 根据控制 WS_PATH 推导音频 WS_PATH（默认把 /ws 替换成 /ws_audio，保留 ?device=...）
static String buildAudioPath(const String& ctrlPath) {
  // 例如：/ws?device=car1 -> /ws_audio?device=car1
  int q = ctrlPath.indexOf('?');
  String base = (q >= 0) ? ctrlPath.substring(0, q) : ctrlPath;
  String query = (q >= 0) ? ctrlPath.substring(q) : "";
  if (base == "/ws") return String("/ws_audio") + query;
  // 其他情况兜底：直接用 /ws_audio + query
  return String("/ws_audio") + query;
}


static void cfgSave(const String& ssid, const String& pass,
                    const String& host, uint16_t port, const String& path) {
  prefs.begin("carcfg", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putString("wshost", host);
  prefs.putUShort("wsport", port);
  prefs.putString("wspath", path);
  prefs.end();
}

// 尝试连接 WiFi（timeoutMs 内未连上返回 false）
static bool wifiConnectTry(uint32_t timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // 降低抖动
  WiFi.disconnect(true, true);
  delay(80);

  oledText("WiFi...", cfg_ssid.c_str());
  Serial.printf("[WiFi] Connecting to %s\n", cfg_ssid.c_str());

  WiFi.begin(cfg_ssid.c_str(), cfg_pass.c_str());

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    oledText("WiFi OK", ip.c_str());
    Serial.printf("[WiFi] OK IP=%s\n", ip.c_str());

    // 可选：mDNS（便于局域网调试）
    MDNS.begin(("car1-" + macSuffix2()).c_str());
    return true;
  }

  oledText("WiFi FAIL");
  Serial.println("[WiFi] FAIL");
  return false;
}

static void cfgHandleRoot() {
  cfgTouch();
  String html =
    "<!doctype html><html><head><meta charset='utf-8'/>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<title>Car Setup</title>"
    "<style>body{font-family:system-ui;margin:18px;}input{width:100%;padding:10px;margin:6px 0;}"
    "button{padding:10px 14px;margin-top:10px;width:100%;}</style>"
    "</head><body>"
    "<h2>ESP32 Car Setup</h2>"
    "<p>连接失败会自动进入这里。保存后会重启生效。</p>"
    "<form action='/save' method='POST'>"
    "<label>SSID</label><input name='ssid' id='ssid' placeholder='WiFi name' value='" + cfg_ssid + "'/>"
    "<label>Password</label><input name='pass' type='password' placeholder='WiFi password' value='" + cfg_pass + "'/>"
    "<hr/>"
    "<label>WS Host</label><input name='wshost' value='" + cfg_ws_host + "'/>"
    "<label>WS Port</label><input name='wsport' value='" + String(cfg_ws_port) + "'/>"
    "<label>WS Path</label><input name='wspath' value='" + cfg_ws_path + "'/>"
    "<button type='submit'>Save & Reboot</button>"
    "</form>"
    "<button onclick='scan()'>Scan WiFi</button>"
    "<ul id='list'></ul>"
    "<script>"
    "function scan(){fetch('/scan').then(r=>r.json()).then(a=>{"
    "let ul=document.getElementById('list');ul.innerHTML='';"
    "a.forEach(x=>{let li=document.createElement('li');"
    "li.innerHTML=`<a href='#' onclick=\\\"document.getElementById('ssid').value='${x.ssid}'\\\">${x.ssid}</a> (${x.rssi})`;"
    "ul.appendChild(li);});});}"
    "</script>"
    "</body></html>";
  cfgServer.send(200, "text/html", html);
}

static void cfgHandleScan() {
  cfgTouch();
  int n = WiFi.scanNetworks(false, true);
  String json = "[";
  for (int i=0;i<n;i++) {
    if (i) json += ",";
    String ssid = WiFi.SSID(i);
    ssid.replace("\"", "");
    json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
  }
  json += "]";
  cfgServer.send(200, "application/json", json);
}

static void cfgHandleSave() {
  cfgTouch();
  String ssid = cfgServer.arg("ssid"); ssid.trim();
  String pass = cfgServer.arg("pass");
  String host = cfgServer.arg("wshost"); host.trim();
  String portStr = cfgServer.arg("wsport"); portStr.trim();
  String path = cfgServer.arg("wspath"); path.trim();

  if (ssid.length() == 0) { cfgServer.send(400, "text/plain", "SSID required"); return; }
  uint16_t port = (uint16_t)portStr.toInt();
  if (host.length() == 0) host = cfg_ws_host;
  if (port == 0) port = cfg_ws_port;
  if (path.length() == 0) path = cfg_ws_path;

  cfgSave(ssid, pass, host, port, path);
  cfgServer.send(200, "text/plain", "Saved. Rebooting...");
  delay(800);
  ESP.restart();
}

// 进入配网模式：开启 AP + DNS 劫持 + 配网页
static void startConfigPortal() {
  if (cfgMode) return;
  cfgMode = true;

  cfgPortalStartMs = millis();
  cfgLastActivityMs = cfgPortalStartMs;
  cfgLastStaNum = 0;

  motor::stopAll();
  cameraEnabled = false;   // 配网时不推流，省资源
  ledOff();                // 默认不亮

  String apName = "CAR-SETUP-" + macSuffix2();

  // 为了支持扫描 WiFi：用 AP+STA 模式
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.softAP(apName.c_str());
  IPAddress apIP = WiFi.softAPIP();

  oledText("AP Config:", apName.c_str());
  delay(1100);
  String ipStr = apIP.toString();
  oledText("Open:", ipStr.c_str());
Serial.printf("[CFG] AP=%s IP=%s\n", apName.c_str(), apIP.toString().c_str());

  dnsServer.start(53, "*", apIP);

  cfgServer.on("/", HTTP_GET, cfgHandleRoot);
  cfgServer.on("/scan", HTTP_GET, cfgHandleScan);
  cfgServer.on("/save", HTTP_POST, cfgHandleSave);

  // 让手机/电脑的“自动弹窗”更容易成功（不同系统会请求不同 URL）
  cfgServer.on("/generate_204", HTTP_GET, [](){ cfgServer.send(204, "text/plain", ""); });              // Android
  cfgServer.on("/hotspot-detect.html", HTTP_GET, [](){ cfgServer.send(200, "text/html", "OK"); });     // iOS
  cfgServer.on("/library/test/success.html", HTTP_GET, [](){ cfgServer.send(200, "text/html", "OK"); });// iOS
  cfgServer.on("/ncsi.txt", HTTP_GET, [](){ cfgServer.send(200, "text/plain", "Microsoft NCSI"); });  // Windows

  // Captive portal：任何未知路径都重定向到主页
  cfgServer.onNotFound([](){
    cfgTouch();
    cfgServer.sendHeader("Location", "/", true);
    cfgServer.send(302, "text/plain", "");
  });

  cfgServer.begin();
}

static void configPortalLoop() {
  dnsServer.processNextRequest();
  cfgServer.handleClient();
}


static void stopConfigPortal() {
  if (!cfgMode) return;
  // 停止 Captive Portal
  cfgServer.stop();
  dnsServer.stop();
  // 关闭 AP
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  cfgMode = false;
}

static inline void cfgTouch() {
  cfgLastActivityMs = millis();
}

// 配网模式下：1min 没有任何交互（且没人连着 AP）-> 退出配网，重试 WiFi
static void configPortalIdleCheckAndRetryWiFi() {
  if (!cfgMode) return;

  const uint32_t now = millis();
  const uint8_t staNum = WiFi.softAPgetStationNum();
  if (staNum != cfgLastStaNum) {
    cfgLastStaNum = staNum;
    cfgTouch();  // 有人连上/断开热点，也算一次“活跃”
  }

  // 只有在“没人连着热点”的情况下，才触发超时重试，避免用户正在配网被打断
  if (staNum == 0 && (now - cfgLastActivityMs) > CFG_IDLE_TO_RETRY_MS) {
    Serial.println("[CFG] idle timeout -> retry WiFi");
    stopConfigPortal();

    // 重试连网（30s）
    cfgLoad();
    bool ok = wifiConnectTry(30000);
    if (!ok) {
      startConfigPortal();
      return;
    }

    // 连上网后启动 WebSocket（与 setup() 同逻辑）
    ws.begin(cfg_ws_host.c_str(), cfg_ws_port, cfg_ws_path.c_str());
    ws.onEvent(onWsEvent);
    ws.setReconnectInterval(1500);
    ws.enableHeartbeat(15000, 3000, 2);

    cfg_ws_audio_path = buildAudioPath(cfg_ws_path);
    wsAudio.begin(cfg_ws_host.c_str(), cfg_ws_port, cfg_ws_audio_path.c_str());
    Serial.printf("[WS-AUDIO] path=%s\n", cfg_ws_audio_path.c_str());
wsAudio.onEvent(onWsAudioEvent);
wsAudio.setReconnectInterval(1500);
wsAudio.enableHeartbeat(15000, 3000, 2);

    wifiLostSince = 0;
    lastCmdMs = millis();
  }
}
void setup() {
  Serial.begin(115200);

// ===== 音频队列 + 任务（对讲/监听）=====
qMic = xQueueCreate(12, sizeof(AudioFrame));   // 上行：12帧缓冲 (~240ms)
qSpk = xQueueCreate(24, sizeof(AudioFrame));   // 下行：24帧缓冲 (~480ms)
xTaskCreatePinnedToCore(micTask, "micTask", 4096, nullptr, 1, nullptr, 0);
xTaskCreatePinnedToCore(spkTask, "spkTask", 4096, nullptr, 1, nullptr, 0);

  delay(200);
  Serial.println("\n[BOOT] ESP32-S3-CAM car (OV3660)");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // PCA9685
  pca.begin();
  pca.setPWMFreq(50);
  delay(10);
  motor::stopAll();

  servoWriteAngle(SERVO_CH_ARM, servoArm);
  servoWriteAngle(SERVO_CH_FOREARM, servoForearm);
  servoWriteAngle(SERVO_CH_CLAW, servoClaw);

  // OLED
  OLED_ADDR_7BIT = scanOLEDAddr();
  u8g2.setI2CAddress(OLED_ADDR_7BIT << 1);
  u8g2.begin();
  oledText("ESP32-S3-CAM", "OLED OK");

  // WS2812 flash
  flashInit();
  flashOff();

  // Camera
  if (!initCamera()) {
    oledText("Camera FAIL");
  }


// WiFi（30s 内连不上 -> 进入配网模式）
cfgLoad();
bool ok = wifiConnectTry(30000);
if (!ok) {
  startConfigPortal();
}

// WebSocket：只在 STA 已连网时启动
if (WiFi.status() == WL_CONNECTED) {
  ws.begin(cfg_ws_host.c_str(), cfg_ws_port, cfg_ws_path.c_str());
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(1500);
  ws.enableHeartbeat(15000, 3000, 2);

// 音频对讲 WS（/ws_audio?device=...）
    cfg_ws_audio_path = buildAudioPath(cfg_ws_path);
    wsAudio.begin(cfg_ws_host.c_str(), cfg_ws_port, cfg_ws_audio_path.c_str());
    Serial.printf("[WS-AUDIO] path=%s\n", cfg_ws_audio_path.c_str());
wsAudio.onEvent(onWsAudioEvent);
wsAudio.setReconnectInterval(1500);
wsAudio.enableHeartbeat(15000, 3000, 2);
}

// WebSocket
  lastCmdMs = millis();
}

void loop() {
  // 配网模式：只跑配网页（不跑 WS、不推流）
  if (cfgMode) {
    configPortalLoop();
    configPortalIdleCheckAndRetryWiFi();
    delay(2);
    return;
  }

  // 业务循环
  ws.loop();
  wsAudio.loop();
  audioTxLoop();

  const uint32_t now = millis();
  if (now - lastPingMs > 10000 && ws.isConnected()) {
    lastPingMs = now;
    ws.sendTXT("{\"type\":\"ping\"}");
  }

  // OLED 表情 & 安全
  oledLoop();
  timeoutSafety();

  // 相机驱动维护（待机时 deinit，唤醒后按需 init）
  cameraService();

  // 视频推流（WS 连上且 cameraEnabled 才会推）
  pushFrameOnce();

  // 麦克风（如果开启了上报）
  micLoop();

  // 氛围彩灯（呼吸/闪烁状态机）
  ledLoop();

  // 断网 30s -> 进入配网模式
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiLostSince == 0) wifiLostSince = now;
    if (now - wifiLostSince > 30000) startConfigPortal();
  } else {
    wifiLostSince = 0;
  }

  delay(1);
}
