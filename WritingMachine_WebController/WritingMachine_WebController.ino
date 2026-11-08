// ============================================================
//  WritingMachine_WebController.ino
//  ESP32-WROOM-32 + 4×A4988 + 2Y/1X/1Z 写字机 Wi-Fi 控制台
//  - 参数 / 轴启用状态 掉电保存
//  - 上电位置默认清零为 (0, 0, 0)
//  - 表格首行点击可切换轴启用，末行为 往返/单次/停止/清零/保存
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <FastAccelStepper.h>
#include <Preferences.h>

// ================= 配置热点 =================
const char* AP_SSID = "WritingMachine";
const char* AP_PASS = "12345678";

WebServer server(80);
DNSServer dnsServer;
Preferences prefs;

// ================= WiFi 状态 =================
bool   staConnected = false;
bool   apActive     = false;
String staIP  = "";
String apIP   = "";
String savedSSID = "";
String savedPass = "";

bool     closeAPPending = false;
uint32_t closeAPTime = 0;

// ================= 引脚定义 =================
#define X_STEP_PIN 13
#define X_DIR_PIN  15
#define Y_STEP_L   26
#define Y_DIR_L    25
#define Y_STEP_R   14
#define Y_DIR_R    27
#define Z_STEP_PIN 33
#define Z_DIR_PIN  32
#define EN_PIN     21

// ================= 轴位掩码 =================
#define AXIS_X 0x01
#define AXIS_Y 0x02
#define AXIS_Z 0x04

// ================= 运行模式 =================
#define RUN_MODE_LOOP 0
#define RUN_MODE_ONCE 1

// ================= 电机对象 =================
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *xStepper = NULL;
FastAccelStepper *yLeft    = NULL;
FastAccelStepper *yRight   = NULL;
FastAccelStepper *zStepper = NULL;

// ================= 可调参数 =================
struct AxisParam {
  uint32_t speed = 1000;
  uint32_t acceleration = 2000;
  int32_t  travel = 1000;
  uint32_t pauseMs = 500;
  uint32_t maxTravelMm = 0;  // 最大行程 (mm)，0 = 不限制
};
AxisParam xParam, yParam, zParam;

// ================= 运行状态 =================
bool     systemRunning = false;
uint8_t  axisMask = AXIS_X | AXIS_Y | AXIS_Z;
uint8_t  runMode  = RUN_MODE_LOOP;
bool     apiRunning    = false;  // /moveto 接口触发的单次往返

// ================= 往复运动状态（网页端与 API 共用）=================
// stage: 0 去程  1 到位停顿  2 回程  3 原点停顿  4 完成/未参与
enum AxisAction { AXIS_NONE = 0, AXIS_GO_HOME, AXIS_GO_TARGET };

struct AxisCycle {
  bool     enabled  = false;
  bool     repeat   = false;  // 往返模式：原点停顿结束后继续下一轮
  int32_t  target   = 0;
  uint32_t dwellMs  = 0;      // 到位停顿
  uint32_t homeMs   = 0;      // 回到原点后的停顿
  uint32_t resumeAt = 0;
  uint8_t  stage    = 4;
};

AxisCycle xCycle, yCycle, zCycle;
bool      cycleStarted = false;  // 本轮去程是否已下发

// ================= 点击运动状态（/click：移动到点 + Z 轴单击）=================
// phase: 0 下发 X/Y 去程  1 等 X/Y 到位  2 Z 下压  3 Z 抬起
struct ClickState {
  bool     hasX = false, hasY = false;
  int32_t  targetX = 0, targetY = 0;
  int32_t  zTrip = 500;   // Z 轴点击行程（步）
  int32_t  zBase = 0;     // 点击前 Z 的当前位置
  uint8_t  phase = 0;
};
ClickState click;
bool       clickRunning = false;

// ================= 步/mm 换算 =================
// 10000 步 = 10 cm，即 100 步/mm；不同丝杆/细分请修改此值
#define STEPS_PER_MM 100

// 目标超出最大行程（mm，0=不限制）时返回 true
bool targetExceedsLimit(int32_t target, uint32_t maxTravelMm) {
  if (maxTravelMm == 0) return false;
  int32_t maxSteps = (int32_t)maxTravelMm * STEPS_PER_MM;
  return target < 0 || target > maxSteps;
}

// 默认最大行程 (mm)，可在网页"运动参数"表中修改并保存
void initParamDefaults() {
  xParam.maxTravelMm = 210;
  yParam.maxTravelMm = 300;
  zParam.maxTravelMm = 10;
}

// ================= EN 控制 =================
void setMotorEnable(bool en) {
  digitalWrite(EN_PIN, en ? LOW : HIGH);
}

void stopAllSteppers() {
  if (xStepper) xStepper->stopMove();
  if (yLeft)    yLeft->stopMove();
  if (yRight)   yRight->stopMove();
  if (zStepper) zStepper->stopMove();
}

// 停掉正在进行的往复（网页端或 API）并清空各轴状态
void stopCycles() {
  systemRunning = false;
  apiRunning    = false;
  cycleStarted  = false;
  xCycle = AxisCycle();
  yCycle = AxisCycle();
  zCycle = AxisCycle();
  clickRunning = false;
  click = ClickState();
}

// 装配一个轴的往复任务
void armCycle(AxisCycle& axis, bool enabled, int32_t target,
              uint32_t dwellMs, uint32_t homeMs, bool repeat) {
  axis.enabled  = enabled;
  axis.target   = target;
  axis.dwellMs  = dwellMs;
  axis.homeMs   = homeMs;
  axis.repeat   = repeat;
  axis.resumeAt = 0;
  axis.stage    = enabled ? 0 : 4;
}

bool machineBusy() {
  return systemRunning || apiRunning || clickRunning;
}

// 任务进行中时拒绝新的运动指令：上一条任务成功完成后才可接受下一步
void sendBusy() {
  server.send(409, "application/json",
              "{\"ok\":false,\"msg\":\"busy, wait current task to finish\"}");
}

// /status 里各轴的阶段，只列出本次参与的轴
String stageJson() {
  String json = "";
  if (xCycle.enabled) json += "\"x\":" + String(xCycle.stage);
  if (yCycle.enabled) { if (json.length() > 0) json += ","; json += "\"y\":" + String(yCycle.stage); }
  if (zCycle.enabled) { if (json.length() > 0) json += ","; json += "\"z\":" + String(zCycle.stage); }
  return json;
}

// ================= 应用参数 =================
void applyParams() {
  xStepper->setSpeedInHz(xParam.speed);
  xStepper->setAcceleration(xParam.acceleration);
  yLeft->setSpeedInHz(yParam.speed);
  yLeft->setAcceleration(yParam.acceleration);
  yRight->setSpeedInHz(yParam.speed);
  yRight->setAcceleration(yParam.acceleration);
  zStepper->setSpeedInHz(zParam.speed);
  zStepper->setAcceleration(zParam.acceleration);
}

// ================= NVS：运动参数 =================
void loadParams() {
  prefs.begin("writing", true);
  xParam.speed        = prefs.getUInt("xSpeed",  xParam.speed);
  xParam.acceleration = prefs.getUInt("xAccel",  xParam.acceleration);
  xParam.travel       = prefs.getInt ("xTravel", xParam.travel);
  xParam.pauseMs      = prefs.getUInt("xPause",  xParam.pauseMs);
  xParam.maxTravelMm  = prefs.getUInt("xMaxMm",  xParam.maxTravelMm);

  yParam.speed        = prefs.getUInt("ySpeed",  yParam.speed);
  yParam.acceleration = prefs.getUInt("yAccel",  yParam.acceleration);
  yParam.travel       = prefs.getInt ("yTravel", yParam.travel);
  yParam.pauseMs      = prefs.getUInt("yPause",  yParam.pauseMs);
  yParam.maxTravelMm  = prefs.getUInt("yMaxMm",  yParam.maxTravelMm);

  zParam.speed        = prefs.getUInt("zSpeed",  zParam.speed);
  zParam.acceleration = prefs.getUInt("zAccel",  zParam.acceleration);
  zParam.travel       = prefs.getInt ("zTravel", zParam.travel);
  zParam.pauseMs      = prefs.getUInt("zPause",  zParam.pauseMs);
  zParam.maxTravelMm  = prefs.getUInt("zMaxMm",  zParam.maxTravelMm);

  axisMask            = prefs.getUInt("axisMask", axisMask);
  runMode             = prefs.getUInt("runMode",  runMode);
  prefs.end();
}

void saveParamsToNVS() {
  prefs.begin("writing", false);
  prefs.putUInt("xSpeed",  xParam.speed);
  prefs.putUInt("xAccel",  xParam.acceleration);
  prefs.putInt ("xTravel", xParam.travel);
  prefs.putUInt("xPause",  xParam.pauseMs);
  prefs.putUInt("xMaxMm",  xParam.maxTravelMm);

  prefs.putUInt("ySpeed",  yParam.speed);
  prefs.putUInt("yAccel",  yParam.acceleration);
  prefs.putInt ("yTravel", yParam.travel);
  prefs.putUInt("yPause",  yParam.pauseMs);
  prefs.putUInt("yMaxMm",  yParam.maxTravelMm);

  prefs.putUInt("zSpeed",  zParam.speed);
  prefs.putUInt("zAccel",  zParam.acceleration);
  prefs.putInt ("zTravel", zParam.travel);
  prefs.putUInt("zPause",  zParam.pauseMs);
  prefs.putUInt("zMaxMm",  zParam.maxTravelMm);

  prefs.putUInt("axisMask", axisMask);
  prefs.putUInt("runMode",  runMode);
  prefs.end();
}

// ================= NVS：WiFi 凭据 =================
void loadWiFiCreds() {
  prefs.begin("wifi", true);
  savedSSID = prefs.getString("ssid", "");
  savedPass = prefs.getString("pass", "");
  prefs.end();
}

void saveWiFiCreds(const String& ssid, const String& pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

// ================= WiFi 工具 =================
String escapeJson(const String& s) {
  String out = s;
  out.replace("\\", "\\\\");
  out.replace("\"", "\\\"");
  return out;
}

void enableAP() {
  if (apActive) return;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  apActive = true;
  apIP = WiFi.softAPIP().toString();
  dnsServer.start(53, "*", WiFi.softAPIP());

  Serial.print("AP 已开启: "); Serial.print(AP_SSID);
  Serial.print("  密码: "); Serial.print(AP_PASS);
  Serial.print("  IP: "); Serial.println(apIP);
}

void disableAP() {
  if (!apActive) return;
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apActive = false;
  Serial.println("AP 已关闭，仅保持 STA 模式");
}

// ================= HTML 页面 =================
String axisToggleHtml(uint8_t bit, const char* axis, const char* label) {
  bool on = (axisMask & bit) != 0;
  String html = "<div class='axis-toggle ";
  if (on) html += "active";
  html += "' data-axis='";
  html += axis;
  html += "' onclick='toggleAxis(this)'>";
  html += label;
  html += "</div>";
  return html;
}

String buildPage() {
  String html = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>写字机控制台</title>
<style>
:root{
  --bg:#f0f2f5;--card:#fff;--primary:#2563eb;--success:#16a34a;
  --danger:#dc2626;--warning:#f59e0b;--text:#1e293b;--muted:#64748b;
  --border:#e2e8f0;
}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
  background:var(--bg);color:var(--text);padding:16px;padding-bottom:60px}
.container{max-width:960px;margin:0 auto}
h1{font-size:22px;margin-bottom:16px;font-weight:700}
h1 span{color:var(--primary)}
.card{background:var(--card);border-radius:14px;padding:18px;margin-bottom:14px;
  box-shadow:0 1px 3px rgba(0,0,0,.06);border:1px solid var(--border)}
.card h3{font-size:15px;font-weight:600;margin-bottom:12px;display:flex;
  align-items:center;gap:8px}
.card h3 .tag{font-size:11px;font-weight:500;color:var(--muted);
  background:var(--bg);padding:2px 8px;border-radius:10px}
.status-row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.status-badge{display:inline-flex;align-items:center;gap:6px;
  padding:6px 14px;border-radius:20px;font-size:13px;font-weight:600}
.status-badge.on{background:#dcfce7;color:#166534}
.status-badge.off{background:#fee2e2;color:#991b1b}
.dot{width:8px;height:8px;border-radius:50%;background:currentColor}
.status-badge.on .dot{animation:pulse 1.5s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.info-line{font-size:12px;color:var(--muted);margin-top:6px;word-break:break-all}
.coord-line{display:flex;gap:18px;margin-top:10px;padding-top:10px;
  border-top:1px dashed var(--border);font-size:13px;color:var(--muted)}
.coord-line b{color:var(--text);font-family:ui-monospace,Menlo,monospace;font-size:14px}
.row{display:flex;align-items:center;gap:8px;margin-bottom:8px}
.row label{font-size:13px;color:var(--muted);min-width:70px}
.row input{flex:1;padding:9px 12px;border:1px solid var(--border);
  border-radius:8px;font-size:14px;background:#fafbfc;transition:.15s}
.row input:focus{outline:none;border-color:var(--primary);background:#fff}
.btn{padding:10px 18px;border:none;border-radius:10px;font-size:14px;
  font-weight:600;cursor:pointer;transition:.15s;
  display:inline-flex;align-items:center;gap:6px}
.btn:active{transform:scale(.97)}
.btn-primary{background:var(--primary);color:#fff}
.btn-primary:hover{background:#1d4ed8}
.btn-success{background:var(--success);color:#fff}
.btn-success:hover{background:#15803d}
.btn-danger{background:var(--danger);color:#fff}
.btn-danger:hover{background:#b91c1c}
.btn-warning{background:var(--warning);color:#fff}
.btn-warning:hover{background:#d97706}
.btn-row{display:flex;gap:8px;flex-wrap:wrap}
.note{font-size:12px;color:var(--muted);margin-top:8px;line-height:1.5}
#wifiList select{width:100%;padding:10px;font-size:14px;border:1px solid var(--border);
  border-radius:8px;background:#fafbfc;margin-top:8px}
.divider{height:1px;background:var(--border);margin:14px 0}

/* 参数表格 */
.param-table{width:100%;border-collapse:collapse;border:1px solid var(--border);
  border-radius:10px;overflow:hidden;table-layout:fixed}
.param-table th{background:#f1f5f9;padding:10px 6px;font-size:13px;
  font-weight:700;color:var(--primary);border-bottom:1px solid var(--border);
  text-align:center}
.param-table th.param-head{background:#eef2ff;color:var(--muted);
  font-weight:600;width:110px}
.param-table td{padding:8px;border-bottom:1px solid var(--border);
  border-left:1px solid var(--border)}
.param-table td.param-label{background:#fafbfc;font-size:13px;
  color:var(--muted);font-weight:600;text-align:center;
  border-left:none;white-space:nowrap}
.param-table tbody tr:last-child td{border-bottom:none}
.param-table input{width:100%;padding:9px 10px;border:1px solid var(--border);
  border-radius:8px;font-size:14px;background:#fff;transition:.15s;
  box-sizing:border-box;text-align:center}
.param-table input:focus{outline:none;border-color:var(--primary);
  box-shadow:0 0 0 3px rgba(37,99,235,.1)}

/* 轴启用切换 */
.axis-toggle{display:inline-block;padding:6px 14px;border-radius:8px;
  cursor:pointer;font-weight:700;font-size:13px;transition:.15s;
  background:#fff;color:#94a3b8;border:2px solid var(--border);user-select:none}
.axis-toggle:hover{border-color:#cbd5e1}
.axis-toggle.active{background:var(--primary);color:#fff;border-color:var(--primary)}

/* 底部按钮行 */
.param-foot td{padding:12px 10px;border-top:2px solid var(--border);border-left:none}
.action-buttons{display:flex;flex-wrap:wrap;gap:8px;justify-content:center}

@media(max-width:600px){
  h1{font-size:18px}
  .param-table th,.param-table td{padding:6px 3px;font-size:12px}
  .param-table th.param-head{width:80px}
  .param-table input{padding:7px 4px;font-size:13px}
  .axis-toggle{padding:5px 10px;font-size:12px}
}
</style></head><body><div class="container">
<h1>✍️ 写字机 <span>控制台</span></h1>
)rawliteral";

  // ---- 状态卡片 ----
  html += "<div class='card'>";
  html += "<div class='status-row'>";
  html += "<div id='runBadge' class='status-badge ";
  html += machineBusy() ? "on" : "off";
  html += "'><span class='dot'></span>";
  html += machineBusy() ? "运行中" : "已停止";
  html += "</div>";
  html += "<div class='status-badge ";
  html += staConnected ? "on" : "off";
  html += "'><span class='dot'></span>";
  html += staConnected ? "WiFi 已连接" : "WiFi 未连接";
  html += "</div></div>";

  if (staConnected) {
    html += "<div class='info-line'>路由器：" + savedSSID + "　IP：http://" + staIP + "</div>";
  }
  if (apActive) {
    html += "<div class='info-line'>配置热点：" + String(AP_SSID) + "　IP：http://" + apIP + "</div>";
  }
  html += "<div class='coord-line'>";
  html += "<span>X: <b id='posX'>0</b></span>";
  html += "<span>Y: <b id='posY'>0</b></span>";
  html += "<span>Z: <b id='posZ'>0</b></span>";
  html += "</div>";
  html += "</div>";

  // ---- 运动参数卡片 ----
  html += "<div class='card'><h3>运动参数 <span class='tag'>三轴</span></h3>";
  html += "<div style='overflow-x:auto'>";
  html += "<table class='param-table'>";

  html += "<thead><tr>";
  html += "<th class='param-head'>参数</th>";
  html += "<th>" + axisToggleHtml(AXIS_X, "x", "X 轴") + "</th>";
  html += "<th>" + axisToggleHtml(AXIS_Y, "y", "Y 轴") + "</th>";
  html += "<th>" + axisToggleHtml(AXIS_Z, "z", "Z 轴") + "</th>";
  html += "</tr></thead>";

  html += "<tbody>";
  html += "<tr>";
  html += "<td class='param-label'>速度 (Hz)</td>";
  html += "<td><input type='number' id='xSpeed' value='" + String(xParam.speed) + "'></td>";
  html += "<td><input type='number' id='ySpeed' value='" + String(yParam.speed) + "'></td>";
  html += "<td><input type='number' id='zSpeed' value='" + String(zParam.speed) + "'></td>";
  html += "</tr>";

  html += "<tr>";
  html += "<td class='param-label'>加速度</td>";
  html += "<td><input type='number' id='xAccel' value='" + String(xParam.acceleration) + "'></td>";
  html += "<td><input type='number' id='yAccel' value='" + String(yParam.acceleration) + "'></td>";
  html += "<td><input type='number' id='zAccel' value='" + String(zParam.acceleration) + "'></td>";
  html += "</tr>";

  html += "<tr>";
  html += "<td class='param-label'>行程 (步)</td>";
  html += "<td><input type='number' id='xTravel' value='" + String(xParam.travel) + "'></td>";
  html += "<td><input type='number' id='yTravel' value='" + String(yParam.travel) + "'></td>";
  html += "<td><input type='number' id='zTravel' value='" + String(zParam.travel) + "'></td>";
  html += "</tr>";

  html += "<tr>";
  html += "<td class='param-label'>停顿 (ms)</td>";
  html += "<td><input type='number' id='xPause' value='" + String(xParam.pauseMs) + "'></td>";
  html += "<td><input type='number' id='yPause' value='" + String(yParam.pauseMs) + "'></td>";
  html += "<td><input type='number' id='zPause' value='" + String(zParam.pauseMs) + "'></td>";
  html += "</tr>";
  html += "<tr>";
  html += "<td class='param-label'>最大行程 (mm)</td>";
  html += "<td><input type='number' id='xMaxMm' value='" + String(xParam.maxTravelMm) + "' title='0 = 不限制'></td>";
  html += "<td><input type='number' id='yMaxMm' value='" + String(yParam.maxTravelMm) + "' title='0 = 不限制'></td>";
  html += "<td><input type='number' id='zMaxMm' value='" + String(zParam.maxTravelMm) + "' title='0 = 不限制'></td>";
  html += "</tr>";
  html += "</tbody>";

  // 末行：往返、单次、停止、清零、保存
  html += "<tfoot class='param-foot'><tr><td colspan='4'>";
  html += "<div class='action-buttons'>";
  html += "<button class='btn btn-success' onclick='startRun(\"loop\")'>▶ 往返</button>";
  html += "<button class='btn btn-success' onclick='startRun(\"once\")'>▶ 单次</button>";
  html += "<button class='btn btn-danger' onclick='stopRun()'>■ 停止</button>";
  html += "<button class='btn btn-warning' onclick='homeAxes()'>⟲ 清零</button>";
  html += "<button class='btn btn-primary' onclick='saveParams()'>💾 保存</button>";
  html += "</div>";
  html += "</td></tr></tfoot>";

  html += "</table>";
  html += "</div>";
  html += "<div class='note'>表头点击可启用/停用对应轴；参数会写入 NVS，断电后自动恢复。"
          "各轴按自己的停顿值独立往返，不再共用最大值。"
          "最大行程是 API 与手动运行的安全上限 (mm，100 步/mm)，0 表示不限制。</div>";
  html += "<div class='note'>接口：http://" + String(staConnected ? staIP : apIP) +
          "/moveto?x=1000&amp;y=1000&amp;z=500&amp;xv=150000&amp;xac=150000&amp;xsl=100"
          "&amp;yv=150000&amp;yac=150000&amp;ysl=100&amp;zv=150000&amp;zac=150000&amp;zsl=100"
          "　各轴到位后按自己的 sl 停顿再各自返回原点；v=速度Hz，ac=加速度，sl=停顿ms，"
          "加 save=1 可写入 NVS，进度查 /status。</div>";
  html += "</div>";

  // ---- WiFi 配置（最下面） ----
  html += "<div class='card'><h3>WiFi 配置</h3>";
  if (!apActive && staConnected) {
    html += "<button class='btn btn-warning' onclick='enableAP()'>开启配置热点</button>";
    html += "<div class='note'>当前未开 AP。如需更换 WiFi，点此临时开启热点，连上后配置。</div>";
  } else {
    html += "<div class='btn-row'><button class='btn btn-warning' onclick='scanWifi()'>扫描附近 WiFi</button></div>";
    html += "<div id='scanStatus' class='note'></div>";
    html += "<div id='wifiList'></div>";
    html += "<div class='row' style='margin-top:8px'><label>密码</label>";
    html += "<input type='password' id='wifiPass' placeholder='输入所选 WiFi 的密码'></div>";
    html += "<button class='btn btn-primary' onclick='connectWifi()'>连接所选 WiFi</button>";
    html += "<div class='note'>连接成功后 AP 会自动关闭，请记下新 STA IP，切回路由器 WiFi 后用该 IP 访问。</div>";
  }
  html += "</div>";

  // ---- 脚本 ----
  html += R"rawliteral(
<script>
function toggleAxis(el){
  el.classList.toggle('active');
}
function getAxes(){
  const els = document.querySelectorAll('.axis-toggle.active');
  let s = '';
  els.forEach(el => s += el.dataset.axis);
  return s;
}

function saveParams(){
  let p = new URLSearchParams({
    xSpeed: document.getElementById('xSpeed').value,
    xAccel: document.getElementById('xAccel').value,
    xTravel: document.getElementById('xTravel').value,
    xPause: document.getElementById('xPause').value,
    xMaxMm: document.getElementById('xMaxMm').value,
    ySpeed: document.getElementById('ySpeed').value,
    yAccel: document.getElementById('yAccel').value,
    yTravel: document.getElementById('yTravel').value,
    yPause: document.getElementById('yPause').value,
    yMaxMm: document.getElementById('yMaxMm').value,
    zSpeed: document.getElementById('zSpeed').value,
    zAccel: document.getElementById('zAccel').value,
    zTravel: document.getElementById('zTravel').value,
    zPause: document.getElementById('zPause').value,
    zMaxMm: document.getElementById('zMaxMm').value
  });
  fetch('/save?' + p.toString()).then(() => alert('参数已保存'));
}

function startRun(mode){
  const axes = getAxes();
  if(!axes){ alert('请至少启用一个轴'); return; }
  fetch('/start?axes=' + axes + '&mode=' + mode)
    .then(r => r.text())
    .then(t => {
      if(t === 'NO_AXIS'){ alert('请至少启用一个轴'); return; }
      if(t.indexOf('busy') >= 0){ alert('机器正在执行任务，完成后才能接受新指令'); }
      if(t.indexOf('TRAVEL_LIMIT') >= 0){ alert('行程超出最大行程限制，请在运动参数中检查'); }
    });
}
function stopRun(){ fetch('/stop'); }
function homeAxes(){
  if(confirm('位置清零会把所有轴计数归零，确认滑台已在起点？'))
    fetch('/home');
}

async function enableAP(){
  const res = await fetch('/enableap');
  const d = await res.json();
  alert('配置热点已开启\nSSID: ' + d.ssid + '\n密码: ' + d.pass + '\nIP: http://' + d.ip + '\n\n请连接该热点后访问上面的 IP');
  location.reload();
}
async function scanWifi(){
  document.getElementById('scanStatus').innerText = '扫描中，约 3~5 秒...';
  document.getElementById('wifiList').innerHTML = '';
  try{
    const res = await fetch('/scan');
    const list = await res.json();
    if(list.length === 0){ document.getElementById('scanStatus').innerText = '没有扫描到 WiFi'; return; }
    let h = '<select id="ssidSel" size="6">';
    list.forEach(w => { h += '<option value="' + w.ssid + '">' + w.ssid + '  (' + w.rssi + ' dBm)</option>'; });
    h += '</select>';
    document.getElementById('wifiList').innerHTML = h;
    document.getElementById('scanStatus').innerText = '找到 ' + list.length + ' 个网络';
  }catch(e){
    document.getElementById('scanStatus').innerText = '扫描失败：' + e;
  }
}
async function connectWifi(){
  const sel = document.getElementById('ssidSel');
  if(!sel){ alert('请先扫描并选择 WiFi'); return; }
  const ssid = sel.value;
  const pass = document.getElementById('wifiPass').value;
  document.getElementById('scanStatus').innerText = '正在连接 ' + ssid + '，最多 20 秒...';
  try{
    const res = await fetch('/connect?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass));
    const d = await res.json();
    if(d.ok){
      document.getElementById('scanStatus').innerHTML =
        '连接成功！<br>新 IP：<b>http://' + d.ip + '</b><br>AP 将在几秒后关闭，请切换到路由器 WiFi 后用该 IP 访问。';
    }else{
      document.getElementById('scanStatus').innerText = '连接失败：' + d.msg;
    }
  }catch(e){
    document.getElementById('scanStatus').innerText = '请求失败：' + e;
  }
}

async function pollStatus(){
  try{
    const r = await fetch('/status');
    const d = await r.json();
    const badge = document.getElementById('runBadge');
    if(d.running){
      badge.className = 'status-badge on';
      badge.innerHTML = "<span class='dot'></span>运行中";
    }else{
      badge.className = 'status-badge off';
      badge.innerHTML = "<span class='dot'></span>已停止";
    }
    document.getElementById('posX').innerText = d.x;
    document.getElementById('posY').innerText = d.y;
    document.getElementById('posZ').innerText = d.z;
  }catch(e){}
}
setInterval(pollStatus, 800);
pollStatus();
</script>
</div></body></html>
)rawliteral";

  return html;
}

// ================= 路由 =================
void handleMoveTo();  // handleNotFound 也会调用，前置声明

void handleRoot() {
  server.send(200, "text/html", buildPage());
}

void handleNotFound() {
  // 兼容 http://IP/任意路径?x=..&y=.. 形式的接口调用
  if (server.hasArg("x") || server.hasArg("y") || server.hasArg("z")) {
    handleMoveTo();
    return;
  }
  if (!apActive) {
    server.send(404, "text/plain", "Not Found");
    return;
  }
  String host = server.hostHeader();
  if (host.length() == 0 || host.indexOf(apIP) < 0) {
    server.sendHeader("Location", "http://" + apIP + "/", true);
    server.send(302, "text/plain", "");
    return;
  }
  server.send(404, "text/plain", "Not Found");
}

void handleEnableAP() {
  enableAP();
  String resp = "{\"ok\":true,\"ssid\":\"" + String(AP_SSID) +
                "\",\"pass\":\"" + String(AP_PASS) +
                "\",\"ip\":\"" + apIP + "\"}";
  server.send(200, "application/json", resp);
}

void handleScan() {
  int n = WiFi.scanNetworks();
  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    json += "{\"ssid\":\"" + escapeJson(WiFi.SSID(i)) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
  }
  json += "]";
  WiFi.scanDelete();
  server.send(200, "application/json", json);
}

void handleConnect() {
  if (!server.hasArg("ssid")) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"缺少 SSID\"}");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");

  Serial.print("尝试连接 WiFi: "); Serial.println(ssid);

  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid.c_str(), pass.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    staConnected = true;
    staIP = WiFi.localIP().toString();
    savedSSID = ssid;
    savedPass = pass;
    saveWiFiCreds(ssid, pass);
    Serial.print("连接成功，IP: "); Serial.println(staIP);

    if (apActive) {
      closeAPPending = true;
      closeAPTime = millis();
    }
    String resp = "{\"ok\":true,\"ip\":\"" + staIP + "\"}";
    server.send(200, "application/json", resp);
  } else {
    staConnected = false;
    Serial.println("连接失败");
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"连接超时，请检查密码或信号\"}");
  }
}

void handleSave() {
  if (server.hasArg("xSpeed"))  xParam.speed        = server.arg("xSpeed").toInt();
  if (server.hasArg("xAccel"))  xParam.acceleration = server.arg("xAccel").toInt();
  if (server.hasArg("xTravel")) xParam.travel       = server.arg("xTravel").toInt();
  if (server.hasArg("xPause"))  xParam.pauseMs      = server.arg("xPause").toInt();
  if (server.hasArg("xMaxMm")) { long v = server.arg("xMaxMm").toInt(); if (v >= 0) xParam.maxTravelMm = (uint32_t)v; }

  if (server.hasArg("ySpeed"))  yParam.speed        = server.arg("ySpeed").toInt();
  if (server.hasArg("yAccel"))  yParam.acceleration = server.arg("yAccel").toInt();
  if (server.hasArg("yTravel")) yParam.travel       = server.arg("yTravel").toInt();
  if (server.hasArg("yPause"))  yParam.pauseMs      = server.arg("yPause").toInt();
  if (server.hasArg("yMaxMm")) { long v = server.arg("yMaxMm").toInt(); if (v >= 0) yParam.maxTravelMm = (uint32_t)v; }

  if (server.hasArg("zSpeed"))  zParam.speed        = server.arg("zSpeed").toInt();
  if (server.hasArg("zAccel"))  zParam.acceleration = server.arg("zAccel").toInt();
  if (server.hasArg("zTravel")) zParam.travel       = server.arg("zTravel").toInt();
  if (server.hasArg("zPause"))  zParam.pauseMs      = server.arg("zPause").toInt();
  if (server.hasArg("zMaxMm")) { long v = server.arg("zMaxMm").toInt(); if (v >= 0) zParam.maxTravelMm = (uint32_t)v; }

  applyParams();
  saveParamsToNVS();
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  if (machineBusy()) {
    sendBusy();
    return;
  }

  String axes = server.hasArg("axes") ? server.arg("axes") : "";
  axisMask = 0;
  if (axes.indexOf('x') >= 0) axisMask |= AXIS_X;
  if (axes.indexOf('y') >= 0) axisMask |= AXIS_Y;
  if (axes.indexOf('z') >= 0) axisMask |= AXIS_Z;

  if (server.hasArg("mode")) {
    runMode = (server.arg("mode") == "once") ? RUN_MODE_ONCE : RUN_MODE_LOOP;
  }

  if (axisMask == 0) {
    server.send(200, "text/plain", "NO_AXIS");
    return;
  }

  const char* over = 0;
  if ((axisMask & AXIS_X) && targetExceedsLimit(xParam.travel, xParam.maxTravelMm)) over = "x";
  else if ((axisMask & AXIS_Y) && targetExceedsLimit(yParam.travel, yParam.maxTravelMm)) over = "y";
  else if ((axisMask & AXIS_Z) && targetExceedsLimit(zParam.travel, zParam.maxTravelMm)) over = "z";
  if (over) {
    server.send(400, "text/plain", String("TRAVEL_LIMIT_") + over);
    return;
  }

  prefs.begin("writing", false);
  prefs.putUInt("axisMask", axisMask);
  prefs.putUInt("runMode", runMode);
  prefs.end();

  stopCycles();
  stopAllSteppers();

  bool repeat = (runMode == RUN_MODE_LOOP);
  armCycle(xCycle, (axisMask & AXIS_X) != 0, xParam.travel, xParam.pauseMs, xParam.pauseMs, repeat);
  armCycle(yCycle, (axisMask & AXIS_Y) != 0, yParam.travel, yParam.pauseMs, yParam.pauseMs, repeat);
  armCycle(zCycle, (axisMask & AXIS_Z) != 0, zParam.travel, zParam.pauseMs, zParam.pauseMs, repeat);

  systemRunning = true;
  server.send(200, "text/plain", "STARTED");
}

void handleStop() {
  stopCycles();
  stopAllSteppers();
  server.send(200, "text/plain", "STOPPED");
}

void handleHome() {
  stopCycles();
  stopAllSteppers();
  if (xStepper) xStepper->setCurrentPosition(0);
  if (yLeft)    yLeft->setCurrentPosition(0);
  if (yRight)   yRight->setCurrentPosition(0);
  if (zStepper) zStepper->setCurrentPosition(0);
  server.send(200, "text/plain", "HOMED");
}

void handleStatus() {
  String json = "{";
  json += "\"running\":" + String(machineBusy() ? "true" : "false") + ",";
  json += "\"api\":" + String(apiRunning ? "true" : "false") + ",";
  json += "\"stage\":{" + stageJson() + "},";
  json += "\"click\":" + String(clickRunning ? "true" : "false") + ",";
  json += "\"clickPhase\":" + String(click.phase) + ",";
  json += "\"x\":" + String(xStepper ? xStepper->getCurrentPosition() : 0) + ",";
  json += "\"y\":" + String(yLeft ? yLeft->getCurrentPosition() : 0) + ",";
  json += "\"z\":" + String(zStepper ? zStepper->getCurrentPosition() : 0);
  json += "}";
  server.send(200, "application/json", json);
}

// ================= API：定点往返 =================
// GET /moveto?x=1000&y=1000&z=500
//            &xv=150000&xac=150000&xsl=100
//            &yv=150000&yac=150000&ysl=100
//            &zv=150000&zac=150000&zsl=100
//            [&pause=300][&save=1]
//   x/y/z 目标步数，v 速度(Hz)，ac 加速度，sl 该轴到位停顿(ms)，缺省项沿用表格中已保存的参数
//   pause 统一覆盖三轴停顿，save=1 把速度/加速度/停顿写入 NVS
// 各轴到位后按自己的 sl 停顿，再各自返回原点 (0,0,0)。接口立即返回，进度查 /status。

// 读取单个轴的 v/ac/sl，只接受有效值，返回参数是否有改动
bool readAxisTuning(const String& axis, AxisParam& param) {
  bool changed = false;

  String key = axis + "v";
  if (server.hasArg(key)) {
    long value = server.arg(key).toInt();
    if (value > 0) { param.speed = (uint32_t)value; changed = true; }
  }

  key = axis + "ac";
  if (server.hasArg(key)) {
    long value = server.arg(key).toInt();
    if (value > 0) { param.acceleration = (uint32_t)value; changed = true; }
  }

  key = axis + "sl";
  if (server.hasArg(key)) {
    long value = server.arg(key).toInt();
    if (value >= 0) { param.pauseMs = (uint32_t)value; changed = true; }
  }

  return changed;
}

// sl 报告本次实际生效的该轴停顿值
String axisTuningJson(const char* name, const AxisParam& param, const AxisCycle& axis) {
  return "\"" + String(name) + "\":{\"on\":" + String(axis.enabled ? "true" : "false") +
         ",\"v\":" + String(param.speed) +
         ",\"ac\":" + String(param.acceleration) +
         ",\"sl\":" + String(axis.dwellMs) + "}";
}

void handleMoveTo() {
  if (machineBusy()) {
    sendBusy();
    return;
  }

  bool hasX = server.hasArg("x");
  bool hasY = server.hasArg("y");
  bool hasZ = server.hasArg("z");

  if (!hasX && !hasY && !hasZ) {
    server.send(400, "application/json",
                "{\"ok\":false,\"msg\":\"missing x/y/z, usage: /moveto?x=1000&y=1000\"}");
    return;
  }

  // 行程保护：目标超出最大行程 (mm) 则拒绝
  int32_t targetX = hasX ? (int32_t)server.arg("x").toInt() : 0;
  int32_t targetY = hasY ? (int32_t)server.arg("y").toInt() : 0;
  int32_t targetZ = hasZ ? (int32_t)server.arg("z").toInt() : 0;
  String over = "";
  if (hasX && targetExceedsLimit(targetX, xParam.maxTravelMm)) over += "x ";
  if (hasY && targetExceedsLimit(targetY, yParam.maxTravelMm)) over += "y ";
  if (hasZ && targetExceedsLimit(targetZ, zParam.maxTravelMm)) over += "z ";
  if (over.length() > 0) {
    String msg = "{\"ok\":false,\"msg\":\"target exceeds max travel: ";
    msg += over; msg += "\"}";
    server.send(400, "application/json", msg);
    return;
  }

  // 忙检查已通过，此处必然空闲；清状态仅为兜底
  stopCycles();
  stopAllSteppers();

  bool tuningChanged = false;
  if (readAxisTuning("x", xParam)) tuningChanged = true;
  if (readAxisTuning("y", yParam)) tuningChanged = true;
  if (readAxisTuning("z", zParam)) tuningChanged = true;
  if (tuningChanged) applyParams();

  bool saved = false;
  if (tuningChanged && server.hasArg("save") && server.arg("save") != "0") {
    saveParamsToNVS();
    saved = true;
  }

  // 停顿各轴独立：带 pause 时统一覆盖，否则用各自的 sl
  bool     hasGlobalPause = server.hasArg("pause");
  uint32_t globalPause = hasGlobalPause ? (uint32_t)server.arg("pause").toInt() : 0;

  // 接口任务只跑一轮，回到原点即结束
  armCycle(xCycle, hasX, targetX,
           hasGlobalPause ? globalPause : xParam.pauseMs, 0, false);
  armCycle(yCycle, hasY, targetY,
           hasGlobalPause ? globalPause : yParam.pauseMs, 0, false);
  armCycle(zCycle, hasZ, targetZ,
           hasGlobalPause ? globalPause : zParam.pauseMs, 0, false);

  apiRunning = true;

  Serial.print("API 定点往返:");
  if (hasX) {
    Serial.print("  X="); Serial.print(xCycle.target);
    Serial.print(" 停顿="); Serial.print(xCycle.dwellMs); Serial.print("ms");
  }
  if (hasY) {
    Serial.print("  Y="); Serial.print(yCycle.target);
    Serial.print(" 停顿="); Serial.print(yCycle.dwellMs); Serial.print("ms");
  }
  if (hasZ) {
    Serial.print("  Z="); Serial.print(zCycle.target);
    Serial.print(" 停顿="); Serial.print(zCycle.dwellMs); Serial.print("ms");
  }
  if (tuningChanged) {
    Serial.print("  [X v="); Serial.print(xParam.speed);
    Serial.print(" ac="); Serial.print(xParam.acceleration);
    Serial.print("] [Y v="); Serial.print(yParam.speed);
    Serial.print(" ac="); Serial.print(yParam.acceleration);
    Serial.print("] [Z v="); Serial.print(zParam.speed);
    Serial.print(" ac="); Serial.print(zParam.acceleration); Serial.print("]");
    if (saved) Serial.print("  已存 NVS");
  }
  Serial.println();

  String resp = "{\"ok\":true,\"target\":{\"x\":" + String(xCycle.target) +
                ",\"y\":" + String(yCycle.target) +
                ",\"z\":" + String(zCycle.target) +
                "},\"tuning\":{" + axisTuningJson("x", xParam, xCycle) + "," +
                axisTuningJson("y", yParam, yCycle) + "," +
                axisTuningJson("z", zParam, zCycle) + "}," +
                "\"saved\":" + String(saved ? "true" : "false") +
                ",\"msg\":\"started, poll /status\"}";
  server.send(200, "application/json", resp);
}

// ================= API：点击运动（移动到点 + Z 轴单击）=================
// GET /click?x=10000&y=15000[&z=500]     绝对坐标：X/Y 移动到 (x, y) 并停住
// GET /clickto?x=1000&y=1500[&z=500]     相对坐标：X/Y 移动到 当前+(x, y) 并停住
//   到位后 Z 轴做 zTrip 步的单次往返，X/Y 不返回原点。
//   z 缺省 500；未给出的 X/Y 不移动。上一条任务完成后才接受新指令。
void armClick(bool hasX, int32_t targetX, bool hasY, int32_t targetY, int32_t zTrip) {
  stopCycles();
  stopAllSteppers();

  click.hasX    = hasX;
  click.hasY    = hasY;
  click.targetX = targetX;
  click.targetY = targetY;
  click.zTrip   = zTrip;
  click.zBase   = zStepper ? zStepper->getCurrentPosition() : 0;
  click.phase   = 0;
  clickRunning  = true;

  Serial.print("API 点击: X="); Serial.print(click.targetX);
  Serial.print("  Y="); Serial.print(click.targetY);
  Serial.print("  Z单击="); Serial.print(click.zTrip); Serial.println("步");
}

String clickStartJson(int32_t zTrip) {
  return "{\"ok\":true,\"target\":{\"x\":" + String(click.targetX) +
         ",\"y\":" + String(click.targetY) +
         "},\"zTrip\":" + String(zTrip) +
         ",\"msg\":\"started, poll /status\"}";
}

void handleClick() {
  if (machineBusy()) { sendBusy(); return; }

  bool    hasX  = server.hasArg("x");
  bool    hasY  = server.hasArg("y");
  int32_t zTrip = 500;
  if (server.hasArg("z")) zTrip = (int32_t)server.arg("z").toInt();

  int32_t targetX = hasX ? (int32_t)server.arg("x").toInt() : 0;
  int32_t targetY = hasY ? (int32_t)server.arg("y").toInt() : 0;
  if ((hasX && targetExceedsLimit(targetX, xParam.maxTravelMm)) ||
      (hasY && targetExceedsLimit(targetY, yParam.maxTravelMm))) {
    server.send(400, "application/json",
                "{\"ok\":false,\"msg\":\"target exceeds max travel\"}");
    return;
  }

  armClick(hasX, targetX, hasY, targetY, zTrip);
  server.send(200, "application/json", clickStartJson(zTrip));
}

void handleClickTo() {
  if (machineBusy()) { sendBusy(); return; }

  bool    hasX  = server.hasArg("x");
  bool    hasY  = server.hasArg("y");
  int32_t zTrip = 500;
  if (server.hasArg("z")) zTrip = (int32_t)server.arg("z").toInt();

  // 相对坐标：当前 + 偏移（忙检查通过时机器必然空闲，位置计数准确）
  int32_t baseX = xStepper ? xStepper->getCurrentPosition() : 0;
  int32_t baseY = yLeft ? yLeft->getCurrentPosition() : 0;
  int32_t offX  = hasX ? (int32_t)server.arg("x").toInt() : 0;
  int32_t offY  = hasY ? (int32_t)server.arg("y").toInt() : 0;

  int32_t targetX = baseX + offX;
  int32_t targetY = baseY + offY;
  if ((hasX && targetExceedsLimit(targetX, xParam.maxTravelMm)) ||
      (hasY && targetExceedsLimit(targetY, yParam.maxTravelMm))) {
    server.send(400, "application/json",
                "{\"ok\":false,\"msg\":\"target exceeds max travel\"}");
    return;
  }

  armClick(hasX, targetX, hasY, targetY, zTrip);
  server.send(200, "application/json", clickStartJson(zTrip));
}

// ================= 非阻塞往复推进（网页端与 API 共用）=================
// 推进单个轴，返回它接下来要下发的动作
AxisAction advanceAxisCycle(AxisCycle& axis, bool running) {
  if (!axis.enabled || axis.stage >= 4) return AXIS_NONE;

  switch (axis.stage) {
    case 0:  // 去程：到位后开始计自己的停顿
      if (!running) {
        axis.resumeAt = millis() + axis.dwellMs;
        axis.stage = 1;
      }
      break;

    case 1:  // 到位停顿：到点就各自返回，不等其他轴
      if ((int32_t)(millis() - axis.resumeAt) >= 0) {
        axis.stage = 2;
        return AXIS_GO_HOME;
      }
      break;

    case 2:  // 回程：等回到原点
      if (!running) {
        if (axis.homeMs > 0) {
          axis.resumeAt = millis() + axis.homeMs;
          axis.stage = 3;
        } else if (axis.repeat) {
          axis.stage = 0;
          return AXIS_GO_TARGET;
        } else {
          axis.stage = 4;
        }
      }
      break;

    case 3:  // 原点停顿：往返模式接着下一轮，单次模式结束
      if ((int32_t)(millis() - axis.resumeAt) >= 0) {
        if (axis.repeat) {
          axis.stage = 0;
          return AXIS_GO_TARGET;
        }
        axis.stage = 4;
      }
      break;
  }
  return AXIS_NONE;
}

void updateCycles() {
  if (!systemRunning && !apiRunning) return;

  if (!cycleStarted) {
    setMotorEnable(true);
    if (xCycle.enabled) xStepper->moveTo(xCycle.target, false);
    if (yCycle.enabled) {
      yLeft->moveTo(yCycle.target, false);
      yRight->moveTo(yCycle.target, false);
    }
    if (zCycle.enabled) zStepper->moveTo(zCycle.target, false);
    cycleStarted = true;
    return;
  }

  AxisAction actX = advanceAxisCycle(xCycle, xStepper->isRunning());
  if (actX == AXIS_GO_HOME)   xStepper->moveTo(0, false);
  if (actX == AXIS_GO_TARGET) xStepper->moveTo(xCycle.target, false);

  bool yRunning = yLeft->isRunning() || yRight->isRunning();
  AxisAction actY = advanceAxisCycle(yCycle, yRunning);
  if (actY == AXIS_GO_HOME) {
    yLeft->moveTo(0, false);
    yRight->moveTo(0, false);
  }
  if (actY == AXIS_GO_TARGET) {
    yLeft->moveTo(yCycle.target, false);
    yRight->moveTo(yCycle.target, false);
  }

  AxisAction actZ = advanceAxisCycle(zCycle, zStepper->isRunning());
  if (actZ == AXIS_GO_HOME)   zStepper->moveTo(0, false);
  if (actZ == AXIS_GO_TARGET) zStepper->moveTo(zCycle.target, false);

  // 单次任务：所有参与轴都跑完才停机断使能
  if (xCycle.stage >= 4 && yCycle.stage >= 4 && zCycle.stage >= 4) {
    bool wasApi = apiRunning;
    systemRunning = false;
    apiRunning    = false;
    cycleStarted  = false;
    setMotorEnable(false);
    Serial.println(wasApi ? "API 定点往返完成，各轴已回到原点"
                          : "单次运行完成，各轴已回到原点");
  }
}

// ================= 点击运动推进 =================
void updateClick() {
  if (!clickRunning) return;

  switch (click.phase) {
    case 0:  // 下发 X/Y 去程（到位后停住，不返回原点）
      setMotorEnable(true);
      if (click.hasX) xStepper->moveTo(click.targetX, false);
      if (click.hasY) {
        yLeft->moveTo(click.targetY, false);
        yRight->moveTo(click.targetY, false);
      }
      click.phase = 1;
      break;

    case 1:  // 等 X/Y 都到位，然后 Z 下压
      if ((!click.hasX || !xStepper->isRunning()) &&
          (!click.hasY || !(yLeft->isRunning() || yRight->isRunning()))) {
        zStepper->moveTo(click.zBase + click.zTrip, false);
        click.phase = 2;
      }
      break;

    case 2:  // 等 Z 下压到位，然后抬起
      if (!zStepper->isRunning()) {
        zStepper->moveTo(click.zBase, false);
        click.phase = 3;
      }
      break;

    case 3:  // 等 Z 抬起到位，结束
      if (!zStepper->isRunning()) {
        clickRunning = false;
        click.phase  = 0;
        setMotorEnable(false);
        Serial.println("点击完成：X/Y 停在目标，Z 已抬起");
      }
      break;
  }
}

// ================= setup =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  initParamDefaults();
  loadParams();
  loadWiFiCreds();

  bool initialConnected = false;

  if (savedSSID.length() > 0) {
    Serial.print("尝试自动连接 WiFi: "); Serial.println(savedSSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(savedSSID.c_str(), savedPass.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      initialConnected = true;
      staConnected = true;
      staIP = WiFi.localIP().toString();
      Serial.print("STA 连接成功，IP: http://"); Serial.println(staIP);
      Serial.println("AP 热点未开启");
    } else {
      Serial.println("30 秒内未连接成功");
    }
  } else {
    Serial.println("未保存 WiFi 凭据");
  }

  if (!initialConnected) {
    enableAP();
    Serial.println("已开启 AP 热点，连接后浏览器会自动跳转到配置页");
  }

  pinMode(EN_PIN, OUTPUT);
  setMotorEnable(false);

  engine.init();
  xStepper = engine.stepperConnectToPin(X_STEP_PIN);
  yLeft    = engine.stepperConnectToPin(Y_STEP_L);
  yRight   = engine.stepperConnectToPin(Y_STEP_R);
  zStepper = engine.stepperConnectToPin(Z_STEP_PIN);

  if (!xStepper || !yLeft || !yRight || !zStepper) {
    Serial.println("电机初始化失败");
    while (1);
  }

  xStepper->setDirectionPin(X_DIR_PIN);
  yLeft->setDirectionPin(Y_DIR_L);
  yRight->setDirectionPin(Y_DIR_R);
  zStepper->setDirectionPin(Z_DIR_PIN, false);  // Z 方向已翻转

  // 上电默认位置清零 (0, 0, 0)
  xStepper->setCurrentPosition(0);
  yLeft->setCurrentPosition(0);
  yRight->setCurrentPosition(0);
  zStepper->setCurrentPosition(0);
  Serial.println("上电位置默认清零：X=0  Y=0  Z=0");

  applyParams();

  server.on("/",        handleRoot);
  server.on("/enableap",handleEnableAP);
  server.on("/scan",    handleScan);
  server.on("/connect", handleConnect);
  server.on("/save",    handleSave);
  server.on("/start",   handleStart);
  server.on("/stop",    handleStop);
  server.on("/home",    handleHome);
  server.on("/status",  handleStatus);
  server.on("/moveto",  handleMoveTo);
  server.on("/move",    handleMoveTo);  // 兼容旧路径
  server.on("/click",   handleClick);
  server.on("/clickto", handleClickTo);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Web 服务器已启动");
}

// ================= loop =================
void loop() {
  server.handleClient();
  if (apActive) {
    dnsServer.processNextRequest();
  }
  updateCycles();
  updateClick();

  if (closeAPPending && millis() - closeAPTime > 2000) {
    disableAP();
    closeAPPending = false;
  }
}
