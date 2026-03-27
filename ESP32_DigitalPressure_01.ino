/**
 * ESP32 Digital Pressure Sensor (WNK Series) I2C Reader + Web Dashboard
 *
 * MCU: ESP32-WROOM-32UE
 * Sensor: WNK21/WD19/WNK811/WNK80mA/WNK8010 (3.3V I2C)
 * I2C Address: 0x6D (7-bit)
 *
 * Wiring:
 *   ESP32 GPIO21 (SDA) -> Sensor SDA
 *   ESP32 GPIO22 (SCL) -> Sensor SCL
 *   ESP32 3.3V         -> Sensor VCC
 *   ESP32 GND          -> Sensor GND
 *
 * Web Dashboard:
 *   - Connect to WiFi (STA mode)
 *   - Access http://<ESP32_IP>/ for live gauge display
 *   - Pressure gauge (circular) + Temperature gauge (thermometer)
 *   - Auto-refresh every 5 seconds via JSON API
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// ==================== WiFi 설정 ====================
const char* ssid     = "KT_GiGA_9748";       // WiFi SSID 입력
// const char* ssid     = "KT_GiGA_5G_9748";       // WiFi SSID 입력
// const char* password = "YOUR_PASSWORD";   // WiFi 비밀번호 입력
const char* password = "9cf0bkd529";   // WiFi 비밀번호 입력

// ==================== I2C 설정 ====================
#define SENSOR_I2C_ADDR   0x6D    // 7-bit address (0xDA >> 1)
#define I2C_SDA_PIN       21
#define I2C_SCL_PIN       22
#define I2C_CLOCK_FREQ    400000  // 400kHz

// 레지스터 주소
#define REG_PRESSURE      0x06    // 압력 데이터 시작 주소 (0x06, 0x07, 0x08)
#define REG_TEMPERATURE   0x09    // 온도 데이터 시작 주소 (0x09, 0x0A, 0x0B)

// 센서 압력 레인지 (kPa)
#define PRESSURE_RANGE    2000.0f

// 단위 변환: 1 kPa = 10 mbar
#define KPA_TO_MBAR       10.0f

// 영점 보정값 (mbar)
#define PRESSURE_OFFSET   40.0f

// 읽기 주기 (ms)
#define READ_INTERVAL     1000

// 이동평균 필터 샘플 수
#define FILTER_SIZE       10

// ==================== 웹서버 ====================
#define WEB_SERVER_PORT   8282    // 비표준 포트 (기본 80 대신 사용)
WebServer server(WEB_SERVER_PORT);

// ==================== 전역 센서 데이터 (웹 API용) ====================
volatile float g_pressure      = 0.0f;
volatile float g_pressureAvg   = 0.0f;
volatile float g_temperature   = 0.0f;
volatile float g_temperatureAvg = 0.0f;
volatile int32_t g_rawP        = 0;
volatile int32_t g_rawT        = 0;
volatile bool g_sensorOk       = false;

// ==================== 이동평균 필터 ====================
typedef struct
{
  float   buffer[FILTER_SIZE];
  uint8_t index;
  uint8_t count;
  float   sum;
} MovingAvgFilter_t;

MovingAvgFilter_t pressureFilter;
MovingAvgFilter_t temperatureFilter;

void filterInit(MovingAvgFilter_t *f)
{
  for (int i = 0; i < FILTER_SIZE; i++)
  {
    f->buffer[i] = 0.0f;
  }
  f->index = 0;
  f->count = 0;
  f->sum   = 0.0f;
}

float filterUpdate(MovingAvgFilter_t *f, float newValue)
{
  if (f->count >= FILTER_SIZE)
  {
    f->sum -= f->buffer[f->index];
  }
  else
  {
    f->count++;
  }

  f->buffer[f->index] = newValue;
  f->sum += newValue;

  f->index++;
  if (f->index >= FILTER_SIZE)
  {
    f->index = 0;
  }

  return f->sum / (float)f->count;
}

// ==================== 센서 읽기 ====================
bool readSensor24bit(uint8_t regAddr, int32_t *data)
{
  Wire.beginTransmission(SENSOR_I2C_ADDR);
  Wire.write(regAddr);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  uint8_t bytesRead = Wire.requestFrom((uint8_t)SENSOR_I2C_ADDR, (uint8_t)3);
  if (bytesRead != 3)
  {
    return false;
  }

  uint32_t raw = 0;
  raw  = (uint32_t)Wire.read() << 16;
  raw |= (uint32_t)Wire.read() << 8;
  raw |= (uint32_t)Wire.read();

  if (raw & 0x800000)
  {
    *data = (int32_t)raw - 16777216;
  }
  else
  {
    *data = (int32_t)raw;
  }

  return true;
}

float calculatePressure(int32_t rawData)
{
  float fadc = (float)rawData;
  float adc = 3.3f * fadc / 8388608.0f;
  float pressure_kPa = PRESSURE_RANGE * (adc - 0.5f) / 2.0f;
  float pressure_mbar = pressure_kPa * KPA_TO_MBAR;
  return pressure_mbar + PRESSURE_OFFSET;
}

float calculateTemperature(int32_t rawData)
{
  float fadc = (float)rawData;
  float temperature = 25.0f + fadc / 65536.0f;
  return temperature;
}

// ==================== 웹 페이지 HTML ====================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Pressure & Temperature</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{
  font-family:'Segoe UI',system-ui,-apple-system,sans-serif;
  background:linear-gradient(135deg,#0f0c29,#302b63,#24243e);
  min-height:100vh;color:#e0e0e0;
  display:flex;flex-direction:column;align-items:center;
  padding:20px;
}
h1{
  font-size:1.6rem;font-weight:300;letter-spacing:2px;
  margin:10px 0 5px;color:#fff;text-transform:uppercase;
}
.subtitle{font-size:0.8rem;color:#888;margin-bottom:25px;letter-spacing:1px}
.dashboard{
  display:flex;flex-wrap:wrap;justify-content:center;
  gap:30px;max-width:900px;width:100%;
}
.card{
  background:rgba(255,255,255,0.05);
  backdrop-filter:blur(10px);
  border:1px solid rgba(255,255,255,0.08);
  border-radius:20px;padding:30px 25px 20px;
  display:flex;flex-direction:column;align-items:center;
  min-width:300px;flex:1;
  box-shadow:0 8px 32px rgba(0,0,0,0.3);
  transition:transform 0.3s;
}
.card:hover{transform:translateY(-4px)}
.card-title{
  font-size:0.75rem;text-transform:uppercase;letter-spacing:3px;
  color:#aaa;margin-bottom:15px;
}
.value-row{
  display:flex;gap:20px;margin-top:12px;
  font-size:0.75rem;color:#999;
}
.value-row span{display:flex;flex-direction:column;align-items:center}
.value-row .label{font-size:0.6rem;text-transform:uppercase;letter-spacing:1px;margin-bottom:2px}
.value-row .num{font-size:1rem;color:#fff;font-weight:600}
.status-bar{
  margin-top:25px;display:flex;align-items:center;gap:8px;
  font-size:0.7rem;color:#888;
}
.dot{width:8px;height:8px;border-radius:50%;background:#555}
.dot.ok{background:#4ade80;box-shadow:0 0 8px #4ade8066}
.dot.err{background:#f87171;box-shadow:0 0 8px #f8717166}

/* Pressure gauge canvas */
#pressureCanvas{width:250px;height:250px}

/* Thermometer SVG */
.thermo-wrap{position:relative;width:100px;height:250px}
.thermo-wrap svg{width:100%;height:100%}

/* History Table */
.history-card{
  background:rgba(255,255,255,0.05);
  backdrop-filter:blur(10px);
  border:1px solid rgba(255,255,255,0.08);
  border-radius:20px;padding:25px;
  max-width:900px;width:100%;margin-top:30px;
  box-shadow:0 8px 32px rgba(0,0,0,0.3);
}
.history-card .card-title{
  font-size:0.75rem;text-transform:uppercase;letter-spacing:3px;
  color:#aaa;margin-bottom:15px;text-align:center;
}
.history-table{width:100%;border-collapse:collapse}
.history-table th{
  font-size:0.65rem;text-transform:uppercase;letter-spacing:1.5px;
  color:#888;padding:8px 12px;
  border-bottom:1px solid rgba(255,255,255,0.1);
  text-align:center;
}
.history-table td{
  font-size:0.85rem;color:#ddd;padding:7px 12px;
  border-bottom:1px solid rgba(255,255,255,0.04);
  text-align:center;font-variant-numeric:tabular-nums;
}
.history-table tr:first-child td{color:#fff;font-weight:600}
.history-table tr:hover td{background:rgba(255,255,255,0.03)}
.history-table .time-col{color:#999;font-size:0.75rem}
.no-data{text-align:center;color:#666;padding:20px;font-size:0.8rem}

@media(max-width:680px){
  .dashboard{flex-direction:column;align-items:center}
  .card{min-width:unset;width:100%;max-width:360px}
  .history-card{padding:15px}
  .history-table th,.history-table td{padding:5px 6px;font-size:0.75rem}
}
</style>
</head>
<body>
<h1>Pressure & Temperature</h1>
<p class="subtitle">ESP32 WNK Sensor Dashboard</p>

<div class="dashboard">
  <!-- Pressure Card -->
  <div class="card">
    <div class="card-title">Pressure</div>
    <canvas id="pressureCanvas" width="500" height="500"></canvas>
    <div class="value-row">
      <span><span class="label">Current</span><span class="num" id="pCur">--</span></span>
      <span><span class="label">Filtered</span><span class="num" id="pAvg">--</span></span>
      <span><span class="label">Unit</span><span class="num">mbar</span></span>
    </div>
  </div>

  <!-- Temperature Card -->
  <div class="card">
    <div class="card-title">Temperature</div>
    <div class="thermo-wrap">
      <svg id="thermoSvg" viewBox="0 0 100 280"></svg>
    </div>
    <div class="value-row">
      <span><span class="label">Current</span><span class="num" id="tCur">--</span></span>
      <span><span class="label">Filtered</span><span class="num" id="tAvg">--</span></span>
      <span><span class="label">Unit</span><span class="num">&deg;C</span></span>
    </div>
  </div>
</div>

<!-- History Table -->
<div class="history-card">
  <div class="card-title">Measurement History (Latest 10)</div>
  <table class="history-table">
    <thead>
      <tr>
        <th>#</th>
        <th>Time</th>
        <th>Pressure (mbar)</th>
        <th>Pressure Avg</th>
        <th>Temp (&deg;C)</th>
        <th>Temp Avg</th>
      </tr>
    </thead>
    <tbody id="historyBody">
      <tr><td colspan="6" class="no-data">Waiting for data...</td></tr>
    </tbody>
  </table>
</div>

<div class="status-bar">
  <div class="dot" id="statusDot"></div>
  <span id="statusText">Connecting...</span>
  <span style="margin-left:12px" id="updateTime">--</span>
</div>

<script>
// ============ Pressure Gauge (Canvas) ============
const pCanvas = document.getElementById('pressureCanvas');
const pCtx = pCanvas.getContext('2d');
const W = pCanvas.width, H = pCanvas.height;
const cx = W/2, cy = H/2 + 20, R = 190;
const startAngle = 0.75 * Math.PI;
const endAngle   = 2.25 * Math.PI;
const P_MIN = 0, P_MAX = 20000; // mbar

function drawPressureGauge(value, avg) {
  pCtx.clearRect(0, 0, W, H);
  value = Math.max(P_MIN, Math.min(P_MAX, value));

  // Background arc
  pCtx.beginPath();
  pCtx.arc(cx, cy, R, startAngle, endAngle, false);
  pCtx.lineWidth = 30;
  pCtx.strokeStyle = 'rgba(255,255,255,0.06)';
  pCtx.lineCap = 'round';
  pCtx.stroke();

  // Gradient arc (value)
  const ratio = (value - P_MIN) / (P_MAX - P_MIN);
  const valAngle = startAngle + ratio * (endAngle - startAngle);
  const grad = pCtx.createLinearGradient(cx - R, cy, cx + R, cy);
  grad.addColorStop(0, '#06b6d4');
  grad.addColorStop(0.5, '#8b5cf6');
  grad.addColorStop(1, '#f43f5e');
  pCtx.beginPath();
  pCtx.arc(cx, cy, R, startAngle, valAngle, false);
  pCtx.lineWidth = 30;
  pCtx.strokeStyle = grad;
  pCtx.lineCap = 'round';
  pCtx.stroke();

  // Glow effect
  pCtx.beginPath();
  pCtx.arc(cx, cy, R, startAngle, valAngle, false);
  pCtx.lineWidth = 30;
  pCtx.strokeStyle = grad;
  pCtx.shadowColor = '#8b5cf6';
  pCtx.shadowBlur = 20;
  pCtx.stroke();
  pCtx.shadowBlur = 0;

  // Tick marks & labels
  const ticks = [0, 2000, 4000, 6000, 8000, 10000, 12000, 14000, 16000, 18000, 20000];
  ticks.forEach(t => {
    const a = startAngle + (t / P_MAX) * (endAngle - startAngle);
    const cos = Math.cos(a), sin = Math.sin(a);
    const inner = R - 25, outer = R + 8;
    pCtx.beginPath();
    pCtx.moveTo(cx + inner * cos, cy + inner * sin);
    pCtx.lineTo(cx + outer * cos, cy + outer * sin);
    pCtx.strokeStyle = 'rgba(255,255,255,0.2)';
    pCtx.lineWidth = 2;
    pCtx.lineCap = 'butt';
    pCtx.stroke();

    // Label
    const lr = R - 45;
    pCtx.fillStyle = 'rgba(255,255,255,0.4)';
    pCtx.font = '18px sans-serif';
    pCtx.textAlign = 'center';
    pCtx.textBaseline = 'middle';
    pCtx.fillText((t/1000).toFixed(0), cx + lr * cos, cy + lr * sin);
  });

  // Needle
  const needleLen = R - 55;
  const needleAngle = startAngle + ratio * (endAngle - startAngle);
  const nx = cx + needleLen * Math.cos(needleAngle);
  const ny = cy + needleLen * Math.sin(needleAngle);

  // Needle shadow
  pCtx.beginPath();
  pCtx.moveTo(cx, cy);
  pCtx.lineTo(nx, ny);
  pCtx.strokeStyle = 'rgba(255,255,255,0.15)';
  pCtx.lineWidth = 6;
  pCtx.lineCap = 'round';
  pCtx.stroke();

  // Needle line
  pCtx.beginPath();
  pCtx.moveTo(cx, cy);
  pCtx.lineTo(nx, ny);
  pCtx.strokeStyle = '#fff';
  pCtx.lineWidth = 3;
  pCtx.lineCap = 'round';
  pCtx.stroke();

  // Center dot
  pCtx.beginPath();
  pCtx.arc(cx, cy, 10, 0, Math.PI * 2);
  pCtx.fillStyle = '#fff';
  pCtx.fill();
  pCtx.beginPath();
  pCtx.arc(cx, cy, 5, 0, Math.PI * 2);
  pCtx.fillStyle = '#302b63';
  pCtx.fill();

  // Digital value
  pCtx.fillStyle = '#fff';
  pCtx.font = 'bold 42px sans-serif';
  pCtx.textAlign = 'center';
  pCtx.textBaseline = 'middle';
  pCtx.fillText(value.toFixed(1), cx, cy + 65);
  pCtx.fillStyle = 'rgba(255,255,255,0.4)';
  pCtx.font = '16px sans-serif';
  pCtx.fillText('mbar', cx, cy + 92);

  // "x1000" unit hint
  pCtx.fillStyle = 'rgba(255,255,255,0.25)';
  pCtx.font = '13px sans-serif';
  pCtx.fillText('x1000 mbar', cx, cy - R + 60);
}

// ============ Temperature Thermometer (SVG) ============
const thermoSvg = document.getElementById('thermoSvg');
const T_MIN = -10, T_MAX = 60;

function buildThermometer() {
  thermoSvg.innerHTML = `
    <defs>
      <linearGradient id="mercuryGrad" x1="0" y1="1" x2="0" y2="0">
        <stop offset="0%" stop-color="#f43f5e"/>
        <stop offset="100%" stop-color="#fb923c"/>
      </linearGradient>
      <filter id="glow">
        <feGaussianBlur stdDeviation="3" result="blur"/>
        <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
      </filter>
    </defs>
    <!-- Outer tube -->
    <rect x="35" y="20" width="20" height="200" rx="10"
          fill="rgba(255,255,255,0.06)" stroke="rgba(255,255,255,0.12)" stroke-width="1"/>
    <!-- Bulb -->
    <circle cx="45" cy="235" r="22"
            fill="rgba(255,255,255,0.06)" stroke="rgba(255,255,255,0.12)" stroke-width="1"/>
    <!-- Mercury column (dynamic) -->
    <rect id="mercuryCol" x="39" y="200" width="12" height="20" rx="6"
          fill="url(#mercuryGrad)" filter="url(#glow)"/>
    <!-- Mercury bulb -->
    <circle id="mercuryBulb" cx="45" cy="235" r="16"
            fill="url(#mercuryGrad)" filter="url(#glow)"/>
    <!-- Tick marks -->
    ${buildThermoTicks()}
    <!-- Value text -->
    <text id="thermoVal" x="45" y="270" text-anchor="middle"
          fill="#fff" font-size="14" font-weight="bold">--</text>
    <text x="45" y="282" text-anchor="middle"
          fill="rgba(255,255,255,0.4)" font-size="8">&deg;C</text>
  `;
}

function buildThermoTicks() {
  let s = '';
  for (let t = T_MIN; t <= T_MAX; t += 10) {
    const y = 220 - ((t - T_MIN) / (T_MAX - T_MIN)) * 200;
    s += `<line x1="58" y1="${y}" x2="68" y2="${y}" stroke="rgba(255,255,255,0.2)" stroke-width="1"/>`;
    s += `<text x="75" y="${y + 3}" fill="rgba(255,255,255,0.4)" font-size="9">${t}</text>`;
  }
  return s;
}

function updateThermometer(value) {
  value = Math.max(T_MIN, Math.min(T_MAX, value));
  const ratio = (value - T_MIN) / (T_MAX - T_MIN);
  const colHeight = ratio * 200;
  const colY = 220 - colHeight;
  const col = document.getElementById('mercuryCol');
  const val = document.getElementById('thermoVal');
  if (col) {
    col.setAttribute('y', colY);
    col.setAttribute('height', colHeight);
  }
  if (val) val.textContent = value.toFixed(2);
}

buildThermometer();

// ============ History Table ============
const MAX_HISTORY = 10;
const history = [];

function addHistory(d) {
  const now = new Date();
  const entry = {
    time: now.toLocaleTimeString(),
    pressure: d.pressure,
    pressureAvg: d.pressureAvg,
    temperature: d.temperature,
    temperatureAvg: d.temperatureAvg
  };
  history.unshift(entry);
  if (history.length > MAX_HISTORY) history.pop();
  renderHistory();
}

function renderHistory() {
  const tbody = document.getElementById('historyBody');
  if (history.length === 0) {
    tbody.innerHTML = '<tr><td colspan="6" class="no-data">Waiting for data...</td></tr>';
    return;
  }
  tbody.innerHTML = history.map((h, i) =>
    `<tr>
      <td>${i + 1}</td>
      <td class="time-col">${h.time}</td>
      <td>${h.pressure.toFixed(1)}</td>
      <td>${h.pressureAvg.toFixed(1)}</td>
      <td>${h.temperature.toFixed(2)}</td>
      <td>${h.temperatureAvg.toFixed(2)}</td>
    </tr>`
  ).join('');
}

// ============ Data Fetch ============
let animPressure = 0;

function fetchData() {
  fetch('/api/data')
    .then(r => r.json())
    .then(d => {
      animPressure = d.pressureAvg;
      drawPressureGauge(d.pressureAvg, d.pressureAvg);
      updateThermometer(d.temperatureAvg);
      document.getElementById('pCur').textContent = d.pressure.toFixed(1);
      document.getElementById('pAvg').textContent = d.pressureAvg.toFixed(1);
      document.getElementById('tCur').textContent = d.temperature.toFixed(2);
      document.getElementById('tAvg').textContent = d.temperatureAvg.toFixed(2);
      document.getElementById('statusDot').className = 'dot ok';
      document.getElementById('statusText').textContent = 'Sensor Online';
      const now = new Date();
      document.getElementById('updateTime').textContent =
        now.toLocaleTimeString() + ' updated';
      addHistory(d);
    })
    .catch(() => {
      document.getElementById('statusDot').className = 'dot err';
      document.getElementById('statusText').textContent = 'Connection Lost';
    });
}

// Initial draw
drawPressureGauge(0, 0);
updateThermometer(0);

// Fetch immediately then every 5 seconds
fetchData();
setInterval(fetchData, 5000);
</script>
</body>
</html>
)rawliteral";

// ==================== 웹 핸들러 ====================
void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApiData()
{
  char json[256];
  snprintf(json, sizeof(json),
    "{\"pressure\":%.2f,\"pressureAvg\":%.2f,"
    "\"temperature\":%.2f,\"temperatureAvg\":%.2f,"
    "\"rawP\":%ld,\"rawT\":%ld,\"ok\":%s}",
    g_pressure, g_pressureAvg,
    g_temperature, g_temperatureAvg,
    (long)g_rawP, (long)g_rawT,
    g_sensorOk ? "true" : "false"
  );
  server.send(200, "application/json", json);
}

void handleNotFound()
{
  server.send(404, "text/plain", "Not Found");
}

// ==================== Setup ====================
void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  // I2C 초기화
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_FREQ);

  // 필터 초기화
  filterInit(&pressureFilter);
  filterInit(&temperatureFilter);

  Serial.println("================================");
  Serial.println("WNK I2C Pressure Sensor + Web Dashboard");
  Serial.printf("I2C Addr: 0x%02X, Range: %.0f kPa (%.0f mbar)\n",
                SENSOR_I2C_ADDR, PRESSURE_RANGE, PRESSURE_RANGE * KPA_TO_MBAR);
  Serial.printf("Pressure Offset: %.1f mbar\n", PRESSURE_OFFSET);
  Serial.printf("Moving Average Filter: %d samples\n", FILTER_SIZE);
  Serial.println("================================");

  // 센서 연결 확인
  Wire.beginTransmission(SENSOR_I2C_ADDR);
  if (Wire.endTransmission() == 0)
  {
    Serial.println("Sensor connected.");
  }
  else
  {
    Serial.println("ERROR: Sensor not found! Check wiring.");
  }

  // WiFi 연결
  Serial.printf("\nConnecting to WiFi: %s", ssid);
  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40)
  {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(" Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Open browser: http://%s:%d/\n", WiFi.localIP().toString().c_str(), WEB_SERVER_PORT);
  }
  else
  {
    Serial.println("\nWiFi connection failed! Web dashboard unavailable.");
    Serial.println("Continuing with serial output only...");
  }

  // 웹서버 라우팅
  server.on("/", handleRoot);
  server.on("/api/data", handleApiData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.printf("Web server started on port %d\n", WEB_SERVER_PORT);
  Serial.println();
}

// ==================== Loop ====================
void loop()
{
  // 웹서버 클라이언트 처리
  server.handleClient();

  // 센서 읽기
  static unsigned long lastRead = 0;
  unsigned long now = millis();
  if (now - lastRead >= READ_INTERVAL)
  {
    lastRead = now;

    int32_t pressureRaw = 0;
    int32_t temperatureRaw = 0;

    bool pOk = readSensor24bit(REG_PRESSURE, &pressureRaw);
    bool tOk = readSensor24bit(REG_TEMPERATURE, &temperatureRaw);

    if (pOk && tOk)
    {
      float pressureNow    = calculatePressure(pressureRaw);
      float temperatureNow = calculateTemperature(temperatureRaw);

      float pressureAvg    = filterUpdate(&pressureFilter, pressureNow);
      float temperatureAvg = filterUpdate(&temperatureFilter, temperatureNow);

      // 전역 변수 업데이트 (웹 API용)
      g_pressure       = pressureNow;
      g_pressureAvg    = pressureAvg;
      g_temperature    = temperatureNow;
      g_temperatureAvg = temperatureAvg;
      g_rawP           = pressureRaw;
      g_rawT           = temperatureRaw;
      g_sensorOk       = true;

      Serial.printf("P: %8.2f mbar (avg: %8.2f) | T: %6.2f C (avg: %6.2f) | Raw P: %ld, T: %ld\n",
                    pressureNow, pressureAvg, temperatureNow, temperatureAvg,
                    pressureRaw, temperatureRaw);
    }
    else
    {
      g_sensorOk = false;
      Serial.print("Read error -");
      if (!pOk) Serial.print(" Pressure");
      if (!tOk) Serial.print(" Temperature");
      Serial.println();
    }
  }
}
