#include "web_server.h"

#if SENSOR_TYPE == 2 || SENSOR_TYPE == 3

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SD_MMC.h>
#include "esp_wifi.h"

static WebServer server(80);
static bool      serverRunning = false;

// ── HTML 페이지 (모바일 최적화) ───────────────────────────────────────────
static const char HTML_HEAD[] PROGMEM =
  "<!DOCTYPE html><html><head>"
  "<meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>Vibration Logger</title>"
  "<style>"
  "*{box-sizing:border-box;margin:0;padding:0}"
  "body{font-family:-apple-system,sans-serif;background:#0d0d1a;color:#e0e0e0;padding:16px}"
  "h1{color:#7eb8f7;font-size:1.15em;margin-bottom:4px}"
  ".sub{font-size:0.75em;color:#888;margin-bottom:16px}"
  ".card{background:#1a1a2e;border-radius:12px;padding:14px;margin-bottom:12px;border:1px solid #2a2a40}"
  ".fname{font-size:0.9em;color:#7eb8f7;margin-bottom:8px;word-break:break-all}"
  "audio{width:100%;height:40px;margin-top:2px}"
  ".log-btn{display:inline-block;margin-top:8px;font-size:0.75em;color:#aaa;text-decoration:none;"
  "background:#0d0d1a;border-radius:6px;padding:4px 8px;border:1px solid #333}"
  ".refresh{display:block;text-align:center;color:#7eb8f7;font-size:0.85em;margin-bottom:16px;"
  "text-decoration:none;border:1px solid #7eb8f7;border-radius:8px;padding:8px}"
  "</style></head><body>"
  "<h1>&#127932; Vibration WAV Files</h1>";

static const char HTML_TAIL[] PROGMEM =
  "<br><a class='refresh' href='/'>&#8635; Refresh</a>"
  "</body></html>";

// ── SD에서 VIBxxxx.wav 파일 목록 수집 ────────────────────────────────────
static int collectWavFiles(String* names, int maxCount)
{
  File root = SD_MMC.open("/");
  if (!root || !root.isDirectory()) return 0;

  int count = 0;
  File f = root.openNextFile();
  while (f && count < maxCount)
  {
    if (!f.isDirectory())
    {
      String n = f.name();
      if (!n.startsWith("/")) n = "/" + n;
      String lower = n; lower.toLowerCase();
      if (lower.startsWith("/vib") && lower.endsWith(".wav"))
        names[count++] = n;
    }
    f = root.openNextFile();
  }
  root.close();

  // 최신 파일이 위로 오도록 내림차순 정렬
  for (int i = 0; i < count - 1; i++)
    for (int j = i + 1; j < count; j++)
      if (names[j] > names[i]) { String t = names[i]; names[i] = names[j]; names[j] = t; }

  return count;
}

// ── GET / : 파일 목록 HTML ────────────────────────────────────────────────
static void handleRoot()
{
  String names[128];
  int total = collectWavFiles(names, 128);
  int count = min(total, 5);  // 최신 5개만 표시

  String html = FPSTR(HTML_HEAD);
  html += "<p class='sub'>최근 ";
  html += String(count);
  html += " / 전체 ";
  html += String(total);
  html += " file(s) &nbsp;|&nbsp; <a style='color:#888' href='http://";
  html += WiFi.localIP().toString();
  html += "'>IP: ";
  html += WiFi.localIP().toString();
  html += "</a></p>";

  for (int i = 0; i < count; i++)
  {
    String wavPath = names[i];
    String logPath = wavPath.substring(0, wavPath.length() - 4) + ".log";

    html += "<div class='card'>";
    html += "<div class='fname'>&#127908; ";
    html += wavPath.substring(1);   // leading / 제거
    html += "</div>";
    html += "<audio controls preload='none' src='";
    html += wavPath;
    html += "'></audio>";
    if (SD_MMC.exists(logPath))
    {
      html += "<a class='log-btn' href='";
      html += logPath;
      html += "'>&#128196; Log</a>";
    }
    html += "</div>";
  }

  html += FPSTR(HTML_TAIL);
  server.send(200, "text/html; charset=utf-8", html);
}

// ── GET /VIBxxxx.wav or /VIBxxxx.log : 파일 스트리밍 ─────────────────────
static void handleFile()
{
  String path = server.uri();
  if (!SD_MMC.exists(path)) { server.send(404, "text/plain", "Not found"); return; }

  File f = SD_MMC.open(path, FILE_READ);
  if (!f) { server.send(500, "text/plain", "Open failed"); return; }

  String lower = path; lower.toLowerCase();
  String mime = "application/octet-stream";
  if (lower.endsWith(".wav"))      mime = "audio/wav";
  else if (lower.endsWith(".log")) mime = "text/plain; charset=utf-8";

  server.streamFile(f, mime);
  f.close();
}

// ── Public API ────────────────────────────────────────────────────────────
void webServerBegin()
{
  if (serverRunning) return;

  // WiFi 드라이버 초기화 상태 확인
  // - ESP_ERR_WIFI_NOT_INIT : 최초 호출 또는 syncNTP 후 → 전체 초기화
  // - ESP_OK                : esp_wifi_stop()만 된 상태 → RF만 재시작
  wifi_mode_t curMode;
  if (esp_wifi_get_mode(&curMode) == ESP_ERR_WIFI_NOT_INIT)
  {
    // 전체 초기화 (최초 or NTP 이후)
    WiFi.mode(WIFI_AP);
  }
  else
  {
    // 드라이버 살아있음 — RF만 재시작 (netif 재등록 없음 → 에러 없음)
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_start();
  }
  delay(200);

  // 빈 문자열이면 오픈 AP, 8자 이상이면 WPA2
  bool apOk = (strlen(AP_PASSWORD) == 0)
    ? WiFi.softAP(AP_SSID)
    : WiFi.softAP(AP_SSID, AP_PASSWORD);

  if (!apOk)
  {
    Serial.println("[AP] softAP() FAILED — password < 8 chars?");
    return;
  }
  delay(100);

  if (MDNS.begin(MDNS_HOSTNAME))
    Serial.printf("[mDNS] http://%s.local (iOS/Mac)\n", MDNS_HOSTNAME);

  server.on("/", handleRoot);
  server.onNotFound(handleFile);
  server.begin();
  serverRunning = true;

  Serial.println("========================================");
  Serial.printf(" WiFi SSID : %s\n", AP_SSID);
  Serial.printf(" Password  : %s\n", strlen(AP_PASSWORD) ? AP_PASSWORD : "(없음 — 오픈)");
  Serial.printf(" 접속 주소 : http://%s/\n", WiFi.softAPIP().toString().c_str());
  Serial.println("========================================");
}

void webServerStop()
{
  if (serverRunning) { server.stop(); serverRunning = false; }
  MDNS.end();
  // softAPdisconnect(false): 클라이언트만 해제, WiFi 모드 유지
  // esp_wifi_stop(): RF 송수신 중단 (ADC 간섭 방지), 드라이버는 초기화 상태 유지
  WiFi.softAPdisconnect(false);
  esp_wifi_stop();
  delay(100);
  Serial.println("[WebServer] Stopped — RF OFF");
}

void webServerLoop()
{
  if (serverRunning) server.handleClient();
}

#endif // SENSOR_TYPE == 2 || SENSOR_TYPE == 3
