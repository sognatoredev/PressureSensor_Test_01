#include "web_server.h"

#if SENSOR_TYPE == 2 || SENSOR_TYPE == 3

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SD_MMC.h>
#include "esp_wifi.h"

static WebServer server(80);
static bool      serverRunning  = false;

// ── 상태 플래그 (wav_recorder ↔ web_server 공유) ─────────────────────────
static volatile bool     g_recordReq   = false;  // REC 버튼 → recorder 소비
static volatile bool     g_stopReq     = false;  // STOP 버튼 → recorder 소비
static volatile bool     g_isRecording = false;  // recorder → UI 전환
static volatile uint32_t g_elapsedSec  = 0;      // 녹음 경과 시간(초) 표시용

// ── CSS 공통 스타일 ───────────────────────────────────────────────────────
static const char HTML_STYLE[] PROGMEM =
  "<!DOCTYPE html><html><head>"
  "<meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>MEMS Mic Logger</title>"
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
  // REC 버튼 (대기 상태)
  ".rec-btn{display:block;text-align:center;background:#c0392b;color:#fff;"
  "font-size:1.1em;font-weight:bold;border:none;border-radius:10px;"
  "padding:16px;margin-bottom:14px;width:100%;cursor:pointer;letter-spacing:1px;"
  "text-decoration:none}"
  ".rec-btn:active{background:#922b21}"
  // STOP 버튼 (녹음 중 상태)
  ".stop-btn{display:block;text-align:center;background:#e67e22;color:#fff;"
  "font-size:1.1em;font-weight:bold;border:none;border-radius:10px;"
  "padding:16px;margin-bottom:14px;width:100%;cursor:pointer;letter-spacing:1px;"
  "text-decoration:none;animation:pulse 1s infinite}"
  "@keyframes pulse{0%,100%{opacity:1}50%{opacity:.6}}"
  // 녹음 중 상태 표시
  ".rec-status{background:#1a1a2e;border:1px solid #e67e22;border-radius:12px;"
  "padding:18px;margin-bottom:14px;text-align:center}"
  ".rec-time{font-size:2.2em;font-weight:bold;color:#e67e22;letter-spacing:2px;margin:8px 0}"
  ".rec-label{font-size:0.8em;color:#aaa}"
  // 전체 삭제 버튼
  ".del-btn{display:block;text-align:center;background:#2c2c3e;color:#e74c3c;"
  "font-size:0.85em;font-weight:bold;border:1px solid #e74c3c;border-radius:8px;"
  "padding:10px;margin-top:10px;width:100%;cursor:pointer;letter-spacing:1px;"
  "text-decoration:none}"
  ".del-btn:active{background:#e74c3c;color:#fff}"
  // 결과 메시지 박스
  ".msg-ok{background:#1a2e1a;border:1px solid #27ae60;border-radius:10px;"
  "padding:14px;margin-bottom:14px;color:#2ecc71;font-size:0.9em;text-align:center}"
  ".msg-err{background:#2e1a1a;border:1px solid #e74c3c;border-radius:10px;"
  "padding:14px;margin-bottom:14px;color:#e74c3c;font-size:0.9em;text-align:center}"
  "</style></head><body>";

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

// ── 경과 시간 포맷 (초 → "MM:SS" or "HH:MM:SS") ─────────────────────────
static String formatElapsed(uint32_t sec)
{
  char buf[12];
  uint32_t h = sec / 3600;
  uint32_t m = (sec % 3600) / 60;
  uint32_t s = sec % 60;
  if (h > 0)
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", h, m, s);
  else
    snprintf(buf, sizeof(buf), "%02u:%02u", m, s);
  return String(buf);
}

// ── GET / : 메인 페이지 ───────────────────────────────────────────────────
static void handleRoot()
{
  String html = FPSTR(HTML_STYLE);
  html += "<h1>&#127908; MEMS Mic Logger</h1>";

  if (g_isRecording)
  {
    // ── 녹음 중 UI ──────────────────────────────────────────────────────
    html += "<div class='rec-status'>";
    html += "<div class='rec-label'>&#128308; REC</div>";
    html += "<div class='rec-time' id='rt'>";
    html += formatElapsed(g_elapsedSec);
    html += "</div>";
    html += "<div class='rec-label'>녹음 중 — STOP을 눌러 저장</div>";
    html += "</div>";

    html += "<a class='stop-btn' href='/stop'>&#9632;&nbsp; STOP</a>";

    // JS: 1초마다 경과 시간 카운트업 (서버 재요청 없이 로컬 갱신)
    html += "<script>"
            "var s=";
    html += String(g_elapsedSec);
    html += ";"
            "var el=document.getElementById('rt');"
            "setInterval(function(){"
              "s++;"
              "var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;"
              "el.textContent=(h>0?(h<10?'0'+h:h)+':':'')"
                "+(m<10?'0'+m:m)+':'+(sec<10?'0'+sec:sec);"
            "},1000);"
            "</script>";

    // 녹음 중에는 자동 새로고침 (30초마다, STOP 누른 경우 대비)
    html += "<meta http-equiv='refresh' content='30'>";
  }
  else
  {
    // ── 대기 중 UI ──────────────────────────────────────────────────────
    html += "<a class='rec-btn' href='/record'>&#9679;&nbsp; REC</a>";

    String names[128];
    int total = collectWavFiles(names, 128);
    int count = min(total, 5);  // 최신 5개만 표시

    html += "<p class='sub'>최근 ";
    html += String(count);
    html += " / 전체 ";
    html += String(total);
    html += " file(s)</p>";

    for (int i = 0; i < count; i++)
    {
      String wavPath = names[i];
      String logPath = wavPath.substring(0, wavPath.length() - 4) + ".log";

      html += "<div class='card'>";
      html += "<div class='fname'>&#127908; ";
      html += wavPath.substring(1);  // leading / 제거
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

    html += "<br><a class='refresh' href='/'>&#8635; Refresh</a>";

    // 전체 삭제 버튼 (파일이 1개 이상일 때만 표시)
    if (total > 0)
    {
      html += "<a class='del-btn' href='/delete_all'"
              " onclick=\"return confirm('SD에 저장된 WAV·LOG 파일 ";
      html += String(total);
      html += "개를 모두 삭제합니다.\\n\\n이 작업은 되돌릴 수 없습니다.\\n계속하시겠습니까?')\">"
              "&#128465;&nbsp; 전체 삭제 (";
      html += String(total);
      html += "개)</a>";
    }
  }

  html += "</body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

// ── GET /record : REC 버튼 ────────────────────────────────────────────────
static void handleRecord()
{
  if (!g_isRecording)
    g_recordReq = true;
  server.sendHeader("Location", "/");
  server.send(303);
}

// ── GET /stop : STOP 버튼 ─────────────────────────────────────────────────
static void handleStop()
{
  if (g_isRecording)
    g_stopReq = true;
  server.sendHeader("Location", "/");
  server.send(303);
}

// ── GET /status : JSON 상태 (폴링용) ──────────────────────────────────────
static void handleStatus()
{
  char buf[64];
  snprintf(buf, sizeof(buf),
    "{\"recording\":%s,\"elapsed\":%lu}",
    g_isRecording ? "true" : "false",
    (unsigned long)g_elapsedSec);
  server.send(200, "application/json", buf);
}

// ── GET /delete_all : SD의 VIB*.wav / VIB*.log 전체 삭제 ─────────────────
static void handleDeleteAll()
{
  // 녹음 중에는 삭제 금지
  if (g_isRecording)
  {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  // SD 루트에서 VIB로 시작하는 파일 모두 수집
  File root = SD_MMC.open("/");
  if (!root || !root.isDirectory())
  {
    server.send(500, "text/plain", "SD open failed");
    return;
  }

  // 삭제 대상 파일명 수집 (열려 있는 디렉터리 반복자와 remove()를 동시에 쓰면 안전하지 않으므로 먼저 목록화)
  const int MAX_DEL = 256;
  String    delList[MAX_DEL];
  int       delCount = 0;

  File f = root.openNextFile();
  while (f && delCount < MAX_DEL)
  {
    if (!f.isDirectory())
    {
      String n = f.name();
      if (!n.startsWith("/")) n = "/" + n;
      String lower = n; lower.toLowerCase();
      if (lower.startsWith("/vib") &&
          (lower.endsWith(".wav") || lower.endsWith(".log")))
      {
        delList[delCount++] = n;
      }
    }
    f = root.openNextFile();
  }
  root.close();

  // 수집된 목록 순서대로 삭제
  int okCount  = 0;
  int errCount = 0;
  for (int i = 0; i < delCount; i++)
  {
    if (SD_MMC.remove(delList[i]))
    {
      Serial.printf("[DEL] %s OK\n", delList[i].c_str());
      okCount++;
    }
    else
    {
      Serial.printf("[DEL] %s FAILED\n", delList[i].c_str());
      errCount++;
    }
  }

  Serial.printf("[DEL] Done: deleted %d / failed %d\n", okCount, errCount);

  // 결과 페이지 반환
  String html = FPSTR(HTML_STYLE);
  html += "<h1>&#127908; MEMS Mic Logger</h1>";

  if (delCount == 0)
  {
    html += "<div class='msg-ok'>삭제할 파일이 없습니다.</div>";
  }
  else if (errCount == 0)
  {
    html += "<div class='msg-ok'>&#10003;&nbsp; ";
    html += String(okCount);
    html += "개 파일을 모두 삭제했습니다.</div>";
  }
  else
  {
    html += "<div class='msg-err'>&#9888;&nbsp; 삭제 완료 ";
    html += String(okCount);
    html += "개 / 실패 ";
    html += String(errCount);
    html += "개</div>";
  }

  html += "<a class='refresh' href='/'>&#8592; 메인으로</a>";
  html += "</body></html>";
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

  wifi_mode_t curMode;
  if (esp_wifi_get_mode(&curMode) == ESP_ERR_WIFI_NOT_INIT)
  {
    WiFi.mode(WIFI_AP);
  }
  else
  {
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_start();
  }
  delay(200);

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

  server.on("/",          handleRoot);
  server.on("/record",    handleRecord);
  server.on("/stop",      handleStop);
  server.on("/status",    handleStatus);
  server.on("/delete_all",handleDeleteAll);
  server.onNotFound(handleFile);
  server.begin();
  serverRunning = true;

  Serial.println("========================================");
  Serial.printf(" WiFi SSID : %s\n", AP_SSID);
  Serial.printf(" Password  : %s\n", strlen(AP_PASSWORD) ? AP_PASSWORD : "(none — open)");
  Serial.printf(" URL       : http://%s/\n", WiFi.softAPIP().toString().c_str());
  Serial.println("========================================");
}

void webServerStop()
{
  if (serverRunning) { server.stop(); serverRunning = false; }
  MDNS.end();
  WiFi.softAPdisconnect(false);
  esp_wifi_stop();
  btStop();
  delay(500);
  Serial.println("[WebServer] Stopped — WiFi/BT RF OFF");
}

void webServerLoop()
{
  if (serverRunning) server.handleClient();
}

bool webRecordConsumed()
{
  if (!g_recordReq) return false;
  g_recordReq = false;
  return true;
}

bool webStopConsumed()
{
  if (!g_stopReq) return false;
  g_stopReq = false;
  return true;
}

void webSetRecording(bool isRecording)
{
  g_isRecording = isRecording;
  if (!isRecording) g_elapsedSec = 0;
}

void webSetElapsed(uint32_t seconds)
{
  g_elapsedSec = seconds;
}

#endif // SENSOR_TYPE == 2 || SENSOR_TYPE == 3
