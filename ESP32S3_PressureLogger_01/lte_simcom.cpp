/**
 * lte_simcom.cpp — SIMCOM SIM7000 LTE Cat.M1 드라이버
 *
 * 이전 개발자 루틴 기반 (modem.c 참조):
 *   AT+CGREG?/AT+CREG?  → 네트워크 등록 대기 (stat=5 로밍)
 *   AT+CGATT?           → GPRS 연결 확인
 *   AT+SAPBR=3,1,"apn","em" / AT+SAPBR=1,1  → Bearer 설정·활성화
 *   AT+CIPSHUT          → 이전 TCP 세션 초기화
 *   AT+CSTT="em"        → APN 설정 (TCPIP 스택)
 *   AT+CIICR            → 무선 연결 활성화
 *   AT+CIFSR            → IP 확인
 *   AT+CIPSTART="TCP","host","port" → TCP 연결
 *   AT+CIPSEND          → raw HTTP 전송 (Ctrl+Z 종료)
 *   AT+CIPCLOSE / AT+CIPSHUT → 연결 종료
 */

#include "lte_simcom.h"

#if USE_LTE

#include <Arduino.h>
#include <HardwareSerial.h>

static HardwareSerial lteSerial(1);  // UART1
static String g_resp;

// ── 내부 유틸리티 ────────────────────────────────────────────────────────────

static bool lteAt(const char* cmd, const char* expect, uint32_t timeoutMs = 5000)
{
    delay(20);
    while (lteSerial.available()) lteSerial.read();

    if (cmd && cmd[0]) {
        Serial.printf("[LTE] >> %s\n", cmd);
        lteSerial.println(cmd);
    }

    g_resp = "";
    g_resp.reserve(512);

    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        while (lteSerial.available()) g_resp += (char)lteSerial.read();
        if (g_resp.indexOf(expect) >= 0) {
            delay(50);
            while (lteSerial.available()) g_resp += (char)lteSerial.read();
            Serial.printf("[LTE] << %s\n", g_resp.c_str());
            return true;
        }
        if (g_resp.indexOf("ERROR") >= 0) {
            Serial.printf("[LTE] << ERR: %s\n", g_resp.c_str());
            return false;
        }
    }
    Serial.printf("[LTE] TIMEOUT (wanted: \"%s\")\n", expect);
    if (g_resp.length() > 0) Serial.printf("[LTE] Got: %s\n", g_resp.c_str());
    return false;
}

static void ltePowerKeyPulse(uint32_t holdMs)
{
    digitalWrite(LTE_PWRKEY_PIN, LOW);
    delay(holdMs);
    digitalWrite(LTE_PWRKEY_PIN, HIGH);
}

// ── 공개 API ─────────────────────────────────────────────────────────────────

bool lteInit()
{
    pinMode(LTE_PWRKEY_PIN, OUTPUT);
    digitalWrite(LTE_PWRKEY_PIN, HIGH);
    lteSerial.begin(LTE_BAUD, SERIAL_8N1, LTE_RX_PIN, LTE_TX_PIN);

    Serial.println("[LTE] ── SIM7000 Init ──────────────────────");

    // 1. 모뎀 생존 확인
    Serial.println("[LTE] Checking modem...");
    bool alive = false;
    for (int i = 0; i < 3; i++) {
        if (lteAt("AT", "OK", 1500)) { alive = true; break; }
    }
    if (!alive) {
        Serial.println("[LTE] Powering on modem (PWRKEY 1000ms)...");
        ltePowerKeyPulse(1000);
        delay(5000);
        for (int i = 0; i < 10; i++) {
            if (lteAt("AT", "OK", 2000)) { alive = true; break; }
            delay(500);
        }
    }
    if (!alive) { Serial.println("[LTE] Modem not responding"); return false; }
    Serial.println("[LTE] Modem alive OK");

    // 2. 에코 OFF
    lteAt("ATE0", "OK", 2000);

    // 3. SIM 확인
    if (!lteAt("AT+CPIN?", "READY", 5000)) {
        Serial.println("[LTE] SIM not ready");
        return false;
    }

    // 4. 네트워크 등록 대기 — CGREG(GPRS) / CREG(GSM) 교번 확인
    //    stat=5 → 로밍 등록 (emnify / SKTelecom)
    Serial.println("[LTE] Waiting for network registration...");
    bool registered = false;
    for (int i = 0; i < 60 && !registered; i++) {
        // 홀수=CGREG, 짝수=CREG 교번
        if (i % 2 == 0) lteAt("AT+CGREG?", "+CGREG:", 3000);
        else             lteAt("AT+CREG?",  "+CREG:",  3000);

        if (g_resp.indexOf(",1") >= 0 || g_resp.indexOf(",5") >= 0)
            registered = true;
        else
            delay(1000);
    }
    if (!registered) { Serial.println("[LTE] Network registration failed"); return false; }
    Serial.println("[LTE] Network registered OK");

    // 5. GPRS 연결 확인
    Serial.println("[LTE] Checking GPRS attachment...");
    bool attached = false;
    for (int i = 0; i < 10; i++) {
        lteAt("AT+CGATT?", "+CGATT:", 3000);
        if (g_resp.indexOf("+CGATT: 1") >= 0) { attached = true; break; }
        delay(2000);
    }
    if (!attached) { Serial.println("[LTE] GPRS not attached"); return false; }
    Serial.println("[LTE] GPRS attached OK");

    // 6. Bearer 프로파일 설정 및 활성화 (SAPBR)
    lteAt("AT+SAPBR=3,1,\"apn\",\"" LTE_APN "\"", "OK", 5000);
    lteAt("AT+SAPBR=1,1", "OK", 10000);

    // 7. TCPIP 스택 초기화
    lteAt("AT+CIPSHUT", "SHUT OK", 5000);

    // 8. APN 설정 (TCPIP 스택)
    if (!lteAt("AT+CSTT=\"" LTE_APN "\"", "OK", 5000)) {
        Serial.println("[LTE] CSTT failed");
        return false;
    }

    // 9. 무선 연결 활성화
    if (!lteAt("AT+CIICR", "OK", 10000)) {
        Serial.println("[LTE] CIICR failed");
        return false;
    }

    // 10. IP 주소 확인 (CIFSR는 OK 없이 IP 직접 반환)
    delay(20);
    while (lteSerial.available()) lteSerial.read();
    lteSerial.println("AT+CIFSR");
    g_resp = "";
    uint32_t t0 = millis();
    while (millis() - t0 < 5000) {
        while (lteSerial.available()) g_resp += (char)lteSerial.read();
        // IP 주소 형식: 숫자.숫자.숫자.숫자
        if (g_resp.length() > 6 && g_resp.indexOf('.') >= 0) {
            delay(50);
            while (lteSerial.available()) g_resp += (char)lteSerial.read();
            break;
        }
    }
    g_resp.trim();
    if (g_resp.length() == 0 || g_resp.indexOf("ERROR") >= 0) {
        Serial.println("[LTE] CIFSR failed — no IP");
        return false;
    }
    Serial.printf("[LTE] >> AT+CIFSR\n[LTE] << %s\n", g_resp.c_str());
    Serial.printf("[LTE] IP: %s\n", g_resp.c_str());

    Serial.println("[LTE] Init OK");
    Serial.println("[LTE] ──────────────────────────────────────");
    return true;
}

bool lteHttpGet(const char* path, char* respBuf, int respBufLen)
{
    char cmd[256];

    Serial.println("[LTE] ── HTTP GET ──────────────────────────");
    Serial.printf("[LTE] Server : http://%s:%d%s\n",
                  LTE_SERVER_HOST, LTE_SERVER_PORT, path);

    // 1. TCP 연결
    snprintf(cmd, sizeof(cmd),
             "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"",
             LTE_SERVER_HOST, LTE_SERVER_PORT);
    if (!lteAt(cmd, "CONNECT OK", 30000)) {
        // "ALREADY CONNECT" → 이미 연결된 경우 허용
        if (g_resp.indexOf("ALREADY CONNECT") < 0) {
            Serial.println("[LTE] CIPSTART failed");
            return false;
        }
        Serial.println("[LTE] Already connected");
    }
    Serial.println("[LTE] TCP connected OK");

    // 2. HTTP GET 요청 전송
    //    AT+CIPSEND → '>' 프롬프트 → 데이터 + 0x1A(Ctrl+Z)
    char httpReq[256];
    int reqLen = snprintf(httpReq, sizeof(httpReq),
                          "GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n",
                          path, LTE_SERVER_HOST);

    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", reqLen);
    if (!lteAt(cmd, ">", 10000)) {
        Serial.println("[LTE] CIPSEND prompt failed");
        lteAt("AT+CIPCLOSE", "CLOSE OK", 5000);
        return false;
    }

    // '>' 수신 후 데이터 전송
    lteSerial.print(httpReq);
    Serial.printf("[LTE] Sent: %s", httpReq);

    // 3. 응답 수신 (SEND OK + HTTP 응답)
    Serial.println("[LTE] Waiting for response...");
    g_resp = "";
    g_resp.reserve(1024);
    uint32_t t0 = millis();
    bool sendOk = false;
    while (millis() - t0 < 30000) {
        while (lteSerial.available()) g_resp += (char)lteSerial.read();
        if (!sendOk && g_resp.indexOf("SEND OK") >= 0) sendOk = true;
        // HTTP 응답 끝 감지: 빈 줄(\r\n\r\n) 이후 본문 포함 or CLOSED
        if ((sendOk && g_resp.indexOf("\r\n\r\n") >= 0) ||
             g_resp.indexOf("CLOSED") >= 0) {
            delay(200);
            while (lteSerial.available()) g_resp += (char)lteSerial.read();
            break;
        }
    }
    Serial.printf("[LTE] Response:\n%s\n", g_resp.c_str());

    // 4. HTTP 상태코드 파싱
    bool success = false;
    int httpIdx = g_resp.indexOf("HTTP/");
    if (httpIdx >= 0) {
        int spIdx = g_resp.indexOf(' ', httpIdx);
        if (spIdx >= 0) {
            int statusCode = atoi(g_resp.c_str() + spIdx + 1);
            Serial.printf("[LTE] HTTP Status: %d\n", statusCode);
            success = (statusCode >= 200 && statusCode < 300);
        }
    }

    // 5. 본문 추출
    int bodyStart = g_resp.indexOf("\r\n\r\n");
    if (bodyStart >= 0 && respBuf && respBufLen > 1) {
        String body = g_resp.substring(bodyStart + 4);
        body.trim();
        strncpy(respBuf, body.c_str(), respBufLen - 1);
        respBuf[respBufLen - 1] = '\0';
    }

    // 6. 연결 종료
    lteAt("AT+CIPCLOSE", "CLOSE OK", 5000);
    lteAt("AT+CIPSHUT",  "SHUT OK",  5000);

    Serial.printf("[LTE] HTTP GET %s\n", success ? "SUCCESS" : "FAILED");
    Serial.println("[LTE] ──────────────────────────────────────");
    return success;
}

// ── AT 패스스루 ──────────────────────────────────────────────────────────────

void ltePassthroughBegin()
{
    pinMode(LTE_PWRKEY_PIN, OUTPUT);
    digitalWrite(LTE_PWRKEY_PIN, HIGH);
    lteSerial.begin(LTE_BAUD, SERIAL_8N1, LTE_RX_PIN, LTE_TX_PIN);

    Serial.println("========================================");
    Serial.println("  LTE AT Passthrough Mode");
    Serial.printf ("  UART1  TX=GPIO%d  RX=GPIO%d  %dbaud\n",
                   LTE_TX_PIN, LTE_RX_PIN, LTE_BAUD);
    Serial.println("  Serial Monitor: 115200, Both NL & CR");
    Serial.println("  Type AT commands and press Enter.");
    Serial.println("  'PWRON'  → PWRKEY pulse (1000ms)");
    Serial.println("========================================");
}

void ltePassthroughLoop()
{
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return;

        if (line.equalsIgnoreCase("PWRON")) {
            Serial.println("[PT] PWRKEY pulse 1000ms...");
            digitalWrite(LTE_PWRKEY_PIN, LOW);
            delay(1000);
            digitalWrite(LTE_PWRKEY_PIN, HIGH);
            Serial.println("[PT] PWRKEY released. Wait ~5s for modem boot.");
            return;
        }

        Serial.print("[TX] ");
        Serial.println(line);
        lteSerial.println(line);
    }

    while (lteSerial.available()) {
        Serial.write(lteSerial.read());
    }
}

#endif // USE_LTE
