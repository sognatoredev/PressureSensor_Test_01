#include "time_manager.h"
#include "config.h"
#include "led_indicator.h"
#include <WiFi.h>
#include "time.h"

static bool    ntpSynced    = false;
static int64_t epochOffsetS = 0;

void syncNTP()
{
#if USE_NTP
  ledSetState(LED_WIFI_CONNECTING);
  Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retry = 0;
  
  while (WiFi.status() != WL_CONNECTED && retry++ < 40)
  {
    for (int i = 0; i < 50; i++) 
    {
      ledUpdate();
      delay(10);
    }
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(" Connected");
    configTime(9 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    struct tm ti;
    if (getLocalTime(&ti, 5000))
    {
      ntpSynced = true;
      time_t now; time(&now);
      epochOffsetS = (int64_t)now - (int64_t)(millis() / 1000);
      Serial.printf("NTP OK: %04d-%02d-%02d %02d:%02d:%02d\n",
        ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday,
        ti.tm_hour, ti.tm_min, ti.tm_sec);
    }
    else
    {
      Serial.println("NTP failed → millis-based time");
    }
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
  }
  else
  {
    Serial.println("\nWiFi failed → millis-based time");
    WiFi.mode(WIFI_OFF);
  }
#endif
}

void getTimeString(char *buf, size_t len)
{
  unsigned long ms = millis();
  if (ntpSynced)
  {
    int64_t    em   = epochOffsetS * 1000LL + (int64_t)ms;
    time_t     es   = (time_t)(em / 1000);
    int        msec = (int)(em % 1000);
    struct tm *ti   = localtime(&es);
    snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
      ti->tm_year+1900, ti->tm_mon+1, ti->tm_mday,
      ti->tm_hour, ti->tm_min, ti->tm_sec, msec);
  }
  else
  {
    unsigned long s = ms/1000, m = s/60, h = m/60;
    snprintf(buf, len, "%02lu:%02lu:%02lu.%03lu", h, m%60, s%60, ms%1000);
  }
}
