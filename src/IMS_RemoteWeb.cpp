#include "IMS_RemoteWeb.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer server(80);

static const char* INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>IMS Remote</title></head>"
"<body style='font-family:sans-serif;'>"
"<h2>IMS Remote Control</h2>"
"<button onclick=\"fetch('/api/start',{method:'POST'})\">Start</button>"
"<button onclick=\"fetch('/api/pause',{method:'POST'})\">Pause</button>"
"<button onclick=\"fetch('/api/save',{method:'POST'})\">Save</button>"
"<p id='msg'></p>"
"<script>"
"async function post(p){"
"  let r=await fetch(p,{method:'POST'});"
"  let t=await r.text();"
"  document.getElementById('msg').innerText=t;"
"}"
"</script>"
"</body></html>";

bool IMS_RemoteWeb::beginAP(const char* ssid, const char* pass) {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(ssid, pass);
  if (!ok) return false;

  Serial.print("[AP] SSID: "); Serial.println(ssid);
  Serial.print("[AP] IP: "); Serial.println(WiFi.softAPIP());

  setupRoutes_();
  server.begin();
  Serial.println("[HTTP] Server started");
  return true;
}

void IMS_RemoteWeb::setupRoutes_() {
  // 首页
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
    req->send(200, "text/html", INDEX_HTML);
  });

  // Start
  server.on("/api/start", HTTP_POST, [this](AsyncWebServerRequest* req){
    if (cb_start_) cb_start_();
    req->send(200, "text/plain", "START OK");
  });

  // Pause
  server.on("/api/pause", HTTP_POST, [this](AsyncWebServerRequest* req){
    if (cb_pause_) cb_pause_();
    req->send(200, "text/plain", "PAUSE OK");
  });

  // Save
  server.on("/api/save", HTTP_POST, [this](AsyncWebServerRequest* req){
    if (cb_save_) cb_save_();
    req->send(200, "text/plain", "SAVE OK");
  });
}
