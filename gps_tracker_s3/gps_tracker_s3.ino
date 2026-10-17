/*
 * ESP32-S3 + NEO-6M GPS  ->  Live map (Leaflet + OpenStreetMap)
 *
 * The board joins your WiFi and hosts a web page. Open the IP it prints
 * on the Serial Monitor in any browser on the same network. The page shows
 * a map of India and drops/moves a pin at your live GPS location.
 *
 * Wiring:
 *   NEO-6M VCC -> 3V3
 *   NEO-6M GND -> GND
 *   NEO-6M TX  -> D7 / GPIO44 (ESP RX)
 *   NEO-6M RX  -> D6 / GPIO43 (ESP TX)
 *
 * Libraries: TinyGPSPlus
 * Board core: esp32 by Espressif
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ---- Credentials live in secrets.h (git-ignored). ----
// Copy secrets.h.example to secrets.h and fill in your own values.
#include "secrets.h"
const uint32_t TG_REPEAT_MS = 30000;   // keep alerting every 30s while outside
// ------------------------------------------------------

// Geofence (set from the web panel, enforced here on the device)
bool   g_fenceSet = false;
double g_fenceLat = 0, g_fenceLng = 0, g_fenceRadius = 0;
bool   g_wasOutside = false;
uint32_t g_lastAlert = 0;

// Telegram incoming-message polling
long     g_tgOffset = 0;        // next update id to fetch
bool     g_tgPrimed = false;    // skip replying to backlog on first poll
uint32_t g_lastPollTg = 0;
const uint32_t TG_POLL_MS = 3000;   // check for new commands every 3s

// GPS on Serial1 (hardware UART). RX pin receives GPS TX.
static const int GPS_RX_PIN = 44;  // D7  ESP RX  <- GPS TX
static const int GPS_TX_PIN = 43;  // D6  ESP TX  -> GPS RX
static const uint32_t GPS_BAUD = 9600;

// Status LEDs (external, through 220 ohm to GND)
static const int LED_GREEN = 1;   // D0  - WiFi connected
static const int LED_RED   = 2;   // D1  - searching / disconnected
#ifdef LED_BUILTIN
static const int LED_ONBOARD = LED_BUILTIN;  // XIAO onboard LED (active LOW)
#else
static const int LED_ONBOARD = 21;
#endif

TinyGPSPlus gps;
WebServer server(80);

// Latest fix
double g_lat = 20.5937;   // default: center of India
double g_lng = 78.9629;
bool   g_hasFix = false;
double g_altitude = 0.0;
double g_speedKmph = 0.0;
uint32_t g_satellites = 0;
char   g_timeUtc[32] = "--:--:--";

// --- Web page: Leaflet map that polls /data every 2s ---
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ESP32-S3 GPS Tracker</title>
  <link rel="stylesheet"
        href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
        integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY="
        crossorigin="" />
  <style>
    :root { color-scheme: light dark; }
    html, body { margin: 0; height: 100%; font-family: system-ui, sans-serif; }
    #map { position: absolute; top: 0; bottom: 0; left: 0; right: 0; }
    #hud {
      position: absolute; z-index: 1000; top: 10px; left: 10px;
      background: rgba(20,20,25,.82); color: #fff; padding: 10px 14px;
      border-radius: 10px; font-size: 13px; line-height: 1.5;
      box-shadow: 0 2px 10px rgba(0,0,0,.3); max-width: 70vw;
    }
    #hud b { color: #6cf; }
    .nofix { color: #f88; }
    .fix   { color: #8f8; }
    #hud hr { border: none; border-top: 1px solid rgba(255,255,255,.2); margin: 8px 0; }
    .fence-row { margin: 4px 0; display: flex; align-items: center; gap: 6px; }
    #hud button {
      background: #2a6cf0; color: #fff; border: none; border-radius: 6px;
      padding: 4px 9px; font-size: 12px; cursor: pointer;
    }
    #hud button.active { background: #e0a100; }
    #hud button.clear { background: #555; }
    #hud input[type=range] { width: 110px; }
    .inside  { color: #8f8; }
    .outside { color: #ff6b6b; font-weight: bold; }
    #alert {
      display: none; position: absolute; z-index: 1200; top: 14px;
      left: 50%; transform: translateX(-50%);
      background: #d32f2f; color: #fff; padding: 12px 22px;
      border-radius: 10px; font-size: 16px; font-weight: bold;
      box-shadow: 0 4px 16px rgba(0,0,0,.4);
    }
    #alert.show { display: block; animation: pulse 1s infinite; }
    @keyframes pulse { 0%,100% { opacity: 1; } 50% { opacity: .65; } }
  </style>
</head>
<body>
  <div id="map"></div>
  <div id="alert">Device is OUTSIDE the geofence!</div>
  <div id="hud">
    <div><b>ESP32-S3 GPS Tracker</b></div>
    <div id="status" class="nofix">Waiting for GPS fix…</div>
    <div id="coords">Lat: --, Lng: --</div>
    <div id="meta">Sats: -- · Alt: -- m · Spd: -- km/h</div>
    <div id="utc">UTC: --:--:--</div>
    <hr>
    <div class="fence-row">
      <button id="setFence">Set geofence</button>
      <button id="clearFence" class="clear">Clear</button>
    </div>
    <div class="fence-row">
      Radius: <input id="radius" type="range" min="20" max="5000" step="10" value="300">
      <span id="radiusVal">300</span> m
    </div>
    <div id="fenceState">Geofence: not set (click "Set geofence", then click the map)</div>
  </div>

  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
          integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
          crossorigin=""></script>
  <script>
    // Full world map
    const map = L.map('map', {
      center: [20.0, 0.0],
      zoom: 2,
      minZoom: 2,
      maxZoom: 19,
      zoomControl: true,
      scrollWheelZoom: true,
      worldCopyJump: true
    });

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    // Red location dot
    let dot = null, firstFix = true;

    // ---------- Geofence ----------
    let fenceCenter = null;     // L.latLng
    let fenceRadius = 300;      // metres
    let fenceCircle = null;
    let arming = false;
    let wasOutside = false;     // for edge-triggered alerts

    const setBtn   = document.getElementById('setFence');
    const clearBtn = document.getElementById('clearFence');
    const radEl    = document.getElementById('radius');
    const radVal   = document.getElementById('radiusVal');
    const fState   = document.getElementById('fenceState');
    const alertBox = document.getElementById('alert');

    // Restore a saved fence
    try {
      const s = JSON.parse(localStorage.getItem('geofence') || 'null');
      if (s) { fenceRadius = s.r; radEl.value = s.r; radVal.textContent = s.r;
               setFence(L.latLng(s.lat, s.lng), s.r); }
    } catch (e) {}

    function saveFence() {
      if (fenceCenter) localStorage.setItem('geofence',
        JSON.stringify({ lat: fenceCenter.lat, lng: fenceCenter.lng, r: fenceRadius }));
    }

    function pushFence() {   // tell the ESP32 so it alerts even if browser is closed
      if (!fenceCenter) return;
      fetch('/setfence?lat=' + fenceCenter.lat + '&lng=' + fenceCenter.lng +
            '&r=' + fenceRadius).catch(() => {});
    }

    function setFence(latlng, r) {
      fenceCenter = latlng; fenceRadius = r;
      if (fenceCircle) map.removeLayer(fenceCircle);
      fenceCircle = L.circle(latlng, {
        radius: r, color: '#2a6cf0', weight: 2,
        fillColor: '#2a6cf0', fillOpacity: 0.12
      }).addTo(map);
      fState.textContent = 'Geofence set · radius ' + r + ' m';
      saveFence();
      pushFence();
    }

    setBtn.onclick = () => {
      arming = !arming;
      setBtn.classList.toggle('active', arming);
      setBtn.textContent = arming ? 'Click map…' : 'Set geofence';
      if (arming && 'Notification' in window && Notification.permission === 'default') {
        Notification.requestPermission();
      }
    };

    clearBtn.onclick = () => {
      if (fenceCircle) map.removeLayer(fenceCircle);
      fenceCircle = null; fenceCenter = null; wasOutside = false;
      localStorage.removeItem('geofence');
      fState.textContent = 'Geofence: not set';
      alertBox.classList.remove('show');
      fetch('/clearfence').catch(() => {});
    };

    radEl.oninput = () => {
      fenceRadius = parseInt(radEl.value, 10);
      radVal.textContent = fenceRadius;
      if (fenceCircle) { fenceCircle.setRadius(fenceRadius); saveFence(); pushFence();
        fState.textContent = 'Geofence set · radius ' + fenceRadius + ' m'; }
    };

    map.on('click', (e) => {
      if (!arming) return;
      arming = false;
      setBtn.classList.remove('active');
      setBtn.textContent = 'Set geofence';
      setFence(e.latlng, fenceRadius);
    });

    function beep() {
      try {
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        const o = ctx.createOscillator(), g = ctx.createGain();
        o.type = 'square'; o.frequency.value = 880;
        o.connect(g); g.connect(ctx.destination); g.gain.value = 0.2;
        o.start(); setTimeout(() => { o.stop(); ctx.close(); }, 400);
      } catch (e) {}
    }

    function checkFence(lat, lng) {
      if (!fenceCenter) { alertBox.classList.remove('show'); return; }
      const dist = map.distance(fenceCenter, L.latLng(lat, lng)); // metres
      const outside = dist > fenceRadius;
      const over = Math.max(0, dist - fenceRadius);
      fState.innerHTML = 'Geofence · r=' + fenceRadius + ' m · dist=' +
        dist.toFixed(0) + ' m · <span class="' + (outside ? 'outside' : 'inside') +
        '">' + (outside ? 'OUTSIDE (+' + over.toFixed(0) + ' m)' : 'inside') + '</span>';
      alertBox.classList.toggle('show', outside);
      if (outside && !wasOutside) {   // edge: just left the fence
        beep();
        if ('Notification' in window && Notification.permission === 'granted') {
          new Notification('Geofence breach', {
            body: 'Device left the area (' + over.toFixed(0) + ' m beyond the boundary).'
          });
        }
      }
      wasOutside = outside;
    }
    // ------------------------------

    async function poll() {
      try {
        const r = await fetch('/data', { cache: 'no-store' });
        const d = await r.json();

        document.getElementById('coords').textContent =
          'Lat: ' + d.lat.toFixed(6) + ', Lng: ' + d.lng.toFixed(6);
        document.getElementById('meta').textContent =
          'Sats: ' + d.sats + ' · Alt: ' + d.alt.toFixed(1) +
          ' m · Spd: ' + d.spd.toFixed(1) + ' km/h';
        document.getElementById('utc').textContent = 'UTC: ' + d.utc;

        const st = document.getElementById('status');
        if (d.fix) {
          st.textContent = 'GPS FIX'; st.className = 'fix';
          if (!dot) {
            dot = L.circleMarker([d.lat, d.lng], {
              radius: 8, color: '#fff', weight: 2,
              fillColor: '#e53935', fillOpacity: 1
            }).addTo(map).bindPopup('You are here');
          } else {
            dot.setLatLng([d.lat, d.lng]);
          }
          if (firstFix) { map.setView([d.lat, d.lng], 17); firstFix = false; }
          checkFence(d.lat, d.lng);
        } else {
          st.textContent = 'Waiting for GPS fix…'; st.className = 'nofix';
        }
      } catch (e) {
        document.getElementById('status').textContent = 'Connection lost…';
      }
    }
    poll();
    setInterval(poll, 1000);
  </script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// Minimal URL-encoder for the Telegram message text
String urlencode(const String& s) {
  String out; char buf[4];
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') out += c;
    else { snprintf(buf, sizeof(buf), "%%%02X", (unsigned char)c); out += buf; }
  }
  return out;
}

// Fire a Telegram message (HTTPS). Non-blocking enough for our 30s cadence.
void sendTelegram(const String& text) {
  WiFiClientSecure client;
  client.setInsecure();                 // skip cert check (simple + reliable)
  HTTPClient https;
  String url = "https://api.telegram.org/bot" + String(TG_TOKEN) + "/sendMessage";
  if (https.begin(client, url)) {
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String body = "chat_id=" + String(TG_CHAT) + "&text=" + urlencode(text) +
                  "&disable_web_page_preview=false";
    int code = https.POST(body);
    Serial.printf("[Telegram] HTTP %d\n", code);
    https.end();
  } else {
    Serial.println("[Telegram] begin() failed");
  }
}

// Send a native Telegram location pin (shows a real map in the chat)
void sendLocationTg(const String& chatId, double lat, double lng) {
  WiFiClientSecure client; client.setInsecure();
  HTTPClient https;
  String url = "https://api.telegram.org/bot" + String(TG_TOKEN) + "/sendLocation";
  if (https.begin(client, url)) {
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String body = "chat_id=" + chatId +
                  "&latitude=" + String(lat, 6) +
                  "&longitude=" + String(lng, 6);
    int code = https.POST(body);
    Serial.printf("[Telegram] sendLocation HTTP %d\n", code);
    https.end();
  }
}

// Reply to a chat with the device's current location (pin + text)
void replyWithLocation(const String& chatId) {
  if (g_hasFix) {
    sendLocationTg(chatId, g_lat, g_lng);
    String t = "Live location\nLat: " + String(g_lat, 6) +
               "\nLng: " + String(g_lng, 6) +
               "\nSats: " + String((unsigned)g_satellites) +
               "\nhttps://maps.google.com/?q=" + String(g_lat, 6) + "," + String(g_lng, 6);
    sendTelegram(t);
  } else {
    sendTelegram("No GPS fix yet - device can't see satellites. Try again shortly.");
  }
}

// Poll Telegram for new messages; reply to any command from the owner chat
void pollTelegram() {
  WiFiClientSecure client; client.setInsecure();
  HTTPClient https;
  String url = "https://api.telegram.org/bot" + String(TG_TOKEN) +
               "/getUpdates?timeout=0&offset=" + String(g_tgOffset);
  if (!https.begin(client, url)) return;
  int code = https.GET();
  if (code == 200) {
    String payload = https.getString();
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (!err) {
      for (JsonObject upd : doc["result"].as<JsonArray>()) {
        long updateId = upd["update_id"].as<long>();
        g_tgOffset = updateId + 1;                 // advance so we don't re-read
        JsonObject msg = upd["message"];
        if (msg.isNull()) continue;
        String fromChat = String((long long)msg["chat"]["id"].as<long long>());
        if (!g_tgPrimed) continue;                  // ignore backlog on first poll
        if (fromChat != String(TG_CHAT)) continue;  // only the owner
        Serial.printf("[Telegram] cmd from %s: %s\n",
                      fromChat.c_str(), msg["text"] | "");
        replyWithLocation(fromChat);                // any message -> send location
      }
      g_tgPrimed = true;   // from here on, respond to new messages
    }
  }
  https.end();
}

// Great-circle distance in metres
double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  return R * 2 * atan2(sqrt(a), sqrt(1-a));
}

void handleSetFence() {
  if (server.hasArg("lat") && server.hasArg("lng") && server.hasArg("r")) {
    g_fenceLat = server.arg("lat").toDouble();
    g_fenceLng = server.arg("lng").toDouble();
    g_fenceRadius = server.arg("r").toDouble();
    g_fenceSet = true;
    g_wasOutside = false;               // reset alert state
    g_lastAlert = 0;
    Serial.printf("[Fence] set center=%.6f,%.6f r=%.0f m\n",
                  g_fenceLat, g_fenceLng, g_fenceRadius);
    server.send(200, "text/plain", "fence set");
  } else {
    server.send(400, "text/plain", "need lat,lng,r");
  }
}

void handleClearFence() {
  g_fenceSet = false;
  g_wasOutside = false;
  Serial.println("[Fence] cleared");
  server.send(200, "text/plain", "fence cleared");
}

void handleData() {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"fix\":%s,\"lat\":%.6f,\"lng\":%.6f,\"sats\":%u,"
    "\"alt\":%.1f,\"spd\":%.1f,\"utc\":\"%s\"}",
    g_hasFix ? "true" : "false",
    g_lat, g_lng, (unsigned)g_satellites,
    g_altitude, g_speedKmph, g_timeUtc);
  server.send(200, "application/json", buf);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nESP32-S3 GPS Tracker starting...");

  // Status LEDs
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);          // start in "searching" (red on)
  digitalWrite(LED_ONBOARD, HIGH);      // onboard is active-LOW -> HIGH = off

  // GPS UART
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // WiFi (non-blocking: loop() keeps trying and shows red until connected)
  WiFi.mode(WIFI_STA);

  // Print the exact reason on every disconnect (diagnoses auth vs other)
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("\n[WiFi EVENT] disconnected, reason=%d ",
                  info.wifi_sta_disconnected.reason);
    switch (info.wifi_sta_disconnected.reason) {
      case 2:   Serial.println("(AUTH_EXPIRE - usually wrong password)"); break;
      case 15:  Serial.println("(4WAY_HANDSHAKE_TIMEOUT - WRONG PASSWORD)"); break;
      case 201: Serial.println("(NO_AP_FOUND - SSID not seen)"); break;
      case 202: Serial.println("(AUTH_FAIL - WRONG PASSWORD)"); break;
      case 205: Serial.println("(CONNECTION_FAIL)"); break;
      default:  Serial.println("(see esp_wifi_types.h)"); break;
    }
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // Wait so a serial capture can attach, then scan once (reliable, radio idle)
  delay(2000);
  Serial.println("\n--- Scanning for WiFi networks ---");
  int n = WiFi.scanNetworks();
  bool targetSeen = false;
  int targetRssi = 0;
  if (n <= 0) {
    Serial.println("No networks found!");
  } else {
    for (int i = 0; i < n; i++) {
      bool match = (WiFi.SSID(i) == String(WIFI_SSID));
      if (match) { targetSeen = true; targetRssi = WiFi.RSSI(i); }
      Serial.printf("  [%2d] \"%s\"  RSSI=%d dBm  %s%s\n",
        i, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
        (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "OPEN" : "secured",
        match ? "  <<< TARGET" : "");
    }
  }
  Serial.printf(">>> Target \"%s\" in range: %s", WIFI_SSID, targetSeen ? "YES" : "NO");
  if (targetSeen) Serial.printf(" (%d dBm)", targetRssi);
  Serial.println("\n----------------------------------");
  WiFi.scanDelete();

  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to \"%s\" (red LED on while searching)", WIFI_SSID);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setfence", handleSetFence);
  server.on("/clearfence", handleClearFence);
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  // Feed GPS parser
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    g_lat = gps.location.lat();
    g_lng = gps.location.lng();
    g_hasFix = true;
  }
  if (gps.altitude.isValid())   g_altitude   = gps.altitude.meters();
  if (gps.speed.isValid())      g_speedKmph  = gps.speed.kmph();
  if (gps.satellites.isValid()) g_satellites = gps.satellites.value();
  if (gps.time.isValid()) {
    snprintf(g_timeUtc, sizeof(g_timeUtc), "%02d:%02d:%02d",
             gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  // --- Poll Telegram for /location requests ---
  if (WiFi.status() == WL_CONNECTED && millis() - g_lastPollTg > TG_POLL_MS) {
    g_lastPollTg = millis();
    pollTelegram();
  }

  // --- Geofence enforcement + Telegram alerts (device-side) ---
  if (g_fenceSet && g_hasFix && WiFi.status() == WL_CONNECTED) {
    double dist = distanceMeters(g_fenceLat, g_fenceLng, g_lat, g_lng);
    bool outside = dist > g_fenceRadius;
    if (outside) {
      bool justLeft = !g_wasOutside;
      if (justLeft || millis() - g_lastAlert > TG_REPEAT_MS) {
        g_lastAlert = millis();
        double over = dist - g_fenceRadius;
        String msg = "OUTSIDE area\n";
        msg += "Lat: " + String(g_lat, 6) + "\n";
        msg += "Lng: " + String(g_lng, 6) + "\n";
        msg += String(over, 0) + " m beyond boundary\n";
        msg += "https://maps.google.com/?q=" + String(g_lat, 6) + "," + String(g_lng, 6);
        sendTelegram(msg);
      }
    }
    g_wasOutside = outside;
  }

  // --- WiFi state + status LEDs ---
  static bool wasConnected = false;
  static uint32_t lastReconnect = 0;
  static uint32_t lastBlink = 0;
  static bool blinkState = false;
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected) {
    digitalWrite(LED_GREEN, HIGH);      // green on
    digitalWrite(LED_RED, LOW);         // red off
    digitalWrite(LED_ONBOARD, LOW);     // onboard on (active-LOW)
    if (!wasConnected) {
      Serial.print("\nWiFi CONNECTED. Dashboard: http://");
      Serial.println(WiFi.localIP());
      wasConnected = true;
    }
  } else {
    digitalWrite(LED_GREEN, LOW);       // green off
    digitalWrite(LED_ONBOARD, HIGH);    // onboard off
    // blink red while searching
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      blinkState = !blinkState;
      digitalWrite(LED_RED, blinkState ? HIGH : LOW);
    }
    if (wasConnected) { Serial.println("\nWiFi lost - searching (red LED)..."); wasConnected = false; }
    // retry connection periodically
    if (millis() - lastReconnect > 8000) {
      lastReconnect = millis();
      Serial.print(".");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }

  server.handleClient();

  // Periodic status print so the IP is always visible on the serial monitor
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 5000) {
    lastPrint = millis();
    Serial.print("[status] WiFi=");
    if (connected) {
      Serial.print("OK  Dashboard: http://");
      Serial.print(WiFi.localIP());
    } else {
      Serial.print("SEARCHING");
    }
    Serial.print("  GPS fix=");
    Serial.print(g_hasFix ? "YES" : "no");
    Serial.print("  sats=");
    Serial.print(g_satellites);
    Serial.print("  lat=");
    Serial.print(g_lat, 6);
    Serial.print("  lng=");
    Serial.print(g_lng, 6);
    Serial.print("  | GPS chars=");
    Serial.print(gps.charsProcessed());
    Serial.print(" sentences=");
    Serial.print(gps.sentencesWithFix());
    Serial.print(" checksumErr=");
    Serial.println(gps.failedChecksum());
  }
}
