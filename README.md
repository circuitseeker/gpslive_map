# gpslive_map

ESP32-S3 + NEO-6M GPS tracker with a live web map, geofence alerts, and a
Telegram bot. Built for the **Seeed Studio XIAO ESP32-S3**.

## What it does

- Reads GPS coordinates from a u-blox **NEO-6M** over UART.
- Joins WiFi and **hosts its own web dashboard** (Leaflet + OpenStreetMap) with a
  red dot at the live location, free zoom/pan, auto-zoom to the fix.
- **Geofence**: draw a circle on the map; if the device leaves it you get an
  on-page alert plus a **Telegram message** ("OUTSIDE area" + coordinates + map
  link), repeating while outside. The fence is enforced on the device, so alerts
  fire even with no browser open.
- **On-demand location**: message the Telegram bot and it replies with a native
  map pin + coordinates.
- Two status LEDs: green = WiFi connected, red = searching.

## Hardware / wiring

| Signal | NEO-6M | XIAO label | GPIO |
|---|---|---|---|
| Power | VCC | 3V3 | — |
| Ground | GND | GND | — |
| GPS -> ESP (RX) | TX | D7 | 44 |
| ESP -> GPS (TX) | RX | D6 | 43 |
| Green LED (+) | — | D0 | 1 |
| Red LED (+) | — | D1 | 2 |

LEDs go through a 220 ohm resistor to GND. Power the NEO-6M from **3V3** (the
ESP32-S3 is not 5 V tolerant). Plug in the U.FL WiFi antenna.

## Setup

1. Install the ESP32 core and libraries:
   ```bash
   arduino-cli core install esp32:esp32
   arduino-cli lib install TinyGPSPlus
   arduino-cli lib install ArduinoJson
   ```
2. Copy `gps_tracker_s3/secrets.h.example` to `gps_tracker_s3/secrets.h` and fill
   in your WiFi and Telegram details. `secrets.h` is git-ignored.
3. Build and upload:
   ```bash
   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 gps_tracker_s3
   arduino-cli upload -p <PORT> --fqbn esp32:esp32:XIAO_ESP32S3 gps_tracker_s3
   ```
4. Open the IP printed on the serial monitor (115200 baud) in a browser on the
   same WiFi.

## Notes

- The ESP32-S3 WiFi is 2.4 GHz only; SSID and password are case-sensitive.
- First GPS fix needs a clear view of the sky and can take a few minutes.
