/*
  T-Beam v1.1 (SX1278 433 MHz) — POCSAG 1200 (RadioLib PagerClient, FIXED + RIC de-ROT1 + Status Footer)
  Locked params from your autotune:
    f = 439.9900 MHz, invert = false, shift = 4500 Hz

  - Accepts ALL RICs (mask=0)
  - Reads all queued messages each loop
  - Auto-escalates to >=2 batches if a decode looks corrupted
  - Applies ASCII(-1) only to specific RICs (e.g., Skyper Index 4512)
  - OLED (SSD1306 128x64 I2C 0x3C) rolling log + footer (RSSI + time since last decode)

  Next rev idea: map buttons for sleep/clear/etc.

  Requires: RadioLib (PagerClient), Adafruit GFX, Adafruit SSD1306
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
// If your RadioLib build gates Pager, you can force-enable it:
// #define RADIOLIB_BUILD_PAGER 1
#include <RadioLib.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------- T-Beam v1.1 (SX1278) pins --------
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_CS     18
#define LORA_RST    23
#define LORA_DIO0   26
#define LORA_DIO1   33
#define LORA_DIO2   32   // DATA pin for Pager direct mode

// -------- OLED --------
#define OLED_SDA        21
#define OLED_SCL        22
#define OLED_ADDR       0x3C
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -------- Radio & Pager --------
SX1278 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
PagerClient pager(&radio);

// -------- Fixed RF config (your working combo) --------
static const float   RX_FREQ_MHZ = 439.9900f;  // change if you retune TX
static const uint16_t BITRATE    = 1200;
static const bool     INVERT     = false;
static const uint16_t SHIFT_HZ   = 4500;

// -------- RICs that consistently come as +1 ASCII and need de-ROT1 --------
static const uint32_t RIC_ROT1_LIST[] = {
  4512,   // Skyper Index (from your logs)
};

// -------- UI State --------
String lastLines[4] = {"POCSAG @ 439.9900", "waiting…", "", ""};
uint32_t lastDecodeMs = 0;
float currentRssiDbm = NAN;        // updated ~1 Hz
uint32_t lastRssiMs = 0;

// -------- Helpers --------
static void pushLine(const String& s) {
  lastLines[0] = lastLines[1];
  lastLines[1] = lastLines[2];
  lastLines[2] = lastLines[3];
  lastLines[3] = s;
  Serial.println(s);
}

// Quick FSK RSSI read (SX127x RegRssiValue = 0x11 in FSK)
float readRSSI_dBm() {
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(0x11 & 0x7F);         // read addr 0x11
  uint8_t v = SPI.transfer(0);
  digitalWrite(LORA_CS, HIGH);
  // Datasheet note: approx -RSSI/2 dBm for FSK mode
  return -((float)v / 2.0f);
}

// format mm:ss
String fmtAge(uint32_t msAgo) {
  uint32_t s = msAgo / 1000;
  uint32_t mm = s / 60;
  uint32_t ss = s % 60;
  char buf[8];
  snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)mm, (unsigned)ss);
  return String(buf);
}

static void drawScreen(bool full = true) {
  if (full) {
    display.clearDisplay();
    display.setCursor(0, 0);
    for (int i = 0; i < 4; i++) display.println(lastLines[i]);

    // separator
    display.drawFastHLine(0, 50, SCREEN_WIDTH, SSD1306_WHITE);
  } else {
    // just clear footer area
    display.fillRect(0, 51, SCREEN_WIDTH, SCREEN_HEIGHT - 51, SSD1306_BLACK);
  }

  // Footer: line 1 = RSSI, line 2 = last decode age
  display.setCursor(0, 54);
  if (!isnan(currentRssiDbm)) {
    display.printf("RSSI: %.1f dBm", currentRssiDbm);
  } else {
    display.print("RSSI: --.- dBm");
  }

  display.setCursor(0, 62);
  if (lastDecodeMs > 0) {
    uint32_t age = millis() - lastDecodeMs;
    display.print("Last: ");
    display.print(fmtAge(age));
    display.print(" ago");
  } else {
    display.print("Last: --:--");
  }

  display.display();
}

static bool ricNeedsRot1(uint32_t ric) {
  for (size_t i = 0; i < sizeof(RIC_ROT1_LIST)/sizeof(RIC_ROT1_LIST[0]); i++) {
    if (RIC_ROT1_LIST[i] == ric) return true;
  }
  return false;
}

// Simple corruption check: if too many chars are outside a typical pager set,
// treat as “probably partial/corrupted” and try again with more batches.
static bool looksCorrupted(const String& s) {
  if (s.length() < 3) return false;
  int bad = 0, vis = 0;
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if (c == '\r' || c == '\n' || c == '\t' || c == ' ') continue;
    vis++;
    bool ok =
      (c >= '0' && c <= '9') ||
      (c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') ||
      (c == '-') || (c == '_') || (c == '+') || (c == '/') ||
      (c == '(') || (c == ')') || (c == ':') || (c == '.') || (c == ',') ||
      (c == '[') || (c == ']');
    if (!ok) bad++;
  }
  return (vis >= 8) && (bad * 100 / vis > 25);
}

static String deRot1(const String& s) {
  String out; out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if (c >= 0x21 && c <= 0x7E) out += char(c - 1);
    else out += s[i];
  }
  return out;
}

void setup() {
  Serial.begin(115200);
  delay(150);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    drawScreen(true);
  }

  // SPI + radio base
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  int16_t st = radio.beginFSK();
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("beginFSK failed: ") + st);
    drawScreen(true);
    while (true) delay(1000);
  }

  // Pager direct-mode config (fixed combo)
  st = pager.begin(RX_FREQ_MHZ, BITRATE, INVERT, SHIFT_HZ);
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("pager.begin err: ") + st);
    drawScreen(true);
    while (true) delay(1000);
  }

  // Accept ALL RICs, DATA = DIO2 (GPIO32 on T-Beam)
  st = pager.startReceive(LORA_DIO2, 0, 0);
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("startReceive err: ") + st);
    drawScreen(true);
    while (true) delay(1000);
  }

  pushLine("Fixed cfg: inv=0 sh=4500");
  drawScreen(true);
}

void loop() {
  static bool needTwoBatches = false;   // escalate threshold after a corrupt read
  const uint8_t minBatches = needTwoBatches ? 2 : 1;

  // Drain everything currently buffered so we don’t miss header/payload pairs.
  bool showedAnything = false;
  while (pager.available() >= minBatches) {
    String text; uint32_t ric = 0;
    int16_t st = pager.readData(text, 0, &ric);
    if (st != RADIOLIB_ERR_NONE) {
      // transient decoder hiccup; break out and retry on next loop
      break;
    }

    if (text.length() == 0) text = "(empty)";

    // If this RIC is known to be +1 ASCII, apply de-ROT1 (only then).
    if (ricNeedsRot1(ric)) {
      text = deRot1(text);
    }

    // If what we got looks corrupted, escalate threshold and skip displaying it
    if (looksCorrupted(text)) {
      needTwoBatches = true;
      continue;
    } else {
      needTwoBatches = false;
    }

    // Show message
    char line[192];
    snprintf(line, sizeof(line), "#%u: %s", (unsigned)ric, text.c_str());
    pushLine(line);
    lastDecodeMs = millis();
    drawScreen(true);
    showedAnything = true;
  }

  // Footer refresh (RSSI + last decode age) once per second
  uint32_t now = millis();
  if (now - lastRssiMs >= 1000) {
    lastRssiMs = now;
    currentRssiDbm = readRSSI_dBm();
    drawScreen(false);  // only redraw footer area
  }

  if (!showedAnything) {
    delay(5);
  }
}
