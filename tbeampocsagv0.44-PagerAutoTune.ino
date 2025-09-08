/*
  T-Beam v1.1 (SX1278 433 MHz) — POCSAG 1200 (RadioLib PagerClient, Auto-Tune)
  - Direct mode via PagerClient, DIO2 = DATA (GPIO32)
  - Auto-scans invert, freq offset, and shift until messages are found
  - Shows current try + RSSI on OLED, logs to Serial
  - Accepts ALL RICs (mask=0)

  Requires RadioLib with PagerClient, Adafruit GFX + SSD1306
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
// If needed on your build, you can uncomment the next line:
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

// -------- Base config --------
static const float   BASE_FREQ_MHZ = 439.9875f;
static const uint16_t BITRATE      = 1200;

// Scan candidates
static const int   kNumOffsets = 3;
static const float kOffsetHz[kNumOffsets] = {0.0f, +2500.0f, -2500.0f};    // center offsets
static const int   kNumShifts  = 3;
static const uint16_t kShiftHz[kNumShifts] = {4500, 4000, 5000};           // deviation-ish
static const int   kNumInvert  = 2;
static const bool  kInvert[kNumInvert] = {false, true};

static const uint32_t DWELL_MS = 4000;     // how long to try each combo before moving on
static const uint32_t READ_LOOP_MS = 5;    // loop delay

// OLED/Serial helpers
String lastLines[4] = {"POCSAG @ 439.9875", "auto-tune…", "", ""};
void pushLine(const String& s) {
  lastLines[0] = lastLines[1];
  lastLines[1] = lastLines[2];
  lastLines[2] = lastLines[3];
  lastLines[3] = s;
  Serial.println(s);
}
void drawScreen(const char* status = nullptr, float rssi = NAN) {
  display.clearDisplay();
  display.setCursor(0, 0);
  for (int i = 0; i < 4; i++) display.println(lastLines[i]);
  if (status) {
    display.println();
    display.println(status);
  }
  if (!isnan(rssi)) {
    display.printf("RSSI: %.1f dBm\n", rssi);
  }
  display.display();
}

// Quick FSK RSSI read (SX127x RegRssiValue = 0x11 in FSK)
float readRSSI_dBm() {
  // Note: RadioLib doesn't expose direct reg read here; reuse module SPI.
  // SX127x FSK RSSI register is 0x11 (approx scale ~ -RSSI/2 dBm)
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(0x11 & 0x7F);
  uint8_t v = SPI.transfer(0);
  digitalWrite(LORA_CS, HIGH);
  return -((float)v / 2.0f);
}

// Start/stop helpers for a given combo
bool startCombo(float freqMHz, bool invert, uint16_t shift) {
  // (Re)initialize radio base if needed
  // radio.beginFSK() only once at boot is usually enough, but re-calling is harmless.
  int16_t st = pager.begin(freqMHz, BITRATE, invert, shift);
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("pager.begin err: ") + st);
    return false;
  }
  st = pager.startReceive(LORA_DIO2, 0, 0); // accept ALL RICs
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("startReceive err: ") + st);
    return false;
  }
  char buf[64];
  snprintf(buf, sizeof(buf), "Trying f=%.4fMHz off=%.1fkHz inv=%d sh=%u",
           freqMHz, (freqMHz - BASE_FREQ_MHZ)*1000.0f, invert ? 1 : 0, shift);
  pushLine(buf);
  return true;
}

void stopCombo() {
  // There’s no explicit stop in PagerClient; re-calling begin()/startReceive() replaces config.
  // We can still log:
  pushLine("…no decode, next combo");
}

// State
bool locked = false;
float lockedFreq = BASE_FREQ_MHZ;
bool lockedInvert = false;
uint16_t lockedShift = 4500;

void setup() {
  Serial.begin(115200);
  delay(200);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    drawScreen();
  }

  // SPI + radio base init
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  int16_t st = radio.beginFSK();
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("beginFSK failed: ") + st);
    drawScreen();
    while (true) delay(1000);
  }

  pushLine("Auto-tune starting…");
  drawScreen();
}

void loop() {
  static int idxInv = 0, idxOff = 0, idxShift = 0;
  static uint32_t comboStart = 0;
  static bool comboActive = false;

  if (!locked) {
    if (!comboActive) {
      float freq = BASE_FREQ_MHZ + (kOffsetHz[idxOff] / 1e6f);
      bool invert = kInvert[idxInv];
      uint16_t shift = kShiftHz[idxShift];

      if (!startCombo(freq, invert, shift)) {
        // Move to next combo immediately on init error
        idxShift = (idxShift + 1) % kNumShifts;
        if (idxShift == 0) { idxOff = (idxOff + 1) % kNumOffsets; if (idxOff == 0) idxInv = (idxInv + 1) % kNumInvert; }
      } else {
        comboActive = true;
        comboStart = millis();
      }
    } else {
      // Combo running: check for data
      // Show RSSI once in a while
      static uint32_t lastMeter = 0;
      uint32_t now = millis();
      if (now - lastMeter >= 500) {
        lastMeter = now;
        float rssi = readRSSI_dBm();
        drawScreen("scanning…", rssi);
      }

      // If we’ve buffered at least one batch, try to read a message
      if (pager.available() >= 1) {
        String text; uint32_t ric = 0;
        int16_t st = pager.readData(text, 0, &ric);
        if (st == RADIOLIB_ERR_NONE) {
          locked = true;
          // Record the working combo
          lockedFreq = BASE_FREQ_MHZ + (kOffsetHz[idxOff] / 1e6f);
          lockedInvert = kInvert[idxInv];
          lockedShift = kShiftHz[idxShift];

          pushLine("LOCKED!");
          char line[96];
          snprintf(line, sizeof(line), "f=%.4fMHz inv=%d sh=%u",
                   lockedFreq, lockedInvert?1:0, lockedShift);
          pushLine(line);

          // Show the first message
          if (text.length() == 0) text = "(empty)";
          char msgLine[192];
          snprintf(msgLine, sizeof(msgLine), "#%u: %s", (unsigned)ric, text.c_str());
          pushLine(msgLine);
          drawScreen();

          // Stay in the locked branch below
        } else {
          // readData failed; keep dwelling until timeout
        }
      }

      // Dwell timeout: rotate to next combo
      if (!locked && millis() - comboStart >= DWELL_MS) {
        stopCombo();
        comboActive = false;

        // advance
        idxShift = (idxShift + 1) % kNumShifts;
        if (idxShift == 0) {
          idxOff = (idxOff + 1) % kNumOffsets;
          if (idxOff == 0) {
            idxInv = (idxInv + 1) % kNumInvert;
          }
        }
      }
    }

    delay(READ_LOOP_MS);
    return;
  }

  // ---- Locked: normal receive/print loop ----
  // Keep printing messages as they arrive; also keep an eye on RSSI
  if (pager.available() >= 1) {
    String text; uint32_t ric = 0;
    int16_t st = pager.readData(text, 0, &ric);
    if (st == RADIOLIB_ERR_NONE) {
      if (text.length() == 0) text = "(empty)";
      char msgLine[192];
      snprintf(msgLine, sizeof(msgLine), "#%u: %s", (unsigned)ric, text.c_str());
      pushLine(msgLine);
      drawScreen();
    }
  }

  static uint32_t lastRSSI = 0;
  uint32_t now = millis();
  if (now - lastRSSI >= 1000) {
    lastRSSI = now;
    float rssi = readRSSI_dBm();
    drawScreen("locked", rssi);
  }

  delay(READ_LOOP_MS);
}
