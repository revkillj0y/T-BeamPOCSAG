/*
  T-Beam v1.1 (SX1278 433 MHz) — POCSAG 1200 using RadioLib PagerClient (direct mode)
  - Center: 439.9875 MHz
  - Speed: 1200 bps
  - Shift: ~4500 Hz (default)
  - Accepts ALL pager addresses (mask=0)
  - Shows decoded text + RIC on 0.96" SSD1306 (I2C 0x3C)

  Requires:
    - RadioLib (with PagerClient enabled)
    - Adafruit GFX + Adafruit SSD1306

  If your RadioLib gates Pager, you can force-enable:
    #define RADIOLIB_BUILD_PAGER 1
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// If needed, uncomment the next line to force-build Pager in some RadioLib builds
//#define RADIOLIB_BUILD_PAGER 1
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
#define LORA_DIO2   32   // DATA pin for direct mode (use this in startReceive)

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

// -------- Config --------
static const float   RX_FREQ_MHZ = 439.9875f;  // your channel
static const uint16_t BITRATE    = 1200;       // MMDVM/WPSD / DAPNET default
static const bool     INVERT     = true;      // try true if needed
static const uint16_t SHIFT_HZ   = 4500;       // default in Pager (4.5 kHz)

// OLED log buffer
String lastLines[4] = {"POCSAG @ 439.9875", "initializing…", "", ""};

void pushLine(const String& s) {
  lastLines[0] = lastLines[1];
  lastLines[1] = lastLines[2];
  lastLines[2] = lastLines[3];
  lastLines[3] = s;
  Serial.println(s);
}
void drawScreen() {
  display.clearDisplay();
  display.setCursor(0, 0);
  for (int i = 0; i < 4; i++) display.println(lastLines[i]);
  display.display();
}

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

  // SPI for SX1278
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Bring up radio (FSK base)
  int16_t st = radio.beginFSK();
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("beginFSK failed: ") + st);
    drawScreen();
    while (true) delay(1000);
  }

  // Pager client: direct mode init
  // begin(base MHz, speed bps, invert=false, shift=4500 Hz)  (see Pager.h begin signature)
  st = pager.begin(RX_FREQ_MHZ, BITRATE, INVERT, SHIFT_HZ);
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("pager.begin failed: ") + st);
    drawScreen();
    while (true) delay(1000);
  }

  // Start receiving via SX1278 DIO2 as DATA pin.
  // Use addr=0, mask=0 to accept ALL pager addresses. (addressMatched uses (addr & mask) == (filterAddr & mask))
  st = pager.startReceive(LORA_DIO2, 0, 0);
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("startReceive failed: ") + st);
    drawScreen();
    while (true) delay(1000);
  }

  pushLine("Pager direct ready (1200 bps)");
  drawScreen();
}

void loop() {
  // Wait until at least 1–2 batches are buffered (short msgs fit in 1–2)
  if (pager.available() >= 1) {
    String text;
    uint32_t ric = 0;
    int16_t st = pager.readData(text, 0, &ric);
    if (st == RADIOLIB_ERR_NONE) {
      if (text.length() == 0) text = "(empty)";
      char line[192];
      snprintf(line, sizeof(line), "#%u: %s", (unsigned)ric, text.c_str());
      pushLine(line);
      drawScreen();
    } else {
      // Uncomment for debugging:
      // Serial.printf("readData err: %d\n", st);
    }
  }

  delay(5);
}
