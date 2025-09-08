/*
  T-Beam v1.1 (SX1278 433 MHz) — POCSAG 1200 via RadioLib Pager client (auto-compat)
  - Adapts to RadioLib variants:
      * POCSAGClient + POCSAGMessage + begin(POCSAG_MODE_1200) + receive(msg)
      * PagerClient + begin(POCSAG_MODE_1200 or 1200) + receive(ric, text)
  - FSK mode (not LoRa), OLED display (SSD1306 128x64 I2C 0x3C)
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- T-Beam v1.1 LoRa/SX1278 pins ----------
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_CS     18
#define LORA_RST    23
#define LORA_DIO0   26
#define LORA_DIO1   33   // not required here
#define LORA_DIO2   32   // not required here

// ---------- OLED ----------
#define OLED_SDA        21
#define OLED_SCL        22
#define OLED_ADDR       0x3C
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- Radio ----------
SX1278 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);

// ---------- Try to detect which pager API is available ----------
#if __has_include(<POCSAG.h>) || __has_include(<RadioLib/POCSAG.h>)
  #if __has_include(<POCSAG.h>)
    #include <POCSAG.h>
  #else
    #include <RadioLib/POCSAG.h>
  #endif
  #define HAVE_POCSAG_CLIENT 1
#elif __has_include(<Pager.h>) || __has_include(<RadioLib/Pager.h>)
  #if __has_include(<Pager.h>)
    #include <Pager.h>
  #else
    #include <RadioLib/Pager.h>
  #endif
  #define HAVE_PAGER_CLIENT 1
#else
  #warning "No POCSAG/Pager client headers found in your RadioLib. Please update RadioLib to a version that includes POCSAG."
#endif

#ifndef POCSAG_MODE_1200
  // Fallback: if enum not defined, treat as integer bitrate
  #define POCSAG_MODE_1200 1200
#endif

// Instantiate the right client type
#if defined(HAVE_POCSAG_CLIENT)
  POCSAGClient pager(&radio);
#elif defined(HAVE_PAGER_CLIENT)
  PagerClient pager(&radio);
#endif

// ---------- Config ----------
static const float  RX_FREQ_MHZ = 439.9875;  // your channel
static const float  BITRATE     = 1200.0;    // DAPNET/MMDVM default
static const uint32_t FDEV_HZ   = 4500;      // try 3500..6000 if needed
static const uint32_t RXBW_HZ   = 25000;     // 25 kHz (use 50000 or 62500 if not locking)

// OLED log buffer
String lastLines[4] = {"POCSAG @ 439.9875", "initializing…", "", ""};

// ---------- Helpers ----------
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

  // SPI
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Start radio in FSK mode
  Serial.println("Init SX1278 FSK…");
  int16_t st = radio.beginFSK();
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("beginFSK failed: ") + st);
    drawScreen();
    while (true) delay(1000);
  }

  // Front-end settings
  radio.setFrequency(RX_FREQ_MHZ);
  radio.setBitRate(BITRATE);
  radio.setFrequencyDeviation(FDEV_HZ);
  radio.setRxBandwidth(RXBW_HZ);
  radio.setDataShaping(RADIOLIB_SHAPING_NONE);
  radio.setEncoding(RADIOLIB_ENCODING_MANCHESTER);  // POCSAG uses Manchester
  radio.setPreambleLength(18);                       // not critical
  radio.setCRC(false);

  // Init pager client @ 1200 bps (enum-backed or integer fallback)
#if defined(HAVE_POCSAG_CLIENT) || defined(HAVE_PAGER_CLIENT)
  st = pager.begin(POCSAG_MODE_1200);
  if (st != RADIOLIB_ERR_NONE) {
    // Try fallback begin(1200) if enum form not supported
    st = pager.begin(1200);
  }
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("pager.begin failed: ") + st);
    drawScreen();
    while (true) delay(1000);
  }
#else
  pushLine("No Pager client in this RadioLib!");
  drawScreen();
  while (true) delay(1000);
#endif

  pushLine("Pager ready (1200 bps)");
  drawScreen();
}

void loop() {
#if defined(HAVE_POCSAG_CLIENT)
  // Preferred API: POCSAGMessage
  POCSAGMessage msg;
  int16_t state = pager.receive(msg);

  if (state == RADIOLIB_ERR_NONE) {
    String text;
    if (msg.isNumeric()) {
      text = msg.numeric.length() ? msg.numeric : "(numeric)";
    } else {
      text = msg.alpha.length() ? msg.alpha : "(alpha)";
    }
    char line[192];
    snprintf(line, sizeof(line), "#%u fn=%u: %s",
             (unsigned)msg.address, (unsigned)msg.function, text.c_str());
    pushLine(line);
    drawScreen();
  }

#elif defined(HAVE_PAGER_CLIENT)
  // Older API: receive(ric, text)
  String text;
  uint32_t ric = 0;
  int16_t state = pager.receive(ric, text);

  if (state == RADIOLIB_ERR_NONE) {
    if (text.length() == 0) text = "(empty)";
    char line[192];
    snprintf(line, sizeof(line), "#%u: %s", (unsigned)ric, text.c_str());
    pushLine(line);
    drawScreen();
  }
#else
  // No pager client available — nothing to do
  delay(500);
  return;
#endif

  delay(5);
}
