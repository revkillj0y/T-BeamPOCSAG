/*
  T-Beam v1.1 (SX1278 433 MHz) — POCSAG 1200 Receiver (RadioLib POCSAG client)
  - Uses RadioLib's built-in POCSAG/Pager receiver (with BCH/parity)
  - FSK mode on SX1278
  - Displays decoded messages on 0.96" SSD1306 (I2C 0x3C)

  Libraries (Library Manager):
    - RadioLib by jgromes
    - Adafruit GFX
    - Adafruit SSD1306
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

// ---------- POCSAG client (RadioLib) ----------
// Most current RadioLib versions name this POCSAGClient and provide POCSAGMessage, POCSAG_MODE_* enums.
POCSAGClient pager(&radio);

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

  // Init pager client @ 1200 bps (enum-based API)
  // If this line doesn't compile on your RadioLib, see the alternatives below in loop() comments.
  st = pager.begin(POCSAG_MODE_1200);
  if (st != RADIOLIB_ERR_NONE) {
    pushLine(String("pager.begin failed: ") + st);
    drawScreen();
    while (true) delay(1000);
  }

  pushLine("Pager ready (1200 bps)");
  drawScreen();
}

void loop() {
  // RadioLib has a few API variants; try this common one first:
  //   POCSAGMessage msg; int16_t state = pager.receive(msg);
  POCSAGMessage msg;
  int16_t state = pager.receive(msg);

  if (state == RADIOLIB_ERR_NONE) {
    // msg.address (RIC), msg.function, msg.alpha (string), msg.numeric (string), msg.isNumeric()
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
  // else: ignore timeouts / no-data codes

  delay(5);
}

/*
  If your RadioLib version has different names/signatures, try these swaps:

  1) Class name:
     - Change `POCSAGClient` to `PagerClient`

  2) begin() signature:
     - Use `pager.begin(POCSAG_MODE_1200);`   // preferred, enum
     - If that doesn't exist, try `pager.begin(1200);`  // older int-based API
     - Or some versions use `pager.begin(); pager.setMode(POCSAG_MODE_1200);`

  3) receive() signature:
     - `POCSAGMessage msg; pager.receive(msg);`   // preferred
     - or: `String text; uint32_t ric; pager.receive(ric, text);`
     - or: `String text; pager.receive(text);`

  Keep the SX1278 wiring/SPI exactly as above.
*/
