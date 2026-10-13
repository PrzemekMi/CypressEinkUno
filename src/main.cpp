#include <Arduino.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include "../include/aip_image.h"

// CY8CKIT-028-EPD shield (Pervasive Displays E2271CS021, 264x176, G2 COG).
// The 1-bit image is stored in program memory; only one display line uses RAM.

// Arduino header pins used by CY8CKIT-028-EPD (check against the shield schematic)
const uint8_t PIN_CS        = 10;  // SPI: MOSI=D11, MISO=D12, SCK=D13
const uint8_t PIN_RST       = 2;
const uint8_t PIN_BUSY      = 3;
const uint8_t PIN_EN        = 4;   // panel power (VCC) enable
const uint8_t PIN_DISCHARGE = 5;
const uint8_t PIN_BORDER    = 6;
const uint8_t PIN_IO_EN     = 7;   // shield's IO buffer enable, active LOW

const uint16_t HEIGHT         = 176;
const uint16_t WIDTH          = 264;
const uint8_t  BYTES_PER_LINE = WIDTH / 8;   // 33
const uint8_t  BYTES_PER_SCAN = HEIGHT / 4;  // 44
const uint8_t  CHANNEL_SELECT[] = {0x72, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x00};
const uint8_t  VOLTAGE_LEVEL  = 0x00;
const unsigned long STAGE_TIME_MS = 630;     // valid for ~20..40 C; increase when colder

enum Stage { COMPENSATE, WHITE, INVERSE, NORMAL };

SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);

// Fills one display line, MSB = leftmost pixel, bit 1 = black.
void renderLine(uint16_t y, uint8_t *buf) {
  const uint16_t rowOffset = y * BYTES_PER_LINE;
  for (uint8_t i = 0; i < BYTES_PER_LINE; ++i) {
    buf[i] = pgm_read_byte(&AIP_IMAGE[rowOffset + i]);
  }
}

// ---------------------------------------------------------------- low-level SPI

void spiSend2(uint8_t a, uint8_t b) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(a);
  SPI.transfer(b);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(10);
}

void writeReg(uint8_t index, uint8_t value) {
  spiSend2(0x70, index);
  spiSend2(0x72, value);
}

void writeRegBuf(uint8_t index, const uint8_t *data, uint8_t len) {
  spiSend2(0x70, index);
  digitalWrite(PIN_CS, LOW);
  for (uint8_t i = 0; i < len; ++i) SPI.transfer(data[i]);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(10);
}

uint8_t readReg(uint8_t index) {
  spiSend2(0x70, index);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x73);
  uint8_t r = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(10);
  return r;
}

uint8_t readCogId() {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x71);
  uint8_t r = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  return r;
}

bool waitWhileBusy(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (digitalRead(PIN_BUSY) == HIGH) {
    if (millis() - start > timeoutMs) return false;
    delay(1);
  }
  return true;
}

// ---------------------------------------------------------------- G2 COG driver

bool epdPowerOn() {
  digitalWrite(PIN_IO_EN, LOW);
  digitalWrite(PIN_RST, LOW);
  digitalWrite(PIN_EN, LOW);
  digitalWrite(PIN_DISCHARGE, LOW);
  digitalWrite(PIN_BORDER, LOW);
  digitalWrite(PIN_CS, LOW);
  SPI.begin();
  SPI.beginTransaction(spiSettings);
  delay(5);

  digitalWrite(PIN_EN, HIGH);
  delay(10);
  digitalWrite(PIN_RST, HIGH);
  digitalWrite(PIN_BORDER, HIGH);
  digitalWrite(PIN_CS, HIGH);
  delay(5);
  digitalWrite(PIN_RST, LOW);
  delay(5);
  digitalWrite(PIN_RST, HIGH);
  delay(5);

  if (!waitWhileBusy(1000)) { Serial.println(F("BUSY timeout")); return false; }

  readCogId();
  uint8_t cogId = readCogId();
  if ((cogId & 0x0f) != 0x02) {
    Serial.print(F("Unexpected COG ID: 0x")); Serial.println(cogId, HEX);
    return false;
  }

  writeReg(0x02, 0x40);  // disable OE
  if ((readReg(0x0f) & 0x80) == 0) { Serial.println(F("Panel broken")); return false; }

  writeReg(0x0b, 0x02);  // power saving mode
  writeRegBuf(0x01, CHANNEL_SELECT, sizeof(CHANNEL_SELECT));
  writeReg(0x07, 0xd1);  // high power mode oscillator
  writeReg(0x08, 0x02);  // power setting
  writeReg(0x09, 0xc2);  // Vcom level
  writeReg(0x04, 0x03);  // power setting
  writeReg(0x03, 0x01);  // driver latch on
  writeReg(0x03, 0x00);  // driver latch off
  delay(5);

  for (uint8_t i = 0; i < 4; ++i) {
    writeReg(0x05, 0x01);  // positive charge pump
    delay(240);
    writeReg(0x05, 0x03);  // negative charge pump
    delay(40);
    writeReg(0x05, 0x0f);  // Vcom charge pump
    delay(40);
    if (readReg(0x0f) & 0x40) return true;  // DC/DC ok
  }
  writeReg(0x02, 0x40);
  Serial.println(F("DC/DC failed"));
  return false;
}

// line < 0 sends a dummy line (no scan line selected)
void sendLine(int16_t line, const uint8_t *data, uint8_t fixedValue, Stage stage) {
  spiSend2(0x70, 0x0a);
  delayMicroseconds(10);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x72);
  SPI.transfer(0x00);  // border byte

  // odd pixels, right to left
  for (uint8_t b = BYTES_PER_LINE; b > 0; --b) {
    uint8_t p = fixedValue;
    if (data) {
      p = data[b - 1] & 0x55;
      switch (stage) {
        case COMPENSATE: p = 0xaa | (p ^ 0x55); break;
        case WHITE:      p = 0x55 + (p ^ 0x55); break;
        case INVERSE:    p = 0x55 | ((p ^ 0x55) << 1); break;
        case NORMAL:     p = 0xaa | p; break;
      }
      p = ((p >> 6) & 0x03) | (((p >> 4) & 0x03) << 2) | (((p >> 2) & 0x03) << 4) | ((p & 0x03) << 6);
    }
    SPI.transfer(p);
  }

  // scan bytes
  int16_t scanPos = (line < 0) ? -1 : (HEIGHT - line - 1) >> 2;
  uint8_t scanByte = 0x03 << ((line & 0x03) << 1);
  for (uint8_t b = 0; b < BYTES_PER_SCAN; ++b) {
    SPI.transfer(b == scanPos ? scanByte : 0x00);
  }

  // even pixels, left to right
  for (uint8_t b = 0; b < BYTES_PER_LINE; ++b) {
    uint8_t p = fixedValue;
    if (data) {
      p = data[b] & 0xaa;
      switch (stage) {
        case COMPENSATE: p = 0xaa | ((p ^ 0xaa) >> 1); break;
        case WHITE:      p = 0x55 + ((p ^ 0xaa) >> 1); break;
        case INVERSE:    p = 0x55 | (p ^ 0xaa); break;
        case NORMAL:     p = 0xaa | (p >> 1); break;
      }
    }
    SPI.transfer(p);
  }

  digitalWrite(PIN_CS, HIGH);
  writeReg(0x02, 0x07);  // output data to panel
}

void frame(bool useImage, uint8_t fixedValue, Stage stage) {
  uint8_t buf[BYTES_PER_LINE];
  writeReg(0x04, VOLTAGE_LEVEL);
  for (uint16_t line = 0; line < HEIGHT; ++line) {
    if (useImage) renderLine(line, buf);
    sendLine(line, useImage ? buf : nullptr, fixedValue, stage);
  }
}

void frameRepeat(bool useImage, uint8_t fixedValue, Stage stage) {
  unsigned long start = millis();
  do {
    frame(useImage, fixedValue, stage);
  } while (millis() - start < STAGE_TIME_MS);
}

void epdPowerOff() {
  frame(false, 0x00, NORMAL);  // nothing frame

  // 2.7" panel: pulse border pin instead of border dummy line
  delay(25);
  digitalWrite(PIN_BORDER, LOW);
  delay(200);
  digitalWrite(PIN_BORDER, HIGH);

  if ((readReg(0x0f) & 0x40) == 0) Serial.println(F("DC/DC failed at power off"));

  writeReg(0x03, 0x01);  // latch reset on
  writeReg(0x02, 0x05);  // output enable off
  writeReg(0x05, 0x0e);  // Vcom charge pump off
  writeReg(0x05, 0x02);  // negative charge pump off
  writeReg(0x04, 0x0c);  // discharge
  delay(120);
  writeReg(0x05, 0x00);  // all charge pumps off
  writeReg(0x07, 0x0d);  // oscillator off
  writeReg(0x04, 0x50);  // discharge internal
  delay(40);
  writeReg(0x04, 0xa0);
  delay(40);
  writeReg(0x04, 0x00);

  digitalWrite(PIN_RST, LOW);
  digitalWrite(PIN_EN, LOW);
  digitalWrite(PIN_BORDER, LOW);

  SPI.endTransaction();
  SPI.end();
  pinMode(MOSI, OUTPUT); digitalWrite(MOSI, LOW);
  pinMode(SCK, OUTPUT);  digitalWrite(SCK, LOW);
  digitalWrite(PIN_CS, LOW);

  digitalWrite(PIN_DISCHARGE, HIGH);
  delay(150);
  digitalWrite(PIN_DISCHARGE, LOW);
  digitalWrite(PIN_IO_EN, HIGH);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_DISCHARGE, OUTPUT);
  pinMode(PIN_BORDER, OUTPUT);
  pinMode(PIN_IO_EN, OUTPUT);
  pinMode(PIN_BUSY, INPUT);

  if (!epdPowerOn()) {
    epdPowerOff();
    return;
  }

  // Old content is unknown: flash to black, then white, then draw the new image
  frameRepeat(false, 0xff, COMPENSATE);
  frameRepeat(false, 0xaa, WHITE);
  frameRepeat(true, 0x00, INVERSE);
  frameRepeat(true, 0x00, NORMAL);

  epdPowerOff();
  Serial.println(F("Done"));
}

void loop() {
}