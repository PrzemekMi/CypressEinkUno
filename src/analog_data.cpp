#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <math.h>

// CY8CKIT-028-EPD shield on an Uno; SDA/SCL are the Uno's A4/A5 pins.
const uint8_t PIN_CS = 10;
const uint8_t PIN_RST = 2;
const uint8_t PIN_BUSY = 3;
const uint8_t PIN_EN = 4;
const uint8_t PIN_DISCHARGE = 5;
const uint8_t PIN_BORDER = 6;
const uint8_t PIN_IO_EN = 7;
const uint8_t PIN_THERM_VDD = A0;
const uint8_t PIN_THERM_OUT = A1;
const uint8_t PIN_THERM_GND = A3;

const uint16_t HEIGHT = 176;
const uint16_t WIDTH = 264;
const uint8_t BYTES_PER_LINE = WIDTH / 8;
const uint8_t BYTES_PER_SCAN = HEIGHT / 4;
const uint8_t CHANNEL_SELECT[] = {0x72, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x00};
const unsigned long STAGE_TIME_MS = 630;
const unsigned long UPDATE_INTERVAL_MS = 60000;
const uint8_t BMI160_CHIP_ID = 0xd1;
const uint8_t BMI160_ADDRESS = 0x68;

enum Stage { COMPENSATE, WHITE, INVERSE, NORMAL };
SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);

const uint8_t SCALE = 3;
const uint8_t FONT_HEIGHT = 7;
const uint8_t CHAR_WIDTH = 6;
const uint8_t MAX_LINE_CHARS = WIDTH / (CHAR_WIDTH * SCALE);
const uint8_t TEXT_Y = 10;
const uint8_t LINE_HEIGHT = 26;
char text[6][16];

// Column-major 5x7 font; bit 0 is the top row.
const char GLYPH_CHARS[] = " +-./:0123456789CDEGMOPRSTXYZ";
const uint8_t GLYPHS[][5] PROGMEM = {
  {0x00, 0x00, 0x00, 0x00, 0x00}, // space
  {0x08, 0x08, 0x3e, 0x08, 0x08}, // +
  {0x08, 0x08, 0x08, 0x08, 0x08}, // -
  {0x00, 0x60, 0x60, 0x00, 0x00}, // .
  {0x20, 0x10, 0x08, 0x04, 0x02}, // /
  {0x00, 0x36, 0x36, 0x00, 0x00}, // :
  {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 0
  {0x00, 0x42, 0x7f, 0x40, 0x00}, // 1
  {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
  {0x21, 0x41, 0x45, 0x4b, 0x31}, // 3
  {0x18, 0x14, 0x12, 0x7f, 0x10}, // 4
  {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
  {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 6
  {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
  {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
  {0x06, 0x49, 0x49, 0x29, 0x1e}, // 9
  {0x3e, 0x41, 0x41, 0x41, 0x22}, // C
  {0x7f, 0x41, 0x41, 0x22, 0x1c}, // D
  {0x7f, 0x49, 0x49, 0x49, 0x41}, // E
  {0x3e, 0x41, 0x49, 0x49, 0x7a}, // G
  {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // M
  {0x3e, 0x41, 0x41, 0x41, 0x3e}, // O
  {0x7f, 0x09, 0x09, 0x09, 0x06}, // P
  {0x7f, 0x09, 0x19, 0x29, 0x46}, // R
  {0x46, 0x49, 0x49, 0x49, 0x31}, // S
  {0x01, 0x01, 0x7f, 0x01, 0x01}, // T
  {0x63, 0x14, 0x08, 0x14, 0x63}, // X
  {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
  {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

uint8_t glyphColumn(char character, uint8_t column) {
  const char *found = strchr(GLYPH_CHARS, character);
  if (!found || character == '\0') return 0;
  return pgm_read_byte(&GLYPHS[found - GLYPH_CHARS][column]);
}

void renderLine(uint16_t line, uint8_t *buffer) {
  memset(buffer, 0, BYTES_PER_LINE);
  for (uint8_t textLine = 0; textLine < 6; ++textLine) {
    uint16_t top = TEXT_Y + textLine * LINE_HEIGHT;
    if (line < top || line >= top + FONT_HEIGHT * SCALE) continue;
    uint8_t row = (line - top) / SCALE;
    uint8_t length = strlen(text[textLine]);
    if (length > MAX_LINE_CHARS) length = MAX_LINE_CHARS;
    uint16_t left = (WIDTH - length * CHAR_WIDTH * SCALE) / 2;
    for (uint8_t character = 0; character < length; ++character) {
      for (uint8_t column = 0; column < 5; ++column) {
        if (!(glyphColumn(text[textLine][character], column) & (1 << row))) continue;
        uint16_t x = left + (character * CHAR_WIDTH + column) * SCALE;
        for (uint8_t pixel = 0; pixel < SCALE; ++pixel) {
          uint16_t position = x + pixel;
          buffer[position >> 3] |= 0x80 >> (position & 7);
        }
      }
    }
  }
}

bool writeImu(uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(BMI160_ADDRESS);
  Wire.write(registerAddress);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readImu(uint8_t registerAddress, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(BMI160_ADDRESS);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BMI160_ADDRESS, length) != length) return false;
  for (uint8_t index = 0; index < length; ++index) buffer[index] = Wire.read();
  return true;
}

bool initImu() {
  uint8_t chipId;
  if (!readImu(0x00, &chipId, 1) || chipId != BMI160_CHIP_ID) return false;
  if (!writeImu(0x7e, 0x15)) return false; // gyro normal mode
  delay(80);
  return writeImu(0x42, 0x28) && writeImu(0x43, 0x03); // 100 Hz, +/-250 dps
}

bool readGyro(int16_t *axes) {
  uint8_t raw[6];
  if (!readImu(0x0c, raw, sizeof(raw))) return false;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    axes[axis] = static_cast<int16_t>(static_cast<uint16_t>(raw[axis * 2]) |
                                      (static_cast<uint16_t>(raw[axis * 2 + 1]) << 8));
  }
  return true;
}

bool readTemperature(int16_t &tenths) {
  digitalWrite(PIN_THERM_GND, LOW);
  digitalWrite(PIN_THERM_VDD, HIGH);
  delay(10);
  analogRead(PIN_THERM_OUT);
  uint16_t sum = 0;
  for (uint8_t sample = 0; sample < 8; ++sample) sum += analogRead(PIN_THERM_OUT);
  digitalWrite(PIN_THERM_VDD, LOW);
  uint16_t reading = (sum + 4) / 8;
  if (reading == 0 || reading >= 1023) return false;
  // Shield wiring: VDD -> 10k reference -> OUT -> 10k NTC -> GND.
  float resistance = 10000.0f * reading / (1023 - reading);
  float kelvin = 1.0f / (1.0f / 298.15f + log(resistance / 10000.0f) / 3380.0f);
  tenths = lround((kelvin - 273.15f) * 10.0f);
  return true;
}

void formatValue(char *destination, size_t size, char axis, int16_t tenths) {
  uint16_t magnitude = tenths < 0 ? -static_cast<int32_t>(tenths) : tenths;
  snprintf(destination, size, "%c: %c%u.%u", axis, tenths < 0 ? '-' : '+',
           magnitude / 10, magnitude % 10);
}

void updateText(bool imuReady) {
  int16_t temperature;
  strcpy(text[0], "TEMP C");
  if (readTemperature(temperature)) formatValue(text[1], sizeof(text[1]), 'T', temperature);
  else strcpy(text[1], "T: --.-");
  strcpy(text[2], "GYRO D/S");
  int16_t axes[3];
  bool valid = imuReady && readGyro(axes);
  const char axisNames[] = "XYZ";
  for (uint8_t axis = 0; axis < 3; ++axis) {
    if (valid) {
      int16_t tenths = lround(axes[axis] / 13.12f);
      formatValue(text[3 + axis], sizeof(text[3 + axis]), axisNames[axis], tenths);
    } else {
      snprintf(text[3 + axis], sizeof(text[3 + axis]), "%c: --.-", axisNames[axis]);
    }
  }
}

void spiSend2(uint8_t first, uint8_t second) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(first);
  SPI.transfer(second);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(10);
}

void writeReg(uint8_t index, uint8_t value) {
  spiSend2(0x70, index);
  spiSend2(0x72, value);
}

void writeRegBuf(uint8_t index, const uint8_t *data, uint8_t length) {
  spiSend2(0x70, index);
  digitalWrite(PIN_CS, LOW);
  for (uint8_t position = 0; position < length; ++position) SPI.transfer(data[position]);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(10);
}

uint8_t readReg(uint8_t index) {
  spiSend2(0x70, index);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x73);
  uint8_t result = SPI.transfer(0);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(10);
  return result;
}

uint8_t readCogId() {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x71);
  uint8_t result = SPI.transfer(0);
  digitalWrite(PIN_CS, HIGH);
  return result;
}

bool waitWhileBusy(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (digitalRead(PIN_BUSY) == HIGH) {
    if (millis() - start > timeoutMs) return false;
    delay(1);
  }
  return true;
}

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

  writeReg(0x02, 0x40);
  if ((readReg(0x0f) & 0x80) == 0) { Serial.println(F("Panel broken")); return false; }
  writeReg(0x0b, 0x02);
  writeRegBuf(0x01, CHANNEL_SELECT, sizeof(CHANNEL_SELECT));
  writeReg(0x07, 0xd1);
  writeReg(0x08, 0x02);
  writeReg(0x09, 0xc2);
  writeReg(0x04, 0x03);
  writeReg(0x03, 0x01);
  writeReg(0x03, 0x00);
  delay(5);

  for (uint8_t attempt = 0; attempt < 4; ++attempt) {
    writeReg(0x05, 0x01);
    delay(240);
    writeReg(0x05, 0x03);
    delay(40);
    writeReg(0x05, 0x0f);
    delay(40);
    if (readReg(0x0f) & 0x40) return true;
  }
  writeReg(0x02, 0x40);
  Serial.println(F("DC/DC failed"));
  return false;
}

void sendLine(int16_t line, const uint8_t *data, uint8_t fixedValue, Stage stage) {
  spiSend2(0x70, 0x0a);
  delayMicroseconds(10);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x72);
  SPI.transfer(0x00);

  for (uint8_t position = BYTES_PER_LINE; position > 0; --position) {
    uint8_t pixel = fixedValue;
    if (data) {
      pixel = data[position - 1] & 0x55;
      switch (stage) {
        case COMPENSATE: pixel = 0xaa | (pixel ^ 0x55); break;
        case WHITE:      pixel = 0x55 + (pixel ^ 0x55); break;
        case INVERSE:    pixel = 0x55 | ((pixel ^ 0x55) << 1); break;
        case NORMAL:     pixel = 0xaa | pixel; break;
      }
      pixel = ((pixel >> 6) & 0x03) | (((pixel >> 4) & 0x03) << 2) |
              (((pixel >> 2) & 0x03) << 4) | ((pixel & 0x03) << 6);
    }
    SPI.transfer(pixel);
  }

  int16_t scanPosition = (HEIGHT - line - 1) >> 2;
  uint8_t scanByte = 0x03 << ((line & 0x03) << 1);
  for (uint8_t position = 0; position < BYTES_PER_SCAN; ++position) {
    SPI.transfer(position == scanPosition ? scanByte : 0x00);
  }

  for (uint8_t position = 0; position < BYTES_PER_LINE; ++position) {
    uint8_t pixel = fixedValue;
    if (data) {
      pixel = data[position] & 0xaa;
      switch (stage) {
        case COMPENSATE: pixel = 0xaa | ((pixel ^ 0xaa) >> 1); break;
        case WHITE:      pixel = 0x55 + ((pixel ^ 0xaa) >> 1); break;
        case INVERSE:    pixel = 0x55 | (pixel ^ 0xaa); break;
        case NORMAL:     pixel = 0xaa | (pixel >> 1); break;
      }
    }
    SPI.transfer(pixel);
  }

  digitalWrite(PIN_CS, HIGH);
  writeReg(0x02, 0x07);
}

void frame(bool useImage, uint8_t fixedValue, Stage stage) {
  uint8_t buffer[BYTES_PER_LINE];
  writeReg(0x04, 0x00);
  for (uint16_t line = 0; line < HEIGHT; ++line) {
    if (useImage) renderLine(line, buffer);
    sendLine(line, useImage ? buffer : nullptr, fixedValue, stage);
  }
}

void frameRepeat(bool useImage, uint8_t fixedValue, Stage stage) {
  unsigned long start = millis();
  do {
    frame(useImage, fixedValue, stage);
  } while (millis() - start < STAGE_TIME_MS);
}

void epdPowerOff() {
  frame(false, 0x00, NORMAL);
  delay(25);
  digitalWrite(PIN_BORDER, LOW);
  delay(200);
  digitalWrite(PIN_BORDER, HIGH);
  if ((readReg(0x0f) & 0x40) == 0) Serial.println(F("DC/DC failed at power off"));
  writeReg(0x03, 0x01);
  writeReg(0x02, 0x05);
  writeReg(0x05, 0x0e);
  writeReg(0x05, 0x02);
  writeReg(0x04, 0x0c);
  delay(120);
  writeReg(0x05, 0x00);
  writeReg(0x07, 0x0d);
  writeReg(0x04, 0x50);
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
  pinMode(SCK, OUTPUT); digitalWrite(SCK, LOW);
  digitalWrite(PIN_CS, LOW);
  digitalWrite(PIN_DISCHARGE, HIGH);
  delay(150);
  digitalWrite(PIN_DISCHARGE, LOW);
  digitalWrite(PIN_IO_EN, HIGH);
}

void refreshDisplay() {
  if (!epdPowerOn()) {
    digitalWrite(PIN_RST, LOW);
    digitalWrite(PIN_EN, LOW);
    digitalWrite(PIN_IO_EN, HIGH);
    SPI.endTransaction();
    SPI.end();
    return;
  }
  frameRepeat(false, 0xff, COMPENSATE);
  frameRepeat(false, 0xaa, WHITE);
  frameRepeat(true, 0x00, INVERSE);
  frameRepeat(true, 0x00, NORMAL);
  epdPowerOff();
}

bool imuReady;
unsigned long lastUpdate;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_DISCHARGE, OUTPUT);
  pinMode(PIN_BORDER, OUTPUT);
  pinMode(PIN_IO_EN, OUTPUT);
  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_THERM_VDD, OUTPUT);
  pinMode(PIN_THERM_GND, OUTPUT);
  digitalWrite(PIN_THERM_VDD, LOW);
  digitalWrite(PIN_THERM_GND, LOW);
  Wire.begin();
  // AVR Wire enables 5 V pull-ups; use external 3.3 V I2C pull-ups instead.
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  imuReady = initImu();
  if (!imuReady) Serial.println(F("BMI160 not found"));
  updateText(imuReady);
  refreshDisplay();
  lastUpdate = millis();
}

void loop() {
  if (millis() - lastUpdate < UPDATE_INTERVAL_MS) return;
  lastUpdate = millis();
  if (!imuReady) imuReady = initImu();
  updateText(imuReady);
  refreshDisplay();
}