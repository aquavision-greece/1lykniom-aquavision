#include <Wire.h>
#include <OneWire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>

// ---------- Pins ----------
#define ORP_PIN A1
#define PH_PIN A0
#define TDS_PIN A2
#define TURBIDITY_PIN A3
#define BUTTON_PIN 4
#define DS18S20_PIN 2

// ---------- Objects ----------
hd44780_I2Cexp lcd;
OneWire ds(DS18S20_PIN);
DFRobot_PH ph;

// ---------- Constants ----------
#define VOLTAGE 5.00
#define TDS_VREF 5.0
#define ORP_ARRAY_LENGTH 40
#define TDS_SCOUNT 30

// ---------- Buffers ----------
int orpArray[ORP_ARRAY_LENGTH];
int tdsBuffer[TDS_SCOUNT];

int orpIndex = 0;
int tdsIndex = 0;

// ---------- Variables ----------
float temperature = 25.0;
float phVoltage = 0;

float lastPH = 7;
float lastTDS = 0;
float lastTurb = 0;
float lastORP = 0;

int currentSensor = 0;

// =======================================================
// Utility functions
// =======================================================

double averageArray(int *arr, int len) {
  long sum = 0;
  int min = arr[0], max = arr[0];

  for (int i = 0; i < len; i++) {
    if (arr[i] < min) min = arr[i];
    if (arr[i] > max) max = arr[i];
    sum += arr[i];
  }

  sum -= min + max;
  return (double)sum / (len - 2);
}

int getMedian(int *arr, int len) {
  int temp[len];
  memcpy(temp, arr, len * sizeof(int));

  for (int i = 0; i < len - 1; i++) {
    for (int j = i + 1; j < len; j++) {
      if (temp[i] > temp[j]) {
        int t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  return (len % 2) ? temp[len / 2] :
         (temp[len / 2] + temp[len / 2 - 1]) / 2;
}

float getTemperature() {
  byte data[9], addr[8];

  if (!ds.search(addr)) {
    ds.reset_search();
    return temperature;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) return temperature;

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);
  delay(750);

  ds.reset();
  ds.select(addr);
  ds.write(0xBE);

  for (int i = 0; i < 9; i++) data[i] = ds.read();

  int16_t raw = (data[1] << 8) | data[0];
  return raw / 16.0;
}

// =======================================================
// Status logic (per sensor)
// =======================================================

String statusText(int s) {
  if (s == 2) return "GOOD";
  if (s == 1) return "RISKY";
  return "BAD";
}

int orpStatus(float orp) {
  if (orp > 650) return 2;
  if (orp > 500) return 1;
  return 0;
}

int phStatus(float ph) {
  if (ph >= 6.5 && ph <= 8.5) return 2;
  if ((ph >= 5.5 && ph < 6.5) || (ph > 8.5 && ph <= 9)) return 1;
  return 0;
}

int tdsStatus(float tds) {
  if (tds < 300) return 2;
  if (tds < 600) return 1;
  return 0;
}

int turbStatus(float v) {
  if (v < 2.5) return 2;
  if (v < 3.5) return 1;
  return 0;
}

// =======================================================
// Setup
// =======================================================

void setup() {
  Serial.begin(115200);

  lcd.begin(16, 2);
  lcd.clear();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  ph.begin();
}

// =======================================================
// Loop
// =======================================================

void loop() {

  static int lastButton = HIGH;
  static unsigned long lastUpdate = 0;

  // ---------- Button ----------
  int btn = digitalRead(BUTTON_PIN);

  if (btn == LOW && lastButton == HIGH) {
    currentSensor = (currentSensor + 1) % 5;
    lcd.clear();
    delay(200);
  }
  lastButton = btn;

  // ---------- Refresh rate ----------
  if (millis() - lastUpdate < 500) return;
  lastUpdate = millis();

  lcd.setCursor(0, 0);

  switch (currentSensor) {

    // ================= ORP =================
    case 0: {
      orpArray[orpIndex++] = analogRead(ORP_PIN);
      if (orpIndex >= ORP_ARRAY_LENGTH) orpIndex = 0;

      double avg = averageArray(orpArray, ORP_ARRAY_LENGTH);
      double orp = ((30 * VOLTAGE * 1000) -
                   (75 * avg * VOLTAGE * 1000 / 1024)) / 75;

      lastORP = orp;

      lcd.print("ORP:");
      lcd.print((int)orp);
      lcd.setCursor(0, 1);
      lcd.print(statusText(orpStatus(orp)));
      break;
    }

    // ================= pH =================
    case 1: {
      phVoltage = analogRead(PH_PIN) / 1024.0 * 5000;
      float pHValue = ph.readPH(phVoltage, temperature);

      lastPH = pHValue;

      lcd.print("pH:");
      lcd.print(pHValue, 2);
      lcd.setCursor(0, 1);
      lcd.print(statusText(phStatus(pHValue)));
      break;
    }

    // ================= TDS =================
    case 2: {
      tdsBuffer[tdsIndex++] = analogRead(TDS_PIN);
      if (tdsIndex >= TDS_SCOUNT) tdsIndex = 0;

      float v = getMedian(tdsBuffer, TDS_SCOUNT) * TDS_VREF / 1024.0;
      float comp = v / (1.0 + 0.02 * (temperature - 25.0));

      float tds = (133.42 * comp * comp * comp -
                   255.86 * comp * comp +
                   857.39 * comp) * 0.5;

      lastTDS = tds;

      lcd.print("TDS:");
      lcd.print(tds, 0);
      lcd.print("ppm");
      lcd.setCursor(0, 1);
      lcd.print(statusText(tdsStatus(tds)));
      break;
    }

    // ================= Turbidity =================
    case 3: {
      float v = analogRead(TURBIDITY_PIN) * (VOLTAGE / 1024.0);

      lastTurb = v;

      lcd.print("Turb:");
      lcd.print(v, 2);
      lcd.setCursor(0, 1);
      lcd.print(statusText(turbStatus(v)));
      break;
    }

    // ================= Temperature =================
    case 4: {
      temperature = getTemperature();

      lcd.print("Temp:");
      lcd.print(temperature, 1);
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("INFO");
      break;
    }
  }

  // ---------- pH calibration ----------
  ph.calibration(phVoltage, temperature);
}