#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C dan pin
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define SENSOR_PIN A0
#define SSR_PIN 8
#define BUTTON_HOLD 7
#define BUTTON_MODE 2

#define JUMLAH_SAMPLE_ADC 20
#define DEBOUNCE_DELAY 50
#define LCD_UPDATE_DELAY 1000
#define ADC_SAMPLE_DELAY 100

#define DEADBAND 5.0f
#define SETPOINT_STEP 1.0f

float setpoint = 70.0f;
float suhu = 0;
float error = 0;
float halfDeadband = DEADBAND / 2;

unsigned int adcBuffer[JUMLAH_SAMPLE_ADC] = {0};
unsigned char adcIndex = 0;
float adcAvg = 0;
float fGetTemperature(float adc) {
  float a = 1.53e-5;
  float b = 0.1012;
  float c = -5.25;
  return a * adc * adc + b * adc + c;
}

// Mode: 0 = idle, 1 = tambah, 2 = kurang
int mode = 0;
bool lcdHold = false;
bool ssrState = false;

unsigned long lastAdcTime = 0;
unsigned long lastLcdTime = 0;
unsigned long lastDebounceTimeHold = 0;
unsigned long lastDebounceTimeMode = 0;
bool lastHoldState = HIGH;
bool lastModeState = HIGH;

void setup() {
  pinMode(SENSOR_PIN, INPUT);
  pinMode(SSR_PIN, OUTPUT);
  pinMode(BUTTON_HOLD, INPUT_PULLUP);
  pinMode(BUTTON_MODE, INPUT_PULLUP);

  digitalWrite(SSR_PIN, HIGH); // SSR OFF awal (karena LOW = ON untuk SSR)

  lcd.init();
  lcd.backlight();

  Serial.begin(19200);
  Serial.println("Kontrol Temperatur - Gabungan");
}

void loop() {
  // ----------- Sampling ADC -------------
  if (millis() - lastAdcTime >= ADC_SAMPLE_DELAY) {
    lastAdcTime = millis();
    adcBuffer[adcIndex] = analogRead(SENSOR_PIN);
    adcIndex = (adcIndex + 1) % JUMLAH_SAMPLE_ADC;

    unsigned long total = 0;
    for (int i = 0; i < JUMLAH_SAMPLE_ADC; i++) {
      total += adcBuffer[i];
    }
    adcAvg = (float)total / JUMLAH_SAMPLE_ADC;
    suhu = fGetTemperature(adcAvg);
  }

  // ----------- Kontrol SSR (Hysteresis) -------------
  error = setpoint - suhu;
  if (error > halfDeadband) {
    digitalWrite(SSR_PIN, LOW);  // ON (aktif jika LOW)
    ssrState = true;
  } else if (error < -halfDeadband) {
    digitalWrite(SSR_PIN, HIGH); // OFF
    ssrState = false;
  }

  // ----------- Tombol HOLD (pin 7) -------------
  bool currentHoldState = digitalRead(BUTTON_HOLD);
  if (currentHoldState != lastHoldState && millis() - lastDebounceTimeHold > DEBOUNCE_DELAY) {
    lastDebounceTimeHold = millis();
    if (currentHoldState == LOW) {
      lcdHold = !lcdHold;
    }
  }
  lastHoldState = currentHoldState;

  // ----------- Tombol MODE (pin 2) -------------
  bool currentModeState = digitalRead(BUTTON_MODE);
  if (currentModeState != lastModeState && millis() - lastDebounceTimeMode > DEBOUNCE_DELAY) {
    lastDebounceTimeMode = millis();
    if (currentModeState == LOW) {
      mode = (mode + 1) % 3;
      if (mode == 1) {
        setpoint += SETPOINT_STEP;
      } else if (mode == 2) {
        setpoint -= SETPOINT_STEP;
      }
    }
  }
  lastModeState = currentModeState;

  // ----------- Update LCD -------------
  if (!lcdHold && millis() - lastLcdTime >= LCD_UPDATE_DELAY) {
    lastLcdTime = millis();
    lcd.setCursor(0, 0);
    lcd.print("Suhu: ");
    lcd.print(suhu, 1);
    lcd.print((char)223); // Derajat
    lcd.print("C    ");

    lcd.setCursor(0, 1);
    lcd.print("Set : ");
    lcd.print(setpoint, 1);
    lcd.print((char)223);
    lcd.print("C ");
    lcd.print(ssrState ? "ON " : "OFF");

    lcd.setCursor(15, 1);
    if (lcdHold) lcd.print("H");
    else lcd.print(" ");
  }

  // Serial log
  Serial.print("Suhu: ");
  Serial.print(suhu);
  Serial.print(" C | Set: ");
  Serial.print(setpoint);
  Serial.print(" C | SSR: ");
  Serial.println(ssrState ? "ON" : "OFF");
}
