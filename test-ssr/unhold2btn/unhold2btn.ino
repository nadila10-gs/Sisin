#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define SENSOR_PIN A0
#define SSR_PIN 8
#define BUTTON_NAIK 7
#define BUTTON_TURUN 2

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

bool kontrolAktif = false;
bool ssrState = false;

unsigned long lastAdcTime = 0;
unsigned long lastLcdTime = 0;
unsigned long lastDebounceNaik = 0;
unsigned long lastDebounceTurun = 0;

void setup() {
  pinMode(SENSOR_PIN, INPUT);
  pinMode(SSR_PIN, OUTPUT);
  pinMode(BUTTON_NAIK, INPUT_PULLUP);
  pinMode(BUTTON_TURUN, INPUT_PULLUP);

  digitalWrite(SSR_PIN, HIGH); // SSR OFF awal (karena LOW = ON)

  lcd.init();
  lcd.backlight();

  Serial.begin(19200);
  Serial.println("Kontrol Temperatur - Revisi Dua Tombol");
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

  // ----------- Baca Tombol -------------
  bool tombolNaik = digitalRead(BUTTON_NAIK) == LOW;
  bool tombolTurun = digitalRead(BUTTON_TURUN) == LOW;

  // ----------- Mulai Kontrol jika kedua tombol ditekan -------------
  if (tombolNaik && tombolTurun) {
    kontrolAktif = true;
  }

  // ----------- Penyesuaian Setpoint -------------
  if (tombolNaik && !tombolTurun && millis() - lastDebounceNaik > DEBOUNCE_DELAY) {
    lastDebounceNaik = millis();
    setpoint += SETPOINT_STEP;
  }

  if (tombolTurun && !tombolNaik && millis() - lastDebounceTurun > DEBOUNCE_DELAY) {
    lastDebounceTurun = millis();
    setpoint -= SETPOINT_STEP;
  }

  // ----------- Kontrol SSR (jika aktif) -------------
  if (kontrolAktif) {
    error = setpoint - suhu;
    if (error > halfDeadband) {
      digitalWrite(SSR_PIN, LOW);  // SSR ON
      ssrState = true;
    } else if (error < -halfDeadband) {
      digitalWrite(SSR_PIN, HIGH); // SSR OFF
      ssrState = false;
    }
  } else {
    digitalWrite(SSR_PIN, HIGH); // SSR OFF jika belum mulai
    ssrState = false;
  }

  // ----------- Update LCD -------------
  if (millis() - lastLcdTime >= LCD_UPDATE_DELAY) {
    lastLcdTime = millis();
    lcd.setCursor(0, 0);
    lcd.print("Suhu: ");
    lcd.print(suhu, 1);
    lcd.print((char)223); // simbol derajat
    lcd.print("C    ");

    lcd.setCursor(0, 1);
    lcd.print("Set : ");
    lcd.print(setpoint, 1);
    lcd.print((char)223);
    lcd.print("C ");
    lcd.print(ssrState ? "ON " : "OFF");
  }

  // ----------- Serial Log -------------
  Serial.print("Suhu: ");
  Serial.print(suhu);
  Serial.print(" C | Set: ");
  Serial.print(setpoint);
  Serial.print(" C | SSR: ");
  Serial.println(ssrState ? "ON" : "OFF");
}
