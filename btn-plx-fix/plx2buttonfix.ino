#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- Pin mapping ----------
#define SENSOR_PIN     A0
#define SSR_PIN        8
#define BUTTON_NAIK    7
#define BUTTON_TURUN   2

// ---------- Konstanta sistem ----------
#define JUMLAH_SAMPLE_ADC  20
#define DEBOUNCE_DELAY     50
#define LCD_UPDATE_DELAY   1000   // ms
#define ADC_SAMPLE_DELAY   100    // ms
#define DEADBAND           5.0f
#define SETPOINT_STEP      1.0f

// ---------- Variabel kontrol ----------
float setpoint      = 70.0f;
float suhu          = 0;
float halfDeadband  = DEADBAND / 2;
bool  kontrolAktif  = false;
bool  ssrState      = false;

// ---------- Buffer ADC ----------
unsigned int adcBuffer[JUMLAH_SAMPLE_ADC] = {0};
byte         adcIndex     = 0;

// ---------- Waktu ----------
unsigned long lastAdcTime     = 0;
unsigned long lastLcdTime     = 0;
unsigned long lastDebounceUp  = 0;
unsigned long lastDebounceDn  = 0;

// ---------- Konversi ADC→°C  (quadratic) ----------
float fGetTemperature(int adc)
{
  const float a = 1.53e-5;
  const float b = 0.1012;
  const float c = -5.25;
  return a * adc * adc + b * adc + c;
}

void setup()
{
  pinMode(SENSOR_PIN,   INPUT);
  pinMode(SSR_PIN,      OUTPUT);
  pinMode(BUTTON_NAIK,  INPUT_PULLUP);
  pinMode(BUTTON_TURUN, INPUT_PULLUP);

  digitalWrite(SSR_PIN, HIGH);          // SSR OFF (aktif LOW)

  lcd.init();
  lcd.backlight();

  // ---------- Serial / PLX-DAQ ----------
  Serial.begin(9600);                   // baud default PLX-DAQ
  delay(200);                           // beri waktu koneksi
  Serial.println("CLEARSHEET");         // bersihkan dari baris 1
  Serial.println("LABEL,Date,Time,Temp,Setpoint,SSR,millis");

  // (Opsional) set label checkbox kalau mau:
  // Serial.println("CUSTOMBOX1,LABEL,Stop log?");
}

void loop()
{
  // ====== 1. Sampling ADC ======
  if (millis() - lastAdcTime >= ADC_SAMPLE_DELAY) {
    lastAdcTime = millis();

    adcBuffer[adcIndex] = analogRead(SENSOR_PIN);
    adcIndex = (adcIndex + 1) % JUMLAH_SAMPLE_ADC;

    unsigned long total = 0;
    for (byte i = 0; i < JUMLAH_SAMPLE_ADC; i++) total += adcBuffer[i];
    suhu = fGetTemperature(total / JUMLAH_SAMPLE_ADC);
  }

  // ====== 2. Baca tombol ======
  bool upPressed   = digitalRead(BUTTON_NAIK)  == LOW;
  bool dnPressed   = digitalRead(BUTTON_TURUN) == LOW;

  // start kontrol bila dua-duanya ditekan
  if (upPressed && dnPressed) kontrolAktif = true;

  if (upPressed && !dnPressed && millis() - lastDebounceUp > DEBOUNCE_DELAY) {
    lastDebounceUp = millis();
    setpoint += SETPOINT_STEP;
  }

  if (dnPressed && !upPressed && millis() - lastDebounceDn > DEBOUNCE_DELAY) {
    lastDebounceDn = millis();
    setpoint -= SETPOINT_STEP;
  }

  // ====== 3. Kontrol SSR (jika aktif) ======
  if (kontrolAktif) {
    float error = setpoint - suhu;
    if (error >  halfDeadband) { digitalWrite(SSR_PIN, LOW);  ssrState = true;  }
    if (error < -halfDeadband) { digitalWrite(SSR_PIN, HIGH); ssrState = false; }
  } else {
    digitalWrite(SSR_PIN, HIGH);
    ssrState = false;
  }

  // ====== 4. Update LCD & kirim ke PLX-DAQ ======
  if (millis() - lastLcdTime >= LCD_UPDATE_DELAY) {
    lastLcdTime = millis();

    // ---- LCD ----
    lcd.setCursor(0, 0);
    lcd.print("Suhu: "); lcd.print(suhu, 1); lcd.print((char)223); lcd.print("C   ");

    lcd.setCursor(0, 1);
    lcd.print("Set : "); lcd.print(setpoint, 1); lcd.print((char)223); lcd.print("C ");
    lcd.print(ssrState ? "ON " : "OFF");

    // ---- PLX-DAQ (CSV) ----
    Serial.print("DATA,DATE,TIME,");
    Serial.print(suhu, 2);        Serial.print(",");
    Serial.print(setpoint, 2);    Serial.print(",");
    Serial.print(ssrState ? "ON" : "OFF"); Serial.print(",");
    Serial.println(millis());
  }
}
