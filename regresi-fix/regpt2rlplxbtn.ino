/*
  Sistem Instrumentasi & Metode Pengukuran – 2025
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ---------- Konstanta ---------- */
#define SELANG_WAKTU_SAMPLING   100      
#define SELANG_WAKTU_LCD        1000     
#define SELANG_WAKTU_DEBOUNCE   50       
#define DEADBAND_IN_CELCIUS     1.0F
#define RESOLUSI_SETPOINT_C     1.0F

#define BUTTON_INC              2
#define BUTTON_DEC              7
#define HEATER_PIN              9
#define PIN_ADC                 A0

/* ---------- Variabel ---------- */
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned int ui_ADC[10] = {0};
unsigned char uc_arrayIndex      = 0;
float f_ADCMovAvg                = 0;

unsigned long ul_SampTimeMS      = 0;
unsigned long ul_LCDTimeMS       = 0;
unsigned long ul_DebounceMS_inc  = 0;
unsigned long ul_DebounceMS_dec  = 0;

bool  lastButtonState_inc = LOW;
bool  lastButtonState_dec = LOW;
bool  currentState        = LOW;

float f_TempFilter_C      = 0;
float f_SetPoint_C        = 75.0F;
float f_Error_C           = 0;
const float f_HalfDeadband_C = DEADBAND_IN_CELCIUS / 2.0F;

/* ---------- Fungsi konversi ---------- */
float fGetTemperature(float adc_value)
{
  const float a = 8.1e-5, b = 0.0257, c = 13.8;
  return a * adc_value * adc_value + b * adc_value + c;
}

/* ---------- Kontrol Heater ---------- */
inline void NyalakanHeater() { digitalWrite(HEATER_PIN, LOW);  }
inline void MatikanHeater()  { digitalWrite(HEATER_PIN, HIGH); }

/* ================= SET-UP ================= */
void setup()
{
  pinMode(PIN_ADC, INPUT);
  pinMode(BUTTON_INC, INPUT);
  pinMode(BUTTON_DEC, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
  MatikanHeater();

  lcd.init();
  lcd.backlight();

  Serial.begin(19200);                        
  delay(200);
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,Date,Time,Temp,Setpoint,SSR,millis");

  lcd.setCursor(0, 0); lcd.print("Suhu: ");
  lcd.setCursor(0, 1); lcd.print("Set : ");
}

/* ================= LOOP ================== */
void loop()
{
  /* ---- 1. Sampling & Kontrol ---- */
  if (millis() - ul_SampTimeMS >= SELANG_WAKTU_SAMPLING) {
    ul_SampTimeMS = millis();

    ui_ADC[uc_arrayIndex] = analogRead(PIN_ADC);
    uc_arrayIndex = (uc_arrayIndex + 1) % 10;

    unsigned long sum = 0;
    for (byte i = 0; i < 10; i++) sum += ui_ADC[i];
    f_ADCMovAvg   = (float)sum / 10.0F;
    f_TempFilter_C = fGetTemperature(f_ADCMovAvg);

    f_Error_C = f_SetPoint_C - f_TempFilter_C;
    if (f_Error_C >  f_HalfDeadband_C) NyalakanHeater();
    if (f_Error_C < -f_HalfDeadband_C) MatikanHeater();
  }

  /* ---- 2. Tombol naik SP ---- */
  currentState = digitalRead(BUTTON_INC);
  if ( currentState && !lastButtonState_inc &&
       millis() - ul_DebounceMS_inc >= SELANG_WAKTU_DEBOUNCE ) {
    ul_DebounceMS_inc = millis();
    f_SetPoint_C += RESOLUSI_SETPOINT_C;
    Serial.println("SP Increased!");
  }
  lastButtonState_inc = currentState;

  /* ---- 3. Tombol turun SP ---- */
  currentState = digitalRead(BUTTON_DEC);
  if ( currentState && !lastButtonState_dec &&
       millis() - ul_DebounceMS_dec >= SELANG_WAKTU_DEBOUNCE ) {
    ul_DebounceMS_dec = millis();
    f_SetPoint_C -= RESOLUSI_SETPOINT_C;
    Serial.println("SP Decreased!");
  }
  lastButtonState_dec = currentState;

  /* ---- 4. LCD & PLX-DAQ setiap 1 s ---- */
  if (millis() - ul_LCDTimeMS >= SELANG_WAKTU_LCD) {
    ul_LCDTimeMS = millis();
    char deg = 223;                              

    /* LCD */
    lcd.setCursor(6, 0); lcd.print("       ");
    lcd.setCursor(6, 0); lcd.print(f_TempFilter_C, 2);
    lcd.print(deg); lcd.print("C");

    lcd.setCursor(6, 1); lcd.print("       ");
    lcd.setCursor(6, 1); lcd.print(f_SetPoint_C, 2);
    lcd.print(deg); lcd.print("C ");
    lcd.print(digitalRead(HEATER_PIN) == LOW ? "ON " : "OFF");

    /* PLX-DAQ */
    Serial.print("DATA,DATE,TIME,");
    Serial.print(f_TempFilter_C); Serial.print(",");
    Serial.print(f_SetPoint_C, 2);   Serial.print(",");
    Serial.print(digitalRead(HEATER_PIN) == LOW ? "ON" : "OFF"); Serial.print(",");
    Serial.println(millis());
  }
}
