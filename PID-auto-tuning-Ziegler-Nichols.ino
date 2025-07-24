#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#define BUTTON_INC       2
#define BUTTON_DEC       7
#define HEATER_PIN       9
#define PIN_ADC          A0
#define JUMLAH_SAMPLE_MOVAVG     10
#define DEBOUNCE_TIME_MS         50
#define SAMPLING_INTERVAL_MS     100
#define LCD_INTERVAL_MS          1000
#define TPO_PERIOD_MS            2000
#define SETPOINT_RES_C           1.0F
#define DEADBAND_C               0.5F
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned int adc_samples[JUMLAH_SAMPLE_MOVAVG] = {0};
unsigned char index_adc = 0;
float adc_avg = 0, suhu_C = 0, error_C = 0;
unsigned long lastSampleTime = 0;
unsigned long lastLCDTime = 0;
unsigned long millisStart = 0;
float setpoint_C = 75.0;
double pid_input = 0, pid_output = 0, pid_setpoint = 75.0;
PID myPID(&pid_input, &pid_output, &pid_setpoint, 0, 0, 0, DIRECT); 
unsigned long lastTPOBase = 0;
bool heaterOn = false;
float getTempFromADC(float adc_val) {
  const float a = 8.1e-5;
  const float b = 0.0257;
  const float c = 13.8;
  return a * adc_val * adc_val + b * adc_val + c;
}

void nyalakanHeater()  { digitalWrite(HEATER_PIN, LOW);  heaterOn = true; }
void matikanHeater()   { digitalWrite(HEATER_PIN, HIGH); heaterOn = false; }
void doZNTuning() {
  myPID.SetMode(MANUAL);
  double Ku = 45.0;
  double Pu = 6.0;
  double Kp_z = 0.6 * Ku;
  double Ki_z = 2 * Kp_z / Pu;
  double Kd_z = Kp_z * Pu / 8;
  myPID.SetTunings(Kp_z, Ki_z, Kd_z);
  myPID.SetMode(AUTOMATIC);
  Serial.println("=== ZN TUNING ===");
  Serial.print("Kp = "); Serial.println(Kp_z);
  Serial.print("Ki = "); Serial.println(Ki_z);
  Serial.print("Kd = "); Serial.println(Kd_z);
}

void setup() {
  pinMode(PIN_ADC, INPUT);
  pinMode(BUTTON_INC, INPUT);
  pinMode(BUTTON_DEC, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
  matikanHeater();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Suhu: ");
  lcd.setCursor(0, 1); lcd.print("Set : ");
  Serial.begin(19200);
  delay(200);
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,Waktu (s),Suhu,Setpoint,Heater,PID_output");
  doZNTuning(); 
  myPID.SetOutputLimits(0, TPO_PERIOD_MS);
  millisStart = millis();
}


void loop() {
  unsigned long now = millis();
  	  if (now - lastSampleTime >= SAMPLING_INTERVAL_MS) {
    lastSampleTime = now;
    adc_samples[index_adc] = analogRead(PIN_ADC);
    index_adc = (index_adc + 1) % JUMLAH_SAMPLE_MOVAVG;
    unsigned long total = 0;
    for (int i = 0; i < JUMLAH_SAMPLE_MOVAVG; i++) total += adc_samples[i];
    adc_avg = total / (float)JUMLAH_SAMPLE_MOVAVG;
    suhu_C = getTempFromADC(adc_avg);
    error_C = setpoint_C - suhu_C;

    pid_input = suhu_C;
    pid_setpoint = setpoint_C;
    myPID.Compute();
    if (now - lastTPOBase >= TPO_PERIOD_MS) {
      lastTPOBase = now;
      if (pid_output > 0) nyalakanHeater();
      else matikanHeater();
    }
    if ((now - lastTPOBase) >= pid_output && heaterOn) {
      matikanHeater();
    }
  }
  static unsigned long lastDebounceUp = 0, lastDebounceDown = 0;
  static bool lastStateUp = LOW, lastStateDown = LOW;
  bool currentUp = digitalRead(BUTTON_INC);
  bool currentDown = digitalRead(BUTTON_DEC);
  if (currentUp && !lastStateUp && (now - lastDebounceUp > DEBOUNCE_TIME_MS)) {
    setpoint_C += SETPOINT_RES_C;
    lastDebounceUp = now;
  }
  if (currentDown && !lastStateDown && (now - lastDebounceDown > DEBOUNCE_TIME_MS)) {
    setpoint_C -= SETPOINT_RES_C;
    lastDebounceDown = now;
  }
  lastStateUp = currentUp;
  lastStateDown = currentDown;
  if (now - lastLCDTime >= LCD_INTERVAL_MS) {
    lastLCDTime = now;
    char deg = 223;
    lcd.setCursor(6, 0); lcd.print("       ");
    lcd.setCursor(6, 0); lcd.print(suhu_C, 2); lcd.print(deg); lcd.print("C");

    lcd.setCursor(6, 1); lcd.print("       ");
    lcd.setCursor(6, 1); lcd.print(setpoint_C, 2); lcd.print(deg); lcd.print("C ");
    lcd.print(heaterOn ? "ON " : "OFF");
    float seconds = (now - millisStart) / 1000.0;
    Serial.print("DATA,");
    Serial.print(seconds, 1); Serial.print(",");
    Serial.print(suhu_C, 2); Serial.print(",");
    Serial.print(setpoint_C, 2); Serial.print(",");
    Serial.print(heaterOn ? "ON" : "OFF"); Serial.print(",");
    Serial.println(pid_output, 2);
  }
}
