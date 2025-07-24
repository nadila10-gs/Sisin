#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#define SELANG_WAKTU_LCD           1000     
#define SELANG_WAKTU_DEBOUNCE      50       
#define SELANG_WAKTU_SAMPLING      100
#define RESOLUSI_SETPOINT_C        1.0F
#define DEADBAND_IN_CELCIUS        1.0F
#define JUMLAH_SAMPLE_MOVAVG       10
#define BUTTON_INC                 2
#define BUTTON_DEC                 7
#define HEATER_PIN                 9
#define PIN_ADC                    A0
unsigned int  ui_ADC[JUMLAH_SAMPLE_MOVAVG] = {0};
unsigned char uc_arrayIndex     = 0;
float         f_ADCMovAvg       = 0;
float         f_TempFilter_C    = 0;
float         f_Error_C         = 0;
unsigned long ul_SampTimeMS     = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long ul_LCDTimeMS      = 0;
unsigned long ul_Debounce_incMS = 0;
unsigned long ul_Debounce_decMS = 0;
bool          lastBtn_inc       = LOW;
bool          lastBtn_dec       = LOW;
bool          currentState      = LOW;

float f_SetPoint_C       = 75.0F;
double pid_input    = 0;
double pid_output   = 0;
double pid_setpoint = 75.0;
double Kp = 65.050;
double Ki = 0.008;
double Kd = 1.504;

PID myPID(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);

const unsigned long TPO_PERIOD     = 2000; 
unsigned long       ul_TPOLastBase = 0;

float fGetTemperature(float adc_value)
{ 
  const float a = 8.1e-5;
  const float b = 0.0257;
  const float c = 13.8;
  return a * adc_value * adc_value + b * adc_value + c;
}

inline void NyalakanHeater() { digitalWrite(HEATER_PIN, LOW);  }
inline void MatikanHeater()  { digitalWrite(HEATER_PIN, HIGH); }
void setup()
{
  pinMode(PIN_ADC, INPUT);
  pinMode(BUTTON_INC, INPUT);
  pinMode(BUTTON_DEC, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
  MatikanHeater();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Suhu: ");
  lcd.setCursor(0, 1); lcd.print("Set : ");
  Serial.begin(19200);                        
  delay(200);
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,Date,Time,Temp,Setpoint,SSR,PID_out,millis");
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, TPO_PERIOD);
}

void loop()
{
    if (millis() - ul_SampTimeMS >= SELANG_WAKTU_SAMPLING) {
    ul_SampTimeMS = millis();
    ui_ADC[uc_arrayIndex] = analogRead(PIN_ADC);
    uc_arrayIndex = (uc_arrayIndex + 1) % JUMLAH_SAMPLE_MOVAVG;
    unsigned long ul_total = 0;
    for (byte i = 0; i < JUMLAH_SAMPLE_MOVAVG; i++) ul_total += ui_ADC[i];
    f_ADCMovAvg     = (float)ul_total / JUMLAH_SAMPLE_MOVAVG;
    f_TempFilter_C  = fGetTemperature(f_ADCMovAvg);
    f_Error_C       = f_SetPoint_C - f_TempFilter_C;
    pid_input       = f_TempFilter_C;
    pid_setpoint    = f_SetPoint_C;
    myPID.Compute();

    if (millis() - ul_TPOLastBase >= TPO_PERIOD) {
      ul_TPOLastBase = millis();
      if (pid_output > 0) NyalakanHeater();
      else                MatikanHeater();
    }

    if ((millis() - ul_TPOLastBase) >= pid_output && digitalRead(HEATER_PIN) == LOW) {
      MatikanHeater();
    }
  }

  currentState = digitalRead(BUTTON_INC);
  if ( currentState && !lastBtn_inc &&
       millis() - ul_Debounce_incMS >= SELANG_WAKTU_DEBOUNCE ) {
    ul_Debounce_incMS = millis();
    f_SetPoint_C += RESOLUSI_SETPOINT_C;
    Serial.println("SP Increased!");
  }
  lastBtn_inc = currentState;
  currentState = digitalRead(BUTTON_DEC);
  if ( currentState && !lastBtn_dec &&
       millis() - ul_Debounce_decMS >= SELANG_WAKTU_DEBOUNCE ) {
    ul_Debounce_decMS = millis();
    f_SetPoint_C -= RESOLUSI_SETPOINT_C;
    Serial.println("SP Decreased!");
  }
  lastBtn_dec = currentState;
  if (millis() - ul_LCDTimeMS >= SELANG_WAKTU_LCD) {
    ul_LCDTimeMS = millis();
    char deg = 223;

    lcd.setCursor(6, 0); lcd.print("       ");
    lcd.setCursor(6, 0); lcd.print(f_TempFilter_C, 2);
    lcd.print(deg); lcd.print("C");
    lcd.setCursor(6, 1); lcd.print("       ");
    lcd.setCursor(6, 1); lcd.print(f_SetPoint_C, 2);
    lcd.print(deg); lcd.print("C ");
    lcd.print(digitalRead(HEATER_PIN) == LOW ? "ON " : "OFF");

    Serial.print("DATA,DATE,TIME,");
    Serial.print(f_TempFilter_C); Serial.print(",");
    Serial.print(f_SetPoint_C, 2); Serial.print(",");
    Serial.print(digitalRead(HEATER_PIN) == LOW ? "ON" : "OFF");
    Serial.print(",");
    Serial.print(pid_output); Serial.print(",");
    Serial.println(millis());
  }
}
