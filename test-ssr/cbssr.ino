const int SSR = 8;
String serialInput = "";

void setup(){
  Serial.begin(19200);
  Serial.available();
  Serial.readStringUntil('\n');
  pinMode(SSR, OUTPUT);  // Jadikan SSR sebagai output
}

void loop(){
   // Kontrol SSR via SerialMonitor
     if (Serial.available() > 0) {
      serialInput = Serial.readStringUntil('\n');
      serialInput.trim(); // Menghilangkan spasi / enter

      if (serialInput.equalsIgnoreCase("ON")){
        digitalWrite(SSR, HIGH);
        Serial.println("Menghidupkan SSR");
      } else if (serialInput.equalsIgnoreCase("OFF")){
        digitalWrite(SSR, LOW);
        Serial.println("Mematikan SSR");
      } else {
        Serial.println("Input tidak dikenal, masukkan ON atau OFF!");
      }
     }
}