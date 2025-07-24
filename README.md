# PID Temperature Control System with Arduino

This project is a real-time temperature control system using a PID (Proportional–Integral–Derivative) controller. It reads temperature from an NTC thermistor, processes it with a microcontroller (Arduino), and controls a heating element to maintain a setpoint. The output is shown on an LCD and logged to Excel using PLX-DAQ.

---

## Features

- Reads temperature via NTC thermistor (via ADC + moving average)
- Controls heater output using TPO (Time Proportional Output) method
- Supports **auto-tuning** (Ziegler–Nichols) and **manual tuning** for PID
- Setpoint adjustable via buttons (increment & decrement)
- Displays real-time temperature and control status on LCD
- Logs data to Excel via PLX-DAQ (in seconds)

---

## Control Logic

1. The system calculates error = setpoint - current temperature.
2. PID algorithm computes the control output based on tuned Kp, Ki, and Kd values.
3. TPO logic turns the heater ON/OFF based on output pulse width.
4. LCD displays current temperature, setpoint, and heater status.
5. PLX-DAQ logs time, temperature, setpoint, PID output, and heater state.

---

## Tuning Method

- **Auto tuning** uses fixed values of `Ku` and `Pu` based on Ziegler–Nichols to compute optimal Kp, Ki, Kd.
- **Manual tuning** is done by observing system response and adjusting gains to minimize:
  - Rise time (< 4 min)
  - Overshoot (< 0.5°C)
  - Settling time (< 5 min)
  - Steady-state error (≈ 0)

---

## Components Used

- Arduino Uno
- NTC thermistor + Wheatstone bridge + amplifier
- LCD I2C 16x2
- Push buttons (for setpoint adjustment)
- Heater (or simulation using LED)
- PLX-DAQ for real-time logging to Excel

---

## Code Structure

- `setup()`: Initializes sensor, LCD, serial, PID tuning
- `loop()`: 
  - Samples temperature (with moving average)
  - Computes PID and controls heater (TPO logic)
  - Reads buttons for setpoint changes
  - Updates LCD
  - Sends data to Excel (PLX-DAQ)

---
Contact 
📧 Email: nadila4295@gmail.com 
🔗 LinkedIn: www.linkedin.com/in/nadila-gusriyani-8a5877242

