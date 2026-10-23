# Automated Mini Conveyor Belt Speed Regulator

**Author:** Sandeep Dhangar | ESP32 + L298N + DC motor + optical sensor | Closed-loop PID

## 1. Objective
Mini conveyor belt jiski speed load change hone par bhi automatically target value par rehti hai. Optical sensor se actual speed measure hoti hai, PID controller L298N ko PWM deta hai.

## 2. Block diagram
```
 Pot (target RPM) --> [ESP32 PID, every 100 ms] --PWM(ENA)+IN1/IN2--> [L298N] --> DC motor --> belt roller
                              ^                                                       |
                              |                                           slotted disc on shaft
                              +----- pulse count (ISR) <---- Optical sensor <---------+
 OLED : set RPM, actual RPM, belt cm/s, PWM, load
```

## 3. Components and wiring (real hardware, SIM_MODE 0)
| Part | Connection |
|---|---|
| L298N ENA | ESP32 GPIO14 (PWM, 5 kHz). ENA jumper hata do |
| L298N IN1 / IN2 | GPIO26 / GPIO25 |
| L298N OUT1/OUT2 | DC motor |
| L298N 12V in | Motor supply (6-12 V) |
| L298N GND | **ESP32 GND se common** (zaroori) |
| Optical sensor (slotted/IR) OUT | GPIO34 (3.3 V logic) |
| Pot | 3V3 / GPIO32 / GND, target speed |
| Button | GPIO27 to GND |
| OLED SSD1306 | SDA 21, SCL 22 |

Encoder disc: motor shaft par **40 slots** wali disc, optical sensor slot se pulse deta hai. Roller diameter 2 cm (`ROLLER_D_CM` change karo apne hisaab se).

## 4. Control logic
- **Measure:** ISR rising edges count karta hai. Speed = pulses in last 0.5 s (5 x 100 ms sliding window) / 40 slots x 60. Resolution about 3 RPM.
- **Control:** PID (Kp 0.35, Ki 1.2, Kd 0), 100 ms loop, output 0-255 PWM.
- **Anti-windup:** output saturate ho to integral ruk jata hai.
- **Dead zone compensation:** L298N + motor kam duty par nahi chalti, isliye minimum start duty 55.
- **Belt velocity** = RPM / 60 x pi x roller diameter. (150 RPM = 15.7 cm/s, 240 RPM = 25.1 cm/s)

## 5. How to run in Wokwi (SIM_MODE 1, default)
Wokwi me L298N aur DC motor ka reliable part nahi hai, isliye firmware me **software motor model** hai jo optical pulses generate karta hai. Control loop, PID, PWM aur L298N ke IN1/IN2/ENA signals real hain (LEDs se dikhte hain).
1. Naya ESP32 project, **file ka naam `sketch.ino` rakho** (`main.c` nahi).
2. `sketch.ino`, `diagram.json`, `libraries.txt` paste karo.
3. Start. Pot se target speed badlo, button se LOAD on/off karo.
4. Serial monitor CSV deta hai: `t_ms,setpoint_rpm,measured_rpm,pwm,load`.

Real hardware ke liye `#define SIM_MODE 0` karo.

## 6. Test results
> Ye numbers `docs/sim_model.py` (same constants ka Python model) se hain. Wokwi ya hardware run ke baad apne screenshot/serial log se in numbers ko replace/confirm karo. Plot: `docs/response.png`.

| # | Condition | Result |
|---|---|---|
| T1 | Step 0 -> 150 RPM | Rise time (90%) 0.9 s, overshoot 0.6%, settle (2%) 1.1 s, steady error 0.01 RPM |
| T2 | Setpoint 150 -> 240 RPM | Overshoot 1.6%, settle 0.6 s, steady error 0.02 RPM |
| T3 | Load ON at 240 RPM | Speed dip 31 RPM, recovery (2%) 1.5 s, steady error 0.1 RPM |
| T4 | Load OFF at 150 RPM | Overshoot 24 RPM, back to setpoint, steady error 0.2 RPM |
| T5 | Pot = 0 | Motor stop, integrator reset |

**Open loop vs closed loop (240 RPM, fixed PWM 196):** unloaded 239 RPM, load aane par **192 RPM** (20% drop). Closed loop me load ke baad bhi 240 RPM par wapas aa jata hai. Yehi closed-loop ka faayda hai.

## 7. Files
```
sketch.ino            firmware (ESP32, core 3.x)
diagram.json          Wokwi circuit
libraries.txt         Wokwi libraries
docs/sim_model.py     PID tuning + test model
docs/response.png     response plot
README.md             this report
```

## 8. Limitations
- Wokwi run me motor aur sensor software model hain, real mechanical test nahi.
- PID gains model ke liye tune hain. Real motor par Kp/Ki thode retune karne pad sakte hain.
