// ============================================================
// smorphi_encoder_ros.ino
// ESP32  |  Smorphi 4-wheel Mecanum
// ============================================================
//
// SERIAL PROTOCOL (115200 baud)
// ─────────────────────────────
// TX → RPi every 20ms:
//   "c1,c2,c3,c4\n"   ← cumulative encoder ticks (signed long)
//   order: M1(FL), M2(FR), M3(RR), M4(RL)
//
// RX ← RPi:
//   "d1,d2,d3,d4,p1,p2,p3,p4\n"
//   d1..d4 = direction per wheel: 1=FORWARD, 2=BACKWARD, 0=RELEASE
//   p1..p4 = PWM per wheel: 0–255
//   This allows full independent wheel control for all mecanum motions
//   including diagonal (forward+rotate, etc.)
//
// WHEEL LAYOUT (top view):
//   M1(FL) ╲  ╱ M2(FR)
//   M4(RL) ╱  ╲ M3(RR)
//
// ENCODER: RISING edge of channel A only.
//   A==B → FORWARD (counts++)   A!=B → REVERSE (counts--)
// ============================================================

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ── Encoder pins ──────────────────────────────────────────────
const int enc1A = 4,  enc1B = 5;   // M1 FL
const int enc2A = 19, enc2B = 18;  // M2 FR
const int enc3A = 25, enc3B = 23;  // M3 RR
const int enc4A = 26, enc4B = 27;  // M4 RL

volatile long counts[4] = {0, 0, 0, 0};

// ── Motor shield ──────────────────────────────────────────────
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motors[4] = {nullptr, nullptr, nullptr, nullptr};
// motors[0]=M1(FL)  motors[1]=M2(FR)  motors[2]=M3(RR)  motors[3]=M4(RL)

// ── ISRs — RISING only ────────────────────────────────────────
void IRAM_ATTR isr1() { (digitalRead(enc1A)==digitalRead(enc1B)) ? counts[0]++ : counts[0]--; }
void IRAM_ATTR isr2() { (digitalRead(enc2A)==digitalRead(enc2B)) ? counts[1]++ : counts[1]--; }
void IRAM_ATTR isr3() { (digitalRead(enc3A)==digitalRead(enc3B)) ? counts[2]++ : counts[2]--; }
void IRAM_ATTR isr4() { (digitalRead(enc4A)==digitalRead(enc4B)) ? counts[3]++ : counts[3]--; }

// ── Stop all motors ───────────────────────────────────────────
void stopAll() {
    for (int i = 0; i < 4; i++) {
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
    }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10);
    Wire.begin(21, 22);

    if (!AFMS.begin()) {
        Serial.println("MotorShield not found!");
        while (1);
    }

    motors[0] = AFMS.getMotor(1);  // M1 FL
    motors[1] = AFMS.getMotor(2);  // M2 FR
    motors[2] = AFMS.getMotor(3);  // M3 RR
    motors[3] = AFMS.getMotor(4);  // M4 RL

    pinMode(enc1A, INPUT_PULLUP); pinMode(enc1B, INPUT_PULLUP);
    pinMode(enc2A, INPUT_PULLUP); pinMode(enc2B, INPUT_PULLUP);
    pinMode(enc3A, INPUT_PULLUP); pinMode(enc3B, INPUT_PULLUP);
    pinMode(enc4A, INPUT_PULLUP); pinMode(enc4B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(enc1A), isr1, RISING);
    attachInterrupt(digitalPinToInterrupt(enc2A), isr2, RISING);
    attachInterrupt(digitalPinToInterrupt(enc3A), isr3, RISING);
    attachInterrupt(digitalPinToInterrupt(enc4A), isr4, RISING);

    stopAll();
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {

    // 1. Parse "d1,d2,d3,d4,p1,p2,p3,p4\n" from RPi
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        // Need exactly 8 comma-separated values
        int vals[8];
        int count = 0;
        int start = 0;
        for (int i = 0; i <= line.length() && count < 8; i++) {
            if (i == line.length() || line.charAt(i) == ',') {
                vals[count++] = line.substring(start, i).toInt();
                start = i + 1;
            }
        }

        if (count == 8) {
            for (int i = 0; i < 4; i++) {
                int dir = vals[i];       // 0=RELEASE, 1=FORWARD, 2=BACKWARD
                int pwm = constrain(vals[4 + i], 0, 255);
                motors[i]->setSpeed(pwm);
                if      (dir == 1) motors[i]->run(FORWARD);
                else if (dir == 2) motors[i]->run(BACKWARD);
                else               motors[i]->run(RELEASE);
            }
        }
    }

    // 2. Publish encoder counts at 50 Hz
    static uint32_t lastPub = 0;
    if (millis() - lastPub >= 20) {
        lastPub = millis();
        noInterrupts();
        long c0=counts[0], c1=counts[1], c2=counts[2], c3=counts[3];
        interrupts();
        Serial.printf("%ld,%ld,%ld,%ld\n", c0, c1, c2, c3);
    }
}
