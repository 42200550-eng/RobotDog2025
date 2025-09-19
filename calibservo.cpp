#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo config
static const uint8_t NUM_SERVO = 3;
static const uint8_t SERVO_CH[NUM_SERVO] = {0, 2, 4};       // kênh PCA9685 theo servo index
static const bool     INVERT[NUM_SERVO]   = {false, false, false}; // đảo chiều theo từng servo

// Biên xung (µs) – calib theo servo thực tế
static const int MIN_US[NUM_SERVO] = {450, 400, 400};
static const int MAX_US[NUM_SERVO] = {2550, 2600, 2700};

// Tần số PWM (Hz) — an toàn hơn cho digital servo
static const float PWM_FREQ = 300.0f;

static inline int clampi(int v, int a, int b) {
  return v < a ? a : (v > b ? b : v);
}

// Map góc → µs theo từng servo (tâm 90°), có invert và clamp an toàn
static int angleToUs(uint8_t idx, float deg) {
  if (idx >= NUM_SERVO) return 1500;
  int usMin = clampi(MIN_US[idx], 500, 2500);
  int usMax = clampi(MAX_US[idx], 500, 2500);
  if (usMin > usMax) { int t = usMin; usMin = usMax; usMax = t; }

  const float us_per_deg = (usMax - usMin) / 180.0f;
  const float mid_us = (usMin + usMax) * 0.5f;

  float us = mid_us + (deg - 90.0f) * us_per_deg;

  if (INVERT[idx]) {
    // đối xứng quanh midpoint
    us = usMin + usMax - us;
  }

  return clampi((int)lroundf(us), 500, 2500);
}

static void setServoAngle(uint8_t idx, float deg) {
  if (idx >= NUM_SERVO) return;
  const uint8_t ch = SERVO_CH[idx];
  const int us = angleToUs(idx, deg);
  pwm.writeMicroseconds(ch, us);
  Serial.print("Servo#"); Serial.print(idx);
  Serial.print(" (ch "); Serial.print(ch); Serial.print(") -> ");
  Serial.print(deg); Serial.print(" deg -> ");
  Serial.print(us); Serial.println(" us");
}

static void sweepServo(uint8_t idx) {
  if (idx >= NUM_SERVO) return;
  const uint8_t ch = SERVO_CH[idx];
  int usMin = clampi(MIN_US[idx], 500, 2500);
  int usMax = clampi(MAX_US[idx], 500, 2500);
  if (usMin > usMax) { int t = usMin; usMin = usMax; usMax = t; }

  // Quét tới
  for (int us = usMin; us <= usMax; us += 10) {
    pwm.writeMicroseconds(ch, us);
    delay(15);
    Serial.println(us);
  }
  Serial.println("Reached max position");

  // Quét ngược lại
  for (int us = usMax; us >= usMin; us -= 10) {
    pwm.writeMicroseconds(ch, us);
    delay(15);
    Serial.println(us);
  }
  Serial.println("Reached min position");

}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // ESP32: SDA=21, SCL=22
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQ);
  delay(50);

  sweepServo(2);

}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.length()) {
        line.replace(',', ' ');
        int sep = line.indexOf(' ');
        if (sep > 0) {
          int idx = line.substring(0, sep).toInt();
          float deg = line.substring(sep + 1).toFloat();
          if (idx >= 0 && idx < NUM_SERVO) {
            setServoAngle((uint8_t)idx, deg);
          } else {
            Serial.println("ERR: servo index out of range. Valid: 0..1");
          }
        } else {
          Serial.println("Usage: <servoIndex> <angleDeg>  (e.g. 0 45)");
        }
      }
      line = "";
    } else {
      line += c;
    }
  }
}
