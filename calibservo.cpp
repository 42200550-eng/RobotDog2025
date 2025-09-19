#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Forward declaration
void moveToDeg(int deg);

// Tham chiếu constraint trong hardware.ino
int d_constraint_min[3] = {-50, 30, 60}; 
int d_constraint_max[3] = {80, 150, 140};

// Channel servo của Front Left
int s_output[3] = {0, 2, 8}; // hip, shoulder, knee

int currentJoint = 0;

// Servo PWM frequency (typical hobby servo: 50–60 Hz)
static const float PWM_FREQ = 50.0f;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQ);

  Serial.println("=== Calib theo góc IK ===");
  Serial.println("Commands:");
  Serial.println("  l - move to minDeg");
  Serial.println("  h - move to maxDeg");
  Serial.println("  m - move to midDeg");
  Serial.println("  n - next joint");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'l') {
      moveToDeg(d_constraint_min[currentJoint]);
    } else if (c == 'h') {
      moveToDeg(d_constraint_max[currentJoint]);
    } else if (c == 'm') {
      int mid = (d_constraint_min[currentJoint] + d_constraint_max[currentJoint]) / 2;
      moveToDeg(mid);
    } else if (c == 'n') {
      currentJoint++;
      if (currentJoint >= 3) currentJoint = 0;
      Serial.print("Now calibrating joint: ");
      Serial.println(currentJoint);
    }
  }
}

static inline uint16_t us_to_tick(uint16_t us) {
  // tick = us * 4096 * Fpwm / 1e6
  float t = (float)us * 4096.0f * PWM_FREQ / 1000000.0f;
  if (t < 0) t = 0; if (t > 4095) t = 4095;
  return (uint16_t)(t + 0.5f);
}

void moveToDeg(int deg) {
  // Map theo biên constraint của khớp đang chọn → dải µs an toàn
  int minD = d_constraint_min[currentJoint];
  int maxD = d_constraint_max[currentJoint];
  if (minD > maxD) { int t = minD; minD = maxD; maxD = t; }

  // Lưu ý: đây chỉ là tạm thời để dò, không phải mapping cuối cùng
  int us = map(constrain(deg, minD, maxD), minD, maxD, 1000, 2000);
  us = constrain(us, 800, 2200);

  uint16_t tick = us_to_tick((uint16_t)us);
  pwm.setPWM(s_output[currentJoint], 0, tick);

  Serial.print("Joint "); Serial.print(currentJoint);
  Serial.print(" -> "); Serial.print(deg); Serial.print(" deg, ");
  Serial.print(us); Serial.print(" us, tick="); Serial.println(tick);
}