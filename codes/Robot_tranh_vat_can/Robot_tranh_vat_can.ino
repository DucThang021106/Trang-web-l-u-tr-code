#include <Arduino.h>
#include <Servo.h>
/* ===== HC-SR04 ===== */
const unsigned int TRIG_PIN = A0;
const unsigned int ECHO_PIN = A1;
const unsigned int BAUD_RATE = 9600;
/* ===== TCRT5000L 5 kênh (Dọc gầm xe: Dùng làm chống rơi) ===== */
const int S_L2 = 8;
const int S_L1 = 9;
const int S_M  = 10;
const int S_R1 = 11;
const int S_R2 = 12;

/* ===== Motor pins L298N ===== */
const int ENA = 5;    // PWM Left
const int ENB = 6;    // PWM Right
const int IN1 = 0;    // Left DIR1
const int IN2 = 1;    // Left DIR2
const int IN3 = 4;    // Right DIR1
const int IN4 = 7;    // Right DIR2

int speed_left  = 130;
int speed_right = 150;

/* ===== Servo ===== */
const int SERVO_PIN = A5;
Servo myservo;
const int CENTER_ANGLE = 90;
const int LEFT_ANGLE   = 180;
const int RIGHT_ANGLE  = 10;

/* ===== Ngưỡng vật cản ===== */
const int OBSTACLE_THRESHOLD   = 35;   // phía trước < 35 cm -> dừng và lùi
const int SCAN_THRESHOLD_CM    = 50;   // khi quét trái/phải: >= 50 cm là đường thông
const int EQUAL_TOLERANCE_CM   = 5;    // 2 bên chênh nhau <= 5 cm coi là bằng nhau
const int NO_ECHO_CM           = 400;  // không có echo = rất xa (đường thông)
/* ===== Lùi lại ===== */
const float REVERSE_CM = 7.0f;                    // quãng đường lùi
const unsigned long REVERSE_TIMEOUT_MS = 1500;    // an toàn nếu encoder không đếm

/* ===== Turning flag ===== */
bool isTurning = false;

/* ===== Encoder + hình học ===== */
const uint8_t PIN_ENC_L = 2;
const uint8_t PIN_ENC_R = 3;
const uint16_t PULSES_PER_REV = 20;
const float WHEEL_RADIUS_M = 0.0325f;
const float WHEEL_BASE_M   = 0.0950f;
const float CIRC_M = 2.0f * 3.1415926f * WHEEL_RADIUS_M;

volatile long encL_total = 0;
volatile long encR_total = 0;

/* ===== ISR Encoder ===== */
void encL_isr() {
  encL_total++;
}
void encR_isr() {
  encR_total++;
}

/* ===== Motor L298N helper ===== */
void motorWriteLR(int pwmL, int pwmR) {
  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);
  // Left motor
  if (pwmL > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwmL);
  } else if (pwmL < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(pwmL));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  // Right motor
  if (pwmR > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwmR);
  } else if (pwmR < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(pwmR));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// Khai báo trước các hàm điều hướng
void stop();
void forward();
void backward();
void turn_left();
void turn_right();
void turn_right_90_Degrees();
void turn_left_90_Degrees();
void reverseDistanceCm(float cm);
void chooseDirectionAndTurn(int leftDistance, int rightDistance);
int getDistance();
int scanDirection(int angle);

/* ===== Setup ===== */
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Cấu hình chân cảm biến chống rơi 5 mắt
  pinMode(S_L2, INPUT);
  pinMode(S_L1, INPUT);
  pinMode(S_M,  INPUT);
  pinMode(S_R1, INPUT);
  pinMode(S_R2, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stop();

  myservo.attach(SERVO_PIN);
  myservo.write(CENTER_ANGLE);
  delay(500);

  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), encR_isr, RISING);

  // Hạt giống ngẫu nhiên từ chân analog đang để trống
  randomSeed(analogRead(A3));
}

/* ===== Loop ===== */
void loop() {
  // Kiểm tra chống rơi bậc thang liên tục từ 5 mắt hồng ngoại dưới gầm
  bool l2 = digitalRead(S_L2);
  bool l1 = digitalRead(S_L1);
  bool m  = digitalRead(S_M);
  bool r1 = digitalRead(S_R1);
  bool r2 = digitalRead(S_R2);

  // Nếu cả 5 mắt đều mất phản xạ (nghĩa là hẫng chân / gặp bậc thang đi xuống)
  if (!l2 && !l1 && !m && !r1 && !r2) {
    stop();
    backward();     // Lùi lại ngay lập tức
    delay(500);
    turn_right_90_Degrees(); // Quay đầu tránh rơi
    return;
  }

  if (isTurning) {
    delay(100);
    return;
  }

  int frontDistance = getDistance();
  if (frontDistance == 0) {
    stop();
    delay(100);
    return;
  } else {
    if (frontDistance < OBSTACLE_THRESHOLD) {
      // 1) Dừng và lùi lại khoảng 7 cm
      stop();
      delay(100);
      reverseDistanceCm(REVERSE_CM);

      // 2) Quét trái / phải
      int leftDistance = scanDirection(LEFT_ANGLE);
      int rightDistance = scanDirection(RIGHT_ANGLE);
      myservo.write(CENTER_ANGLE);
      delay(500);

      // 3) Chọn hướng và quay
      chooseDirectionAndTurn(leftDistance, rightDistance);
    }
    else {
      forward();
    }
  }
  delay(200);
}

/* ===== Chọn hướng quay ===== */
void chooseDirectionAndTurn(int leftDistance, int rightDistance) {
  bool leftFree  = (leftDistance  >= SCAN_THRESHOLD_CM);
  bool rightFree = (rightDistance >= SCAN_THRESHOLD_CM);
  bool goRight;

  if (leftFree && rightFree) {
    // Hai bên đều thông -> chọn ngẫu nhiên
    goRight = random(2);
  } else if (leftFree) {
    goRight = false;
  } else if (rightFree) {
    goRight = true;
  } else {
    // Hai bên đều bị chặn -> chọn bên xa hơn, nếu bằng nhau thì ngẫu nhiên
    if (abs(leftDistance - rightDistance) <= EQUAL_TOLERANCE_CM) {
      goRight = random(2);
    } else {
      goRight = (rightDistance > leftDistance);
    }
  }

  if (goRight) turn_right_90_Degrees();
  else         turn_left_90_Degrees();
}

/* ===== Siêu âm + quét servo ===== */
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return 0;
  int distance = duration / 29 / 2;
  return distance;
}

int scanDirection(int angle) {
  myservo.write(angle);
  delay(700);
  int d = getDistance();
  // Không có echo trong tầm đo = không có vật cản gần -> coi là thông
  if (d == 0) d = NO_ECHO_CM;
  return d;
}

/* ===== Motor functions dùng L298N ===== */
void stop() {
  motorWriteLR(0, 0);
}
void forward() {
  motorWriteLR(speed_left, speed_right);
}
void backward() {
  motorWriteLR(-speed_left, -speed_right);
}
void turn_left() {
  motorWriteLR(-speed_left, speed_right);
}
void turn_right() {
  motorWriteLR(speed_left, -speed_right);
}

/* ===== Lùi theo quãng đường bằng encoder ===== */
long pulsesForDistanceCm(float cm) {
  float revs = (cm / 100.0f) / CIRC_M;
  return (long)(revs * PULSES_PER_REV + 0.5f);
}

void reverseDistanceCm(float cm) {
  long target = pulsesForDistanceCm(cm);
  long L0, R0;
  noInterrupts();
  L0 = encL_total;
  R0 = encR_total;
  interrupts();

  unsigned long t0 = millis();
  backward();
  while (millis() - t0 < REVERSE_TIMEOUT_MS) {
    long Ld, Rd;
    noInterrupts();
    Ld = encL_total - L0;
    Rd = encR_total - R0;
    interrupts();
    if ((Ld + Rd) / 2 >= target) break;   // trung bình 2 bánh đủ quãng đường
    delay(2);
  }
  stop();
  delay(150);
}

/* ===== Quay 90° bằng encoder ===== */
long pulsesForSpinDeg(float deg) {
  float theta = fabs(deg) * 3.14159265f / 180.0f;
  float s = theta * (WHEEL_BASE_M / 2.0f);
  float revs = s / CIRC_M;
  return (long)(revs * PULSES_PER_REV + 0.5f);
}

void turn_right_90_Degrees() {
  isTurning = true;
  long target = 7.8f * pulsesForSpinDeg(90.0f);
  long L0, R0;
  noInterrupts();
  L0 = encL_total;
  R0 = encR_total;
  interrupts();

  while (true) {
    turn_right();
    long Ld, Rd;
    noInterrupts();
    Ld = abs(encL_total - L0);
    Rd = abs(encR_total - R0);
    interrupts();
    if (Ld >= target && Rd >= target) break;
    delay(5);
  }
  stop();
  delay(200);
  isTurning = false;
}

void turn_left_90_Degrees() {
  isTurning = true;
  long target = 7.8f * pulsesForSpinDeg(90.0f);
  long L0, R0;
  noInterrupts();
  L0 = encL_total;
  R0 = encR_total;
  interrupts();

  while (true) {
    turn_left();
    long Ld, Rd;
    noInterrupts();
    Ld = abs(encL_total - L0);
    Rd = abs(encR_total - R0);
    interrupts();
    if (Ld >= target && Rd >= target) break;
    delay(5);
  }
  stop();
  delay(200);
  isTurning = false;
}