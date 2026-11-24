#include <Arduino.h>

// ================= Chân =================
const int S_PIN[5] = {8, 9, 10, 11, 12};   // L2, L1, M, R1, R2
const int LINE_LEVEL = LOW;                // mức tín hiệu khi mắt THẤY line (giữ như code cũ)
 
const int LED_DBG = 13;   // tắt = bám line | sáng = đang tìm lại | nháy nhanh = FAIL
 
const int ENA = 5;   // PWM trái
const int ENB = 6;   // PWM phải
const int IN1 = 0;   // trái  DIR1 (chân 0/1 dùng chung USB: rút dây khi nạp code)
const int IN2 = 1;   // trái  DIR2
const int IN3 = 4;   // phải  DIR1
const int IN4 = 7;   // phải  DIR2

const int ENC_L = 2;
const int ENC_R = 3;

// ================= Kích thước cơ khí (cm) =================
const float SENSOR_TO_AXLE_CM = 6.0f;   // !!! PHẢI ĐO: từ hàng mắt dò tới trục hai bánh
const float LINE_W_CM  = 1.9f;          // bề rộng line (1.7 ~ 2.0)
const float TRACK_CM   = 9.5f;          // khoảng cách 2 bánh
const float WHEEL_R_CM = 3.25f;
const float PPR        = 20.0f;         // xung / vòng
const float CIRC_CM    = 2.0f * 3.1415926f * WHEEL_R_CM;

// Hệ số hiệu chỉnh thực nghiệm (giữ từ code cũ của bạn, nên đo lại)
const float DIST_CAL = 1.5f;   // nhân số xung khi đi thẳng
const float SPIN_CAL = 3.0f;   // nhân số xung khi xoay tại chỗ

// Hai bánh lệch nhau thì chỉnh ở đây
const float PWM_SCALE_L = 1.0f;
const float PWM_SCALE_R = 1.0f;

// ================= Tham số điều khiển (PWM trước khi nhân SCALE) =================
// --- Bám line ---
const int   BASE_PWM       = 110;
const int   MIN_FOLLOW_PWM = 70;
const float SLOW_GAIN      = 18.0f;  // giảm tốc theo |pos|
const float KP             = 30.0f;
const float KD             = 12.0f;
const unsigned long PD_PERIOD_MS = 8;

// --- Góc / mất line ---
const int CREEP_PWM   = 80;
const int REVERSE_PWM = 75;
const int ADVANCE_PWM = 85;
const int SWEEP_PWM   = 95;

const int   ALIGN_PWM_MIN  = 75;
const float ALIGN_PWM_GAIN = 12.0f;
const float ALIGN_TOL      = 0.5f;   // |pos| <= 0.5 coi như đã trùng line
const int   ALIGN_OK_TICKS = 15;     // phải đứng yên trùng line ~15 ms

const int   LOST_CONFIRM_TICKS = 3;  // số mẫu liên tiếp = 0 mới coi là mất line

const float CREEP_MAX_CM   = 12.0f;  // creep quá đoạn này mà vẫn còn line -> báo động giả
const float EV_KEEP_CM     = 10.0f;  // nhớ "dấu hiệu góc" trong quãng này
const float EV_BLOCK_CM    = 15.0f;  // sau báo động giả, bỏ qua dấu hiệu góc quãng này
const float REVERSE_MAX_CM = 8.0f;
const float SWEEP_MAX_DEG  = 165.0f; // < 180 để không bắt nhầm đường vừa đi tới
const float PROBATION_CM   = 6.0f;   // tìm lại xong phải bám đủ quãng này mới coi là thành công
const byte  MAX_FAIL       = 3;
const int   DEFAULT_SWEEP_DIR = +1;  // không có manh mối: thử quay TRÁI trước

const unsigned long BRAKE_MS           = 80;
const unsigned long REVERSE_TIMEOUT_MS = 2000;
const unsigned long ADVANCE_TIMEOUT_MS = 1500;
const unsigned long SWEEP_TIMEOUT_MS   = 9000;
const unsigned long ALIGN_TIMEOUT_MS   = 2000;
const unsigned long SWEEP_PAUSE_MS     = 100;   // nghỉ giữa hai lần quét

const unsigned long TICK_US = 1000;     // chu kỳ đọc cảm biến + chạy máy trạng thái

// ================= Encoder =================
volatile long encL_total = 0;
volatile long encR_total = 0;
volatile unsigned long encL_last_us = 0;
volatile unsigned long encR_last_us = 0;
const unsigned long ENC_DEBOUNCE_US = 500;

void encL_isr() {
  unsigned long now = micros();
  if (now - encL_last_us >= ENC_DEBOUNCE_US) { encL_total++; encL_last_us = now; }
}
void encR_isr() {
  unsigned long now = micros();
  if (now - encR_last_us >= ENC_DEBOUNCE_US) { encR_total++; encR_last_us = now; }
}

// Quãng đường trung bình hai bánh (xung), tăng đơn điệu, bất kể tiến/lùi/xoay
long odoPulses() {
  noInterrupts();
  long l = encL_total, r = encR_total;
  interrupts();
  return (l + r) / 2;
}

long cmToPulses(float cm) {
  return (long)(cm / CIRC_CM * PPR * DIST_CAL + 0.5f);
}

long degToPulses(float deg) {
  float arc = (TRACK_CM * 0.5f) * (deg * 3.1415926f / 180.0f) * SPIN_CAL;
  return (long)(arc / CIRC_CM * PPR + 0.5f);
}

// ================= Động cơ L298N =================
void setMotor(int inA, int inB, int en, int v, float scale) {
  int pwm = (int)(abs(v) * scale);
  if (pwm > 255) pwm = 255;
  if (v > 0)      { digitalWrite(inA, HIGH); digitalWrite(inB, LOW); }
  else if (v < 0) { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); }
  else            { digitalWrite(inA, LOW);  digitalWrite(inB, LOW); }
  analogWrite(en, v == 0 ? 0 : pwm);
}

// l, r: -255..255, âm = lùi
void drive(int l, int r) {
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);
  setMotor(IN1, IN2, ENA, l, PWM_SCALE_L);
  setMotor(IN3, IN4, ENB, r, PWM_SCALE_R);
}

// dir=+1 quay trái tại chỗ, dir=-1 quay phải tại chỗ
void pivot(int dir, int pwm) {
  if (dir > 0) drive(-pwm, +pwm);
  else         drive(+pwm, -pwm);
}

// Phanh chủ động: IN1=IN2 và EN cao -> hai đầu động cơ nối tắt, dừng ngay
void motorsBrake() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

// Thả trôi (dùng khi FAIL/chờ)
void motorsStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ================= Cảm biến =================
uint8_t mask = 0;          // mặt nạ đã lọc (2 mẫu liên tiếp giống nhau mới nhận)
uint8_t rawPrev = 0;
int zeroCnt = 0;           // số mẫu liên tiếp mask == 0

uint8_t readRawMask() {
  uint8_t m = 0;
  for (uint8_t i = 0; i < 5; i++) {
    m <<= 1;
    if (digitalRead(S_PIN[i]) == LINE_LEVEL) m |= 1;
  }
  return m;   // L2 = bit4 ... R2 = bit0
}

void sampleSensors() {
  uint8_t raw = readRawMask();
  if (raw == rawPrev) mask = raw;     // lọc nhiễu: 2 mẫu cách nhau 1 ms phải giống nhau
  rawPrev = raw;
  zeroCnt = (mask == 0) ? (zeroCnt + 1) : 0;
}

uint8_t popcount5(uint8_t m) {
  uint8_t n = 0;
  while (m) { n += (m & 1); m >>= 1; }
  return n;
}

bool contiguous(uint8_t m) {
  if (!m) return true;
  while (!(m & 1)) m >>= 1;
  return (m & (m + 1)) == 0;
}

// Vị trí trung bình có trọng số: -2 -1 0 +1 +2
float calcPos(uint8_t m) {
  int s = 0, n = 0;
  for (int i = 0; i < 5; i++) {
    if (m & (1 << (4 - i))) { s += (i - 2); n++; }
  }
  return n ? (float)s / n : 0.0f;
}

// DẤU HIỆU GÓC: line "rộng bất thường" (>=3 mắt) hoặc có 2 cụm sáng tách rời
// (đỉnh góc nhọn: hai nhánh còn chưa chập vào nhau). Trả về bên có nhánh phụ.
bool isEvidence(uint8_t m, int8_t &dir) {
  if (popcount5(m) >= 3 || !contiguous(m)) {
    int l = ((m >> 4) & 1) + ((m >> 3) & 1);   // L2 + L1
    int r = ((m >> 1) & 1) + (m & 1);          // R1 + R2
    dir = (l > r) ? +1 : ((r > l) ? -1 : 0);
    return true;
  }
  dir = 0;
  return false;
}

// ================= Trạng thái =================
enum State : uint8_t {
  ST_WAIT_LINE, ST_FOLLOW, ST_CREEP, ST_BRAKE, ST_REVERSE_EDGE,
  ST_ADVANCE, ST_SWEEP, ST_ALIGN, ST_FAIL
};
enum LostKind : uint8_t { KIND_CORNER, KIND_SIDE };

State state = ST_WAIT_LINE;
State brakeNext = ST_FOLLOW;
unsigned long brakeMs = 0;
unsigned long stateT0 = 0;
long odo0 = 0;

float lastPos = 0.0f;      // vị trí hợp lệ gần nhất (không tính lúc có dấu hiệu góc)
float prevPos = 0.0f;
unsigned long lastPdMs = 0;

// Bộ nhớ dấu hiệu góc (tính theo QUÃNG ĐƯỜNG, không theo thời gian)
bool  evSeen = false;
int8_t evDir = 0;
long  evOdo = 0;
long  evBlockUntil = 0;

// Tìm lại line
LostKind lostKind = KIND_CORNER;
int   sweepDir = DEFAULT_SWEEP_DIR;
int   lastSweepDir = DEFAULT_SWEEP_DIR;
float pivotPos = 0.0f;     // vị trí xoay (xung), dương = cùng chiều sweepDir
uint8_t sweepStage = 0;
unsigned long sweepPauseUntil = 0;
long lastEL = 0, lastER = 0;
float alignPos = 0.0f;
int   alignOk = 0;

// Chống lặp vô hạn
bool probation = false;
long probOdo0 = 0;
byte failStreak = 0;
int  failOkTicks = 0;

// ---- Khai báo trước ----
void enter(State s);
void enterBrake(State next, unsigned long ms);
void startLost();
void planRecovery();
void retryOrFail();

void snapEnc() {
  noInterrupts();
  lastEL = encL_total;
  lastER = encR_total;
  interrupts();
}

void enter(State s) {
  state = s;
  stateT0 = millis();
  odo0 = odoPulses();
  if (s == ST_SWEEP) {
    pivotPos = 0.0f;
    sweepStage = 0;
    sweepPauseUntil = 0;
    snapEnc();
  } else if (s == ST_ALIGN) {
    alignOk = 0;
  } else if (s == ST_FAIL) {
    motorsStop();
    failOkTicks = 0;
  }
}

void enterBrake(State next, unsigned long ms) {
  motorsBrake();
  brakeNext = next;
  brakeMs = ms;
  enter(ST_BRAKE);
}

void markEvidence(int8_t dir, long o) {
  if (!evSeen) evDir = 0;
  evSeen = true;
  evOdo = o;
  if (dir != 0) evDir = dir;
}

// Quyết định kiểu mất line và hướng tìm lại
void planRecovery() {
  bool evValid = evSeen && (odoPulses() - evOdo) <= cmToPulses(EV_KEEP_CM);

  if (evValid) {
    // Đã thấy dấu hiệu góc trước khi mất -> góc thật, quay về bên có nhánh phụ
    lostKind = KIND_CORNER;
    sweepDir = evDir;
    if (sweepDir == 0)
      sweepDir = (lastPos < -0.4f) ? +1 : ((lastPos > 0.4f) ? -1 : DEFAULT_SWEEP_DIR);
  } else if (fabs(lastPos) >= 1.5f) {
    // Mất line khi đang lệch hẳn ra mép -> văng khỏi line ở đoạn cong: lùi lại rồi xoay vào
    lostKind = KIND_SIDE;
    alignPos = lastPos;
    sweepDir = (lastPos < 0) ? +1 : -1;
  } else {
    // Mất line từ giữa mà không có manh mối: thử DEFAULT, nếu đã fail thì đổi phía
    lostKind = KIND_CORNER;
    sweepDir = (failStreak > 0) ? -lastSweepDir : DEFAULT_SWEEP_DIR;
  }
  if (lostKind == KIND_CORNER) lastSweepDir = sweepDir;

  enterBrake(ST_REVERSE_EDGE, BRAKE_MS);
}

void startLost() {
  motorsBrake();
  if (probation) {                 // vừa tìm lại xong mà lại mất ngay -> tính là 1 lần thất bại
    probation = false;
    failStreak++;
    if (failStreak >= MAX_FAIL) { enter(ST_FAIL); return; }
  }
  planRecovery();
}

void retryOrFail() {
  failStreak++;
  if (failStreak >= MAX_FAIL) { enter(ST_FAIL); return; }
  planRecovery();
}

// ================= Các bước =================
void stepFollow() {
  if (mask == 0) {
    if (zeroCnt >= LOST_CONFIRM_TICKS) startLost();
    return;                         // chưa chắc là mất line: giữ lệnh motor cũ
  }

  long o = odoPulses();
  int8_t dir;
  if (isEvidence(mask, dir)) {
    markEvidence(dir, o);
    if (o >= evBlockUntil) { enter(ST_CREEP); return; }
  } else {
    lastPos = calcPos(mask);
  }

  if (probation && (o - probOdo0) >= cmToPulses(PROBATION_CM)) {
    probation = false;
    failStreak = 0;
    evSeen = false;
  }

  unsigned long now = millis();
  if (now - lastPdMs < PD_PERIOD_MS) return;
  lastPdMs = now;

  float pos = calcPos(mask);
  float d = pos - prevPos;
  prevPos = pos;

  float steer = KP * pos + KD * d;
  float base = BASE_PWM - SLOW_GAIN * fabs(pos);
  if (base < MIN_FOLLOW_PWM) base = MIN_FOLLOW_PWM;

  drive((int)(base + steer), (int)(base - steer));
}

void stepCreep() {
  drive(CREEP_PWM, CREEP_PWM);       // chạy thẳng, bỏ qua nhánh phụ

  if (mask == 0) {
    if (zeroCnt >= LOST_CONFIRM_TICKS) startLost();
    return;
  }
  int8_t dir;
  long o = odoPulses();
  if (isEvidence(mask, dir)) markEvidence(dir, o);

  if (o - odo0 > cmToPulses(CREEP_MAX_CM)) {   // đi hết đoạn mà vẫn còn line: báo động giả
    evSeen = false;
    evBlockUntil = o + cmToPulses(EV_BLOCK_CM);
    prevPos = calcPos(mask);
    enter(ST_FOLLOW);
  }
}

void stepReverse() {
  drive(-REVERSE_PWM, -REVERSE_PWM);
  if (mask != 0) {                              // chạm lại mép line
    alignPos = calcPos(mask);
    enterBrake(lostKind == KIND_CORNER ? ST_ADVANCE : ST_ALIGN, 50);
    return;
  }
  if (odoPulses() - odo0 > cmToPulses(REVERSE_MAX_CM) ||
      millis() - stateT0 > REVERSE_TIMEOUT_MS) {
    retryOrFail();
  }
}

void stepAdvance() {
  drive(ADVANCE_PWM, ADVANCE_PWM);
  long need = cmToPulses(SENSOR_TO_AXLE_CM - LINE_W_CM * 0.5f);
  if (odoPulses() - odo0 >= need || millis() - stateT0 > ADVANCE_TIMEOUT_MS) {
    enterBrake(ST_SWEEP, 60);
  }
}

void stepSweep() {
  unsigned long now = millis();

  if (now < sweepPauseUntil) {                  // nghỉ giữa hai lần quét
    motorsBrake();
    snapEnc();
    return;
  }

  // Cập nhật góc đã xoay (xung), dương theo chiều sweepDir ban đầu
  long el, er;
  noInterrupts();
  el = encL_total;
  er = encR_total;
  interrupts();
  float dp = ((el - lastEL) + (er - lastER)) * 0.5f;
  lastEL = el;
  lastER = er;
  pivotPos += (sweepStage == 0) ? dp : -dp;

  if (mask != 0) {                              // thấy line -> dừng ngay, căn chỉnh
    alignPos = calcPos(mask);
    enterBrake(ST_ALIGN, 30);
    return;
  }

  long limit = degToPulses(SWEEP_MAX_DEG);
  if (sweepStage == 0) {
    pivot(sweepDir, SWEEP_PWM);
    if (pivotPos >= limit) {                    // hết giới hạn phía 1 -> quay qua phía 2 (xuyên qua hướng ban đầu)
      sweepStage = 1;
      sweepPauseUntil = now + SWEEP_PAUSE_MS;
    }
  } else {
    pivot(-sweepDir, SWEEP_PWM);
    if (pivotPos <= -limit) {                   // hết cả hai phía -> thất bại
      retryOrFail();
      return;
    }
  }

  if (now - stateT0 > SWEEP_TIMEOUT_MS) retryOrFail();
}

void stepAlign() {
  if (mask != 0) alignPos = calcPos(mask);

  bool ok = (mask != 0) && fabs(alignPos) <= ALIGN_TOL &&
            popcount5(mask) <= 2 && contiguous(mask);

  if (ok) {
    motorsBrake();
    if (++alignOk >= ALIGN_OK_TICKS) {
      evSeen = false;
      probation = true;
      probOdo0 = odoPulses();
      prevPos = alignPos;
      lastPos = alignPos;
      enter(ST_FOLLOW);
      return;
    }
  } else {
    alignOk = 0;
    int dir = (alignPos < 0) ? +1 : ((alignPos > 0) ? -1 : sweepDir);
    int pwm = ALIGN_PWM_MIN + (int)(ALIGN_PWM_GAIN * fabs(alignPos));
    pivot(dir, pwm);
  }

  if (millis() - stateT0 > ALIGN_TIMEOUT_MS) retryOrFail();
}

void stepFail() {
  motorsStop();
  // Đặt xe lại lên line, M thấy line ổn định ~0.3 s thì tự chạy lại
  if (mask == 0b00100) {
    if (++failOkTicks >= 300) {
      failStreak = 0;
      probation = false;
      evSeen = false;
      prevPos = 0.0f;
      lastPos = 0.0f;
      enter(ST_FOLLOW);
    }
  } else {
    failOkTicks = 0;
  }
}

void runState() {
  switch (state) {
    case ST_WAIT_LINE:
      motorsStop();
      if (mask != 0) { prevPos = calcPos(mask); enter(ST_FOLLOW); }
      break;
    case ST_FOLLOW:       stepFollow();  break;
    case ST_CREEP:        stepCreep();   break;
    case ST_BRAKE:
      if (millis() - stateT0 >= brakeMs) enter(brakeNext);
      break;
    case ST_REVERSE_EDGE: stepReverse(); break;
    case ST_ADVANCE:      stepAdvance(); break;
    case ST_SWEEP:        stepSweep();   break;
    case ST_ALIGN:        stepAlign();   break;
    case ST_FAIL:         stepFail();    break;
  }
}

void updateLed() {
  if (state == ST_FOLLOW || state == ST_WAIT_LINE) digitalWrite(LED_DBG, LOW);
  else if (state == ST_FAIL) digitalWrite(LED_DBG, ((millis() / 100) & 1) ? HIGH : LOW);
  else digitalWrite(LED_DBG, HIGH);
}

// ================= Setup / Loop =================
void setup() {
  // Không dùng Serial vì chân 0/1 đã dành cho motor trái
  for (int i = 0; i < 5; i++) pinMode(S_PIN[i], INPUT);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LED_DBG, OUTPUT);

  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_isr, RISING);

  motorsStop();
  delay(300);
}

void loop() {
  static unsigned long lastTickUs = 0;
  unsigned long now = micros();
  if (now - lastTickUs < TICK_US) return;
  lastTickUs = now;

  sampleSensors();
  runState();
  updateLed();
}