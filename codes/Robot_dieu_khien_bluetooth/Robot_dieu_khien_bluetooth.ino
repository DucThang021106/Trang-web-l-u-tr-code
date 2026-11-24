#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>
#include <Servo.h>
#define LINE_1 10
#define LINE_2 11
#define LINE_3 12
#define LINE_4 0
#define LINE_5 1
#define TRIG_PIN A0
#define ECHO_PIN A1
#define SERVO_PIN A5
#define ENA 5
#define IN1 0
#define IN2 1
#define ENB 6
#define IN3 4
#define IN4 7
#define ENC_L 2
#define ENC_R 3 
Servo myServo;
int pwmLeft = 130;
int pwmRight = 150;  
int servoAngle = 90;
int lastWrittenAngle = -1;
unsigned long lastServoUpdate = 0;
void leftMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);}
  analogWrite(ENA, abs(pwm));}
void rightMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (pwm < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);}
  analogWrite(ENB, abs(pwm));}
void stopMotor() {
  leftMotor(0);
  rightMotor(0);}
void forward() {
  leftMotor(pwmLeft);
  rightMotor(pwmRight);
}
void backward() {
  leftMotor(-pwmLeft);
  rightMotor(-pwmRight);
}
void turnLeft() {
  leftMotor(-pwmLeft);
  rightMotor(pwmRight);
}
void turnRight() {
  leftMotor(pwmLeft);
  rightMotor(-pwmRight);
}
void writeServoIfNeeded(int angle) {
  angle = constrain(angle, 0, 180);
  if (angle != lastWrittenAngle) {
    myServo.write(angle);
    lastWrittenAngle = angle;}}
void setup() {
  pinMode(LINE_1, INPUT_PULLUP);
  pinMode(LINE_2, INPUT_PULLUP);
  pinMode(LINE_3, INPUT_PULLUP);
  pinMode(LINE_4, INPUT_PULLUP);
  pinMode(LINE_5, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  stopMotor();
  myServo.attach(SERVO_PIN);
  writeServoIfNeeded(90);
  Dabble.begin(9600, A3, A2);}
void loop() {
  Dabble.processInput();
  if (GamePad.isUpPressed()) {
    forward();
  } else if (GamePad.isDownPressed()) {
    backward();
  } else if (GamePad.isLeftPressed()) {
    turnLeft();
  } else if (GamePad.isRightPressed()) {
    turnRight();
  } else {
    stopMotor();
  }
  if (GamePad.isCrossPressed()) {
    if (millis() - lastServoUpdate > 30) {
      servoAngle = constrain(servoAngle + 2, 0, 180);
      writeServoIfNeeded(servoAngle);
      lastServoUpdate = millis();}}
  if (GamePad.isTrianglePressed()) {
    if (millis() - lastServoUpdate > 30) {
      servoAngle = constrain(servoAngle - 2, 0, 180);
      writeServoIfNeeded(servoAngle);
      lastServoUpdate = millis();}}
  if (GamePad.isSquarePressed()) {
    servoAngle = 180;
    writeServoIfNeeded(servoAngle);}
  if (GamePad.isCirclePressed()) {
    servoAngle = 10;
    writeServoIfNeeded(servoAngle);}
  if (GamePad.isStartPressed()) {
    servoAngle = 90;
    writeServoIfNeeded(servoAngle);}}