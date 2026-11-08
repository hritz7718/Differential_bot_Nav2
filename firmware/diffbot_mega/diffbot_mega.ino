#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ADIY L293D header: A1 A2 EN_A VCC GND EN_B B2 B1.
constexpr uint8_t kRightPwmPin = 5;   // EN_A, right motor on MA
constexpr uint8_t kRightIn1Pin = 7;   // A1
constexpr uint8_t kRightIn2Pin = 8;   // A2
constexpr uint8_t kLeftPwmPin = 6;    // EN_B, left motor on MB
constexpr uint8_t kLeftIn1Pin = 9;    // B1
constexpr uint8_t kLeftIn2Pin = 10;   // B2

constexpr uint8_t kRightEncoderAPin = 2;
constexpr uint8_t kRightEncoderBPin = 22;
constexpr uint8_t kLeftEncoderAPin = 3;
constexpr uint8_t kLeftEncoderBPin = 23;
constexpr uint8_t kEstopPin = 18;

// Change these signs only after the raised-wheel polarity test.
constexpr int8_t kRightMotorSign = -1;
constexpr int8_t kLeftMotorSign = 1;
constexpr int8_t kRightEncoderSign = -1;
constexpr int8_t kLeftEncoderSign = 1;
constexpr bool kUseEstop = false;

constexpr long kEncoderCountsPerRevolution = 450;
constexpr float kRadiansPerCount = 2.0F * PI / kEncoderCountsPerRevolution;
constexpr float kMaximumWheelSpeed = 6.0F;
constexpr int kMaximumPwm = 90;  // Conservative initial limit for the L293D.
constexpr unsigned long kControlPeriodUs = 10000;  // 100 Hz.
constexpr unsigned long kTelemetryPeriodMs = 20;   // 50 Hz.
constexpr unsigned long kCommandTimeoutMs = 250;

constexpr float kFeedForward = 12.0F;
constexpr float kProportional = 4.0F;
constexpr float kIntegral = 8.0F;
constexpr float kIntegralLimit = 6.0F;

volatile long left_ticks = 0;
volatile long right_ticks = 0;
long previous_left_ticks = 0;
long previous_right_ticks = 0;
float left_target_rad_s = 0.0F;
float right_target_rad_s = 0.0F;
float left_measured_rad_s = 0.0F;
float right_measured_rad_s = 0.0F;
float left_integral = 0.0F;
float right_integral = 0.0F;
unsigned long last_command_ms = 0;
unsigned long last_control_us = 0;
unsigned long last_telemetry_ms = 0;
uint32_t telemetry_sequence = 0;
uint32_t crc_error_count = 0;
char receive_buffer[96];
uint8_t receive_length = 0;

uint16_t crc16Ccitt(const char *text)
{
  uint16_t crc = 0xFFFF;
  while (*text != '\0') {
    crc ^= static_cast<uint16_t>(static_cast<uint8_t>(*text++)) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void sendPayload(const char *payload)
{
  char crc_text[5];
  snprintf(crc_text, sizeof(crc_text), "%04X", crc16Ccitt(payload));
  Serial.print(payload);
  Serial.print('*');
  Serial.println(crc_text);
}

void leftEncoderInterrupt()
{
  const int8_t direction =
    digitalRead(kLeftEncoderAPin) == digitalRead(kLeftEncoderBPin) ? 1 : -1;
  left_ticks += kLeftEncoderSign * direction;
}

void rightEncoderInterrupt()
{
  const int8_t direction =
    digitalRead(kRightEncoderAPin) == digitalRead(kRightEncoderBPin) ? 1 : -1;
  right_ticks += kRightEncoderSign * direction;
}

void setMotor(
  uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin, int8_t motor_sign, int pwm)
{
  const int signed_pwm = constrain(pwm * motor_sign, -kMaximumPwm, kMaximumPwm);
  if (signed_pwm > 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (signed_pwm < 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
  }
  analogWrite(pwm_pin, abs(signed_pwm));
}

int pidOutput(float target, float measured, float &integral, float dt)
{
  if (fabs(target) < 0.02F) {
    integral = 0.0F;
    return 0;
  }
  const float error = target - measured;
  integral = constrain(integral + error * dt, -kIntegralLimit, kIntegralLimit);
  const float output = kFeedForward * target + kProportional * error + kIntegral * integral;
  return static_cast<int>(constrain(output, -static_cast<float>(kMaximumPwm),
    static_cast<float>(kMaximumPwm)));
}

void processCommand(char *line)
{
  char *separator = strrchr(line, '*');
  if (separator == nullptr || strlen(separator + 1) != 4) {
    ++crc_error_count;
    return;
  }
  const uint16_t received_crc = static_cast<uint16_t>(strtoul(separator + 1, nullptr, 16));
  *separator = '\0';
  if (crc16Ccitt(line) != received_crc) {
    ++crc_error_count;
    return;
  }

  char *save_pointer = nullptr;
  char *type = strtok_r(line, ",", &save_pointer);
  char *sequence = strtok_r(nullptr, ",", &save_pointer);
  char *left_value = strtok_r(nullptr, ",", &save_pointer);
  char *right_value = strtok_r(nullptr, ",", &save_pointer);
  if (type == nullptr || strcmp(type, "C") != 0 || sequence == nullptr ||
    left_value == nullptr || right_value == nullptr ||
    strtok_r(nullptr, ",", &save_pointer) != nullptr)
  {
    return;
  }

  (void)strtoul(sequence, nullptr, 10);
  left_target_rad_s = constrain(
    strtol(left_value, nullptr, 10) / 1000.0F, -kMaximumWheelSpeed, kMaximumWheelSpeed);
  right_target_rad_s = constrain(
    strtol(right_value, nullptr, 10) / 1000.0F, -kMaximumWheelSpeed, kMaximumWheelSpeed);
  last_command_ms = millis();
}

void readSerial()
{
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n') {
      receive_buffer[receive_length] = '\0';
      if (receive_length > 0) {
        processCommand(receive_buffer);
      }
      receive_length = 0;
    } else if (character != '\r') {
      if (receive_length < sizeof(receive_buffer) - 1) {
        receive_buffer[receive_length++] = character;
      } else {
        receive_length = 0;
      }
    }
  }
}

void updateControl(unsigned long now_us)
{
  const float dt = (now_us - last_control_us) / 1000000.0F;
  last_control_us = now_us;

  noInterrupts();
  const long current_left_ticks = left_ticks;
  const long current_right_ticks = right_ticks;
  interrupts();

  left_measured_rad_s =
    (current_left_ticks - previous_left_ticks) * kRadiansPerCount / dt;
  right_measured_rad_s =
    (current_right_ticks - previous_right_ticks) * kRadiansPerCount / dt;
  previous_left_ticks = current_left_ticks;
  previous_right_ticks = current_right_ticks;

  const bool command_stale = millis() - last_command_ms > kCommandTimeoutMs;
  const bool estop_active = kUseEstop && digitalRead(kEstopPin) == LOW;
  if (command_stale || estop_active) {
    left_target_rad_s = 0.0F;
    right_target_rad_s = 0.0F;
  }

  const int left_pwm = pidOutput(
    left_target_rad_s, left_measured_rad_s, left_integral, dt);
  const int right_pwm = pidOutput(
    right_target_rad_s, right_measured_rad_s, right_integral, dt);
  setMotor(kLeftPwmPin, kLeftIn1Pin, kLeftIn2Pin, kLeftMotorSign, left_pwm);
  setMotor(kRightPwmPin, kRightIn1Pin, kRightIn2Pin, kRightMotorSign, right_pwm);
}

void sendTelemetry()
{
  noInterrupts();
  const long current_left_ticks = left_ticks;
  const long current_right_ticks = right_ticks;
  interrupts();

  uint8_t flags = 0;
  if (millis() - last_command_ms > kCommandTimeoutMs) {
    flags |= 0x01;
  }
  if (kUseEstop && digitalRead(kEstopPin) == LOW) {
    flags |= 0x02;
  }

  char payload[96];
  snprintf(
    payload, sizeof(payload), "S,%lu,%ld,%ld,%ld,%ld,%u",
    static_cast<unsigned long>(++telemetry_sequence), current_left_ticks,
    current_right_ticks, lround(left_measured_rad_s * 1000.0F),
    lround(right_measured_rad_s * 1000.0F), flags);
  sendPayload(payload);
}

void setup()
{
  pinMode(kRightPwmPin, OUTPUT);
  pinMode(kRightIn1Pin, OUTPUT);
  pinMode(kRightIn2Pin, OUTPUT);
  pinMode(kLeftPwmPin, OUTPUT);
  pinMode(kLeftIn1Pin, OUTPUT);
  pinMode(kLeftIn2Pin, OUTPUT);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kEstopPin, INPUT_PULLUP);

  setMotor(kLeftPwmPin, kLeftIn1Pin, kLeftIn2Pin, kLeftMotorSign, 0);
  setMotor(kRightPwmPin, kRightIn1Pin, kRightIn2Pin, kRightMotorSign, 0);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderInterrupt, CHANGE);

  Serial.begin(115200);
  last_command_ms = millis();
  last_control_us = micros();
  last_telemetry_ms = millis();
}

void loop()
{
  readSerial();

  const unsigned long now_us = micros();
  if (now_us - last_control_us >= kControlPeriodUs) {
    updateControl(now_us);
  }
  const unsigned long now_ms = millis();
  if (now_ms - last_telemetry_ms >= kTelemetryPeriodMs) {
    last_telemetry_ms = now_ms;
    sendTelemetry();
  }
}
