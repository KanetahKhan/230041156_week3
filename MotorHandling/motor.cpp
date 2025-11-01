#include "motor.h"

Motor::Motor(uint8_t pinA, uint8_t pinB)
: _a(pinA), _b(pinB), _begun(false) {}

void Motor::begin() {
  pinMode(_a, OUTPUT);
  pinMode(_b, OUTPUT);
  digitalWrite(_a, LOW);
  digitalWrite(_b, LOW);
  _begun = true;
}

void Motor::forward(uint8_t pwm) {
  if(!_begun) begin();
  analogWrite(_a, pwm);
  digitalWrite(_b, LOW);
}

void Motor::backward(uint8_t pwm) {
  if(!_begun) begin();
  digitalWrite(_a, LOW);
  analogWrite(_b, pwm);
}

void Motor::stop() {
  if(!_begun) begin();
  digitalWrite(_a, LOW);
  digitalWrite(_b, LOW);
}

void Motor::brake(bool enable) {
  if(!_begun) begin();
  if (enable) {
    digitalWrite(_a, HIGH);
    digitalWrite(_b, HIGH);
  } else {
    stop();
  }
}
