#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  public:
    // two direction pin is used
    Motor(uint8_t pinA, uint8_t pinB);
    void begin();
    void forward(uint8_t pwm);
    void backward(uint8_t pwm);
    void stop();
    void brake(bool enable = true);

  private:
    uint8_t _a;
    uint8_t _b;
    bool _begun;
};

#endif 
