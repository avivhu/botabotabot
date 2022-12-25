// From: https://github.com/kurimawxx00/arduino-self-balancing-robot

#include "MotorController.hpp"
#include "Arduino.h"
#include <Streaming.h>

OneInputMotorController::OneInputMotorController(bool invert) : _invert(invert)
{
}

void OneInputMotorController::init(int enablePin, int directionPin, int pwmChannel)
{
    _enablePin = enablePin;
    _directionPin = directionPin;
    _pwmChannel = pwmChannel;

    pinMode(_enablePin, OUTPUT);
    pinMode(_directionPin, OUTPUT);

    // Set up PWM
    const int freq = 20000;
    const int resolution = 8;
    ledcSetup(_pwmChannel, freq, resolution);
    ledcAttachPin(_enablePin, _pwmChannel);

    move(0);
}

void OneInputMotorController::spin(int pwm)
{
    move(pwm);
}

void OneInputMotorController::move(int speed)
{
    const auto directionVal = _invert ?
        (speed >= 0 ? HIGH : LOW) :
        (speed >= 0 ? LOW : HIGH);
    digitalWrite(_directionPin, directionVal);
    _analogWrite(abs(speed));
    _curPwm = speed;
}

void OneInputMotorController::_analogWrite(int dutyCycle)
{
    assert(dutyCycle >= 0);
    ledcWrite(_pwmChannel, dutyCycle);
}
