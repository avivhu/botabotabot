// From: https://github.com/kurimawxx00/arduino-self-balancing-robot

#include "MotorController.hpp"
#include "Arduino.h"
#include <Streaming.h>

MotorController::MotorController(bool invert) : _invert(invert)
{

}

void MotorController::init(int in1, int in2, int pwmChannel)
{
    // Set PWM properties
    const int freq = 20000;
    const int resolution = 8;
    _pwmChannel = pwmChannel;
    // Configure LED PWM functionalitites
    ledcSetup(_pwmChannel, freq, resolution);

    _in1 = in1;
    _in2 = in2;
    _prevSpeed = 0;

    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
}

void MotorController::spin(int pwm)
{
    if (_invert)
    {
        pwm *= -1;
    }

    move(pwm);

    // if (pwm > 0)
    //     forward(pwm);
    // else if (pwm < 0)
    //     reverse(pwm);
    // else
    //     brake();
}

void MotorController::forward(int pwm)
{
    assert(pwm > 0);
    move(pwm);
}

void MotorController::reverse(int pwm)
{
    assert(pwm < 0);
    move(pwm);
}

void MotorController::brake()
{
    move(0);
}


void MotorController::move(int speed)
{
    if (speed > 0)
    {
        if (_prevSpeed <= 0)
        {
            ledcDetachPin(_in2);
            digitalWrite(_in2, LOW);
            ledcAttachPin(_in1, _pwmChannel);
        }
        _analogWrite(abs(speed));
    }
    else if (speed < 0)
    {
        if (_prevSpeed >= 0)
        {
            ledcDetachPin(_in1);
            digitalWrite(_in1,  LOW);
            ledcAttachPin(_in2, _pwmChannel);
        }
        _analogWrite(abs(speed));
    }
    else
    {
        // If speed == 0, set both pins to LOW
        if (_prevSpeed != 0)
        {
            ledcDetachPin(_in1);
            ledcDetachPin(_in2);
            digitalWrite(_in1, LOW);
            digitalWrite(_in2, LOW);
        }
    }
    _prevSpeed = speed;
}

void MotorController::_analogWrite(int dutyCycle)
{
    assert(dutyCycle >= 0);
    ledcWrite(_pwmChannel, dutyCycle);
}
