#pragma once

// This code was modified from a class by Juan Miguel Jimeno. His license follows.
//        Copyright (c) 2021 Juan Miguel Jimeno
//       
//        Licensed under the Apache License, Version 2.0 (the "License");
//        you may not use this file except in compliance with the License.
//        You may obtain a copy of the License at
//       
//            http://www.apache.org/licenses/LICENSE-2.0
//       
//        Unless required by applicable law or agreed to in writing, software
//        distributed under the License is distributed on an "AS IS" BASIS,
//        WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//        See the License for the specific language governing permissions and
//        limitations under the License.




// Quoting Juan Miguel Jimeno:
//   GENERIC_1_IN_MOTOR_DRIVER - Motor drivers that have EN (pwm) pin, and 1 direction pin (usual DIR pin).
//   These drivers usually have logic gates included to lessen the pins required in controlling the driver.
//   Example: Pololu MC33926 Motor Driver Shield.
// 
class OneInputMotorController
{
public:
    OneInputMotorController(bool invert = false);
    void init(int enablePin, int directionPin, int pwmChannel);
    void spin(int pwm);

private:
    void forward(int pwm);
    void reverse(int pwm);
    void move(int speed);
    void _analogWrite(int dutyCycle);

    int _enablePin, _directionPin;

    int _pwmChannel;
    int _curPwm;
    bool _invert;
};
