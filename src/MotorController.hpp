#pragma once

// This code was modifed from a class by Juan Miguel Jimeno. His license follows.
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

class MotorController
{
public:
    MotorController(bool invert = false);
    void init(int in1, int in2, int pwmChannel);
    void spin(int pwm);

private:
    void forward(int pwm);
    void reverse(int pwm);
    void brake();
    void move(int speed);
    void _analogWrite(int dutyCycle);

    int _in1, _in2;

    int _pwmChannel;
    int _prevSpeed;
    bool _invert;
};
