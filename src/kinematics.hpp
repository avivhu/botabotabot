
// Based on code by Juan Miguel Jimeno. This is his license:
//         Copyright (c) 2021 Juan Miguel Jimeno
//
//         Licensed under the Apache License, Version 2.0 (the "License");
//         you may not use this file except in compliance with the License.
//         You may obtain a copy of the License at
//
//             http://www.apache.org/licenses/LICENSE-2.0
//
//         Unless required by applicable law or agreed to in writing, software
//         distributed under the License is distributed on an "AS IS" BASIS,
//         WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//         See the License for the specific language governing permissions and
//         limitations under the License.

#include <cmath>

class Kinematics
{
public:
    Kinematics(float bodyWheelOffsetMeters, float wheelRadiusMeters) : 
        _bodyWheelOffsetMeters(bodyWheelOffsetMeters),
        _wheelRadiusMeters(wheelRadiusMeters)
    {
    }

    struct rpm
    {
        float motorRpm[3];
    };

    struct velocities
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };

    struct pwm
    {
        int motorPwm[3];
    };

    rpm getRPM(float linear_x, float linear_y, float angular_z);

    float _bodyWheelOffsetMeters;
    float _wheelRadiusMeters;
};

// Receive platform velocity and compute the required RPM for each motor to achieve this velocity
// linear_x, linear_y - In meters/sec
// angular_z - In degrees/sec
Kinematics::rpm Kinematics::getRPM(float linearX, float linearY, float angularZ)
{
    const auto sin_60 = 0.8660254037844386f;

    const auto angularZRadians = float(M_PI) * angularZ / 180;

    // Compute angular speed of each wheel in radians / sec
    auto u1 = (-_bodyWheelOffsetMeters * angularZRadians + 1.0f * linearX) / _wheelRadiusMeters;
    auto u2 = (-_bodyWheelOffsetMeters * angularZRadians - 0.5f * linearX + -sin_60 * linearY) / _wheelRadiusMeters;
    auto u3 = (-_bodyWheelOffsetMeters * angularZRadians - 0.5f * linearX + +sin_60 * linearY) / _wheelRadiusMeters;

    // Convert to rad/sec to RPM (revs per minute)
    u1 *= 60.0 / (2 * M_PI);
    u2 *= 60.0 / (2 * M_PI);
    u3 *= 60.0 / (2 * M_PI);

    return {{u1, u2, u3}};
};