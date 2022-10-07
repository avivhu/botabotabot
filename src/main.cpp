#include <Arduino.h>
#include <Streaming.h>
#include <memory>
#include <vector>
#include <MotorController.hpp>
#include <WiFi.h>

#include "Encoder.hpp"
#include "PID.h"
#include "web.hpp"
#include "private.hpp"

using namespace std;

using MotorController = OneInputMotorController;

const int SERIAL_BAUD = 115200;

////////////// Motor pins
const int MOTOR_1_ENABLE = 25;
const int MOTOR_1_DIRECTION = 26;
const int MOTOR_1_PWM_CHANNEL = 0;

const int MOTOR_2_ENABLE = 27;
const int MOTOR_2_DIRECTION = 14;
const int MOTOR_2_PWM_CHANNEL = 1;

const int MOTOR_3_ENABLE = 12;
const int MOTOR_3_DIRECTION = 13;
const int MOTOR_3_PWM_CHANNEL = 2;


///////// Motor Encoders /////////////
// Set pins so all wheels count "up" in the ccw direction when observed from outside the robot.
const int ENCODER_1_OUT_1 = 36;
const int ENCODER_1_OUT_2 = 39;
const bool ENCODER_1_INV = true;

const int ENCODER_2_OUT_1 = 34;
const int ENCODER_2_OUT_2 = 35;
const bool ENCODER_2_INV = true;

const int ENCODER_3_OUT_1 = 32;
const int ENCODER_3_OUT_2 = 33;
const bool ENCODER_3_INV = true;


const float ENCODER_COUNTS_PER_REVOLUTION = 360 * 74.8; // Use encoder CPR (360) times gear reduction ratio (74.8)

////////////// PID ////////////////////////

const auto K_P = 3.0;
const auto K_I = 0.4;
const auto K_D = 0;
const auto PWM_MIN = -255;
const auto PWM_MAX = 255;
PID motor1Pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2Pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3Pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
float Kp, Ki, Kd;
float setpoint = 12.0;
/////////////////// Web server   /////////////////////


//////////////////////////////////////////

MotorController motor1Controller;
MotorController motor2Controller;
MotorController motor3Controller;

unique_ptr<Encoder> motor1Encoder;
unique_ptr<Encoder> motor2Encoder;
unique_ptr<Encoder> motor3Encoder;

string parseCommand(const char *data, size_t len)
{
    std::string message(data, data+len);

    if (message.compare(0, 4, "pid ") == 0)
    {
        Serial << "PID PID PID setpoint" << endl;
        Serial << "->" << Kp << ' ' << Ki << ' ' << Kd << ' ' << setpoint << endl;
        int Kp_int, Ki_int, Kd_int, setpoint_int;
        int ok = sscanf(message.c_str() + strlen("pid "), "%f %f %f %f", &Kp, &Ki, &Kd, &setpoint);
        if (ok != 4)
        {
            Serial.println("Invalid format");
            return "Invalid format";
        }
        else
        {
            Serial << "->" << Kp << ' ' << Ki << ' ' << Kd << ' ' << setpoint << endl;
            motor1Pid.updateConstants(Kp, Ki, Kd);
            motor2Pid.updateConstants(Kp, Ki, Kd);
            motor3Pid.updateConstants(Kp, Ki, Kd);
        }
        return "parseCommand OK";
    }
    else
    {
        return "parseCommand UNKNONW: " + message;
    }
}

void ConnectToWifi()
{
    // Connect to Wi-Fi
    WiFi.begin(Private::ssid(), Private::password());
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial << "Connecting to WiFi.." << endl;
    }
    // Print ESP Local IP Address
    Serial << "IP: " << WiFi.localIP() << endl;
}

char buffer[128];
int buflen = 0;

bool readLine()
{
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\r')
        {
            continue; // Ignore silly Windows \r that precedes \n
        }
        buffer[buflen++] = c;
        buffer[buflen] = 0;
        if (c == '\n')
        {
            buffer[buflen - 1] = 0;
            buflen = 0;
            return true;
        }
    }
    return false;
}

void readInput()
{
    //// Read input
    if (readLine())
    {
        auto res = parseCommand(buffer, buflen);
        Serial << res.c_str() << endl;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Booting");

    ConnectToWifi();

    StartWebServer([](uint8_t *data, size_t len)
                   { return parseCommand((const char *)data, len); });

    motor1Encoder.reset(new Encoder(ENCODER_1_OUT_1, ENCODER_1_OUT_2, ENCODER_COUNTS_PER_REVOLUTION, ENCODER_1_INV));
    motor2Encoder.reset(new Encoder(ENCODER_2_OUT_1, ENCODER_2_OUT_2, ENCODER_COUNTS_PER_REVOLUTION, ENCODER_2_INV));
    motor3Encoder.reset(new Encoder(ENCODER_3_OUT_1, ENCODER_3_OUT_2, ENCODER_COUNTS_PER_REVOLUTION, ENCODER_3_INV));

    motor1Controller.init(MOTOR_1_ENABLE, MOTOR_1_DIRECTION, MOTOR_1_PWM_CHANNEL);
    motor2Controller.init(MOTOR_2_ENABLE, MOTOR_2_DIRECTION, MOTOR_2_PWM_CHANNEL);
    motor3Controller.init(MOTOR_3_ENABLE, MOTOR_3_DIRECTION, MOTOR_3_PWM_CHANNEL);

    Serial << "Ready" << endl;
}



struct MotorStatus
{
    int64_t     count;
    float       rpm;
    float       requestedRpm;
    int         pwm;
};

MotorStatus controlSpeed(MotorController& motorController, Encoder& motorEncoder, PID& motorPid)
{
    auto count = motorEncoder.encoder().getCount();
    auto rpm = motorEncoder.getRPM();
    auto requestedRpm = setpoint;
    auto pwm = (int)motorPid.compute(requestedRpm, rpm);
    motorController.spin(pwm);

    MotorStatus status = {
        count,
        rpm,
        requestedRpm,
        pwm,
    };
    return status;
}

HardwareSerial& toStream(HardwareSerial& s, float f1, float f2, float f3)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "[%.2f %.2f %.2f]", f1, f2, f3);
    s << buf;
    return s;
}

unsigned long prevPrintTime = 0;
void loop()
{
    readInput();

    auto st1 = controlSpeed(motor1Controller, *motor1Encoder, motor1Pid);
    auto st2 = controlSpeed(motor2Controller, *motor2Encoder, motor2Pid);
    auto st3 = controlSpeed(motor3Controller, *motor3Encoder, motor3Pid);

    auto now = millis();
    bool printStatus = false;
    if (now - prevPrintTime > 1000)
    {
        prevPrintTime = now;
        printStatus = true;
    }

    if (printStatus)
    {
        Serial << " RPM =";
        toStream(Serial, st1.rpm, st2.rpm, st3.rpm);
        Serial << " PWM = ";
        toStream(Serial, st1.pwm, st2.pwm, st3.pwm);
        Serial << " Count = ";
        toStream(Serial, st1.count, st2.count, st3.count);
        Serial << " Revs = ";
        toStream(Serial, (st1.count / ENCODER_COUNTS_PER_REVOLUTION), (st2.count / ENCODER_COUNTS_PER_REVOLUTION), (st3.count / ENCODER_COUNTS_PER_REVOLUTION));
        Serial << endl;
    }


    delay(10);
}
