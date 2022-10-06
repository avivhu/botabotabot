#include <Arduino.h>
#include <Streaming.h>
#include <memory>
#include <vector>
#include <MotorController.hpp>
#include <WiFi.h>

#include "Encoder.hpp"
#include "PID.h"
#include "web.hpp"

using namespace std;

const int SERIAL_BAUD = 115200;

////////////// Motor pins
const int MOTOR_1_IN_1 = 33;
const int MOTOR_1_IN_2 = 32;
const int MOTOR_1_PWM_CHANNEL = 0;

///////// Motor Encoders /////////////
// Set pins so both wheels count "up" in the forward direction
const int ENCODER_1_OUT_1 = 34;
const int ENCODER_1_OUT_2 = 35;

const float ENCODER_COUNTS_PER_REVOLUTION = 360 * 74.8; // Use encoder CPR (360) times gear reduction ratio (74.8)

////////////// PID ////////////////////////

const auto K_P = 40.0;
const auto K_I = 0.6;
const auto K_D = 0;
const auto PWM_MIN = -255;
const auto PWM_MAX = 255;
PID motor1Pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
float Kp, Ki, Kd;
float setpoint = 5.0;
/////////////////// Web server   /////////////////////

auto ssid = "....";
auto password = "....";

//////////////////////////////////////////

MotorController motor1Controller;
unique_ptr<Encoder> motor1Encoder;

volatile int64_t encoder1Count = 0;

string parseCommand(const char *data, size_t len)
{
    if (strncmp(data, "pid ", 4) == 0)
    {
        Serial << "PID PID PID setpoint" << endl;
        Serial << "->" << Kp << ' ' << Ki << ' ' << Kd << ' ' << setpoint << endl;
        int Kp_int, Ki_int, Kd_int, setpoint_int;
        int ok = sscanf(data + strlen("pid "), "%f %f %f %f", &Kp, &Ki, &Kd, &setpoint);
        if (ok != 4)
        {
            Serial.println("Invalid format");
            return "Invalid format";
        }
        else
        {
            Serial << "->" << Kp << ' ' << Ki << ' ' << Kd << ' ' << setpoint << endl;
            motor1Pid.updateConstants(Kp, Ki, Kd);
        }
        return "parseCommand OK";
    }
    else
    {
        return "parseCommand UNKNONW";
    }
}

void ConnectToWifi()
{
    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial << "Connecting to WiFi.." << endl;
    }
    // Print ESP Local IP Address
    Serial << "IP: " << WiFi.localIP() << endl;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Booting");

    ConnectToWifi();

    StartWebServer([](uint8_t *data, size_t len)
                   { return parseCommand((const char *)data, len); });

    motor1Encoder.reset(new Encoder(ENCODER_1_OUT_1, ENCODER_1_OUT_2, ENCODER_COUNTS_PER_REVOLUTION));
    motor1Controller.init(MOTOR_1_IN_1, MOTOR_1_IN_2, MOTOR_1_PWM_CHANNEL);

    Serial << "Ready" << endl;
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

unsigned long prevPrintTime = 0;
void loop()
{
    readInput();
    auto newCount1 = motor1Encoder->encoder().getCount();
    auto currentRpm1 = motor1Encoder->getRPM();

    auto req_rpm_motor1 = setpoint;
    auto computedPwm = motor1Pid.compute(req_rpm_motor1, currentRpm1);
    motor1Controller.spin(computedPwm);

    auto now = millis();
    if (now - prevPrintTime > 1000)
    {
        Serial << "Encoder = " << newCount1 << " Revs = " << (newCount1 / ENCODER_COUNTS_PER_REVOLUTION) << " RPM =" << currentRpm1 << " PWM = " << computedPwm << endl;
        prevPrintTime = now;
    }
    delay(10);
}
