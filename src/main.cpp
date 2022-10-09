#include <Arduino.h>
#include <Streaming.h>
#include <memory>
#include <vector>
#include <MotorController.hpp>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "private.hpp"
#include "web.hpp"
#include "Encoder.hpp"
#include "PID.h"
#include "kinematics.hpp"

using namespace std;

using MotorController = OneInputMotorController;

const int SERIAL_BAUD = 115200;

////////////// Motor pins
const int MOTOR_1_ENABLE = 32;
const int MOTOR_1_DIRECTION = 33;
const int MOTOR_1_PWM_CHANNEL = 0;

const int MOTOR_2_ENABLE = 25;
const int MOTOR_2_DIRECTION = 26;
const int MOTOR_2_PWM_CHANNEL = 1;

const int MOTOR_3_ENABLE = 12;
const int MOTOR_3_DIRECTION = 13;
const int MOTOR_3_PWM_CHANNEL = 2;

///////// Motor Encoders /////////////
// Set pins so all wheels count "up" in the ccw direction when observed from outside the robot.
const int ENCODER_1_OUT_1 = 36; // AKA VP Pin
const int ENCODER_1_OUT_2 = 39; // AKA VN Pin
const bool ENCODER_1_INV = true;

const int ENCODER_2_OUT_1 = 35;
const int ENCODER_2_OUT_2 = 34;
const bool ENCODER_2_INV = true;

const int ENCODER_3_OUT_1 = 14;
const int ENCODER_3_OUT_2 = 27;
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
///////////////////  Kinematics  /////////////////////

const auto PLATFORM_WHEEL_OFFSET_METERS = 0.104f;
const auto WHEEL_RADIUS_METERS = 58.0f / 1000 / 2; // Wheel diameter is 58 mm
Kinematics kinematics(PLATFORM_WHEEL_OFFSET_METERS, WHEEL_RADIUS_METERS);

float linearX = 0;  // Meters  / sec
float linearY = 0;  // Meters  / sec
float angularZ = 0; // Degrees / sec


///////////////////  Web server  /////////////////////

const char* HOSTNAME = "botabotabot";

//////////////////////////////////////////

MotorController motor1Controller;
MotorController motor2Controller;
MotorController motor3Controller;

unique_ptr<Encoder> motor1Encoder;
unique_ptr<Encoder> motor2Encoder;
unique_ptr<Encoder> motor3Encoder;

void InitOta()
{
    // Code from: https://github.com/espressif/arduino-esp32/blob/master/libraries/ArduinoOTA/examples/BasicOTA/BasicOTA.ino

    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname(HOSTNAME);

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA
        .onStart([]()
                 {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("OTA Start updating " + type); })
        .onEnd([]()
               { Serial.println("\nOTA End"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
      Serial.printf("OTA Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

string parseCommand(const char *data, size_t len)
{
    std::string message(data, data + len);

    if (message.compare(0, 4, "pid ") == 0)
    {
        Serial << "PID PID PID" << endl;
        Serial << "->" << Kp << ' ' << Ki << ' ' << Kd << endl;
        int ok = sscanf(message.c_str() + strlen("pid "), "%f %f %f", &Kp, &Ki, &Kd);
        if (ok != 3)
        {
            Serial.println("Invalid format");
            return "Invalid format";
        }
        else
        {
            Serial << "->" << Kp << ' ' << Ki << ' ' << Kd << endl;
            motor1Pid.updateConstants(Kp, Ki, Kd);
            motor2Pid.updateConstants(Kp, Ki, Kd);
            motor3Pid.updateConstants(Kp, Ki, Kd);
        }
    }
    else if (message.compare(0, 4, "vel ") == 0)
    {
        Serial << "LinearX LinearY VelocityZ" << endl;
        int ok = sscanf(message.c_str() + strlen("vel "), "%f %f %f", &linearX, &linearY, &angularZ);
        if (ok != 3)
        {
            Serial.println("Invalid format");
            return "Invalid format";
        }
        else
        {
            Serial << "Velocities->" << linearX << ' ' << linearY << ' ' << angularZ << endl;
            motor1Pid.updateConstants(Kp, Ki, Kd);
            motor2Pid.updateConstants(Kp, Ki, Kd);
            motor3Pid.updateConstants(Kp, Ki, Kd);
        }
    }
    else
    {
        return "parseCommand UNKNONW: " + message;
    }
    return "parseCommand OK";
}

void ConnectToWifi()
{
    // Connect to Wi-Fi
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(HOSTNAME); //define hostname
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

    InitOta();

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
    int64_t count;
    float rpm;
    float requestedRpm;
    int pwm;
};

MotorStatus controlSpeed(MotorController &motorController, Encoder &motorEncoder, PID &motorPid, const float requestedRpm)
{
    auto count = motorEncoder.encoder().getCount();
    auto rpm = motorEncoder.getRPM();
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

HardwareSerial &toStream(HardwareSerial &s, float f1, float f2, float f3)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "[%.2f %.2f %.2f]", f1, f2, f3);
    s << buf;
    return s;
}


unsigned long prevPrintTime = 0;
void loop()
{
    ArduinoOTA.handle();

    // Serial << "1" << endl;
    // motor1Controller.spin(255);
    // delay(1000);
    // motor1Controller.spin(-255);
    // delay(3000);
    // motor1Controller.spin(0);
    // Serial << "2" << endl;
    // motor2Controller.spin(255);
    // delay(1000);
    // motor2Controller.spin(-255);
    // delay(3000);
    // motor2Controller.spin(0);
    // Serial << "3" << endl;
    // motor3Controller.spin(255);
    // delay(1000);
    // motor3Controller.spin(-255);
    // delay(3000);
    // motor3Controller.spin(0);
    // delay(1000);
    // Serial << "123" << endl;
    // motor1Controller.spin(255);
    // motor2Controller.spin(255);
    // motor3Controller.spin(255);
    // delay(3000);
    // motor1Controller.spin(0);
    // motor2Controller.spin(0);
    // motor3Controller.spin(0);


    readInput();

    // Compute desired wheel RPMs from desired body velocity
    Kinematics::rpm requestedRpm = kinematics.getRPM(linearX, linearY, angularZ);

    // Adjust wheel effort for each wheel to achieve requested RPM
    auto st1 = controlSpeed(motor1Controller, *motor1Encoder, motor1Pid, requestedRpm.motor1);
    auto st2 = controlSpeed(motor2Controller, *motor2Encoder, motor2Pid, requestedRpm.motor2);
    auto st3 = controlSpeed(motor3Controller, *motor3Encoder, motor3Pid, requestedRpm.motor3);

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
        Serial << " Requested RPM =";
        toStream(Serial, requestedRpm.motor1, requestedRpm.motor2, requestedRpm.motor3);
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
