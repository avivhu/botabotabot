#include <Arduino.h>
#include <Streaming.h>
#include <memory>
#include <vector>
#include <LearnedMotorControl.hpp>
#include <MotorController.hpp>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>

#include "private.hpp"
#include "web.hpp"
#include "Encoder.hpp"
#include "PID.h"
#include "kinematics.hpp"

using namespace std;

#define LEARNED_MOTOR_CONTROL

typedef std::function<int(float)> RpmToPwmComputeFunction;

using MotorController = OneInputMotorController;

LearnedMotorControl learnedMotorControl;

#ifdef LEARNED_MOTOR_CONTROL
typedef LearnedMotorControl AbstractPid;
#else
typedef PID AbstractPid;
#endif

const int SERIAL_BAUD = 115200;

////////////// Motor pins
const int MOTOR_0_ENABLE = 32;
const int MOTOR_0_DIRECTION = 33;
const int MOTOR_0_PWM_CHANNEL = 0;

const int MOTOR_1_ENABLE = 25;
const int MOTOR_1_DIRECTION = 26;
const int MOTOR_1_PWM_CHANNEL = 1;

const int MOTOR_2_ENABLE = 12;
const int MOTOR_2_DIRECTION = 13;
const int MOTOR_2_PWM_CHANNEL = 2;

///////// Motor Encoders /////////////
// Set pins so all wheels count "up" in the ccw direction when observed from outside the robot.
const int ENCODER_0_OUT_1 = 36; // AKA VP Pin
const int ENCODER_0_OUT_2 = 39; // AKA VN Pin
const bool ENCODER_0_INV = true;

const int ENCODER_1_OUT_1 = 35;
const int ENCODER_1_OUT_2 = 34;
const bool ENCODER_1_INV = true;

const int ENCODER_2_OUT_1 = 14;
const int ENCODER_2_OUT_2 = 27;
const bool ENCODER_2_INV = true;

const int N_MOTORS = 3;

const float ENCODER_COUNTS_PER_REVOLUTION = 360 * 74.8; // Use encoder CPR (360) times gear reduction ratio (74.8)

////////////// PID ////////////////////////

const auto K_P = 3.0;
const auto K_I = 0.4;
const auto K_D = 0;
const auto PWM_MIN = -255;
const auto PWM_MAX = 255;
array<PID, N_MOTORS> motorPids = {
    PID(PWM_MIN, PWM_MAX, K_P, K_I, K_D),
    PID(PWM_MIN, PWM_MAX, K_P, K_I, K_D),
    PID(PWM_MIN, PWM_MAX, K_P, K_I, K_D),
};
float Kp, Ki, Kd;
///////////////////  Kinematics  /////////////////////

const auto PLATFORM_WHEEL_OFFSET_METERS = 0.104f;
const auto WHEEL_RADIUS_METERS = 58.0f / 1000 / 2; // Wheel diameter is 58 mm
Kinematics kinematics(PLATFORM_WHEEL_OFFSET_METERS, WHEEL_RADIUS_METERS);

float linearX = 0;  // Meters  / sec
float linearY = 0;  // Meters  / sec
float angularZ = 0; // Degrees / sec

////////////////////////////////////////////// State
enum State
{
    STOP = 0,
    DRIVE = 1,
    CALIB = 2,
};

State state = State::DRIVE;

void StopAllMotors();

void SetState(State newState)
{
    if (state != newState)
    {
        Serial << "Old State: " << state << " New State: " << newState << endl;
        state = newState;
    }
}

///////////////////  Web server  /////////////////////

const char *HOSTNAME = "botabotabot";

//////////////////////////////////////////

array<MotorController, N_MOTORS> motorControllers;
array<unique_ptr<Encoder>, N_MOTORS> motorEncoders;

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
            for (auto &motorPid : motorPids)
            {
                motorPid.updateConstants(Kp, Ki, Kd);
            }
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
            SetState(State::DRIVE);
            Serial << "Velocities->" << linearX << ' ' << linearY << ' ' << angularZ << endl;
        }
    }
    else if (message.compare(0, 5, "calib") == 0)
    {
        SetState(State::CALIB);
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
    WiFi.setHostname(HOSTNAME); // define hostname
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

    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

#ifdef LEARNED_MOTOR_CONTROL
    learnedMotorControl.LoadFromFile();
    learnedMotorControl.PrintMappingSamples();
#endif

    ConnectToWifi();

    InitOta();

    StartWebServer([](uint8_t *data, size_t len)
                   { return parseCommand((const char *)data, len); });

    learnedMotorControl.LoadFromFile();

    motorEncoders[0].reset(new Encoder(ENCODER_0_OUT_1, ENCODER_0_OUT_2, ENCODER_COUNTS_PER_REVOLUTION, ENCODER_0_INV));
    motorEncoders[1].reset(new Encoder(ENCODER_1_OUT_1, ENCODER_1_OUT_2, ENCODER_COUNTS_PER_REVOLUTION, ENCODER_1_INV));
    motorEncoders[2].reset(new Encoder(ENCODER_2_OUT_1, ENCODER_2_OUT_2, ENCODER_COUNTS_PER_REVOLUTION, ENCODER_2_INV));

    motorControllers[0].init(MOTOR_0_ENABLE, MOTOR_0_DIRECTION, MOTOR_0_PWM_CHANNEL);
    motorControllers[1].init(MOTOR_1_ENABLE, MOTOR_1_DIRECTION, MOTOR_1_PWM_CHANNEL);
    motorControllers[2].init(MOTOR_2_ENABLE, MOTOR_2_DIRECTION, MOTOR_2_PWM_CHANNEL);

    SetState(State::STOP);
    Serial << "Ready" << endl;
}

struct MotorStatus
{
    int64_t count;
    float rpm;
    float requestedRpm;
    int pwm;
};

MotorStatus controlSpeed(MotorController &motorController, Encoder &motorEncoder, AbstractPid &motorPid, const float requestedRpm)
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

void StopAllMotors()
{
    for (auto mc : motorControllers)
    {
        mc.spin(0);
    }
}

unsigned long prevPrintTime = 0;

static void DriveLoop()
{

    // Compute desired wheel RPMs from desired body velocity
    Kinematics::rpm requestedRpm = kinematics.getRPM(linearX, linearY, angularZ);

    // Adjust wheel effort for each wheel to achieve requested RPM
    MotorStatus st[3];
    for (size_t i = 0; i < N_MOTORS; ++i)
    {
#ifdef LEARNED_MOTOR_CONTROL
        st[i] = controlSpeed(motorControllers[i], *motorEncoders[i], learnedMotorControl, requestedRpm.motorRpm[i]);
#else
        st[i] = controlSpeed(motorControllers[i], *motorEncoders[i], motorPids[i], requestedRpm.motorRpm[i]);
#endif
    }

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
        toStream(Serial, st[0].rpm, st[1].rpm, st[2].rpm);
        Serial << " Requested RPM =";
        toStream(Serial, requestedRpm.motorRpm[0], requestedRpm.motorRpm[1], requestedRpm.motorRpm[2]);
        Serial << " PWM = ";
        toStream(Serial, st[0].pwm, st[1].pwm, st[2].pwm);
        Serial << " Count = ";
        toStream(Serial, st[0].count, st[1].count, st[2].count);
        Serial << " Revs = ";
        toStream(Serial, (st[0].count / ENCODER_COUNTS_PER_REVOLUTION), (st[1].count / ENCODER_COUNTS_PER_REVOLUTION), (st[2].count / ENCODER_COUNTS_PER_REVOLUTION));
        Serial << endl;
    }
    delay(10);
}

void motorTest()
{
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
}

struct StatusRecord
{
    unsigned long micros;
    int pwm[N_MOTORS];
    float rpm[N_MOTORS];
    int64_t count[N_MOTORS];
};

StatusRecord ReadStatus()
{
    StatusRecord status;
    status.micros = micros();
    for (size_t i = 0; i < N_MOTORS; ++i)
    {
        status.pwm[i] = motorControllers[i].getPwm();
        status.count[i] = motorEncoders[i]->encoder().getCount();
        status.rpm[i] = motorEncoders[i]->getRPM();
    }

    return status;
}

void CollectStatuses(vector<StatusRecord> &statusOut, uint32_t durationMs, uint32_t sampleDelayMs = 100)
{
    auto start = millis();

    while (millis() - start < durationMs)
    {
        StatusRecord status = ReadStatus();
        statusOut.emplace_back(status);
        delay(sampleDelayMs);
    }
}

vector<StatusRecord> calibData;

void PrintCalibDataToFile(const vector<StatusRecord> &data, const char *outFileName)
{
    // Save Data
    auto ff = SPIFFS.open(outFileName, "w");
    if (!ff)
    {
        Serial << "Error opening calibdata file" << endl;
        return;
    }

    // Print header
    ff.printf("micros");
    ff.printf("\tpwm0\tpwm1\tpwm2");
    ff.printf("\trpm0\trpm1\trpm3");
    ff.printf("\tcount0\tcount1\tcount2");
    ff.printf("\n");

    // Print data
    for (const auto &status : calibData)
    {
        ff.printf("%ld", status.micros);
        ff.printf("\t%d\t%d\t%d", status.pwm[0], status.pwm[1], status.pwm[2]);
        ff.printf("\t%f\t%f\t%f", status.rpm[0], status.rpm[1], status.rpm[2]);
        ff.printf("\t%ld\t%ld\t%ld", status.count[0], status.count[1], status.count[2]);
        ff.printf("\n");
    }
    ff.close();
}

// Run motor at various speeds and record the resuling RPM.
// Save this to a calibration file for later use with learnedMotorControl.
void MotorControlCalibration()
{
    Serial << "Collecting calibration data" << endl;
    const auto DELAY_FOR_MOTOR_STEADY_STATE_MS = 500;
    const auto RECORD_DATA_MS = 1000;

    learnedMotorControl.Clear();
    StopAllMotors();
    delay(DELAY_FOR_MOTOR_STEADY_STATE_MS);

    calibData.clear();

    // Collect data from each motor into memory
    vector<StatusRecord> dummy;
    const auto ACTIVE_MOTOR = 0;

    vector<int> pwmsToSample;
    for (auto spin = 0; spin <= 255; spin += 20)
    {
        pwmsToSample.push_back(spin);
    }
    pwmsToSample.push_back(255);

    for (const auto pwm : pwmsToSample)
    {
        Serial << "Setting PWM " << pwm << endl;
        motorControllers[ACTIVE_MOTOR].spin(pwm);
        CollectStatuses(dummy, DELAY_FOR_MOTOR_STEADY_STATE_MS); // Throw away some data
        dummy.clear();
        CollectStatuses(calibData, RECORD_DATA_MS); // Record data

        auto lastRpm = calibData.back().rpm[ACTIVE_MOTOR];
        learnedMotorControl.AddSample(lastRpm, pwm);
    }

    PrintCalibDataToFile(calibData, "/data/calibdata.tsv");

    learnedMotorControl.Recompute();

    learnedMotorControl.SaveToFile();

    learnedMotorControl.PrintMappingSamples();
    Serial << "Calibration done" << endl;
    StopAllMotors();
}

void loop()
{
    ArduinoOTA.handle();
    readInput();

    if (state == State::DRIVE)
    {
        DriveLoop();
    }
    else if (state == State::CALIB)
    {
        MotorControlCalibration();
        SetState(State::STOP);
    }
    else
    {
        assert(state == State::STOP);
        StopAllMotors();
        delay(10);
    }
}
