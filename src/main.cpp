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
#include "XboxDriveController.hpp"

using namespace std;

#define LEARNED_MOTOR_CONTROL // Set to use precomputed motor PWM instead of the PID controller

typedef std::function<int(float)> RpmToPwmComputeFunction;

using MotorController = OneInputMotorController;

LearnedMotorControl learnedMotorControl;

#ifdef LEARNED_MOTOR_CONTROL
typedef LearnedMotorControl AbstractPid;
#else
typedef PID AbstractPid;
#endif

const int SERIAL_BAUD = 115200;
const int N_MOTORS = 3;
const auto MAX_MOTOR_RPM = 110; // If we try to drive above the RPM, the wheel propotional speeds won't be right, causing skidding.
const int DRIVE_LOOP_DELAY_MS = 10;


// => Platform positive rotation is Counter Clock-Wise as seen from the top looking down.
// => Wheels positive rotation is Counter Clock-Wise as seen from inside the platform looking out.
// => Encoders positive rotation is same as wheels.

////////////// Motor Pins //////////////
const int MOTOR_0_ENABLE = 32;
const int MOTOR_0_DIRECTION = 33;
const int MOTOR_0_PWM_CHANNEL = 0;
const bool MOTOR_0_INV = false;

const int MOTOR_1_ENABLE = 25;
const int MOTOR_1_DIRECTION = 26;
const int MOTOR_1_PWM_CHANNEL = 1;
const bool MOTOR_1_INV = false;

const int MOTOR_2_ENABLE = 12;
const int MOTOR_2_DIRECTION = 13;
const int MOTOR_2_PWM_CHANNEL = 2;
const bool MOTOR_2_INV = false;

///////// Motor Encoder Pins /////////////

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

const float ENCODER_COUNTS_PER_REVOLUTION = 11 * 45; // Use encoder CPR (11) times gear reduction ratio (45)

////////////// PID Control ////////////////////////

const auto K_P = 3.0;
const auto K_I = 0.2;
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

// Physical dimensions of the platform
const auto PLATFORM_WHEEL_OFFSET_METERS = 0.104f;
const auto WHEEL_RADIUS_METERS = 58.0f / 1000 / 2; // Wheel diameter is 58 mm

Kinematics kinematics(PLATFORM_WHEEL_OFFSET_METERS, WHEEL_RADIUS_METERS);

float linearX = 0;  // Meters  / sec
float linearY = 0;  // Meters  / sec
float angularZ = 0; // Degrees / sec

///////////////////////////////////////


XboxDriveController xboxDriveController;
bool tryUseXboxController = false;

//////////////// Robot State //////////

enum RobotState
{
    STOP = 0,
    DRIVE = 1,
    CALIB = 2,
    MOTOR_TEST = 3,
};

RobotState state = RobotState::STOP;
void StopAllMotors();

void SetState(RobotState newState)
{
    if (state != newState)
    {
        Serial << "Old State: " << state << " New State: " << newState << endl;
        state = newState;
    }
}

///////////////////  Web server  /////////////////////

const char *HOSTNAME = "botabotabot";

/////////////////////////////////////////////////////

array<MotorController, N_MOTORS> motorControllers = {
    MotorController(MOTOR_0_INV),
    MotorController(MOTOR_1_INV),
    MotorController(MOTOR_2_INV)};
array<unique_ptr<Encoder>, N_MOTORS> motorEncoders;

// Over-the-air code upload for ESP32
void InitOta()
{
    // Code from: https://github.com/espressif/arduino-esp32/blob/master/libraries/ArduinoOTA/examples/BasicOTA/BasicOTA.ino

    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname(HOSTNAME);

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

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

// Handle text command, which may come from Serial Port, Web Socket, etc.
string HandleTextCommand(const std::string& message)
{
    Serial.write("HandleTextCommand: ");
    Serial.write(message.c_str());

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
            SetState(RobotState::DRIVE);
            Serial << "Velocities->" << linearX << ' ' << linearY << ' ' << angularZ << endl;
        }
    }
    else if (message.compare(0, 4, "mot ") == 0)
    {
        // Test motors - set PWM directly
        Serial << "PWM PWM PWM" << endl;
        int pwm[3];
        int ok = sscanf(message.c_str() + strlen("mot "), "%d %d %d", &pwm[0], &pwm[1], &pwm[2]);
        if (ok != 3)
        {
            Serial.println("Invalid format");
            return "Invalid format";
        }
        else
        {
            SetState(RobotState::MOTOR_TEST);
            Serial << "PWM->" << pwm[0] << ' ' << pwm[1] << ' ' << pwm[2] << endl;
            for (auto i = 0; i < 3; ++i)
            {
                motorControllers[i].spin(pwm[i]);
            }
        }
    }
    else if (message.compare(0, 5, "calib") == 0)
    {
        SetState(RobotState::CALIB);
    }
    else if (message.compare(0, 4, "xbox") == 0)
    {
        tryUseXboxController = !tryUseXboxController;
        if (tryUseXboxController)
        {
            Serial << "Start trying to find Xbox controller" << endl;
        }
        else
        {
            Serial << "Stop trying to find Xbox controller" << endl;
        }
    }
    else
    {
        return "HandleTextCommand UNKNONW: " + message;
    }
    return "HandleTextCommand OK";
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


bool TryReadSerialLine(string& str)
{
    static char buffer[128];
    static int buflen = 0;

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
            str = string(buffer, buffer + buflen);
            buflen = 0;
            return true;
        }
    }
    return false;
}

void HandleSerialInput()
{
    // Accumulate serial input text.
    // Run HandleTextCommand only when the entire input with newline is received.
    string line;
    if (TryReadSerialLine(line))
    {
        auto res = HandleTextCommand(line);
        Serial << res.c_str() << endl;
    }
}

// Read input from Xbox Controller if connected
bool TryReadXboxControllerInput()
{
    float dx, dy, dRot;
    XboxDriveController::DriveState driveState;
    bool restart;
    if (!xboxDriveController.TryReadXboxController(dx, dy, dRot, driveState, restart))
    {
        return false;
    }

    if (restart)
    {
        Serial << "Restarting Robot" << endl;
        ESP.restart();
    }
    else if (driveState == XboxDriveController::DriveState::TURN_OFF)
    {
        Serial << "Turning drive off" << endl;
        SetState(RobotState::STOP);
    }
    else if (driveState == XboxDriveController::DriveState::TURN_ON)
    {
        Serial << "Turnining drive on" << endl;
        SetState(RobotState::DRIVE);
    }
    else
    {
        assert(driveState == XboxDriveController::DriveState::NO_CHANGE);

        // Convert proportional velocities in dx,dy,dRot [-1,1] to absolute units (meters/sec, degrees/sec)

        // Convert the wheel top speed to a linear speed
        const auto maxWheelRotationVelocityRadiansPerSec = MAX_MOTOR_RPM * (2 * M_PI) / 60;
        const auto maxTranslationVelocityMetersPerSec = WHEEL_RADIUS_METERS * maxWheelRotationVelocityRadiansPerSec;
        const auto maxPlatformRotationVelocityRadiansPerSec = maxTranslationVelocityMetersPerSec / PLATFORM_WHEEL_OFFSET_METERS;
        const auto maxPlatformRotationVelocityDegreesPerSec = maxPlatformRotationVelocityRadiansPerSec * 180 / M_PI;

        linearX = dx * maxTranslationVelocityMetersPerSec;
        linearY = dy * maxTranslationVelocityMetersPerSec;
        angularZ = dRot * maxPlatformRotationVelocityDegreesPerSec;
    }
    return true;
}

bool TryConnectToXboxController(const int attempts = 5)
{
    // Try to connect to XBOX controller for a few seconds
    bool connected = false;
    xboxDriveController.Setup();
    Serial << "Trying to connect to XBOX controller: ";
    for (auto i = 0; i < attempts; ++i)
    {
        Serial << ".";
        connected = TryReadXboxControllerInput();
        if (connected)
        {
            break;
        }
        delay(1000);
    }
    if (connected)
    {
        Serial << "Connected !" << endl;
    }
    else
    {
        Serial << "Failed to connect." << endl;
    }
    return connected;
}

void setup()
{
    try
    {
        Serial.begin(115200);
        Serial.println("Booting");

        Serial << "Pausing for 3 seconds while you enter first command." << endl;
        delay(3000);
        Serial << "Continuing boot" << endl;

        if (!SPIFFS.begin(true))
        {
            throw runtime_error("An Error has occurred while mounting SPIFFS");
        }

        ConnectToWifi();

        InitOta();

        StartWebServer(
            [](uint8_t *data, size_t len)
            {
                auto s = (const char *)data;
                string message(s, s + len);
                return HandleTextCommand(message);
            });

    tryUseXboxController = TryConnectToXboxController();

    #ifdef LEARNED_MOTOR_CONTROL
        try
        {
            learnedMotorControl.LoadFromFile();
        }
        catch(const std::exception& e)
        {
            Serial << "Bad calibration file, ignoring it: " << e.what() << endl;
        }
        
        learnedMotorControl.PrintMappingSamples();
    #endif

        motorEncoders[0].reset(new Encoder(ENCODER_0_OUT_1, ENCODER_0_OUT_2, ENCODER_COUNTS_PER_REVOLUTION, ENCODER_0_INV));
        motorEncoders[1].reset(new Encoder(ENCODER_1_OUT_1, ENCODER_1_OUT_2, ENCODER_COUNTS_PER_REVOLUTION, ENCODER_1_INV));
        motorEncoders[2].reset(new Encoder(ENCODER_2_OUT_1, ENCODER_2_OUT_2, ENCODER_COUNTS_PER_REVOLUTION, ENCODER_2_INV));

        motorControllers[0].init(MOTOR_0_ENABLE, MOTOR_0_DIRECTION, MOTOR_0_PWM_CHANNEL);
        motorControllers[1].init(MOTOR_1_ENABLE, MOTOR_1_DIRECTION, MOTOR_1_PWM_CHANNEL);
        motorControllers[2].init(MOTOR_2_ENABLE, MOTOR_2_DIRECTION, MOTOR_2_PWM_CHANNEL);

        SetState(RobotState::STOP);
        Serial << "Ready" << endl;
    }
    catch(const std::exception& e)
    {
        Serial << "Exception: " << e.what() << endl;
    }
}

struct MotorStatus
{
    int64_t count;
    float rpm;
    float requestedRpm;
    int pwm;
};

MotorStatus ControlSpeed(MotorController &motorController, Encoder &motorEncoder, AbstractPid &motorPid, const float requestedRpm)
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

HardwareSerial &ToStream(HardwareSerial &s, float f1, float f2, float f3)
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


static void DriveLoop()
{
    // Compute desired wheel RPMs from desired body velocity
    Kinematics::rpm requestedRpm = kinematics.getRPM(linearX, linearY, angularZ);

    // Adjust wheel effort for each wheel to achieve requested RPM
    MotorStatus st[3];
    for (size_t i = 0; i < N_MOTORS; ++i)
    {
#ifdef LEARNED_MOTOR_CONTROL
        st[i] = ControlSpeed(motorControllers[i], *motorEncoders[i], learnedMotorControl, requestedRpm.motorRpm[i]);
#else
        st[i] = ControlSpeed(motorControllers[i], *motorEncoders[i], motorPids[i], requestedRpm.motorRpm[i]);
#endif
    }

    auto now = millis();
    bool printStatus = false;
    static unsigned long prevPrintTime = 0;
    if (now - prevPrintTime > 1000)
    {
        prevPrintTime = now;
        printStatus = true;
    }

    if (printStatus)
    {
        Serial << " RPM =";
        ToStream(Serial, st[0].rpm, st[1].rpm, st[2].rpm);
        Serial << " Requested RPM =";
        ToStream(Serial, requestedRpm.motorRpm[0], requestedRpm.motorRpm[1], requestedRpm.motorRpm[2]);
        Serial << " PWM = ";
        ToStream(Serial, st[0].pwm, st[1].pwm, st[2].pwm);
        Serial << " Count = ";
        ToStream(Serial, st[0].count, st[1].count, st[2].count);
        Serial << " Revs = ";
        ToStream(Serial, (st[0].count / ENCODER_COUNTS_PER_REVOLUTION), (st[1].count / ENCODER_COUNTS_PER_REVOLUTION), (st[2].count / ENCODER_COUNTS_PER_REVOLUTION));
        Serial << endl;

        {
            char buf[128];
            snprintf(buf, sizeof(buf), "linearX, linearY, angularZ [%.2f %.2f %.2f]", linearX, linearY, angularZ);
            SendTextWeb(buf);
        }
    }
}

struct MotorStatusRecord
{
    unsigned long micros;
    int pwm[N_MOTORS];
    float rpm[N_MOTORS];
    int64_t count[N_MOTORS];
};

MotorStatusRecord ReadStatus()
{
    MotorStatusRecord status;
    status.micros = micros();
    for (size_t i = 0; i < N_MOTORS; ++i)
    {
        status.pwm[i] = motorControllers[i].getPwm();
        status.count[i] = motorEncoders[i]->encoder().getCount();
        status.rpm[i] = motorEncoders[i]->getRPM();
    }

    return status;
}

// Read and accumulate motor status for the specfied duration.
void CollectStatuses(vector<MotorStatusRecord> &statusOut, uint32_t durationMs, uint32_t sampleDelayMs = 100)
{
    auto start = millis();

    while (millis() - start < durationMs)
    {
        MotorStatusRecord status = ReadStatus();
        statusOut.emplace_back(status);
        delay(sampleDelayMs);
    }
}

void PrintCalibDataToFile(const vector<MotorStatusRecord> &calibData, const char *outFileName)
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
    try
    {

        Serial << "Collecting calibration data" << endl;
        const auto DELAY_FOR_MOTOR_STEADY_STATE_MS = 500;
        const auto RECORD_DATA_MS = 1000;

        learnedMotorControl.Clear();
        StopAllMotors();
        delay(DELAY_FOR_MOTOR_STEADY_STATE_MS);


        // Collect data from each motor into memory
        vector<MotorStatusRecord> motorCalibData;
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

            // Throw away some data while motor speed stabilizes
            vector<MotorStatusRecord> dummy;
            CollectStatuses(dummy, DELAY_FOR_MOTOR_STEADY_STATE_MS);
            
             // Record data
            CollectStatuses(motorCalibData, RECORD_DATA_MS);
            auto lastRpm = motorCalibData.back().rpm[ACTIVE_MOTOR];
            learnedMotorControl.AddSample(lastRpm, pwm);
        }

        PrintCalibDataToFile(motorCalibData, "/data/calibdata.tsv");

        learnedMotorControl.Recompute();

        learnedMotorControl.SaveToFile();

        learnedMotorControl.PrintMappingSamples();
        Serial << "Calibration done" << endl;
        StopAllMotors();
    }
    catch(const std::exception& e)
    {
        Serial << "Exception: " << e.what() << endl;
    }
}

void loop()
{
    ArduinoOTA.handle();
    HandleSerialInput();

    if (tryUseXboxController)
    {
        bool ok = TryReadXboxControllerInput();
    }

    if (state == RobotState::DRIVE)
    {
        DriveLoop();
        delay(DRIVE_LOOP_DELAY_MS);
    }
    else if (state == RobotState::CALIB)
    {
        MotorControlCalibration();
        SetState(RobotState::STOP);
    }
    else if (state == RobotState::STOP)
    {
        StopAllMotors();
        delay(DRIVE_LOOP_DELAY_MS);
    }
    else if (state == RobotState::MOTOR_TEST)
    {
        // Print status of encoders
        MotorStatusRecord status = ReadStatus();
        for (int i = 0; i < N_MOTORS; ++i)
        {
            char msg[512];
            snprintf(msg, sizeof(msg), "Motor %d: RPM=%6f PWM=%3d COUNT=%7d", i, status.rpm[i], status.pwm[i], status.count[i]);
            Serial << msg << endl;
        }
        Serial << endl;
        delay(1000);
    }
    else
    {
        assert(false);
    }
    delay(1);
}
