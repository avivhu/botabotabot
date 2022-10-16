#include <Arduino.h>
#include <Streaming.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// Required to replace with your xbox address
// XboxSeriesXControllerESP32_asukiaaa::Core
// xboxController("44:16:22:5e:b2:d4");


/// @brief Convert Xbox controller inputs to robot drive commands.
class XboxDriveController
{
public:
    enum DriveState
    {
        NO_CHANGE,
        TURN_OFF,
        TURN_ON,
    };

    void Setup()
    {
        _driveOn = false;
        _xboxController.begin();
    }

    /// @brief Return robot speeds and other control input from Xbox Controller.
    /// @param dx          Proportional X velocity  in range [-1, 1].
    /// @param dy          Proportional Y velocity  in range [-1, 1].
    /// @param dRot        Proportional angular speed in range [-1, 1].
    /// @param driveState  Indicates drive on / drive off changes.
    /// @param restart     Robot restart is requested.
    /// @return            true if succeded to read controller.
    bool TryReadXboxController(float &dx, float &dy, float &dRot, DriveState &driveState, bool &restart)
    {
        _xboxController.onLoop();
        if (_xboxController.isConnected())
        {
            // Serial.println("Xbox controller address: " + _xboxController.buildDeviceAddressStr());
            // Serial.print(_xboxController.xboxNotif.toString());
            // unsigned long receivedAt = _xboxController.getReceiveNotificationAt();

            restart = (_xboxController.xboxNotif.btnLB && _xboxController.xboxNotif.btnRB);

            // Dir up to turn on drive, Dir down to turn off drive
            // X button is pressed
            if (!_driveOn && _xboxController.xboxNotif.btnDirUp)
            {
                // Drive on
                // Save the current joystick values and use them to trim the joysticks.
                // If this the neutral joystick values are non-zero. Copy the trim values.
                _joysticksTrim = _xboxController.xboxNotif;
                driveState = DriveState::TURN_ON;
                _driveOn = true;
            }
            else if (_driveOn && _xboxController.xboxNotif.btnDirDown)
            {
                driveState = DriveState::TURN_OFF;
                _driveOn = false;
            }
            else
            {
                driveState = DriveState::NO_CHANGE;
            }

            uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
            dx = ConvertJoystickToNeg1to1(_xboxController.xboxNotif.joyLHori, _joysticksTrim.joyLHori);
            dy = ConvertJoystickToNeg1to1(_xboxController.xboxNotif.joyLVert, _joysticksTrim.joyLVert);
            dRot = ConvertJoystickToNeg1to1(_xboxController.xboxNotif.joyRHori, _joysticksTrim.joyRHori);
            // Serial << "Xbox controller " << dx << " " << dy << " " << dRot << endl;
            // Serial.print("joyLHori rate: ");
            // Serial.println(dx);
            // Serial.print("joyLVert rate: ");
            // Serial.println(dy);
            // Serial.print("joyRHori rate: ");
            // Serial.println(da);
            // Serial.println("battery " + String(_xboxController.battery) + "%");
            // Serial.println("received at " + String(receivedAt));
            return true;
        }
        else
        {
            Serial << "Xbox controller not connected. Count: " << _xboxController.getCountFailedConnection() << endl;
            return false;
        }
    }

private:
    static float ConvertJoystickToNeg1to1(const float value, const float trim)
    {
        const uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
        float res = (value - trim) / joystickMax; // Convert to -0.5, 0.5 range (nearly, because of the trim)
        res = res * 2;                            // Convert to -1,1 range
        res = constrain(res, -1, 1);
        return res;
    }

    XboxSeriesXControllerESP32_asukiaaa::Core _xboxController;
    typeof(_xboxController.xboxNotif) _joysticksTrim;
    bool _driveOn;
};
