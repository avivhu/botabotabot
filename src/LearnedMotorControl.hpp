#include <Arduino.h>
#include <Streaming.h>
#include <vector>
#include <SPIFFS.h>


/// @brief Use precomputed mapping from desired RPM to motor effort (PWM).
/// This is instead of a PID controller that gives an adaptive policy of this function.
/// The motivation for this class was that one of my motor encoders was damaged, and 
/// with this class I don't need a working encoder for every motor.
class LearnedMotorControl
{
public:
    struct RpmPwm
    {
        float rpm;
        int pwm;

        bool operator<(const RpmPwm& other) const
        {
            return rpm < other.rpm;
        }
    };

    // This function gives the PID-like interface.
    // Ignore measured_value because the RPM->PWM mapping is simply fixed
    float compute(float setpoint, float measured_value) const
    {
        return LookupPwmByRpm(setpoint);
    }

    LearnedMotorControl() : _fileName("/data/motorcalib.txt"){};

    void Clear()
    {
        _rpmToPwmMap.clear();
    }

    void AddSample(float rpm, int pwm)
    {
        _rpmToPwmMap.push_back({rpm, pwm});
    }

    void Recompute()
    {
        sort(_rpmToPwmMap.begin(), _rpmToPwmMap.end());
    }

    void SaveToFile() const
    {
        auto file = SPIFFS.open(_fileName.c_str(), "w");
        for (auto pair : _rpmToPwmMap)
        {
            file.printf("%f\t%d\n", pair.rpm, pair.pwm);
        }
        file.close();
    }

    void LoadFromFile()
    {
        auto file = SPIFFS.open(_fileName.c_str(), "r");
        if (!file)
        {
            Serial << "No calibration data file in: " << String(_fileName.c_str()) << endl;
            return;
        }

        _rpmToPwmMap.clear();

        while (file.available())
        {
            auto rpm = file.parseFloat();
            auto pwm = (int)file.parseInt();
            file.readStringUntil('\n');// Consume newline
            assert(_rpmToPwmMap.empty() || (
                _rpmToPwmMap.back().rpm < rpm &&
                _rpmToPwmMap.back().pwm < pwm
            ));
            RpmPwm item = {rpm, pwm};
            _rpmToPwmMap.emplace_back(item);
            Serial << "rpm pwm: " << rpm << " " << pwm << endl;
        }
        Serial << "Read " << _rpmToPwmMap.size() << " calibration values." << endl;
        file.close();
    }

    void PrintMappingSamples()
    {
        for (auto rpm = -200.0f; rpm < 200.0f; rpm += 5)
        {
            Serial << "rpm pwm: " << rpm << " " << LookupPwmByRpm(rpm) << endl;
        }
    }

    int LookupPwmByRpm(float rpm) const
    {
        const auto absRpm = abs(rpm);
        float absPwm = LookupPwmByRpmInner(absRpm);

        auto pwm = (int)constrain(absPwm, 0, 255);

        if (rpm < 0)
        {
            pwm = -pwm;
        }
        return pwm;
    }

    int LookupPwmByRpmInner(float absRpm) const
    {
        RpmPwm searchValue = {absRpm, 0};
        auto it = lower_bound(_rpmToPwmMap.begin(), _rpmToPwmMap.end(), searchValue);

        if (it == _rpmToPwmMap.begin())
        {
            return _rpmToPwmMap.front().pwm;
        }
        if (it == _rpmToPwmMap.end())
        {
            return _rpmToPwmMap.back().pwm;
        }

        auto prev = (it - 1);

        auto rpm1 = it->rpm;
        auto pwm1 = it->pwm;
        auto rpm0 = prev->rpm;
        auto pwm0 = prev->pwm;

        auto res = linearInterp(rpm0, rpm1, pwm0, pwm1, absRpm);
        return res;
    }

    static float linearInterp(float x0, float x1, float y0, float y1, float x)
    {
        auto aa = (x - x0) / (x1 - x0);
        return y0 * (1 - aa) + y1 * aa;
    }

    std::vector<RpmPwm> _rpmToPwmMap;
    std::string _fileName;
};
