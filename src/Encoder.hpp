#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <deque>

const auto SLIDING_WINDOW_DURATION_USEC = 200.0 * 1000; // 2vel 0 0 000 MS

class Encoder
{
public:
    Encoder(uint8_t pin1, uint8_t pin2, float counts_per_rev, bool invert = false)
        : _encoder(),
          _counts_per_rev(counts_per_rev),
          _last_report(0)
    {
        if (invert)
        {
            auto temp_pin = pin1;
            pin1 = pin2;
            pin2 = temp_pin;
        }

        // Enable the weak pull down resistors
        // ESP32Encoder::useInternalWeakPullResistors=DOWN;
        // Enable the weak pull up resistors
        ESP32Encoder::useInternalWeakPullResistors = UP;
        _encoder.attachSingleEdge(pin1, pin2);

        getRPM(); // Init the "prev upadate time, prev ticks" and throw away the value.
    }

    ESP32Encoder &encoder()
    {
        return _encoder;
    }

    float getRPM()
    {
        // Compute the motor's RPM based on encoder ticks and delta time.
        int64_t encoder_ticks = _encoder.getCount();
        unsigned long current_time = micros();

        // Add to sliding window
        _tick_measurements.push_back({current_time, encoder_ticks});
        while (current_time - std::get<0>(_tick_measurements.front()) > SLIDING_WINDOW_DURATION_USEC)
        {
            _tick_measurements.pop_front();
        }

        const auto prev_update_time = std::get<0>(_tick_measurements.front());
        const auto prev_encoder_ticks = std::get<1>(_tick_measurements.front());

        const unsigned long dt = current_time - prev_update_time;
        if (dt == 0)
        {
            // Happens at the first time we run this
            return 0;
        }
        
        const int64_t delta_ticks = encoder_ticks - prev_encoder_ticks;

        // Convert the time from milliseconds to minutes
        const double dtm = ((double)dt) / 60000000;

        // Calculate wheel's speed (RPM)
        const auto rpm = (float)((delta_ticks / _counts_per_rev) / dtm);

        {
            auto sec_elapsed = (current_time - _last_report) / 1e6;
            if (sec_elapsed > 5)
            {
                Serial << "this=" << ((size_t)(this)) % 1000 << " dt(micros)=" << dt << " delta_ticks=" << delta_ticks << " RPM=" << rpm << endl;
                _last_report = current_time;
            }
        }
        return rpm;
    }

private:

    unsigned long _last_report; // For debugging
    ESP32Encoder _encoder;
    float _counts_per_rev;
    std::deque<std::tuple<unsigned long, int64_t>> _tick_measurements; // Sliding window samples of micros(), tick count
};
