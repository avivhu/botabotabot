#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>

class Encoder
{
public:
    Encoder(uint8_t pin1, uint8_t pin2, float counts_per_rev, bool invert = false)
        : _encoder(), counts_per_rev_(counts_per_rev)
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
        unsigned long dt = current_time - prev_update_time_;

        // Convert the time from milliseconds to minutes
        double dtm = ((double)dt) / 60000000;
        int64_t delta_ticks = encoder_ticks - prev_encoder_ticks_;

        // Calculate wheel's speed (RPM)
        prev_update_time_ = current_time;
        prev_encoder_ticks_ = encoder_ticks;

        return (float)((delta_ticks / counts_per_rev_) / dtm);
    }

private:
    ESP32Encoder _encoder;
    unsigned long prev_update_time_;
    int64_t prev_encoder_ticks_;
    float counts_per_rev_;
};
