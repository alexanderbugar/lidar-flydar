#pragma once

#include <SimpleFOC.h>

class AxisEncoder : public Sensor {
public:
    explicit AxisEncoder(TwoWire& bus) : bus_(bus) {}
    bool begin();
    bool valid() const { return valid_; }

private:
    float getSensorAngle() override;
    TwoWire& bus_;
    bool valid_ = false;
};

class Gimbal {
public:
    Gimbal();
    bool begin();
    void tick();
    void stop();
    bool position(float& radians);

private:
    AxisEncoder encoder_;
    BLDCDriver3PWM driver_;
    BLDCMotor motor_;
    bool encoderOk_ = false;
    bool running_ = false;
    uint32_t lastTickUs_ = 0;
    float target_ = 0;
};
