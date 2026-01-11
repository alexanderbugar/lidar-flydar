#pragma once

#include <SimpleFOC.h>

class Gimbal {
public:
    Gimbal();
    bool begin();
    void tick();

private:
    BLDCDriver3PWM driver_;
    BLDCMotor motor_;
    bool running_ = false;
};
