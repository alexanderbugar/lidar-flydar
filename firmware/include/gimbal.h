#pragma once

#include <SimpleFOC.h>

class Gimbal {
public:
    Gimbal();
    bool begin();

private:
    BLDCDriver3PWM driver_;
};
