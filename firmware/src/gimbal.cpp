#include "gimbal.h"
#include "config.h"

Gimbal::Gimbal()
    : driver_(cfg::motorA, cfg::motorB, cfg::motorC, cfg::motorEn),
      motor_(cfg::motorPolePairs) {}

bool Gimbal::begin() {
    driver_.voltage_power_supply = cfg::motorSupply;
    if (!driver_.init()) return false;
    motor_.linkDriver(&driver_);
    motor_.controller = MotionControlType::velocity_openloop;
    motor_.voltage_limit = cfg::motorVoltage;
    motor_.velocity_limit = 2;
    running_ = motor_.init() != 0;
    if (!running_) driver_.disable();
    return running_;
}

void Gimbal::tick() {
    if (running_) motor_.move(cfg::motorSpeedRad);
}
