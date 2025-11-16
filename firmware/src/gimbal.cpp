#include "gimbal.h"
#include "config.h"

Gimbal::Gimbal() : driver_(cfg::motorA, cfg::motorB, cfg::motorC, cfg::motorEn) {}

bool Gimbal::begin() {
    driver_.voltage_power_supply = cfg::motorSupply;
    if (!driver_.init()) return false;
    driver_.disable();
    return true;
}
