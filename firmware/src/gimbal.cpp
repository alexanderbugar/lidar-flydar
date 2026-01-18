#include "gimbal.h"
#include "config.h"

bool AxisEncoder::begin() {
    if (getSensorAngle() < 0) return false;
    Sensor::init();
    return valid_;
}

float AxisEncoder::getSensorAngle() {
    valid_ = false;
    bus_.beginTransmission(cfg::axisAddress);
    bus_.write(uint8_t(0x0b));
    if (bus_.endTransmission(false) != 0) return -1;
    if (bus_.requestFrom(cfg::axisAddress, size_t(3)) != 3) return -1;
    const int status = bus_.read();
    const int hi = bus_.read();
    const int lo = bus_.read();
    // MD set, ML/MH clear. use raw angle, regardless of EEPROM settings.
    if ((status & 0x38) != 0x20) return -1;
    valid_ = true;
    return (((hi & 0x0f) << 8) | lo) * (2.0f*PI / 4096.0f);
}

Gimbal::Gimbal()
    : encoder_(Wire1), driver_(cfg::motorA, cfg::motorB, cfg::motorC, cfg::motorEn),
      motor_(cfg::motorPolePairs) {}

bool Gimbal::begin() {
    const bool busOk = Wire1.begin(cfg::axisSda, cfg::axisScl, cfg::axisI2cHz);
    Wire1.setTimeOut(2);
    encoderOk_ = busOk && encoder_.begin();
    if (!cfg::driveMotor || !encoderOk_) return encoderOk_;
    driver_.voltage_power_supply = cfg::motorSupply;
    if (!driver_.init()) return false;
    motor_.linkDriver(&driver_);
    motor_.linkSensor(&encoder_);
    motor_.controller = MotionControlType::velocity;
    motor_.torque_controller = TorqueControlType::voltage;
    motor_.voltage_sensor_align = 1.0f;
    motor_.PID_velocity.P = 0.32f;
    motor_.PID_velocity.I = 1.2f;
    motor_.LPF_velocity.Tf = 0.035f;
    // motor_.controller = MotionControlType::velocity_openloop;  // sang, didn't turn under load
    motor_.voltage_limit = cfg::motorVoltage;
    motor_.velocity_limit = 2;
    running_ = motor_.init() && motor_.initFOC() && encoder_.valid();
    if (!running_) driver_.disable();
    return running_;
}

void Gimbal::tick() {
    if (!running_) {
        if (encoderOk_) encoder_.update();
        return;
    }

    motor_.loopFOC();
    if (!encoder_.valid()) {
        motor_.disable();
        running_ = false;
        return;
    }
    motor_.move(cfg::motorSpeedRad);
}

bool Gimbal::position(float& radians) {
    if (!encoderOk_ || !encoder_.valid()) return false;
    radians = encoder_.getMechanicalAngle();
    return true;
}
