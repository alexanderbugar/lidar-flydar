#pragma once

#include <Wire.h>

#include "math3d.h"

struct ImuSample {
    Vec3 accelG;
    Vec3 gyroRad;
};

class Imu {
public:
    explicit Imu(TwoWire& bus) : bus_(bus) {}
    bool begin();
    bool read(ImuSample& out);
    bool calibrate();

private:
    TwoWire& bus_;
    Vec3 bias_{};
    bool readRaw(ImuSample& out);
    bool writeReg(uint8_t reg, uint8_t value);
    bool readRegs(uint8_t reg, uint8_t* out, size_t count);
};
