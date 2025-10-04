#pragma once

#include <Wire.h>

struct Vec3 {
    float x, y, z;
};

struct ImuSample {
    Vec3 accelG;
    Vec3 gyroRad;
};

class Imu {
public:
    explicit Imu(TwoWire& bus) : bus_(bus) {}
    bool begin();
    bool read(ImuSample& out);

private:
    TwoWire& bus_;
    bool writeReg(uint8_t reg, uint8_t value);
    bool readRegs(uint8_t reg, uint8_t* out, size_t count);
};
