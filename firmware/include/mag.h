#pragma once

#include "imu.h"

class Mag {
public:
    explicit Mag(TwoWire& bus) : bus_(bus) {}
    bool begin();
    bool read(Vec3& counts);

private:
    TwoWire& bus_;
    bool writeReg(uint8_t reg, uint8_t value);
    bool readRegs(uint8_t reg, uint8_t* out, size_t count);
};
