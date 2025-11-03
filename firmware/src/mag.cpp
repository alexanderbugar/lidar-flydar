#include "mag.h"
#include "config.h"

namespace {
int16_t word(const uint8_t* p) {
    return int16_t(uint16_t(p[0]) | (uint16_t(p[1]) << 8));
}
}

bool Mag::writeReg(uint8_t reg, uint8_t value) {
    bus_.beginTransmission(cfg::magAddress);
    bus_.write(reg);
    bus_.write(value);
    return bus_.endTransmission() == 0;
}

bool Mag::readRegs(uint8_t reg, uint8_t* out, size_t count) {
    bus_.beginTransmission(cfg::magAddress);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;
    if (bus_.requestFrom(cfg::magAddress, count) != count) return false;
    for (size_t i = 0; i < count; ++i) out[i] = bus_.read();
    return true;
}

bool Mag::begin() {
    if (!writeReg(0x0a, 0x80)) return false;
    delay(10);
    // set/reset period, then continuous 50 Hz, 8 G, OSR 512.
    return writeReg(0x0b, 1) && writeReg(0x09, 0x15);
}

bool Mag::read(Vec3& counts) {
    uint8_t status = 0;
    if (!readRegs(0x06, &status, 1) || !(status & 1)) return false;
    uint8_t raw[6];
    if (!readRegs(0, raw, sizeof(raw)) || (status & 2)) return false;
    counts = {float(word(raw)), float(word(raw + 2)), float(word(raw + 4))};
    return true;
}

Vec3 Mag::corrected(Vec3 counts) {
    const float v[] = {counts.x - cfg::magBias[0], counts.y - cfg::magBias[1],
                       counts.z - cfg::magBias[2]};
    float out[3]{};
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            out[row] += cfg::magMatrix[row][col] * v[col];
    return {out[0], out[1], out[2]};
}
