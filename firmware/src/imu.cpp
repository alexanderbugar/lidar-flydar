#include "imu.h"
#include <cmath>
#include "config.h"

namespace {
constexpr float radiansPerDegree = 0.01745329252f;
int16_t word(const uint8_t* p) {
    return int16_t((uint16_t(p[0]) << 8) | p[1]);
}

Vec3 axes(const uint8_t* p, float scale) {
    return {word(p) * scale, word(p + 2) * scale, word(p + 4) * scale};
}
}

bool Imu::writeReg(uint8_t reg, uint8_t value) {
    bus_.beginTransmission(cfg::imuAddress);
    bus_.write(reg);
    bus_.write(value);
    return bus_.endTransmission() == 0;
}

bool Imu::readRegs(uint8_t reg, uint8_t* out, size_t count) {
    bus_.beginTransmission(cfg::imuAddress);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;
    if (bus_.requestFrom(cfg::imuAddress, count) != count) return false;
    for (size_t i = 0; i < count; ++i) out[i] = bus_.read();
    return true;
}

bool Imu::begin() {
    uint8_t id = 0;
    if (!readRegs(0x75, &id, 1) || id != 0x68) return false;
    if (!writeReg(0x6b, 0x01)) return false;
    delay(50);
    return writeReg(0x1a, 3) && writeReg(0x19, 4) &&
           writeReg(0x1b, 0) && writeReg(0x1c, 0);
}

bool Imu::readRaw(ImuSample& out) {
    uint8_t raw[14];
    if (!readRegs(0x3b, raw, sizeof(raw))) return false;
    out.accelG = axes(raw, 1.0f / 16384.0f);
    out.gyroRad = axes(raw + 8, radiansPerDegree / 131.0f);
    return true;
}

bool Imu::read(ImuSample& out) {
    if (!readRaw(out)) return false;
    out.gyroRad.x -= bias_.x;
    out.gyroRad.y -= bias_.y;
    out.gyroRad.z -= bias_.z;
    return true;
}

bool Imu::calibrate() {
    Vec3 sum{};
    unsigned count = 0;
    for (unsigned i = 0; i < 240; ++i) {
        delay(5);
        ImuSample s;
        if (!readRaw(s)) continue;
        const auto a = s.accelG;
        const auto g = s.gyroRad;
        const float a2 = a.x*a.x + a.y*a.y + a.z*a.z;
        const float g2 = g.x*g.x + g.y*g.y + g.z*g.z;
        if (a2 < 0.81f || a2 > 1.21f || g2 > 0.04f) continue;
        sum.x += g.x;
        sum.y += g.y;
        sum.z += g.z;
        ++count;
    }
    if (count < 200) return false;
    bias_ = {sum.x/count, sum.y/count, sum.z/count};
    return true;
}
