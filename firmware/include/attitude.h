#pragma once

#include "imu.h"

class Attitude {
public:
    void update(const ImuSample& sample, float dt);
    float roll() const { return roll_; }
    float heading(Vec3 magBody) const;
    float pitch() const { return pitch_; }

private:
    bool seeded_ = false;
    float roll_ = 0;
    float pitch_ = 0;
};
