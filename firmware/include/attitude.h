#pragma once

#include "imu.h"

class Attitude {
public:
    void update(const ImuSample& sample, float dt);
    void correctHeading(Vec3 magBody, float dt);
    float roll() const;
    float pitch() const;
    bool ready() const { return seeded_; }
    Quat rotation() const { return q_; }

private:
    bool seeded_ = false;
    Quat q_;
};
