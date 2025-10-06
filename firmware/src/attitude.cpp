#include "attitude.h"
#include <cmath>

void Attitude::update(const ImuSample& sample, float dt) {
    const auto a = sample.accelG;
    const float ax = atan2f(a.y, a.z);
    const float ay = atan2f(-a.x, hypotf(a.y, a.z));
    if (!seeded_) {
        roll_ = ax;
        pitch_ = ay;
        seeded_ = true;
        return;
    }
    if (dt <= 0 || dt > 0.05f) return;
    const float retain = 0.35f / (0.35f + dt);
    roll_ += sample.gyroRad.x * dt;
    pitch_ += sample.gyroRad.y * dt;
    roll_ += (1 - retain) * remainderf(ax - roll_, 6.28318530718f);
    pitch_ += (1 - retain) * (ay - pitch_);
}
