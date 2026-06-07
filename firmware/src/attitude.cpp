#include "attitude.h"
#include <cmath>

void Attitude::update(const ImuSample& sample, float dt) {
    const Vec3 a = sample.accelG;
    const float a2 = a.dot(a);
    const bool gravity = a2 > 0.72f && a2 < 1.32f;
    if (!seeded_) {
        if (!gravity) return;
        const float r = atan2f(a.y, a.z);
        const float p = atan2f(-a.x, hypotf(a.y, a.z));
        q_ = Quat::about({0, 1, 0}, p) * Quat::about({1, 0, 0}, r);
        seeded_ = true;
    }
    if (dt <= 0 || dt > 0.05f) return;

    const auto g = sample.gyroRad;
    Quat dq = q_ * Quat{0, g.x*0.5f, g.y*0.5f, g.z*0.5f};
    if (gravity) {
        // Madgwick sec. 3.3: gravity residual in sensor axes, then J transpose.
        const auto w = q_.w, x = q_.x, y = q_.y, z = q_.z;
        const Vec3 err = q_.conjugate().rotate({0, 0, 1}) - a*(1/sqrtf(a2));
        const float sw = 2*(-y*err.x + x*err.y);
        const float sx = 2*(z*err.x + w*err.y - 2*x*err.z);
        const float sy = 2*(-w*err.x + z*err.y - 2*y*err.z);
        const float sz = 2*(x*err.x + y*err.y);
        const float norm = sqrtf(sw*sw + sx*sx + sy*sy + sz*sz);
        if (norm > 1e-6f) {
            const float gain = 0.055f / norm;  // .12 made the motor buzz show up as tilt
            dq.w -= gain*sw;
            dq.x -= gain*sx;
            dq.y -= gain*sy;
            dq.z -= gain*sz;
        }
    }
    q_ = Quat{q_.w + dq.w*dt, q_.x + dq.x*dt,
              q_.y + dq.y*dt, q_.z + dq.z*dt}.unit();
}

void Attitude::correctHeading(Vec3 m, float dt) {
    if (!seeded_ || dt <= 0 || dt > 0.2f) return;
    const float field2 = m.dot(m);
    if (field2 < 0.55f || field2 > 1.55f) return;

    // tilt compensation without extracting roll and pitch at the pole.
    const Vec3 level = q_.rotate(m);
    if (hypotf(level.x, level.y) < 0.2f) return;
    const float error = -atan2f(level.y, level.x);
    q_ = (Quat::about({0, 0, 1}, error * dt / (2.0f + dt)) * q_).unit();
}
