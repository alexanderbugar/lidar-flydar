#include "fusion.h"
#include "config.h"
#include <algorithm>
#include <cstdlib>
#include <cmath>

int64_t scan::FrameClock::observe(uint16_t stampMs, int64_t receivedUs) {
    constexpr int64_t wireUs = (ld::frameBytes * 10000000LL + cfg::lidarBaud - 1)
                              / cfg::lidarBaud;
    constexpr int wrapMs = 30000;
    const int deltaMs = (stampMs + wrapMs - lastStamp_) % wrapMs;
    const int64_t arrivalUs = receivedUs - wireUs;
    const int64_t gapUs = receivedUs - lastRxUs_;
    if (!lastRxUs_ || gapUs > 1000000 || std::abs(gapUs - deltaMs*1000LL) > 100000) {
        sensorUs_ = 0;
        offsetUs_ = arrivalUs;
    } else {
        sensorUs_ += deltaMs*1000LL;
        // uart scheduling only makes a packet late. follow the low edge
        // and let it creep up slowly for clock drift.
        offsetUs_ = std::min<int64_t>(offsetUs_ + 2, arrivalUs - sensorUs_);
    }
    lastStamp_ = stampMs;
    lastRxUs_ = receivedUs;
    return offsetUs_ + sensorUs_;
}

void scan::PoseHistory::add(AxisSample pose) {
    if (poses_.size() && pose.us <= poses_.newest().us) return;
    poses_.push(pose);
}

bool scan::PoseHistory::at(int64_t us, AxisSample& pose) const {
    if (!poses_.size() || us > poses_.newest().us) return false;
    for (size_t i = 0; i < poses_.size(); ++i) {
        const auto& older = poses_.newest(i);
        if (older.us == us) {
            pose = older;
            return true;
        }
        if (older.us > us) continue;
        const auto& newer = poses_.newest(i - 1);
        const auto gap = newer.us - older.us;
        if (gap > 12000) return false;
        constexpr float turn = 6.28318530718f;
        const float t = float(us - older.us) / gap;
        const float delta = remainderf(newer.radians - older.radians, turn);
        pose = {us, remainderf(older.radians + delta*t, turn)};
        return true;
    }
    return false;
}

void scan::project(const TimedFrame& timed, const PoseHistory& poses, CloudFrame& out) {
    const auto& frame = timed.lidar;
    out.count = 0;
    if (!frame.speedDps) return;

    const unsigned span = (36000u + frame.endCdeg - frame.startCdeg) % 36000u;
    const unsigned durationUs = span * 10000u / frame.speedDps;
    if (durationUs > 25000) return;  // more than this and the timing is already toast
    out.startUs = timed.endUs - durationUs;
    if (out.startUs < 0) return;
    for (size_t i = 0; i < ld::pointCount; ++i) {
        const auto echo = frame.points[i];
        if (!echo.mm) continue;
        const auto offset = uint16_t(durationUs * i / (ld::pointCount - 1));
        AxisSample axis;
        if (!poses.at(out.startUs + offset, axis)) continue;

        const float angle = ld::angleRad(frame, i);
        const float r = echo.mm * 0.001f;
        const Vec3 flat{r*cosf(angle), r*sinf(angle), 0};
        // shaft is in the scan plane. about Z just spins the same flat picture.
        const auto head = Quat::about({1, 0, 0}, axis.radians);
        out.points[out.count++] = {head.rotate(flat), offset, echo.intensity};
    }
}
