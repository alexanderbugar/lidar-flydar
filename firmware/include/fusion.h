#pragma once

#include "ld_lidar.h"

namespace scan {
struct TimedFrame {
    ld::Frame lidar;
    int64_t endUs;
};

class FrameClock {
public:
    int64_t observe(uint16_t stampMs, int64_t receivedUs);

private:
    int64_t lastRxUs_ = 0;
    int64_t offsetUs_ = 0;
    int64_t sensorUs_ = 0;
    uint16_t lastStamp_ = 0;
};
}
