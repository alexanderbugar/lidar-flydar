#pragma once

#include "ld_lidar.h"
#include "math3d.h"
#include "ring_buffer.h"

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

struct AxisSample {
    int64_t us;
    float radians;
};

class PoseHistory {
public:
    void add(AxisSample pose);
    bool at(int64_t us, AxisSample& pose) const;

private:
    RingBuffer<AxisSample, 128> poses_;
};

struct Point {
    Vec3 metres;
    uint16_t offsetUs;
    uint8_t intensity;
};

struct CloudFrame {
    int64_t startUs;
    size_t count;
    Point points[ld::pointCount];
};

void project(const TimedFrame& frame, const PoseHistory& poses, CloudFrame& out);
}
