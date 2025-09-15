#pragma once

#include <WiFiUdp.h>
#include "ld_lidar.h"

struct __attribute__((packed)) PolarPoint {
    uint16_t angleCdeg;
    uint16_t mm;
    uint8_t intensity;
};

struct __attribute__((packed)) PolarPacket {
    uint32_t sequence;
    uint16_t stampMs;
    PolarPoint points[ld::pointCount];
};
static_assert(sizeof(PolarPacket) == 66, "someone moved the wire layout");

class NetStream {
public:
    void begin();
    void send(const ld::Frame& frame);
    bool ready() const;

private:
    WiFiUDP udp_;
    uint32_t sequence_ = 0;
};
