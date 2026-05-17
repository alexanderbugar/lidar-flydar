#pragma once

#include <WiFiUdp.h>
#include "fusion.h"

struct __attribute__((packed)) CloudHeader {
    uint32_t magic;
    uint32_t sequence;
    uint64_t startUs;
    uint32_t dropped;
    uint16_t count;
};

struct __attribute__((packed)) WirePoint {
    float x, y, z;
    uint16_t offsetUs;
    uint8_t intensity;
};

struct __attribute__((packed)) WireCloud {
    CloudHeader header;
    WirePoint points[ld::pointCount];
};
static_assert(sizeof(CloudHeader) == 22, "udps are little endian on the s3");
static_assert(sizeof(WirePoint) == 15, "point layout changed");
static_assert(sizeof(WireCloud) == 202, "cloud layout changed");

class NetStream {
public:
    void begin();
    void send(const scan::CloudFrame& frame, uint32_t queueDrops);


private:
    WiFiUDP udp_;
    uint32_t sequence_ = 0;
    uint32_t sendDrops_ = 0;
};
