#pragma once

#include <cstddef>
#include <cstdint>

namespace ld {
constexpr size_t frameBytes = 47;
constexpr size_t pointCount = 12;

struct Point {
    uint16_t mm;
    uint8_t intensity;
};

struct Frame {
    uint16_t speedDps;
    uint16_t startCdeg;
    Point points[pointCount];
    uint16_t endCdeg;
    uint16_t stampMs;
};

float angleRad(const Frame& frame, size_t point);
uint8_t crc8(const uint8_t* data, size_t size);

class Parser {
public:
    bool push(uint8_t byte, Frame& out);

    uint32_t badCrc() const { return badCrc_; }

private:
    uint32_t badCrc_ = 0;
    enum class State { Header, Length, Body };
    State state_ = State::Header;
    uint8_t bytes_[frameBytes] = {};
    size_t used_ = 0;
};
}
