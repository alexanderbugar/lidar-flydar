#include "ld_lidar.h"

namespace ld {
namespace {
constexpr uint8_t header = 0x54;
constexpr uint8_t verLen = 0x2c;

uint16_t word(const uint8_t* p) {
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
}

bool Parser::push(uint8_t byte, Frame& out) {
    switch (state_) {
    case State::Header:
        if (byte == header) {
            bytes_[0] = byte;
            state_ = State::Length;
        }
        return false;
    case State::Length:
        if (byte == verLen) {
            bytes_[1] = byte;
            used_ = 2;
            state_ = State::Body;
        } else if (byte != header) {
            state_ = State::Header;
        }
        return false;
    case State::Body:
        bytes_[used_++] = byte;
        if (used_ < frameBytes) return false;
        state_ = State::Header;
        break;
    }

    out.speedDps = word(bytes_ + 2);
    out.startCdeg = word(bytes_ + 4);
    for (size_t i = 0; i < pointCount; ++i) {
        const uint8_t* p = bytes_ + 6 + 3 * i;
        out.points[i] = {word(p), p[2]};
    }
    out.endCdeg = word(bytes_ + 42);
    out.stampMs = word(bytes_ + 44);
    return true;
}
}
