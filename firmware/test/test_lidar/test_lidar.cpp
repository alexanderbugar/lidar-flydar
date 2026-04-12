#include <array>
#include <unity.h>
#include "ld_lidar.h"

namespace {
using Bytes = std::array<uint8_t, ld::frameBytes>;

void word(Bytes& bytes, size_t at, uint16_t value) {
    bytes[at] = uint8_t(value);
    bytes[at + 1] = uint8_t(value >> 8);
}

// Make the CRC without the firmware table so a mistaken table doesn't
// manufacture its own passing fixture.
uint8_t bitCrc(const uint8_t* bytes, size_t size) {
    uint8_t crc = 0;
    for (size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = uint8_t((crc << 1) ^ ((crc & 0x80) ? 0x4d : 0));
    }
    return crc;
}

Bytes packet() {
    Bytes bytes{0x54, 0x2c};
    word(bytes, 2, 2160);
    word(bytes, 4, 35956);
    for (size_t i = 0; i < 12; ++i) {
        word(bytes, 6 + 3*i, 780 + 7*i);
        bytes[8 + 3*i] = uint8_t(30 + i);
    }
    word(bytes, 42, 44);
    word(bytes, 44, 29999);
    bytes.back() = bitCrc(bytes.data(), bytes.size() - 1);
    return bytes;
}

unsigned feed(ld::Parser& parser, ld::Frame& out, const uint8_t* p, size_t size) {
    unsigned count = 0;
    for (size_t i = 0; i < size; ++i) count += parser.push(p[i], out);
    return count;
}

void crc_vector() {
    const uint8_t text[] = {'1','2','3','4','5','6','7','8','9'};
    TEST_ASSERT_EQUAL_UINT8(bitCrc(text, sizeof(text)), ld::crc8(text, sizeof(text)));
    TEST_ASSERT_EQUAL_UINT8(0, ld::crc8(nullptr, 0));
}

void split_reads() {
    ld::Parser parser;
    ld::Frame out{};
    const auto bytes = packet();
    const uint8_t noise[] = {0x2c, 0x54, 0x10, 0x54};
    TEST_ASSERT_EQUAL_UINT(0, feed(parser, out, noise, sizeof(noise)));
    TEST_ASSERT_EQUAL_UINT(0, feed(parser, out, bytes.data(), 1));
    TEST_ASSERT_EQUAL_UINT(0, feed(parser, out, bytes.data() + 1, 18));
    TEST_ASSERT_EQUAL_UINT(1, feed(parser, out, bytes.data() + 19, 28));
    TEST_ASSERT_EQUAL_UINT16(2160, out.speedDps);
    TEST_ASSERT_EQUAL_UINT16(29999, out.stampMs);
    TEST_ASSERT_EQUAL_UINT16(857, out.points[11].mm);
    TEST_ASSERT_EQUAL_UINT8(41, out.points[11].intensity);
}

void bad_crc_and_recovery() {
    ld::Parser parser;
    ld::Frame out{};
    const auto good = packet();
    auto bad = good;
    bad[11] ^= 4;
    TEST_ASSERT_EQUAL_UINT(0, feed(parser, out, bad.data(), bad.size()));
    TEST_ASSERT_EQUAL_UINT32(1, parser.badCrc());
    TEST_ASSERT_EQUAL_UINT(1, feed(parser, out, good.data(), good.size()));
}

void recover_one_missing_byte() {
    ld::Parser parser;
    ld::Frame out{};
    const auto good = packet();
    feed(parser, out, good.data(), 11);
    feed(parser, out, good.data() + 12, good.size() - 12);
    TEST_ASSERT_EQUAL_UINT(1, feed(parser, out, good.data(), good.size()));
    TEST_ASSERT_EQUAL_UINT32(1, parser.badCrc());
}

void impossible_fields() {
    ld::Parser parser;
    ld::Frame out{};
    auto bytes = packet();
    word(bytes, 4, 36000);
    bytes.back() = bitCrc(bytes.data(), bytes.size() - 1);
    TEST_ASSERT_EQUAL_UINT(0, feed(parser, out, bytes.data(), bytes.size()));
    TEST_ASSERT_EQUAL_UINT32(1, parser.badFields());
    TEST_ASSERT_EQUAL_UINT32(0, parser.badCrc());
}

void angle_across_zero() {
    ld::Frame out{};
    out.startCdeg = 35956;
    out.endCdeg = 44;
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, 6.282487f, ld::angleRad(out, 5));
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, 0.000698f, ld::angleRad(out, 6));
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, 0.007679f, ld::angleRad(out, 11));
}
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(crc_vector);
    RUN_TEST(split_reads);
    RUN_TEST(bad_crc_and_recovery);
    RUN_TEST(recover_one_missing_byte);
    RUN_TEST(impossible_fields);
    RUN_TEST(angle_across_zero);
    return UNITY_END();
}
