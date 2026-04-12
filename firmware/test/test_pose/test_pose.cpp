#include <unity.h>
#include "fusion.h"

namespace {
constexpr float halfTurn = 3.14159265359f;

void closeTo(Vec3 want, Vec3 got) {
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, want.x, got.x);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, want.y, got.y);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, want.z, got.z);
}

void rotation_and_inverse() {
    const auto x = Quat::about({1, 0, 0}, halfTurn/2);
    const auto z = Quat::about({0, 0, 1}, halfTurn/2);
    const auto combo = x*z;
    closeTo({0, 0, 1}, combo.rotate({1, 0, 0}));
    closeTo({1, 0, 0}, combo.conjugate().rotate({0, 0, 1}));
}

void antipode() {
    const auto q = Quat::about({1, 0, 0}, halfTurn/2);
    const auto middle = q.toward({-q.w, -q.x, -q.y, -q.z}, 0.5f);
    closeTo({0, 0, 1}, middle.rotate({0, 1, 0}));
}

void angle_and_gap() {
    scan::PoseHistory poses;
    poses.add({1000, 3.13f, {}});
    poses.add({3000, -3.13f, {}});
    poses.add({21000, 0, {}});
    scan::AxisSample at{};
    TEST_ASSERT_TRUE(poses.at(2000, at));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, halfTurn, fabsf(at.radians));
    TEST_ASSERT_FALSE(poses.at(200, at));
    TEST_ASSERT_FALSE(poses.at(12000, at));
    TEST_ASSERT_FALSE(poses.at(22000, at));
}

void lidar_clock_wrap() {
    scan::FrameClock clock;
    const auto before = clock.observe(29998, 900000);
    const auto after = clock.observe(1, 903180);
    TEST_ASSERT_INT_WITHIN(5, 3000, int(after - before));
    TEST_ASSERT_LESS_THAN_INT(903180, int(after));
}

void projection_uses_time_and_both_rotations() {
    scan::TimedFrame timed{};
    timed.endUs = 4000;
    timed.lidar.speedDps = 1000;
    timed.lidar.startCdeg = 27000;
    timed.lidar.endCdeg = 27200;
    timed.lidar.points[0] = {1000, 44};

    scan::PoseHistory poses;
    const auto base = Quat::about({0, 0, 1}, halfTurn/2);
    poses.add({1000, 0, base});
    poses.add({3000, halfTurn, base});

    scan::CloudFrame out{};
    scan::project(timed, poses, out);
    TEST_ASSERT_EQUAL_UINT(1, out.count);
    TEST_ASSERT_EQUAL_UINT(2000, unsigned(out.startUs));
    TEST_ASSERT_EQUAL_UINT16(0, out.points[0].offsetUs);
    TEST_ASSERT_EQUAL_UINT8(44, out.points[0].intensity);
    closeTo({0, 0, 1}, out.points[0].metres);
}
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(rotation_and_inverse);
    RUN_TEST(antipode);
    RUN_TEST(angle_and_gap);
    RUN_TEST(lidar_clock_wrap);
    RUN_TEST(projection_uses_time_and_both_rotations);
    return UNITY_END();
}
