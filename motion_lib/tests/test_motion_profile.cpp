#include "motion_lib/motion_profile.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

bool nearly_equal(double a, double b, double eps = 1e-6)
{
    return std::fabs(a - b) <= eps;
}

int failures = 0;

void expect(bool condition, const char* message)
{
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", message);
        ++failures;
    }
}

void test_starts_and_ends_at_rest()
{
    motion_lib::MotionProfile profile;
    profile.setParam(0.0, 10.0, 2.0, 4.0, 20.0);

    double p, v, a, j;
    profile.compute(0.0, p, v, a, j);
    expect(nearly_equal(p, 0.0), "position at t=0 should equal pos_i");
    expect(nearly_equal(v, 0.0), "velocity at t=0 should be zero");
    expect(nearly_equal(a, 0.0), "acceleration at t=0 should be zero");

    const double duration = profile.duration();
    profile.compute(duration, p, v, a, j);
    expect(nearly_equal(p, 10.0, 1e-4), "position at t=duration should equal pos_f");
    expect(nearly_equal(v, 0.0, 1e-4), "velocity at t=duration should be zero");
    expect(nearly_equal(a, 0.0, 1e-4), "acceleration at t=duration should be zero");
}

void test_clamped_before_start_and_after_end()
{
    motion_lib::MotionProfile profile;
    profile.setParam(0.0, 10.0, 2.0, 4.0, 20.0);

    double p, v, a, j;
    profile.compute(-1.0, p, v, a, j);
    expect(nearly_equal(p, 0.0), "position before t=0 should clamp to pos_i");

    profile.compute(profile.duration() + 1.0, p, v, a, j);
    expect(nearly_equal(p, 10.0), "position after duration should clamp to pos_f");
}

void test_reverse_move()
{
    motion_lib::MotionProfile profile;
    profile.setParam(10.0, 0.0, 2.0, 4.0, 20.0);

    double p, v, a, j;
    profile.compute(profile.duration(), p, v, a, j);
    expect(nearly_equal(p, 0.0, 1e-4), "reverse move should end at pos_f");
}

void test_velocity_never_exceeds_max()
{
    motion_lib::MotionProfile profile;
    const double v_max = 2.0;
    profile.setParam(0.0, 10.0, v_max, 4.0, 20.0);

    const int steps = 200;
    for (int i = 0; i <= steps; ++i) {
        const double t = profile.duration() * i / steps;
        double p, v, a, j;
        profile.compute(t, p, v, a, j);
        expect(v <= v_max + 1e-6, "velocity should never exceed vel_max");
    }
}

} // namespace

int main()
{
    test_starts_and_ends_at_rest();
    test_clamped_before_start_and_after_end();
    test_reverse_move();
    test_velocity_never_exceeds_max();

    if (failures == 0) {
        std::printf("All tests passed.\n");
        return EXIT_SUCCESS;
    }

    std::fprintf(stderr, "%d test(s) failed.\n", failures);
    return EXIT_FAILURE;
}
