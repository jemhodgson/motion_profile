#include "motion_lib/motion_profile.hpp"

#include <algorithm>
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

motion_lib::MotionProfileParams basicParams()
{
    motion_lib::MotionProfileParams params;
    params.pos_i = 0.0;
    params.pos_f = 10.0;
    params.vel_max = 2.0;
    params.acc_max = 4.0;
    params.dec_max = 4.0;
    params.jerk_acc_start = 20.0;
    params.jerk_acc_end = 20.0;
    params.jerk_dec_start = 20.0;
    params.jerk_dec_end = 20.0;
    return params;
}

void test_starts_and_ends_at_rest()
{
    auto params = basicParams();
    motion_lib::MotionProfile profile;
    profile.setParam(params);

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
    auto params = basicParams();
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    double p, v, a, j;
    profile.compute(-1.0, p, v, a, j);
    expect(nearly_equal(p, 0.0), "position before t=0 should clamp to pos_i");

    profile.compute(profile.duration() + 1.0, p, v, a, j);
    expect(nearly_equal(p, 10.0), "position after duration should clamp to pos_f");
}

void test_reverse_move()
{
    auto params = basicParams();
    params.pos_i = 10.0;
    params.pos_f = 0.0;
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    double p, v, a, j;
    profile.compute(profile.duration(), p, v, a, j);
    expect(nearly_equal(p, 0.0, 1e-4), "reverse move should end at pos_f");
}

void test_velocity_never_exceeds_max()
{
    auto params = basicParams();
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    const int steps = 200;
    for (int i = 0; i <= steps; ++i) {
        const double t = profile.duration() * i / steps;
        double p, v, a, j;
        profile.compute(t, p, v, a, j);
        expect(v <= params.vel_max + 1e-6, "velocity should never exceed vel_max");
    }
}

void test_short_move_never_reaches_vel_max()
{
    auto params = basicParams();
    params.pos_f = 0.05; // too short to reach cruise speed
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    double peak_v = 0.0;
    const int steps = 500;
    for (int i = 0; i <= steps; ++i) {
        const double t = profile.duration() * i / steps;
        double p, v, a, j;
        profile.compute(t, p, v, a, j);
        peak_v = std::max(peak_v, v);
    }
    expect(peak_v < params.vel_max - 1e-3, "short move should not reach vel_max");

    double p, v, a, j;
    profile.compute(profile.duration(), p, v, a, j);
    expect(nearly_equal(p, params.pos_f, 1e-4), "short move should still land exactly on pos_f");
}

void test_asymmetric_acceleration_and_deceleration()
{
    // A much harder deceleration than acceleration should take noticeably
    // less time to stop than to get going, for an otherwise identical move.
    auto params = basicParams();
    params.acc_max = 2.0;
    params.dec_max = 8.0;
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    double p, v, a, j;
    // Time to reach cruise velocity vs. time to decelerate from it should differ.
    profile.compute(0.001, p, v, a, j);

    // Find when cruise (max) velocity is first reached and when it's left.
    const int steps = 2000;
    double t_reach_cruise = -1.0, t_leave_cruise = -1.0;
    double max_v = 0.0;
    for (int i = 0; i <= steps; ++i) {
        const double t = profile.duration() * i / steps;
        profile.compute(t, p, v, a, j);
        max_v = std::max(max_v, v);
    }
    for (int i = 0; i <= steps; ++i) {
        const double t = profile.duration() * i / steps;
        profile.compute(t, p, v, a, j);
        if (t_reach_cruise < 0.0 && v >= max_v - 1e-3)
            t_reach_cruise = t;
        if (v >= max_v - 1e-3)
            t_leave_cruise = t;
    }
    const double accel_time = t_reach_cruise;
    const double decel_time = profile.duration() - t_leave_cruise;
    expect(decel_time < accel_time, "harder deceleration should take less time than acceleration");
}

void test_distinct_jerk_values_change_the_profile()
{
    auto slow_start = basicParams();
    slow_start.jerk_acc_start = 5.0; // much gentler ramp-in than everything else

    auto uniform = basicParams();

    motion_lib::MotionProfile profile_slow_start;
    profile_slow_start.setParam(slow_start);
    motion_lib::MotionProfile profile_uniform;
    profile_uniform.setParam(uniform);

    expect(profile_slow_start.duration() > profile_uniform.duration(),
           "a gentler acceleration-start jerk should lengthen the move");
}

void test_pre_delay_holds_position_then_moves()
{
    auto params = basicParams();
    params.pre_delay = 1.0;
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    double p, v, a, j;
    profile.compute(0.5, p, v, a, j);
    expect(nearly_equal(p, params.pos_i), "position during pre_delay should stay at pos_i");
    expect(nearly_equal(v, 0.0), "velocity during pre_delay should be zero");

    profile.compute(params.pre_delay + 0.001, p, v, a, j);
    expect(v > 0.0, "motion should begin once pre_delay has elapsed");

    auto no_delay = basicParams();
    motion_lib::MotionProfile profile_no_delay;
    profile_no_delay.setParam(no_delay);
    expect(nearly_equal(profile.duration(), profile_no_delay.duration() + params.pre_delay, 1e-9),
           "total duration should equal pre_delay plus the move duration");
}

} // namespace

int main()
{
    test_starts_and_ends_at_rest();
    test_clamped_before_start_and_after_end();
    test_reverse_move();
    test_velocity_never_exceeds_max();
    test_short_move_never_reaches_vel_max();
    test_asymmetric_acceleration_and_deceleration();
    test_distinct_jerk_values_change_the_profile();
    test_pre_delay_holds_position_then_moves();

    if (failures == 0) {
        std::printf("All tests passed.\n");
        return EXIT_SUCCESS;
    }

    std::fprintf(stderr, "%d test(s) failed.\n", failures);
    return EXIT_FAILURE;
}
