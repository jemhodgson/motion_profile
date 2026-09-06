#include "motion_lib/motion_profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

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

void test_stop_decelerates_to_rest_from_cruise()
{
    auto params = basicParams();
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    double p, v, a, j;
    profile.compute(2.5, p, v, a, j); // somewhere in cruise
    expect(nearly_equal(a, 0.0), "sanity check: should be in cruise (a=0) before stopping");
    const double p_at_stop = p;

    profile.stop(2.5);
    expect(profile.duration() < 5.63706, "stopping should shorten the trajectory versus completing the move");

    double prev_p = p_at_stop;
    bool monotonic = true;
    const int steps = 1000;
    for (int i = 0; i <= steps; ++i) {
        const double t = 2.5 + (profile.duration() - 2.5) * i / steps;
        profile.compute(t, p, v, a, j);
        if (p < prev_p - 1e-9)
            monotonic = false;
        prev_p = p;
    }
    expect(monotonic, "position should stay monotonic while stopping");
    expect(nearly_equal(v, 0.0, 1e-4), "velocity should reach zero after stopping");
    expect(nearly_equal(a, 0.0, 1e-4), "acceleration should reach zero after stopping");
    expect(p < params.pos_f - 1e-3, "stopping from cruise should land short of the original pos_f");
    expect(p > p_at_stop - 1e-9, "the axis should not travel backwards while stopping");
}

void test_stop_while_still_accelerating_still_reaches_rest()
{
    // Aborting while acceleration is still positive is the hard case: the
    // axis keeps speeding up briefly (jerk can't flip acceleration
    // instantly) before the deceleration actually starts removing speed.
    auto params = basicParams();
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    double p, v, a, j;
    profile.compute(0.15, p, v, a, j);
    expect(a > 0.0, "sanity check: should still be accelerating before stopping");
    const double v_at_stop = v;

    profile.stop(0.15);

    double max_v = 0.0;
    const int steps = 1000;
    for (int i = 0; i <= steps; ++i) {
        const double t = 0.15 + (profile.duration() - 0.15) * i / steps;
        profile.compute(t, p, v, a, j);
        max_v = std::max(max_v, v);
    }
    expect(max_v >= v_at_stop, "velocity may keep rising briefly right after an abort mid-acceleration");
    expect(nearly_equal(v, 0.0, 1e-4), "velocity should still reach exactly zero");
    expect(nearly_equal(a, 0.0, 1e-4), "acceleration should still reach exactly zero");
}

void test_stop_reports_stopping_then_stopped()
{
    struct Log {
        motion_lib::Phase phases[8];
        int count = 0;
    };
    auto onPhaseChange = [](motion_lib::Phase phase, void* user_data) {
        Log* log = static_cast<Log*>(user_data);
        if (log->count < 8)
            log->phases[log->count++] = phase;
    };

    auto params = basicParams();
    motion_lib::MotionProfile profile;
    profile.setParam(params);
    Log log;
    profile.setPhaseChangeCallback(onPhaseChange, &log);

    double p, v, a, j;
    profile.compute(2.5, p, v, a, j);
    profile.stop(2.5);

    const int steps = 1000;
    for (int i = 0; i <= steps; ++i) {
        const double t = 2.5 + (profile.duration() - 2.5) * i / steps;
        profile.compute(t, p, v, a, j);
    }

    expect(log.count >= 2, "should report at least Stopping and Stopped");
    if (log.count >= 2) {
        expect(log.phases[log.count - 2] == motion_lib::Phase::Stopping,
               "second-to-last phase should be Stopping");
        expect(log.phases[log.count - 1] == motion_lib::Phase::Stopped,
               "last phase should be Stopped");
    }
}

void test_stop_only_takes_effect_once()
{
    auto params = basicParams();
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    double p, v, a, j;
    profile.compute(2.5, p, v, a, j);
    profile.stop(2.5);
    const double duration_after_first_stop = profile.duration();

    profile.stop(0.1); // should be ignored: a stop was already requested
    expect(nearly_equal(profile.duration(), duration_after_first_stop),
           "a later stop() call should be ignored once one has taken effect");
}

void test_stop_before_motion_or_after_done_is_harmless()
{
    auto params = basicParams();

    motion_lib::MotionProfile early;
    early.setParam(params);
    early.stop(0.0); // before pos_i's motion even begins
    double p, v, a, j;
    early.compute(early.duration(), p, v, a, j);
    expect(nearly_equal(p, params.pos_i), "stopping before motion starts should rest at pos_i");
    expect(nearly_equal(v, 0.0), "should be at rest");

    motion_lib::MotionProfile late;
    late.setParam(params);
    late.stop(late.duration() + 100.0); // long after the move already finished
    late.compute(late.duration(), p, v, a, j);
    expect(nearly_equal(p, params.pos_f, 1e-3), "stopping after the move finished should rest at pos_f");
    expect(nearly_equal(v, 0.0), "should be at rest");
}

void test_setParam_clears_a_previous_stop()
{
    auto params = basicParams();
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    double p, v, a, j;
    profile.compute(1.0, p, v, a, j);
    profile.stop(1.0);
    expect(profile.duration() < 5.63706, "sanity check: stop should have taken effect");

    profile.setParam(params); // re-planning the same move should drop the stop
    profile.compute(profile.duration(), p, v, a, j);
    expect(nearly_equal(p, params.pos_f, 1e-3), "re-calling setParam should discard the earlier stop");
}

void test_remaining_counts_down_to_zero()
{
    auto params = basicParams();
    params.pre_delay = 0.5;
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    expect(nearly_equal(profile.remaining(0.0), profile.duration()),
           "remaining at t=0 should equal the full duration, including pre_delay");
    expect(nearly_equal(profile.remaining(0.25), profile.duration() - 0.25),
           "remaining during pre_delay should count down normally");
    expect(nearly_equal(profile.remaining(profile.duration()), 0.0),
           "remaining at duration() should be exactly zero");
    expect(nearly_equal(profile.remaining(profile.duration() + 10.0), 0.0),
           "remaining should clamp to zero past duration(), not go negative");

    // Should reflect a stop()'s shortened duration too.
    double p, v, a, j;
    profile.compute(2.5, p, v, a, j);
    profile.stop(2.5);
    expect(nearly_equal(profile.remaining(2.5), profile.duration() - 2.5),
           "remaining should reflect the shortened duration after stop()");
    expect(nearly_equal(profile.remaining(profile.duration()), 0.0),
           "remaining should still reach exactly zero once stopped");
}

void test_phase_change_callback_fires_in_order()
{
    struct Log {
        motion_lib::Phase phases[16];
        int count = 0;
    };

    auto onPhaseChange = [](motion_lib::Phase phase, void* user_data) {
        Log* log = static_cast<Log*>(user_data);
        if (log->count < 16)
            log->phases[log->count++] = phase;
    };

    auto params = basicParams();
    params.pre_delay = 0.2;
    motion_lib::MotionProfile profile;
    profile.setParam(params);

    Log log;
    profile.setPhaseChangeCallback(onPhaseChange, &log);

    const int steps = 500;
    for (int i = 0; i <= steps; ++i) {
        const double t = profile.duration() * i / steps;
        double p, v, a, j;
        profile.compute(t, p, v, a, j);
    }

    const motion_lib::Phase expected[] = {
        motion_lib::Phase::PreDelay,
        motion_lib::Phase::AccelRampUp,
        motion_lib::Phase::AccelHold,
        motion_lib::Phase::AccelRampDown,
        motion_lib::Phase::Cruise,
        motion_lib::Phase::DecelRampUp,
        motion_lib::Phase::DecelHold,
        motion_lib::Phase::DecelRampDown,
        motion_lib::Phase::Done,
    };
    const int expected_count = sizeof(expected) / sizeof(expected[0]);

    expect(log.count == expected_count, "callback should fire exactly once per phase entered");
    for (int i = 0; i < std::min(log.count, expected_count); ++i) {
        expect(log.phases[i] == expected[i], "phases should be reported in trajectory order");
    }

    // Clearing the callback should stop further notifications.
    profile.setPhaseChangeCallback(nullptr, nullptr);
    const int count_before = log.count;
    double p, v, a, j;
    profile.compute(0.0, p, v, a, j);
    expect(log.count == count_before, "clearing the callback should stop notifications");
}

void test_position_never_overshoots_target()
{
    // Position should be monotonic and never exceed the target, whether
    // cruise is reached, whether the accel/decel plateaus are reached, and
    // regardless of direction.
    struct Case { const char* label; double pos_i; double pos_f; };
    const Case cases[] = {
        {"normal", 0.0, 10.0},
        {"velocity-triangular", 0.0, 1.0},
        {"acceleration-triangular", 0.0, 0.02},
        {"reverse", 10.0, 0.0},
    };

    for (const auto& c : cases) {
        auto params = basicParams();
        params.pos_i = c.pos_i;
        params.pos_f = c.pos_f;
        motion_lib::MotionProfile profile;
        profile.setParam(params);

        const bool forward = c.pos_f >= c.pos_i;
        const int steps = 2000;
        double prev_p = c.pos_i;
        for (int i = 0; i <= steps; ++i) {
            const double t = profile.duration() * i / steps;
            double p, v, a, j;
            profile.compute(t, p, v, a, j);

            if (forward) {
                expect(p <= c.pos_f + 1e-6, "position should never exceed pos_f");
                expect(p >= prev_p - 1e-9, "position should be monotonic non-decreasing");
            }
            else {
                expect(p >= c.pos_f - 1e-6, "position should never undershoot pos_f (reverse move)");
                expect(p <= prev_p + 1e-9, "position should be monotonic non-increasing");
            }
            prev_p = p;
        }
    }
}

// Property-based fuzz test: generate a large number of random (but
// physically sane) parameter sets, including extreme/lopsided ones, and
// check invariants that must hold for *any* valid trajectory rather than
// values specific to one hand-picked case. About a third of trials also
// abort the move at a random time, to stress the stop() ramp solver.
//
// The seed is fixed so a failure is reproducible: rerun with the same
// seed and it fails on the same trial. If this ever fails, print the
// trial's parameters (already included in the failure message) and feed
// them into a standalone repro rather than trying to read them off a
// stack trace.
void test_randomized_trajectories_are_always_valid()
{
    std::mt19937 rng(20240607);
    std::uniform_real_distribution<double> pos_dist(-50.0, 50.0);
    std::uniform_real_distribution<double> vel_dist(0.01, 20.0);
    std::uniform_real_distribution<double> limit_dist(0.01, 30.0);
    std::uniform_real_distribution<double> jerk_dist(0.01, 200.0);
    std::uniform_real_distribution<double> delay_dist(0.0, 1.0);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);

    const int trials = 1000;
    int trials_with_failures = 0;
    int stop_trials = 0;
    double worst_velocity_overshoot_after_stop = 0.0;

    for (int trial = 0; trial < trials; ++trial) {
        motion_lib::MotionProfileParams params;
        params.pos_i = pos_dist(rng);
        params.pos_f = pos_dist(rng);
        params.vel_max = vel_dist(rng);
        params.acc_max = limit_dist(rng);
        params.dec_max = limit_dist(rng);
        params.jerk_acc_start = jerk_dist(rng);
        params.jerk_acc_end = jerk_dist(rng);
        params.jerk_dec_start = jerk_dist(rng);
        params.jerk_dec_end = jerk_dist(rng);
        params.pre_delay = delay_dist(rng);

        motion_lib::MotionProfile profile;
        profile.setParam(params);

        const bool forward = params.pos_f >= params.pos_i;
        const double far_bound = params.pos_f; // the trajectory must never overshoot past its own target

        const bool will_stop = unit_dist(rng) < 0.3;
        const double stop_at = will_stop ? profile.duration() * unit_dist(rng) : -1.0;
        bool has_stopped = false;
        if (will_stop)
            ++stop_trials;

        char ctx[256];
        std::snprintf(ctx, sizeof(ctx),
            "trial %d: pi=%.4f pf=%.4f vmax=%.4f amax=%.4f dmax=%.4f "
            "ja0=%.4f ja1=%.4f jd0=%.4f jd1=%.4f delay=%.4f stop_at=%.4f",
            trial, params.pos_i, params.pos_f, params.vel_max, params.acc_max, params.dec_max,
            params.jerk_acc_start, params.jerk_acc_end, params.jerk_dec_start, params.jerk_dec_end,
            params.pre_delay, stop_at);

        const int failures_before = failures;
        double prev_p = params.pos_i;
        const int samples = 250;
        // Snapshot the planned duration for the sampling schedule: stop()
        // shrinks profile.duration() mid-loop, and re-reading it per
        // iteration would make t jump backward right after the abort.
        const double planned_duration = profile.duration();

        for (int i = 0; i <= samples; ++i) {
            const double t = planned_duration * i / samples;

            if (will_stop && !has_stopped && t >= stop_at) {
                profile.stop(stop_at);
                has_stopped = true;
            }

            double p, v, a, j;
            profile.compute(t, p, v, a, j);

            expect(std::isfinite(p) && std::isfinite(v) && std::isfinite(a) && std::isfinite(j), ctx);

            if (forward) {
                expect(p >= prev_p - 1e-6, ctx); // monotonic
                if (!has_stopped)
                    expect(p <= far_bound + 1e-3, ctx); // never overshoot pos_f on the planned move
            }
            else {
                expect(p <= prev_p + 1e-6, ctx);
                if (!has_stopped)
                    expect(p >= far_bound - 1e-3, ctx);
            }
            if (!has_stopped)
                expect(std::fabs(v) <= params.vel_max + 1e-3, ctx);
            else
                worst_velocity_overshoot_after_stop = std::max(worst_velocity_overshoot_after_stop, std::fabs(v) - params.vel_max);

            prev_p = p;
        }

        double p, v, a, j;
        profile.compute(profile.duration() + 1.0, p, v, a, j); // well past the end
        expect(nearly_equal(v, 0.0, 1e-3), ctx);
        expect(nearly_equal(a, 0.0, 1e-3), ctx);
        if (!will_stop)
            expect(nearly_equal(p, params.pos_f, 1e-2), ctx);

        if (failures > failures_before)
            ++trials_with_failures;
    }

    std::printf(
        "randomized trials: %d run (%d included a stop()), %d had at least one failure; "
        "worst post-stop |v| overshoot past vel_max: %.6f\n",
        trials, stop_trials, trials_with_failures, worst_velocity_overshoot_after_stop);
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
    test_phase_change_callback_fires_in_order();
    test_stop_decelerates_to_rest_from_cruise();
    test_stop_while_still_accelerating_still_reaches_rest();
    test_stop_reports_stopping_then_stopped();
    test_stop_only_takes_effect_once();
    test_stop_before_motion_or_after_done_is_harmless();
    test_setParam_clears_a_previous_stop();
    test_remaining_counts_down_to_zero();
    test_position_never_overshoots_target();
    test_randomized_trajectories_are_always_valid();

    if (failures == 0) {
        std::printf("All tests passed.\n");
        return EXIT_SUCCESS;
    }

    std::fprintf(stderr, "%d test(s) failed.\n", failures);
    return EXIT_FAILURE;
}
