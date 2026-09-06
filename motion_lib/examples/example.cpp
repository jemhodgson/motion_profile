#include "motion_lib/motion_profile.hpp"

#include <cstdio>

namespace {

const char* phaseName(motion_lib::Phase phase)
{
    switch (phase) {
        case motion_lib::Phase::PreDelay: return "PreDelay";
        case motion_lib::Phase::AccelRampUp: return "AccelRampUp";
        case motion_lib::Phase::AccelHold: return "AccelHold";
        case motion_lib::Phase::AccelRampDown: return "AccelRampDown";
        case motion_lib::Phase::Cruise: return "Cruise";
        case motion_lib::Phase::DecelRampUp: return "DecelRampUp";
        case motion_lib::Phase::DecelHold: return "DecelHold";
        case motion_lib::Phase::DecelRampDown: return "DecelRampDown";
        case motion_lib::Phase::Done: return "Done";
        case motion_lib::Phase::Stopping: return "Stopping";
        case motion_lib::Phase::Stopped: return "Stopped";
    }
    return "?";
}

void onPhaseChange(motion_lib::Phase phase, void* /*user_data*/)
{
    std::printf(">> entering phase: %s\n", phaseName(phase));
}

} // namespace

int main()
{
    motion_lib::MotionProfileParams params;
    params.pos_i = 0.0;
    params.pos_f = 10.0;
    params.vel_max = 2.0;
    params.acc_max = 4.0;
    params.dec_max = 6.0;          // decelerates harder than it accelerates
    params.jerk_acc_start = 20.0;
    params.jerk_acc_end = 15.0;
    params.jerk_dec_start = 30.0;
    params.jerk_dec_end = 25.0;
    params.pre_delay = 0.5;        // half a second hold before the move starts

    motion_lib::MotionProfile profile;
    profile.setParam(params);
    profile.setPhaseChangeCallback(onPhaseChange);

    const double duration = profile.duration();
    std::printf("trajectory duration (incl. pre-delay): %.4f s\n", duration);
    std::printf("%-8s %-10s %-10s %-10s %-10s\n", "t", "p", "v", "a", "j");

    const int steps = 24;
    for (int i = 0; i <= steps; ++i) {
        const double t = duration * i / steps;
        double p, v, a, j;
        profile.compute(t, p, v, a, j);
        std::printf("%-8.3f %-10.4f %-10.4f %-10.4f %-10.4f\n", t, p, v, a, j);
    }

    // Now show aborting a move partway through: ask for the same move
    // again, but pull the plug 1.2s in and let it decelerate to a stop
    // from wherever it happens to be, instead of reaching pos_f.
    std::printf("\n--- aborting the same move at t=1.2s ---\n");
    motion_lib::MotionProfile aborted;
    aborted.setParam(params);
    aborted.setPhaseChangeCallback(onPhaseChange);

    double p, v, a, j;
    aborted.compute(1.2, p, v, a, j);
    std::printf("state just before abort: p=%.4f v=%.4f a=%.4f\n", p, v, a);

    aborted.stop(1.2);
    std::printf("%-8s %-10s %-10s %-10s %-10s\n", "t", "p", "v", "a", "j");
    for (int i = 0; i <= steps; ++i) {
        const double t = 1.2 + (aborted.duration() - 1.2) * i / steps;
        aborted.compute(t, p, v, a, j);
        std::printf("%-8.3f %-10.4f %-10.4f %-10.4f %-10.4f\n", t, p, v, a, j);
    }

    return 0;
}
