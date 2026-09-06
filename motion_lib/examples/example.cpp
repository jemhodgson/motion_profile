#include "motion_lib/motion_profile.hpp"

#include <cstdio>

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

    return 0;
}
