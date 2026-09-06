#include "motion_lib/motion_profile.hpp"

#include <cstdio>

int main()
{
    motion_lib::MotionProfile profile;
    profile.setParam(/*pos_i=*/0.0, /*pos_f=*/10.0, /*vel_max=*/2.0, /*acc_max=*/4.0, /*jerk_max=*/20.0);

    const double duration = profile.duration();
    std::printf("trajectory duration: %.4f s\n", duration);
    std::printf("%-8s %-10s %-10s %-10s %-10s\n", "t", "p", "v", "a", "j");

    const int steps = 20;
    for (int i = 0; i <= steps; ++i) {
        const double t = duration * i / steps;
        double p, v, a, j;
        profile.compute(t, p, v, a, j);
        std::printf("%-8.3f %-10.4f %-10.4f %-10.4f %-10.4f\n", t, p, v, a, j);
    }

    return 0;
}
