#ifndef MOTION_LIB_MOTION_PROFILE_HPP
#define MOTION_LIB_MOTION_PROFILE_HPP

namespace motion_lib {

// Parameters for a single point-to-point move.
//
// Acceleration and deceleration are independent (acc_max need not equal
// dec_max), and each of the four jerk-limited ramps in the profile --
// ramping into the acceleration, out of it into cruise, into the
// deceleration, and out of it to a stop -- has its own jerk magnitude.
// All of vel_max, acc_max, dec_max and the jerk_* fields are magnitudes
// (positive values); direction is inferred from pos_f - pos_i.
struct MotionProfileParams
{
    double pos_i = 0.0;
    double pos_f = 0.0;
    double vel_max = 0.0;
    double acc_max = 0.0;
    double dec_max = 0.0;
    double jerk_acc_start = 0.0;
    double jerk_acc_end = 0.0;
    double jerk_dec_start = 0.0;
    double jerk_dec_end = 0.0;

    // Time to hold at pos_i, before the move begins.
    double pre_delay = 0.0;
};

// MotionProfile
//
// Generates a smooth, jerk-limited (S-curve) point-to-point trajectory.
// Plain C++, no external dependencies and no managed runtime -- suitable
// for embedding directly into firmware or other unmanaged code.
class MotionProfile
{
public:
    MotionProfile();

    // Configure the trajectory. See MotionProfileParams for field meanings.
    void setParam(const MotionProfileParams& params);

    // Total trajectory duration in seconds (including pre_delay), once
    // setParam() has been called.
    double duration() const { return duration_; }

    // Evaluate the trajectory at time t (with t = 0 at the start of
    // pre_delay, i.e. the start of the whole profile).
    //  t   Time at which to evaluate the trajectory
    //  p   Output position at time t
    //  v   Output velocity at time t
    //  a   Output acceleration at time t
    //  j   Output jerk at time t
    void compute(double t, double& p, double& v, double& a, double& j) const;

    // Evaluate the trajectory at time t relative to a trajectory start time t0.
    void compute(double t, double t0, double& p, double& v, double& a, double& j) const
    {
        compute(t - t0, p, v, a, j);
    }

private:
    double pi_;
    double pf_;
    double dir_;
    double pre_delay_;
    double duration_;

    double j_acc_start_, j_acc_end_, j_dec_start_, j_dec_end_;

    // Cumulative phase-boundary times, relative to the start of motion
    // (i.e. after pre_delay has elapsed).
    double t1_, t2_, t3_, t4_, t5_, t6_, t7_;

    // Phase-boundary state (position/velocity/acceleration magnitudes,
    // signed by dir_ and offset by pi_ at evaluation time).
    double a1_, v1_, p1_;
    double a2_, v2_, p2_;
    double v3_, p3_;
    double v4_, p4_;
    double a5_, v5_, p5_;
    double a6_, v6_, p6_;
};

} // namespace motion_lib

#endif // MOTION_LIB_MOTION_PROFILE_HPP
