#ifndef MOTION_LIB_MOTION_PROFILE_HPP
#define MOTION_LIB_MOTION_PROFILE_HPP

namespace motion_lib {

// MotionProfile
//
// Generates a smooth, jerk-limited (S-curve) point-to-point trajectory
// given a maximum velocity, acceleration and jerk. Plain C++, no
// external dependencies and no managed runtime -- suitable for
// embedding directly into firmware or other unmanaged code.
class MotionProfile
{
public:
    MotionProfile();

    // Configure the trajectory.
    //  pos_i     Initial position
    //  pos_f     Final position
    //  vel_max   Max/cruise velocity
    //  acc_max   Max acceleration
    //  jerk_max  Max jerk
    void setParam(
        double pos_i,
        double pos_f,
        double vel_max,
        double acc_max,
        double jerk_max);

    // Total trajectory duration, in seconds, once setParam() has been called.
    double duration() const { return t7_; }

    // Evaluate the trajectory at time t (with t = 0 at the start of the move).
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
    double v_max_;
    double a_max_;
    double j_max_;

    double t1_, t2_, t3_, t4_, t5_, t6_, t7_;
    double a1_, v1_, p1_;
    double a2_, v2_, p2_;
    double v3_, p3_;
    double v4_, p4_;
    double a5_, v5_, p5_;
    double a6_, v6_, p6_;
};

} // namespace motion_lib

#endif // MOTION_LIB_MOTION_PROFILE_HPP
