#include "motion_lib/motion_profile.hpp"

#include <cmath>

namespace motion_lib {

namespace {

// Solve the timing of a jerk-in / constant-limit-plateau / jerk-out ramp
// that changes velocity by delta_v, bounded by a magnitude limit. If the
// limit can't be reached before delta_v is used up, the plateau collapses
// (t_plateau = 0) and the ramp peaks below limit.
void solveRamp(
    double delta_v,
    double limit,
    double jerk_in,
    double jerk_out,
    double& t_in,
    double& t_plateau,
    double& t_out,
    double& peak)
{
    const double inv_sum = 1.0 / jerk_in + 1.0 / jerk_out;
    const double delta_v_at_limit = 0.5 * limit * limit * inv_sum;

    if (delta_v <= delta_v_at_limit) {
        peak = std::sqrt(delta_v / (0.5 * inv_sum));
        t_in = peak / jerk_in;
        t_out = peak / jerk_out;
        t_plateau = 0.0;
    }
    else {
        peak = limit;
        t_in = limit / jerk_in;
        t_out = limit / jerk_out;
        t_plateau = (delta_v - delta_v_at_limit) / limit;
    }
}

// Distance covered accelerating from 0 to vc and decelerating from vc
// back to 0, given independent accel/decel limits and jerks. Used both
// to search for the achievable cruise velocity and, once found, to
// derive the phase boundary states.
double accelDecelDistance(
    double vc,
    double acc_max, double jerk_acc_start, double jerk_acc_end,
    double dec_max, double jerk_dec_start, double jerk_dec_end)
{
    double t1, t2, t3, peak_acc;
    solveRamp(vc, acc_max, jerk_acc_start, jerk_acc_end, t1, t2, t3, peak_acc);

    const double v1 = 0.5 * jerk_acc_start * t1 * t1;
    const double p1 = jerk_acc_start * t1 * t1 * t1 / 6.0;
    const double v2 = v1 + peak_acc * t2;
    const double p2 = p1 + v1 * t2 + 0.5 * peak_acc * t2 * t2;
    const double p3 = p2 + v2 * t3 + 0.5 * peak_acc * t3 * t3 - jerk_acc_end * t3 * t3 * t3 / 6.0;

    double t5, t6, t7, peak_dec;
    solveRamp(vc, dec_max, jerk_dec_start, jerk_dec_end, t5, t6, t7, peak_dec);

    const double v5 = vc - 0.5 * jerk_dec_start * t5 * t5;
    const double p5 = vc * t5 - jerk_dec_start * t5 * t5 * t5 / 6.0;
    const double v6 = v5 - peak_dec * t6;
    const double p6 = p5 + v5 * t6 - 0.5 * peak_dec * t6 * t6;
    const double p7 = p6 + v6 * t7 - 0.5 * peak_dec * t7 * t7 + jerk_dec_end * t7 * t7 * t7 / 6.0;

    return p3 + p7;
}

} // namespace

MotionProfile::MotionProfile()
    : pi_(0), pf_(0), dir_(1), pre_delay_(0), duration_(0),
      j_acc_start_(0), j_acc_end_(0), j_dec_start_(0), j_dec_end_(0),
      t1_(0), t2_(0), t3_(0), t4_(0), t5_(0), t6_(0), t7_(0),
      a1_(0), v1_(0), p1_(0),
      a2_(0), v2_(0), p2_(0),
      v3_(0), p3_(0),
      v4_(0), p4_(0),
      a5_(0), v5_(0), p5_(0),
      a6_(0), v6_(0), p6_(0)
{
}

void MotionProfile::setParam(const MotionProfileParams& params)
{
    pi_ = params.pos_i;
    pf_ = params.pos_f;
    dir_ = (pf_ >= pi_) ? 1.0 : -1.0;
    pre_delay_ = params.pre_delay;

    j_acc_start_ = params.jerk_acc_start;
    j_acc_end_ = params.jerk_acc_end;
    j_dec_start_ = params.jerk_dec_start;
    j_dec_end_ = params.jerk_dec_end;

    const double s = std::fabs(pf_ - pi_);

    // Find the cruise velocity actually reached: vel_max if the move is
    // long enough to accelerate up to it and decelerate back down again
    // within distance s, otherwise the largest vc (< vel_max) for which
    // accelerating to vc and immediately decelerating covers exactly s.
    double vc, t4;
    const double dist_at_vmax = accelDecelDistance(
        params.vel_max,
        params.acc_max, j_acc_start_, j_acc_end_,
        params.dec_max, j_dec_start_, j_dec_end_);

    if (dist_at_vmax <= s) {
        vc = params.vel_max;
        t4 = (s - dist_at_vmax) / params.vel_max;
    }
    else {
        double lo = 0.0, hi = params.vel_max;
        for (int i = 0; i < 100; ++i) {
            const double mid = 0.5 * (lo + hi);
            const double d = accelDecelDistance(
                mid,
                params.acc_max, j_acc_start_, j_acc_end_,
                params.dec_max, j_dec_start_, j_dec_end_);
            if (d < s)
                lo = mid;
            else
                hi = mid;
        }
        vc = 0.5 * (lo + hi);
        t4 = 0.0;
    }

    // Acceleration phases: ramp up to peak accel (or below, if it's never
    // reached), hold it, then ramp down to zero as velocity reaches vc.
    double t1, t2, t3, peak_acc;
    solveRamp(vc, params.acc_max, j_acc_start_, j_acc_end_, t1, t2, t3, peak_acc);

    a1_ = peak_acc;
    v1_ = 0.5 * j_acc_start_ * t1 * t1;
    p1_ = j_acc_start_ * t1 * t1 * t1 / 6.0;

    a2_ = peak_acc;
    v2_ = v1_ + a2_ * t2;
    p2_ = p1_ + v1_ * t2 + 0.5 * a2_ * t2 * t2;

    v3_ = v2_ + a2_ * t3 - 0.5 * j_acc_end_ * t3 * t3;
    p3_ = p2_ + v2_ * t3 + 0.5 * a2_ * t3 * t3 - j_acc_end_ * t3 * t3 * t3 / 6.0;

    v4_ = v3_;
    p4_ = p3_ + v3_ * t4;

    // Deceleration phases: ramp into peak decel (or below), hold it, then
    // ramp back to zero as velocity reaches 0.
    double t5, t6, t7, peak_dec;
    solveRamp(vc, params.dec_max, j_dec_start_, j_dec_end_, t5, t6, t7, peak_dec);

    a5_ = -peak_dec;
    v5_ = v4_ - 0.5 * j_dec_start_ * t5 * t5;
    p5_ = p4_ + v4_ * t5 - j_dec_start_ * t5 * t5 * t5 / 6.0;

    a6_ = a5_;
    v6_ = v5_ + a5_ * t6;
    p6_ = p5_ + v5_ * t6 + 0.5 * a5_ * t6 * t6;

    t1_ = t1;
    t2_ = t1_ + t2;
    t3_ = t2_ + t3;
    t4_ = t3_ + t4;
    t5_ = t4_ + t5;
    t6_ = t5_ + t6;
    t7_ = t6_ + t7;

    duration_ = pre_delay_ + t7_;
}

void MotionProfile::compute(double t, double& p, double& v, double& a, double& j) const
{
    const double te = t - pre_delay_;
    double dt;
    double pp, vv, aa, jj;

    if (te <= 0) {
        jj = 0;
        aa = 0;
        vv = 0;
        pp = 0;
    }
    else if (te < t1_) {
        jj = j_acc_start_;
        aa = j_acc_start_ * te;
        vv = 0.5 * aa * te;
        pp = j_acc_start_ * te * te * te / 6.0;
    }
    else if (te < t2_) {
        dt = te - t1_;
        jj = 0;
        aa = a1_;
        vv = v1_ + a1_ * dt;
        pp = p1_ + (v1_ + a1_ / 2.0 * dt) * dt;
    }
    else if (te < t3_) {
        dt = te - t2_;
        jj = -j_acc_end_;
        aa = a2_ - j_acc_end_ * dt;
        vv = v2_ + (a2_ - j_acc_end_ / 2.0 * dt) * dt;
        pp = p2_ + (v2_ + (a2_ - j_acc_end_ / 3.0 * dt) / 2.0 * dt) * dt;
    }
    else if (te < t4_) {
        jj = 0;
        aa = 0;
        vv = v3_;
        pp = p3_ + v3_ * (te - t3_);
    }
    else if (te < t5_) {
        dt = te - t4_;
        jj = -j_dec_start_;
        aa = -j_dec_start_ * dt;
        vv = v4_ - j_dec_start_ / 2.0 * dt * dt;
        pp = p4_ + (v4_ - j_dec_start_ / 6.0 * dt * dt) * dt;
    }
    else if (te < t6_) {
        dt = te - t5_;
        jj = 0;
        aa = a5_;
        vv = v5_ + a5_ * dt;
        pp = p5_ + (v5_ + a5_ / 2.0 * dt) * dt;
    }
    else if (te < t7_) {
        dt = te - t6_;
        jj = j_dec_end_;
        aa = a6_ + j_dec_end_ * dt;
        vv = v6_ + (a6_ + j_dec_end_ / 2.0 * dt) * dt;
        pp = p6_ + (v6_ + (a6_ + j_dec_end_ / 3.0 * dt) / 2.0 * dt) * dt;
    }
    else {
        jj = 0;
        aa = 0;
        vv = 0;
        pp = std::fabs(pf_ - pi_);
    }

    j = dir_ * jj;
    a = dir_ * aa;
    v = dir_ * vv;
    p = pi_ + dir_ * pp;
}

} // namespace motion_lib
