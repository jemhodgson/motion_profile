#include "motion_lib/motion_profile.hpp"

#include <cmath>

namespace motion_lib {

MotionProfile::MotionProfile()
    : pi_(0), pf_(0), v_max_(0), a_max_(0), j_max_(0),
      t1_(0), t2_(0), t3_(0), t4_(0), t5_(0), t6_(0), t7_(0),
      a1_(0), v1_(0), p1_(0),
      a2_(0), v2_(0), p2_(0),
      v3_(0), p3_(0),
      v4_(0), p4_(0),
      a5_(0), v5_(0), p5_(0),
      a6_(0), v6_(0), p6_(0)
{
}

void MotionProfile::setParam(double pos_i, double pos_f, double vel_max, double acc_max, double jerk_max)
{
    pi_ = pos_i;
    pf_ = pos_f;
    v_max_ = vel_max;
    a_max_ = acc_max;
    j_max_ = jerk_max;

    if (pi_ > pf_) {
        v_max_ = -v_max_;
        a_max_ = -a_max_;
        j_max_ = -j_max_;
    }

    const double s = pf_ - pi_;
    const double a_max2 = a_max_ * a_max_;
    const double va = std::fabs(a_max2 / j_max_);
    const double sa = std::fabs(2.0 * va * a_max_ / j_max_);

    const double a_max_div_j_max = std::fabs(a_max_ / j_max_);
    const double sqrt_v_max_div_j_max = std::sqrt(v_max_ / j_max_);

    double sv;
    if (v_max_ * j_max_ < a_max2)
        sv = std::fabs(v_max_ * 2.0 * sqrt_v_max_div_j_max);
    else
        sv = std::fabs(v_max_ * (v_max_ / a_max_ + a_max_div_j_max));

    double tj, ta, tv;
    if (v_max_ <= va && s >= sa) {
        tj = sqrt_v_max_div_j_max;
        ta = tj;
        tv = s / v_max_;
    }
    else if (v_max_ >= va && s <= sa) {
        tj = std::cbrt(s / (2.0 * j_max_));
        ta = tj;
        tv = 2.0 * tj;
    }
    else if (v_max_ <= va && s <= sa) {
        if (s >= sv) {
            tj = sqrt_v_max_div_j_max;
            ta = tj;
            tv = s / v_max_;
        }
        else {
            tj = a_max_div_j_max;
            ta = v_max_ / a_max_;
            tv = s / v_max_;
        }
    }
    else { // v_max_ >= va && s >= sa
        tj = a_max_div_j_max;
        if (s >= sv) {
            ta = v_max_ / a_max_;
            tv = s / v_max_;
        }
        else {
            ta = 0.5 * (std::sqrt((4.0 * s + a_max_ * a_max_div_j_max * a_max_div_j_max) / a_max_) - a_max_div_j_max);
            tv = ta + tj;
        }
    }

    // Time intervals for the 7 phases of the jerk-limited profile.
    t1_ = tj;
    t2_ = ta;
    t3_ = ta + tj;
    t4_ = tv;
    t5_ = tv + tj;
    t6_ = tv + ta;
    t7_ = tv + tj + ta;

    a1_ = j_max_ * t1_;
    v1_ = a1_ * t1_ / 2.0;
    p1_ = pi_ + v1_ * t1_ / 3.0;

    double dt = t2_ - t1_;
    a2_ = a1_;
    v2_ = v1_ + a1_ * dt;
    p2_ = p1_ + (v1_ + a1_ / 2.0 * dt) * dt;

    dt = t3_ - t2_;
    v3_ = v2_ + (a2_ - j_max_ / 2.0 * dt) * dt;
    p3_ = p2_ + (v2_ + (a2_ - j_max_ / 3.0 * dt) / 2.0 * dt) * dt;

    v4_ = v3_;
    p4_ = p3_ + v3_ * (t4_ - t3_);

    dt = t5_ - t4_;
    a5_ = -j_max_ * dt;
    v5_ = v4_ - j_max_ / 2.0 * dt * dt;
    p5_ = p4_ + (v4_ - j_max_ / 6.0 * dt * dt) * dt;

    dt = t6_ - t5_;
    a6_ = a5_;
    v6_ = v5_ - a_max_ * dt;
    p6_ = p5_ + (v5_ + a5_ / 2.0 * dt) * dt;
}

void MotionProfile::compute(double t, double& p, double& v, double& a, double& j) const
{
    double dt;
    if (t <= 0) {
        j = 0;
        a = 0;
        v = 0;
        p = pi_;
    }
    else if (t < t1_) {
        j = j_max_;
        a = j_max_ * t;
        v = a / 2.0 * t;
        p = pi_ + v * t / 3.0;
    }
    else if (t < t2_) {
        dt = t - t1_;
        j = 0;
        a = a1_;
        v = v1_ + a1_ * dt;
        p = p1_ + (v1_ + a1_ / 2.0 * dt) * dt;
    }
    else if (t < t3_) {
        dt = t - t2_;
        j = -j_max_;
        a = a2_ - j_max_ * dt;
        v = v2_ + (a2_ - j_max_ / 2.0 * dt) * dt;
        p = p2_ + (v2_ + (a2_ - j_max_ / 3.0 * dt) / 2.0 * dt) * dt;
    }
    else if (t < t4_) {
        j = 0;
        a = 0;
        v = v3_;
        p = p3_ + v3_ * (t - t3_);
    }
    else if (t < t5_) {
        dt = t - t4_;
        j = -j_max_;
        a = -j_max_ * dt;
        v = v4_ - j_max_ / 2.0 * dt * dt;
        p = p4_ + (v4_ - j_max_ / 6.0 * dt * dt) * dt;
    }
    else if (t < t6_) {
        dt = t - t5_;
        j = 0;
        a = a5_;
        v = v5_ - a_max_ * dt;
        p = p5_ + (v5_ + a5_ / 2.0 * dt) * dt;
    }
    else if (t < t7_) {
        dt = t - t6_;
        j = j_max_;
        a = a6_ + j_max_ * dt;
        v = v6_ + (a6_ + j_max_ / 2.0 * dt) * dt;
        p = p6_ + (v6_ + (a6_ + j_max_ / 3.0 * dt) / 2.0 * dt) * dt;
    }
    else {
        j = 0;
        a = 0;
        v = 0;
        p = pf_;
    }
}

} // namespace motion_lib
