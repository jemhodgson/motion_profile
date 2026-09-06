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

// The segment of the profile a given time falls in. AccelHold, Cruise and
// DecelHold are zero-duration (and so never reported) whenever the move
// never reaches the corresponding limit -- see the triangular-move cases.
// Stopping/Stopped only occur after stop() has been called; they replace
// whatever phase the planned move would otherwise have been in.
enum class Phase
{
    PreDelay,
    AccelRampUp,
    AccelHold,
    AccelRampDown,
    Cruise,
    DecelRampUp,
    DecelHold,
    DecelRampDown,
    Done,
    Stopping,
    Stopped
};

// A plain function-pointer callback (no std::function, to keep this
// dependency-free and usable from unmanaged/embedded code). user_data is
// whatever was passed to setPhaseChangeCallback(), unchanged.
using PhaseChangeCallback = void (*)(Phase phase, void* user_data);

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
    // Also resets phase-change tracking, so the next compute() call fires
    // the callback (if any) for whatever phase it lands in.
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
    // If a phase-change callback is installed and the phase at t differs
    // from the phase reported by the previous compute() call, the
    // callback fires before this call returns. Intended for compute()
    // being called with non-decreasing t, as in a typical control loop;
    // an out-of-order call still fires the callback for whatever phase
    // change it observes relative to the previous call.
    void compute(double t, double& p, double& v, double& a, double& j) const;

    // Evaluate the trajectory at time t relative to a trajectory start time t0.
    void compute(double t, double t0, double& p, double& v, double& a, double& j) const
    {
        compute(t - t0, p, v, a, j);
    }

    // Install (or clear, by passing nullptr) a callback invoked from
    // compute() on every phase transition. Not thread-safe: compute()
    // tracks the last-reported phase internally, so don't call compute()
    // on the same MotionProfile instance from multiple threads while a
    // callback is set.
    void setPhaseChangeCallback(PhaseChangeCallback callback, void* user_data = nullptr);

    // Abort the planned move: from time t onward (same time base as
    // compute()), abandon it and decelerate to a stop from wherever the
    // trajectory is at time t, using dec_max, jerk_dec_start and
    // jerk_dec_end -- the same limits the planned deceleration already
    // uses. Only the first call takes effect; later calls are ignored
    // until the next setParam(). After this, compute() for any t' >= t
    // returns points on the stop trajectory (Phase::Stopping, then
    // Phase::Stopped once at rest) instead of the original plan, and
    // duration() reflects the new, shorter total length.
    void stop(double t);

private:
    void evaluatePlannedMove(double te, Phase& phase, double& p, double& v, double& a, double& j) const;
    void evaluateStop(double t, Phase& phase, double& p, double& v, double& a, double& j) const;

    double pi_;
    double pf_;
    double dir_;
    double pre_delay_;
    double duration_;

    double acc_max_, dec_max_;
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

    PhaseChangeCallback phase_change_callback_;
    void* phase_change_user_data_;
    mutable int last_phase_index_;

    // Stop-trajectory state, set by stop(). All in the same local
    // (magnitude-space, pre-dir_/pi_) frame as the boundary state above.
    bool stop_requested_;
    double stop_time_;
    double stop_t1_, stop_t2_, stop_t3_; // cumulative, relative to stop_time_
    double stop_p0_, stop_v0_, stop_a0_; // state at the moment stop() was called
    double stop_peak_;                   // peak (most negative) acceleration reached, in [-dec_max_, 0]
    double stop_v1_, stop_p1_;           // end of ramp-to-peak
    double stop_v2_, stop_p2_;           // end of hold at peak
    double stop_pf_;                     // final rest position
};

} // namespace motion_lib

#endif // MOTION_LIB_MOTION_PROFILE_HPP
