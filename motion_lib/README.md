# motion_lib

A small, dependency-free C++ library for generating jerk-limited (S-curve)
point-to-point motion trajectories. It's plain, unmanaged C++11 with no
external dependencies -- suitable for embedding directly into firmware or
other resource-constrained code.

Acceleration and deceleration are independent (they don't need to match),
and each of the four jerk-limited ramps in the profile -- into
acceleration, out of it into cruise, into deceleration, and out of it to a
stop -- has its own jerk value. An optional pre-move delay holds the
trajectory at its start position before motion begins.

Given `MotionProfileParams`, `MotionProfile` computes position, velocity,
acceleration and jerk at any point in time along the resulting trajectory.

## Build

```sh
cd motion_lib
mkdir build && cd build
cmake ..
make
```

This builds:
- `libmotion_lib.a` -- the static library
- `motion_lib_example` -- a small demo that prints a sampled trajectory
- `motion_lib_tests` -- the test suite (run with `ctest` or directly)

## Usage

```cpp
#include "motion_lib/motion_profile.hpp"

motion_lib::MotionProfileParams params;
params.pos_i = 0.0;
params.pos_f = 10.0;
params.vel_max = 2.0;
params.acc_max = 4.0;           // acceleration limit
params.dec_max = 6.0;           // deceleration limit (can differ from acc_max)
params.jerk_acc_start = 20.0;   // jerk ramping into acceleration
params.jerk_acc_end = 15.0;     // jerk ramping out of acceleration, into cruise
params.jerk_dec_start = 30.0;   // jerk ramping into deceleration
params.jerk_dec_end = 25.0;     // jerk ramping out of deceleration, to a stop
params.pre_delay = 0.5;         // hold at pos_i for 0.5s before moving

motion_lib::MotionProfile profile;
profile.setParam(params);

double p, v, a, j;
profile.compute(/*t=*/1.0, p, v, a, j);
```

`profile.duration()` returns the total trajectory length, including
`pre_delay`. If the move is too short to reach `vel_max`, the achievable
cruise velocity is found automatically and the trajectory still lands
exactly on `pos_f`.

## Test

```sh
cd motion_lib/build
ctest --output-on-failure
```
