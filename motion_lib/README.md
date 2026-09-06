# motion_lib

A small, dependency-free C++ library for generating jerk-limited (S-curve)
point-to-point motion trajectories. It's plain, unmanaged C++11 with no
external dependencies -- suitable for embedding directly into firmware or
other resource-constrained code.

Given a start position, end position, maximum velocity, maximum
acceleration and maximum jerk, `MotionProfile` computes position,
velocity, acceleration and jerk at any point in time along the resulting
7-phase trajectory.

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

motion_lib::MotionProfile profile;
profile.setParam(/*pos_i=*/0.0, /*pos_f=*/10.0, /*vel_max=*/2.0,
                  /*acc_max=*/4.0, /*jerk_max=*/20.0);

double p, v, a, j;
profile.compute(/*t=*/1.0, p, v, a, j);
```

## Test

```sh
cd motion_lib/build
ctest --output-on-failure
```
