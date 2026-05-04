# Planar 3-Link Robot

A small C++ project demonstrating forward and inverse kinematics for a planar 3-link revolute arm, built on [Eigen](https://eigen.tuxfamily.org).

[![CI](https://github.com/behnamasadi/planar_3_link_robot/actions/workflows/ci.yml/badge.svg)](https://github.com/behnamasadi/planar_3_link_robot/actions/workflows/ci.yml)
![License](https://img.shields.io/badge/license-BSD-blue.svg)

## Robot model

```text
ground ──┤J₀├── link 1 ──┤J₁├── link 2 ──┤J₂├── link 3 ──◆ end-effector
   ▲     q₀              q₁              q₂
fixed
```

- **3 revolute joints** with angles $q = (q_0, q_1, q_2)^\top$
- **3 unit-length links** (rigid bodies between consecutive joints)
- **Planar** (motion in the $xy$-plane only)
- **Base fixed** at the world origin $(0, 0)$; joint $J_0$ is grounded but rotates link 1 around the base
- End-effector pose is $(x, y, \theta) \in \mathbb{R}^3$ — the tip of link 3 plus the cumulative orientation $\theta = q_0 + q_1 + q_2$

Configuration space is $\mathbb{R}^3$, task space (full pose) is $\mathbb{R}^3$ — a square IK problem with no kinematic redundancy. (For position-only tasks, dropping $\theta$ leaves 1 DOF of redundancy; see the notebook §6.)

## What's implemented

| Concept | File | Function |
|---|---|---|
| Forward kinematics $T(q)$ | [`src/task.cpp`](src/task.cpp) | `forward_kinematics` |
| Pose extraction $(x, y, \theta)$ | [`src/task.cpp`](src/task.cpp) | `transformationMatrixToPose` |
| Numerical Jacobian | [`src/task.cpp`](src/task.cpp) | `numericalDifferentiationFK` |
| Moore-Penrose pseudo-inverse via SVD | [`src/task.cpp`](src/task.cpp) | `pseudoInverse` |
| Damped Newton inverse kinematics | [`src/task.cpp`](src/task.cpp) | `inverse_kinematics` |
| Driver / example | [`src/main.cpp`](src/main.cpp) | `main` |
| Unit tests | [`tests/src/kinematics_tests.cpp`](tests/src/kinematics_tests.cpp) | GTest |

The IK uses a damped Newton iteration with a magnitude-clamped pose error step: $q_{k+1} = q_k + \alpha\,J^{+}(q_k)\,\tilde e_k$, where $\tilde e_k = \mathrm{clamp}(p^* - f(q_k))$.

## Theory and walkthrough

A Jupyter notebook walks through every equation and maps it to the C++ code, with figures:

- [`docs/kinematics.ipynb`](docs/kinematics.ipynb)

It covers FK derivation, the analytical vs. numerical Jacobian, SVD-based pseudo-inverse, the damped Newton IK loop, and a section on null-space and redundancy resolution (with the self-motion manifold visualised).

## Dependencies

- CMake ≥ 3.21
- A C++17 compiler
- [Eigen3](https://eigen.tuxfamily.org) (with the `unsupported/Eigen/NumericalDiff` module)
- [GoogleTest](https://github.com/google/googletest) (for the unit tests)
- Ninja (recommended; the CMake preset uses Ninja Multi-Config)

On Ubuntu / Debian: `sudo apt install cmake ninja-build libeigen3-dev libgtest-dev`.

## Build

The project uses [CMake presets](CMakePresets.json) with a Ninja Multi-Config generator (one build tree, four configurations).

```bash
# Configure (once)
cmake --preset ninja-multi

# Build
cmake --build --preset ninja-multi-debug
# or:  ninja-multi-release  /  ninja-multi-relwithdebinfo  /  ninja-multi-minsizerel
```

## Run

```bash
./build/Debug/main
```

## Tests

```bash
ctest --test-dir build -C Debug --output-on-failure
```

## Project layout

```text
.
├── CMakeLists.txt           # top-level build
├── CMakePresets.json        # Ninja Multi-Config presets
├── src/
│   ├── main.cpp             # example driver
│   ├── task.hpp             # type aliases + declarations
│   └── task.cpp             # FK, Jacobian, pseudo-inverse, IK
├── tests/
│   ├── CMakeLists.txt
│   └── src/kinematics_tests.cpp
├── docs/
│   └── kinematics.ipynb     # math + code walkthrough with figures
├── .github/workflows/       # GitHub Actions CI
└── .vscode/                 # editor format-on-save settings
```

## Continuous integration

GitHub Actions builds and tests Debug + Release on every push and pull request. See [`.github/workflows/ci.yml`](.github/workflows/ci.yml).

## License

BSD — see [LICENSE](LICENSE).
