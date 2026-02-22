# softy

Spring-damper systems and Verlet integration for games -- ropes, cloth, and soft bodies.

[![no_std](https://img.shields.io/badge/no__std-compatible-green.svg)](https://doc.rust-lang.org/reference/names/preludes.html#the-no_std-attribute)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](#license)

**[Live Demo](https://robdavenport.github.io/softy/)** -- try it in your browser (compiled to WebAssembly)

## Overview

- **Analytical springs** -- closed-form critically/under/over-damped spring-damper systems (no Euler drift)
- **Verlet dynamics** -- position-based integration with implicit velocity
- **Constraint solver** -- distance, pin, angle, and bounds constraints with iterative relaxation
- **Ropes & cloth** -- `VerletChain` and `VerletGrid` with tear support
- **Soft bodies** -- 2D pressure-based deformable shapes
- **Generic** -- works with any float/vector type via `Float` and `Vec` traits
- **Deterministic** -- reproducible results across runs
- **`no_std` compatible** -- works in embedded and WASM environments

## Quick Start

```rust
use softy::Spring1D;

// Critically damped spring: smoothly moves from 0.0 toward 10.0
let mut spring = Spring1D::<f32>::critically_damped(0.0, 10.0, 5.0);

// Advance by 1/60th of a second
spring.update(1.0 / 60.0);
println!("value: {}", spring.value()); // smoothly approaches 10.0
```

## Features

- **Spring types** -- critically damped, underdamped (oscillating), and overdamped; 1D/2D/3D type aliases
- **Verlet particles** -- position-based dynamics with gravity, damping, and mass
- **Constraint solver** -- configurable iteration count and relaxation
  - `DistanceConstraint` -- maintain distance between particles
  - `PinConstraint` -- pin a particle to a fixed point
  - `AngleConstraint` -- limit angle between particle triplets
  - `BoundsConstraint` -- keep particles within a region
- **VerletChain** -- rope simulation with configurable segment count and stiffness
- **VerletGrid** -- cloth simulation with structural, shear, and bend links; supports tearing
- **SoftBody** -- 2D pressure-based soft body with internal volume preservation
- **StepObserver** -- trait for monitoring physics steps (debugging, profiling)
- **`no_std`** -- depends only on `alloc`; uses `libm` for math

## Feature Flags

| Flag  | Default | Description                  |
|-------|---------|------------------------------|
| `std` | off     | Enables standard library use |

## API Overview

| Type                 | Role                                          |
|----------------------|-----------------------------------------------|
| `Spring<V>`          | Analytical spring-damper (generic over vector) |
| `Spring1D/2D/3D`    | Dimensional type aliases for `Spring`          |
| `DampingMode`        | Critical / underdamped / overdamped selection  |
| `Particle`           | Verlet particle with position, mass, pinning   |
| `ConstraintSolver`   | Iterative constraint relaxation solver         |
| `DistanceConstraint` | Maintains distance between two particles       |
| `PinConstraint`      | Pins a particle to a world-space point         |
| `AngleConstraint`    | Limits angle formed by three particles         |
| `BoundsConstraint`   | Keeps particles inside a rectangular region    |
| `VerletChain`        | Rope / chain built from particles + distance constraints |
| `VerletGrid`         | Cloth grid with optional tearing               |
| `SoftBody`           | 2D pressure-based deformable body              |
| `SolverConfig`       | Iteration count, relaxation, sub-stepping      |
| `StepObserver`       | Trait to observe each physics step              |
| `PhysicsError`       | Error type for invalid configurations          |

## License

Licensed under either of

- [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0)
- [MIT License](http://opensource.org/licenses/MIT)

at your option.
