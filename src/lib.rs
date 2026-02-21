//! Spring-damper systems and Verlet integration for games.
//!
//! `softy` provides analytical spring-damper systems (critically/under/over-damped)
//! and position-based Verlet dynamics with constraint solving. Designed for game use:
//! camera follow, UI juice, ropes, cloth, and soft bodies.
//!
//! # Features
//!
//! - **Analytical springs**: Closed-form spring-damper (no Euler drift)
//! - **Verlet integration**: Position-based dynamics with implicit velocity
//! - **Constraint solver**: Distance, pin, angle, bounds — iterative relaxation
//! - **Ropes & cloth**: `VerletChain` and `VerletGrid` with tear support
//! - **Soft bodies**: 2D pressure-based deformable shapes
//! - **Observable**: Monitor physics steps via the `StepObserver` trait
//! - **`no_std` compatible**: Works in embedded and WASM environments

#![no_std]

extern crate alloc;

pub mod float;
pub mod vec;
pub mod spring;
pub mod particle;
pub mod constraint;
pub mod solver;
pub mod chain;
pub mod grid;
pub mod softbody;
pub mod observer;
pub mod config;
pub mod error;

// Re-export primary API
pub use float::Float;
pub use vec::{Vec, Scalar, Vec2, Vec3};
pub use spring::{Spring, Spring1D, Spring2D, Spring3D, DampingMode};
pub use particle::Particle;
pub use constraint::{Constraint, ConstraintDispatch, DistanceConstraint, PinConstraint, AngleConstraint, BoundsConstraint};
pub use solver::ConstraintSolver;
pub use chain::{VerletChain, ChainConfig};
pub use grid::{VerletGrid, GridConfig};
pub use softbody::SoftBody;
pub use config::SolverConfig;
pub use observer::{StepObserver, NoOpStepObserver};
pub use error::PhysicsError;
