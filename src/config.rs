//! Configuration types for the physics solver.

use crate::float::Float;
use crate::vec::Vec;

/// Configuration for the constraint solver and simulation.
///
/// # Builder Pattern
/// ```
/// use softy::config::SolverConfig;
/// use softy::vec::Vec2;
///
/// let config: SolverConfig<Vec2<f32>> = SolverConfig::new()
///     .with_iterations(8)
///     .with_gravity(Vec2::new(0.0, -9.81))
///     .with_damping(0.99)
///     .with_sub_steps(2);
/// ```
pub struct SolverConfig<V: Vec> {
    /// Number of constraint solver iterations per sub-step.
    /// More iterations = more accurate but slower. Default: 4.
    pub iterations: usize,
    /// Gravity acceleration vector. Default: zero (no gravity).
    pub gravity: V,
    /// Velocity damping factor [0, 1]. 1.0 = no damping. Default: 0.99.
    pub damping: V::Scalar,
    /// Number of sub-steps per frame. Higher = more stable. Default: 1.
    pub sub_steps: usize,
}

impl<V: Vec> SolverConfig<V> {
    /// Create a new config with default values.
    pub fn new() -> Self {
        SolverConfig {
            iterations: 4,
            gravity: V::zero(),
            damping: V::Scalar::from_f32(0.99),
            sub_steps: 1,
        }
    }

    /// Set the number of constraint iterations.
    pub fn with_iterations(mut self, iterations: usize) -> Self {
        self.iterations = iterations;
        self
    }

    /// Set the gravity vector.
    pub fn with_gravity(mut self, gravity: V) -> Self {
        self.gravity = gravity;
        self
    }

    /// Set the damping factor.
    pub fn with_damping(mut self, damping: V::Scalar) -> Self {
        self.damping = damping;
        self
    }

    /// Set the number of sub-steps.
    pub fn with_sub_steps(mut self, sub_steps: usize) -> Self {
        self.sub_steps = sub_steps.max(1);
        self
    }
}

impl<V: Vec> Default for SolverConfig<V> {
    fn default() -> Self {
        Self::new()
    }
}
