//! Error types for physics operations.

use core::fmt;

/// Errors that can occur during physics operations.
#[derive(Debug, Clone, PartialEq)]
pub enum PhysicsError {
    /// Mass must be positive and finite.
    InvalidMass,
    /// Stiffness must be in [0, 1].
    InvalidStiffness,
    /// Frequency must be positive.
    InvalidFrequency,
    /// Damping ratio must be positive.
    InvalidDampingRatio,
    /// Particle index is out of bounds.
    ParticleOutOfBounds { index: usize, count: usize },
    /// A constraint could not be satisfied within the iteration limit.
    ConstraintViolation,
    /// Grid dimensions must be at least 2x2.
    InvalidGridDimensions,
    /// Soft body must have at least 3 segments.
    InsufficientSegments,
}

impl fmt::Display for PhysicsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PhysicsError::InvalidMass => write!(f, "mass must be positive and finite"),
            PhysicsError::InvalidStiffness => write!(f, "stiffness must be in [0, 1]"),
            PhysicsError::InvalidFrequency => write!(f, "frequency must be positive"),
            PhysicsError::InvalidDampingRatio => write!(f, "damping ratio must be positive"),
            PhysicsError::ParticleOutOfBounds { index, count } => {
                write!(f, "particle index {} out of bounds (count: {})", index, count)
            }
            PhysicsError::ConstraintViolation => write!(f, "constraint could not be satisfied"),
            PhysicsError::InvalidGridDimensions => write!(f, "grid must be at least 2x2"),
            PhysicsError::InsufficientSegments => write!(f, "soft body needs at least 3 segments"),
        }
    }
}
