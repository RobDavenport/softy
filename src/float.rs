//! Floating-point abstraction trait for generic numeric operations.

use core::cmp::PartialOrd;
use core::ops::{Add, Sub, Mul, Div, Neg};

/// Trait abstracting floating-point operations needed by the physics engine.
///
/// Implemented for `f32` and `f64`. Could be extended to fixed-point types.
pub trait Float:
    Copy
    + Clone
    + PartialEq
    + PartialOrd
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
    + Default
    + core::fmt::Debug
{
    /// The additive identity (0.0).
    fn zero() -> Self;
    /// The multiplicative identity (1.0).
    fn one() -> Self;
    /// Half (0.5).
    fn half() -> Self;
    /// Two (2.0).
    fn two() -> Self;
    /// Pi (~3.14159).
    fn pi() -> Self;
    /// Square root.
    fn sqrt(self) -> Self;
    /// Sine.
    fn sin(self) -> Self;
    /// Cosine.
    fn cos(self) -> Self;
    /// Natural exponential (e^self).
    fn exp(self) -> Self;
    /// Absolute value.
    fn abs(self) -> Self;
    /// Minimum of two values.
    fn min(self, other: Self) -> Self;
    /// Maximum of two values.
    fn max(self, other: Self) -> Self;
    /// Convert from f32 (for constants and configuration).
    fn from_f32(v: f32) -> Self;
    /// Arctangent of y/x, with correct quadrant.
    fn atan2(y: Self, x: Self) -> Self;
    /// Floor.
    fn floor(self) -> Self;
    /// Ceiling.
    fn ceil(self) -> Self;

    /// Clamp self to [min, max].
    fn clamp(self, min: Self, max: Self) -> Self {
        self.max(min).min(max)
    }

    /// Linear interpolation: self + (other - self) * t
    fn lerp(self, other: Self, t: Self) -> Self {
        self + (other - self) * t
    }

    /// Check if approximately zero within epsilon.
    fn is_near_zero(self, epsilon: Self) -> bool {
        self.abs() < epsilon
    }
}

impl Float for f32 {
    fn zero() -> Self { 0.0 }
    fn one() -> Self { 1.0 }
    fn half() -> Self { 0.5 }
    fn two() -> Self { 2.0 }
    fn pi() -> Self { core::f32::consts::PI }
    fn sqrt(self) -> Self { libm::sqrtf(self) }
    fn sin(self) -> Self { libm::sinf(self) }
    fn cos(self) -> Self { libm::cosf(self) }
    fn exp(self) -> Self { libm::expf(self) }
    fn abs(self) -> Self { libm::fabsf(self) }
    fn min(self, other: Self) -> Self { if self < other { self } else { other } }
    fn max(self, other: Self) -> Self { if self > other { self } else { other } }
    fn from_f32(v: f32) -> Self { v }
    fn atan2(y: Self, x: Self) -> Self { libm::atan2f(y, x) }
    fn floor(self) -> Self { libm::floorf(self) }
    fn ceil(self) -> Self { libm::ceilf(self) }
}

impl Float for f64 {
    fn zero() -> Self { 0.0 }
    fn one() -> Self { 1.0 }
    fn half() -> Self { 0.5 }
    fn two() -> Self { 2.0 }
    fn pi() -> Self { core::f64::consts::PI }
    fn sqrt(self) -> Self { libm::sqrt(self) }
    fn sin(self) -> Self { libm::sin(self) }
    fn cos(self) -> Self { libm::cos(self) }
    fn exp(self) -> Self { libm::exp(self) }
    fn abs(self) -> Self { libm::fabs(self) }
    fn min(self, other: Self) -> Self { if self < other { self } else { other } }
    fn max(self, other: Self) -> Self { if self > other { self } else { other } }
    fn from_f32(v: f32) -> Self { v as f64 }
    fn atan2(y: Self, x: Self) -> Self { libm::atan2(y, x) }
    fn floor(self) -> Self { libm::floor(self) }
    fn ceil(self) -> Self { libm::ceil(self) }
}
