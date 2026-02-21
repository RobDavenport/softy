//! Vector types and traits for physics calculations.

use crate::float::Float;
use core::ops::{Add, Sub, Neg};

/// Trait for vector types used in physics calculations.
///
/// Abstracts over dimensionality (1D, 2D, 3D) so all physics code
/// is generic over the vector type.
pub trait Vec:
    Copy
    + Clone
    + Add<Output = Self>
    + Sub<Output = Self>
    + Neg<Output = Self>
    + PartialEq
    + Default
    + core::fmt::Debug
{
    /// The scalar (float) type for this vector.
    type Scalar: Float;

    /// Zero vector.
    fn zero() -> Self;

    /// Vector with all components set to the same value.
    fn splat(value: Self::Scalar) -> Self;

    /// Dot product.
    fn dot(self, other: Self) -> Self::Scalar;

    /// Squared length (avoids sqrt).
    fn length_sq(self) -> Self::Scalar {
        self.dot(self)
    }

    /// Length (magnitude).
    fn length(self) -> Self::Scalar {
        self.length_sq().sqrt()
    }

    /// Normalize to unit length. Returns zero vector if length is near zero.
    fn normalize(self) -> Self {
        let len = self.length();
        if len.is_near_zero(Self::Scalar::from_f32(1e-10)) {
            Self::zero()
        } else {
            self.scale(Self::Scalar::one() / len)
        }
    }

    /// Scale all components by a scalar.
    fn scale(self, s: Self::Scalar) -> Self;

    /// Component-wise multiplication.
    fn component_mul(self, other: Self) -> Self;

    /// Distance between two points.
    fn distance(self, other: Self) -> Self::Scalar {
        (self - other).length()
    }

    /// Squared distance between two points.
    fn distance_sq(self, other: Self) -> Self::Scalar {
        (self - other).length_sq()
    }

    /// Linear interpolation between self and other.
    fn lerp(self, other: Self, t: Self::Scalar) -> Self {
        self + (other - self).scale(t)
    }
}

// --------------------------------------------------------------------------
// Scalar<F> — 1D wrapper
// --------------------------------------------------------------------------

/// 1D "vector" — a scalar value implementing the Vec trait.
///
/// Useful for 1D spring systems (e.g., camera zoom, UI element position).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Scalar<F: Float>(pub F);

impl<F: Float> Add for Scalar<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self { Scalar(self.0 + rhs.0) }
}

impl<F: Float> Sub for Scalar<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self { Scalar(self.0 - rhs.0) }
}

impl<F: Float> Neg for Scalar<F> {
    type Output = Self;
    fn neg(self) -> Self { Scalar(-self.0) }
}

impl<F: Float> Vec for Scalar<F> {
    type Scalar = F;
    fn zero() -> Self { Scalar(F::zero()) }
    fn splat(value: F) -> Self { Scalar(value) }
    fn dot(self, other: Self) -> F { self.0 * other.0 }
    fn scale(self, s: F) -> Self { Scalar(self.0 * s) }
    fn component_mul(self, other: Self) -> Self { Scalar(self.0 * other.0) }
}

// --------------------------------------------------------------------------
// Vec2<F> — 2D vector
// --------------------------------------------------------------------------

/// 2D vector for planar physics (cloth, 2D soft bodies, 2D ropes).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Vec2<F: Float> {
    pub x: F,
    pub y: F,
}

impl<F: Float> Vec2<F> {
    /// Create a new 2D vector.
    pub fn new(x: F, y: F) -> Self { Vec2 { x, y } }

    /// 2D cross product (returns scalar): self.x * other.y - self.y * other.x
    pub fn cross(self, other: Self) -> F {
        self.x * other.y - self.y * other.x
    }

    /// Perpendicular vector (rotated 90 degrees counter-clockwise).
    pub fn perp(self) -> Self {
        Vec2 { x: -self.y, y: self.x }
    }
}

impl<F: Float> Add for Vec2<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self { Vec2 { x: self.x + rhs.x, y: self.y + rhs.y } }
}

impl<F: Float> Sub for Vec2<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self { Vec2 { x: self.x - rhs.x, y: self.y - rhs.y } }
}

impl<F: Float> Neg for Vec2<F> {
    type Output = Self;
    fn neg(self) -> Self { Vec2 { x: -self.x, y: -self.y } }
}

impl<F: Float> Vec for Vec2<F> {
    type Scalar = F;
    fn zero() -> Self { Vec2 { x: F::zero(), y: F::zero() } }
    fn splat(value: F) -> Self { Vec2 { x: value, y: value } }
    fn dot(self, other: Self) -> F { self.x * other.x + self.y * other.y }
    fn scale(self, s: F) -> Self { Vec2 { x: self.x * s, y: self.y * s } }
    fn component_mul(self, other: Self) -> Self {
        Vec2 { x: self.x * other.x, y: self.y * other.y }
    }
}

// --------------------------------------------------------------------------
// Vec3<F> — 3D vector
// --------------------------------------------------------------------------

/// 3D vector for spatial physics (3D ropes, cloth in 3D space).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Vec3<F: Float> {
    pub x: F,
    pub y: F,
    pub z: F,
}

impl<F: Float> Vec3<F> {
    /// Create a new 3D vector.
    pub fn new(x: F, y: F, z: F) -> Self { Vec3 { x, y, z } }

    /// 3D cross product.
    pub fn cross(self, other: Self) -> Self {
        Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
}

impl<F: Float> Add for Vec3<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Vec3 { x: self.x + rhs.x, y: self.y + rhs.y, z: self.z + rhs.z }
    }
}

impl<F: Float> Sub for Vec3<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Vec3 { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z }
    }
}

impl<F: Float> Neg for Vec3<F> {
    type Output = Self;
    fn neg(self) -> Self { Vec3 { x: -self.x, y: -self.y, z: -self.z } }
}

impl<F: Float> Vec for Vec3<F> {
    type Scalar = F;
    fn zero() -> Self { Vec3 { x: F::zero(), y: F::zero(), z: F::zero() } }
    fn splat(value: F) -> Self { Vec3 { x: value, y: value, z: value } }
    fn dot(self, other: Self) -> F {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    fn scale(self, s: F) -> Self {
        Vec3 { x: self.x * s, y: self.y * s, z: self.z * s }
    }
    fn component_mul(self, other: Self) -> Self {
        Vec3 { x: self.x * other.x, y: self.y * other.y, z: self.z * other.z }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec2_length() {
        let v = Vec2::new(3.0f32, 4.0);
        assert!((v.length() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn vec3_cross() {
        let i = Vec3::new(1.0f32, 0.0, 0.0);
        let j = Vec3::new(0.0f32, 1.0, 0.0);
        let k = i.cross(j);
        assert!((k.x - 0.0).abs() < 1e-6);
        assert!((k.y - 0.0).abs() < 1e-6);
        assert!((k.z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn scalar_dot() {
        let a = Scalar(3.0f32);
        let b = Scalar(4.0f32);
        assert!((a.dot(b) - 12.0).abs() < 1e-6);
    }

    #[test]
    fn normalize_zero_vector() {
        let v = Vec2::<f32>::zero();
        let n = v.normalize();
        assert_eq!(n, Vec2::zero());
    }

    #[test]
    fn lerp_midpoint() {
        let a = Vec2::new(0.0f32, 0.0);
        let b = Vec2::new(10.0f32, 10.0);
        let mid = a.lerp(b, 0.5);
        assert!((mid.x - 5.0).abs() < 1e-6);
        assert!((mid.y - 5.0).abs() < 1e-6);
    }

    #[test]
    fn distance_calculation() {
        let a = Vec2::new(0.0f32, 0.0);
        let b = Vec2::new(3.0f32, 4.0);
        assert!((a.distance(b) - 5.0).abs() < 1e-6);
    }
}
