# softy Implementation Plan

## What This Is

A `no_std` Rust library for spring-damper systems and Verlet integration, targeting game use cases: camera follow, UI juice, ropes, cloth, and soft bodies. All module files are scaffolded with doc-comment stubs. Your job: fill in the implementations, write tests, build the WASM demo.

**Key design principles:**
- Analytical (closed-form) springs — no Euler integration drift
- Position-based Verlet dynamics with iterative constraint relaxation
- Generic over float type (`f32`/`f64`) and vector dimensionality (1D/2D/3D)
- Deterministic: same inputs always produce same outputs

## Hard Rules

- `#![no_std]` with `extern crate alloc` — no std dependency in core library
- `rand_core` and `libm` are the ONLY dependencies
- All tests: `cargo test --target x86_64-pc-windows-msvc`
- WASM check: `cargo build --target wasm32-unknown-unknown --release`
- Deterministic: same inputs must produce same results
- No f32 rounding drift: springs use closed-form analytical solutions, NOT Euler integration
- All physics math uses the `Float` trait — never raw `f32`/`f64` directly in generic code

## Reference Implementation

The sibling libraries use the same patterns. Key files to study:
- `../wavfc/src/lib.rs` — `no_std` lib structure with re-exports
- `../wavfc/Cargo.toml` — workspace + features pattern
- `../rulebound/src/lib.rs` — module organization and re-export style
- `../rulebound/demo-wasm/src/lib.rs` — WASM FFI bindings pattern
- `../rulebound/demo-wasm/www/` — Demo UI pattern

## Implementation Steps

### Phase 1: Math Foundations (`float.rs` + `vec.rs`)

#### Step 1: Float Trait (`src/float.rs`)

The `Float` trait abstracts over numeric types so all physics code is generic. Implement for `f32` and `f64`.

```rust
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

// Implement Float for f32
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

// Implement Float for f64 (same pattern, using libm double-precision functions)
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
```

**IMPORTANT:** Since this is `no_std`, you CANNOT use `f32::sqrt()` etc. directly (those are in std). You must use the `libm` crate for math functions. Add `libm` as a dependency:

```toml
[dependencies]
rand_core = { version = "0.6", default-features = false }
libm = "0.2"
```

**Alternative (no extra dep):** Use inline assembly or manual approximations. But `libm` is the standard `no_std` math solution and is a pure-Rust implementation with no dependencies of its own. **Use `libm`.**

#### Step 2: Vec Trait and Types (`src/vec.rs`)

```rust
use crate::float::Float;
use core::ops::{Add, Sub, Mul, Neg};

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
        // self + (other - self) * t
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
```

**Tests for Phase 1:**
- `Vec2::new(3, 4).length()` approx equals `5.0`
- `Vec3::cross` correctness: i x j = k
- `Scalar` dot product is multiplication
- `normalize` of zero vector returns zero vector
- `lerp(a, b, 0.5)` is midpoint
- `distance(a, b)` matches manual calculation

---

### Phase 2: Springs (`spring.rs`)

#### Step 3: DampingMode and Spring Struct

```rust
use crate::float::Float;
use crate::vec::Vec;

/// Damping mode for a spring-damper system.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DampingMode<F: Float> {
    /// Damping ratio = 1.0. Reaches target as fast as possible with no overshoot.
    CriticallyDamped,
    /// Damping ratio < 1.0. Oscillates around target with decreasing amplitude.
    Underdamped { ratio: F },
    /// Damping ratio > 1.0. Approaches target slowly, no oscillation.
    Overdamped { ratio: F },
}

/// Analytical spring-damper system with closed-form solution.
///
/// Uses exact mathematical solutions instead of Euler integration,
/// eliminating drift and energy gain/loss from numerical errors.
///
/// # Type Parameters
/// - `V`: The vector type (Scalar, Vec2, or Vec3)
///
/// # Example
/// ```
/// use softy::spring::Spring;
/// use softy::vec::Vec2;
///
/// let mut spring: Spring<Vec2<f32>> = Spring::critically_damped(
///     Vec2::new(0.0, 0.0),  // initial position
///     Vec2::new(10.0, 5.0), // target
///     4.0,                   // frequency (angular_freq = 2*pi*freq)
/// );
///
/// // Advance 1/60th of a second
/// spring.update(1.0 / 60.0);
/// ```
pub struct Spring<V: Vec> {
    /// Current value of the spring.
    current: V,
    /// Current velocity.
    velocity: V,
    /// Target value the spring approaches.
    target: V,
    /// Angular frequency (omega = 2 * pi * frequency_hz).
    angular_freq: V::Scalar,
    /// Damping mode and ratio.
    mode: DampingMode<V::Scalar>,
}
```

#### Step 4: Spring Constructors

```rust
impl<V: Vec> Spring<V> {
    /// Create a critically damped spring (damping ratio = 1.0).
    ///
    /// Reaches target as fast as possible with zero overshoot.
    /// Best for: camera follow, UI element transitions.
    ///
    /// # Arguments
    /// - `initial`: Starting position
    /// - `target`: Target position
    /// - `frequency_hz`: Oscillation frequency in Hz (higher = stiffer/faster)
    pub fn critically_damped(initial: V, target: V, frequency_hz: V::Scalar) -> Self {
        let two_pi = V::Scalar::two() * V::Scalar::pi();
        Spring {
            current: initial,
            velocity: V::zero(),
            target,
            angular_freq: two_pi * frequency_hz,
            mode: DampingMode::CriticallyDamped,
        }
    }

    /// Create an underdamped spring (damping ratio < 1.0).
    ///
    /// Oscillates around the target with exponentially decaying amplitude.
    /// Best for: bouncy UI, wobbly objects, springy game feel.
    ///
    /// # Arguments
    /// - `initial`: Starting position
    /// - `target`: Target position
    /// - `frequency_hz`: Oscillation frequency in Hz
    /// - `damping_ratio`: Damping ratio in (0.0, 1.0). Lower = more bouncy.
    ///
    /// # Panics
    /// Panics if `damping_ratio` is not in (0.0, 1.0).
    pub fn underdamped(
        initial: V,
        target: V,
        frequency_hz: V::Scalar,
        damping_ratio: V::Scalar,
    ) -> Self {
        // Debug assert: ratio must be in (0, 1)
        let two_pi = V::Scalar::two() * V::Scalar::pi();
        Spring {
            current: initial,
            velocity: V::zero(),
            target,
            angular_freq: two_pi * frequency_hz,
            mode: DampingMode::Underdamped { ratio: damping_ratio },
        }
    }

    /// Create an overdamped spring (damping ratio > 1.0).
    ///
    /// Approaches target slowly without oscillation.
    /// Best for: heavy/sluggish objects, slow fade-ins.
    ///
    /// # Arguments
    /// - `initial`: Starting position
    /// - `target`: Target position
    /// - `frequency_hz`: Oscillation frequency in Hz
    /// - `damping_ratio`: Damping ratio > 1.0. Higher = slower.
    ///
    /// # Panics
    /// Panics if `damping_ratio` is not > 1.0.
    pub fn overdamped(
        initial: V,
        target: V,
        frequency_hz: V::Scalar,
        damping_ratio: V::Scalar,
    ) -> Self {
        let two_pi = V::Scalar::two() * V::Scalar::pi();
        Spring {
            current: initial,
            velocity: V::zero(),
            target,
            angular_freq: two_pi * frequency_hz,
            mode: DampingMode::Overdamped { ratio: damping_ratio },
        }
    }
}
```

#### Step 5: Spring Update — The Core Math

This is the most important function. Each damping mode has a different closed-form solution. All solutions are computed relative to the target (offset = current - target).

```rust
impl<V: Vec> Spring<V> {
    /// Advance the spring by `dt` seconds using the analytical closed-form solution.
    ///
    /// This does NOT use Euler integration. Each damping mode has an exact
    /// mathematical solution that is evaluated directly, preventing energy
    /// drift and ensuring deterministic results.
    pub fn update(&mut self, dt: V::Scalar) {
        // Work in offset-from-target space
        let x0 = self.current - self.target; // initial displacement from target
        let v0 = self.velocity;               // initial velocity
        let w = self.angular_freq;            // omega

        let (new_offset, new_velocity) = match self.mode {
            DampingMode::CriticallyDamped => {
                // Critically damped (zeta = 1):
                //   x(t) = (x0 + (v0 + w*x0)*t) * e^(-w*t)
                //   v(t) = (v0 - (v0 + w*x0)*w*t) * e^(-w*t)
                //
                // Derivation: characteristic equation has repeated root r = -w
                // General solution: x(t) = (C1 + C2*t) * e^(-w*t)
                // With initial conditions x(0) = x0, x'(0) = v0:
                //   C1 = x0
                //   C2 = v0 + w*x0
                let exp_term = (-w * dt).exp(); // e^(-w*t)
                let c2 = v0 + x0.scale(w);      // v0 + w*x0 (this is a vector)

                // x(t) = (x0 + c2*t) * exp
                let new_x = (x0 + c2.scale(dt)).scale(exp_term);

                // v(t) = (v0 - c2*w*t) * exp
                let new_v = (v0 - c2.scale(w * dt)).scale(exp_term);

                (new_x, new_v)
            }

            DampingMode::Underdamped { ratio: zeta } => {
                // Underdamped (0 < zeta < 1):
                //   wd = w * sqrt(1 - zeta^2)           — damped frequency
                //   x(t) = e^(-zeta*w*t) * (A*cos(wd*t) + B*sin(wd*t))
                //   where A = x0, B = (v0 + zeta*w*x0) / wd
                //
                //   v(t) = e^(-zeta*w*t) * (
                //       (B*wd - A*zeta*w)*cos(wd*t)
                //     - (A*wd + B*zeta*w)*sin(wd*t)
                //   )
                let one = V::Scalar::one();
                let wd = w * (one - zeta * zeta).sqrt(); // damped angular frequency
                let exp_term = (-zeta * w * dt).exp();
                let cos_term = (wd * dt).cos();
                let sin_term = (wd * dt).sin();

                let a = x0;
                // B = (v0 + zeta*w*x0) / wd
                let b = (v0 + x0.scale(zeta * w)).scale(one / wd);

                // x(t) = exp * (A*cos + B*sin)
                let new_x = (a.scale(cos_term) + b.scale(sin_term)).scale(exp_term);

                // v(t) = exp * ((B*wd - A*zeta*w)*cos - (A*wd + B*zeta*w)*sin)
                let v_cos_coeff = b.scale(wd) - a.scale(zeta * w);
                let v_sin_coeff = a.scale(wd) + b.scale(zeta * w);
                let new_v = (v_cos_coeff.scale(cos_term) - v_sin_coeff.scale(sin_term))
                    .scale(exp_term);

                (new_x, new_v)
            }

            DampingMode::Overdamped { ratio: zeta } => {
                // Overdamped (zeta > 1):
                //   s = sqrt(zeta^2 - 1)
                //   r1 = -w * (zeta - s)      — slow decay root
                //   r2 = -w * (zeta + s)      — fast decay root
                //   x(t) = C1*e^(r1*t) + C2*e^(r2*t)
                //   where C1 = (r2*x0 - v0) / (r2 - r1)
                //         C2 = (v0 - r1*x0) / (r2 - r1)
                //   v(t) = C1*r1*e^(r1*t) + C2*r2*e^(r2*t)
                let one = V::Scalar::one();
                let s = (zeta * zeta - one).sqrt();
                let r1 = -w * (zeta - s);
                let r2 = -w * (zeta + s);
                let denom = one / (r2 - r1);

                // C1 = (r2*x0 - v0) / (r2 - r1)
                let c1 = (x0.scale(r2) - v0).scale(denom);
                // C2 = (v0 - r1*x0) / (r2 - r1)
                let c2 = (v0 - x0.scale(r1)).scale(denom);

                let exp1 = (r1 * dt).exp();
                let exp2 = (r2 * dt).exp();

                let new_x = c1.scale(exp1) + c2.scale(exp2);
                let new_v = c1.scale(r1 * exp1) + c2.scale(r2 * exp2);

                (new_x, new_v)
            }
        };

        // Convert back from offset space to absolute position
        self.current = self.target + new_offset;
        self.velocity = new_velocity;
    }

    /// Set a new target for the spring to approach.
    pub fn set_target(&mut self, target: V) {
        self.target = target;
    }

    /// Get the current value.
    pub fn value(&self) -> V {
        self.current
    }

    /// Get the current velocity.
    pub fn velocity(&self) -> V {
        self.velocity
    }

    /// Check if the spring has settled (position and velocity near zero offset).
    pub fn is_settled(&self, eps_pos: V::Scalar, eps_vel: V::Scalar) -> bool {
        let offset = self.current - self.target;
        offset.length_sq() < eps_pos * eps_pos
            && self.velocity.length_sq() < eps_vel * eps_vel
    }

    /// Get the angular frequency.
    pub fn angular_freq(&self) -> V::Scalar {
        self.angular_freq
    }

    /// Get the damping mode.
    pub fn mode(&self) -> &DampingMode<V::Scalar> {
        &self.mode
    }

    /// Reset the spring to a new position with zero velocity.
    pub fn reset(&mut self, position: V) {
        self.current = position;
        self.velocity = V::zero();
    }

    /// Instantly move the spring to its target with zero velocity.
    pub fn snap_to_target(&mut self) {
        self.current = self.target;
        self.velocity = V::zero();
    }
}
```

#### Type Aliases

```rust
use crate::vec::{Scalar, Vec2, Vec3};

/// 1D spring (e.g., camera zoom, scroll position, opacity).
pub type Spring1D<F> = Spring<Scalar<F>>;
/// 2D spring (e.g., camera follow, UI element position).
pub type Spring2D<F> = Spring<Vec2<F>>;
/// 3D spring (e.g., camera orbit, 3D object tracking).
pub type Spring3D<F> = Spring<Vec3<F>>;
```

**Tests for Phase 2:**
- **Convergence**: After many updates, all 3 modes reach target within epsilon
- **Critically damped — no overshoot**: Track the distance to target; it should monotonically decrease (or at most remain constant)
- **Underdamped — oscillation**: The sign of (current - target) should flip at least once during the first few updates
- **Overdamped — slow approach**: After same time, overdamped spring should be farther from target than critically damped
- **set_target**: Changing target mid-simulation works correctly
- **is_settled**: Returns true once spring reaches target
- **Determinism**: Same inputs produce identical results across multiple runs
- **Zero dt**: `update(0.0)` should not change state

---

### Phase 3: Verlet Core (`particle.rs` + `constraint.rs` + `solver.rs`)

#### Step 6: Particle (`src/particle.rs`)

```rust
use crate::float::Float;
use crate::vec::Vec;

/// A Verlet particle — position-based dynamics with implicit velocity.
///
/// Verlet integration stores current and previous position instead of
/// position and velocity. Velocity is implicit: v = (pos - prev_pos) / dt.
/// This makes constraint solving trivial (just move positions) and provides
/// natural damping.
///
/// # Fields
/// - `pos`: Current position
/// - `prev_pos`: Position from the previous time step
/// - `acceleration`: Accumulated forces / mass for this frame
/// - `mass`: Particle mass (used for force application)
/// - `inv_mass`: 1.0 / mass (0.0 for pinned/static particles)
/// - `pinned`: If true, the particle ignores forces and constraints
#[derive(Clone, Debug)]
pub struct Particle<V: Vec> {
    pub pos: V,
    pub prev_pos: V,
    pub acceleration: V,
    pub mass: V::Scalar,
    pub inv_mass: V::Scalar,
    pub pinned: bool,
}

impl<V: Vec> Particle<V> {
    /// Create a new particle at `pos` with the given `mass`.
    pub fn new(pos: V, mass: V::Scalar) -> Self {
        Particle {
            pos,
            prev_pos: pos,
            acceleration: V::zero(),
            mass,
            inv_mass: V::Scalar::one() / mass,
            pinned: false,
        }
    }

    /// Create a pinned (static) particle at `pos`.
    ///
    /// Pinned particles have inv_mass = 0 and are not moved by forces or constraints.
    pub fn pinned(pos: V) -> Self {
        Particle {
            pos,
            prev_pos: pos,
            acceleration: V::zero(),
            mass: V::Scalar::zero(),
            inv_mass: V::Scalar::zero(),
            pinned: true,
        }
    }

    /// Apply a force (acceleration += force / mass). Ignored if pinned.
    pub fn apply_force(&mut self, force: V) {
        if !self.pinned {
            self.acceleration = self.acceleration + force.scale(self.inv_mass);
        }
    }

    /// Apply a raw acceleration (no mass division). Ignored if pinned.
    /// Useful for gravity (same acceleration for all masses).
    pub fn apply_acceleration(&mut self, accel: V) {
        if !self.pinned {
            self.acceleration = self.acceleration + accel;
        }
    }

    /// Verlet integration step.
    ///
    /// Formula: new_pos = pos + (pos - prev_pos) * damping + acceleration * dt^2
    ///
    /// The `damping` parameter controls velocity damping (1.0 = no damping,
    /// 0.99 = slight damping, 0.95 = heavy damping). Values should be in [0, 1].
    pub fn integrate(&mut self, dt: V::Scalar, damping: V::Scalar) {
        if self.pinned {
            return;
        }
        let velocity = (self.pos - self.prev_pos).scale(damping);
        let new_pos = self.pos + velocity + self.acceleration.scale(dt * dt);
        self.prev_pos = self.pos;
        self.pos = new_pos;
        self.acceleration = V::zero(); // reset accumulated acceleration
    }

    /// Get the implicit velocity: (pos - prev_pos) / dt.
    pub fn velocity(&self, dt: V::Scalar) -> V {
        (self.pos - self.prev_pos).scale(V::Scalar::one() / dt)
    }

    /// Get the implicit velocity without dt normalization: (pos - prev_pos).
    /// Useful when dt is constant and you just need a direction/magnitude.
    pub fn velocity_raw(&self) -> V {
        self.pos - self.prev_pos
    }

    /// Pin this particle at its current position.
    pub fn pin(&mut self) {
        self.pinned = true;
        self.inv_mass = V::Scalar::zero();
        self.prev_pos = self.pos;
    }

    /// Unpin this particle, restoring its original mass.
    pub fn unpin(&mut self, mass: V::Scalar) {
        self.pinned = false;
        self.mass = mass;
        self.inv_mass = V::Scalar::one() / mass;
    }

    /// Move a pinned particle to a new position.
    ///
    /// Only works on pinned particles. For non-pinned particles, use forces.
    pub fn move_to(&mut self, pos: V) {
        if self.pinned {
            self.prev_pos = self.pos;
            self.pos = pos;
        }
    }
}
```

#### Step 7: Constraints (`src/constraint.rs`)

```rust
use crate::float::Float;
use crate::vec::Vec;
use crate::particle::Particle;
use alloc::vec::Vec as AllocVec;

/// A constraint that can be applied to a set of particles.
///
/// Constraints are solved iteratively — each constraint adjusts particle
/// positions to satisfy its condition. Multiple iterations converge toward
/// a valid solution.
pub enum Constraint<V: Vec> {
    /// Maintain a fixed distance between two particles.
    Distance(DistanceConstraint<V>),
    /// Pin a particle to a fixed position.
    Pin(PinConstraint<V>),
    /// Maintain an angle between three particles.
    Angle(AngleConstraint<V>),
    /// Keep particles within axis-aligned bounds.
    Bounds(BoundsConstraint<V>),
}

/// Distance constraint: keeps two particles at a fixed rest length.
///
/// # Projection Math
/// ```text
/// delta = b.pos - a.pos
/// current_dist = |delta|
/// error = current_dist - rest_length
/// correction = delta.normalize() * error * 0.5 * stiffness
/// a.pos += correction  (if not pinned, weighted by inv_mass)
/// b.pos -= correction  (if not pinned, weighted by inv_mass)
/// ```
///
/// Mass weighting: each particle moves proportional to its inv_mass
/// relative to the total inv_mass of both particles:
/// ```text
/// w_total = a.inv_mass + b.inv_mass
/// a.pos += correction * (a.inv_mass / w_total)
/// b.pos -= correction * (b.inv_mass / w_total)
/// ```
pub struct DistanceConstraint<V: Vec> {
    /// Index of first particle.
    pub a: usize,
    /// Index of second particle.
    pub b: usize,
    /// Target distance between the two particles.
    pub rest_length: V::Scalar,
    /// Stiffness in [0, 1]. 1.0 = rigid, lower = stretchy.
    pub stiffness: V::Scalar,
}

/// Pin constraint: holds a particle at a fixed position.
///
/// # Projection Math
/// ```text
/// correction = pin_position - particle.pos
/// particle.pos += correction * stiffness
/// ```
pub struct PinConstraint<V: Vec> {
    /// Index of the pinned particle.
    pub particle: usize,
    /// Target position.
    pub position: V,
    /// Stiffness in [0, 1]. 1.0 = perfectly rigid pin.
    pub stiffness: V::Scalar,
}

/// Angle constraint: maintains an angle between three particles (a-b-c).
///
/// The angle is measured at particle `b` between segments b->a and b->c.
///
/// # Projection Math
/// ```text
/// ba = a.pos - b.pos
/// bc = c.pos - b.pos
/// current_angle = atan2(cross(ba, bc), dot(ba, bc))
/// angle_error = current_angle - target_angle
/// // Rotate a and c around b to reduce angle_error
/// rotation = angle_error * stiffness * 0.5
/// a.pos = rotate(a.pos - b.pos, -rotation) + b.pos
/// c.pos = rotate(c.pos - b.pos, +rotation) + b.pos
/// ```
///
/// Note: Angle constraints are only meaningful in 2D. For 3D, they
/// constrain the angle in the plane defined by the three particles.
pub struct AngleConstraint<V: Vec> {
    /// First endpoint.
    pub a: usize,
    /// Center vertex (angle measured here).
    pub b: usize,
    /// Second endpoint.
    pub c: usize,
    /// Target angle in radians.
    pub target_angle: V::Scalar,
    /// Stiffness in [0, 1].
    pub stiffness: V::Scalar,
}

/// Bounds constraint: keeps particles within an axis-aligned bounding box.
///
/// # Projection Math
/// For each component of each particle:
/// ```text
/// if particle.pos[i] < min[i]:
///     particle.pos[i] = min[i]
///     // Reflect velocity: prev_pos[i] = pos[i] + (pos[i] - prev_pos[i]) * restitution
/// if particle.pos[i] > max[i]:
///     particle.pos[i] = max[i]
///     // Reflect velocity similarly
/// ```
pub struct BoundsConstraint<V: Vec> {
    /// Minimum corner of the bounding box.
    pub min: V,
    /// Maximum corner of the bounding box.
    pub max: V,
    /// Bounciness factor [0, 1]. 0 = no bounce, 1 = perfect bounce.
    pub restitution: V::Scalar,
}

impl<V: Vec> Constraint<V> {
    /// Apply this constraint to the given particles.
    ///
    /// Adjusts particle positions to satisfy the constraint condition.
    pub fn solve(&self, particles: &mut [Particle<V>]) {
        match self {
            Constraint::Distance(c) => c.solve(particles),
            Constraint::Pin(c) => c.solve(particles),
            Constraint::Angle(c) => c.solve(particles),
            Constraint::Bounds(c) => c.solve(particles),
        }
    }
}

impl<V: Vec> DistanceConstraint<V> {
    /// Create a new distance constraint.
    pub fn new(a: usize, b: usize, rest_length: V::Scalar, stiffness: V::Scalar) -> Self {
        DistanceConstraint { a, b, rest_length, stiffness }
    }

    /// Create a distance constraint with rest_length computed from current positions.
    pub fn from_particles(a: usize, b: usize, particles: &[Particle<V>], stiffness: V::Scalar) -> Self {
        let rest_length = particles[a].pos.distance(particles[b].pos);
        DistanceConstraint { a, b, rest_length, stiffness }
    }

    /// Project particles to satisfy the distance constraint.
    pub fn solve(&self, particles: &mut [Particle<V>]) {
        // Implementation: compute delta, error, mass-weighted correction
        // See projection math in struct docs above
        todo!()
    }
}

impl<V: Vec> PinConstraint<V> {
    pub fn new(particle: usize, position: V, stiffness: V::Scalar) -> Self {
        PinConstraint { particle, position, stiffness }
    }

    pub fn solve(&self, particles: &mut [Particle<V>]) {
        todo!()
    }
}

impl<V: Vec> AngleConstraint<V> {
    pub fn new(a: usize, b: usize, c: usize, target_angle: V::Scalar, stiffness: V::Scalar) -> Self {
        AngleConstraint { a, b, c, target_angle, stiffness }
    }

    pub fn solve(&self, particles: &mut [Particle<V>]) {
        todo!()
    }
}

impl<V: Vec> BoundsConstraint<V> {
    pub fn new(min: V, max: V, restitution: V::Scalar) -> Self {
        BoundsConstraint { min, max, restitution }
    }

    pub fn solve(&self, particles: &mut [Particle<V>]) {
        todo!()
    }
}
```

**BoundsConstraint for Vec2 specifically** — since bounds need per-component access, implement `solve` specifically for `Vec2<F>` and `Vec3<F>`. Provide a trait-level method if possible, or implement separately:

```rust
impl<F: Float> BoundsConstraint<Vec2<F>> {
    pub fn solve_2d(&self, particles: &mut [Particle<Vec2<F>>]) {
        for p in particles.iter_mut() {
            if p.pinned { continue; }
            // X axis
            if p.pos.x < self.min.x {
                p.pos.x = self.min.x;
                let vel_x = p.pos.x - p.prev_pos.x;
                p.prev_pos.x = p.pos.x + vel_x * self.restitution;
            } else if p.pos.x > self.max.x {
                p.pos.x = self.max.x;
                let vel_x = p.pos.x - p.prev_pos.x;
                p.prev_pos.x = p.pos.x + vel_x * self.restitution;
            }
            // Y axis (same pattern)
            if p.pos.y < self.min.y {
                p.pos.y = self.min.y;
                let vel_y = p.pos.y - p.prev_pos.y;
                p.prev_pos.y = p.pos.y + vel_y * self.restitution;
            } else if p.pos.y > self.max.y {
                p.pos.y = self.max.y;
                let vel_y = p.pos.y - p.prev_pos.y;
                p.prev_pos.y = p.pos.y + vel_y * self.restitution;
            }
        }
    }
}
```

An alternative design: add `clamp_component` and per-component access methods to the `Vec` trait. This is a design decision — either approach works. The implementer should choose one and be consistent.

#### Step 8: Solver (`src/solver.rs`)

```rust
use crate::float::Float;
use crate::vec::Vec;
use crate::particle::Particle;
use crate::constraint::Constraint;
use crate::config::SolverConfig;
use crate::observer::StepObserver;
use alloc::vec::Vec as AllocVec;

/// Iterative constraint solver for Verlet particle systems.
///
/// Each step:
/// 1. Apply gravity and external forces to all particles
/// 2. Integrate particles (Verlet step)
/// 3. Iteratively solve constraints (N iterations)
/// 4. Notify observer
///
/// Sub-stepping: if `config.sub_steps > 1`, the dt is divided into
/// smaller steps for more accurate simulation.
pub struct ConstraintSolver<V: Vec> {
    /// All particles in the system.
    pub particles: AllocVec<Particle<V>>,
    /// All constraints.
    pub constraints: AllocVec<Constraint<V>>,
}

impl<V: Vec> ConstraintSolver<V> {
    /// Create a new empty solver.
    pub fn new() -> Self {
        ConstraintSolver {
            particles: AllocVec::new(),
            constraints: AllocVec::new(),
        }
    }

    /// Add a particle and return its index.
    pub fn add_particle(&mut self, particle: Particle<V>) -> usize {
        let idx = self.particles.len();
        self.particles.push(particle);
        idx
    }

    /// Add a constraint.
    pub fn add_constraint(&mut self, constraint: Constraint<V>) {
        self.constraints.push(constraint);
    }

    /// Run one simulation step.
    ///
    /// # Arguments
    /// - `dt`: Time step in seconds
    /// - `config`: Solver configuration (iterations, gravity, damping, sub_steps)
    /// - `observer`: Optional observer for monitoring (use `&mut NoOpStepObserver` for none)
    pub fn step<O: StepObserver>(
        &mut self,
        dt: V::Scalar,
        config: &SolverConfig<V>,
        observer: &mut O,
    ) {
        let sub_dt = dt / V::Scalar::from_f32(config.sub_steps as f32);

        for _sub in 0..config.sub_steps {
            // 1. Apply gravity to all particles
            for p in self.particles.iter_mut() {
                p.apply_acceleration(config.gravity);
            }

            // 2. Integrate all particles
            for p in self.particles.iter_mut() {
                p.integrate(sub_dt, config.damping);
            }
            observer.on_integrate();

            // 3. Solve constraints iteratively
            for i in 0..config.iterations {
                for constraint in self.constraints.iter() {
                    constraint.solve(&mut self.particles);
                }
                observer.on_constraint_iteration(i);
            }
        }

        observer.on_step_complete();
    }

    /// Get the number of particles.
    pub fn particle_count(&self) -> usize {
        self.particles.len()
    }

    /// Get the number of constraints.
    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }

    /// Get a reference to a particle by index.
    pub fn particle(&self, index: usize) -> &Particle<V> {
        &self.particles[index]
    }

    /// Get a mutable reference to a particle by index.
    pub fn particle_mut(&mut self, index: usize) -> &mut Particle<V> {
        &mut self.particles[index]
    }

    /// Remove a constraint by index.
    pub fn remove_constraint(&mut self, index: usize) -> Constraint<V> {
        self.constraints.swap_remove(index)
    }

    /// Clear all constraints (particles remain).
    pub fn clear_constraints(&mut self) {
        self.constraints.clear();
    }
}
```

**Tests for Phase 3:**
- **Free-fall**: Single particle with gravity, after N steps position matches analytical s = 0.5*g*t^2
- **Distance constraint**: Two particles connected by distance constraint maintain rest length after solving
- **Pin constraint**: Pinned particle does not move when forces applied
- **Mass weighting**: Heavier particle moves less during distance constraint resolution
- **Multiple iterations**: More iterations produce better constraint satisfaction

---

### Phase 4: Chains (`chain.rs`)

#### Step 9: VerletChain

```rust
use crate::float::Float;
use crate::vec::Vec;
use crate::particle::Particle;
use crate::constraint::{Constraint, DistanceConstraint};
use crate::config::SolverConfig;
use crate::observer::StepObserver;
use alloc::vec::Vec as AllocVec;

/// Configuration for creating a chain.
pub struct ChainConfig<F: Float> {
    /// Stiffness of distance constraints [0, 1].
    pub stiffness: F,
    /// Mass per particle.
    pub particle_mass: F,
}

impl<F: Float> Default for ChainConfig<F> {
    fn default() -> Self {
        ChainConfig {
            stiffness: F::one(),
            particle_mass: F::one(),
        }
    }
}

/// A rope/string built from Verlet particles and distance constraints.
///
/// Particles are connected sequentially with distance constraints:
/// ```text
///   [0] --- [1] --- [2] --- [3] --- ... --- [N]
/// ```
///
/// # Example
/// ```
/// use softy::chain::VerletChain;
/// use softy::vec::Vec2;
/// use softy::config::SolverConfig;
/// use softy::observer::NoOpStepObserver;
///
/// let mut chain: VerletChain<Vec2<f32>> = VerletChain::new(
///     Vec2::new(0.0, 10.0),   // start
///     Vec2::new(10.0, 10.0),  // end
///     10,                      // segments
///     Default::default(),      // chain config
/// );
/// chain.pin(0);  // pin the first particle
///
/// let config = SolverConfig::default();
/// chain.step(1.0 / 60.0, &config, &mut NoOpStepObserver);
/// ```
pub struct VerletChain<V: Vec> {
    particles: AllocVec<Particle<V>>,
    constraints: AllocVec<DistanceConstraint<V>>,
}

impl<V: Vec> VerletChain<V> {
    /// Create a new chain from `start` to `end` with `segments` segments.
    ///
    /// Creates `segments + 1` particles evenly spaced between start and end,
    /// connected by `segments` distance constraints.
    pub fn new(
        start: V,
        end: V,
        segments: usize,
        config: ChainConfig<V::Scalar>,
    ) -> Self {
        let mut particles = AllocVec::with_capacity(segments + 1);
        let mut constraints = AllocVec::with_capacity(segments);

        let segment_length = start.distance(end) / V::Scalar::from_f32(segments as f32);

        for i in 0..=segments {
            let t = V::Scalar::from_f32(i as f32) / V::Scalar::from_f32(segments as f32);
            let pos = start.lerp(end, t);
            particles.push(Particle::new(pos, config.particle_mass));
        }

        for i in 0..segments {
            constraints.push(DistanceConstraint::new(
                i, i + 1,
                segment_length,
                config.stiffness,
            ));
        }

        VerletChain { particles, constraints }
    }

    /// Pin a particle at the given index.
    pub fn pin(&mut self, index: usize) {
        self.particles[index].pin();
    }

    /// Unpin a particle, restoring the given mass.
    pub fn unpin(&mut self, index: usize, mass: V::Scalar) {
        self.particles[index].unpin(mass);
    }

    /// Move a pinned particle to a new position.
    pub fn move_pin(&mut self, index: usize, pos: V) {
        self.particles[index].move_to(pos);
    }

    /// Apply a force to all particles.
    pub fn apply_force(&mut self, force: V) {
        for p in self.particles.iter_mut() {
            p.apply_force(force);
        }
    }

    /// Apply an acceleration to all particles (e.g., gravity).
    pub fn apply_acceleration(&mut self, accel: V) {
        for p in self.particles.iter_mut() {
            p.apply_acceleration(accel);
        }
    }

    /// Apply a force to a specific particle.
    pub fn apply_force_at(&mut self, index: usize, force: V) {
        self.particles[index].apply_force(force);
    }

    /// Run one simulation step.
    pub fn step<O: StepObserver>(
        &mut self,
        dt: V::Scalar,
        config: &SolverConfig<V>,
        observer: &mut O,
    ) {
        let sub_dt = dt / V::Scalar::from_f32(config.sub_steps as f32);

        for _sub in 0..config.sub_steps {
            // Apply gravity
            for p in self.particles.iter_mut() {
                p.apply_acceleration(config.gravity);
            }

            // Integrate
            for p in self.particles.iter_mut() {
                p.integrate(sub_dt, config.damping);
            }
            observer.on_integrate();

            // Solve constraints
            for i in 0..config.iterations {
                for c in self.constraints.iter() {
                    c.solve(&mut self.particles);
                }
                observer.on_constraint_iteration(i);
            }
        }

        observer.on_step_complete();
    }

    /// Get all particle positions as a slice.
    pub fn positions(&self) -> AllocVec<V> {
        self.particles.iter().map(|p| p.pos).collect()
    }

    /// Get the number of particles.
    pub fn len(&self) -> usize {
        self.particles.len()
    }

    /// Check if the chain is empty.
    pub fn is_empty(&self) -> bool {
        self.particles.is_empty()
    }

    /// Get the number of segments (= constraints).
    pub fn segment_count(&self) -> usize {
        self.constraints.len()
    }

    /// Get a reference to a particle.
    pub fn particle(&self, index: usize) -> &Particle<V> {
        &self.particles[index]
    }

    /// Get a mutable reference to a particle.
    pub fn particle_mut(&mut self, index: usize) -> &mut Particle<V> {
        &mut self.particles[index]
    }
}
```

**Tests for Phase 4:**
- Chain has correct number of particles and segments
- Pinned endpoint stays fixed while chain swings under gravity
- `move_pin` moves the pinned particle
- Chain total length approximately equals sum of rest lengths after settling

---

### Phase 5: Grid + Soft Body (`grid.rs` + `softbody.rs`)

#### Step 10: VerletGrid (`src/grid.rs`)

```rust
use crate::float::Float;
use crate::vec::Vec;
use crate::particle::Particle;
use crate::constraint::DistanceConstraint;
use crate::config::SolverConfig;
use crate::observer::StepObserver;
use alloc::vec::Vec as AllocVec;

/// Configuration for a cloth grid.
pub struct GridConfig<F: Float> {
    /// Number of columns.
    pub cols: usize,
    /// Number of rows.
    pub rows: usize,
    /// Spacing between adjacent particles.
    pub spacing: F,
    /// Stiffness for structural constraints (horizontal/vertical neighbors).
    pub structural_stiffness: F,
    /// Stiffness for shear constraints (diagonal neighbors).
    pub shear_stiffness: F,
    /// Stiffness for bend constraints (skip-one neighbors).
    pub bend_stiffness: F,
    /// Mass per particle.
    pub particle_mass: F,
}

/// A cloth mesh built from a grid of Verlet particles.
///
/// Constraint types:
/// - **Structural**: Horizontal and vertical neighbors (rest = spacing)
/// - **Shear**: Diagonal neighbors (rest = spacing * sqrt(2))
/// - **Bend**: Skip-one horizontal/vertical (rest = spacing * 2)
///
/// ```text
///   Grid layout (3x3):
///
///   [0,0] -- [1,0] -- [2,0]     Structural: --
///     |  \  /  |  \  /  |       Shear: \ /
///   [0,1] -- [1,1] -- [2,1]     Bend: connects [0,0]-[2,0], etc.
///     |  \  /  |  \  /  |
///   [0,2] -- [1,2] -- [2,2]
/// ```
pub struct VerletGrid<V: Vec> {
    particles: AllocVec<Particle<V>>,
    constraints: AllocVec<DistanceConstraint<V>>,
    cols: usize,
    rows: usize,
}

impl<V: Vec> VerletGrid<V> {
    /// Create a new grid starting at `origin` with the given configuration.
    ///
    /// The grid extends in the positive X direction (columns) and
    /// positive Y direction (rows) from the origin. For cloth simulation
    /// in 2D, Y typically points downward (gravity direction).
    ///
    /// # Particle indexing
    /// The particle at (col, row) has index `row * cols + col`.
    pub fn new(origin: V, config: &GridConfig<V::Scalar>) -> Self {
        // 1. Create particles in row-major order
        // 2. Create structural constraints (horizontal + vertical)
        // 3. Create shear constraints (diagonals)
        // 4. Create bend constraints (skip-one)
        todo!()
    }

    /// Get the linear index for a (col, row) position.
    pub fn index(&self, col: usize, row: usize) -> usize {
        row * self.cols + col
    }

    /// Pin a particle at (col, row).
    pub fn pin(&mut self, col: usize, row: usize) {
        let idx = self.index(col, row);
        self.particles[idx].pin();
    }

    /// Unpin a particle at (col, row).
    pub fn unpin(&mut self, col: usize, row: usize, mass: V::Scalar) {
        let idx = self.index(col, row);
        self.particles[idx].unpin(mass);
    }

    /// Pin the entire top row.
    pub fn pin_top_row(&mut self) {
        for col in 0..self.cols {
            self.pin(col, 0);
        }
    }

    /// Move a pinned particle to a new position.
    pub fn move_pin(&mut self, col: usize, row: usize, pos: V) {
        let idx = self.index(col, row);
        self.particles[idx].move_to(pos);
    }

    /// Tear the cloth at (col, row) by removing all constraints connected to that particle.
    pub fn tear_at(&mut self, col: usize, row: usize) {
        let idx = self.index(col, row);
        self.constraints.retain(|c| c.a != idx && c.b != idx);
    }

    /// Apply a force to all particles.
    pub fn apply_force(&mut self, force: V) {
        for p in self.particles.iter_mut() {
            p.apply_force(force);
        }
    }

    /// Apply acceleration (e.g., gravity) to all particles.
    pub fn apply_acceleration(&mut self, accel: V) {
        for p in self.particles.iter_mut() {
            p.apply_acceleration(accel);
        }
    }

    /// Run one simulation step.
    pub fn step<O: StepObserver>(
        &mut self,
        dt: V::Scalar,
        config: &SolverConfig<V>,
        observer: &mut O,
    ) {
        // Same pattern as chain: sub-steps, gravity, integrate, solve
        todo!()
    }

    /// Get all particle positions.
    pub fn positions(&self) -> AllocVec<V> {
        self.particles.iter().map(|p| p.pos).collect()
    }

    /// Get particle position at (col, row).
    pub fn position_at(&self, col: usize, row: usize) -> V {
        self.particles[self.index(col, row)].pos
    }

    /// Get grid dimensions.
    pub fn cols(&self) -> usize { self.cols }
    pub fn rows(&self) -> usize { self.rows }

    /// Total number of particles.
    pub fn particle_count(&self) -> usize { self.particles.len() }

    /// Total number of constraints.
    pub fn constraint_count(&self) -> usize { self.constraints.len() }
}
```

#### Step 11: SoftBody (`src/softbody.rs`)

```rust
use crate::float::Float;
use crate::vec::{Vec, Vec2};
use crate::particle::Particle;
use crate::constraint::DistanceConstraint;
use crate::config::SolverConfig;
use crate::observer::StepObserver;
use alloc::vec::Vec as AllocVec;

/// A 2D pressure-based soft body.
///
/// Particles form a closed polygon connected by distance constraints.
/// Pressure forces push particles outward to maintain the body's volume.
///
/// # Pressure Model
/// ```text
/// current_area = shoelace_formula(positions)
/// pressure_ratio = target_area / current_area
/// For each edge (a, b):
///     edge_normal = perp(b - a).normalize()
///     force = edge_normal * pressure * pressure_ratio * edge_length
///     Apply force/2 to a and b
/// ```
pub struct SoftBody<F: Float> {
    particles: AllocVec<Particle<Vec2<F>>>,
    constraints: AllocVec<DistanceConstraint<Vec2<F>>>,
    /// Target area for pressure calculation.
    target_area: F,
    /// Pressure coefficient — higher = more rigid/inflated.
    pressure: F,
}

impl<F: Float> SoftBody<F> {
    /// Create a circular soft body.
    ///
    /// # Arguments
    /// - `center`: Center position
    /// - `radius`: Radius of the circle
    /// - `segments`: Number of edge segments (more = smoother)
    /// - `pressure`: Pressure coefficient
    /// - `stiffness`: Distance constraint stiffness
    pub fn circle(
        center: Vec2<F>,
        radius: F,
        segments: usize,
        pressure: F,
        stiffness: F,
    ) -> Self {
        // Create particles around a circle
        // Connect sequential particles with distance constraints
        // Also connect opposite particles for structural integrity (optional cross-constraints)
        // Calculate initial area as target_area
        todo!()
    }

    /// Create a rectangular soft body.
    ///
    /// # Arguments
    /// - `center`: Center position
    /// - `width`: Rectangle width
    /// - `height`: Rectangle height
    /// - `segments_per_side`: Particles per side (minimum 2)
    /// - `pressure`: Pressure coefficient
    /// - `stiffness`: Distance constraint stiffness
    pub fn rectangle(
        center: Vec2<F>,
        width: F,
        height: F,
        segments_per_side: usize,
        pressure: F,
        stiffness: F,
    ) -> Self {
        todo!()
    }

    /// Apply a force to all particles.
    pub fn apply_force(&mut self, force: Vec2<F>) {
        for p in self.particles.iter_mut() {
            p.apply_force(force);
        }
    }

    /// Apply an impulse at the nearest particle to `point`.
    pub fn poke(&mut self, point: Vec2<F>, impulse: Vec2<F>) {
        // Find nearest particle, apply impulse (adjust prev_pos)
        todo!()
    }

    /// Run one simulation step.
    pub fn step<O: StepObserver>(
        &mut self,
        dt: F,
        config: &SolverConfig<Vec2<F>>,
        observer: &mut O,
    ) {
        // 1. Calculate current area
        // 2. Calculate pressure forces and apply to particles
        // 3. Apply gravity
        // 4. Integrate
        // 5. Solve constraints
        todo!()
    }

    /// Calculate the current area using the shoelace formula.
    ///
    /// ```text
    /// area = 0.5 * |sum(x[i]*y[i+1] - x[i+1]*y[i])|
    /// ```
    pub fn area(&self) -> F {
        let n = self.particles.len();
        if n < 3 {
            return F::zero();
        }
        let mut sum = F::zero();
        for i in 0..n {
            let j = (i + 1) % n;
            let a = self.particles[i].pos;
            let b = self.particles[j].pos;
            sum = sum + (a.x * b.y - b.x * a.y);
        }
        (sum * F::half()).abs()
    }

    /// Check if a point is inside the soft body using ray casting.
    pub fn contains(&self, point: Vec2<F>) -> bool {
        // Ray casting: count intersections of horizontal ray from point
        // with polygon edges. Odd count = inside.
        todo!()
    }

    /// Get the centroid (average position) of the soft body.
    pub fn centroid(&self) -> Vec2<F> {
        let n = F::from_f32(self.particles.len() as f32);
        let mut sum = Vec2::zero();
        for p in &self.particles {
            sum = sum + p.pos;
        }
        sum.scale(F::one() / n)
    }

    /// Get all particle positions.
    pub fn positions(&self) -> AllocVec<Vec2<F>> {
        self.particles.iter().map(|p| p.pos).collect()
    }

    /// Get the number of particles.
    pub fn particle_count(&self) -> usize {
        self.particles.len()
    }
}
```

**Tests for Phase 5:**
- Grid creates correct number of particles: cols * rows
- Grid creates structural constraints: (cols-1)*rows + cols*(rows-1)
- Pinned top row holds while grid drapes under gravity
- `tear_at` removes constraints, causing separation
- SoftBody circle `area()` approximately equals pi*r^2 initially
- SoftBody pressure maintains area under gravity
- `contains` correctly identifies interior/exterior points

---

### Phase 6: Infrastructure (`observer.rs` + `config.rs` + `error.rs`)

#### Step 12: Observer (`src/observer.rs`)

```rust
/// Trait for observing physics simulation steps.
///
/// Implement this trait to monitor solver progress (e.g., for debugging,
/// visualization, or performance profiling). All methods have default
/// no-op implementations.
pub trait StepObserver {
    /// Called after all particles have been integrated (Verlet step).
    fn on_integrate(&mut self) {}

    /// Called after each constraint iteration.
    fn on_constraint_iteration(&mut self, _iteration: usize) {}

    /// Called when a simulation step is fully complete.
    fn on_step_complete(&mut self) {}
}

/// A no-op observer that does nothing. Use as default when no observation needed.
pub struct NoOpStepObserver;

impl StepObserver for NoOpStepObserver {}
```

#### Step 13: Config (`src/config.rs`)

```rust
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
        self.sub_steps = sub_steps;
        self
    }
}

impl<V: Vec> Default for SolverConfig<V> {
    fn default() -> Self {
        Self::new()
    }
}
```

#### Step 14: Error (`src/error.rs`)

```rust
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
```

---

### Phase 7: Wire Up `lib.rs`

Add re-exports to `src/lib.rs` after the module declarations:

```rust
// Re-export primary API
pub use float::Float;
pub use vec::{Vec, Scalar, Vec2, Vec3};
pub use spring::{Spring, Spring1D, Spring2D, Spring3D, DampingMode};
pub use particle::Particle;
pub use constraint::{Constraint, DistanceConstraint, PinConstraint, AngleConstraint, BoundsConstraint};
pub use solver::ConstraintSolver;
pub use chain::{VerletChain, ChainConfig};
pub use grid::{VerletGrid, GridConfig};
pub use softbody::SoftBody;
pub use config::SolverConfig;
pub use observer::{StepObserver, NoOpStepObserver};
pub use error::PhysicsError;
```

---

### Phase 8: Tests

Write these test files:

#### `tests/spring_tests.rs`

```rust
use softy::{Spring, Spring1D, Spring2D, DampingMode, Vec2, Scalar};

#[test]
fn critically_damped_converges() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(10.0), 4.0,
    );
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
    }
    assert!((spring.value().0 - 10.0).abs() < 0.001);
}

#[test]
fn critically_damped_no_overshoot() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(10.0), 4.0,
    );
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
        assert!(spring.value().0 <= 10.001, "Overshoot detected: {}", spring.value().0);
    }
}

#[test]
fn underdamped_oscillates() {
    let mut spring: Spring1D<f32> = Spring::underdamped(
        Scalar(0.0), Scalar(10.0), 4.0, 0.2,
    );
    let mut crossed = false;
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
        if spring.value().0 > 10.0 {
            crossed = true;
            break;
        }
    }
    assert!(crossed, "Underdamped spring should overshoot target");
}

#[test]
fn overdamped_slower_than_critical() {
    let mut critical: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(10.0), 4.0,
    );
    let mut over: Spring1D<f32> = Spring::overdamped(
        Scalar(0.0), Scalar(10.0), 4.0, 2.0,
    );
    for _ in 0..30 {
        critical.update(1.0 / 60.0);
        over.update(1.0 / 60.0);
    }
    // After same time, critical should be closer to target
    let critical_dist = (critical.value().0 - 10.0).abs();
    let over_dist = (over.value().0 - 10.0).abs();
    assert!(critical_dist < over_dist);
}

#[test]
fn spring_2d_converges() {
    let mut spring: Spring2D<f32> = Spring::critically_damped(
        Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0), 4.0,
    );
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
    }
    let v = spring.value();
    assert!((v.x - 5.0).abs() < 0.001);
    assert!((v.y - 5.0).abs() < 0.001);
}

#[test]
fn is_settled_works() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(1.0), 8.0,
    );
    assert!(!spring.is_settled(0.01, 0.01));
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
    }
    assert!(spring.is_settled(0.01, 0.01));
}

#[test]
fn set_target_mid_simulation() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(5.0), 4.0,
    );
    for _ in 0..100 {
        spring.update(1.0 / 60.0);
    }
    spring.set_target(Scalar(-5.0));
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
    }
    assert!((spring.value().0 - (-5.0)).abs() < 0.001);
}

#[test]
fn zero_dt_no_change() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(3.0), Scalar(10.0), 4.0,
    );
    let before = spring.value();
    spring.update(0.0);
    assert_eq!(spring.value(), before);
}

#[test]
fn determinism() {
    for _ in 0..10 {
        let mut spring: Spring2D<f32> = Spring::underdamped(
            Vec2::new(1.0, 2.0), Vec2::new(10.0, -5.0), 3.0, 0.3,
        );
        for _ in 0..500 {
            spring.update(1.0 / 60.0);
        }
        let v = spring.value();
        // These should be exactly the same every run (closed-form, deterministic)
        assert_eq!(v.x, v.x); // NaN check
        // Store first run result and compare (within the same test)
    }
}
```

#### `tests/verlet_tests.rs`

```rust
use softy::{Particle, Vec2, ConstraintSolver, SolverConfig, Constraint, DistanceConstraint, NoOpStepObserver};

#[test]
fn free_fall_gravity() {
    let mut p: Particle<Vec2<f32>> = Particle::new(Vec2::new(0.0, 100.0), 1.0);
    let g = Vec2::new(0.0, -9.81);
    let dt = 1.0 / 60.0;
    let steps = 60; // 1 second

    for _ in 0..steps {
        p.apply_acceleration(g);
        p.integrate(dt, 1.0); // no damping
    }

    // After 1 second: y ≈ 100 - 0.5 * 9.81 * 1^2 ≈ 95.095
    // Verlet accumulates slightly differently but should be close
    let expected_y = 100.0 - 0.5 * 9.81 * 1.0;
    assert!((p.pos.y - expected_y).abs() < 1.0, "pos.y = {}, expected ≈ {}", p.pos.y, expected_y);
}

#[test]
fn distance_constraint_maintains_length() {
    let mut solver: ConstraintSolver<Vec2<f32>> = ConstraintSolver::new();
    solver.add_particle(Particle::new(Vec2::new(0.0, 0.0), 1.0));
    solver.add_particle(Particle::new(Vec2::new(5.0, 0.0), 1.0));
    solver.add_constraint(Constraint::Distance(
        DistanceConstraint::new(0, 1, 5.0, 1.0),
    ));

    // Move particle 1 far away
    solver.particle_mut(1).pos = Vec2::new(20.0, 0.0);

    let config = SolverConfig::new().with_iterations(10);
    solver.step(1.0 / 60.0, &config, &mut NoOpStepObserver);

    let dist = solver.particle(0).pos.distance(solver.particle(1).pos);
    assert!((dist - 5.0).abs() < 0.5, "Distance should be near 5.0, got {}", dist);
}

#[test]
fn pinned_particle_stays_fixed() {
    let mut p: Particle<Vec2<f32>> = Particle::pinned(Vec2::new(5.0, 5.0));
    p.apply_force(Vec2::new(1000.0, 1000.0));
    p.integrate(1.0 / 60.0, 1.0);
    assert_eq!(p.pos.x, 5.0);
    assert_eq!(p.pos.y, 5.0);
}
```

#### `tests/chain_tests.rs`

```rust
use softy::{VerletChain, ChainConfig, SolverConfig, Vec2, NoOpStepObserver};

#[test]
fn chain_correct_particle_count() {
    let chain: VerletChain<Vec2<f32>> = VerletChain::new(
        Vec2::new(0.0, 0.0),
        Vec2::new(10.0, 0.0),
        10,
        ChainConfig::default(),
    );
    assert_eq!(chain.len(), 11); // segments + 1
    assert_eq!(chain.segment_count(), 10);
}

#[test]
fn chain_swings_under_gravity() {
    let mut chain: VerletChain<Vec2<f32>> = VerletChain::new(
        Vec2::new(0.0, 10.0),
        Vec2::new(10.0, 10.0),
        10,
        ChainConfig::default(),
    );
    chain.pin(0); // pin start

    let config = SolverConfig::new()
        .with_gravity(Vec2::new(0.0, -9.81))
        .with_iterations(8)
        .with_sub_steps(2);

    for _ in 0..120 {
        chain.step(1.0 / 60.0, &config, &mut NoOpStepObserver);
    }

    // The end should have dropped below the start
    let start_y = chain.particle(0).pos.y;
    let end_y = chain.particle(chain.len() - 1).pos.y;
    assert!(end_y < start_y, "End should drop below pinned start");
}
```

#### `tests/determinism.rs`

```rust
use softy::{Spring, Spring2D, Vec2, VerletChain, ChainConfig, SolverConfig, NoOpStepObserver};

#[test]
fn spring_deterministic() {
    let results: Vec<_> = (0..10).map(|_| {
        let mut spring: Spring2D<f32> = Spring::underdamped(
            Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0), 3.0, 0.3,
        );
        for _ in 0..500 {
            spring.update(1.0 / 60.0);
        }
        spring.value()
    }).collect();

    for r in &results[1..] {
        assert_eq!(results[0].x, r.x);
        assert_eq!(results[0].y, r.y);
    }
}

#[test]
fn chain_deterministic() {
    let results: Vec<_> = (0..5).map(|_| {
        let mut chain: VerletChain<Vec2<f32>> = VerletChain::new(
            Vec2::new(0.0, 10.0), Vec2::new(10.0, 10.0), 10, ChainConfig::default(),
        );
        chain.pin(0);
        let config = SolverConfig::new()
            .with_gravity(Vec2::new(0.0, -9.81))
            .with_iterations(8);
        for _ in 0..60 {
            chain.step(1.0 / 60.0, &config, &mut NoOpStepObserver);
        }
        chain.positions()
    }).collect();

    for r in &results[1..] {
        for (a, b) in results[0].iter().zip(r.iter()) {
            assert_eq!(a.x, b.x);
            assert_eq!(a.y, b.y);
        }
    }
}
```

---

### Phase 9: WASM Demo

The demo should have 4 tabs, each showcasing a different feature.

#### Tab 1: Springs
- Three balls on screen, each following the mouse cursor
- Left ball: critically damped (smooth follow)
- Middle ball: underdamped (bouncy follow)
- Right ball: overdamped (sluggish follow)
- Sliders: frequency, damping ratio
- Visual: draw trail lines showing path history

#### Tab 2: Rope
- A Verlet chain with the top pinned
- Click and drag any particle
- Gravity pulls the rope down
- Sliders: segment count, stiffness, gravity strength, solver iterations

#### Tab 3: Cloth
- A VerletGrid pinned at top corners
- Click and drag any pin point
- Click to tear (remove constraints at click point)
- Toggle button for wind (horizontal force that varies sinusoidally)
- Sliders: grid size, stiffness, gravity, wind strength

#### Tab 4: Soft Body
- Several SoftBody circles bouncing in a box
- Click to poke (apply impulse away from click point)
- Gravity pulls them down
- Walls contain them (bounds constraint)
- Sliders: pressure, stiffness, gravity

**WASM bindings pattern** (in `demo-wasm/src/lib.rs`):
- Export structs wrapping the softy types
- Export update/render functions called from JS requestAnimationFrame loop
- Use `web-sys` for canvas 2D rendering, or return position data to JS for rendering

**JS pattern** (in `demo-wasm/www/main.js`):
- ES6 module importing the wasm package
- `requestAnimationFrame` loop calling wasm `step()` + JS `render()`
- Tab switching hides/shows relevant controls
- Canvas 2D rendering: lines for rope/cloth, circles for particles/soft bodies

---

### Phase 10: Benchmarks

Replace the placeholder in `benches/physics.rs`:

```rust
use criterion::{criterion_group, criterion_main, Criterion};
use softy::*;

fn bench_spring_update(c: &mut Criterion) {
    c.bench_function("spring_critically_damped_1000_steps", |b| {
        b.iter(|| {
            let mut spring: Spring2D<f32> = Spring::critically_damped(
                Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0), 4.0,
            );
            for _ in 0..1000 {
                spring.update(1.0 / 60.0);
            }
            spring.value()
        });
    });
}

fn bench_chain_simulation(c: &mut Criterion) {
    c.bench_function("chain_50_segments_60_steps", |b| {
        b.iter(|| {
            let mut chain: VerletChain<Vec2<f32>> = VerletChain::new(
                Vec2::new(0.0, 10.0), Vec2::new(10.0, 10.0), 50, ChainConfig::default(),
            );
            chain.pin(0);
            let config = SolverConfig::new()
                .with_gravity(Vec2::new(0.0, -9.81))
                .with_iterations(8);
            for _ in 0..60 {
                chain.step(1.0 / 60.0, &config, &mut NoOpStepObserver);
            }
            chain.positions()
        });
    });
}

fn bench_grid_simulation(c: &mut Criterion) {
    c.bench_function("grid_20x20_cloth_60_steps", |b| {
        b.iter(|| {
            // Create 20x20 grid, pin top row, simulate 60 steps
            // ... (requires VerletGrid to be implemented)
        });
    });
}

criterion_group!(benches, bench_spring_update, bench_chain_simulation, bench_grid_simulation);
criterion_main!(benches);
```

---

## Algorithm References

### Verlet Integration

```text
Stormer-Verlet (position form):
    new_pos = pos + (pos - prev_pos) * damping + acceleration * dt^2
    prev_pos = pos
    pos = new_pos

Velocity extraction:
    velocity = (pos - prev_pos) / dt

Properties:
    - Second-order accurate (unlike Euler which is first-order)
    - Implicit velocity = natural damping
    - Position-based = trivial constraint solving (just move positions)
    - Time-reversible (without damping)
```

### Constraint Projection: Distance

```text
Given particles A and B with rest_length L and stiffness k:
    delta = B.pos - A.pos
    dist = |delta|
    if dist == 0: skip (degenerate case)

    error = dist - L
    direction = delta / dist  (unit vector from A to B)
    correction = direction * error * k

    // Mass-weighted distribution
    w_total = A.inv_mass + B.inv_mass
    if w_total == 0: skip (both pinned)

    A.pos += correction * (A.inv_mass / w_total)
    B.pos -= correction * (B.inv_mass / w_total)
```

### Constraint Projection: Pin

```text
Given particle P and pin position T with stiffness k:
    correction = T - P.pos
    P.pos += correction * k
```

### Constraint Projection: Angle (2D)

```text
Given particles A, B, C where angle is at B:
    ba = A.pos - B.pos
    bc = C.pos - B.pos
    current_angle = atan2(cross(ba, bc), dot(ba, bc))
    error = current_angle - target_angle

    // Normalize error to [-pi, pi]
    rotation = error * stiffness * 0.5

    // Rotate A around B by -rotation
    // Rotate C around B by +rotation
    // 2D rotation: x' = x*cos(a) - y*sin(a), y' = x*sin(a) + y*cos(a)
```

### Constraint Projection: Bounds (2D)

```text
For each particle P:
    For each axis i (x, y):
        if P.pos[i] < min[i]:
            P.pos[i] = min[i]
            // Reflect velocity for bounce
            vel_i = P.pos[i] - P.prev_pos[i]
            P.prev_pos[i] = P.pos[i] + vel_i * restitution
        if P.pos[i] > max[i]:
            P.pos[i] = max[i]
            vel_i = P.pos[i] - P.prev_pos[i]
            P.prev_pos[i] = P.pos[i] + vel_i * restitution
```

### Spring Closed-Form Solutions

#### Critically Damped (zeta = 1)

```text
Characteristic equation: s^2 + 2*w*s + w^2 = 0
Repeated root: s = -w

General solution (relative to target):
    x(t) = (C1 + C2*t) * e^(-w*t)

With x(0) = x0, x'(0) = v0:
    C1 = x0
    C2 = v0 + w*x0

Position: x(t) = (x0 + (v0 + w*x0)*t) * e^(-w*t)
Velocity: v(t) = (v0 - (v0 + w*x0)*w*t) * e^(-w*t)
```

#### Underdamped (0 < zeta < 1)

```text
Characteristic equation: s^2 + 2*zeta*w*s + w^2 = 0
Roots: s = -zeta*w +/- j*wd  where wd = w*sqrt(1 - zeta^2)

General solution (relative to target):
    x(t) = e^(-zeta*w*t) * (A*cos(wd*t) + B*sin(wd*t))

With x(0) = x0, x'(0) = v0:
    A = x0
    B = (v0 + zeta*w*x0) / wd

Position: x(t) = e^(-zeta*w*t) * (A*cos(wd*t) + B*sin(wd*t))
Velocity: v(t) = e^(-zeta*w*t) * (
    (B*wd - A*zeta*w)*cos(wd*t)
  - (A*wd + B*zeta*w)*sin(wd*t)
)
```

#### Overdamped (zeta > 1)

```text
Characteristic equation: s^2 + 2*zeta*w*s + w^2 = 0
Roots: r1 = -w*(zeta - s), r2 = -w*(zeta + s)  where s = sqrt(zeta^2 - 1)

General solution (relative to target):
    x(t) = C1*e^(r1*t) + C2*e^(r2*t)

With x(0) = x0, x'(0) = v0:
    C1 = (r2*x0 - v0) / (r2 - r1)
    C2 = (v0 - r1*x0) / (r2 - r1)

Position: x(t) = C1*e^(r1*t) + C2*e^(r2*t)
Velocity: v(t) = C1*r1*e^(r1*t) + C2*r2*e^(r2*t)
```

### Pressure Soft Body (2D)

```text
1. Calculate area using shoelace formula:
    area = 0.5 * |sum(x[i]*y[i+1] - x[i+1]*y[i])|

2. Calculate pressure ratio:
    ratio = target_area / current_area

3. For each edge (i, j = (i+1) % N):
    edge = particles[j].pos - particles[i].pos
    edge_length = |edge|
    normal = perp(edge).normalize()  // outward-pointing normal

    // Force proportional to pressure * ratio * edge_length
    force = normal * pressure * ratio * edge_length

    // Apply half to each endpoint
    particles[i].apply_force(force * 0.5)
    particles[j].apply_force(force * 0.5)

Note: The normal direction matters — it must point outward. For a
counter-clockwise polygon, the outward normal of edge (a, b) is:
    perp(b - a) = Vec2(-(b.y - a.y), b.x - a.x)
For clockwise, negate.
```

---

## Dependency Note

The `libm` crate must be added to `Cargo.toml` for `no_std` math:

```toml
[dependencies]
rand_core = { version = "0.6", default-features = false }
libm = "0.2"
```

This is the ONLY additional dependency beyond `rand_core`. The `libm` crate is pure Rust, `no_std`, and provides `sqrtf`, `sinf`, `cosf`, `expf`, `fabsf`, `atan2f`, `floorf`, `ceilf` (and their `f64` counterparts).

---

## Verification Checklist

```bash
# In softy/
cargo test --target x86_64-pc-windows-msvc
cargo build --target wasm32-unknown-unknown --release
cargo bench

# WASM demo
cd demo-wasm
wasm-pack build --target web --release
# Serve demo-wasm/www/ with a local HTTP server and test in browser
```
