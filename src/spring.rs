//! Analytical spring-damper systems with closed-form solutions.

use crate::float::Float;
use crate::vec::Vec;
use crate::vec::{Scalar, Vec2, Vec3};

/// Damping mode for a spring-damper system.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DampingMode<F: Float> {
    CriticallyDamped,
    Underdamped { ratio: F },
    Overdamped { ratio: F },
}

/// Analytical spring-damper system with closed-form solution.
pub struct Spring<V: Vec> {
    current: V,
    velocity: V,
    target: V,
    angular_freq: V::Scalar,
    mode: DampingMode<V::Scalar>,
}

impl<V: Vec> Spring<V> {
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

    pub fn underdamped(initial: V, target: V, frequency_hz: V::Scalar, damping_ratio: V::Scalar) -> Self {
        debug_assert!(damping_ratio > V::Scalar::zero() && damping_ratio < V::Scalar::one(),
            "underdamped requires damping_ratio in (0.0, 1.0)");
        let two_pi = V::Scalar::two() * V::Scalar::pi();
        Spring {
            current: initial,
            velocity: V::zero(),
            target,
            angular_freq: two_pi * frequency_hz,
            mode: DampingMode::Underdamped { ratio: damping_ratio },
        }
    }

    pub fn overdamped(initial: V, target: V, frequency_hz: V::Scalar, damping_ratio: V::Scalar) -> Self {
        debug_assert!(damping_ratio > V::Scalar::one(),
            "overdamped requires damping_ratio > 1.0");
        let two_pi = V::Scalar::two() * V::Scalar::pi();
        Spring {
            current: initial,
            velocity: V::zero(),
            target,
            angular_freq: two_pi * frequency_hz,
            mode: DampingMode::Overdamped { ratio: damping_ratio },
        }
    }

    pub fn update(&mut self, dt: V::Scalar) {
        let x0 = self.current - self.target;
        let v0 = self.velocity;
        let w = self.angular_freq;

        let (new_offset, new_velocity) = match self.mode {
            DampingMode::CriticallyDamped => {
                let exp_term = (-w * dt).exp();
                let c2 = v0 + x0.scale(w);
                let new_x = (x0 + c2.scale(dt)).scale(exp_term);
                let new_v = (v0 - c2.scale(w * dt)).scale(exp_term);
                (new_x, new_v)
            }
            DampingMode::Underdamped { ratio: zeta } => {
                let one = V::Scalar::one();
                let wd = w * (one - zeta * zeta).sqrt();
                let exp_term = (-zeta * w * dt).exp();
                let cos_term = (wd * dt).cos();
                let sin_term = (wd * dt).sin();

                let a = x0;
                let b = (v0 + x0.scale(zeta * w)).scale(one / wd);

                let new_x = (a.scale(cos_term) + b.scale(sin_term)).scale(exp_term);

                let v_cos_coeff = b.scale(wd) - a.scale(zeta * w);
                let v_sin_coeff = a.scale(wd) + b.scale(zeta * w);
                let new_v = (v_cos_coeff.scale(cos_term) - v_sin_coeff.scale(sin_term))
                    .scale(exp_term);

                (new_x, new_v)
            }
            DampingMode::Overdamped { ratio: zeta } => {
                let one = V::Scalar::one();
                let s = (zeta * zeta - one).sqrt();
                let r1 = -w * (zeta - s);
                let r2 = -w * (zeta + s);
                let denom = one / (r2 - r1);

                let c1 = (x0.scale(r2) - v0).scale(denom);
                let c2 = (v0 - x0.scale(r1)).scale(denom);

                let exp1 = (r1 * dt).exp();
                let exp2 = (r2 * dt).exp();

                let new_x = c1.scale(exp1) + c2.scale(exp2);
                let new_v = c1.scale(r1 * exp1) + c2.scale(r2 * exp2);

                (new_x, new_v)
            }
        };

        self.current = self.target + new_offset;
        self.velocity = new_velocity;
    }

    pub fn set_target(&mut self, target: V) { self.target = target; }
    pub fn value(&self) -> V { self.current }
    pub fn velocity(&self) -> V { self.velocity }

    pub fn is_settled(&self, eps_pos: V::Scalar, eps_vel: V::Scalar) -> bool {
        let offset = self.current - self.target;
        offset.length_sq() < eps_pos * eps_pos
            && self.velocity.length_sq() < eps_vel * eps_vel
    }

    pub fn angular_freq(&self) -> V::Scalar { self.angular_freq }
    pub fn mode(&self) -> &DampingMode<V::Scalar> { &self.mode }

    pub fn reset(&mut self, position: V) {
        self.current = position;
        self.velocity = V::zero();
    }

    pub fn snap_to_target(&mut self) {
        self.current = self.target;
        self.velocity = V::zero();
    }
}

pub type Spring1D<F> = Spring<Scalar<F>>;
pub type Spring2D<F> = Spring<Vec2<F>>;
pub type Spring3D<F> = Spring<Vec3<F>>;
