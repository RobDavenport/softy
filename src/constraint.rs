//! Physics constraints for Verlet integration: distance, pin, angle, bounds.

use crate::float::Float;
use crate::vec::Vec;
use crate::vec::{Scalar, Vec2, Vec3};
use crate::particle::Particle;

/// Trait for dispatching type-specific constraint solving.
///
/// AngleConstraint and BoundsConstraint require per-component access,
/// which the generic `Vec` trait doesn't expose. This trait provides
/// concrete dispatch for each vector type.
///
/// Implemented for `Scalar<F>`, `Vec2<F>`, and `Vec3<F>`.
pub trait ConstraintDispatch: Vec + Sized {
    fn dispatch_angle(c: &AngleConstraint<Self>, particles: &mut [Particle<Self>]);
    fn dispatch_bounds(c: &BoundsConstraint<Self>, particles: &mut [Particle<Self>]);
}

/// A constraint that can be applied to a set of particles.
pub enum Constraint<V: Vec> {
    Distance(DistanceConstraint<V>),
    Pin(PinConstraint<V>),
    Angle(AngleConstraint<V>),
    Bounds(BoundsConstraint<V>),
}

pub struct DistanceConstraint<V: Vec> {
    pub a: usize,
    pub b: usize,
    pub rest_length: V::Scalar,
    pub stiffness: V::Scalar,
}

pub struct PinConstraint<V: Vec> {
    pub particle: usize,
    pub position: V,
    pub stiffness: V::Scalar,
}

pub struct AngleConstraint<V: Vec> {
    pub a: usize,
    pub b: usize,
    pub c: usize,
    pub target_angle: V::Scalar,
    pub stiffness: V::Scalar,
}

pub struct BoundsConstraint<V: Vec> {
    pub min: V,
    pub max: V,
    pub restitution: V::Scalar,
}

impl<V: ConstraintDispatch> Constraint<V> {
    pub fn solve(&self, particles: &mut [Particle<V>]) {
        match self {
            Constraint::Distance(c) => c.solve(particles),
            Constraint::Pin(c) => c.solve(particles),
            Constraint::Angle(c) => V::dispatch_angle(c, particles),
            Constraint::Bounds(c) => V::dispatch_bounds(c, particles),
        }
    }
}

impl<V: Vec> DistanceConstraint<V> {
    pub fn new(a: usize, b: usize, rest_length: V::Scalar, stiffness: V::Scalar) -> Self {
        DistanceConstraint { a, b, rest_length, stiffness }
    }

    pub fn from_particles(a: usize, b: usize, particles: &[Particle<V>], stiffness: V::Scalar) -> Self {
        let rest_length = particles[a].pos.distance(particles[b].pos);
        DistanceConstraint { a, b, rest_length, stiffness }
    }

    pub fn solve(&self, particles: &mut [Particle<V>]) {
        let a_pos = particles[self.a].pos;
        let b_pos = particles[self.b].pos;
        let a_inv = particles[self.a].inv_mass;
        let b_inv = particles[self.b].inv_mass;

        let w_total = a_inv + b_inv;
        if w_total.is_near_zero(V::Scalar::from_f32(1e-10)) {
            return; // both pinned
        }

        let delta = b_pos - a_pos;
        let dist = delta.length();
        if dist.is_near_zero(V::Scalar::from_f32(1e-10)) {
            return; // degenerate
        }

        let error = dist - self.rest_length;
        let correction = delta.scale(error * self.stiffness / dist);

        if !particles[self.a].pinned {
            particles[self.a].pos = particles[self.a].pos + correction.scale(a_inv / w_total);
        }
        if !particles[self.b].pinned {
            particles[self.b].pos = particles[self.b].pos - correction.scale(b_inv / w_total);
        }
    }
}

impl<V: Vec> PinConstraint<V> {
    pub fn new(particle: usize, position: V, stiffness: V::Scalar) -> Self {
        PinConstraint { particle, position, stiffness }
    }

    pub fn solve(&self, particles: &mut [Particle<V>]) {
        let correction = self.position - particles[self.particle].pos;
        particles[self.particle].pos = particles[self.particle].pos + correction.scale(self.stiffness);
    }
}

impl<V: Vec> AngleConstraint<V> {
    pub fn new(a: usize, b: usize, c: usize, target_angle: V::Scalar, stiffness: V::Scalar) -> Self {
        AngleConstraint { a, b, c, target_angle, stiffness }
    }
}

impl<F: Float> AngleConstraint<Vec2<F>> {
    pub fn solve_2d(&self, particles: &mut [Particle<Vec2<F>>]) {
        let ba = particles[self.a].pos - particles[self.b].pos;
        let bc = particles[self.c].pos - particles[self.b].pos;

        let current_angle = F::atan2(ba.cross(bc), ba.dot(bc));
        let error = current_angle - self.target_angle;
        let rotation = error * self.stiffness * F::half();

        let cos_r = rotation.cos();
        let sin_r = rotation.sin();

        // Rotate A around B by -rotation
        if !particles[self.a].pinned {
            let rel = particles[self.a].pos - particles[self.b].pos;
            let rotated = Vec2::new(
                rel.x * cos_r + rel.y * sin_r,
                -rel.x * sin_r + rel.y * cos_r,
            );
            particles[self.a].pos = particles[self.b].pos + rotated;
        }

        // Rotate C around B by +rotation
        if !particles[self.c].pinned {
            let rel = particles[self.c].pos - particles[self.b].pos;
            let rotated = Vec2::new(
                rel.x * cos_r - rel.y * sin_r,
                rel.x * sin_r + rel.y * cos_r,
            );
            particles[self.c].pos = particles[self.b].pos + rotated;
        }
    }
}

impl<V: Vec> BoundsConstraint<V> {
    pub fn new(min: V, max: V, restitution: V::Scalar) -> Self {
        BoundsConstraint { min, max, restitution }
    }
}

impl<F: Float> BoundsConstraint<Vec2<F>> {
    pub fn solve_2d(&self, particles: &mut [Particle<Vec2<F>>]) {
        for p in particles.iter_mut() {
            if p.pinned { continue; }
            if p.pos.x < self.min.x {
                p.pos.x = self.min.x;
                let vel_x = p.pos.x - p.prev_pos.x;
                p.prev_pos.x = p.pos.x + vel_x * self.restitution;
            } else if p.pos.x > self.max.x {
                p.pos.x = self.max.x;
                let vel_x = p.pos.x - p.prev_pos.x;
                p.prev_pos.x = p.pos.x + vel_x * self.restitution;
            }
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

impl<F: Float> BoundsConstraint<Vec3<F>> {
    pub fn solve_3d(&self, particles: &mut [Particle<Vec3<F>>]) {
        for p in particles.iter_mut() {
            if p.pinned { continue; }
            if p.pos.x < self.min.x {
                p.pos.x = self.min.x;
                let vel_x = p.pos.x - p.prev_pos.x;
                p.prev_pos.x = p.pos.x + vel_x * self.restitution;
            } else if p.pos.x > self.max.x {
                p.pos.x = self.max.x;
                let vel_x = p.pos.x - p.prev_pos.x;
                p.prev_pos.x = p.pos.x + vel_x * self.restitution;
            }
            if p.pos.y < self.min.y {
                p.pos.y = self.min.y;
                let vel_y = p.pos.y - p.prev_pos.y;
                p.prev_pos.y = p.pos.y + vel_y * self.restitution;
            } else if p.pos.y > self.max.y {
                p.pos.y = self.max.y;
                let vel_y = p.pos.y - p.prev_pos.y;
                p.prev_pos.y = p.pos.y + vel_y * self.restitution;
            }
            if p.pos.z < self.min.z {
                p.pos.z = self.min.z;
                let vel_z = p.pos.z - p.prev_pos.z;
                p.prev_pos.z = p.pos.z + vel_z * self.restitution;
            } else if p.pos.z > self.max.z {
                p.pos.z = self.max.z;
                let vel_z = p.pos.z - p.prev_pos.z;
                p.prev_pos.z = p.pos.z + vel_z * self.restitution;
            }
        }
    }
}

// ConstraintDispatch implementations for all shipped Vec types.

impl<F: Float> ConstraintDispatch for Scalar<F> {
    fn dispatch_angle(_c: &AngleConstraint<Self>, _particles: &mut [Particle<Self>]) {
        // Angle constraints are not meaningful in 1D.
    }
    fn dispatch_bounds(_c: &BoundsConstraint<Self>, _particles: &mut [Particle<Self>]) {
        // 1D bounds would need per-component access on Scalar; no-op for now.
    }
}

impl<F: Float> ConstraintDispatch for Vec2<F> {
    fn dispatch_angle(c: &AngleConstraint<Self>, particles: &mut [Particle<Self>]) {
        c.solve_2d(particles);
    }
    fn dispatch_bounds(c: &BoundsConstraint<Self>, particles: &mut [Particle<Self>]) {
        c.solve_2d(particles);
    }
}

impl<F: Float> ConstraintDispatch for Vec3<F> {
    fn dispatch_angle(_c: &AngleConstraint<Self>, _particles: &mut [Particle<Self>]) {
        // Angle constraints are only meaningful in 2D.
    }
    fn dispatch_bounds(c: &BoundsConstraint<Self>, particles: &mut [Particle<Self>]) {
        c.solve_3d(particles);
    }
}
