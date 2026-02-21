//! 2D pressure-based soft body simulation.

use crate::float::Float;
use crate::vec::{Vec, Vec2};
use crate::particle::Particle;
use crate::constraint::DistanceConstraint;
use crate::config::SolverConfig;
use crate::observer::StepObserver;
use alloc::vec::Vec as AllocVec;

/// A 2D pressure-based soft body.
pub struct SoftBody<F: Float> {
    particles: AllocVec<Particle<Vec2<F>>>,
    constraints: AllocVec<DistanceConstraint<Vec2<F>>>,
    target_area: F,
    pressure: F,
}

impl<F: Float> SoftBody<F> {
    /// Create a circular soft body.
    pub fn circle(
        center: Vec2<F>,
        radius: F,
        segments: usize,
        pressure: F,
        stiffness: F,
    ) -> Self {
        let mut particles = AllocVec::with_capacity(segments);
        let two_pi = F::two() * F::pi();

        for i in 0..segments {
            let angle = two_pi * F::from_f32(i as f32) / F::from_f32(segments as f32);
            let x = center.x + radius * angle.cos();
            let y = center.y + radius * angle.sin();
            particles.push(Particle::new(Vec2::new(x, y), F::one()));
        }

        let mut constraints = AllocVec::with_capacity(segments);
        // Sequential edge constraints
        for i in 0..segments {
            let j = (i + 1) % segments;
            let rest = particles[i].pos.distance(particles[j].pos);
            constraints.push(DistanceConstraint::new(i, j, rest, stiffness));
        }

        // Cross constraints for structural integrity (connect opposite particles)
        if segments >= 4 {
            let half = segments / 2;
            for i in 0..half {
                let j = i + half;
                let rest = particles[i].pos.distance(particles[j].pos);
                constraints.push(DistanceConstraint::new(i, j, rest, stiffness * F::half()));
            }
        }

        // Calculate initial area as target
        let target_area = Self::compute_area(&particles);

        SoftBody { particles, constraints, target_area, pressure }
    }

    /// Create a rectangular soft body.
    pub fn rectangle(
        center: Vec2<F>,
        width: F,
        height: F,
        segments_per_side: usize,
        pressure: F,
        stiffness: F,
    ) -> Self {
        let segs = if segments_per_side < 2 { 2 } else { segments_per_side };
        let half_w = width * F::half();
        let half_h = height * F::half();
        let mut particles = AllocVec::new();

        // Bottom edge (left to right)
        for i in 0..segs {
            let t = F::from_f32(i as f32) / F::from_f32(segs as f32);
            let x = center.x - half_w + width * t;
            let y = center.y - half_h;
            particles.push(Particle::new(Vec2::new(x, y), F::one()));
        }
        // Right edge (bottom to top)
        for i in 0..segs {
            let t = F::from_f32(i as f32) / F::from_f32(segs as f32);
            let x = center.x + half_w;
            let y = center.y - half_h + height * t;
            particles.push(Particle::new(Vec2::new(x, y), F::one()));
        }
        // Top edge (right to left)
        for i in 0..segs {
            let t = F::from_f32(i as f32) / F::from_f32(segs as f32);
            let x = center.x + half_w - width * t;
            let y = center.y + half_h;
            particles.push(Particle::new(Vec2::new(x, y), F::one()));
        }
        // Left edge (top to bottom)
        for i in 0..segs {
            let t = F::from_f32(i as f32) / F::from_f32(segs as f32);
            let x = center.x - half_w;
            let y = center.y + half_h - height * t;
            particles.push(Particle::new(Vec2::new(x, y), F::one()));
        }

        let n = particles.len();
        let mut constraints = AllocVec::with_capacity(n);
        for i in 0..n {
            let j = (i + 1) % n;
            let rest = particles[i].pos.distance(particles[j].pos);
            constraints.push(DistanceConstraint::new(i, j, rest, stiffness));
        }

        let target_area = Self::compute_area(&particles);

        SoftBody { particles, constraints, target_area, pressure }
    }

    fn compute_area(particles: &[Particle<Vec2<F>>]) -> F {
        let n = particles.len();
        if n < 3 {
            return F::zero();
        }
        let mut sum = F::zero();
        for i in 0..n {
            let j = (i + 1) % n;
            let a = particles[i].pos;
            let b = particles[j].pos;
            sum = sum + (a.x * b.y - b.x * a.y);
        }
        (sum * F::half()).abs()
    }

    pub fn apply_force(&mut self, force: Vec2<F>) {
        for p in self.particles.iter_mut() {
            p.apply_force(force);
        }
    }

    /// Apply an impulse at the nearest particle to `point`.
    pub fn poke(&mut self, point: Vec2<F>, impulse: Vec2<F>) {
        if self.particles.is_empty() {
            return;
        }
        let mut nearest_idx = 0;
        let mut nearest_dist_sq = self.particles[0].pos.distance_sq(point);
        for (i, p) in self.particles.iter().enumerate().skip(1) {
            let d = p.pos.distance_sq(point);
            if d < nearest_dist_sq {
                nearest_dist_sq = d;
                nearest_idx = i;
            }
        }
        // Apply impulse by adjusting prev_pos (Verlet velocity = pos - prev_pos)
        if !self.particles[nearest_idx].pinned {
            self.particles[nearest_idx].prev_pos = self.particles[nearest_idx].prev_pos - impulse;
        }
    }

    pub fn step<O: StepObserver>(
        &mut self,
        dt: F,
        config: &SolverConfig<Vec2<F>>,
        observer: &mut O,
    ) {
        let sub_dt = dt / F::from_f32(config.sub_steps as f32);

        for _sub in 0..config.sub_steps {
            // 1. Calculate pressure forces
            let current_area = self.area();
            if current_area > F::from_f32(1e-6) {
                let ratio = self.target_area / current_area;
                let n = self.particles.len();
                for i in 0..n {
                    let j = (i + 1) % n;
                    let edge = self.particles[j].pos - self.particles[i].pos;
                    let edge_len = edge.length();
                    if edge_len.is_near_zero(F::from_f32(1e-10)) {
                        continue;
                    }
                    let normal = edge.perp().normalize();
                    let force = normal.scale(self.pressure * ratio * edge_len);
                    let half_force = force.scale(F::half());
                    self.particles[i].apply_force(half_force);
                    self.particles[j].apply_force(half_force);
                }
            }

            // 2. Apply gravity
            for p in self.particles.iter_mut() {
                p.apply_acceleration(config.gravity);
            }

            // 3. Integrate
            for p in self.particles.iter_mut() {
                p.integrate(sub_dt, config.damping);
            }
            observer.on_integrate();

            // 4. Solve constraints
            for iter in 0..config.iterations {
                for c in self.constraints.iter() {
                    c.solve(&mut self.particles);
                }
                observer.on_constraint_iteration(iter);
            }
        }

        observer.on_step_complete();
    }

    /// Calculate the current area using the shoelace formula.
    pub fn area(&self) -> F {
        Self::compute_area(&self.particles)
    }

    /// Check if a point is inside the soft body using ray casting.
    pub fn contains(&self, point: Vec2<F>) -> bool {
        let n = self.particles.len();
        if n < 3 {
            return false;
        }
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let pi = self.particles[i].pos;
            let pj = self.particles[j].pos;
            let dy = pj.y - pi.y;
            if !dy.is_near_zero(F::from_f32(1e-10))
                && ((pi.y > point.y) != (pj.y > point.y))
                && (point.x < (pj.x - pi.x) * (point.y - pi.y) / dy + pi.x)
            {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    /// Get the centroid (average position) of the soft body.
    pub fn centroid(&self) -> Vec2<F> {
        if self.particles.is_empty() {
            return Vec2::zero();
        }
        let n = F::from_f32(self.particles.len() as f32);
        let mut sum = Vec2::zero();
        for p in &self.particles {
            sum = sum + p.pos;
        }
        sum.scale(F::one() / n)
    }

    pub fn positions(&self) -> AllocVec<Vec2<F>> {
        self.particles.iter().map(|p| p.pos).collect()
    }

    pub fn particle_count(&self) -> usize {
        self.particles.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circle_area_approx_pi_r_squared() {
        let body = SoftBody::circle(
            Vec2::new(0.0f32, 0.0),
            1.0,   // radius
            32,    // segments
            1.0,   // pressure
            1.0,   // stiffness
        );
        let expected = core::f32::consts::PI; // pi * 1^2
        let area = body.area();
        // With 32 segments, the inscribed polygon area should be close to pi
        assert!((area - expected).abs() < 0.1, "area = {}, expected ≈ {}", area, expected);
    }

    #[test]
    fn contains_center() {
        let body = SoftBody::circle(
            Vec2::new(5.0f32, 5.0),
            2.0,
            16,
            1.0,
            1.0,
        );
        assert!(body.contains(Vec2::new(5.0, 5.0)), "Center should be inside");
        assert!(!body.contains(Vec2::new(100.0, 100.0)), "Far point should be outside");
    }

    #[test]
    fn centroid_at_center() {
        let body = SoftBody::circle(
            Vec2::new(3.0f32, 4.0),
            1.0,
            16,
            1.0,
            1.0,
        );
        let c = body.centroid();
        assert!((c.x - 3.0).abs() < 0.01, "centroid.x = {}", c.x);
        assert!((c.y - 4.0).abs() < 0.01, "centroid.y = {}", c.y);
    }
}
