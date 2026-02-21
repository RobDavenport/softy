//! Verlet chain (rope/string) built from particles and distance constraints.

use crate::float::Float;
use crate::vec::Vec;
use crate::particle::Particle;
use crate::constraint::DistanceConstraint;
use crate::config::SolverConfig;
use crate::observer::StepObserver;
use alloc::vec::Vec as AllocVec;

/// Configuration for creating a chain.
pub struct ChainConfig<F: Float> {
    pub stiffness: F,
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
pub struct VerletChain<V: Vec> {
    particles: AllocVec<Particle<V>>,
    constraints: AllocVec<DistanceConstraint<V>>,
}

impl<V: Vec> VerletChain<V> {
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

    pub fn pin(&mut self, index: usize) {
        self.particles[index].pin();
    }

    pub fn unpin(&mut self, index: usize, mass: V::Scalar) {
        self.particles[index].unpin(mass);
    }

    pub fn move_pin(&mut self, index: usize, pos: V) {
        self.particles[index].move_to(pos);
    }

    pub fn apply_force(&mut self, force: V) {
        for p in self.particles.iter_mut() {
            p.apply_force(force);
        }
    }

    pub fn apply_acceleration(&mut self, accel: V) {
        for p in self.particles.iter_mut() {
            p.apply_acceleration(accel);
        }
    }

    pub fn apply_force_at(&mut self, index: usize, force: V) {
        self.particles[index].apply_force(force);
    }

    pub fn step<O: StepObserver>(
        &mut self,
        dt: V::Scalar,
        config: &SolverConfig<V>,
        observer: &mut O,
    ) {
        let sub_dt = dt / V::Scalar::from_f32(config.sub_steps as f32);

        for _sub in 0..config.sub_steps {
            for p in self.particles.iter_mut() {
                p.apply_acceleration(config.gravity);
            }

            for p in self.particles.iter_mut() {
                p.integrate(sub_dt, config.damping);
            }
            observer.on_integrate();

            for i in 0..config.iterations {
                for c in self.constraints.iter() {
                    c.solve(&mut self.particles);
                }
                observer.on_constraint_iteration(i);
            }
        }

        observer.on_step_complete();
    }

    pub fn positions(&self) -> AllocVec<V> {
        self.particles.iter().map(|p| p.pos).collect()
    }

    pub fn len(&self) -> usize {
        self.particles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.particles.is_empty()
    }

    pub fn segment_count(&self) -> usize {
        self.constraints.len()
    }

    pub fn particle(&self, index: usize) -> &Particle<V> {
        &self.particles[index]
    }

    pub fn particle_mut(&mut self, index: usize) -> &mut Particle<V> {
        &mut self.particles[index]
    }
}
