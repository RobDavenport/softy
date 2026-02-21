//! Iterative constraint solver for Verlet particle systems.

use crate::float::Float;
use crate::vec::Vec;
use crate::particle::Particle;
use crate::constraint::{Constraint, ConstraintDispatch};
use crate::config::SolverConfig;
use crate::observer::StepObserver;
use alloc::vec::Vec as AllocVec;

/// Iterative constraint solver for Verlet particle systems.
pub struct ConstraintSolver<V: Vec> {
    pub particles: AllocVec<Particle<V>>,
    pub constraints: AllocVec<Constraint<V>>,
}

impl<V: Vec> ConstraintSolver<V> {
    pub fn new() -> Self {
        ConstraintSolver {
            particles: AllocVec::new(),
            constraints: AllocVec::new(),
        }
    }

    pub fn add_particle(&mut self, particle: Particle<V>) -> usize {
        let idx = self.particles.len();
        self.particles.push(particle);
        idx
    }

    pub fn add_constraint(&mut self, constraint: Constraint<V>) {
        self.constraints.push(constraint);
    }

    pub fn step<O: StepObserver>(
        &mut self,
        dt: V::Scalar,
        config: &SolverConfig<V>,
        observer: &mut O,
    ) where V: ConstraintDispatch {
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
                for constraint in self.constraints.iter() {
                    constraint.solve(&mut self.particles);
                }
                observer.on_constraint_iteration(i);
            }
        }

        observer.on_step_complete();
    }

    pub fn particle_count(&self) -> usize { self.particles.len() }
    pub fn constraint_count(&self) -> usize { self.constraints.len() }
    pub fn particle(&self, index: usize) -> &Particle<V> { &self.particles[index] }
    pub fn particle_mut(&mut self, index: usize) -> &mut Particle<V> { &mut self.particles[index] }

    pub fn remove_constraint(&mut self, index: usize) -> Constraint<V> {
        self.constraints.swap_remove(index)
    }

    pub fn clear_constraints(&mut self) {
        self.constraints.clear();
    }
}
