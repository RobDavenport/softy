//! Verlet particles with position-based dynamics.

use crate::float::Float;
use crate::vec::Vec;

/// A Verlet particle — position-based dynamics with implicit velocity.
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
    pub fn new(pos: V, mass: V::Scalar) -> Self {
        let inv_mass = if mass.is_near_zero(V::Scalar::from_f32(1e-10)) {
            V::Scalar::zero()
        } else {
            V::Scalar::one() / mass
        };
        Particle {
            pos,
            prev_pos: pos,
            acceleration: V::zero(),
            mass,
            inv_mass,
            pinned: false,
        }
    }

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

    pub fn apply_force(&mut self, force: V) {
        if !self.pinned {
            self.acceleration = self.acceleration + force.scale(self.inv_mass);
        }
    }

    pub fn apply_acceleration(&mut self, accel: V) {
        if !self.pinned {
            self.acceleration = self.acceleration + accel;
        }
    }

    pub fn integrate(&mut self, dt: V::Scalar, damping: V::Scalar) {
        if self.pinned {
            return;
        }
        let velocity = (self.pos - self.prev_pos).scale(damping);
        let new_pos = self.pos + velocity + self.acceleration.scale(dt * dt);
        self.prev_pos = self.pos;
        self.pos = new_pos;
        self.acceleration = V::zero();
    }

    pub fn velocity(&self, dt: V::Scalar) -> V {
        if dt.is_near_zero(V::Scalar::from_f32(1e-30)) {
            return V::zero();
        }
        (self.pos - self.prev_pos).scale(V::Scalar::one() / dt)
    }

    pub fn velocity_raw(&self) -> V {
        self.pos - self.prev_pos
    }

    pub fn pin(&mut self) {
        self.pinned = true;
        self.inv_mass = V::Scalar::zero();
        self.prev_pos = self.pos;
    }

    pub fn unpin(&mut self, mass: V::Scalar) {
        self.pinned = false;
        self.mass = mass;
        self.inv_mass = if mass.is_near_zero(V::Scalar::from_f32(1e-10)) {
            V::Scalar::zero()
        } else {
            V::Scalar::one() / mass
        };
    }

    pub fn move_to(&mut self, pos: V) {
        if self.pinned {
            self.prev_pos = self.pos;
            self.pos = pos;
        }
    }
}
