//! Verlet grid (cloth) with structural, shear, and bend constraints.

use crate::float::Float;
use crate::vec::Vec;
use crate::vec::Vec2;
use crate::particle::Particle;
use crate::constraint::DistanceConstraint;
use crate::config::SolverConfig;
use crate::observer::StepObserver;
use alloc::vec::Vec as AllocVec;

/// Configuration for a cloth grid.
pub struct GridConfig<F: Float> {
    pub cols: usize,
    pub rows: usize,
    pub spacing: F,
    pub structural_stiffness: F,
    pub shear_stiffness: F,
    pub bend_stiffness: F,
    pub particle_mass: F,
}

/// A cloth mesh built from a grid of Verlet particles.
pub struct VerletGrid<V: Vec> {
    particles: AllocVec<Particle<V>>,
    constraints: AllocVec<DistanceConstraint<V>>,
    cols: usize,
    rows: usize,
}

impl<V: Vec> VerletGrid<V> {
    /// Create a new grid starting at `origin` with the given configuration.
    ///
    /// The grid extends in positive X (columns) and positive Y (rows).
    /// Particle at (col, row) has index `row * cols + col`.
    ///
    /// Creates 3 types of constraints:
    /// - Structural: horizontal + vertical neighbors (rest = spacing)
    /// - Shear: diagonal neighbors (rest = spacing * sqrt(2))
    /// - Bend: skip-one horizontal + vertical (rest = spacing * 2)
    ///
    /// Note: For generic `Vec`, all particles are created at `origin` since
    /// per-component positioning requires concrete vector types. Use
    /// `VerletGrid::<Vec2<F>>::new_2d()` for proper 2D positioning.
    pub fn new(origin: V, config: &GridConfig<V::Scalar>) -> Self {
        let cols = config.cols.max(1);
        let rows = config.rows.max(1);
        let spacing = config.spacing;
        let mut particles = AllocVec::with_capacity(cols * rows);
        let mut constraints = AllocVec::new();

        // For generic Vec, we can only place all particles at origin.
        // Per-component positioning requires concrete types (see new_2d).
        for _row in 0..rows {
            for _col in 0..cols {
                particles.push(Particle::new(origin, config.particle_mass));
            }
        }

        let diag_length = (spacing * spacing + spacing * spacing).sqrt();
        let bend_length = spacing * V::Scalar::two();

        // Structural: horizontal (col, col+1)
        for row in 0..rows {
            for col in 0..(cols - 1) {
                let a = row * cols + col;
                let b = row * cols + col + 1;
                constraints.push(DistanceConstraint::new(a, b, spacing, config.structural_stiffness));
            }
        }

        // Structural: vertical (row, row+1)
        for row in 0..(rows - 1) {
            for col in 0..cols {
                let a = row * cols + col;
                let b = (row + 1) * cols + col;
                constraints.push(DistanceConstraint::new(a, b, spacing, config.structural_stiffness));
            }
        }

        // Shear: diagonals
        for row in 0..(rows - 1) {
            for col in 0..(cols - 1) {
                let tl = row * cols + col;
                let tr = row * cols + col + 1;
                let bl = (row + 1) * cols + col;
                let br = (row + 1) * cols + col + 1;
                // Top-left to bottom-right
                constraints.push(DistanceConstraint::new(tl, br, diag_length, config.shear_stiffness));
                // Top-right to bottom-left
                constraints.push(DistanceConstraint::new(tr, bl, diag_length, config.shear_stiffness));
            }
        }

        // Bend: skip-one horizontal
        for row in 0..rows {
            for col in 0..(cols.saturating_sub(2)) {
                let a = row * cols + col;
                let b = row * cols + col + 2;
                constraints.push(DistanceConstraint::new(a, b, bend_length, config.bend_stiffness));
            }
        }

        // Bend: skip-one vertical
        for row in 0..(rows.saturating_sub(2)) {
            for col in 0..cols {
                let a = row * cols + col;
                let b = (row + 2) * cols + col;
                constraints.push(DistanceConstraint::new(a, b, bend_length, config.bend_stiffness));
            }
        }

        VerletGrid { particles, constraints, cols, rows }
    }

    pub fn index(&self, col: usize, row: usize) -> usize {
        row * self.cols + col
    }

    pub fn pin(&mut self, col: usize, row: usize) {
        let idx = self.index(col, row);
        self.particles[idx].pin();
    }

    pub fn unpin(&mut self, col: usize, row: usize, mass: V::Scalar) {
        let idx = self.index(col, row);
        self.particles[idx].unpin(mass);
    }

    pub fn pin_top_row(&mut self) {
        for col in 0..self.cols {
            self.pin(col, 0);
        }
    }

    pub fn move_pin(&mut self, col: usize, row: usize, pos: V) {
        let idx = self.index(col, row);
        self.particles[idx].move_to(pos);
    }

    pub fn tear_at(&mut self, col: usize, row: usize) {
        let idx = self.index(col, row);
        self.constraints.retain(|c| c.a != idx && c.b != idx);
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

    pub fn position_at(&self, col: usize, row: usize) -> V {
        self.particles[self.index(col, row)].pos
    }

    pub fn cols(&self) -> usize { self.cols }
    pub fn rows(&self) -> usize { self.rows }
    pub fn particle_count(&self) -> usize { self.particles.len() }
    pub fn constraint_count(&self) -> usize { self.constraints.len() }
}

impl<F: Float> VerletGrid<Vec2<F>> {
    /// Create a grid with proper 2D positioning.
    pub fn new_2d(origin: Vec2<F>, config: &GridConfig<F>) -> Self {
        let cols = config.cols.max(1);
        let rows = config.rows.max(1);
        let spacing = config.spacing;
        let mut particles = AllocVec::with_capacity(cols * rows);
        let mut constraints = AllocVec::new();

        for row in 0..rows {
            for col in 0..cols {
                let x = origin.x + F::from_f32(col as f32) * spacing;
                let y = origin.y + F::from_f32(row as f32) * spacing;
                particles.push(Particle::new(Vec2::new(x, y), config.particle_mass));
            }
        }

        let diag_length = (spacing * spacing + spacing * spacing).sqrt();
        let bend_length = spacing * F::two();

        // Structural: horizontal
        for row in 0..rows {
            for col in 0..(cols - 1) {
                let a = row * cols + col;
                let b = row * cols + col + 1;
                constraints.push(DistanceConstraint::new(a, b, spacing, config.structural_stiffness));
            }
        }

        // Structural: vertical
        for row in 0..(rows - 1) {
            for col in 0..cols {
                let a = row * cols + col;
                let b = (row + 1) * cols + col;
                constraints.push(DistanceConstraint::new(a, b, spacing, config.structural_stiffness));
            }
        }

        // Shear: diagonals
        for row in 0..(rows - 1) {
            for col in 0..(cols - 1) {
                let tl = row * cols + col;
                let tr = row * cols + col + 1;
                let bl = (row + 1) * cols + col;
                let br = (row + 1) * cols + col + 1;
                constraints.push(DistanceConstraint::new(tl, br, diag_length, config.shear_stiffness));
                constraints.push(DistanceConstraint::new(tr, bl, diag_length, config.shear_stiffness));
            }
        }

        // Bend: skip-one horizontal
        for row in 0..rows {
            for col in 0..(cols.saturating_sub(2)) {
                let a = row * cols + col;
                let b = row * cols + col + 2;
                constraints.push(DistanceConstraint::new(a, b, bend_length, config.bend_stiffness));
            }
        }

        // Bend: skip-one vertical
        for row in 0..(rows.saturating_sub(2)) {
            for col in 0..cols {
                let a = row * cols + col;
                let b = (row + 2) * cols + col;
                constraints.push(DistanceConstraint::new(a, b, bend_length, config.bend_stiffness));
            }
        }

        VerletGrid { particles, constraints, cols, rows }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vec::Vec2;

    fn test_config() -> GridConfig<f32> {
        GridConfig {
            cols: 4,
            rows: 3,
            spacing: 1.0,
            structural_stiffness: 1.0,
            shear_stiffness: 0.5,
            bend_stiffness: 0.3,
            particle_mass: 1.0,
        }
    }

    #[test]
    fn correct_particle_count() {
        let grid: VerletGrid<Vec2<f32>> = VerletGrid::new_2d(Vec2::new(0.0, 0.0), &test_config());
        assert_eq!(grid.particle_count(), 12); // 4 * 3
    }

    #[test]
    fn structural_constraint_count() {
        let config = test_config(); // 4 cols, 3 rows
        let grid: VerletGrid<Vec2<f32>> = VerletGrid::new_2d(Vec2::new(0.0, 0.0), &config);
        // Horizontal: (4-1)*3 = 9
        // Vertical: 4*(3-1) = 8
        // Shear: (4-1)*(3-1)*2 = 12
        // Bend horiz: (4-2)*3 = 6
        // Bend vert: 4*(3-2) = 4
        // Total: 9 + 8 + 12 + 6 + 4 = 39
        assert_eq!(grid.constraint_count(), 39);
    }

    #[test]
    fn tear_at_removes_constraints() {
        let mut grid: VerletGrid<Vec2<f32>> = VerletGrid::new_2d(Vec2::new(0.0, 0.0), &test_config());
        let before = grid.constraint_count();
        grid.tear_at(1, 1); // middle-ish particle
        assert!(grid.constraint_count() < before, "tear_at should remove constraints");
    }
}
