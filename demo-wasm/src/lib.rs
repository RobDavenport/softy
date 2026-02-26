use softy::{
    ChainConfig, GridConfig, NoOpStepObserver, SoftBody, SolverConfig, Spring, Spring2D, Vec2,
    VerletChain, VerletGrid,
};
use wasm_bindgen::prelude::*;

// ---- Springs Demo ----

#[wasm_bindgen]
pub struct SpringsDemo {
    critical: Spring2D<f32>,
    underdamped: Spring2D<f32>,
    overdamped: Spring2D<f32>,
}

#[wasm_bindgen]
impl SpringsDemo {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        let target = Vec2::new(300.0f32, 300.0);
        SpringsDemo {
            critical: Spring::critically_damped(target, target, 4.0),
            underdamped: Spring::underdamped(target, target, 4.0, 0.2),
            overdamped: Spring::overdamped(target, target, 4.0, 2.0),
        }
    }

    pub fn set_target(&mut self, x: f32, y: f32) {
        let t = Vec2::new(x, y);
        self.critical.set_target(t);
        self.underdamped.set_target(t);
        self.overdamped.set_target(t);
    }

    pub fn update(&mut self, dt: f32) {
        self.critical.update(dt);
        self.underdamped.update(dt);
        self.overdamped.update(dt);
    }

    /// Returns [cx, cy, ux, uy, ox, oy] -- positions of critical, underdamped, overdamped
    pub fn positions(&self) -> Vec<f32> {
        let c = self.critical.value();
        let u = self.underdamped.value();
        let o = self.overdamped.value();
        vec![c.x, c.y, u.x, u.y, o.x, o.y]
    }
}

// ---- Rope Demo ----

#[wasm_bindgen]
pub struct RopeDemo {
    chain: VerletChain<Vec2<f32>>,
    config: SolverConfig<Vec2<f32>>,
}

#[wasm_bindgen]
impl RopeDemo {
    #[wasm_bindgen(constructor)]
    pub fn new(segments: usize) -> Self {
        let mut chain = VerletChain::new(
            Vec2::new(100.0f32, 50.0),
            Vec2::new(500.0, 50.0),
            segments,
            ChainConfig::default(),
        );
        chain.pin(0);
        RopeDemo {
            chain,
            config: SolverConfig::new()
                .with_gravity(Vec2::new(0.0, 200.0))
                .with_iterations(8)
                .with_sub_steps(2),
        }
    }

    pub fn update(&mut self, dt: f32) {
        self.chain.step(dt, &self.config, &mut NoOpStepObserver);
    }

    pub fn move_pin(&mut self, x: f32, y: f32) {
        self.chain.move_pin(0, Vec2::new(x, y));
    }

    /// Returns flat [x0, y0, x1, y1, ...] positions
    pub fn positions(&self) -> Vec<f32> {
        let pos = self.chain.positions();
        let mut out = Vec::with_capacity(pos.len() * 2);
        for p in &pos {
            out.push(p.x);
            out.push(p.y);
        }
        out
    }

    pub fn particle_count(&self) -> usize {
        self.chain.len()
    }
}

// ---- Cloth Demo ----

#[wasm_bindgen]
pub struct ClothDemo {
    grid: VerletGrid<Vec2<f32>>,
    config: SolverConfig<Vec2<f32>>,
    cols: usize,
    rows: usize,
}

#[wasm_bindgen]
impl ClothDemo {
    #[wasm_bindgen(constructor)]
    pub fn new(cols: usize, rows: usize, spacing: f32) -> Self {
        let grid_config = GridConfig {
            cols,
            rows,
            spacing,
            structural_stiffness: 1.0,
            shear_stiffness: 0.5,
            bend_stiffness: 0.3,
            particle_mass: 1.0,
        };
        let mut grid = VerletGrid::new_2d(Vec2::new(140.0f32, 40.0), &grid_config);
        grid.pin(0, 0);
        if cols > 1 {
            grid.pin(cols - 1, 0);
        }

        ClothDemo {
            grid,
            config: SolverConfig::new()
                .with_gravity(Vec2::new(0.0, 180.0))
                .with_iterations(8)
                .with_sub_steps(2),
            cols,
            rows,
        }
    }

    pub fn update(&mut self, dt: f32) {
        self.grid.step(dt, &self.config, &mut NoOpStepObserver);
    }

    pub fn tear_at(&mut self, col: usize, row: usize) {
        self.grid.tear_at(col, row);
    }

    pub fn tear_patch(&mut self, col: usize, row: usize, radius: usize) {
        if self.cols == 0 || self.rows == 0 {
            return;
        }
        let min_col = col.saturating_sub(radius);
        let max_col = col.saturating_add(radius).min(self.cols - 1);
        let min_row = row.saturating_sub(radius);
        let max_row = row.saturating_add(radius).min(self.rows - 1);
        for r in min_row..=max_row {
            if r == 0 {
                continue;
            }
            for c in min_col..=max_col {
                self.grid.tear_at(c, r);
            }
        }
    }

    pub fn apply_wind(&mut self, strength: f32) {
        self.grid.apply_force(Vec2::new(strength, 0.0));
    }

    /// Returns flat [x0, y0, x1, y1, ...] in row-major order
    pub fn positions(&self) -> Vec<f32> {
        let pos = self.grid.positions();
        let mut out = Vec::with_capacity(pos.len() * 2);
        for p in &pos {
            out.push(p.x);
            out.push(p.y);
        }
        out
    }

    pub fn cols(&self) -> usize {
        self.cols
    }
    pub fn rows(&self) -> usize {
        self.rows
    }
}

// ---- Soft Body Demo ----

#[wasm_bindgen]
pub struct SoftBodyDemo {
    bodies: Vec<SoftBody<f32>>,
    config: SolverConfig<Vec2<f32>>,
    min_bound: Vec2<f32>,
    max_bound: Vec2<f32>,
    poke_strength: f32,
}

#[wasm_bindgen]
impl SoftBodyDemo {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        let mut bodies = Vec::new();
        bodies.push(SoftBody::circle(
            Vec2::new(220.0f32, 160.0),
            40.0,
            16,
            48.0,
            0.85,
        ));
        bodies.push(SoftBody::circle(
            Vec2::new(350.0, 130.0),
            32.0,
            12,
            55.0,
            0.85,
        ));
        bodies.push(SoftBody::circle(
            Vec2::new(480.0, 160.0),
            35.0,
            14,
            45.0,
            0.85,
        ));

        let min_bound = Vec2::new(20.0f32, 20.0);
        let max_bound = Vec2::new(680.0f32, 480.0);

        SoftBodyDemo {
            bodies,
            config: SolverConfig::new()
                .with_gravity(Vec2::new(0.0, 150.0))
                .with_iterations(6)
                .with_sub_steps(2),
            min_bound,
            max_bound,
            poke_strength: 8.0,
        }
    }

    pub fn update(&mut self, dt: f32) {
        for body in &mut self.bodies {
            body.step(dt, &self.config, &mut NoOpStepObserver);
            body.apply_bounds(self.min_bound, self.max_bound, 0.3);
        }
    }

    pub fn poke(&mut self, x: f32, y: f32) {
        let point = Vec2::new(x, y);
        let mut target_idx = None;
        for (idx, body) in self.bodies.iter().enumerate() {
            if body.contains(point) {
                target_idx = Some(idx);
                break;
            }
        }
        if target_idx.is_none() {
            let mut best_idx = 0usize;
            let mut best_dist_sq = f32::MAX;
            for (idx, body) in self.bodies.iter().enumerate() {
                let c = body.centroid();
                let dx = c.x - x;
                let dy = c.y - y;
                let d2 = dx * dx + dy * dy;
                if d2 < best_dist_sq {
                    best_dist_sq = d2;
                    best_idx = idx;
                }
            }
            if best_dist_sq <= 70.0 * 70.0 {
                target_idx = Some(best_idx);
            }
        }
        if let Some(idx) = target_idx {
            let centroid = self.bodies[idx].centroid();
            let dx = centroid.x - x;
            let dy = centroid.y - y;
            let len = (dx * dx + dy * dy).sqrt();
            let impulse = if len > 1e-5 {
                Vec2::new(dx * self.poke_strength / len, dy * self.poke_strength / len)
            } else {
                Vec2::new(0.0, -self.poke_strength)
            };
            self.bodies[idx].poke(point, impulse);
        }
    }

    pub fn bounds(&self) -> Vec<f32> {
        vec![
            self.min_bound.x,
            self.min_bound.y,
            self.max_bound.x,
            self.max_bound.y,
        ]
    }

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Returns positions for body at index as flat [x0, y0, x1, y1, ...]
    pub fn body_positions(&self, index: usize) -> Vec<f32> {
        let pos = self.bodies[index].positions();
        let mut out = Vec::with_capacity(pos.len() * 2);
        for p in &pos {
            out.push(p.x);
            out.push(p.y);
        }
        out
    }

    pub fn body_particle_count(&self, index: usize) -> usize {
        self.bodies[index].particle_count()
    }
}
