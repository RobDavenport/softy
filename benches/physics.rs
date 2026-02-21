//! Benchmarks for softy physics simulation.

use criterion::{criterion_group, criterion_main, Criterion};
use softy::*;
use softy::grid::GridConfig;

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
            let config = GridConfig {
                cols: 20,
                rows: 20,
                spacing: 1.0,
                structural_stiffness: 1.0,
                shear_stiffness: 0.5,
                bend_stiffness: 0.3,
                particle_mass: 1.0,
            };
            let mut grid: VerletGrid<Vec2<f32>> = VerletGrid::new_2d(Vec2::new(0.0, 0.0), &config);
            grid.pin_top_row();
            let solver_config = SolverConfig::new()
                .with_gravity(Vec2::new(0.0, -9.81))
                .with_iterations(4);
            for _ in 0..60 {
                grid.step(1.0 / 60.0, &solver_config, &mut NoOpStepObserver);
            }
            grid.positions()
        });
    });
}

criterion_group!(benches, bench_spring_update, bench_chain_simulation, bench_grid_simulation);
criterion_main!(benches);
