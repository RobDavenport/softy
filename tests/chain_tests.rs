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
    chain.pin(0);

    let config = SolverConfig::new()
        .with_gravity(Vec2::new(0.0, -9.81))
        .with_iterations(8)
        .with_sub_steps(2);

    for _ in 0..120 {
        chain.step(1.0 / 60.0, &config, &mut NoOpStepObserver);
    }

    let start_y = chain.particle(0).pos.y;
    let end_y = chain.particle(chain.len() - 1).pos.y;
    assert!(end_y < start_y, "End should drop below pinned start");
}
