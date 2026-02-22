use softy::{VerletChain, ChainConfig, SolverConfig, Vec2, NoOpStepObserver};
use softy::Vec;

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

#[test]
fn chain_move_pin() {
    let mut chain: VerletChain<Vec2<f32>> = VerletChain::new(
        Vec2::new(0.0, 0.0),
        Vec2::new(5.0, 0.0),
        5,
        ChainConfig::default(),
    );
    chain.pin(0);

    let new_pos = Vec2::new(3.0, 4.0);
    chain.move_pin(0, new_pos);

    let p = chain.particle(0);
    assert!(
        (p.pos.x - 3.0).abs() < 1e-6 && (p.pos.y - 4.0).abs() < 1e-6,
        "Pinned particle 0 should have moved to (3.0, 4.0), got ({}, {})",
        p.pos.x,
        p.pos.y,
    );
}

#[test]
fn chain_total_length_after_settling() {
    let segments = 10;
    let start = Vec2::new(0.0, 10.0);
    let end = Vec2::new(10.0, 10.0);

    let mut chain: VerletChain<Vec2<f32>> = VerletChain::new(
        start,
        end,
        segments,
        ChainConfig::default(),
    );
    chain.pin(0);
    chain.pin(segments);

    // Chain starts straight, so total rest length = endpoint distance.
    let expected_total_rest_length: f32 = start.distance(end); // 10.0

    let config = SolverConfig::new()
        .with_gravity(Vec2::new(0.0, -9.81))
        .with_iterations(8)
        .with_sub_steps(2);

    for _ in 0..300 {
        chain.step(1.0 / 60.0, &config, &mut NoOpStepObserver);
    }

    let mut actual_total_length: f32 = 0.0;
    for i in 0..chain.len() - 1 {
        let a = chain.particle(i).pos;
        let b = chain.particle(i + 1).pos;
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        actual_total_length += (dx * dx + dy * dy).sqrt();
    }

    let diff = (actual_total_length - expected_total_rest_length).abs();
    assert!(
        diff < 0.5,
        "Total chain length {:.4} should be within 0.5 of expected rest length {:.4} (diff = {:.4})",
        actual_total_length,
        expected_total_rest_length,
        diff,
    );
}
