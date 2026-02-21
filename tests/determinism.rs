use softy::{Spring, Spring2D, Vec2, VerletChain, ChainConfig, SolverConfig, NoOpStepObserver};

#[test]
fn spring_deterministic() {
    let results: Vec<_> = (0..10).map(|_| {
        let mut spring: Spring2D<f32> = Spring::underdamped(
            Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0), 3.0, 0.3,
        );
        for _ in 0..500 {
            spring.update(1.0 / 60.0);
        }
        spring.value()
    }).collect();

    for r in &results[1..] {
        assert_eq!(results[0].x, r.x);
        assert_eq!(results[0].y, r.y);
    }
}

#[test]
fn chain_deterministic() {
    let results: Vec<_> = (0..5).map(|_| {
        let mut chain: VerletChain<Vec2<f32>> = VerletChain::new(
            Vec2::new(0.0, 10.0), Vec2::new(10.0, 10.0), 10, ChainConfig::default(),
        );
        chain.pin(0);
        let config = SolverConfig::new()
            .with_gravity(Vec2::new(0.0, -9.81))
            .with_iterations(8);
        for _ in 0..60 {
            chain.step(1.0 / 60.0, &config, &mut NoOpStepObserver);
        }
        chain.positions()
    }).collect();

    for r in &results[1..] {
        for (a, b) in results[0].iter().zip(r.iter()) {
            assert_eq!(a.x, b.x);
            assert_eq!(a.y, b.y);
        }
    }
}
