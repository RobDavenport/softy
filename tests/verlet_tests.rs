use softy::{Particle, Vec2, ConstraintSolver, SolverConfig, Constraint, DistanceConstraint, NoOpStepObserver};
use softy::vec::Vec;

#[test]
fn free_fall_gravity() {
    let mut p: Particle<Vec2<f32>> = Particle::new(Vec2::new(0.0, 100.0), 1.0);
    let g = Vec2::new(0.0, -9.81);
    let dt = 1.0 / 60.0;
    let steps = 60;

    for _ in 0..steps {
        p.apply_acceleration(g);
        p.integrate(dt, 1.0);
    }

    let expected_y = 100.0 - 0.5 * 9.81 * 1.0;
    assert!((p.pos.y - expected_y).abs() < 1.0, "pos.y = {}, expected ~ {}", p.pos.y, expected_y);
}

#[test]
fn distance_constraint_maintains_length() {
    let mut solver: ConstraintSolver<Vec2<f32>> = ConstraintSolver::new();
    solver.add_particle(Particle::new(Vec2::new(0.0, 0.0), 1.0));
    solver.add_particle(Particle::new(Vec2::new(5.0, 0.0), 1.0));
    solver.add_constraint(Constraint::Distance(
        DistanceConstraint::new(0, 1, 5.0, 1.0),
    ));

    solver.particle_mut(1).pos = Vec2::new(20.0, 0.0);

    let config = SolverConfig::new().with_iterations(10);
    solver.step(1.0 / 60.0, &config, &mut NoOpStepObserver);

    let dist = solver.particle(0).pos.distance(solver.particle(1).pos);
    assert!((dist - 5.0).abs() < 0.5, "Distance should be near 5.0, got {}", dist);
}

#[test]
fn pinned_particle_stays_fixed() {
    let mut p: Particle<Vec2<f32>> = Particle::pinned(Vec2::new(5.0, 5.0));
    p.apply_force(Vec2::new(1000.0, 1000.0));
    p.integrate(1.0 / 60.0, 1.0);
    assert_eq!(p.pos.x, 5.0);
    assert_eq!(p.pos.y, 5.0);
}

#[test]
fn mass_weighting_heavier_moves_less() {
    // Heavy particle at origin, light particle at (10, 0).
    // Connected by a distance constraint with rest_length 5.0.
    // Current distance is 10.0 so constraint should pull them together.
    let mut particles = [
        Particle::new(Vec2::new(0.0, 0.0), 10.0), // heavy
        Particle::new(Vec2::new(10.0, 0.0), 1.0),  // light
    ];

    let initial_heavy = particles[0].pos;
    let initial_light = particles[1].pos;

    let constraint = DistanceConstraint::new(0, 1, 5.0, 1.0);
    constraint.solve(&mut particles);

    let heavy_displacement = particles[0].pos.distance(initial_heavy);
    let light_displacement = particles[1].pos.distance(initial_light);

    assert!(
        heavy_displacement < light_displacement,
        "Heavier particle should move less: heavy moved {}, light moved {}",
        heavy_displacement, light_displacement
    );

    // Verify both actually moved (constraint was active)
    assert!(heavy_displacement > 0.0, "Heavy particle should have moved");
    assert!(light_displacement > 0.0, "Light particle should have moved");
}
