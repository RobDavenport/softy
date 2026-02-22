use softy::{SoftBody, SolverConfig, Vec2, NoOpStepObserver};

#[test]
fn pressure_maintains_area_under_gravity() {
    let mut body = SoftBody::circle(
        Vec2::new(0.0f32, 10.0),
        2.0,   // radius
        16,    // segments
        50.0,  // pressure
        1.0,   // stiffness
    );

    let initial_area = body.area();
    assert!(initial_area > 0.0, "Initial area should be positive, got {}", initial_area);

    let config = SolverConfig::new()
        .with_gravity(Vec2::new(0.0, -9.81))
        .with_iterations(8)
        .with_sub_steps(2);

    let dt = 1.0 / 60.0;
    for _ in 0..200 {
        body.step(dt, &config, &mut NoOpStepObserver);
    }

    let final_area = body.area();
    assert!(
        final_area > initial_area * 0.5,
        "Pressure should prevent total collapse: final area {} is less than 50% of initial area {}",
        final_area,
        initial_area,
    );
}
