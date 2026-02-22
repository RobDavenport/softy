use softy::{VerletGrid, GridConfig, SolverConfig, Vec2, NoOpStepObserver};

#[test]
fn pinned_top_row_drapes_under_gravity() {
    let config = GridConfig {
        cols: 5,
        rows: 5,
        spacing: 1.0,
        structural_stiffness: 1.0,
        shear_stiffness: 0.5,
        bend_stiffness: 0.3,
        particle_mass: 1.0,
    };

    let mut grid = VerletGrid::<Vec2<f32>>::new_2d(Vec2::new(0.0, 0.0), &config);

    // Record top row (row 0) initial positions before pinning.
    let mut top_initial = Vec::new();
    for col in 0..grid.cols() {
        top_initial.push(grid.position_at(col, 0));
    }

    // Record bottom row (row 4) initial positions.
    let bottom_row = grid.rows() - 1;
    let mut bottom_initial = Vec::new();
    for col in 0..grid.cols() {
        bottom_initial.push(grid.position_at(col, bottom_row));
    }

    // Pin the entire top row.
    grid.pin_top_row();

    let solver = SolverConfig::new()
        .with_gravity(Vec2::new(0.0, -9.81))
        .with_iterations(8)
        .with_sub_steps(2);

    // Simulate for 120 steps (~2 seconds at 60 Hz).
    for _ in 0..120 {
        grid.step(1.0 / 60.0, &solver, &mut NoOpStepObserver);
    }

    // Assert: top row particles have not moved from initial positions.
    for col in 0..grid.cols() {
        let pos = grid.position_at(col, 0);
        let init = top_initial[col];
        assert!(
            (pos.x - init.x).abs() < 1e-6 && (pos.y - init.y).abs() < 1e-6,
            "Top row particle at col {} should remain pinned at ({}, {}), but found ({}, {})",
            col, init.x, init.y, pos.x, pos.y,
        );
    }

    // Assert: bottom row particles have dropped below their initial y positions.
    for col in 0..grid.cols() {
        let pos = grid.position_at(col, bottom_row);
        let init = bottom_initial[col];
        assert!(
            pos.y < init.y,
            "Bottom row particle at col {} should have dropped below initial y {}, but y is {}",
            col, init.y, pos.y,
        );
    }
}
