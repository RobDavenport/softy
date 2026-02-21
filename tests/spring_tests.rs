use softy::{Spring, Spring1D, Spring2D, Vec2, Scalar};

#[test]
fn critically_damped_converges() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(10.0), 4.0,
    );
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
    }
    assert!((spring.value().0 - 10.0).abs() < 0.001);
}

#[test]
fn critically_damped_no_overshoot() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(10.0), 4.0,
    );
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
        assert!(spring.value().0 <= 10.001, "Overshoot detected: {}", spring.value().0);
    }
}

#[test]
fn underdamped_oscillates() {
    let mut spring: Spring1D<f32> = Spring::underdamped(
        Scalar(0.0), Scalar(10.0), 4.0, 0.2,
    );
    let mut crossed = false;
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
        if spring.value().0 > 10.0 {
            crossed = true;
            break;
        }
    }
    assert!(crossed, "Underdamped spring should overshoot target");
}

#[test]
fn overdamped_slower_than_critical() {
    let mut critical: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(10.0), 4.0,
    );
    let mut over: Spring1D<f32> = Spring::overdamped(
        Scalar(0.0), Scalar(10.0), 4.0, 2.0,
    );
    for _ in 0..30 {
        critical.update(1.0 / 60.0);
        over.update(1.0 / 60.0);
    }
    let critical_dist = (critical.value().0 - 10.0).abs();
    let over_dist = (over.value().0 - 10.0).abs();
    assert!(critical_dist < over_dist);
}

#[test]
fn spring_2d_converges() {
    let mut spring: Spring2D<f32> = Spring::critically_damped(
        Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0), 4.0,
    );
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
    }
    let v = spring.value();
    assert!((v.x - 5.0).abs() < 0.001);
    assert!((v.y - 5.0).abs() < 0.001);
}

#[test]
fn is_settled_works() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(1.0), 8.0,
    );
    assert!(!spring.is_settled(0.01, 0.01));
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
    }
    assert!(spring.is_settled(0.01, 0.01));
}

#[test]
fn set_target_mid_simulation() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(0.0), Scalar(5.0), 4.0,
    );
    for _ in 0..100 {
        spring.update(1.0 / 60.0);
    }
    spring.set_target(Scalar(-5.0));
    for _ in 0..1000 {
        spring.update(1.0 / 60.0);
    }
    assert!((spring.value().0 - (-5.0)).abs() < 0.001);
}

#[test]
fn zero_dt_no_change() {
    let mut spring: Spring1D<f32> = Spring::critically_damped(
        Scalar(3.0), Scalar(10.0), 4.0,
    );
    let before = spring.value();
    spring.update(0.0);
    assert_eq!(spring.value(), before);
}

#[test]
fn determinism() {
    let mut first_result = None;
    for _ in 0..10 {
        let mut spring: Spring2D<f32> = Spring::underdamped(
            Vec2::new(1.0, 2.0), Vec2::new(10.0, -5.0), 3.0, 0.3,
        );
        for _ in 0..500 {
            spring.update(1.0 / 60.0);
        }
        let v = spring.value();
        if let Some((fx, fy)) = first_result {
            assert_eq!(v.x, fx);
            assert_eq!(v.y, fy);
        } else {
            first_result = Some((v.x, v.y));
        }
    }
}
