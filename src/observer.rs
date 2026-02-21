//! Step observer trait for monitoring physics simulation progress.

/// Trait for observing physics simulation steps.
///
/// Implement this trait to monitor solver progress (e.g., for debugging,
/// visualization, or performance profiling). All methods have default
/// no-op implementations.
pub trait StepObserver {
    /// Called after all particles have been integrated (Verlet step).
    fn on_integrate(&mut self) {}

    /// Called after each constraint iteration.
    fn on_constraint_iteration(&mut self, _iteration: usize) {}

    /// Called when a simulation step is fully complete.
    fn on_step_complete(&mut self) {}
}

/// A no-op observer that does nothing. Use as default when no observation needed.
pub struct NoOpStepObserver;

impl StepObserver for NoOpStepObserver {}
