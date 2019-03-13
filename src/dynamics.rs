use crate::states::*;

pub fn dynamics_1d( states: States1D, delta: f32 )-> States1D {
    let a = (-1.*delta).exp();
    States1D(states.0 * a)
}

pub fn stop_cond_1d( states: States1D, states_goal: States1D )-> bool {
    (states.0 - states_goal.0).abs() < 0.00001
}
