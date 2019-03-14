use crate::states::*;
use crate::control::*;

// pub fn dynamics_1d( states: States1D, delta: f32 )-> States1D {
//     let a = (-1.*delta).exp();
//     States1D(states.0 * a)
// }

// pub fn stop_cond_1d( states: States1D, states_goal: States1D )-> bool {
//     (states.0 - states_goal.0).abs() < 0.00001
// }


pub fn dynamics_3d_1d( states: States3D, control: Control1D, delta: f32 )-> States3D {
    // let a = (-1.*delta).exp();
    // States3D( [states.0[0] * a, states.0[1] * a, states.0[2] * a] )
    unimplemented!();
}

pub fn stop_cond_3d( states: States3D, states_goal: States3D )-> bool {
    states.0.iter()
        .zip( states_goal.0.iter() )
        .all( |x| ((x.0)-(x.1)).abs() < 0.000001 )
}

pub fn dynamics_6d_3d( states: States6D, control: Control3D, delta: f32 )-> States6D {
    // let a = (-1.*delta).exp();
    // States3D( [states.0[0] * a, states.0[1] * a, states.0[2] * a] )
    unimplemented!();
}

pub fn stop_cond_6d( states: States6D, states_goal: States6D )-> bool {
    states.0.iter()
        .zip( states_goal.0.iter() )
        .all( |x| ((x.0)-(x.1)).abs() < 0.001 )
}

pub fn project_dubins_car_state_to_config( states: States3D ) -> States3D {
    states
}
