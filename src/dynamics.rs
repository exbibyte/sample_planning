use crate::states::*;
use crate::control::*;

use rand::Rng;

// pub fn dynamics_1d( states: States1D, delta: f32 )-> States1D {
//     let a = (-1.*delta).exp();
//     States1D(states.0 * a)
// }

// pub fn stop_cond_1d( states: States1D, states_goal: States1D )-> bool {
//     (states.0 - states_goal.0).abs() < 0.00001
// }

/// dubins car
/// states := [ x,y,theta ]
/// control := u
/// x' = Vcos(theta)
/// y' = Vsin(theta)
/// theta' = u
pub fn dynamics_3d_1d( states: States3D, control: Control1D, delta: f32 )-> States3D {
    let V = 1.;
    let x_dot = V * states.0[2].cos();
    let y_dot = V * states.0[2].sin();
    let theta_dot = control.0[0];
    States3D( [ states.0[0] + x_dot * delta,
                states.0[1] + y_dot * delta,
                states.0[2] + theta_dot * delta ] )
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

///project x and y
pub fn project_dubins_car_state_to_config( states: States3D ) -> States3D {
    States3D( [states.0[0], states.0[1], 0.] )
}

pub fn sampler_parameter_space_dubins_car() -> Control1D {
    
    use std::f32::consts::PI;
    
    let mut rng = rand::thread_rng();
    
    Control1D( [180.*rng.gen_range(-1., 1.)/180.* PI] )
}

// pub fn sampler_state_space_dubins_car() -> States3D {
    
//     use std::f32::consts::PI;
    
//     let mut rng = rand::thread_rng();

//     let mut rng = rand::thread_rng();
//     Control1D( [20.*rng.gen_range(-1., 1.)/180.* PI] )
// }
