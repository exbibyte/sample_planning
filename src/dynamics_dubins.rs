//! dynamics, constraints for dubins car
//!
//! states := [ x,y,theta ]
//! control := u
//! x' = Vcos(theta)
//! y' = Vsin(theta)
//! theta' = u

use crate::states::*;
use crate::control::*;
use crate::planner_param::Param;
use rand::Rng;

///load model info to the caller
pub fn load_model() -> Param<States3D, Control1D, States3D> {
    Param {
        states_init: States3D([0.68, 0.3, 0.]), //default
        // states_init: States3D([0.3, 0.1, 0.]), //default
        // states_config_goal: States3D([0.65,0.85,0.]), //default
        states_config_goal: States3D([0.4,0.65,0.]), //default
        dynamics: dynamics,
        stop_cond: stop_cond,
        project_state_to_config: project_state_space_to_config_space,
        param_sampler: sampler_parameter_space,
        ss_sampler: sampler_state_space,
        ss_metric: statespace_distance,
        sim_delta: 0.05f32, //default
        iterations_bound: 300_000, //default, to be override by caller
    }
}

pub fn dynamics( states: States3D, control: Control1D, delta: f32 )-> States3D {
    
    use std::f32::consts::PI;
    
    let x_dot = states.0[2].cos();
    let y_dot = states.0[2].sin();
    let theta_dot = control.0[0];
    States3D( [ states.0[0] + x_dot * delta,
                states.0[1] + y_dot * delta,
                ( (states.0[2] + theta_dot * delta ) + 2.*PI ) % (2.*PI) ] )
}

///project x and y
pub fn project_state_space_to_config_space( states: States3D ) -> States3D {
    States3D( [states.0[0], states.0[1], 0.] )
}

pub fn sampler_parameter_space( delta: f32 ) -> Control1D {
    
    use std::f32::consts::PI;
    
    let mut rng = rand::thread_rng();
    
    Control1D( [ (rng.gen_range(-1., 1.)*20./180.* PI )/delta ] )
}

pub fn sampler_state_space() -> States3D {
    
    use std::f32::consts::PI;
    
    let mut rng = rand::thread_rng();

    States3D( [ rng.gen_range(0., 1.), //[0,1] range for x
                rng.gen_range(0., 1.), //[0,1] range for y
                rng.gen_range(0., 2. * PI) ] ) //[0,2*PI] for theta
}

///aritrary goal condition
pub fn stop_cond( system_states: States3D, states_config: States3D, states_config_goal: States3D )-> bool {
    states_config.0.iter()
        .zip( states_config_goal.0.iter() )
        .all( |x| ((x.0)-(x.1)).abs() < 0.002 )
}

pub fn statespace_distance( a: States3D, b: States3D ) -> f32 {

    use std::f32::consts::PI;
    
    let va = a.get_vals();
    let vb = b.get_vals();

    let mut ret = 0.;

    //todo: check reference on suitable metric for rigid body orientation
    
    for i in 0..2{
        ret += (va[i] - vb[i])*(va[i] - vb[i]);
    }

    let angle = (va[2] - vb[2]);
    
    let angle_1 = (va[2] + 2.*PI)%(2.*PI);
    let angle_2 = (vb[2] + 2.*PI)%(2.*PI);
    ret += (angle_1-angle_2)*(angle_1-angle_2);

    ret
}
