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

extern crate mazth;
use mazth::mat;

// const config_space_goal : States3D = States3D([0.45,0.65,0.]);
const config_space_goal : States3D = States3D([0.6,0.1,0.]);

///load model info to the caller
pub fn load_model() -> Param<States3D, Control1D, States3D> { //state space and configuration space both are 3 dimensional in this case
    Param {
        // states_init: States3D([0.68, 0.3, 0.]), //default
        states_init: States3D([0.1, 0.1, 0.]), //default
        states_config_goal: config_space_goal, //default
        dynamics: dynamics,
        stop_cond: stop_cond,
        cs_metric: config_space_distance,
        project_state_to_config: project_state_space_to_config_space,
        param_sampler: sampler_parameter_space,
        ss_sampler: sampler_state_space,
        ss_metric: statespace_distance,
        // sim_delta: 0.05f32, //default
        sim_delta: 0.05f32, //default
        iterations_bound: 300_000, //default, to be override by caller

        //motion primitive transform functions
        motion_primitive_xform: Some(motion_primitive_xform),
        motion_primitive_xform_inv: Some(motion_primitive_xform_inv),

        ss_goal_gen: statespace_goal_generator,
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
        // .all( |x| ((x.0)-(x.1)).abs() < 0.002 )
        .all( |x| ((x.0)-(x.1)).abs() < 0.01 )
}

///estimate of closeness to goal condition in configuration space
pub fn config_space_distance( states_config: States3D, states_config_goal: States3D )-> f32 {
    //in this case, just use ss distance metric
    statespace_distance( states_config, states_config_goal )
}

///generate a possible state space valuation that satisfies the goal
pub fn statespace_goal_generator() -> States3D {
    //in this just case, just use the same value as configuration goal state since they map 1:1 with state space
    config_space_goal
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

/// map ``q_end`` to coordinate frame of canonical motion primitive lookup space,
/// where ``q_start`` of the original space is transformed to the origin of the canonical motion primitive lookup space.
pub fn motion_primitive_xform( q_start: States3D, q_end: States3D ) -> States3D {

    //translate by -x, -y of q_start

    //m_translate =[ 1 0 0 -x
    //               0 1 0 -y
    //               0 0 1  0
    //               0 0 0  1 ]

    use mat::*;
    
    let mut m_translate : Mat4<f32> = Mat4::default();

    
    *m_translate.index_mut( 0, 0 ) = 1.;
    *m_translate.index_mut( 1, 1 ) = 1.;
    *m_translate.index_mut( 2, 2 ) = 1.;
    *m_translate.index_mut( 3, 3 ) = 1.;

    *m_translate.index_mut( 0, 3 ) = -q_start.0[0];
    *m_translate.index_mut( 1, 3 ) = -q_start.0[1];

    //rotate by -angle of q_start
    //m_rot =[ cos(angle)  sin(angle)  0   0
    //         -sin(angle) cos(angle)  0   0
    //             0           0       1 -angle
    //             0           0       0   1    ]
    
    let mut m_rot : Mat4<f32> = Mat4::default();

    let angle = q_start.0[2];
    
    *m_rot.index_mut( 0, 0 ) = angle.cos();
    *m_rot.index_mut( 1, 1 ) = angle.cos();

    *m_rot.index_mut( 0, 1 ) = angle.sin();
    *m_rot.index_mut( 1, 0 ) = -angle.sin();

    *m_rot.index_mut( 2, 2 ) = 1.;
    *m_rot.index_mut( 2, 3 ) = -angle;
    *m_rot.index_mut( 3, 3 ) = 1.;

    let mut q : Mat4x1<f32> = Mat4x1::default();
    q[0] = q_end.0[0];
    q[1] = q_end.0[1];
    q[2] = q_end.0[2];
    q[3] = 1.;
    
    //rot( translate( q ) )
    
    let qq = m_rot.mul_mat4x1( & m_translate.mul_mat4x1( &q ).unwrap() ).unwrap();

    States3D( [ qq[0], qq[1], qq[2] ] )
}

/// map ``qq_end`` from motion primitive lookup space back to the original space,
/// where ``q_start`` of the original space is transformed to the origin of the canonical motion primitive lookup space.
pub fn motion_primitive_xform_inv( q_start: States3D, qq_end: States3D ) -> States3D {

    use mat::*;

    //rotate by angle of q_start
    //m_rot =[ cos(angle)  -sin(angle)  0   0
    //         sin(angle)   cos(angle)  0   0
    //             0           0        1 angle
    //             0           0        0   1    ]
    
    let mut m_rot : Mat4<f32> = Mat4::default();

    let angle = q_start.0[2];
    
    *m_rot.index_mut( 0, 0 ) = angle.cos();
    *m_rot.index_mut( 1, 1 ) = angle.cos();

    *m_rot.index_mut( 0, 1 ) = -angle.sin();
    *m_rot.index_mut( 1, 0 ) = angle.sin();

    *m_rot.index_mut( 2, 2 ) = 1.;
    *m_rot.index_mut( 2, 3 ) = angle;
    *m_rot.index_mut( 3, 3 ) = 1.;

    //translate by x, y of q_start
    
    //m_translate =[ 1 0 0 x
    //               0 1 0 y
    //               0 0 1 0
    //               0 0 0 1 ]

    let mut m_translate : Mat4<f32> = Mat4::default();

    *m_translate.index_mut( 0, 0 ) = 1.;
    *m_translate.index_mut( 1, 1 ) = 1.;
    *m_translate.index_mut( 2, 2 ) = 1.;
    *m_translate.index_mut( 3, 3 ) = 1.;

    *m_translate.index_mut( 0, 3 ) = q_start.0[0];
    *m_translate.index_mut( 1, 3 ) = q_start.0[1];
    
    let mut q : Mat4x1<f32> = Mat4x1::default();
    q[0] = qq_end.0[0];
    q[1] = qq_end.0[1];
    q[2] = qq_end.0[2];
    q[3] = 1.;

    //translate( rot( q ) )
    let q = m_translate.mul_mat4x1( & m_rot.mul_mat4x1( &q ).unwrap() ).unwrap();

    States3D( [ q[0], q[1], q[2] ] )
}

#[test]
fn test_motion_primitive_xform(){
    
    use std::f32::consts::PI;
    
    let q_start: States3D = States3D([4., 5., 0.5*PI]);
    let q_end: States3D = States3D([7., 5., 0.*PI]);
    
    let qq_end = motion_primitive_xform( q_start, q_end );

    let eps = 1e-6;
    dbg!(qq_end);
    assert!( qq_end.0[0] > 0. - eps && qq_end.0[0] < 0. + eps );
    assert!( qq_end.0[1] > -3. - eps && qq_end.0[1] < -3. + eps );
    assert!( qq_end.0[2] > -0.5*PI - eps && qq_end.0[2] < -0.5*PI + eps );
}

#[test]
fn test_motion_primitive_xform_inv(){
    
    use std::f32::consts::PI;
    
    let q_start: States3D = States3D([4., 5., 0.5*PI]);
    let qq_end: States3D = States3D([0., -3., -0.5*PI]);
    
    let q_end = motion_primitive_xform_inv( q_start, qq_end );

    let eps = 1e-6;
    dbg!(q_end);
    assert!( q_end.0[0] > 7. - eps && q_end.0[0] < 7. + eps );
    assert!( q_end.0[1] > 5. - eps && q_end.0[1] < 5. + eps );
    assert!( q_end.0[2] > 0.*PI - eps && q_end.0[2] < 0.*PI + eps );
}
