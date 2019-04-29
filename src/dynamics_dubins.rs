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

use std::f32::consts::PI;

use std::ops::{Add,Mul};

impl Add for States3D {
    type Output = States3D;
    
    fn add( self, other:States3D ) -> States3D {
        
        use std::f32::consts::PI;
        
        States3D(
            [ self.0[0] + other.0[0],
              self.0[1] + other.0[1],
              ( self.0[2] + other.0[2] + 2.*PI ) % (2.*PI) ]
        )
    }
}

impl Mul<f32> for States3D {
    type Output = States3D;
    
    fn mul( self, other: f32 ) -> States3D {
        
        use std::f32::consts::PI;
        
        States3D(
            [ self.0[0] * other,
              self.0[1] * other,
              self.0[2] * other ]
        )
    }
}

fn ss_add( a: States3D, b: States3D ) -> States3D {
    a + b
}

fn ss_mul( a: States3D, b: f32 ) -> States3D {
    a * b
}

///load model info to the caller
pub fn load_model() -> Param<States3D, Control1D, States3D> { //state space and configuration space both are 3 dimensional in this case
    Param {
        states_init: States3D([0.5, 0.1, 0.]), //default, override by prob_instances.rs file
        states_goal: States3D([0.8,0.1,0.]), //default, override by prob_instances.rs file
        dynamics: dynamics,
        stop_cond: stop_cond,
        cs_metric: config_space_distance,
        project_state_to_config: project_state_space_to_config_space,
        param_sampler: sampler_parameter_space,
        ss_sampler: sampler_state_space,
        ss_metric: statespace_distance,
        sim_delta: 0.05f32, //default, optinal override by prob_instances.rs file
        iterations_bound: 300_000, //override via commandline and prob_instances.rs file

        //motion primitive transform functions
        motion_primitive_xform: Some(motion_primitive_xform),
        motion_primitive_xform_inv: Some(motion_primitive_xform_inv),

        ss_add: ss_add,
        ss_mul: ss_mul,
    }
}

///calculate change
fn dyn_change( states: States3D, control: Control1D, delta: f32 )-> States3D {
    
    use std::f32::consts::PI;
    
    let x_dot = states.0[2].cos();
    let y_dot = states.0[2].sin();
    let theta_dot = control.0[0];
    States3D( [ x_dot,
                y_dot,
                theta_dot ] )
}

///4th order runge-kutta
fn runge_kutta_4( states: States3D, control: Control1D, delta: f32 )-> States3D {

    use std::f32::consts::PI;
    
    let k1 = {
        
        let mut temp = dyn_change( states.clone(), control.clone(), 0.  );
        temp * delta
    };
    
    let k2 = {
        
        let mut k1_copy = k1.clone() * (1./2.);
        
        let mut temp = dyn_change( states.clone() + k1_copy, control.clone(), delta/2. );
        temp * delta
    };

    let k3 = {
        
        let mut k2_copy = k2.clone() * (1./2.);

        let mut temp = dyn_change( states.clone() + k2_copy, control.clone(), delta/2. );
        temp * delta
    };

    let k4 = {
        
        let mut temp = dyn_change( states.clone() + k3, control.clone(), delta );
        temp * delta
    };
    
    States3D( [ states.0[0] + 1./6.*( k1.0[0] + 2.*k2.0[0] + 2.*k3.0[0] + k4.0[0] ),
                states.0[1] + 1./6.*( k1.0[1] + 2.*k2.0[1] + 2.*k3.0[1] + k4.0[1] ),
                ( states.0[2] + ( 1./6.*( k1.0[2] + 2.*k2.0[2] + 2.*k3.0[2] + k4.0[2] ) ) + 2.*PI ) % ( 2.*PI ) ] )
}

pub fn dynamics( states: States3D, control: Control1D, delta: f32 )-> States3D {
    
    use std::f32::consts::PI;

    #[cfg(not(feature="runge_kutta"))]
    {
        // 1st order
        let temp = dyn_change( states, control, delta ) * delta;
        
        States3D( [ states.0[0] + temp.0[0],
                    states.0[1] + temp.0[1],
                    ( (states.0[2] + temp.0[2] ) + 2.*PI ) % (2.*PI) ] )
    }
    #[cfg(feature="runge_kutta")]
    {
        //4th order
        runge_kutta_4( states, control, delta )
    }
}

///project x and y
pub fn project_state_space_to_config_space( states: States3D ) -> States3D {
    States3D( [states.0[0], states.0[1], 0.] )
}

pub fn sampler_parameter_space( delta: f32 ) -> Control1D {
    
    use std::f32::consts::PI;

    use rand::prelude::*;
    use rand::distributions::Standard;

    let val: f32 = SmallRng::from_entropy().sample(Standard);
    
    Control1D( [ ( 2. * (val-0.5) * 40./180.* PI )/delta ] )
}

pub fn sampler_state_space() -> States3D {
    
    use std::f32::consts::PI;
    
    let mut rng = rand::thread_rng();

    States3D( [ rng.gen_range(0., 1.), //[0,1] range for x
                rng.gen_range(0., 1.), //[0,1] range for y
                rng.gen_range(0., 2. * PI) ] ) //[0,2*PI] for theta
}

///aritrary goal condition
pub fn stop_cond( mut system_states: States3D, mut states_config: States3D, mut states_goal: States3D )-> bool {

    system_states.0[2] = 0.;
    states_goal.0[2] = 0.;
    
    statespace_distance( system_states, states_goal ) < 0.01
}

///estimate of closeness to goal condition in configuration space
pub fn config_space_distance( states_config: States3D, states_config_goal: States3D )-> f32 {
    //in this case, just use ss distance metric
    
    use std::f32::consts::PI;
    
    let va = states_config.get_vals();
    let vb = states_config_goal.get_vals();

    let mut ret = 0.;

    //todo: check reference on suitable metric for rigid body orientation
    
    for i in 0..2{
        ret += (va[i] - vb[i])*(va[i] - vb[i]);
    }
    
    let angle = (va[2] - vb[2]);
    
    let angle_1 = ((va[2] + 2.*PI)%(2.*PI))/(2.*PI);
    let angle_2 = ((vb[2] + 2.*PI)%(2.*PI))/(2.*PI);
    ret += ((angle_1-angle_2)*(angle_1-angle_2));

    //ret
    ret.sqrt()
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
    
    // let angle = (va[2] - vb[2]);
    
    // let angle_1 = (va[2] + 2.*PI)%(2.*PI);
    // let angle_2 = (vb[2] + 2.*PI)%(2.*PI);
    // ret += (angle_1-angle_2)*(angle_1-angle_2);

    let angle_1 = ((va[2] + 2.*PI)%(2.*PI))/(2.*PI);
    let angle_2 = ((vb[2] + 2.*PI)%(2.*PI))/(2.*PI);
    
    ret += ((angle_1-angle_2)*(angle_1-angle_2));

    ret.sqrt()
    // ret
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

    use std::f32::consts::PI;
    
    States3D( [ qq[0], qq[1], ( qq[2] + 2. * PI ) % ( 2. * PI ) ] )
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

    use std::f32::consts::PI;
    
    States3D( [ q[0], q[1], ( q[2] + 2. * PI ) % ( 2. * PI ) ] )
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
