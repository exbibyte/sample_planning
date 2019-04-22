use std::marker::PhantomData;
use std::fmt;

use crate::states::States;
use crate::control::Control;

extern crate mazth;

use zpatial::mazth::i_shape::IShape;
use zpatial::mazth::{rbox::RecBox,triprism::TriPrism};

pub enum StopCondition {
    Iterations(i32),
    TimeMilliSeconds(i32),
}

#[derive(Clone,Debug)]
pub struct Param <T, C, TObs> where T: States, C: Control, TObs: States {
    // pub memory_limit: Option<i32>,
    pub stop_cond: fn(T/*system state*/,TObs/*config state*/,T/*desired state*/)->bool,
    pub states_init: T,
    pub states_goal: T,
    pub dynamics: fn(T,C,f32)->T, //uses state space of system
    pub project_state_to_config: fn(T)->TObs,
    pub sim_delta: f32, //to be used as simulation step size
    pub param_sampler: fn(f32)->C, //sampling in parameter space
    pub ss_sampler: fn()->T, //sampleing in state space
    pub ss_metric: fn(T,T)->f32, //distance function in state space
    pub cs_metric: fn(TObs, TObs) -> f32, //estimated cloness ness in configuration space
    pub iterations_bound: u32,

    //optional, but would ggive error if running motion_primitives feature without providing functions
    pub motion_primitive_xform: Option<fn(T,T)->T>,
    pub motion_primitive_xform_inv: Option<fn(T,T)->T>,
}

#[derive(Clone,Debug)]
pub enum ObsVariant {
    RBOX(Vec<RecBox>),
    TRIPRISM(Vec<TriPrism>),
}

#[derive(Clone,Debug)]
pub struct ParamObstacles <T> where T: States {
    pub obstacles: ObsVariant,
    pub states_info: PhantomData<T>,
}

impl<T,C,TObs> fmt::Display for Param <T, C, TObs> where T: States, C: Control, TObs: States  {
   
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {

        f.debug_struct("Param")
            .field("states_init", &format!("{:?}",&self.states_init) )
            .field("states_goal", &format!("{:?}",&self.states_goal) )
            .field("sim_delta", &self.sim_delta )
            .field("iterations_bound", &self.iterations_bound )
            .finish()
    }
}


#[derive(Clone,Debug)]
pub struct ParamTree {
    pub delta_v: f32,
    pub delta_s: f32,
    pub prop_delta_low: f32,
    pub prop_delta_high: f32,
}

impl Default for ParamTree {
    fn default() -> Self {
        ParamTree {
            delta_v: 0.01,
            delta_s: 0.001,
            prop_delta_low: 0.15,
            prop_delta_high: 1.,
        }       
    }
}

impl fmt::Display for ParamTree {
   
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        
        f.debug_struct("ParamTree")
            .field("delta_v", &format!("{:?}",&self.delta_v) )
            .field("delta_s", &format!("{:?}",&self.delta_s) )
            .field("prop_delta_low", &format!("{:?}",&self.prop_delta_low) )
            .field("prop_delta_high", &format!("{:?}",&self.prop_delta_high) )
            .finish()
    }
}
