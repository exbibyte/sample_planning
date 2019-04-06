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
    pub stop_cond: fn(T/*system state*/,TObs/*config state*/,TObs/*desired configuration*/)->bool,
    pub states_init: T,
    pub states_config_goal: TObs,//todo: also include state space variables as goals?
    pub dynamics: fn(T,C,f32)->T, //uses state space of system
    pub project_state_to_config: fn(T)->TObs,
    pub sim_delta: f32, //to be used as simulation step size
    pub param_sampler: fn(f32)->C, //sampling in parameter space
    pub ss_sampler: fn()->T, //sampleing in state space
    pub ss_metric: fn(T,T)->f32, //distance function in state space
    pub iterations_bound: u32,
    
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
            .field("states_config_goal", &format!("{:?}",&self.states_config_goal) )
            .field("sim_delta", &self.sim_delta )
            .field("iterations_bound", &self.iterations_bound )
            .finish()
    }
}
