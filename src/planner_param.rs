use std::marker::PhantomData;

use crate::states::States;
use crate::control::Control;

extern crate mazth;

use zpatial::mazth::i_shape::IShape;
use zpatial::mazth::rbox::RecBox;

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
    pub dist_delta: f32, //to be used as tree edge expansion resolution
    pub param_sampler: fn()->C, //sampling in parameter space
    pub ss_sampler: fn()->T, //sampleing in state space
    pub ss_metric: fn(T,T)->f32, //distance function in state space
    
}

#[derive(Clone,Debug)]
pub struct ParamObstacles <T> where T: States {
    pub obstacles: Vec<RecBox>,    
    pub states_info: PhantomData<T>,
}
