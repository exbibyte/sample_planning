use crate::states::States;

pub enum StopCondition {
    Iterations(i32),
    TimeMilliSeconds(i32),
}

#[derive(Clone)]
pub struct Param <T> where T: States {
    // pub memory_limit: Option<i32>,
    pub stop_cond: fn(T,T)->bool,
    pub states_init: T,
    pub states_goal: T,
    pub dynamics: fn(T,f32)->T,
}
