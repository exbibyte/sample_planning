pub enum StopCondition {
    Iterations(i32),
    TimeMilliSeconds(i32),
}

pub struct Param <States> {
    // pub memory_limit: Option<i32>,
    // pub stop_cond: StopCondition,
    // pub states_init: States,
    // pub dynamics: Box<Fn(States,f32)->States>,
    pub states_goal: States,
}
