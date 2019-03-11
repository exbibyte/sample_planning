use crate::planner_param::Param;
// use crate::states::States;

pub trait Planner <States> {
    // fn init( param: Param<States> ) -> Planner<States> where Self: Sized;
    fn plan_iteration( & mut self, iteration: u64, time: u64 ) -> bool;
    fn get_trajectories_new( & self ) -> &[[f32;3]];
    fn get_trajectories_past( & self ) -> &[[f32;3]];
    fn get_trajectories_optimal( & self ) -> &[[f32;3]];
    fn get_trajectories( & self ) -> &[[f32;3]];
    // fn get_stats() -> Stats;
}
