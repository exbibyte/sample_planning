use crate::planner_param::Param;
use crate::states::States;

pub trait Planner <TS> where TS: States {
    fn plan_iteration( & mut self, iteration: u64, time: u64 ) -> (bool,bool);
    fn get_trajectories_new( & self ) -> &[[f32;3]];
    fn get_trajectories_past( & self ) -> &[[f32;3]];
    fn get_trajectories_optimal( & self ) -> &[[f32;3]];
    fn get_trajectories( & self ) -> &[[f32;3]];
    fn get_param( & self ) -> Param<TS>;
    fn get_states_current( & self ) -> Option<TS>;
    // fn get_stats() -> Stats;
}
