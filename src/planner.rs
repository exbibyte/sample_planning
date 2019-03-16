use crate::planner_param::Param;
use crate::states::States;
use crate::control::Control;

pub trait Planner <TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    fn plan_iteration( & mut self, iteration: u64, time: u64 ) -> (bool,bool);
    // fn get_trajectories_new( & self ) -> &[[f32;3]];
    // fn get_trajectories_past( & self ) -> &[[f32;3]];
    // fn get_trajectories_optimal( & self ) -> &[[f32;3]];
    fn get_trajectories( & self ) -> &[TObs];
    fn get_trajectories_edges( & self ) -> &[(TObs,TObs)];
    fn get_param( & self ) -> Param<TS,TC,TObs>;
    fn get_states_current( & self ) -> Option<TS>;
    // fn get_stats() -> Stats;
}
