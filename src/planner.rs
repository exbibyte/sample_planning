use crate::planner_param::Param;
use crate::states::States;
use crate::control::Control;

pub trait Planner <TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    fn plan_iteration( & mut self, iteration: Option<u32> ) -> bool;
    fn get_trajectories( & self ) -> &[TObs];
    fn get_trajectories_edges( & self ) -> &[((TObs,TObs),u32)];
    fn get_trajectory_best_edges( & self ) -> &[((TObs,TObs),u32)];
    fn get_param( & self ) -> Param<TS,TC,TObs>;
    fn get_states_current( & self ) -> Option<TS>;
    fn get_witness_pairs( & self ) -> &[(TObs,TObs)];
    fn get_trajectories_mo_prim_candidates( & self ) -> &[(TObs,TObs)];
}
