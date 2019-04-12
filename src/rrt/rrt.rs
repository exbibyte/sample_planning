use crate::states::States;
use crate::control::Control;

use crate::planner_param::{Param,ParamObstacles};

use zpatial::implement::bvh_median::Bvh;


pub trait RRT < TS, TC, TObs > where TS: States, TC: Control, TObs: States {
    fn iterate( & mut self, states_cur: TS ) -> bool;
    fn get_best_trajectory_config_space( & self ) -> Vec<((TObs,TObs),u32)>;
    fn reset( & mut self );
    fn print_stats( &self ){}
}

