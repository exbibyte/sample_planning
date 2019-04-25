use crate::states::States;
use crate::control::Control;

use crate::planner_param::{Param,ParamObstacles};

use zpatial::implement::bvh_median::Bvh;


pub trait RRT < TS, TC, TObs > where TS: States, TC: Control, TObs: States {
    ///returns true if iteration induces change, false otherwise
    fn iterate( & mut self, iteration: Option<u32> ) -> bool;
    fn get_best_trajectory_config_space( & self ) -> Vec<((TObs,TObs),u32)>;
    fn reset( & mut self );
    fn print_stats( &self ){}
    fn get_sampling_distr( & self ) -> Vec<TObs>;
}

