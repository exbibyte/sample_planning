use crate::states::States;
use crate::control::Control;

use crate::planner_param::{Param,ParamObstacles};

use zpatial::implement::bvh_median::Bvh;


pub trait RRT < TS, TC, TObs > where TS: States, TC: Control, TObs: States {
    fn init( param: & Param<TS,TC,TObs>, obstacles: Bvh<usize> ) -> Self;
    fn iterate( & mut self, states_cur: TS ) -> bool;
    fn get_best_trajectory( & self ) -> Option<Vec<TS>>;
    fn reset( & mut self );
}

