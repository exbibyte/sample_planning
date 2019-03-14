extern crate pretty_env_logger;

use std::collections::HashSet;

use crate::rrt::rrt::RRT;
use crate::states::States;
use crate::control::Control;
use crate::planner_param::{Param,ParamObstacles};

// struct Node<TS> {
//     state: TS,
//     children: HashSet<i32>,
// }

// struct Edge {
//     control: 
// }

pub struct RRT_Base<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    pub param: Param<TS,TC,TObs>,
    //todo
    pub obstacles: ParamObstacles<TObs>,
    // memory_nodes: Vec< Node<TS> >,
    // memory_edges: Vec< Edge<TS> >,
}

impl <TS,TC,TObs> RRT < TS,TC,TObs > for RRT_Base<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    fn init( param: & Param<TS,TC,TObs>, obstacles: & ParamObstacles<TObs> ) -> Self {
        //todo process obstacles...
        Self {
            param: param.clone(),
            obstacles: obstacles.clone(),
            // memory_nodes: vec![],
            // memory_edge: vec![],
        }
    }
    
    fn iterate( & mut self, states_cur: TS ) -> bool {
        info!("iterating");

        
        for _ in 0..1000 {
            // let last_config_state = TObs;
            // if self.param.stop_cond( last_config_state, param.
            //sample environment space
            //get nearest node
            //extent node via control toward sample
            //test extended edge and node for collision with environment
            //add to tree if collision free -> need to use a collision detector
            //possible prune less efficient paths in the tree according to cost function by wiring
            
        }
        
        true
    }
    
    fn get_best_trajectory( & self ) -> Option<Vec<TS>> {
        unimplemented!();
    }
}
