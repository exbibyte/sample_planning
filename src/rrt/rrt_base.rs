//! RRT Base
//! implementation samples the parameter space for tree explansion

extern crate pretty_env_logger;

use std::collections::{HashSet,HashMap};

use rand::Rng;

use crate::rrt::rrt::RRT;
use crate::states::States;
use crate::control::Control;
use crate::planner_param::{Param,ParamObstacles};

use zpatial::implement::bvh_median::Bvh;
use zpatial::interface::i_spatial_accel::ISpatialAccel;
use zpatial::mazth::{
    i_bound::IBound,
    i_shape::ShapeType,
    bound::AxisAlignedBBox,
    bound_sphere::BoundSphere,
};

pub struct Node<TS> {
    pub id: usize,
    pub state: TS,
    pub children: HashSet<usize>,
}

pub struct Edge <TC> {
    pub control: TC,
}

pub struct RRT_Base<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    pub param: Param<TS,TC,TObs>,
    pub obstacles: Bvh<usize>,
    pub nodes: Vec< Node<TS> >,
    pub edges: HashMap< (usize,usize), Edge<TC> >,
}

impl <TS,TC,TObs> RRT_Base<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    pub fn new_node_id( & self ) -> usize {
        self.nodes.len()
    }
    pub fn get_trajectory_config_space( & self ) -> Vec<TObs> {
        self.nodes.iter()
            .map(|x| (self.param.project_state_to_config)(x.state.clone()) )
            .collect()
    }
    pub fn get_trajectory_edges_config_space( & self ) -> Vec<(TObs,TObs)> {
        self.edges.iter()
            .map(|x| {
                let id_a = (x.0).0;
                let id_b = (x.0).1;
                let state_a = &self.nodes[id_a].state;
                let state_b = &self.nodes[id_b].state;
                ( (self.param.project_state_to_config)(state_a.clone()),
                  (self.param.project_state_to_config)(state_b.clone()) )
            })
            .collect()
    }
    pub fn reached_goal( & self, states: TS ) -> bool {
        let config_states = (self.param.project_state_to_config)(states.clone());
        (self.param.stop_cond)( states, config_states, self.param.states_config_goal.clone() )
    }
}

impl <TS,TC,TObs> RRT < TS,TC,TObs > for RRT_Base<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    fn init( param: & Param<TS,TC,TObs>, obstacles: Bvh<usize>, obstacles_concrete: ParamObstacles<TObs> ) -> Self {
        //todo process obstacles...
        
        Self {
            param: param.clone(),
            obstacles: obstacles,
            nodes: vec![ Node { id: 0, state: param.states_init.clone(), children: HashSet::new() } ],
            edges: HashMap::new(),
        }
    }

    fn reset( & mut self ){

    }
    
    fn iterate( & mut self, states_cur: TS ) -> bool {
        
        //ignore obstacles for now
        
        self.nodes = vec![ Node { id: 0, state: self.param.states_init.clone(), children: HashSet::new() } ];
        self.edges = HashMap::new();
        
        for i in 0..25000 {
            let param_sample = (self.param.param_sampler)( self.param.sim_delta );
            
            let mut rng = rand::thread_rng();
            
            let state_select_id = rng.gen_range(0,self.nodes.len());
            
            let mut state_update = self.nodes[state_select_id].state.clone();
            
            state_update = (self.param.dynamics)( state_update, param_sample.clone(), self.param.sim_delta );

            let vals = &state_update.get_vals();
            if !self.obstacles.query_intersect_single( &AxisAlignedBBox::init( ShapeType::POINT, &[vals[0] as f64,vals[1] as f64,vals[2] as f64] ) ).unwrap().is_empty() {
                
                continue;
            }
            // info!("state update: {:?}", state_update );
            
            let id_new = self.new_node_id();
            self.nodes.push( Node {id: id_new, state: state_update.clone(), children: HashSet::new() } );

            self.nodes[state_select_id].children.insert(id_new);
            self.edges.insert( (state_select_id, id_new), Edge { control: param_sample } );
            
            // let last_config_state = TObs;
            // if self.param.stop_cond( last_config_state, param.
            //sample environment space
            //get nearest node
            //extent node via control toward sample
            //test extended edge and node for collision with environment
            //add to tree if collision free -> need to use a collision detector
            //possible prune less efficient paths in the tree according to cost function by wiring
            
            if self.reached_goal( state_update ) {
                info!("found a path to goal on iteration: {}", i );
                break;
            }
        }
        true
    }
    
    fn get_best_trajectory( & self ) -> Option<Vec<TS>> {
        unimplemented!();
    }
}
