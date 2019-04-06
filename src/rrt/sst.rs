//! Stable Sparse RRT

extern crate pretty_env_logger;

use std::collections::{HashSet,HashMap};
use std::marker::PhantomData;

use rand::Rng;

use crate::rrt::rrt::RRT;
use crate::states::States;
use crate::control::Control;
use crate::planner_param::{Param,ParamObstacles};

use super::nn_naive::NN_Naive;

use zpatial::implement::bvh_median::Bvh;
use zpatial::interface::i_spatial_accel::ISpatialAccel;
use zpatial::mazth::{
    i_bound::IBound,
    i_shape::ShapeType,
    bound::AxisAlignedBBox,
    bound_sphere::BoundSphere,
    point::Point3,
    line::Line3,
};

use zpatial::mazth::i_shape::IShape;
use zpatial::mazth::{rbox::RecBox,triprism::TriPrism};

use crate::planner_param::*;

pub struct Node<TS> {
    
    ///current node index
    pub id: usize,

    ///state space value
    pub state: TS,

    ///child node indices
    pub children: HashSet<(usize)>,
    
    ///cost in terms of number of steps from the root of the propagation tree
    pub cost: u32,
}

pub struct Edge <TC> {
    pub control: TC,
}

pub struct SST<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    
    pub param: Param<TS,TC,TObs>,
    
    pub obstacles: Bvh<usize>, //bvh contain indices to obstacles in obstacles_actual

    pub obstacles_actual: ParamObstacles<TObs>,

    pub nodes: Vec< Node<TS> >,
    pub witnesses: Vec<TS>,
    pub witness_representative: HashMap< usize, usize >,
    
    ///free slots in nodes for future node initialization
    pub nodes_freelist: Vec<usize>,
    
    ///extra info useful for tree pruning
    pub nodes_active: HashSet< usize >,
    pub nodes_inactive: HashSet< usize >,
    pub link_parent: HashMap< usize, usize >, //node -> node_parent

    ///storage for control input for the state space pair (parent node,child node)
    pub edges: HashMap< (usize,usize), Edge<TC> >,
    pub delta_v: f32,
    pub delta_s: f32,

    pub nn_query: NN_Naive<TS,TC,TObs>,

    pub stat_pruned_nodes: u32,
    pub stat_iter_no_change: u32,

    pub iter_exec: u32,

    pub are_obstacles_boxes: bool,
}

impl <TS,TC,TObs> SST<TS,TC,TObs> where TS: States, TC: Control, TObs: States {

    pub fn get_trajectory_config_space( & self ) -> Vec<TObs> {
        self.nodes.iter()
            .map(|x| (self.param.project_state_to_config)(x.state.clone()) )
            .collect()
    }

    ///returns pairs of (witness, witness representative)
    pub fn get_witness_representatives_config_space( & self ) -> Vec<(TObs,TObs)> {
        self.witness_representative.iter()
            .map( |(idx_witness,idx_repr)| {
                let state_witness = self.witnesses[*idx_witness].clone();
                let state_repr = self.nodes[*idx_repr].state.clone();
                ( (self.param.project_state_to_config)(state_witness),
                  (self.param.project_state_to_config)(state_repr) )
            })
            .collect()
    }
    
    pub fn get_trajectory_edges_config_space( & self ) -> Vec<(TObs,TObs)> {
        
        self.edges.iter()
            .filter(|x| {
                (self.nodes_active.contains(&(x.0).0) ||
                 self.nodes_inactive.contains(&(x.0).0)) &&
                (self.nodes_active.contains(&(x.0).1) ||
                 self.nodes_inactive.contains(&(x.0).1)) })
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

impl <TS,TC,TObs> RRT < TS,TC,TObs > for SST<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    fn init( param: & Param<TS,TC,TObs>, obstacles: Bvh<usize>, obstacles_concrete: ParamObstacles<TObs> ) -> Self {
        //todo process obstacles...

        let box_obstacles = match obstacles_concrete.obstacles {
            ObsVariant::RBOX(_) => true,
            _ => false
        };
        
        Self {
            
            are_obstacles_boxes: box_obstacles,
            
            param: param.clone(),
            obstacles: obstacles,
            obstacles_actual: obstacles_concrete,
            nodes: vec![ Node { id: 0,
                                state: param.states_init.clone(),
                                children: HashSet::new(),
                                cost: 0 } ],

            nodes_freelist: vec![],
            witnesses: vec![],
            witness_representative: HashMap::new(),
            edges: HashMap::new(),
            delta_v: 0.008,
            delta_s: 0.0015,
            
            nodes_active: HashSet::new(),
            nodes_inactive: HashSet::new(),
            link_parent: HashMap::new(),

            nn_query: NN_Naive {
                phantom_ts: PhantomData,
                phantom_tc: PhantomData,
                phantom_tobs: PhantomData,
            },

            stat_pruned_nodes: 0,
            stat_iter_no_change: 0,

            iter_exec: param.iterations_bound,
        }
    }

    fn reset( & mut self ){
        
        self.nodes = vec![ Node { id: 0,
                                  state: self.param.states_init.clone(),
                                  children: HashSet::new(),
                                  cost: 0 } ];

        self.edges = HashMap::new();
        self.witnesses = vec![ self.param.states_init.clone() ];
        self.witness_representative.clear();
        self.nodes_active = HashSet::new();
        self.nodes_active.insert( 0 );
        self.nodes_inactive.clear();
        self.link_parent.clear();
        self.nodes_freelist.clear();
        self.stat_pruned_nodes = 0;
        self.stat_iter_no_change = 0;
        self.iter_exec = self.param.iterations_bound;
    }

    fn iterate( & mut self, states_cur: TS ) -> bool {
        
        self.reset();
        
        for i in 0..self.param.iterations_bound {
            // println!("iteration: {}", i );
            let ss_sample = (self.param.ss_sampler)(); //sampler for state space

            // dbg!(&ss_sample);

            //get best active state in vicinity delta_v of ss_sample, or return nearest active state
            let idx_state_best_nearest = self.nn_query.query_nearest_state_active( ss_sample.clone(),
                                                                                   & self.nodes,
                                                                                   & self.nodes_active,
                                                                                   & self.param,
                                                                                   self.delta_v );

            // dbg!(idx_state_best_nearest);

            //propagation with random delta within range of specified upper bound
            let mut rng = rand::thread_rng();
            let monte_carlo_prop_delta = rng.gen_range(0., 1.) * self.param.sim_delta;

            //sampler for control space
            let param_sample = (self.param.param_sampler)( monte_carlo_prop_delta );

            //propagate/simulate state dynamics
            //todo: collision checking with configuration space
            let state_propagate_cost = self.nodes[idx_state_best_nearest].cost + 1;
            let mut state_propagate = self.nodes[idx_state_best_nearest].state.clone();

            let config_space_coord_before = (self.param.project_state_to_config)(state_propagate.clone());
            
            state_propagate = (self.param.dynamics)( state_propagate,
                                                     param_sample.clone(),
                                                     monte_carlo_prop_delta );

            let config_space_coord_after = (self.param.project_state_to_config)(state_propagate.clone());

            let witness_idx = match self.nn_query.query_nearest_witness( state_propagate.clone(),
                                                                         & self.witnesses, 
                                                                         & self.param,
                                                                         self.delta_s ) {
                Some(idx) => { idx },
                _ => {
                    // println!("no witness found");
                    //no witness found within delta_s vicinity, so create a new witness
                    let idx_new = self.witnesses.len();
                    self.witnesses.push( state_propagate.clone() );
                    idx_new
                },
            };

            match self.witness_representative.get_mut( &witness_idx ) {
                Some(x) => {
                    if state_propagate_cost < self.nodes[*x].cost {

                        let v0 = &config_space_coord_before.get_vals();
                        let v1 = &config_space_coord_after.get_vals();

                        let query_line = Line3::init( &[v0[0] as _, v0[1] as _, v0[2] as _],
                                                      &[v1[0] as _, v1[1] as _, v1[2] as _] );

                        let candidate_collisions = self.obstacles.query_intersect( &query_line._bound ).unwrap();
                        
                        let collision = if candidate_collisions.is_empty() {
                            false
                        }else{
                            match self.obstacles_actual.obstacles {
                                ObsVariant::TRIPRISM(ref x) => {
                                    //narrow stage collision test for tri prisms
                                    candidate_collisions.iter().any(|idx| x[*idx].get_intersect( &query_line ).0 )
                                },
                                _ => { true }, //box same as aabb box
                            }
                        };
                        
                        if collision {
                               self.stat_iter_no_change += 1;
                               continue;
                        }
                        
                        let node_inactive : usize = *x;
                        self.nodes_active.remove(&node_inactive);
                        self.nodes_inactive.insert(node_inactive);

                        //use freelist if possible
                        let idx_node_new = match self.nodes_freelist.pop() {
                            Some(slot) => {
                                self.nodes[slot] = Node { id: slot,
                                                          state: state_propagate.clone(),
                                                          children: HashSet::new(),
                                                          cost: state_propagate_cost };
                                slot
                            },
                            _ => {
                                let idx_node_new = self.nodes.len();
                                self.nodes.push( Node { id: idx_node_new,
                                                        state: state_propagate.clone(),
                                                        children: HashSet::new(),
                                                        cost: state_propagate_cost } );
                                idx_node_new
                            },
                        };
                        assert!(node_inactive != idx_node_new);
                        
                        *x = idx_node_new; //save new representative state idx for current witness
                        self.nodes_active.insert(idx_node_new);

                        self.nodes[idx_state_best_nearest].children.insert(idx_node_new);
                       
                        self.link_parent.insert( idx_node_new, idx_state_best_nearest );

                        self.edges.insert( (idx_state_best_nearest, idx_node_new), Edge { control: param_sample } );
                        
                        // //remove leaf nodes and branches from propagation tree if possible
                        let mut node_prune = node_inactive;
                        
                        loop {
                            if self.nodes[ node_prune ].children.is_empty() &&
                                !self.nodes_active.contains( & node_prune ) {
                                    
                                self.nodes_inactive.remove( & node_prune );
                                self.nodes_freelist.push( node_prune );
                                let parent_idx = match self.link_parent.get( & node_prune ){
                                    Some(par) => { *par },
                                    _ => {
                                        break;
                                    },
                                };
                                self.link_parent.remove( & node_prune );
                                self.edges.remove( &(parent_idx, node_prune) );
                                self.nodes[ parent_idx ].children.remove( & node_prune );
                                node_prune = parent_idx;

                                self.stat_pruned_nodes += 1;
                                    
                            } else {
                                break;
                            }
                        }
                    }else{
                        self.stat_iter_no_change += 1;
                    }
                },
                _ => {
                    //no representative found, so just add the propagated state as representative

                    let v0 = &config_space_coord_before.get_vals();
                    let v1 = &config_space_coord_after.get_vals();

                    let query_line = Line3::init( &[v0[0] as _, v0[1] as _, v0[2] as _],
                                                    &[v1[0] as _, v1[1] as _, v1[2] as _] );

                    let candidate_collisions = self.obstacles.query_intersect( &query_line._bound ).unwrap();
                    
                    let collision = if candidate_collisions.is_empty() {
                        false
                    }else{
                        match self.obstacles_actual.obstacles {
                            ObsVariant::TRIPRISM(ref x) => {
                                //narrow stage collision test for tri prisms
                                candidate_collisions.iter().any(|idx| x[*idx].get_intersect( &query_line ).0 )
                            },
                            _ => { true }, //box same as aabb box
                        }
                    };
                    
                    if collision {
                        self.stat_iter_no_change += 1;
                        continue;
                    }
                    
                    //use freelist if possible
                    let idx_node_new = match self.nodes_freelist.pop() {
                        Some(slot) => {
                            self.nodes[slot] = Node { id: slot,
                                                      state: state_propagate.clone(),
                                                      children: HashSet::new(),
                                                      cost: state_propagate_cost };
                            slot
                        },
                        _ => {
                            let idx_node_new = self.nodes.len();
                            self.nodes.push( Node { id: idx_node_new,
                                                    state: state_propagate.clone(),
                                                    children: HashSet::new(),
                                                    cost: state_propagate_cost } );
                            idx_node_new
                        },
                    };

                    self.witness_representative.insert( witness_idx, idx_node_new ); //save new representative state idx for current witness
                    self.nodes_active.insert(idx_node_new);

                    self.nodes[idx_state_best_nearest].children.insert(idx_node_new);

                    self.link_parent.insert( idx_node_new, idx_state_best_nearest );

                    self.edges.insert( (idx_state_best_nearest, idx_node_new), Edge { control: param_sample } );
                    
                    //no node is made inactive, so pruning necessary
                },
            }
            
            if self.reached_goal( state_propagate ) {
                info!("found a path to goal on iteration: {}", i );
                self.iter_exec = i;
                break;
            }
        }
        self.print_stats();
        true
    }
    
    fn get_best_trajectory( & self ) -> Option<Vec<TS>> {
        unimplemented!();
    }

    fn print_stats( &self ){
        info!( "nodes: {}", self.nodes.len() );
        info!( "nodes active: {}", self.nodes_active.len() );
        info!( "nodes inactive: {}", self.nodes_inactive.len() );
        info!( "pruned_nodes: {}", self.stat_pruned_nodes );
        info!( "nodes freelist: {}", self.nodes_freelist.len() );
        info!( "iterations no change: {}/{}", self.stat_iter_no_change, self.iter_exec );
    }
}
