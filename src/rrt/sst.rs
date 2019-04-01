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
};

pub struct Node<TS> {
    pub id: usize,
    pub state: TS,
    pub children: HashSet<(usize)>,
    pub active_descendents: u32,
    pub cost: u32,
}

pub struct Edge <TC> {
    pub control: TC,
}

pub struct SST<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    pub param: Param<TS,TC,TObs>,
    pub obstacles: Bvh<usize>,//todo

    pub nodes: Vec< Node<TS> >,
    pub witnesses: Vec<TS>,
    pub witness_representative: HashMap< usize, usize >,
    pub nodes_freelist: Vec<usize>, //saves list of indices to nodes
    
    //topology info for the graph
    pub nodes_active: HashSet< usize >,
    pub nodes_inactive: HashSet< usize >,
    pub link_parent: HashMap< usize, usize >, //node -> node_parent
    // pub link_child: HashMap< usize, HashSet< usize > >,
    // pub link_child_active: HashMap< usize, HashSet< usize > >,
    
    pub edges: HashMap< (usize,usize), Edge<TC> >,
    pub delta_v: f32,
    pub delta_s: f32,

    pub nn_query: NN_Naive<TS,TC,TObs>,

    pub stat_pruned_nodes: u32,
}

impl <TS,TC,TObs> SST<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    // pub fn new_node_id( & self ) -> usize {
    //     self.nodes.len()
    // }
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
    fn init( param: & Param<TS,TC,TObs>, obstacles: Bvh<usize> ) -> Self {
        //todo process obstacles...
        
        Self {
            param: param.clone(),
            obstacles: obstacles,
            nodes: vec![ Node { id: 0,
                                state: param.states_init.clone(),
                                children: HashSet::new(),
                                active_descendents: 0,
                                cost: 0 } ],

            nodes_freelist: vec![],
            witnesses: vec![],
            witness_representative: HashMap::new(),
            edges: HashMap::new(),
            delta_v: 0.02,
            delta_s: 0.005,
            
            nodes_active: HashSet::new(),
            nodes_inactive: HashSet::new(),
            link_parent: HashMap::new(),
            // link_child: HashMap::new(),
            // link_child_active: HashMap::new(),

            nn_query: NN_Naive {
                phantom_ts: PhantomData,
                phantom_tc: PhantomData,
                phantom_tobs: PhantomData,
            },

            stat_pruned_nodes: 0,
        }
    }

    fn reset( & mut self ){
        
        self.nodes = vec![ Node { id: 0,
                                  state: self.param.states_init.clone(),
                                  children: HashSet::new(),
                                  active_descendents: 0,
                                  cost: 0 } ];

        self.edges = HashMap::new();
        self.witnesses = vec![ self.param.states_init.clone() ];
        self.nodes_active = HashSet::new();
        self.nodes_active.insert( 0 );
        self.nodes_inactive.clear();
        self.link_parent.clear();
        self.nodes_freelist.clear();
        // self.link_child.clear();
        // self.link_child_active.clear();
        
    }

    fn iterate( & mut self, states_cur: TS ) -> bool {
        
        self.reset();

        for i in 0..125_000 {
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
            
            let param_sample = (self.param.param_sampler)(); //sampler for control space

            //propagate/simulate state dynamics
            //todo: collision checking with configuration space
            let state_propagate_cost = self.nodes[idx_state_best_nearest].cost + 1;
            let mut state_propagate = self.nodes[idx_state_best_nearest].state.clone();
            for i in 0 .. (self.param.dist_delta / self.param.sim_delta) as usize {
                state_propagate = (self.param.dynamics)( state_propagate, param_sample.clone(), self.param.sim_delta );
            }
            
            // let vals = &state_update.get_vals();
            // if !self.obstacles.query_intersect_single( &AxisAlignedBBox::init( ShapeType::POINT, &[vals[0] as f64,vals[1] as f64,vals[2] as f64] ) ).unwrap().is_empty() {
                
            //     continue;
            // }
            // dbg!(&state_propagate);

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
                        let node_inactive = *x;
                        self.nodes_active.remove(x);
                        self.nodes_inactive.insert(*x);

                        //use freelist if possible
                        let idx_node_new = match self.nodes_freelist.pop() {
                            Some(slot) => {
                                self.nodes[slot] = Node { id: slot,
                                                          state: state_propagate.clone(),
                                                          children: HashSet::new(),
                                                          active_descendents: 0,
                                                          cost: state_propagate_cost };
                                slot
                            },
                            _ => {
                                let idx_node_new = self.nodes.len();
                                self.nodes.push( Node { id: idx_node_new,
                                                        state: state_propagate.clone(),
                                                        children: HashSet::new(),
                                                        active_descendents: 0,
                                                        cost: state_propagate_cost } );
                                idx_node_new
                            },
                        };
                        
                        // let idx_node_new = self.nodes.len();
                        
                        *x = idx_node_new; //save new representative state idx for current witness
                        self.nodes_active.insert(idx_node_new);
                        // self.nodes.push( Node { id: idx_node_new,
                        //                         state: state_propagate.clone(),
                        //                         children: HashSet::new(),
                        //                         active_descendents: 0,
                        //                         cost: state_propagate_cost } );

                        self.nodes[idx_state_best_nearest].children.insert(idx_node_new);
                       
                        self.link_parent.insert( idx_node_new, idx_state_best_nearest );

                        self.edges.insert( (idx_state_best_nearest, idx_node_new), Edge { control: param_sample } );

                        //increment active descendent count
                        let mut cur = idx_node_new;
                        loop {
                            match self.link_parent.get(&cur) {
                                Some(par) => {
                                    self.nodes[*par].active_descendents += 1;
                                    cur = *par;
                                },
                                _ => { break; },
                            }
                        }
                        
                        //remove leaf nodes and branches from propagation tree if possible
                        let mut node_prune = node_inactive;
                        loop {
                            if self.nodes[ node_prune ].active_descendents == 0 {
                                
                                self.nodes_inactive.remove( & node_prune );
                                self.nodes_freelist.push( node_prune );
                                let par = match self.link_parent.get( & node_prune ){
                                    Some(par) => { *par },
                                    _ => {break;},
                                };
                                self.edges.remove( &(par, node_prune) );
                                self.link_parent.remove( & node_prune );
                                self.nodes[ par ].children.remove( & node_prune );
                                node_prune = par;

                                self.stat_pruned_nodes += 1;

                            } else {
                                break;
                            }
                        }
                    }
                },
                _ => {
                    //no representative found, so just add the propagated state as representative

                    //use freelist if possible
                    let idx_node_new = match self.nodes_freelist.pop() {
                        Some(slot) => {
                            self.nodes[slot] = Node { id: slot,
                                                      state: state_propagate.clone(),
                                                      children: HashSet::new(),
                                                      active_descendents: 0,
                                                      cost: state_propagate_cost };
                            slot
                        },
                        _ => {
                            let idx_node_new = self.nodes.len();
                            self.nodes.push( Node { id: idx_node_new,
                                                    state: state_propagate.clone(),
                                                    children: HashSet::new(),
                                                    active_descendents: 0,
                                                    cost: state_propagate_cost } );
                            idx_node_new
                        },
                    };

                    
                    // let idx_node_new = self.nodes.len();
                    self.witness_representative.insert( witness_idx, idx_node_new ); //save new representative state idx for current witness
                    self.nodes_active.insert(idx_node_new);
                    // self.nodes.push( Node { id: idx_node_new,
                    //                         state: state_propagate.clone(),
                    //                         children: HashSet::new(),
                    //                         active_descendents: 0,
                    //                         cost: state_propagate_cost } );

                    self.nodes[idx_state_best_nearest].children.insert(idx_node_new);

                    self.link_parent.insert( idx_node_new, idx_state_best_nearest );

                    self.edges.insert( (idx_state_best_nearest, idx_node_new), Edge { control: param_sample } );

                    //increment active descendent count
                    let mut cur = idx_node_new;
                    loop {
                        match self.link_parent.get(&cur) {
                            Some(par) => {
                                self.nodes[*par].active_descendents += 1;
                                cur = *par;
                            },
                            _ => { break; },
                        }
                    }

                    //no node is made inactive, so pruning necessary
                },
            }
            
            if self.reached_goal( state_propagate ) {
                info!("found a path to goal on iteration: {}", i );
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
    }
}
