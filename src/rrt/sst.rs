//! Stable Sparse RRT
//! Based on Asymptotically Optimal Sampling-Based Kinodynnamic Planning paper

extern crate pretty_env_logger;

use std::collections::{HashSet,HashMap};
use std::marker::PhantomData;
use std::cmp::Ordering;

use rand::Rng;
use rand::prelude::*;
use rand::distributions::Standard;

use crate::rrt::rrt::RRT;
use crate::states::States;
use crate::control::Control;
use crate::planner_param::{Param,ParamObstacles};
use crate::moprim::{MoPrim,Motion};

use crate::instrumentation::*;

use super::nn_naive::NN_Naive;
use super::nn_stochastic::NN_Stochastic;

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

use rayon::prelude::*;

#[derive(Debug)]
pub struct Node<TS> {
    
    ///current node index
    pub id: usize,

    ///state space value
    pub state: TS,

    ///child node indices
    pub children: HashSet<(usize)>,
    
    ///cost in terms of number of steps from the root of the propagation tree
    pub cost: f32,
}

pub struct Edge <TC> {
    
    pub control: TC,

    ///additional annotation for differentiating propagation type
    pub kind: u32, //currently: 0: sst monte carlo propagation, 1: motion primitive propagation
}

pub struct SST<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    
    pub param: Param<TS,TC,TObs>,
    
    pub obstacles: Bvh<usize>, //bvh contain indices to obstacles in obstacles_actual

    pub obstacles_actual: ParamObstacles<TObs>,

    pub witnesses: Vec<TS>,

    ///maps witness to indices in nodes
    pub witness_representative: HashMap< usize, usize >,
    
    pub nodes: Vec< Node<TS> >,
    
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
    pub monte_carlo_prop_l: f32,
    pub monte_carlo_prop_h: f32,

    #[cfg(feature="nn_naive")]    
    pub nn_query_brute: NN_Naive<TS,TC,TObs>,

    #[cfg(not(feature="nn_naive"))]
    ///stores nodes
    pub nn_query: NN_Stochastic<TS,TC,TObs>,

    #[cfg(not(feature="nn_naive"))]
    ///stores only witnesses
    pub nn_query_witness: NN_Stochastic<TS,TC,TObs>,

    pub stat_pruned_nodes: u32,
    pub stat_iter_no_change: u32,

    pub stat_iter_collision: u32,

    pub iter_exec: u32,

    pub are_obstacles_boxes: bool,

    ///motion primitive
    #[cfg(feature="motion_primitives")]
    pub mo_prim: MoPrim<TS,TC,TObs>,
    #[cfg(feature="motion_primitives")]
    pub stat_motion_prim_invoked: u32,

    pub idx_reached: Option<usize>,

    pub stat_time_all: f64,
    pub stat_time_mo_prim_query: f64,
    pub stat_time_witness_nn_query: f64,
    pub stat_time_vicinity_best_nn_query: f64,
    pub stat_time_main_prop_check: f64,

    pub stat_count_nn_witness_queries: u64,
    pub stat_count_nn_node_queries: u64,
    
    pub last_moprim_candidates: Vec<(TObs,TObs)>,

    pub stat_witnesses_discovery_rate: f32,

    pub stat_witnesses_new: u32,

    pub witness_disturbance: bool,
}

impl <TS,TC,TObs> SST<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    
    pub fn init( param: & Param<TS,TC,TObs>, obstacles: Bvh<usize>, obstacles_concrete: ParamObstacles<TObs>, param_tree: ParamTree ) -> Self {
        //todo process obstacles...

        let box_obstacles = match obstacles_concrete.obstacles {
            ObsVariant::RBOX(_) => true,
            _ => false
        };
        
        let mut s = Self {
            
            are_obstacles_boxes: box_obstacles,
            
            param: param.clone(),
            obstacles: obstacles,
            obstacles_actual: obstacles_concrete,
            nodes: vec![ Node { id: 0,
                                state: param.states_init.clone(),
                                children: HashSet::new(),
                                cost: 0. } ],

            nodes_freelist: vec![],
            
            witnesses: vec![],
            witness_representative: HashMap::new(),
            
            edges: HashMap::new(),
            
            delta_v: param_tree.delta_v,
            delta_s: param_tree.delta_s,
            monte_carlo_prop_l: param_tree.prop_delta_low,
            monte_carlo_prop_h: param_tree.prop_delta_high,
            
            nodes_active: [0].to_vec().iter().cloned().collect(),
            nodes_inactive: HashSet::new(),
            link_parent: HashMap::new(),

            #[cfg(feature="nn_naive")]
            nn_query_brute: NN_Naive {
                phantom_ts: PhantomData,
                phantom_tc: PhantomData,
                phantom_tobs: PhantomData,
            },

            #[cfg(not(feature="nn_naive"))]
            nn_query: NN_Stochastic::init( param.ss_metric ),
            
            #[cfg(not(feature="nn_naive"))]
            nn_query_witness: NN_Stochastic::init( param.ss_metric ),

            stat_pruned_nodes: 0,
            stat_iter_no_change: 0,
            stat_iter_collision: 0,

            iter_exec: 0,

            #[cfg(feature="motion_primitives")]
            mo_prim: MoPrim::init( param.ss_metric,
                                   param.motion_primitive_xform.expect("motion primitive transform"),
                                   param.motion_primitive_xform_inv.expect("motion primitive transform inverse") ),
            
            #[cfg(feature="motion_primitives")]
            stat_motion_prim_invoked: 0,

            idx_reached: None,

            stat_time_all: 0.,
            stat_time_mo_prim_query: 0.,
            stat_time_witness_nn_query: 0.,
            stat_time_vicinity_best_nn_query: 0.,
            stat_time_main_prop_check: 0.,

            stat_count_nn_witness_queries: 0,
            stat_count_nn_node_queries: 0,
                
            last_moprim_candidates: vec![],

            stat_witnesses_discovery_rate: 0.,

            stat_witnesses_new: 0,
            
            witness_disturbance: false,
        };

        #[cfg(not(feature="nn_naive"))]
        {
            s.create_new_witness( param.states_init.clone() );
            s.add_propagated_state_to_nn_query( param.states_init.clone(), 0 );
        }
        
        s.witness_representative.insert( 0, 0 );        
        s
    }

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
    
    pub fn get_trajectory_edges_config_space( & self ) -> Vec<((TObs,TObs),u32)> {
        
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
                ( ( (self.param.project_state_to_config)(state_a.clone()),
                    (self.param.project_state_to_config)(state_b.clone()) ), (x.1).kind )
            })
            .collect()
    }
    pub fn reached_goal( & self, states: TS ) -> bool {
        let config_states = (self.param.project_state_to_config)(states.clone());
        (self.param.stop_cond)( states, config_states, self.param.states_goal.clone() )
    }

    pub fn get_last_motion_prim_candidates( & mut self ) -> Vec<(TObs,TObs)>{
        self.last_moprim_candidates.clone()
    }
    
    fn prune_nodes( & mut self, node_inactive: usize ){
        
        //remove leaf nodes and branches from propagation tree if possible
        let mut node_prune = node_inactive;
        
        loop {
            if self.nodes[ node_prune ].children.is_empty() &&
                !self.nodes_active.contains( & node_prune ) {
                    
                    self.nodes_inactive.remove( & node_prune );
                    self.nodes_freelist.push( node_prune );

                    #[cfg(not(feature="nn_naive"))]
                    {
                        //remove node from nn_query
                        self.nn_query.remove( node_prune );
                    }
                    
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
    }

    //insert node into propagate tree and return index of the inserted node
    fn insert_node( & mut self,
                      idx_node_nearest: usize,
                      state_propagate: TS,
                      control_propagate: TC,
                      propagation_cost: f32,
                      is_using_motion_prim: bool ) -> usize {
        
        //use freelist if possible
        let idx_node_new = match self.nodes_freelist.pop() {
            Some(slot) => {
                self.nodes[slot] = Node { id: slot,
                                          state: state_propagate,
                                          children: HashSet::new(),
                                          cost: propagation_cost };
                slot
            },
            _ => {
                let idx_node_new = self.nodes.len();
                self.nodes.push( Node { id: idx_node_new,
                                        state: state_propagate,
                                        children: HashSet::new(),
                                        cost: propagation_cost } );
                idx_node_new
            },
        };
        
        self.nodes_active.insert(idx_node_new);

        self.nodes[idx_node_nearest].children.insert(idx_node_new);
        
        self.link_parent.insert( idx_node_new, idx_node_nearest );
        
        self.edges.insert( (idx_node_nearest, idx_node_new), Edge { control: control_propagate,
                                                                    kind: if is_using_motion_prim { 1 } else { 0 } } );

        idx_node_new
    }
    
    fn inactivate_node( & mut self, idx_node: usize ){
        self.nodes_active.remove( &idx_node );
        self.nodes_inactive.insert( idx_node );
    }

    ///return true if there is a collision
    fn collision_check( & mut self, config_space_state_before: &TObs, config_space_state_after: &TObs ) -> bool {
        
        let v0 = config_space_state_before.get_vals_3();
        let v1 = config_space_state_after.get_vals_3();

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

        collision
    }

    ///attempts to use a suitable motion primitive, returning time duration and control if successful
    #[cfg(feature="motion_primitives")]
    fn try_motion_primitive_control( & mut self, state_space_nearest: TS, config_space_coord_before: TObs ) -> Option<(f32, TC)> {

        let mut timer = Timer::default();
        
        if self.mo_prim.lookup.len() >= 500 {

            let cost_threshold = if cfg!(feature="mo_prim_thresh_low"){ 0.1 }
                                 else if cfg!(feature="mo_prim_thresh_high"){ 0.4 }
                                 else { 0.25 };


            let config_space_goal = (self.param.project_state_to_config)(self.param.states_goal.clone());
                                                 
            let d = (self.param.cs_metric)( config_space_coord_before.clone(), config_space_goal.clone() );
                                            
            
            if d < cost_threshold {

                //try using motion primitive to propagate towards goal
                
                let q_query_mo_prim = self.param.states_goal.clone();
                
                let motions : Vec<Motion<_,_>> = self.mo_prim.query_motion( state_space_nearest.clone(),
                                                                            q_query_mo_prim,
                                                                            cost_threshold );                        
                
                let start_point = &config_space_coord_before;

                //test for obstable collision for candidate motions
                let sel_motion = motions.iter().filter_map(|m|{
                    let control = m.u.clone();
                    let time_dur = m.t.clone();
                    
                    let propagate_motion = (self.param.dynamics)( state_space_nearest.clone(),
                                                                  control,
                                                                  time_dur );
                    
                    let end_point = (self.param.project_state_to_config)(propagate_motion.clone());

                    let collision = self.collision_check( &start_point, &end_point );
                        
                    let d_diff = (self.param.cs_metric)( end_point.clone(), config_space_goal.clone() );
                    
                    if collision || d_diff > d {
                        None
                    } else {
                        #[cfg(feature="mo_prim_debug")]
                        {
                            self.last_moprim_candidates.push( (start_point.clone(),
                                                               end_point.clone()) ); //debugging purpose
                        }
                        
                        Some( (d_diff,m) )
                    }

                });

                let motion = sel_motion.min_by(|a,b| a.0.partial_cmp( &b.0 ).unwrap_or(Ordering::Equal) );
                
                match motion {
                    Some((d,Motion{u,t,..})) => {
                        return Some(( *t, u.clone() ))
                    },
                    _ => {},
                }
            }   
        }
        
        let t_delta = timer.dur_ms();

        self.stat_time_mo_prim_query += t_delta;
        
        None
    }

    ///propagation with random time delta and control
    fn generate_monte_carlo_propagation( & mut self ) -> (f32, TC) {

        //enforce bounds
        let mut val: f32 = SmallRng::from_entropy().sample(Standard);
        
        val = if val < self.monte_carlo_prop_l { self.monte_carlo_prop_l } else { val };
        val = if val > self.monte_carlo_prop_h { self.monte_carlo_prop_h } else { val };
        
        let monte_carlo_prop_delta = val * self.param.sim_delta;
        
        //sampler for control space
        let control_sample = (self.param.param_sampler)( monte_carlo_prop_delta );

        ( monte_carlo_prop_delta, control_sample )
    }

    #[cfg(not(feature="nn_naive"))]
    fn create_new_witness( & mut self, state: TS ) -> usize {

        let idx_new = self.witnesses.len();
        
        self.witnesses.push( state.clone() );
        
        self.nn_query_witness.add( state, idx_new, self.param.ss_metric );
        
        idx_new
    }

    #[cfg(not(feature="nn_naive"))]
    fn add_propagated_state_to_nn_query( & mut self, state: TS, id: usize ) {

        self.nn_query.add( state, id, self.param.ss_metric );
    }

    ///return id of the nearest existing propagation node in state space and return a possibly modified state space sample
    fn get_best_vicinity( & mut self, ss_sample: TS ) -> ( usize, TS ) {
        
        #[cfg(feature="nn_naive")]
        {
            let idx_ret = self.nn_query_brute.query_nearest_state_active( ss_sample.clone(),
                                                                          & self.nodes,
                                                                          & self.nodes_active,
                                                                          & self.param,
                                                                          self.delta_v );
            ( idx_ret, ss_sample )
        }
        #[cfg(not(feature="nn_naive"))]
        {
            let mut rng = rand::thread_rng();
            let prob_use_state_prop_sample = rng.gen_range(0., 1.);
            
            if cfg!(feature="state_propagate_sample") && prob_use_state_prop_sample > 0.5 
            {
                
                let ss_samples = (0..5).map( |_| (self.param.ss_sampler)() ).collect::<Vec<_>>();
                
                let sample_nearest_pairs = ss_samples.into_iter()
                    .map(|sample|{
                        let r0 = self.nn_query.query_nearest_threshold( sample.clone(),
                                                                        self.param.ss_metric,
                                                                        self.delta_v );
                        let idx_nearest = if r0.is_empty(){
                            let r1 = self.nn_query.query_nearest_k( sample.clone(),
                                                                    self.param.ss_metric,
                                                                    1 );
                            let (_,idx_ret) = *r1.iter().nth(0).unwrap();
                            idx_ret
                        }else{
                            let (_,idx_ret) = *r0.iter().nth(0).unwrap();
                            idx_ret
                        };
                        
                        (sample,idx_nearest)
                    }).collect::<Vec<_>>();
                
                let (sample_sel,idx_sel) = sample_nearest_pairs.into_iter()
                    .max_by(|(sample_a,idx_nearest_a),(sample_b,idx_nearest_b)|{
                        
                        let dist_a = self.nn_query.query_dist_node_neighbourhood_avg( sample_a.clone(),
                                                                                      *idx_nearest_a,
                                                                                      self.param.ss_metric,
                                                                                      1 );

                        let dist_b = self.nn_query.query_dist_node_neighbourhood_avg( sample_b.clone(),
                                                                                      *idx_nearest_b,
                                                                                      self.param.ss_metric,
                                                                                      1 );
                        dist_a.partial_cmp( & dist_b ).unwrap_or( Ordering::Equal )
                    }).unwrap();
                
                //return a different state space sample
                ( idx_sel, sample_sel )

            } else {
                let mut ret = self.nn_query.query_nearest_threshold( ss_sample.clone(),
                                                                     self.param.ss_metric,
                                                                     self.delta_v );
                if ret.is_empty(){
                    ret = self.nn_query.query_nearest_k( ss_sample.clone(),
                                                         self.param.ss_metric,
                                                         1 );
                }
                
                let (_,idx_ret) = *ret.iter().nth(0).expect("nn query failed to return a node");
                
                ( idx_ret, ss_sample )
            }
        }
    }

    ///returns ( propagation delta, control, is_using_motion_primitive )
    fn select_propagation_params( & mut self, state_space_start: TS, state_config_start: TObs ) -> ( f32, TC, bool ) {
        #[cfg(feature="motion_primitives")]
        {
            let rand_prob = rng.gen_range(0., 1.);
            if rand_prob > 0.5 {
                match self.try_motion_primitive_control( state_space_start, state_config_start ) {
                    Some((t, u)) => {
                        //replace monte carlo propagation time and random control sample with the one from motion primitive
                        ( t, u, true )
                    },
                    _ => {
                        let ( t, u ) = self.generate_monte_carlo_propagation();
                        ( t, u, false )
                    },
                }
            } else {
                let ( t, u ) = self.generate_monte_carlo_propagation();
                ( t, u, false )
            }
        }
        #[cfg(not(feature="motion_primitives"))]
        {
            let ( t, u ) = self.generate_monte_carlo_propagation();
            ( t, u, false )
        }
    }

    ///returns ( idx of witness, is new witness ) associated with the propagated node
    fn get_witness_neighbourhood( & mut self, state: TS ) -> ( usize, bool ) {
        #[cfg(feature="nn_naive")]
        {
            match self.nn_query_brute.query_nearest_witness( state.clone(),
                                                             & self.witnesses, 
                                                             & self.param,
                                                             self.delta_s ) {
                Some(idx) => {
                    ( idx, false )
                },
                _ => {
                    //no witness found within delta_s vicinity, so create a new witness
                    let idx_new = self.witnesses.len();
                    self.witnesses.push( state );
                    ( idx_new, true )
                },
            }
        }
        #[cfg(not(feature="nn_naive"))]
        {
            let ret = self.nn_query_witness.query_nearest_threshold( state.clone(),
                                                                     self.param.ss_metric,
                                                                     self.delta_s );
            match ret.iter().nth(0) {
                Some((_,idx_global)) => {
                    //found witness
                    ( *idx_global,false )
                },
                _ => {
                    //no witness found within delta_s vicinity, so create a new witness
                    let idx_new = self.create_new_witness( state );
                    ( idx_new, true )
                },
            }
        }
    }

    ///disturbance injection for witness representative replacement
    fn witness_representative_disturbance_inject( & mut self ) {
        
        let iter_propagated = self.iter_exec - self.stat_iter_no_change;

        //sliding window
        if self.iter_exec % 200 == 0 {

            self.stat_witnesses_discovery_rate = self.stat_witnesses_new as f32 / 200 as f32;
            self.stat_witnesses_new = 0;
            
            if self.iter_exec > 1000 {

                //trigger disturbance injection if witness discovery rate is low
                if self.stat_witnesses_discovery_rate <= 0.1 {
                    self.witness_disturbance = true;
                } else {
                    self.witness_disturbance = false;
                }
            }
        }
    }
}

impl <TS,TC,TObs> RRT < TS,TC,TObs > for SST<TS,TC,TObs> where TS: States, TC: Control, TObs: States {

    fn reset( & mut self ){
        
        self.nodes = vec![ Node { id: 0,
                                  state: self.param.states_init.clone(),
                                  children: HashSet::new(),
                                  cost: 0. } ];

        self.edges = HashMap::new();
        self.witness_representative.clear();
        self.nodes_active = HashSet::new();
        self.nodes_active.insert( 0 );
        self.nodes_inactive.clear();
        self.link_parent.clear();
        self.nodes_freelist.clear();
        self.stat_pruned_nodes = 0;
        self.stat_iter_no_change = 0;
        self.stat_iter_collision = 0;
        self.iter_exec = 0;
        self.idx_reached = None;
        self.stat_time_mo_prim_query = 0.;
        self.stat_time_witness_nn_query = 0.;
        self.stat_time_vicinity_best_nn_query = 0.;
        self.stat_time_main_prop_check = 0.;
        self.stat_time_all = 0.;
        self.stat_count_nn_witness_queries = 0;
        self.stat_count_nn_node_queries = 0;
        
        self.last_moprim_candidates = vec![];

        #[cfg(not(feature="nn_naive"))]
        {
            self.create_new_witness( self.param.states_init.clone() );
            self.add_propagated_state_to_nn_query( self.param.states_init.clone(), 0 );
        }
        self.witness_representative.insert( 0, 0 );
    }
    
    fn iterate( & mut self, iteration: Option<u32> ) -> bool {

        let mut rng = rand::thread_rng();
        
        if self.idx_reached.is_some() || self.iter_exec >= self.param.iterations_bound {
            return false
        }

        let mut timer_all = Timer::default();

        let iter_batch = match iteration {
            Some(x) => { x },
            _ => { self.param.iterations_bound },
        };

        let config_space_goal = (self.param.project_state_to_config)(self.param.states_goal.clone());
        
        'l_outer: for i in 0..iter_batch {

            self.iter_exec += 1;

            let ( idx_state_best_nearest, ss_sample ) = {

                let ss_sample_seed = (self.param.ss_sampler)();
                
                let mut timer_nn = Timer::default();

                //get best active state in vicinity delta_v of ss_sample, or return nearest active state
                
                let ( idx, sample ) = self.get_best_vicinity( ss_sample_seed );

                let t_delta_nn = timer_nn.dur_ms();
                self.stat_time_vicinity_best_nn_query += t_delta_nn;
                self.stat_count_nn_node_queries += 1;

                ( idx, sample )
            };
            
            let state_start = self.nodes[idx_state_best_nearest].state.clone();
            let config_space_coord_before = (self.param.project_state_to_config)( state_start.clone() );
            
            let( monte_carlo_prop_delta, param_sample, is_using_motion_prim ) = self.select_propagation_params( state_start.clone(),
                                                                                                                config_space_coord_before.clone() );

            let state_propagate_cost = self.nodes[idx_state_best_nearest].cost + monte_carlo_prop_delta;

            let state_propagate = (self.param.dynamics)( state_start.clone(),
                                                         param_sample.clone(),
                                                         monte_carlo_prop_delta );

            let config_space_coord_after = (self.param.project_state_to_config)(state_propagate.clone());

            #[cfg(feature="motion_primitives")]
            {
                let rand_prob = rng.gen_range(0., 1.);
                if rand_prob > 0.85 || self.mo_prim.lookup.len() < self.mo_prim.capacity {
                    //no matter what obstructions are out there, we can still record the motion
                    self.mo_prim.add_motion( state_start,
                                             state_propagate.clone(),
                                             param_sample.clone(),
                                             monte_carlo_prop_delta,
                                             monte_carlo_prop_delta );

                }
            }

            let mut timer = Timer::default();

            //get the witness node in the sampled neighbourhood

            let ( witness_idx, is_new_witness ) = self.get_witness_neighbourhood( state_propagate.clone() );

            let t_delta = timer.dur_ms();
            self.stat_time_witness_nn_query += t_delta;
            self.stat_count_nn_witness_queries += 1;
            
            #[cfg(not(feature="disable_witness_disturbance"))]
            {
                if is_new_witness {
                    self.stat_witnesses_new += 1;
                }
                self.witness_representative_disturbance_inject();
            }

            let reached = self.reached_goal( state_propagate.clone() );
            
            let witness_repr = match self.witness_representative.get( &witness_idx ) {
                Some(x) => { Some(*x) },
                _ => None,
            };
            
            let mut timer2 = Timer::default();
            
            let idx_node = match witness_repr {
                Some( repr ) => {
                    
                    let witness_distrubance_prob = rng.gen_range(0., 1.);

                    if state_propagate_cost < self.nodes[ repr ].cost ||
                        reached  ||
                        ( self.witness_disturbance && witness_distrubance_prob > 0.5 ) {

                        if self.collision_check( &config_space_coord_before, &config_space_coord_after ) {
                            self.stat_iter_no_change += 1;
                            self.stat_iter_collision += 1;
                            None
                        } else {

                            let idx_inserted = self.insert_node( idx_state_best_nearest.clone(),
                                                                 state_propagate.clone(),
                                                                 param_sample.clone(),
                                                                 state_propagate_cost.clone(),
                                                                 is_using_motion_prim );

                            let node_inactive = repr;
                            
                            #[cfg(not(feature="disable_pruning"))]
                            {
                                self.inactivate_node( node_inactive.clone() );
                                
                                //remove leaf nodes and branches from propagation tree if possible
                                self.prune_nodes( node_inactive ); //remove states from nn query as well
                            }
                            //save new representative state idx for current witness
                            *self.witness_representative.get_mut( &witness_idx ).unwrap() = idx_inserted;

                            #[cfg(not(feature="nn_naive"))]
                            {
                                //add propagated state to nn_query
                                self.add_propagated_state_to_nn_query( state_propagate.clone(), idx_inserted );
                            }
                            
                            #[cfg(feature="motion_primitives")]
                            {
                                if is_using_motion_prim {
                                    self.stat_motion_prim_invoked += 1;
                                }
                            }
                            Some(idx_inserted)
                        }
                    } else {
                        self.stat_iter_no_change += 1;
                        None
                    }
                },
                _ => {
                    //no representative found, so just add the propagated state as representative
                    if self.collision_check( &config_space_coord_before, &config_space_coord_after ) {
                        self.stat_iter_no_change += 1;
                        self.stat_iter_collision += 1;
                        None
                    } else {

                        let idx_inserted = self.insert_node( idx_state_best_nearest.clone(),
                                                             state_propagate.clone(),
                                                             param_sample.clone(),
                                                             state_propagate_cost.clone(),
                                                             is_using_motion_prim );

                        self.witness_representative.insert( witness_idx, idx_inserted ); //save new representative state idx for current witness

                        #[cfg(not(feature="nn_naive"))]
                        {
                            //add propagated state to nn_query
                            self.add_propagated_state_to_nn_query( state_propagate.clone(), idx_inserted );
                        }
                        
                        //no node is made inactive, hence no pruning necessary
                        Some(idx_inserted)
                    }
                },
            };

            let t_delta2 = timer2.dur_ms();
            self.stat_time_main_prop_check += t_delta2;
            
            match (idx_node, reached ) {
                (Some(x),true) => {
                    let d_goal = (self.param.cs_metric)( config_space_coord_after.clone(), config_space_goal );
                    info!("found a path to goal on iteration: {}, diff: {}", self.iter_exec, d_goal );
                    self.idx_reached = Some(x);
                    break;
                },
                _ => {},
            }
        }
        
        let t_delta_all = timer_all.dur_ms();
        self.stat_time_all += t_delta_all;
        
        self.print_stats();
        true
    }
    
    fn get_best_trajectory_config_space( & self ) -> Vec<((TObs,TObs),u32)> {
        
        let mut edges = vec![];

        let lim = 1000000;
        let mut count = 0;
        match self.idx_reached {
            Some(x) => {
                let mut idx = x;
                loop {
                    count += 1;
                    if count >= lim {
                        panic!("looping");
                    }
                    idx = match self.link_parent.get( &idx ) {
                        Some(parent) => {
                            edges.push( (*parent, idx) );
                            *parent
                        },
                        _ => { break; },
                    };
                }
            },
            _ => {},
        }

        edges.iter()
            .map(|(parent,child)| {
                let e = self.edges.get( &(*parent,*child) ).expect("edge not found");
                let state_a = &self.nodes[*parent].state;
                let state_b = &self.nodes[*child].state;
                ( ( (self.param.project_state_to_config)(state_a.clone()),
                     (self.param.project_state_to_config)(state_b.clone()) ), e.kind )
            })
            .collect()
    }

    fn print_stats( &self ){
        info!( "witnesses: {}", self.witnesses.len() );
        info!( "nodes: {}", self.nodes.len() );
        info!( "nodes active: {}", self.nodes_active.len() );
        info!( "nodes inactive: {}", self.nodes_inactive.len() );
        info!( "pruned_nodes: {}", self.stat_pruned_nodes );
        info!( "nodes freelist: {}", self.nodes_freelist.len() );
        info!( "disturbance active: {}", if self.witness_disturbance { "Y" } else { "N" } );
        info!( "iterations no change: {}/{}, {:.2}%", self.stat_iter_no_change, self.iter_exec, self.stat_iter_no_change as f32/self.iter_exec as f32 * 100. );
        info!( "iterations collision: {}/{}, {:.2}%", self.stat_iter_collision, self.iter_exec, self.stat_iter_collision as f32/self.iter_exec as f32 * 100. );

        info!( "stat_time_mo_prim_query: {} ms / {}%", self.stat_time_mo_prim_query, self.stat_time_mo_prim_query / self.stat_time_all * 100. );
        
        info!( "stat_time_witness_nn_query: {} ms / {}% / {}ms/query",
                self.stat_time_witness_nn_query,
                self.stat_time_witness_nn_query / self.stat_time_all * 100.,
                self.stat_time_witness_nn_query / self.stat_count_nn_witness_queries as f64 );
        
        info!( "stat_time_vicinity_best_nn_query: {} ms / {}% / {}ms/query",
                self.stat_time_vicinity_best_nn_query,
                self.stat_time_vicinity_best_nn_query / self.stat_time_all * 100.,
                self.stat_time_vicinity_best_nn_query / self.stat_count_nn_node_queries as f64 );
        
        info!( "stat_time_main_prop_check: {} ms / {}%", self.stat_time_main_prop_check, self.stat_time_main_prop_check / self.stat_time_all * 100. );
        
        #[cfg(feature="motion_primitives")]
        {
            self.mo_prim.print_stats();
            info!( "stat_motion_prim_invoked: {}", self.stat_motion_prim_invoked );
        }

        #[cfg(not(feature="nn_naive"))]
        {
            self.nn_query_witness.print_stats();
        }
    }
}
