//! based on stochastic nearest neighbour query from
//! Asymptotically Optimal Sampling-Based Kinodynnamic Planning paper
extern crate pretty_env_logger;

use std::collections::{HashSet,HashMap};
use std::marker::PhantomData;
use std::cmp::Ordering;

use rand::Rng;
use rand::prelude::*;
use rand::distributions::Standard;

use crate::states::States;
use crate::control::Control;
use crate::planner_param::{Param,ParamObstacles};

use crate::rrt::sst::Node;

#[derive(Default,Debug)]
pub struct NN_Stochastic<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    pub phantom_ts: PhantomData< TS >,
    pub phantom_tc: PhantomData< TC >,
    pub phantom_tobs: PhantomData< TObs >,

    pub edges: HashMap< usize, HashSet<usize> >, //bidirectional map of local indices of connected nodes
    
    pub nodes: Vec<TS>,
    pub nodes_map: HashMap< usize, usize >, //map idx -> local index of nodes
    pub inverse_map: HashMap< usize, usize >, //map local index of nodes -> idx

    pub list_alive: Vec<usize>, //stores local indices of nodes
    pub list_free: Vec<usize>, //stores local indices of free slots
}

impl<TS,TC,TObs> NN_Stochastic<TS,TC,TObs> where TS: States, TC: Control, TObs: States {

    fn edge_add( & mut self, a: usize, b: usize ){
        if !self.edges.contains_key( &a ) {
            self.edges.insert( a, HashSet::new() );
        }
        let mut hs = self.edges.get_mut( &a ).unwrap();
        hs.insert( b );
    }

    fn edge_remove( & mut self, idx_local: usize ){
        
        let connected = match self.edges.get( &idx_local ) {
            Some(x) => {
                x.iter().cloned().collect()
            },
            _ => { vec![] },
        };

        connected.iter().for_each(|x|{
            match self.edges.get_mut(x) {
                Some(i) => {
                    i.remove(&idx_local);
                },
                _ => {},
            }
        });
    }

    ///adds a new node by query for k (log(number of total nodes)) nearest nodes
    ///and adding edges to these nodes
    pub fn add( & mut self, state: TS, idx_global: usize, f: fn(TS,TS)->f32 ) -> usize {

        let k = if self.list_alive.is_empty() {
            0
        } else {
            (self.list_alive.len() as f32).log2().ceil() as usize
        };

        assert!( k >= 0 );
        
        let arr = self.query_nearest_k( state.clone(), f, k );

        let node_idx_new = if self.list_free.len() > 0 {
            let idx = self.list_free.pop().unwrap();
            self.nodes[idx] = state;
            idx
        } else {
            let idx = self.nodes.len();
            self.nodes.push(state);
            idx
        };
        
        arr.iter().for_each(|(idx_local,idx_global)|{
            self.edge_add( node_idx_new, *idx_local );
            self.edge_add( *idx_local, node_idx_new );
        });

        self.nodes_map.insert( idx_global, node_idx_new );
        self.inverse_map.insert( node_idx_new, idx_global );

        node_idx_new
    }

    ///removes a given node and its connections
    ///this assumes the node exists internally
    pub fn remove( & mut self, state: TS, idx_global: usize, f: fn(TS,TS)->f32 ) -> usize {
        
        let idx_local = *self.nodes_map.get( &idx_global ).expect("node not exist");

        self.nodes_map.remove( &idx_global );
        
        self.inverse_map.remove( &idx_local );
        
        self.edge_remove( idx_local );
            
        self.list_free.push(idx_local);
        
        idx_local
    }

    ///for small number of nodes, test cost function against each of the nodes,
    ///for large numer of nodes, stochastically sample nodes and narrow down to a single node
    ///and walk over its neighbours and take the min cost node if the cost improves,
    ///iterate until it does not improve anymore.
    ///returns (idx_local,idx_global)
    pub fn query_nearest( & mut self, state_query: TS, f: fn(TS,TS)->f32 ) -> (usize,usize) {
        
        let n = self.list_alive.len();
        
        if n < 30 {
            let idx_local = self.list_alive.iter().min_by(|a,b| {
                let cost_a = f(self.nodes[**a].clone(), state_query.clone());
                let cost_b = f(self.nodes[**b].clone(), state_query.clone());
                cost_a.partial_cmp( &cost_b ).unwrap_or(Ordering::Equal)
            }).expect("no nodes");
            
            (*idx_local,
             *self.inverse_map.get( idx_local ).expect("inverse map node does not exist"))
                
        } else {
            
            let n_sample = (n as f32).sqrt() as usize;
            
            let mut rng = rand::thread_rng();
            
            let mut idx_local = (0..n_sample).map(|x|{ let i : usize = rng.gen_range(0, n); i })
                .min_by(|a,b| {
                    let idx_a = self.list_alive[*a];
                    let idx_b = self.list_alive[*b];
                    let cost_a = f(self.nodes[idx_a].clone(), state_query.clone());
                    let cost_b = f(self.nodes[idx_b].clone(), state_query.clone());
                    cost_a.partial_cmp( &cost_b ).unwrap_or(Ordering::Equal)
                }).expect("no nodes");

            loop {
                let idx_new = match self.edges.get( &idx_local ) {
                    Some(x) => {
                        let temp = [idx_local].to_vec();
                        let idx_nearest = x.iter().chain(temp.iter()).min_by(|a,b|{
                            let cost_a = f(self.nodes[**a].clone(), state_query.clone());
                            let cost_b = f(self.nodes[**b].clone(), state_query.clone());
                            cost_a.partial_cmp( &cost_b ).unwrap_or(Ordering::Equal)
                        }).expect("no nodes");
                        *idx_nearest
                    },
                    _ => {
                        panic!("no nodes");
                    },
                };
                
                if idx_local != idx_new {
                    idx_local = idx_new;
                } else {
                    break;
                }
            }

            (idx_local,
             *self.inverse_map.get( &idx_local ).expect("inverse map node does not exist"))
        }
    }

    ///query nearest and then repeatedly find the best k nodes in the neighbourhood
    ///until convergence of the list
    pub fn query_nearest_k( & mut self, state_query: TS, f: fn(TS,TS)->f32, k: usize ) -> Vec<(usize,usize)> {
        
        let (idx_local,idx_global) = self.query_nearest( state_query.clone(), f );

        let mut items_k = HashSet::new();
        items_k.insert( idx_local );

        //loop until k nearest items converge
        loop {

            let mut temp = HashSet::new();
            
            for i in items_k.iter() {
                match self.edges.get(i){
                    Some(x) => {
                        let mut arr : Vec<_> = x.iter().cloned().collect();
                        arr.push(*i);
                        arr.sort_by(|a,b|{
                            let cost_a = f(self.nodes[*a].clone(), state_query.clone());
                            let cost_b = f(self.nodes[*b].clone(), state_query.clone());
                            cost_a.partial_cmp( &cost_b ).unwrap_or(Ordering::Equal)
                        });
                        arr.iter()
                            .take(k)
                            .for_each(|x| {temp.insert(*x);} );
                    },
                    _ => {},
                }
            }

            let mut temp_arr : Vec<_> = temp.into_iter().collect();
            temp_arr.sort_by(|a,b|{
                let cost_a = f(self.nodes[*a].clone(), state_query.clone());
                let cost_b = f(self.nodes[*b].clone(), state_query.clone());
                cost_a.partial_cmp( &cost_b ).unwrap_or(Ordering::Equal)
            });

            temp = temp_arr.into_iter().take(k).collect();

            //check for convergence of list
            let c = items_k.intersection( &temp ).count();
            if c != items_k.len() {
                items_k = temp.into_iter().collect();
            } else {
                break;
            }
        }

        items_k.into_iter().map(|x|{
            (x, *self.inverse_map.get( &x ).expect("inverse map node does not exist"))
        }).collect()
    }

    ///query nearest and then repeatedly find the best nodes in the neighbourhood that is within some threshold
    ///until convergence of the list
    pub fn query_nearest_threshold( & mut self, state_query: TS, f: fn(TS,TS)->f32, threshold: f32 ) -> Vec<(usize,usize)> {

        let k = if self.list_alive.is_empty() {
            0
        } else {
            (self.list_alive.len() as f32).log2().ceil() as usize
        };
        
        let mut candidates = self.query_nearest_k( state_query.clone(), f, k );
        
        candidates.into_iter().filter(|(idx_local,_)|{
            f(state_query.clone(),self.nodes[*idx_local].clone()) < threshold
        }).collect()
    }
}
