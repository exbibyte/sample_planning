//! Based on stochastic nearest neighbour query from
//! Asymptotically Optimal Sampling-Based Kinodynnamic Planning paper
extern crate pretty_env_logger;

use std::collections::{HashSet,HashMap};
use std::marker::PhantomData;
use std::cmp::Ordering;
use std::cmp;

use rand::Rng;
use rand::prelude::*;
use rand::distributions::Standard;

use crate::states::States;
use crate::control::Control;
use crate::planner_param::{Param,ParamObstacles};

use crate::rrt::sst::Node;

use std::io::Read;

#[derive(Debug)]
pub struct NN_Stochastic<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    pub phantom_ts: PhantomData< TS >,
    pub phantom_tc: PhantomData< TC >,
    pub phantom_tobs: PhantomData< TObs >,

    pub edges: HashMap< usize, HashSet<usize> >, //bidirectional map of local indices of connected nodes
    
    pub nodes: Vec<TS>,
    pub nodes_map: HashMap< usize, usize >, //map global idx -> local index of nodes
    pub inverse_map: HashMap< usize, usize >, //map local index of nodes -> global idx

    pub lookup_alive: HashSet<usize>, //stores local indices of nodes
    // pub list_alive: Vec<usize>, //stores local indices of nodes
    
    pub list_free: Vec<usize>, //stores local indices of free slots

    pub list_valence_fixup: Vec<usize>,

    pub f_metric: fn(TS,TS)->f32,

    pub stat_valence_fixups: usize,
}

// impl<TS,TC,TObs> Default for NN_Stochastic<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
//     fn default() -> Self {
//         Self {
//             phantom_ts: PhantomData,
//             phantom_tc: PhantomData,
//             phantom_tobs: PhantomData,

//             edges: HashMap::new(),
            
//             nodes: vec![],
//             nodes_map: HashMap::new(),
//             inverse_map: HashMap::new(),

//             // list_alive: vec![],
//             list_alive: HashSet::new(),
//             list_free: vec![],
//         }
//     }
// }

impl<TS,TC,TObs> NN_Stochastic<TS,TC,TObs> where TS: States, TC: Control, TObs: States {

    pub fn init( f: fn(TS,TS)->f32 ) -> Self {
        Self {
            phantom_ts: PhantomData,
            phantom_tc: PhantomData,
            phantom_tobs: PhantomData,

            edges: HashMap::new(),
            
            nodes: vec![],
            nodes_map: HashMap::new(),
            inverse_map: HashMap::new(),

            lookup_alive: HashSet::new(),
            // list_alive: vec![],
            list_free: vec![],

            list_valence_fixup: vec![],
            
            f_metric: f,

            stat_valence_fixups: 0,
        }
    }
    
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
            _ => { vec![] }
        };
        
        connected.into_iter().for_each(|x|{
            match self.edges.get_mut(&x) {
                Some(i) => {
                    i.remove(&idx_local);
                    if i.is_empty() {
                        self.edges.remove(&x);
                    }
                },
                _ => {},
            }});

        self.edges.remove( &idx_local );
    }

    ///adds a new node by query for k (log(number of total nodes)) nearest nodes
    ///and adding edges to these nodes
    pub fn add( & mut self, state: TS, idx_global: usize, f: fn(TS,TS)->f32 ) -> usize {
        
        let k = if self.lookup_alive.len() < 50 {
            self.lookup_alive.len()
        } else {
            cmp::min( 2 * ((self.lookup_alive.len() as f32).log2() as usize), self.lookup_alive.len() )
        };

        assert!( k >= 0 );
        
        let arr = self.query_nearest_k( state.clone(), f, k );
        
        let node_idx_new = if self.list_free.len() > 0 {
            let idx = self.list_free.pop().unwrap();
            self.nodes[idx] = state;
            assert!( !self.lookup_alive.contains(&idx) );
            self.lookup_alive.insert(idx);
            idx
        } else {
            let idx = self.nodes.len();
            self.nodes.push(state);
            assert!( !self.lookup_alive.contains(&idx) );
            self.lookup_alive.insert(idx);
            idx
        };

        // println!("self.nodes len: {}, arr length: {}", self.nodes.len(), arr.len() );

        assert!( !self.edges.contains_key( &node_idx_new ) );
        
        arr.iter().for_each(|(idx_l,idx_g)|{
            self.edge_add( node_idx_new, *idx_l );
            self.edge_add( *idx_l, node_idx_new );
        });

        assert!( !self.nodes_map.contains_key( &idx_global ) );
        assert!( !self.inverse_map.contains_key( &node_idx_new ) );
        
        self.nodes_map.insert( idx_global, node_idx_new );
        self.inverse_map.insert( node_idx_new, idx_global );
        
        node_idx_new
    }

    ///removes a given node and its connections
    ///this assumes the node exists internally
    pub fn remove( & mut self, idx_global: usize ) -> usize {
        
        let idx_local = *self.nodes_map.get( &idx_global ).expect("node not exist");

        assert!( self.inverse_map.remove( &idx_local ).is_some() );
        
        assert!( self.nodes_map.remove( &idx_global ).is_some() );
        
        self.edge_remove( idx_local );
        
        self.list_free.push( idx_local );

        self.lookup_alive.remove( &idx_local );
            
        idx_local
    }

    ///get valence number of nodes
    fn sample_valence(&self) -> usize {

        let valence = if self.lookup_alive.len() < 50 {
            self.lookup_alive.len()
        } else {
            cmp::min( (((self.lookup_alive.len() as f32).log2()) as usize), self.lookup_alive.len() )
        };
        
        valence
    }

    fn query_sample_count(&self) -> usize {
        let n = self.lookup_alive.len();
        let valence = if cfg!(feature="nn_sample_log") {
            cmp::min(((n as f32).log2() as usize), n )
        } else {
            (n as f32).sqrt() as usize
        };
        n
    }

    fn valence_fixup(&mut self){
        
        use std::mem;
        
        let mut v = vec![];

        mem::swap(&mut v, &mut self.list_valence_fixup );

        self.stat_valence_fixups += v.len();
        
        v.into_iter()
            .for_each(|x|{
                let state = self.nodes[x].clone();
                let idx_global = *self.inverse_map.get( &x ).expect("inverse map of node not exist");
                self.remove(idx_global);
                self.add( state, idx_global, self.f_metric );
            });
        assert!(self.list_valence_fixup.is_empty());
    }
    
    ///for small number of nodes, test cost function against each of the nodes,
    ///for large numer of nodes, stochastically sample a portion of nodes and narrow down to a single node
    ///and walk over its neighbours and take the min cost node if the cost improves,
    ///iterate until it does not improve anymore.
    ///returns (idx_local,idx_global)
    pub fn query_nearest( & mut self, state_query: TS, f: fn(TS,TS)->f32 ) -> Option<(usize,usize)> {
        
        let n = self.lookup_alive.len();

        if n < 100 {
            
            let idx_local = self.lookup_alive.iter().min_by(|a,b| {
                let cost_a = f(self.nodes[**a].clone(), state_query.clone());
                let cost_b = f(self.nodes[**b].clone(), state_query.clone());
                cost_a.partial_cmp( &cost_b ).unwrap_or(Ordering::Equal)
            });
            
            match idx_local {
                Some(x) => {
                    Some( (*x,
                           *self.inverse_map.get( x ).expect("inverse map node does not exist")) )
                },
                _ => { None },
            }
                
        } else {

            let n_valence = self.sample_valence();
            let n_query_sample = self.query_sample_count();
            
            let mut rng = rand::thread_rng();

            //valence fixups
            {
                let sample_list : Vec<_> = self.lookup_alive.iter().collect();

                let mut candidate_rewire_nodes = HashSet::new();
                
                (0..n_query_sample).map(|x|{ let i : usize = rng.gen_range(0, n); i })
                    .map(|i|{
                        let idx = *sample_list[i];
                        idx
                    }).for_each(|i|{
                        match self.edges.get(&i){
                            Some(neighbours)=>{
                                if neighbours.len() < n_valence *4/5 {
                                    candidate_rewire_nodes.insert(i);
                                }
                            },
                            _ => {
                                candidate_rewire_nodes.insert(i);
                            },
                        }
                    });

                candidate_rewire_nodes.into_iter()
                    .for_each(|i|{
                        self.list_valence_fixup.push( i );
                    });

                self.valence_fixup();
            }

            let sample_list : Vec<_> = self.lookup_alive.iter().collect();
            
            let mut idx_local = {
                (0..n_query_sample).map(|x|{ let i : usize = rng.gen_range(0, n); i })
                    .map(|i|{
                        let idx = *sample_list[i];
                        idx
                    })
                    .min_by(|idx_a,idx_b| { 
                        let cost_a = f(self.nodes[*idx_a].clone(), state_query.clone());
                        let cost_b = f(self.nodes[*idx_b].clone(), state_query.clone());
                        cost_a.partial_cmp( &cost_b ).unwrap_or(Ordering::Equal)
                    }).expect("no nodes")
            };
            
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
                    _ => { idx_local },
                };

                if idx_local != idx_new {
                    idx_local = idx_new;
                } else {
                    break;
                }
            }

            Some( (idx_local,
                   *self.inverse_map.get( &idx_local ).expect("inverse map node does not exist")) )
        }
    }

    ///query nearest and then repeatedly find the best k nodes in the neighbourhood
    ///until convergence of the list
    pub fn query_nearest_k( & mut self,
                              state_query: TS,
                              f: fn(TS,TS)->f32,
                              k: usize ) -> Vec<(usize,usize)> {
        
        let (idx_local,idx_global) = match self.query_nearest( state_query.clone(), f ) {
            Some(x) => { x },
            _ => { return vec![] },
        };

        let mut items_k = HashSet::new();
        items_k.insert( idx_local );
        
        //loop until k nearest items converge

        loop {

            let mut temp = HashSet::new();
            
            for i in items_k.iter() {
                
                if !self.lookup_alive.contains(i) { panic!("not exist"); }
                
                match self.edges.get(i){
                    Some(x) => {
                        
                        let mut arr : Vec<_> = x.iter()
                            // .inspect(|y| if !self.lookup_alive.contains(y) { panic!("not exist"); } )
                            .cloned()
                            .collect();
                        
                        arr.push(*i);                        
                        arr.sort_by(|a,b|{
                            let cost_a = f(self.nodes[*a].clone(), state_query.clone());
                            let cost_b = f(self.nodes[*b].clone(), state_query.clone());
                            cost_a.partial_cmp( &cost_b ).unwrap_or(Ordering::Equal)
                        });
                        arr.iter()
                            .take(k)
                            .for_each(|y| {temp.insert(*y);} );
                    },
                    _ => {
                        if self.lookup_alive.contains(i) {
                            temp.insert(*i);
                        }
                    },
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
            let c = items_k.union( &temp );
            
            if c.count() != items_k.len() {
                items_k = temp.into_iter().collect();
            } else {
                //converged
                break;
            }
        }
        
        items_k.into_iter().map(|x|{
            (x, *self.inverse_map.get( &x ).expect("inverse map node does not exist"))
        }).collect()
    }

    ///query nearest and then repeatedly find the best nodes in the neighbourhood that is within some threshold
    ///until convergence of the list
    pub fn query_nearest_threshold( & mut self,
                                      state_query: TS,
                                      f: fn(TS,TS)->f32,
                                      threshold: f32 ) -> Vec<(usize,usize)> {

        let k = if self.lookup_alive.len() < 30 {
            self.lookup_alive.len()
        } else {
            (self.lookup_alive.len() as f32).log2() as usize
        };
        
        let mut candidates = self.query_nearest_k( state_query.clone(), f, k );
        
        let mut arr : Vec<_> = candidates.into_iter().filter(|(idx_local,_)|{
            f(state_query.clone(),self.nodes[*idx_local].clone()) < threshold
        }).collect();
        
        arr.sort_by(|(idx_a,_),(idx_b,_)|{
            let cost_a = f(self.nodes[*idx_a].clone(), state_query.clone());
            let cost_b = f(self.nodes[*idx_b].clone(), state_query.clone());
            cost_a.partial_cmp( &cost_b ).unwrap_or(Ordering::Equal)
        });
        
        arr
    }

    pub fn query_dist_node_neighbourhood_avg( & self,
                                                state_query: TS,
                                                node_idx_global: usize,
                                                f_ss_metric: fn(TS,TS)->f32,
                                                hop_dist: usize ) -> f32 {
        
        let idx_local = *self.nodes_map.get( &node_idx_global ).expect("node does not exit");

        let dist = f_ss_metric( self.nodes[idx_local].clone(), state_query.clone() );

        let d = match self.edges.get( &idx_local ) {
            Some(x) => {
                if x.is_empty(){
                    dist
                } else {
                    let d_surround = x.iter()
                        .fold( 0., |acc,i| acc + f_ss_metric( self.nodes[*i].clone(), state_query.clone() ) );
                    
                    ( d_surround + dist ) / ( x.len() + 1 ) as f32
                }
            },
            _ => { dist },
        };
        
        d
    }
    
    pub fn print_stats( & self ) {
        info!( "number of valence fixups: {}", self.stat_valence_fixups );
    }
}
