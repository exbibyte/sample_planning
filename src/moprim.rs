//! Motion Primitive
//!
//! This module allows saving and reteival of motion primitives.
//! This requires transformation function to map back and forth between system states and reference frame for looking up motion primitives

use std::collections::HashMap;

use crate::rrt::rrt::RRT;
use crate::states::States;
use crate::control::Control;
use crate::planner_param::{Param,ParamObstacles};

extern crate mazth;

#[derive(Debug,Clone)]
pub struct Motion <TS,TC> where TS: States, TC: Control {
    
    ///state relative uo motion primitve lookup reference frame
    pub q: TS,

    ///control input for the motion
    pub u: TC,

    ///duration of input/motion
    pub t: f32,

    ///cost of motion
    pub c: f32,
}

pub struct MoPrim <TS,TC,TObs> where TS: States, TC: Control, TObs: States {

    ///measure of closeness in state space
    pub ss_metric: fn(TS,TS)->f32,

    ///transform of world to canonical frame of motion primitive lookup
    pub xform: fn(TS,TS)->TS,

    ///transform of canonical motion primitive lookup to world coordinate
    pub xform_inv: fn(TS,TS)->TS,

    ///saved lookup data
    pub lookup: Vec<Motion<TS,TC> >,

    ///capacity of lookup
    pub capacity: usize,

    pub phantom_tobs: std::marker::PhantomData<TObs>,
    
    //temporary
    pub phantom_tc: std::marker::PhantomData<TC>,
}


impl <TS,TC,TObs> MoPrim < TS,TC,TObs > where TS: States, TC: Control, TObs: States {
    
    pub fn init( dist: fn(TS,TS)->f32,
                 transform: fn(TS,TS)->TS,
                 transform_inv: fn(TS,TS)->TS ) -> MoPrim< TS, TC, TObs > {

        use std::marker::PhantomData;
        
        MoPrim {
            ss_metric: dist,
            xform: transform,
            xform_inv: transform_inv,
            lookup: vec![],
            capacity: 1000,
            phantom_tc: PhantomData,
            phantom_tobs: PhantomData,
        }
    }

    /// add a motion to the lookup
    pub fn add_motion( & mut self, q_start: TS, q_end: TS, u: TC, t: f32, c: f32 ){

        //get q_end relative to the lookup frame of reference
        let qq_end = (self.xform)( q_start, q_end );

        //todo: algo for sparcity constraint, use randomize eviction for now
        if self.lookup.len() > self.capacity {
            
            use rand::Rng;
            let mut rng = rand::thread_rng();
            let idx : usize = rng.gen_range(0, self.lookup.len());

            self.lookup[idx] = Motion { q: qq_end,
                                        u: u,
                                        t: t,
                                        c: c };
        } else {

            self.lookup.push( Motion { q: qq_end,
                                       u: u,
                                       t: t,
                                       c: c } );
        }
    }

    ///query and return motion if the resulting state is wthin ``cost_threhold`` distance from the ``q_query``
    pub fn query_motion( & mut self, q_current: TS, q_query: TS, cost_threshold: f32 ) -> Vec<Motion<TS,TC>> {

        //get q_query relative to the lookup frame of reference
        
        let qq_query = (self.xform)( q_current, q_query );
        
        self.lookup.iter()
            .filter_map(|x|{
                if (self.ss_metric)(qq_query.clone(),x.q.clone()) > cost_threshold {
                    None
                } else {
                    Some( x.clone() )
                }
            })
            .collect()
            
    }

    pub fn print_stats( & self ) {
        info!( "motion primitive count: {}", self.lookup.len() );
    }
}
