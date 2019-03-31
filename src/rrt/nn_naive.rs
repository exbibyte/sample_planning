extern crate pretty_env_logger;

use std::collections::{HashSet,HashMap};
use std::marker::PhantomData;

use rand::Rng;

use crate::states::States;
use crate::control::Control;
use crate::planner_param::{Param,ParamObstacles};

use crate::rrt::sst::Node;

#[derive(Default)]
pub struct NN_Naive<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    pub phantom_ts: PhantomData< TS >,
    pub phantom_tc: PhantomData< TC >,
    pub phantom_tobs: PhantomData< TObs >,
}

impl<TS,TC,TObs> NN_Naive<TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    
    ///return the node index of the best node with respect to cost function within delta_v vicinity or else return the node index of the nearest node
    pub fn query_nearest_state_active( & self,
                                       sample_state: TS,
                                       ns: & Vec< Node<TS> >,
                                       nodes_active: & HashSet< usize >,
                                       param: & Param<TS,TC,TObs>,
                                       delta_v: f32 ) -> usize {

        //consider vicinity
        let node_best = nodes_active.iter()
            .map(|x| &ns[*x] )
            .filter(|x| (param.ss_metric)( x.state.clone(), sample_state.clone() ) < delta_v )
            .min_by_key(|x| x.cost );

        match node_best {
            Some(x) => { //non-empty
                x.id
            },
            _ => { //else consider nearest one
                let node_nearest = nodes_active.iter()
                    .map(|x| &ns[*x] )
                    .min_by_key(|x| x.cost ).expect("no nodes active");
                node_nearest.id
            },
        }
    }

    ///return witness index within delta_s of query_state, or else return None
    pub fn query_nearest_witness( & self,
                                  query_state: TS,
                                  witnesses: & Vec< TS >,
                                  param: & Param<TS,TC,TObs>,
                                  delta_s: f32 ) -> Option<usize> {

        use std::cmp::Ordering::Equal;
        
        let witness_match = witnesses.iter().enumerate()
            .filter_map(|(idx,x)|
                        {
                            let dist = (param.ss_metric)( x.clone(), query_state.clone() );
                            if dist< delta_s {
                                Some( (idx,dist) )
                            } else {
                                None
                            }
                        } )
            .min_by(|a,b| a.1.partial_cmp(&b.1).unwrap_or(Equal) );
        match witness_match {
            Some(x) => {
                Some(x.0)
            },
            _ => {
                None
            },
        }
    }
}
