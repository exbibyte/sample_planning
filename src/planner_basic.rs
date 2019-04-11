use crate::planner_param::Param;
use crate::stats::Stats;
use crate::planner::Planner;
use crate::states::States;
use crate::control::Control;

use std::marker::PhantomData;
use std::cell::RefCell;
use rand::Rng;

extern crate pretty_env_logger;

use crate::instrumentation::*;

use crate::rrt::*;
use crate::rrt::rrt::RRT;
use crate::planner_param::*;

use crate::states::*;

use zpatial::implement::bvh_median::Bvh;
use zpatial::interface::i_spatial_accel::*;

use zpatial::mazth::{rbox::RecBox,triprism::TriPrism};

use zpatial::mazth::{
    i_bound::{ IBound, BoundType },
    i_shape::ShapeType,
    bound::AxisAlignedBBox,
    bound_sphere::BoundSphere,
};

pub struct PlannerBasic <TS,TC,TObs> where TS: States, TC: Control, TObs: States {

    param: Param <TS,TC,TObs>,
    param_obstacle: ParamObstacles<TObs>,
    states_cur: Option<TS>,
    trajectory: Vec<TObs>,
    trajectory_edge: Vec<((TObs,TObs),u32)>,
    trajectory_best: Vec<((TObs,TObs),u32)>,
    witness_pairs: Vec<(TObs,TObs)>,
    fini: bool,
    // rrt_tree: rrt_base::RRT_Base<TS,TC,TObs>,
    rrt_tree: sst::SST<TS,TC,TObs>,
}

impl <TS,TC,TObs> PlannerBasic <TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    pub fn init( param: Param<TS,TC,TObs>,
                 param_obs: ParamObstacles<TObs> ) -> PlannerBasic<TS,TC,TObs> {

        use zpatial::mazth::i_shape::IShape;
        
        let mut obs_tree = Bvh::init(10);

        //get bounds as [(idx,aabb_bound)]
        let bounds = match param_obs.obstacles {
            ObsVariant::RBOX(ref x) => {
                x.iter()
                    .enumerate()
                    .map(|x| (x.0, x.1.get_bound()) )
                    .collect::<Vec<_>>()
            },
            ObsVariant::TRIPRISM(ref x) => {
                x.iter()
                    .enumerate()
                    .map(|x| (x.0, x.1.get_bound()) )
                    .collect::<Vec<_>>()
            },
        };

        obs_tree.build_all( &bounds[..] ).is_ok();
        
        Self{
            param: param.clone(),
            param_obstacle: param_obs.clone(),
            states_cur: None,
            trajectory: vec![],
            trajectory_edge: vec![],
            trajectory_best: vec![],
            witness_pairs: vec![],
            fini: false,
            rrt_tree: sst::SST::init( &param,
                                       obs_tree, //contains proxy to obstacles
                                       param_obs, //contains actual obstacles
            ),
        }
    }
}

impl <TS,TC,TObs> Planner<TS,TC,TObs> for PlannerBasic <TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    
    fn get_param( & self ) -> Param<TS,TC,TObs> {
        self.param.clone()
    }
    
    fn plan_iteration( & mut self, iteration: u64, time: u64 ) -> (bool,bool) {
            
        let mut count = 0;

        self.states_cur = Some(self.param.states_init.clone());

        let mut timer = Timer::default();
        
        self.rrt_tree.iterate( self.states_cur.as_ref().unwrap().clone() );

        let t_delta = timer.dur_ms();
        
        info!("plan iteration end, duration: {}ms", t_delta);
        
        self.trajectory = self.rrt_tree.get_trajectory_config_space();
        self.trajectory_edge = self.rrt_tree.get_trajectory_edges_config_space();

        self.trajectory_best = self.rrt_tree.get_best_trajectory_config_space();
        
        self.witness_pairs = self.rrt_tree.get_witness_representatives_config_space();

        //todo
        let end_all = true;
        let end_current = true;
        (end_current, end_all)
    }
    fn get_trajectories( & self ) -> &[TObs] {
        self.trajectory.as_ref()
    }

    fn get_trajectories_edges( & self ) -> &[((TObs,TObs),u32)] {
        self.trajectory_edge.as_ref()
    }

    fn get_trajectory_best_edges( & self ) -> &[((TObs,TObs),u32)] {
        self.trajectory_best.as_ref()
    }

    fn get_states_current( & self ) -> Option<TS> {
        self.states_cur.clone()
    }
    
    fn get_witness_pairs( & self ) -> &[(TObs,TObs)] {
        self.witness_pairs.as_ref()
    }
}
