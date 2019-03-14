use crate::planner_param::Param;
use crate::stats::Stats;
use crate::planner::Planner;
use crate::states::States;
use crate::control::Control;
use crate::obstacle::*;

use std::marker::PhantomData;
use std::cell::RefCell;
use rand::Rng;

extern crate pretty_env_logger;
extern crate chrono;
use chrono::{Duration,DateTime,Local};

use crate::rrt::*;
use crate::rrt::rrt::RRT;
use crate::planner_param::*;

use crate::states::*;

use zpatial::implement::bvh_median::Bvh;
use zpatial::interface::i_spatial_accel::*;

use zpatial::mazth::{
    i_bound::{ IBound, BoundType },
    i_shape::ShapeType,
    bound::AxisAlignedBBox,
    bound_sphere::BoundSphere,
};

pub struct PlannerBasic <TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    // marker_states: PhantomData<TS>,
    points: Vec<[f32;3]>,
    param: Param <TS,TC,TObs>,
    states_cur: Option<TS>,
    obstacles: Bvh<usize>,
}

impl <TS,TC,TObs> PlannerBasic <TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    pub fn init( param: Param<TS,TC,TObs>// , obstacles: ParamObstacles<TObs>
    ) -> PlannerBasic<TS,TC,TObs> {
        let points = {
            let mut rng = rand::thread_rng();
            
            (0..1000).map( |_x| {
                let x = rng.gen_range(-1., 1.);
                let y = rng.gen_range(-1., 1.);
                let z = rng.gen_range(-1., 1.);
                [ x, y, z ]
            } ).collect::<Vec<_>>()
        };

        let objs = points.iter()
            .enumerate()
            .map(|x| (x.0, AxisAlignedBBox::init( ShapeType::SPHERE, &[x.1[0]as f64, x.1[1]as f64, x.1[2]as f64, 0.01f64 ] ) ) )
            .collect::<Vec<_>>();

        let bound_objs = objs.iter().map(|x| (x.0, &x.1 as &IBound ) ).collect::<Vec<_>>();
        
        let mut obs_tree = Bvh::init(10);
        obs_tree.build_all( &bound_objs[..] ).is_ok();
        
        Self{
            // marker_states: PhantomData,
            points: points,
            param: param,
            states_cur: None,
            obstacles: obs_tree,
        }
    }
}

impl <TS,TC,TObs> Planner<TS,TC,TObs> for PlannerBasic <TS,TC,TObs> where TS: States, TC: Control, TObs: States {
    
    fn get_param( & self ) -> Param<TS,TC,TObs> {
        self.param.clone()
    }
    
    fn plan_iteration( & mut self, iteration: u64, time: u64 ) -> (bool,bool) {

        //note: planner algo is implemented here for now

        info!("plan iteration start");
        for _ in 0..100 {
            print!("~");
        }
        println!();

        let t0 = Local::now();
        let mut count = 0;

        self.states_cur = Some(self.param.states_init.clone());

        let v : Vec< (ObstacleInfo, Vec<TObs>) > = vec![];
            
        let obstacles = ParamObstacles { obstacles: v, states_info: PhantomData };
        let mut tree = rrt_base::RRT_Base::init( & self.param, & obstacles );

        tree.iterate( self.states_cur.as_ref().unwrap().clone() );

        // loop {
        //     count += 1;
        //     self.states_cur = Some( (self.param.dynamics)( self.states_cur.as_ref().unwrap().clone(), 0.01 ) );
        //     if count % 100 == 0 {
        //         info!("states: {:?}", self.states_cur.as_ref().unwrap() );
        //     }
            
        //     if (self.param.stop_cond)( self.param.states_goal.clone(), self.states_cur.as_ref().unwrap().clone() ) {
        //         info!("goal states reached after {} iterations", count);
        //         break;
        //     }
        // }
        
        let t1 = Local::now();
        let t_delta = t1.signed_duration_since(t0).num_microseconds().unwrap() as f64;
        info!("plan iteration end, duration: {}us", t_delta);

        // let end_all = true;
        // let end_current = true;
        // (end_current, end_all)
        let end_all = false;
        let end_current = true;
        (end_current, end_all)
    }
    fn get_trajectories_new( & self ) -> &[[f32;3]] {
        unimplemented!();
    }
    fn get_trajectories_past( & self ) -> &[[f32;3]] {
        unimplemented!();
    }
    fn get_trajectories_optimal( & self ) -> &[[f32;3]] {
        unimplemented!();
    }
    fn get_trajectories( & self ) -> &[[f32;3]] {
        self.points.as_ref()
    }

    fn get_states_current( & self ) -> Option<TS> {
        self.states_cur.clone()
    }
    // fn get_stats() -> Stats {

    // }
}
