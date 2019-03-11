use crate::planner_param::Param;
use crate::stats::Stats;
use super::planner::Planner;
use crate::states::States;

use std::marker::PhantomData;
use std::cell::RefCell;

use rand::Rng;

extern crate pretty_env_logger;

pub struct PlannerBasic <States> {
    marker_states: PhantomData<States>,
    points: Vec<[f32;3]>,
}

impl <States> PlannerBasic < States > {
    pub fn init( param: Param<States> ) -> PlannerBasic<States> {
        let points = {
            let mut rng = rand::thread_rng();
            
            (0..200).map( |_x| {
                let x = rng.gen_range(-10., 10.);
                let y = rng.gen_range(-10., 10.);
                let z = rng.gen_range(-10., 10.);
                [ x, y, z ]
            } ).collect()
        };
        Self{
            marker_states: PhantomData,
            points: points,
        }
    }
}

impl <States> Planner<States> for PlannerBasic<States> {
    fn plan_iteration( & mut self, iteration: u64, time: u64 ) -> bool {

        //note: planner algo is implemented here for now

        info!("plan iteration start");
        for _ in 0..100 {
            print!("~");
        }
        println!();


        
        
        
        info!("plan iteration end");
        
        true
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
    // fn get_stats() -> Stats {

    // }
}
