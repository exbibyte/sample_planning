use crate::planner_param::Param;
use crate::stats::Stats;
use super::planner::Planner;
use crate::states::States;

use std::marker::PhantomData;
use std::cell::RefCell;

use rand::Rng;

extern crate pretty_env_logger;

pub struct PlannerBasic <TS> where TS: States {
    marker_states: PhantomData<TS>,
    points: Vec<[f32;3]>,
    param: Param <TS>,
    states_cur: Option<TS>,
}

impl <TS> PlannerBasic <TS> where TS: States {
    pub fn init( param: Param<TS> ) -> PlannerBasic<TS> {
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
            param: param,
            states_cur: None,
        }
    }
}

impl <TS> Planner<TS> for PlannerBasic <TS> where TS: States {
    
    fn get_param( & self ) -> Param<TS> {
        self.param.clone()
    }
    
    fn plan_iteration( & mut self, iteration: u64, time: u64 ) -> (bool,bool) {

        //note: planner algo is implemented here for now

        info!("plan iteration start");
        for _ in 0..100 {
            print!("~");
        }
        println!();

        let mut count = 0;

        self.states_cur = Some(self.param.states_init.clone());
        
        loop {
            count += 1;
            self.states_cur = Some( (self.param.dynamics)( self.states_cur.as_ref().unwrap().clone(), 0.01 ) );
            if count % 100 == 0 {
                info!("states: {:?}", self.states_cur.as_ref().unwrap() );
            }
            
            if (self.param.stop_cond)( self.param.states_goal.clone(), self.states_cur.as_ref().unwrap().clone() ) {
                info!("goal states reached after {} iterations", count);
                break;
            }
        }
        
        info!("plan iteration end");
        
        (true,true)
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
