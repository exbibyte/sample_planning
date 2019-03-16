#[macro_use] extern crate log;

extern crate pretty_env_logger;

use std::env;

mod planner_param;
mod planner;
mod planner_basic;
mod stats;
mod states;
mod dynamics;
mod rrt;
mod control;

use planner_param::{Param,ParamObstacles};
use planner::Planner;
use planner_basic::{PlannerBasic};
use states::*;
use dynamics::*;
use control::*;
// // use stats::Stats;

extern crate kiss3d;
extern crate nalgebra as na;
extern crate mazth;

use zpatial::mazth::rbox::RecBox;
use zpatial::mazth::i_shape::IShape;

use na::{Vector3, UnitQuaternion, Translation3, Point3, U3};
use kiss3d::window::Window;
use kiss3d::light::Light;

impl From<&States3D> for Point3<f32> {
    fn from(i: &States3D) -> Point3<f32> {
        Point3::new( i.0[0],
                     i.0[1],
                     i.0[2] )
    }
}

fn generate_obstacles<TObs>() -> ParamObstacles<TObs> where TObs: States {

    use std::marker::PhantomData;
    use rand::Rng;
    
    //generate boxes for now
    let boxes = {

        let mut rng = rand::thread_rng();
        
        (0..30).map( |_| {

            let size = rng.gen_range(0.01, 0.075);
            let x = rng.gen_range(0.15, 0.85);
            let y = rng.gen_range(0.15, 0.85);
            RecBox::init( &[ x, y, 0. ], size ) //constant height obstacles
        } ).collect::<Vec<_>>()
    };
    
    ParamObstacles {
        obstacles: boxes,
        states_info: PhantomData,
    }
}

fn main() {

    env::set_var("LOG_SETTING", "info" );
    pretty_env_logger::init_custom_env( "LOG_SETTING" );

    //plan ---
    
    let param_sel = 0;
    let params = [
        Param{
            // states_init: States1D(1f32),
            // states_goal: States1D(0f32),
            // dynamics: dynamics_1d,
            // stop_cond: stop_cond_1d,
            //use a Dubins car model (constant velocity, change in heading only, 2d positions)
            states_init: States3D([0.05, 0.05, 0.]), //positions (x,y), heading angle
            states_config_goal: States3D([0.95,0.95,0.]), //(x,y,heading angle)
            dynamics: dynamics_3d_1d, //1 input for change in heading
            stop_cond: stop_cond_3d,
            sim_delta: 0.001f32,
            dist_delta: 0.08f32,
            project_state_to_config: project_dubins_car_state_to_config,
            param_sampler: sampler_parameter_space_dubins_car,
        },
    ];

    let obs = generate_obstacles::<States3D>();
    let obs_copy = obs.clone();
    let mut planner : Box<Planner<States3D,Control1D,States3D> > = 
        Box::new( PlannerBasic::init( params[param_sel].clone(),
                                      obs ) );
    
    let _iterations = 0;
    let _time_game = 0;
    // let (end_current_loop, end_all) = planner.plan_iteration( _iterations, _time_game );

    //render ---
    
    let mut window = Window::new("Sample Planner");
    window.set_light(Light::StickToCamera);

    let (end_current_loop, end_all) = planner.plan_iteration( _iterations, _time_game );
    
    let coords_points : Vec<Point3<f32>> = planner.get_trajectories().iter()
        .map(|x| Point3::from(x) )
        .collect();
        
    let coords : Vec<(Point3<f32>,Point3<f32>)> = planner.get_trajectories_edges().iter()
        .map(|x| {
            let a = (x.0).0;
            let b = (x.1).0;
            ( Point3::new(a[0],a[1],a[2]),
              Point3::new(b[0],b[1],b[2]) )
        })
        .collect();

    let mut g1 = window.add_group();
    g1.append_translation(&Translation3::new(0.5, 0.5, 0.0));

    let mut c = g1.add_cube(1.0, 1.0, 0.0001);
    c.set_color(0.3, 0.3, 0.3);
    c.set_lines_width(5.);
    c.set_points_size(5.0);
    c.set_surface_rendering_activation(false);


    let obs_data : Vec<_> = obs_copy.obstacles.iter()
        .map(|x| {
            let l = x._size as f32;
            ( ( l, l, 0.025),
              (x._ori._val[0] as f32, x._ori._val[1] as f32, x._ori._val[2] as f32) ) } ).collect();
    
    while window.render() {

        // let (end_current_loop, end_all) = planner.plan_iteration( _iterations, _time_game );

        // let coords_points : Vec<Point3<f32>> = planner.get_trajectories().iter()
        //     .map(|x| Point3::from(x) )
        //     .collect();
        
        // let coords : Vec<(Point3<f32>,Point3<f32>)> = planner.get_trajectories_edges().iter()
        //     .map(|x| {
        //         let a = (x.0).0;
        //         let b = (x.1).0;
        //         ( Point3::new(a[0],a[1],a[2]),
        //           Point3::new(b[0],b[1],b[2]) )
        //     })
        //     .collect();
        
        coords.iter()
            .for_each(|x| {
                window.draw_line( &x.0, &x.1, &Point3::new(1.,1.,1.) );
            } );

        //domain perimeter
        window.set_point_size(0.3);
        coords_points.iter()
            .for_each(|x| { window.draw_point( &x, &Point3::new(0.,0.,1.) ); } );

        
        window.set_point_size(10.);
        
        //start point
        window.draw_point( &Point3::from(&params[param_sel].states_init),
                            &Point3::new(0.,1.,0.) );
        //dest point
        window.draw_point( &Point3::from(&params[param_sel].states_config_goal),
                            &Point3::new(1.,0.,0.) );

        obs_data.iter()
            .for_each(|x| {
                let a = x.0;
                let b = x.1;
                let mut c = window.add_cube( a.0, a.1, a.2 );
                c.append_translation( &Translation3::new( b.0, b.1, b.2 ) );
                c.set_color(0.8, 0.8, 0.);
            });
    }
}
