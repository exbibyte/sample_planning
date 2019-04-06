// #![feature(label_break_value)]

#[macro_use] extern crate log;

extern crate pretty_env_logger;

use std::env;
use std::collections::HashMap;

mod planner_param;
mod planner;
mod planner_basic;
mod stats;
mod states;
mod dynamics_dubins;
mod rrt;
mod control;
mod map_loader;

use planner_param::{Param,ParamObstacles,ObsVariant};
use planner::Planner;
use planner_basic::{PlannerBasic};
use states::*;
use control::*;

extern crate chrono;
use chrono::{Duration,DateTime,Local};

extern crate kiss3d;
extern crate nalgebra as na;
extern crate mazth;

use zpatial::mazth::{rbox::RecBox,triprism::TriPrism};
use zpatial::mazth::i_shape::IShape;

use na::{Vector3, UnitQuaternion, Translation3, Point3, U3};
use kiss3d::window::Window;
use kiss3d::light::Light;

extern crate ncollide3d;
use ncollide3d::procedural::{TriMesh,IndexBuffer};

use rrt::kdtree;

extern crate clap;
use clap::{Arg, App, SubCommand};

extern crate serde;
use serde::{Serialize,Deserialize};

#[derive(Serialize, Deserialize)]
struct ObsData(pub [f32; 4]);

#[derive(Serialize, Deserialize)]
struct Obs {
    obs: Vec<ObsData>,
}

impl From<&States3D> for Point3<f32> {
    fn from(i: &States3D) -> Point3<f32> {
        Point3::new( i.0[0],
                     i.0[1],
                     i.0[2] )
    }
}

///alternative to loading a obstacle file
fn generate_obstacles<TObs>() -> ParamObstacles<TObs> where TObs: States {

    use std::marker::PhantomData;
    use rand::Rng;
    
    //generate boxes for now
    let boxes = {

        let mut rng = rand::thread_rng();
        
        (0..1).map( |_| {

            let size = rng.gen_range(0.01, 0.075);
            let x = rng.gen_range(0.25, 0.75);
            let y = rng.gen_range(0.25, 0.75);
            RecBox::init( &[ x, y, 0. ], size ) //constant height obstacles
        } ).collect::<Vec<_>>()
    };
    
    ParamObstacles {
        obstacles: ObsVariant::RBOX(boxes),
        states_info: PhantomData,
    }
}

fn load_obs_from_file<TObs>(f: &str) -> ParamObstacles<TObs> where TObs: States {

    use std::marker::PhantomData;
    use std::fs::File;
    use std::io::Read;
    
    let mut s = String::new();
    let mut f = File::open(f).expect("obstacle file cannot be opened");
    f.read_to_string(& mut s).expect("file cannot be read to string");
    let obs : Obs = serde_json::from_str( s.as_str() ).expect("obstacle deserialization failed");
    
    let boxes = obs.obs.iter()
        .map(|x| {
            let coords = x.0;
            RecBox::init( &[ coords[0] as _, //x
                             coords[1] as _, //y
                             coords[2] as _], //z
                             coords[3] as _) //size
        } ).collect::<Vec<_>>();
    
    ParamObstacles {
        obstacles: ObsVariant::RBOX(boxes),
        states_info: PhantomData,
    }    
}

fn main() {

    env::set_var("LOG_SETTING", "info" );
    pretty_env_logger::init_custom_env( "LOG_SETTING" );

    //command line ---
    let matches = App::new("sample_planner")
        .version("0.0")
        .author("Yuan Liu")
        .about("a toy planner")
        .arg(Arg::with_name("witness")
             .short("w")
             .help("shows witness node and witness representative"))
        .arg(Arg::with_name("iterations")
             .short("i")
             .help("iteration upper bound")
             .default_value("300000")
             .takes_value(true))
        .arg(Arg::with_name("model")
             .short("m")
             .help("model selection")
             .default_value("dubins")
             .takes_value(true))
        .arg(Arg::with_name("obstacle")
             .short("o")
             .help("obstacle file")
             .default_value("obstacles/obs1.txt")
             .takes_value(true))
        .arg(Arg::with_name("custom_map_nodes")
             .short("n")
             .help("custom map file")
             .takes_value(true))
        .arg(Arg::with_name("custom_map_ele")
             .short("e")
             .help("custom map file")
             .takes_value(true))
        .get_matches();

    let display_witness_info = matches.is_present("witness");
    
    let iterations : u32 = matches
        .value_of( "iterations" ).unwrap()
        .parse().expect("iteration argument not a number" );

    //dynamical model selection ---
    
    let model_query = matches.value_of("model").unwrap();

    let models : HashMap<_,_> = vec![
        ("dubins", dynamics_dubins::load_model())
    ].into_iter().collect();

    let model_sel = match models.get( model_query ) {
        Some(m) => {
            info!("model selected: {}", model_query);
            let mut model_default = m.clone();
            model_default.iterations_bound = iterations;
            model_default
        },
        _ => { panic!("model not found: {}", model_query) },
    };

    let mut planner : Box<Planner<States3D,Control1D,States3D> >;
    let mut obs_copy : ParamObstacles<States3D>;

    let mut map_custom_mesh = None;
    let mut map_custom_max_x = 1.;
    let mut map_custom_max_y = 1.;
    
    match ( matches.value_of("custom_map_nodes"), matches.value_of("custom_map_ele") ) {
        (Some(path_nodes),Some(path_ele)) => {
            //load custom map

            use map_loader;

            println!("loading map.");

            let (verts,tris,max_x,max_y) = map_loader::load_map(path_ele,path_nodes);

            map_custom_max_x = max_x;
            map_custom_max_y = max_y;

            println!("loaded map.");
            
            let mesh_points = verts.iter().map(|(x,y)| Point3::new(*x,*y,0.) ).collect::<Vec<_>>();
            let mesh_tris = tris.iter().map(|x| Point3::new(x[0] as u32,x[1] as u32 ,x[2] as u32) ).collect::<Vec<_>>();

            info!("custom map triangle vert count: {}, triangle count: {}", mesh_points.len(), mesh_tris.len() );
            
            let indexbuf = IndexBuffer::Unified( mesh_tris );

            map_custom_mesh = Some(TriMesh::new( mesh_points,
                                            None,
                                            None,
                                            Some(indexbuf) ) );

            let triangle_prims = tris.iter()
                .map(|x| {
                    
                    let v0 = verts[x[0] as usize];
                    let v1 = verts[x[1] as usize];
                    let v2 = verts[x[2] as usize];

                    let height = 1.;
                    let tp = TriPrism::init( &[ v0.0 as _, v0.1 as _, -0.5,
                                                v1.0 as _, v1.1 as _, -0.5,
                                                v2.0 as _, v2.1 as _, -0.5 ], height );
                    tp

                }).collect::<Vec<_>>();

            use std::marker::PhantomData;
            
            let obs = ParamObstacles {
                obstacles: ObsVariant::TRIPRISM(triangle_prims),
                states_info: PhantomData,
            };

            obs_copy = obs.clone();
            
            planner = Box::new( PlannerBasic::init( model_sel.clone(),
                                                    obs ) );
        },
        _ => {
            
            let file_obs : & str = matches.value_of("obstacle").unwrap();
            let obs = load_obs_from_file::<States3D>(file_obs);

            info!( "plan info: {}", &model_sel );
            
            obs_copy = obs.clone();
            
            planner = Box::new( PlannerBasic::init( model_sel.clone(),
                                                    obs ) );            
        },
    }
    
    //plan ---
    let _iterations = 0;
    let _time_game = 0;

    let (end_current_loop, end_all) = planner.plan_iteration( _iterations, _time_game );

    //render ---

    let mut window = Window::new("Sample Planner");
    window.set_light(Light::StickToCamera);
    
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

    //(witness, witness representative) pairs
    let coords_witnesses : Vec<(Point3<f32>,Point3<f32>)> = planner.get_witness_pairs().iter()
        .map(|x| {
            let a = (x.0).0;
            let b = (x.1).0;
            ( Point3::new(a[0],a[1],a[2]),
              Point3::new(b[0],b[1],b[2]) )
        })
        .collect();

    let mut g2 = window.add_group();

    let mut obs_data = vec![];
    
    if map_custom_mesh.is_some(){
        //scale map size to maximum of 1.0 on longest (x,y) dimensions
        let scale = if map_custom_max_x > map_custom_max_y {
            map_custom_max_x }
        else {
            map_custom_max_y
        };
        
        let mut handle = g2.add_trimesh( map_custom_mesh.unwrap(), Vector3::new( 1./scale, 1./scale, 1.) );
        handle.set_color(0.5, 0.5, 0.5);
    } else {
        
        let mut g1 = window.add_group();
        g1.append_translation(&Translation3::new(0.5, 0.5, 0.0));

        let mut c = g1.add_cube(1.0, 1.0, 0.0001);
        c.set_color(0.3, 0.3, 0.3);
        c.set_lines_width(5.);
        c.set_points_size(5.0);
        c.set_surface_rendering_activation(false);
        
        match obs_copy.obstacles {
            ObsVariant::RBOX(ref o) => {
                obs_data = o.iter().map(|x| {
                    let l = x._size as f32;
                    ( ( l, l, 0.025),
                        (x._ori._val[0] as f32, x._ori._val[1] as f32, x._ori._val[2] as f32) ) } ).collect();
            },
            _ => {},
        }
    }
    
    while window.render() {
        
        coords.iter()
            .for_each(|x| {
                window.draw_line( &x.0, &x.1, &Point3::new(1.,1.,1.) );
            } );
            
        //domain perimeter
        window.set_point_size(0.3);    
        
        // coords_points.iter()
        //     .for_each(|x| { window.draw_point( &x, &Point3::new(0.,0.,1.) ); } );

        if display_witness_info {
            coords_witnesses.iter()
                .for_each(|x| {
                    window.draw_line( &x.0, &x.1, &Point3::new(1.,0.,0.) );
                    window.set_point_size(0.45);
                    window.draw_point( &x.0, &Point3::new(1.,0.,1.) );
                    window.draw_point( &x.1, &Point3::new(0.,0.,1.) );
                } );
        }
        
        window.set_point_size(10.);
        
        //start point
        window.draw_point( &Point3::from( &model_sel.states_init ),
                            &Point3::new(0.,1.,0.) );
        //dest point
        window.draw_point( &Point3::from( &model_sel.states_config_goal ),
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
