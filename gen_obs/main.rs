#[macro_use] extern crate log;

extern crate pretty_env_logger;

extern crate kiss3d;

extern crate nalgebra as na;

use kiss3d::window::Window;
use kiss3d::light::Light;

extern crate serde;
use serde::{Serialize,Deserialize};

use std::env;

#[derive(Serialize, Deserialize)]
struct ObsData(pub [f32; 4]);

use na::{Vector3, UnitQuaternion, Translation3, Point3, U3};

extern crate clap;
use clap::{Arg, App, SubCommand};

#[derive(Serialize, Deserialize)]
struct Obs {
    obs: Vec<ObsData>,
}

fn generate_obstacles( num_obs: u32 ) -> Vec<ObsData> {

    use rand::Rng;
    
    //generate boxes for now
    let boxes = {

        let mut rng = rand::thread_rng();
        
        (0..num_obs).map( |_| {

            let size = rng.gen_range(0.01, 0.075);
            let x = rng.gen_range(0.25, 0.75);
            let y = rng.gen_range(0.25, 0.75);
            let z = 0.;
            
            ObsData([x,y,z,size])
    
        } ).collect::<Vec<_>>()
    };
    
    boxes
}

fn main(){
    env::set_var("LOG_SETTING", "info" );
    pretty_env_logger::init_custom_env( "LOG_SETTING" );

    //command line ---
    let matches = App::new("sample_planner")
        .version("0.0")
        .author("Yuan Liu")
        .about("obstacle generator")
        .arg(Arg::with_name("num_obs")
             .short("n")
             .help("number of obstacles")
             .default_value("30")
             .takes_value(true))
        .arg(Arg::with_name("file")
             .short("f")
             .help("output file name")
             .required(true)
             .takes_value(true))
        .get_matches();

    let num_obs : u32 = matches
        .value_of( "num_obs" ).unwrap()
        .parse().expect("number of obstacles argument invalid" );

    let output_file : &str = matches
        .value_of( "file" ).unwrap();

    info!("number of obstacles: {}", num_obs );
    
    let obstacles = Obs{ obs: generate_obstacles( num_obs ) };

    let obs_data : Vec<_> = obstacles.obs.iter()
        .map(|x| {
            let size = x.0[3] as f32;
            ( ( size, size, 0.025),
                (x.0[0] as f32, x.0[1] as f32, x.0[2] as f32) ) } ).collect();

    let mut window = Window::new("Sample Planner");
    window.set_light(Light::StickToCamera);

    let mut g1 = window.add_group();
    g1.append_translation(&Translation3::new(0.5, 0.5, 0.0));

    let mut c = g1.add_cube(1.0, 1.0, 0.0001);
    c.set_color(0.3, 0.3, 0.3);
    c.set_lines_width(5.);
    c.set_points_size(5.0);
    c.set_surface_rendering_activation(false);

    while window.render() {
        obs_data.iter()
            .for_each(|x| {
                let a = x.0;
                let b = x.1;
                let mut c = window.add_cube( a.0, a.1, a.2 );
                c.append_translation( &Translation3::new( b.0, b.1, b.2 ) );
                c.set_color(0.8, 0.8, 0.);
            });
    }

    let serialized = serde_json::to_string( &obstacles ).unwrap();

    info!("serializing to file: {}", output_file );

    use std::fs::File;
    use std::io::Write;
    
    let mut f = File::create( output_file ).expect("file creation");
    f.write_all( serialized.as_bytes() );

    println!("serialized: {}", serialized );
}
