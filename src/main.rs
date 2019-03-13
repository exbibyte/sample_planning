#[macro_use] extern crate log;

extern crate image;
extern crate rand;
extern crate mazth;
extern crate e2rcore;
extern crate pretty_env_logger;
// extern crate chrono;

// use self::chrono::prelude::*;

use std::fs::File;
use std::io::BufReader;
use std::io::Read;
use std::path::Path;
use std::rc::Rc;
use std::f32;

use self::e2rcore::interface::i_ele;
use self::e2rcore::interface::i_game_logic::IGameLogic;
use self::e2rcore::interface::i_ui::{ InputFiltered, KeyCode, /*State, Coord*/ };
use self::e2rcore::interface::i_scheduler::IScheduler;
use self::e2rcore::interface::i_file::IParseStr;
use self::e2rcore::interface::i_md5;

use self::e2rcore::implement::render::renderer_gl;
use self::e2rcore::implement::render::util_gl;
use self::e2rcore::implement::render::texture;
use self::e2rcore::implement::render::camera;
use self::e2rcore::implement::render::light;
use self::e2rcore::implement::render::mesh;
use self::e2rcore::implement::render::primitive;

use self::e2rcore::implement::ui::ui_cam::UiCam;

use self::e2rcore::implement::cam::trackball::TrackBall;

use self::mazth::mat;

use self::rand::Rng;
use self::image::GenericImage;

// use self::rand::distributions::{IndependentSample, Range};

use std::env;

// use std::collections::{ HashSet, HashMap };

use self::e2rcore::interface::i_kernel::IKernel;

use self::e2rcore::implement::kernel::kernel_impl_001::Kernel;

use self::e2rcore::implement::file::*;

use self::e2rcore::interface::i_wavefront;

use self::e2rcore::implement::file::md5common;
use self::e2rcore::implement::file::wavefrontobj;
use self::e2rcore::implement::file::wavefrontcomp;

mod planner_param;
mod planner;
mod planner_basic;
mod stats;
mod states;
mod dynamics;

use planner_param::Param;
use planner::Planner;
use planner_basic::{PlannerBasic};
use states::*;
use dynamics::*;
// use stats::Stats;

pub fn file_open( file_path: & str ) -> Option<String> {
    let path = File::open( file_path ).expect("file path open invalid");
    let mut buf_reader = BufReader::new(path);
    let mut contents = String::new();
    match buf_reader.read_to_string( & mut contents ){
        Err( e ) => { error!("{}", e ); return None },
        _ => (),
    }
    Some(contents)
}

#[derive(Clone, Debug)]
pub struct GameState {
    _exit: bool,
    _continue_compute: bool,
    _time_game: u64,
    _is_init_run_first_time: bool,
    _iterations: u64,
}

impl Default for GameState {
    fn default() -> GameState {

        GameState {
            _exit: false,
            _continue_compute: false,
            _time_game: 0,
            _is_init_run_first_time: false,
            _iterations: 0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct GameStateChangePending {
    
}

impl Default for GameStateChangePending {
    fn default() -> GameStateChangePending {
        GameStateChangePending {
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct GameStateChangeApply {
    _end_compute: bool,
}

impl Default for GameStateChangeApply {
    fn default() -> GameStateChangeApply {
        GameStateChangeApply {
            _end_compute: false,
        }
    }
}

impl From< ComputeUnit > for GameStateChangeApply {
    fn from( _c: ComputeUnit ) -> Self {
        match _c {
            ComputeUnit::SignalEndCompute => {
                Self {
                    _end_compute: true
                }
            },
            _ => {
                Default::default()
            },
        }
    }
}

#[derive(Clone)]
pub enum ComputeUnit {
    SignalEndCompute,
    TBD,
}

#[derive(Clone)]
pub struct ComputeSchedule {
    _compute_units: Vec< ComputeUnit >,
    _index: usize,
}

impl IScheduler for ComputeSchedule {
    type Item = ComputeUnit;
    fn new( _items: &[Self::Item] ) -> ComputeSchedule {
        ComputeSchedule {
            _compute_units: _items.to_vec(),
            _index: 0,
        }
    }
}

impl Iterator for ComputeSchedule {
    type Item = Vec< ComputeUnit >;
    fn next( & mut self ) -> Option< Self::Item > {
        if self._index >= self._compute_units.len() {
            None
        } else {
            //todo
            let s = Some( vec![ self._compute_units[ self._index ].clone() ] );
            self._index += 1;
            s
        }
    }
}

impl From< (GameState, GameStateChangeApply) > for GameState {
    fn from( (_s, _a): (GameState, GameStateChangeApply) ) -> Self {
        //todo
        let mut s = _s.clone();
        if _a._end_compute {
            s._continue_compute = false;
        }
        s
    }
}

pub enum RenderObj {
    InitialRender {
        _path_shader_vs: String,
        _path_shader_fs: String,
    },
    TestGeometry {
        _time_game: u64,
        _light: light::LightAdsPoint,
        _camera: camera::Cam,
        _md5_precompute: Rc< Vec<i_md5::compute::ComputeCollection> >,
    },
    Points {
        _time_game: u64,
        _light: light::LightAdsPoint,
        _camera: camera::Cam,
        _coords: Vec<[f32;3]>,
        _colour: [u8;3],
    }
}


impl From< RenderObj > for Vec< renderer_gl::Event > {
    fn from( _r: RenderObj ) -> Self {
        match _r {
            RenderObj::InitialRender{ _path_shader_vs, _path_shader_fs } => {
                let mut render_events = vec![];
                
                info!("game logic: first time initialization.");

                let vs_src = file_open( _path_shader_vs.as_str() ).expect("vertex shader not retrieved");
                let fs_src = file_open( _path_shader_fs.as_str() ).expect("fragment shader not retrieved");
                let event_load_shader = renderer_gl::Event::LoadShader(
                    vec![
                        ( vs_src, util_gl::ShaderType::VERTEX ),
                        ( fs_src, util_gl::ShaderType::FRAGMENT ),
                    ] );
                render_events.push( event_load_shader );

                info!( "press q to quit." );

                render_events
            },
            RenderObj::TestGeometry{ _time_game, _light, _camera, _md5_precompute } =>{
                let mut render_events = vec![];
                render_events
            },
            RenderObj::Points{ _time_game, _light, _camera, _coords, _colour } =>{

                let mut render_events = vec![];

                let mut rng = rand::thread_rng();
                for i in _coords.iter(){
                    let primitive = primitive::Poly6{ _pos: mat::Mat3x1 { _val: [ i[0], i[1], i[2] ] }, _scale: mat::Mat3x1{ _val: [ 1., 1., 1.] }, _radius: 0.1 };
                    render_events.push( renderer_gl::Event::AddObj( i_ele::Ele::init( primitive ) ) );
                }

                let l = &_light;
                render_events.push( renderer_gl::Event::AddObj( i_ele::Ele::init( l.clone() ) ) );

                render_events.push( renderer_gl::Event::AddObj( i_ele::Ele::init( _camera.clone() ) ) );
                
                render_events
            },
        }
    }
}

pub struct GameLogic {
    _is_init: bool,
    _lights: Vec< light::LightAdsPoint >,
    _camera: camera::Cam,
    _delta: f32,
    _path_shader_vs: String,
    _path_shader_fs: String,
    _state: GameState,
    _uicam: UiCam,
    _planner: Box< Planner<States1D> >,
}
    
impl IGameLogic for GameLogic {

    type EventInput = InputFiltered;
    type EventRender = renderer_gl::Event;
    type GameState = GameState;
    type GameStateChangePending = GameStateChangePending;
    type GameStateChangeApply = GameStateChangeApply;
    type ComputeUnit = ComputeUnit;
    type ComputeSchedule = ComputeSchedule;
    type RenderObj = RenderObj;

    fn new() -> GameLogic {
        
        //camera
        let fov = 114f32;
        let aspect = 1f32;
        let near = 0.001f32;
        let far = 1000f32;
        
        let mut bbox_lower = [ 0f32; 3 ];
        let mut bbox_upper = [ 0f32; 3 ];
        
        let cam_foc_pos = mat::Mat3x1 { _val: [ (bbox_upper[0] + bbox_lower[0])/2.,
                                                (bbox_upper[1] + bbox_lower[1])/2.,
                                                (bbox_upper[2] + bbox_lower[2])/2., ] };
        let cam_up = mat::Mat3x1 { _val: [0f32, 0f32, 1f32] };
        let cam_pos = mat::Mat3x1 { _val: [ bbox_upper[0] + 10.,
                                            bbox_upper[1] + 10.,
                                            bbox_upper[2] + 10.] };
        let cam_id = 0;
        let cam = camera::Cam::init( cam_id, fov, aspect, near, far, cam_pos, cam_foc_pos, cam_up );

        let mut ret = GameLogic {

            _is_init: false,
            _lights: vec![],
            _camera: cam,
            _delta: 0f32,
            _path_shader_vs: String::new(),
            _path_shader_fs: String::new(),
            _state: Default::default(),
            _uicam: UiCam {
                _trackball: TrackBall::new(500.,500.),
                .. Default::default()
            },
            _planner: Box::new( PlannerBasic::init(
                Param{
                    states_init: States1D(1f32),
                    states_goal: States1D(0f32),
                    dynamics: dynamics::dynamics_1d,
                    stop_cond: dynamics::stop_cond_1d,
                }
            ) ),
        };
        
        //lights
        {
            let pos_x = 0.;
            let pos_y = 15.;
            let pos_z = 0.;
            let colour_r = 1.;
            let colour_g = 1.;
            let colour_b = 1.;
            let l = light::LightAdsPoint {
                _id: 0 as u64,
                _pos: mat::Mat3x1 { _val: [ pos_x, pos_y, pos_z ] },
                _ads_val_spec: mat::Mat3x1 { _val: [ colour_r, colour_g, colour_b ] },
                _ads_val_diff: mat::Mat3x1 { _val: [ colour_r, colour_g, colour_b ] },
                _ads_val_amb: mat::Mat3x1 { _val: [ colour_r, colour_g, colour_b ] },
            };
            ret._lights.push( l );
        }
        ret
    }

    ///do some initialization
    fn run_init_hook( & mut self ) -> Result< (), & 'static str > {
        self._path_shader_vs = String::from("asset/shader/ads.vs"); //some hard coded paths for now
        self._path_shader_fs = String::from("asset/shader/ads.fs");
        Ok( () )
    }

    ///computes changed game state given user inputs and current game state
    fn transition_states( & mut self, inputs: & [ InputFiltered ], win_offset: (i32,i32), win_size: (u32,u32) ) -> GameStateChangePending {
        //todo

        for i in inputs.iter() {
            match i {
                &InputFiltered::Button { key: KeyCode::Q, .. } => {
                    self._state._exit = true;
                },
                _ => {},
            };
            self._uicam.process( i, win_offset, win_size );
        }        

        self.set_continue_compute( true );

        // state_change
        Default::default()
    }
    fn get_states( & mut self ) -> & Self::GameState {
        & self._state
    }

    fn get_states_mut( & mut self ) -> & mut Self::GameState {
        & mut self._state
    }
    fn set_continue_compute( & mut self, b: bool ) {
        self._state._continue_compute = b;
    }
    fn continue_compute( & mut self ) -> bool {
        self._state._continue_compute
    }
    fn get_computations( & mut self, _changed_game_state: & GameStateChangePending ) -> Vec< ComputeUnit > {
        //todo: transform changed game state to additional computations

        //for now, put the main computation here (eg: run planner iteration) and ignore other hooks

        let (end_current_loop, end_all) = self._planner.plan_iteration( self._state._iterations, self._state._time_game );
        
        let mut _compute_units = vec![];
        
        if end_all {
            self._state._exit = true;
        }
        
        if end_current_loop {
            //append this to signal compute cycle is complete
            _compute_units.push( ComputeUnit::SignalEndCompute );
        }

        _compute_units
    }
    fn schedule_computes( & mut self, _computes: Vec< ComputeUnit > ) -> Vec< Self::ComputeSchedule > {
        //todo
        let mut _compute_schedule = vec![];

        _compute_schedule
    }
    fn get_renderable_components( & mut self ) -> Vec< RenderObj > {

        //todo: use game specific game logic to produce render objects instead

        let mut v = vec![];

        if !self._state._is_init_run_first_time {
            //does this once to setup some shaders
            self._state._is_init_run_first_time = true;
            let initial_render = RenderObj::InitialRender { _path_shader_fs: self._path_shader_fs.clone(),
                                                            _path_shader_vs: self._path_shader_vs.clone() };
            v.push( initial_render );
        }

        
        //update camera
        
        let focus = self._camera._focus.clone();
        let mut pos = self._camera._pos_orig;
        self._camera._pos_orig = pos;

        let axis_front = focus.minus( & pos ).unwrap().normalize().unwrap();
        let axis_right = axis_front.cross( & self._camera._up ).unwrap().normalize().unwrap();

        let move_front = axis_front.scale( self._uicam._move.0 as f32 * 0.3 ).unwrap();
        // let move_right = axis_right.scale( self._uicam._move.1 as f32 * 0.3 + 0.1 ).unwrap();
        let move_right = axis_right.scale( self._uicam._move.1 as f32 * 0.3 ).unwrap();
        let move_up = self._camera._up.normalize().unwrap().scale( self._uicam._move.2 as f32 * 0.3 ).unwrap();
        
        pos = pos.plus( & move_front.plus( & move_right ).unwrap().plus( & move_up ).unwrap() ).unwrap();
        self._uicam._move = ( 0, 0, 0 );

        let rot_matrix = self._uicam._trackball.get_rot().to_rotation_matrix( true );
        self._uicam._trackball.reset_rot();
        let offset = mat::Mat4x1 { _val: [ pos[0] - focus[0],
                                           pos[1] - focus[1],
                                           pos[2] - focus[2],
                                           0. ] };
        
        let pos_update = rot_matrix.mul_mat4x1( & offset ).unwrap();

        let pos_new = focus.plus( & mat::Mat3x1 { _val: [ pos_update[0], pos_update[1], pos_update[2] ] } ).unwrap();
        self._camera.update_pos( pos_new, focus );

        self._camera._pos_orig = pos_new;
                
        v.push( RenderObj::Points { _time_game: self._state._time_game,
                                     _light: self._lights[0].clone(),
                                     _camera: self._camera.clone(),
                                     _coords: self._planner.get_trajectories().to_vec(),
                                     _colour: [250, 250, 250] } );
        
        self._state._time_game += 1;
        
        self._state._iterations += 1;
        
        v
    }
    fn filter_renderables( & mut self, _r: Vec< RenderObj > ) -> Vec< RenderObj > {
        //todo
        _r
    }

    fn should_exit( & mut self ) -> bool {
        self._state._exit
    }
}

fn main() {

    env::set_var("LOG_SETTING", "info" );
    
    pretty_env_logger::init_custom_env( "LOG_SETTING" );
    
    let mut k : Kernel<GameLogic> = Kernel::new().unwrap();
    
    k.run().is_ok();
    
}

