use std::ops::Index;
use std::cmp::Ordering;
use std::fmt::Debug;

use chrono::{Duration,DateTime,Local};

pub trait DimMetric : Clone + Copy + Index<usize, Output=f32> + Debug {
    fn get_dims(&self) -> usize;
    fn get_vals(&self) -> &[f32];
    fn set_vals(&mut self, vals: &[f32]);
    fn init(v: &[f32]) -> Self;
}

#[derive(Clone,Copy,Debug)]
pub struct Bound<T> where T: DimMetric {
    pub low: T,
    pub upp: T,
}

impl<T> Bound<T> where T: DimMetric {
    #[inline]
    fn init( l: T, u: T ) -> Self {
        Self{
            low: l,
            upp: u,
        }
    }
    pub fn merge( a: &Self, b: &Self ) -> Self {
        
        let s = a.low.get_dims();
        
        assert_eq!( s, b.low.get_dims() );
        
        let arr_a_lo = a.low.get_vals();
        let arr_a_up = a.upp.get_vals();
        let arr_b_lo = b.low.get_vals();
        let arr_b_up = b.upp.get_vals();
        
        let mut v_low = vec![0f32; s];
        let mut v_upp = vec![0f32; s];
        
        for i in 0..s {
            v_low[i] = arr_a_lo[i].min( arr_b_lo[i] );
            v_upp[i] = arr_a_up[i].max( arr_b_up[i] );
        }

        let mut ret = a.clone();
        ret.low.set_vals( &v_low[..] );
        ret.upp.set_vals( &v_upp[..] );
        ret
    }
    pub fn mid( &self, axis: usize ) -> f32 {
        (self.low[axis] + self.upp[axis])/2.
    }
}

#[derive(Clone,Copy,Debug)]
pub struct Coord3 {
    c: [f32;3],
}

impl DimMetric for Coord3 {
    fn get_dims(&self) -> usize { 3 }
    fn get_vals(&self) -> &[f32] { &self.c[..] }
    fn set_vals(&mut self, vals: &[f32]) {
        for i in 0..3 {
            self.c[i] = vals[i];
        }
    }
    fn init(v: &[f32])->Coord3{
        let data = [ v[0], v[1], v[2] ];
        Coord3{
            c: data,
        }
    }
}

impl Index<usize> for Coord3 {
    type Output = f32;
    fn index(&self, index: usize) -> &f32 {
        &self.c[index]
    }
}

#[test]
fn test_bound_merge(){
    let b1 : Bound<Coord3> = Bound {
        low: Coord3 { c: [-5., 10., 99.5] },
        upp: Coord3 { c: [ 15., 20., 109.5] },
    };

    let b2 : Bound<Coord3> = Bound {
        low: Coord3 { c: [-25., 10., 85.5] },
        upp: Coord3 { c: [ 15., 25., 100.5] },
    };
    let b_merge = Bound::merge( &b1, &b2 );

    let merge_low = b_merge.low.get_vals();
    let merge_upp = b_merge.upp.get_vals();

    assert_eq!( merge_low.len(), 3 );
    assert_eq!( merge_upp.len(), 3 );
    
    let expected_low = [ -25., 10., 85.5];
    let expected_upp = [ 15., 25., 109.5];
    
    let eps = 0.0001;
    
    for i in 0..3 {
        assert!( expected_low[i] - eps < merge_low[i] );
        assert!( expected_low[i] + eps > merge_low[i] );
        
        assert!( expected_upp[i] - eps < merge_upp[i] );
        assert!( expected_upp[i] + eps > merge_upp[i] );
    }
}

#[derive(Debug,Clone,Copy)]
pub enum Variant<T> where T: DimMetric {
    Leaf {
        coord: T,
        idx: usize,
        dep: usize,
    },
    Internal {
        b: Bound<T>,
        axis: usize,
        axis_coord: f32,
        l: usize,
        r: usize,
        dep: usize,
    },
    Nil {},
}

pub struct KdTree<T> where T: DimMetric {
    nodes: Vec<Variant<T>>,
    
}

impl<T> KdTree<T> where T: DimMetric {
    pub fn init() -> KdTree<T> {
        KdTree {
            nodes: vec![],
        }
    }
}

impl<T> KdTree<T> where T: DimMetric {

    ///returns (index,depth) of the leaf node
    pub fn search( & mut self, coord_new: T ) -> (usize,usize) {

        let d = coord_new.get_dims();
        
        let mut i = 0;
        let mut depth = 0;
        loop {
            // println!("loop {}", i);
            match & self.nodes[i] {
                Variant::Leaf{coord,idx,dep} => {
                    return (*idx,depth)
                },
                Variant::Internal{ b, axis, axis_coord, l, r,dep } => {
                    let val = coord_new[*axis];
                    if val < *axis_coord {
                        i = *l;
                    } else {
                        i = *r;
                    }
                    depth += 1;
                },
                Variant::Nil{} => { panic!(); },
            }

        }
    }
    
    pub fn insert( & mut self, coord_new: T ) {

        let d = coord_new.get_dims();
        
        if self.nodes.len() == 0 { //insert new root
            self.nodes.push( Variant::Leaf{ coord: coord_new, idx: 0, dep: 1 } );
        } else {
            let (leaf_idx, depth) = self.search( coord_new );
            match self.nodes[leaf_idx] {
                Variant::Leaf{coord,idx,..} => {
                    
                    let bound1 = Bound::init( coord.clone(), coord.clone() );
                    let bound2 = Bound::init( coord_new, coord_new );
                    let bound_merge = Bound::merge(&bound1,&bound2);
                    
                    let idx1 = self.nodes.len();
                    let idx2 = idx1+1;
                    
                    self.nodes.push( Variant::Leaf{coord: coord.clone(), idx: idx1, dep: depth+1} );
                    self.nodes.push( Variant::Leaf{coord: coord_new, idx: idx2, dep: depth+1} );

                    let axis_sel = depth % d;
                    let axis_mid = bound_merge.mid(axis_sel);
                    self.nodes[idx] = Variant::Internal{ b: bound_merge,
                                                         axis: axis_sel,
                                                         axis_coord: axis_mid,
                                                         l: idx1,
                                                         r: idx2,
                                                         dep: depth };

                    // println!("axis: {:?}, axis_mid: {:?}, coord: {:?}, coord_new: {:?} ", axis_sel, axis_mid,coord,coord_new );
                },
                _ => { panic!{}; },
            }
        }
        // println!("inserted new node");
    }

    ///collect hyperplanes, points, and bboxes in the leaves
    pub fn collect_shapes(&self) -> (Vec<(usize,f32)>, Vec<[f32;3]>, Vec<([f32;3],[f32;3])> ) {
        
        let mut points = vec![];
        let mut planes = vec![];
        let mut bboxes = vec![];
        
        for i in self.nodes.iter(){
            match i {
                Variant::Leaf{coord,idx,..} => {
                    
                    let arr = &coord.get_vals();
                    
                    points.push( [ arr[0], arr[1], arr[2] ] );
                },
                Variant::Internal{ b, axis, axis_coord, l, r,.. } => {
                    planes.push( (*axis,*axis_coord) );
                    
                    let bound1 = b.low.get_vals();
                    let bound2 = b.upp.get_vals();
                    let bound = ( [ bound1[0], bound1[1], bound1[2] ],
                                  [ bound2[0], bound2[1], bound2[2] ] );
                    bboxes.push( bound );
                },
                _ => {},
            }
        }

        (planes,points,bboxes)
    }
    
    fn nearest( & mut self, coord: T ) {
        
    }

    //fn split
    pub fn get_depths(&self) -> (f32,f32,f32) {

        let mut dep_max = 0;
        for i in self.nodes.iter() {
            match i {
                Variant::Leaf{coord,idx,dep} => {
                    dep_max = dep_max.max(*dep);
                },
                _ => {},
            }
        }

        println!("dep max collected: {}", dep_max);
                    
        if self.nodes.len() == 0 {
            (0.,0.,0.)
        } else {
            
            let mut depth_avg = 0f32;
            let mut depth_max = -1f32;
            let mut depth_min = 999999999f32;
            let mut count = 0;
            let mut q = vec![(0,0.)];
            
            while !q.is_empty() {
                
                let l = q.pop().unwrap();
                
                let i = l.0;
                let d = l.1;
                
                match self.nodes[i] {
                    Variant::Leaf{coord,idx,dep} => {
                        depth_avg += d;
                        depth_max = depth_max.max(d);
                        depth_min = depth_min.min(d);
                        count += 1;
                    },
                    Variant::Internal{ b, axis, axis_coord, l, r, dep } => {
                        q.push( ( l, d+1.) );
                        q.push( ( r, d+1.) );
                    },
                    _ => {},
                }
            }
            (depth_avg / count as f32, depth_min, depth_max )
        }
    }
}


#[test]
fn stress_insertion(){
    
    let mut t : KdTree<Coord3> = KdTree::init();
    
    use rand::Rng;
    let mut rng = rand::thread_rng();

    let mut randpoints = vec![];
    for _ in 0..1000_000 {
        let x = rng.gen_range(0., 1.);
        let y = rng.gen_range(0., 1.);
        let z = rng.gen_range(0., 1.);
        randpoints.push( [x,y,z] );
    }

    let t0 = Local::now();
    
    for i in randpoints.iter(){
        t.insert( Coord3::init( i ) );
    }

    let t1 = Local::now();
    let t_delta = t1.signed_duration_since(t0).num_microseconds().unwrap() as f64;
    println!("insertation duration: {}us", t_delta);

    println!("depth (avg,min,max): {:?}", t.get_depths() );
    
}
