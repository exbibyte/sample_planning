use std::fmt::Debug;
use std::ops::{Add,Mul};

pub trait States : Clone + Sized + Debug {
    fn get_num_dims(&self) -> i32;
    fn get_vals(&self) -> [f32;3];
}

#[derive(Clone, Copy, Debug)]
pub struct States1D(pub f32);

impl States for States1D {
    fn get_num_dims(&self) -> i32 {
        1
    }
    fn get_vals(&self) -> [f32;3] {
        [ self.0, 0., 0., ]
    }
}

#[derive(Clone, Copy, Debug)]
pub struct States3D(pub[f32;3]);

impl States for States3D {
    fn get_num_dims(&self) -> i32 {
        3
    }
    fn get_vals(&self) -> [f32;3] {
        self.0.clone()
    }
}

impl Add for States3D {
    type Output = States3D;
    
    fn add( self, other:States3D ) -> States3D {
        
        use std::f32::consts::PI;
        
        States3D(
            [ self.0[0] + other.0[0],
              self.0[1] + other.0[1],
              ( self.0[2] + other.0[2] + 2.*PI ) % (2.*PI) ]
        )
    }
}

impl Mul<f32> for States3D {
    type Output = States3D;
    
    fn mul( self, other: f32 ) -> States3D {
        
        use std::f32::consts::PI;
        
        States3D(
            [ self.0[0] * other,
              self.0[1] * other,
              self.0[2] * other ]
        )
    }
}


#[derive(Clone, Copy, Debug)]
pub struct States6D(pub[f32;6]);

impl States for States6D {
    fn get_num_dims(&self) -> i32 {
        6
    }
    fn get_vals(&self) -> [f32;3] {
        [ self.0[0], self.0[1], self.0[2] ]
    }
}
