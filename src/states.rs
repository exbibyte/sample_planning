use std::fmt::Debug;

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
