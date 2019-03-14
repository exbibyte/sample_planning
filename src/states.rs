use std::fmt::Debug;

pub trait States : Clone + Sized + Debug {
    fn get_num_dims(&self) -> i32;
}

#[derive(Clone, Copy, Debug)]
pub struct States1D(pub f32);

impl States for States1D {
    fn get_num_dims(&self) -> i32 {
        1
    }
}

#[derive(Clone, Copy, Debug)]
pub struct States3D(pub[f32;3]);

impl States for States3D {
    fn get_num_dims(&self) -> i32 {
        3
    }
}

#[derive(Clone, Copy, Debug)]
pub struct States6D(pub[f32;6]);

impl States for States6D {
    fn get_num_dims(&self) -> i32 {
        6
    }
}
