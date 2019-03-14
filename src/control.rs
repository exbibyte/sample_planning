use std::fmt::Debug;

pub trait Control : Clone + Sized + Debug {
    fn get_num_dims(&self) -> i32;
}

#[derive(Clone, Copy, Debug)]
pub struct Control1D(pub[f32;1]);

impl Control for Control1D {
    fn get_num_dims(&self) -> i32 {
        1
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Control3D(pub[f32;3]);

impl Control for Control3D {
    fn get_num_dims(&self) -> i32 {
        3
    }
}

