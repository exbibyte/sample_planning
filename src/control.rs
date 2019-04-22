use std::fmt::Debug;
    
pub trait Control : Clone + Debug {
    fn get_num_dims(&self) -> i32;
    fn get_vals(&self) -> Vec<f32>;
}

#[derive(Clone, Copy, Debug)]
pub struct Control1D(pub [f32;1]);

impl Control for Control1D {
    fn get_num_dims(&self) -> i32 {
        1
    }
    fn get_vals(&self) -> Vec<f32> {
        vec![ self.0[0] ]
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Control2D(pub[f32;2]);

impl Control for Control2D {
    fn get_num_dims(&self) -> i32 {
        2
    }
    fn get_vals(&self) -> Vec<f32> {
        self.0.to_vec()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Control3D(pub[f32;3]);

impl Control for Control3D {
    fn get_num_dims(&self) -> i32 {
        3
    }
    fn get_vals(&self) -> Vec<f32> {
        self.0.to_vec()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Control4D(pub[f32;4]);

impl Control for Control4D {
    fn get_num_dims(&self) -> i32 {
        4
    }
    fn get_vals(&self) -> Vec<f32> {
        self.0.to_vec()
    }
}
