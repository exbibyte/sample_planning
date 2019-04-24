use std::fmt::Debug;

pub trait States : Clone + Debug + Default {
    fn get_num_dims(&self) -> i32;
    fn get_vals(&self) -> Vec<f32>;
    fn get_vals_3(&self) -> [f32;3];
    fn set_vals(& mut self, vals: &[f32] );
}

#[derive(Clone, Copy, Debug, Default)]
pub struct States1D(pub f32);

impl States for States1D {
    fn get_num_dims(&self) -> i32 {
        1
    }
    fn get_vals(&self) -> Vec<f32> {
        vec![ self.0 ]
    }
    fn get_vals_3(&self) -> [f32;3] {
        [self.0, 0., 0.]
    }
    fn set_vals(& mut self, vals: &[f32] ){
        debug_assert!( vals.len() == 1 );
        self.0 = vals[0];
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct States2D(pub[f32;2]);

impl States for States2D {
    fn get_num_dims(&self) -> i32 {
        2
    }
    fn get_vals(&self) -> Vec<f32> {
        self.0.to_vec()
    }
    fn get_vals_3(&self) -> [f32;3] {
        [self.0[0], self.0[1], 0.]
    }
    fn set_vals(& mut self, vals: &[f32] ){
        debug_assert!( vals.len() == 2 );
        for i in 0..2 {
            self.0[i] = vals[i];
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct States3D(pub[f32;3]);

impl States for States3D {
    fn get_num_dims(&self) -> i32 {
        3
    }
    fn get_vals(&self) -> Vec<f32> {
        self.0.to_vec()
    }
    fn get_vals_3(&self) -> [f32;3] {
        [self.0[0], self.0[1], self.0[2]]
    }
    fn set_vals(& mut self, vals: &[f32] ){
        debug_assert!( vals.len() == 3 );
        for i in 0..3 {
            self.0[i] = vals[i];
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct States4D(pub[f32;4]);

impl States for States4D {
    fn get_num_dims(&self) -> i32 {
        4
    }
    fn get_vals(&self) -> Vec<f32> {
        self.0.to_vec()
    }
    fn get_vals_3(&self) -> [f32;3] {
        [self.0[0], self.0[1], self.0[2]]
    }
    fn set_vals(& mut self, vals: &[f32] ){
        debug_assert!( vals.len() == 4 );
        for i in 0..4 {
            self.0[i] = vals[i];
        }
    }
}
