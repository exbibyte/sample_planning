pub trait States {
    fn get_num_dims(&self) -> i32;
}

pub struct States3D {}

impl States for States3D {
    fn get_num_dims(&self) -> i32 { 3 }
}

