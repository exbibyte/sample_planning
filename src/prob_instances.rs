use std::collections::HashMap;

use crate::states::*;

pub fn load_3d_3d() -> HashMap< & 'static str, (States3D, States3D)> {
    
    let mut hm = HashMap::new();
    
    hm.insert("obs3", ( States3D([0.2, 0.1, 0.]), States3D([0.7,0.6,0.]) ) );

    
    hm
}
