
use std::collections::{HashMap,HashSet};

/// returns list of vertex coordinates, list of triangle vertex indices, max_x, max_y

pub fn load_map( path_ele: & str, path_nodes: &str ) -> (Vec<(f32,f32)>, Vec<[usize;3]>, f32, f32) {
    
    use std::fs::File;
    use std::io::Read;
    use std::io::BufReader;
    use std::io::BufRead;
    
    let mut f_ele = File::open(path_ele).expect("map file cannot be opened");
    let b_ele = BufReader::new(&f_ele);
    
    let mut f_nodes = File::open(path_nodes).expect("map file cannot be opened");
    let b_nodes = BufReader::new(&f_nodes);

    let mut num_nodes = 0;

    let mut nodes : Vec<(f32,f32)> = vec![];

    let mut max_x = 0.;
    let mut max_y = 0.;

    let mut min_x = 99999.;
    let mut min_y = 99999.;
    
    for(n,line) in b_nodes.lines().enumerate() {
        if n == 0 {
            
            num_nodes = line.unwrap()
                .split_whitespace()
                .nth(0).unwrap()
                .parse::<usize>().expect("num_nodes");
            
            nodes = vec![ (0.,0.); num_nodes ];
            
        } else {
            
            let a = line.expect("line");
            
            let idx = a.split_whitespace().nth(0).unwrap()
                .parse::<usize>().expect("node index");

            assert!(idx<num_nodes);

            let x = a.split_whitespace().nth(1).unwrap()
                .parse::<f32>().expect("node x");
            
            let y = a.split_whitespace().nth(2).unwrap()
                .parse::<f32>().expect("node y");
            
            max_x = if x > max_x { x } else { max_x };
            max_y = if y > max_y { y } else { max_y };

            min_x = if x < min_x { x } else { min_x };
            min_y = if y < min_y { y } else { min_y };
                
            nodes[idx] = (x,y);

            if n >= num_nodes {
                break;
            }
        }
    }

    //shift coordinates by -(min_x,min_y)
    nodes.iter_mut().for_each(|x| { x.0 -= min_x; x.1 -= min_y; } );
    //shift (max_x,max_y) by -(min_x,min_y)
    max_x -= min_x;
    max_y -= min_y;
    
    let mut num_tris = 0;

    let mut tris : Vec<[usize;3]> = vec![];
    
    for(n,line) in b_ele.lines().enumerate() {
        if n == 0 {
            num_tris = line.unwrap()
                .split_whitespace()
                .nth(0).unwrap()
                .parse::<usize>().expect("num_tris");
            
            tris = vec![ [0,0,0]; num_tris ];
            
        } else {
            
            let a = line.expect("line");
            
            let idx = a.split_whitespace().nth(0).unwrap()
                .parse::<usize>().expect("tri index");

            assert!(idx<num_tris);

            let n0 = a.split_whitespace().nth(1).unwrap()
                .parse::<usize>().expect("tri n0");
            
            let n1 = a.split_whitespace().nth(2).unwrap()
                .parse::<usize>().expect("tri n1");

            let n2 = a.split_whitespace().nth(3).unwrap()
                .parse::<usize>().expect("tri n2");

            tris[idx] = [n0,n1,n2];

            if n >= num_tris {
                break;
            }
        }
    }
    
    ( nodes, tris, max_x, max_y )
}
