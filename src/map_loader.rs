
use std::collections::{HashMap,HashSet};

/// returns list of vertex coordinates and list of triangle vertex indices

pub fn load_map( path_ele: & str, path_nodes: &str ) -> (Vec<(f32,f32)>, Vec<[usize;3]>) {
    
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

    for(n,line) in b_nodes.lines().enumerate() {
        if n == 0 {
            
            num_nodes = line.unwrap()
                .split_whitespace()
                .nth(0).unwrap()
                .parse::<usize>().unwrap();
            
            nodes = vec![ (0.,0.); num_nodes ];
            
        } else {
            
            let a = line.unwrap();
            
            let idx = a.split_whitespace().nth(0).unwrap()
                .parse::<usize>().unwrap();

            assert!(idx<num_nodes);

            let x = a.split_whitespace().nth(1).unwrap()
                .parse::<f32>().unwrap();
            
            let y = a.split_whitespace().nth(2).unwrap()
                .parse::<f32>().unwrap();

            nodes[idx] = (x,y);
            if n >= num_nodes {
                break;
            }
        }
    }

    let mut num_tris = 0;

    let mut tris : Vec<[usize;3]> = vec![];
    
    for(n,line) in b_ele.lines().enumerate() {
        if n == 0 {
            
            num_tris = line.unwrap()
                .split_whitespace()
                .nth(0).unwrap()
                .parse::<usize>().unwrap();
            
            tris = vec![ [0,0,0]; num_tris ];
            
        } else {
            let a = line.unwrap();
            
            let idx = a.split_whitespace().nth(0).unwrap()
                .parse::<usize>().unwrap();

            assert!(idx<num_tris);

            let n0 = a.split_whitespace().nth(1).unwrap()
                .parse::<usize>().unwrap();
            
            let n1 = a.split_whitespace().nth(2).unwrap()
                .parse::<usize>().unwrap();

            let n2 = a.split_whitespace().nth(3).unwrap()
                .parse::<usize>().unwrap();

            tris[idx] = [n0,n1,n2];
        }
    }

    ( nodes, tris )
}
