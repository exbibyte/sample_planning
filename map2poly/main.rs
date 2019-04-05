#[macro_use] extern crate log;

extern crate pretty_env_logger;

use std::env;

extern crate clap;
use clap::{Arg, App, SubCommand};

use std::collections::{HashMap,HashSet};

fn main(){
    env::set_var("LOG_SETTING", "info" );
    pretty_env_logger::init_custom_env( "LOG_SETTING" );

    //command line ---
    let matches = App::new("map2poly")
        .version("0.0")
        .author("Yuan Liu")
        .about("transform custom map to poly file for Triangle")
        .arg(Arg::with_name("map")
             .short("m")
             .help("path to map file")
             .required(true)
             .takes_value(true))
        .arg(Arg::with_name("file_output")
             .short("f")
             .help("path to output")
             .required(true)
             .takes_value(true))
        .get_matches();

    let f_in : &str = matches
        .value_of( "map" ).unwrap();

    let f_out : &str = matches
        .value_of( "file_output" ).unwrap();
    
    use std::fs::File;
    use std::io::Read;
    use std::io::BufReader;
    use std::io::BufRead;
    
    let mut f = File::open(f_in).expect("obstacle file cannot be opened");
    let b = BufReader::new(&f);
    let mut h = 0;
    let mut w = 0;
    let mut idx_h = 0;

    let mut m_orig : Vec<Vec<u8>>= vec![];
    
    for(n,line) in b.lines().enumerate() {
        if n == 0 { continue; } 
        else if n == 1 {
            h = line.unwrap()
                .split_whitespace()
                .nth(1).unwrap()
                .parse::<usize>().unwrap();
        }else if n == 2 {
            w = line.unwrap()
                .split_whitespace()
                .nth(1).unwrap()
                .parse::<usize>().unwrap();
        }else if n == 3 { continue; }
        else{
            if idx_h >= h {
                break;
            }
            
            let l = line.unwrap();
            
            let row = l.chars().take(w)
                .map(|x| if x == '.' {1} else {0}).collect::<Vec<_>>();
            
            assert_eq!(w, row.len());
            
            m_orig.push(row);
            
            idx_h += 1;
        }            
    }

    assert_eq!(m_orig.len(),h);

    println!("(h,w): ({},{})", h, w);

    let mut m = m_orig.clone();;
    
    //dilate free space of the map to avoid degenerate area for triangulation
    for i in 0..h as i32 {
        for j in 0..w as i32 {
            if m_orig[i as usize][j as usize] == 1 {
                
                let mut neighbour = vec![ (i-1,j),
                                          (i+1,j),
                                          (i,j-1),
                                          (i,j+1),
                                          (i-1,j-1),
                                          (i+1,j+1),
                                          (i-1,j+1),
                                          (i+1,j-1),
                ];

                neighbour.iter()
                    .filter(|(y,x)| *y >= 0 && *y < h as i32 && *x >= 0 && *x < w as i32 )
                    .filter(|(y,x)| m_orig[*y as usize][*x as usize] == 0 )
                    .for_each(|(y,x)| { m[*y as usize][*x as usize] = 1; } );
                
            }
        }
    }
    
    // for i in 0..h {
    //     for j in 0..w {
    //         if m[i][j] == 1 {
    //             print!(" " );
    //         }else{
    //             print!("0");
    //         }
    //     }
    //     println!();
    // }
    
    //get perimeter of free space
    let mut candidates = HashSet::new();
    
    for i in 0..h as i32 {
        for j in 0..w as i32 {
            if m[i as usize][j as usize] == 1 {

                let mut neighbour = vec![ (i-1,j),
                                          (i+1,j),
                                          (i,j-1),
                                          (i,j+1),
                                          (i-1,j-1),
                                          (i-1,j+1),
                                          (i+1,j+1),
                                          (i+1,j-1)];

                let near_obstacle = neighbour.iter()
                    .filter(|(y,x)| *y >= 0 && *y < h as i32 && *x >= 0 && *x < w as i32 )
                    .any(|(y,x)| m[*y as usize][*x as usize] == 0 );
                
                if near_obstacle ||
                    i == 0 || i == h as i32 -1 || //edge cases where free space hugs the border of the map
                    j == 0 || j == w as i32 -1 {
                    candidates.insert( (i,j) );
                }
            }
        }
    }

    let mut m_out = vec![ vec![ 0; w ]; h ];
    
    candidates.iter().for_each(|(y,x)| { m_out[*y as usize][*x as usize] = 1; } );
    
    // for i in 0..h {
    //     for j in 0..w {
    //         if m_out[i][j] == 1 {
    //             print!(" " );
    //         }else{
    //             print!("0");
    //         }
    //     }
    //     println!();
    // }
    
    let mut used = HashSet::new();

    let mut perimeters = vec![];
    
    for i in candidates.iter() {
        
        if used.contains(i) {
            continue;
        }

        let mut cur = *i;
        
        used.insert(cur);

        let mut active = vec![ cur ];

        let mut last_length = active.len();
        
        loop {
            
            let(y,x) = cur;
            
            let mut neighbour = vec![ (y-1,x),
                                       (y+1,x),
                                       (y,x-1),
                                       (y,x+1) ];
            
            for idx in 0..neighbour.len() {
                if candidates.contains( &neighbour[idx] ) &&
                    !used.contains( &neighbour[idx] ) {

                        active.push( neighbour[idx].clone() );
                        used.insert( neighbour[idx] );
                        cur = neighbour[idx];
                        break;
                    }
            }

            let new_length = active.len();
            
            if new_length == last_length {
                break;
            }

            last_length = new_length;
        }
        
        perimeters.push( active );
    }

    println!("perimenters group count: {:?}", perimeters.len() );

    let mut m_out2 = vec![ vec![ 0; w ]; h ];

    perimeters.sort_unstable_by( |a,b| b.len().cmp( &a.len() ) );

    perimeters = perimeters.iter().filter(|x| x.len() >= 3 ).cloned().collect();
    
    assert!( perimeters.len() > 0 );

    //get points in holes
    let holes = perimeters.iter().skip(1)
        .map(|x| {
            let mut centroid = (0.,0.);
            for i in x.iter() {
                centroid.0 += i.0 as f32;
                centroid.1 += i.1 as f32;
            }
            centroid.0 /= x.len() as f32;
            centroid.1 /= x.len() as f32;

            for i in 0..x.len(){
                let y = x[i].0 as i32;
                let x = x[i].1 as i32;
                let neighbours = vec![ (y+1, x),
                                       (y-1, x),
                                       (y, x+1),
                                        (y, x-1) ];
                for nn in neighbours.iter(){
                    if nn.0 >= 0 && nn.0 < h as i32 &&
                        nn.1 >= 0 && nn.1 < w as i32 {
                        if m[nn.0 as usize][nn.1 as usize] == 0 {
                            centroid.0 = nn.0 as f32;
                            centroid.1 = nn.1 as f32;
                            break;
                        }
                    }
                }
            }
   
            centroid

        }).collect::<Vec<_>>();

    let mut total_verts = 0;
    let mut total_segments = 0;
    
    for i in perimeters.iter(){
            
        total_segments += i.len();
        total_verts += i.len();
        
        // for j in i.iter(){
        //     let (y,x) = j;
        //     m_out2[*y as usize][*x as usize] = 1;
        // }
    }
    
    // for i in 0..h {
    //     for j in 0..w {
    //         if m_out2[i][j] == 1 {
    //             print!(" " );
    //         }else{
    //             print!("0");
    //         }
    //     }
    //     println!();
    // }
    
    use std::io::Write;

    let mut out = File::create( f_out ).expect("file creation");
    
    write!( &mut out, "{} 2 0 0\n",total_verts ).unwrap();

    let mut hash_verts = HashMap::new();
    
    let mut idx_vert = 0;
    for i in perimeters.iter(){
        for j in i.iter(){
            let (y,x) = j;
            write!( &mut out, "{} {} {}\n", idx_vert, x, y ).unwrap(); //x,y
            hash_verts.insert( *j, idx_vert );
            idx_vert += 1;
        }
    }

    write!( &mut out, "{} 0\n",total_segments ).unwrap();

    let mut idx_segment = 0;
    for i in perimeters.iter(){
        for j in 0..i.len() {
            let v1 = i[j];
            let v2 = i[(j+1)%i.len()];
            let idx1 = hash_verts.get(&v1).unwrap();
            let idx2 = hash_verts.get(&v2).unwrap();
            write!( &mut out, "{} {} {}\n", idx_segment, idx1, idx2 ).unwrap();
            idx_segment += 1;
        }
    }

    write!( &mut out, "{}\n", holes.len() ).unwrap();

    for (idx,(y,x)) in holes.iter().enumerate(){
        write!( &mut out, "{} {} {}\n", idx, x, y ).unwrap();
    }
    
}
