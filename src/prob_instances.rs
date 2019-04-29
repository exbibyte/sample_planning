use std::collections::HashMap;

use crate::states::*;
use crate::planner_param::ParamTree;

use std::f32::consts::PI;

/// returns list of instance setup:
/// init state space, goal config space,
/// delta_s, delta_v, monte carlo prop time scale bounds
/// propagation step (optional, defaults to dynamics file setting)
/// iteration upper bound (optional, default to commandline if not provided)
/// map_name (optional, default to commandline input)

pub enum MapPath {
    Game( (& 'static str,& 'static str) ), //*.node path, *.ele path)
    Obs( (& 'static str) ), //obstable map path
}

pub fn load_3d_3d() -> HashMap< & 'static str, (States3D, States3D, ParamTree, Option<f32>, Option<u32>, Option<MapPath> )> {
    
    let mut hm = HashMap::new();
    
    hm.insert("obs3", ( States3D([0.2, 0.1, 0.]),
                        States3D([0.8, 0.8, 0.]),
                        ParamTree {
                            // delta_s: 0.07,
                            // delta_v: 0.12,
                            // delta_s: 0.05,
                            // delta_v: 0.1,
                            // delta_s: 0.0001,
                            // delta_v: 0.0002,
                            delta_s: 0.003,
                            delta_v: 0.006,
                            // prop_delta_low: 0.05,
                            // prop_delta_high: 1.,
                            prop_delta_low: 0.1,
                            prop_delta_high: 1.,   
                        },
                        // Some(0.06),
                        Some(0.015),
                        Some(250_000),
                        Some(MapPath::Obs(&"obstacles/obs3.txt")),
    ) );

    hm.insert("obs_sparse", ( States3D([0.2, 0.1, 0.]),
                              States3D([0.8, 0.8, 0.]),
                              ParamTree {
                                  delta_s: 0.02,
                                  delta_v: 0.04,
                                  prop_delta_low: 0.1,
                                  prop_delta_high: 1.,   
                              },
                              Some(0.03),
                              Some(250_000),
                              Some(MapPath::Obs(&"obstacles/obs_sparse.txt")),
    ) );

    //dragon age files

    hm.insert("den005d", ( States3D([0.5, 0.92, 0.]),
                           States3D([0.3, 0.037, 0.]),
                           ParamTree {
                               delta_s: 0.003,
                               delta_v: 0.006,
                               prop_delta_low: 0.05,
                               prop_delta_high: 1.,   
                           },
                           Some(0.065),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/den005d.1.node",
                                               &"maps_custom/dragon_age/poly/den005d.1.ele" ) ) ),
    ) );

    hm.insert("den501d", ( States3D([0.3, 0.96, -PI]),
                           States3D([0.4, 0.31, 0.]),
                           ParamTree {
                               delta_s: 0.005,
                               delta_v: 0.01,
                               prop_delta_low: 0.05,
                               prop_delta_high: 1.,   
                           },
                           Some(0.065),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/den501d.1.node",
                                               &"maps_custom/dragon_age/poly/den501d.1.ele" ) ) ),
    ) );
    
    hm.insert("den901d", ( States3D([0.17, 0.6, 0.]),
                           States3D([0.65, 0.85, PI]),
                           ParamTree {
                               delta_s: 0.003,
                               delta_v: 0.006,
                               prop_delta_low: 0.25,
                               prop_delta_high: 1.,   
                           },
                           Some(0.065),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/den901d.1.node",
                                               &"maps_custom/dragon_age/poly/den901d.1.ele" ) ) ),
    ) );

    hm.insert("hrt201d", ( States3D([0.11, 0.11, 0.]),
                           States3D([0.83, 0.78, 0.]),
                           ParamTree {
                               delta_s: 0.002,
                               delta_v: 0.004,
                               prop_delta_low: 0.25,
                               prop_delta_high: 1.,   
                           },
                           Some(0.065),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/hrt201d.1.node",
                                               &"maps_custom/dragon_age/poly/hrt201d.1.ele" ) ) ),
    ) );

    hm.insert("isound1", ( States3D([0.52, 0.89, 0.]),
                           States3D([0.82, 0.15, 0.]),
                           ParamTree {
                               delta_s: 0.0001,
                               delta_v: 0.0002,
                               // delta_s: 0.01,
                               // delta_v: 0.02,
                               prop_delta_low: 0.1,
                               // prop_delta_low: 1.,
                               prop_delta_high: 1.,   
                           },
                           // Some(0.05),
                           Some(0.04),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/isound1.1.node",
                                               &"maps_custom/dragon_age/poly/isound1.1.ele" ) ) ),
    ) );
    
    hm.insert("lak100n", ( States3D([0.27, 0.73, 0.]),
                           States3D([0.39, 0.2, 0.]),
                           ParamTree {
                               delta_s: 0.001,
                               delta_v: 0.002,
                               prop_delta_low: 0.2,
                               prop_delta_high: 1.,   
                           },
                           Some(0.05),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/lak100n.1.node",
                                               &"maps_custom/dragon_age/poly/lak100n.1.ele" ) ) ),
    ) );
    
    hm.insert("lak302d", ( States3D([0.35, 0.95, 0.]),
                           States3D([0.32, 0.04, 0.]),
                           ParamTree {
                               delta_s: 0.0075,
                               delta_v: 0.012,
                               prop_delta_low: 0.25,
                               prop_delta_high: 1.,   
                           },
                           Some(0.06),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/lak302d.1.node",
                                               &"maps_custom/dragon_age/poly/lak302d.1.ele" ) ) ),
    ) );

    hm.insert("lak504d", ( States3D([0.35, 0.925, 0.]),
                           States3D([0.32, 0.065, 0.]),
                           ParamTree {
                               delta_s: 0.0075,
                               delta_v: 0.012,
                               prop_delta_low: 0.25,
                               prop_delta_high: 1.,   
                           },
                           Some(0.06),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/lak504d.1.node",
                                               &"maps_custom/dragon_age/poly/lak504d.1.ele" ) ) ),
    ) );
    
    hm.insert("lgt604d", ( States3D([0.05, 0.9, 0.]),
                           States3D([0.95, 0.3, 0.]),
                           ParamTree {
                               delta_s: 0.0075,
                               delta_v: 0.012,
                               prop_delta_low: 0.25,
                               prop_delta_high: 1.,   
                           },
                           Some(0.06),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/lgt604d.1.node",
                                               &"maps_custom/dragon_age/poly/lgt604d.1.ele" ) ) ),
    ) );

    hm.insert("ost004d", ( States3D([0.27, 0.85, 0.]),
                           States3D([0.25, 0.15, 0.]),
                           ParamTree {
                               delta_s: 0.0002,
                               delta_v: 0.005,
                               prop_delta_low: 0.25,
                               prop_delta_high: 1.,   
                           },
                           Some(0.05),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/ost004d.1.node",
                                               &"maps_custom/dragon_age/poly/ost004d.1.ele" ) ) ),
    ) );

    hm.insert("ost100d", ( States3D([0.4, 0.65, PI]),
                           States3D([0.73, 0.33, 0.]),
                           ParamTree {
                               delta_s: 0.003,
                               delta_v: 0.006,
                               prop_delta_low: 0.2,
                               prop_delta_high: 1.,   
                           },
                           Some(0.06),
                           Some(600_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/ost100d.1.node",
                                               &"maps_custom/dragon_age/poly/ost100d.1.ele" ) ) ),
    ) );
    
    hm.insert("orz000d", ( States3D([0.25, 0.9, 0.]),
                           States3D([0.25, 0.1, 0.]),
                           ParamTree {
                               delta_s: 0.004,
                               delta_v: 0.008,
                               // delta_s: 0.001,
                               // delta_v: 0.002,                               
                               prop_delta_low: 0.025,
                               prop_delta_high: 1.,   
                           },
                           Some(0.07),
                           Some(500_000),
                           Some(MapPath::Game((&"maps_custom/dragon_age/poly/orz000d.1.node",
                                               &"maps_custom/dragon_age/poly/orz000d.1.ele" ) ) ),
    ) );    


    //sc1
    hm.insert("AcrosstheCape", ( States3D([0.02, 0.95, 0.]),
                                 States3D([0.9, 0.1, 0.]),
                                 ParamTree {
                                     delta_s: 0.002,
                                     delta_v: 0.004,
                                     prop_delta_low: 0.05,
                                     prop_delta_high: 1.,   
                                 },
                                 Some(0.065),
                                 Some(500_000),
                                 Some(MapPath::Game((&"maps_custom/sc1/poly/AcrosstheCape.1.node",
                                                     &"maps_custom/sc1/poly/AcrosstheCape.1.ele" ) ) ),
    ) );

    hm.insert("Archipelago", ( States3D([0.02, 0.95, 0.]),
                                 States3D([0.9, 0.1, 0.]),
                                 ParamTree {
                                     delta_s: 0.005,
                                     delta_v: 0.01,
                                     prop_delta_low: 0.1,
                                     prop_delta_high: 1.,   
                                 },
                                 Some(0.065),
                                 Some(500_000),
                                 Some(MapPath::Game((&"maps_custom/sc1/poly/Archipelago.1.node",
                                                     &"maps_custom/sc1/poly/Archipelago.1.ele" ) ) ),
    ) );

    hm.insert("BigGameHunters", ( States3D([0.05, 0.95, 0.]),
                                  States3D([0.9, 0.1, 0.]),
                                  ParamTree {
                                      delta_s: 0.0002,
                                      delta_v: 0.005,
                                      prop_delta_low: 0.25,
                                      prop_delta_high: 1.,   
                                  },
                                  Some(0.05),
                                  Some(500_000),
                                  Some(MapPath::Game((&"maps_custom/sc1/poly/BigGameHunters.1.node",
                                                      &"maps_custom/sc1/poly/BigGameHunters.1.ele" ) ) ),
    ) );

    hm.insert("EbonLakes", ( States3D([0.05, 0.9, 0.]),
                             States3D([0.9, 0.05, 0.]),
                             ParamTree {
                                 delta_s: 0.005,
                                 delta_v: 0.01,
                                 prop_delta_low: 0.1,
                                 prop_delta_high: 1.,   
                             },
                             Some(0.065),
                             Some(500_000),
                             Some(MapPath::Game((&"maps_custom/sc1/poly/EbonLakes.1.node",
                                                 &"maps_custom/sc1/poly/EbonLakes.1.ele" ) ) ),
    ) );

    hm.insert("RedCanyons", ( States3D([0.05, 0.9, 0.]),
                             States3D([0.9, 0.05, 0.]),
                             ParamTree {
                                 delta_s: 0.0002,
                                 delta_v: 0.005,
                                 prop_delta_low: 0.25,
                                 prop_delta_high: 1.,   
                             },
                             Some(0.05),
                             Some(500_000),
                             Some(MapPath::Game((&"maps_custom/sc1/poly/RedCanyons.1.node",
                                                 &"maps_custom/sc1/poly/RedCanyons.1.ele" ) ) ),
    ) );

    hm.insert("JungleSiege", ( States3D([0.5, 0.97, 1.5*PI]),
                              States3D([0.5, 0.05, 0.]),
                              ParamTree {
                                  delta_s: 0.0002,
                                  delta_v: 0.005,
                                  prop_delta_low: 0.25,
                                  prop_delta_high: 1.,   
                              },
                              Some(0.05),
                              Some(500_000),
                              Some(MapPath::Game((&"maps_custom/sc1/poly/JungleSiege.1.node",
                                                  &"maps_custom/sc1/poly/JungleSiege.1.ele" ) ) ),
    ) );

    hm.insert("WheelofWar", ( States3D([0.5, 0.98, 0.]),
                               States3D([0.5, 0.03, 0.]),
                               ParamTree {
                                   delta_s: 0.0055,
                                   delta_v: 0.01,
                                   prop_delta_low: 0.25,
                                   prop_delta_high: 1.,   
                               },
                               Some(0.06),
                               Some(500_000),
                               Some(MapPath::Game((&"maps_custom/sc1/poly/WheelofWar.1.node",
                                                   &"maps_custom/sc1/poly/WheelofWar.1.ele" ) ) ),
    ) );


    //dragon age 2
    hm.insert("ca_caverns1", ( States3D([0.21, 0.98, 1.5*PI]),
                               States3D([0.38, 0.02, 0.]),
                               ParamTree {
                                   delta_s: 0.002,
                                   delta_v: 0.004,
                                   prop_delta_low: 0.1,
                                   prop_delta_high: 1.,   
                               },
                               Some(0.065),
                               Some(500_000),
                               Some(MapPath::Game((&"maps_custom/dragon_age_2/poly/ca_caverns1.1.node",
                                                   &"maps_custom/dragon_age_2/poly/ca_caverns1.1.ele" ) ) ),
    ) );

    hm.insert("ht_0_hightown", ( States3D([0.15, 0.35, 0.]),
                                 States3D([0.34, 0.92, 0.]),
                                 ParamTree {
                                     delta_s: 0.002,
                                     delta_v: 0.004,
                                     prop_delta_low: 0.1,
                                     prop_delta_high: 1.,   
                                 },
                                 Some(0.065),
                                 Some(500_000),
                                 Some(MapPath::Game((&"maps_custom/dragon_age_2/poly/ht_0_hightown.1.node",
                                                     &"maps_custom/dragon_age_2/poly/ht_0_hightown.1.ele" ) ) ),
    ) );
    
    hm
}


pub fn load_4d_3d() -> HashMap< & 'static str, (States4D, States4D, ParamTree, Option<f32>, Option<u32>, Option<MapPath> )> {
    
    let mut hm = HashMap::new();
    
    hm.insert("obs3", ( States4D([0.2, 0.2, 0., 0.]),
                        States4D([0.8, 0.8, 0.4, 0.]),
                        ParamTree {
                            delta_s: 0.075,
                            delta_v: 0.12,
                            prop_delta_low: 0.05,
                            prop_delta_high: 1.,   
                        },
                        Some(0.12),
                        Some(250_000),
                        Some(MapPath::Obs(&"obstacles/obs3.txt")),
    ) );

    hm
}
