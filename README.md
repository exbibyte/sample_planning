# Briefing
Sample Based Motion Planning.

This project is intended as educational replication of several general ideas:
- Sparseness
- Motion Primitives
- Importance Sampling

A sub-goal of this project is to integrate and leverage benefits of several of these ideas in a hybrid solution.

Inputs to program
- system dynamics and various constraints are supplied as functions
- environment obstacles

What's in it:
- Sparseness
  - Stable Sparse RRT (https://www.cs.rutgers.edu/~kb572/pubs/stable_sparse_rrt_WAFR14_LLB.pdf)
  - approximate nearest neighbour with stochastic search
- Motion Primitives:
  - lookup for feasible control for steering toward a direction (https://arxiv.org/pdf/1809.02399.pdf)
  - compile flag for enabling its use
  - adapted for run-time lookup filling
  - non-grid based greedy goal-neighbourhood search
- Importance Sampling:
  - adaptive sampling (https://journals.sagepub.com/doi/pdf/10.1177/0278364912444543)
  - used for trajectory optimization with a given fitness function

# Writeups, Benchmarks, etc..
[project](http://github.com/clearlycloudy/sample_planning/blob/master/report.pdf)

# Running Planner
* prerequisites
  * install Rust: https://rustup.rs/
  * internet connection (building project pulls in software dependencies)
  * Make (to build triangulation tool if using custom maps, or obtain executable at: https://www.cs.cmu.edu/~quake/triangle.html)
* build and run in release mode
  * Either:
    * have custom maps already generated (see Generating Custom Maps section)
    * cargo run --release --bin planner -- -p \<problem_instance_name> (other program arguments...)
    * see prob_instances.rs for predefined problem domain list
  * Or:
    * cargo run --release --bin planner -- -o \<file_obstacle> (other program arguments...)
    * sample obstacle file: obstacles/obs3.txt (randomly generated boxes)
  * Or:
    * have custom maps already generated (see Generating Custom Maps section)
    * cargo run --release --bin planner -- -e \<.ele file path> -n \<.node file path> (other program arguments...)
      * eg: cargo run --release --bin planner -- -e maps_custom/dragon_age/poly/ost100d.1.ele -n maps_custom/dragon_age/poly/ost100d.1.node -p orz000d -i 1000000 -b 300 -m dubins
    * would need to modify dynamics model file to change goal states, etc.
    * see custom maps section for .ele and .node details
* optional arguments:
  * -w: show witness node and witness representative pairs
      * drawn as a line(red) with end points (purple: witness), (blue: witness representative)
  * -i \<N>: max iterations
  * -m \<model>: dynamical model selection (see src/dynamics_* files)
      * variants: dubins, airplane (defaults to dubins)
  * -b \<N>: batch N iterations in between rendering calls
  * -h: help
* optional compile-time features:
  * usage:
    * cargo run --release --bin planner --features nn_naive,disable_pruning,(other features...) -- -p \<problem_instance_name> (other program arguments...)
  * variants: see [features] section of Cargo.toml for the list
* sample program:
  * cargo run --release --bin planner --feature nn_sample_log -- -p obs3 -m dubins -i 1000000 -b 200

# Generating Random Obstacles (a couple obstacles exists in obstacles/ folder)
* build and run in release mode with: cargo run --release --bin gen_obs -- -f \<output_file_path>
* required arguments:
  * -f \<output_file_path> (eg: cargo run --release --bin gen_obs -- -f obstacles/obs99.txt)
* optional arguments:
  * -n \<N>: number of obstacles to be generated (default: 30)
* optional features:
  * gen_obs_3d: generate boxes for in 3D domain (defaults to planar domain)
  
# Using Random Obstacles
* cargo run --release --bin planner -- -o <file_obstacle>
  * -o \<file_obstacle>: obstacle file path (eg: -o obstacles/obs2.txt)
    
# Generating Custom Maps (need to run this once in order to use custom maps):
* a set of maps that is mainly used for benchmarking purposes obtainable from https://www.movingai.com/benchmarks/grids.html can be used, these are located in the /maps_custom folder
* character movable space within a map are triangulated for use in the planner as the configuration free space
* triangulation is done using the awesome Triangle software from http://www.cs.cmu.edu/~quake/triangle.html
* the maps are converted into a format for Triangle to process and output is loadable into our planner and further extruded as triangular prisms for use with a general purpose 3D obstacle detector, these intermediate files are stored at /maps_custom/<game>/poly
* some maps might have bad triangulation not useable for the planner (I aimed for working with Dragon Age maps)
* generating intermediate files and map assets for our planner
  * 1st, compile Triangle
    * cd Triangle_v1_6
    * make
  * 2nd, run ./script_map2poly.sh (generates formatted file for Triangle, may take a while)
  * 3rd, run ./script_triangulate_poly.sh (outputs 2D triangulation result as .ele and .node files)
  * all set for use...

# Screenshots
## Simple Environments
<p align="center">
   <img src="images/screenshot0.png" alt="drawing" width="400"/>
   <img src="images/moprim.png" alt="drawing" width="500"/>	
   <img src="images/is_elite_0.png" alt="drawing" width="500"/>
   <img src="images/is_elite_1.png" alt="drawing" width="500"/>
   <img src="images/is_elite_2.png" alt="drawing" width="500"/>	
</p>

## Hard Game Maps
<p align="center">
   <img src="images/custom_map0_original.png" alt="drawing" width="400"/>
   <img src="images/custom_map1_original.png" alt="drawing" width="400"/>
   <img src="images/screenshot1.png" alt="drawing" width="400"/>
   <img src="images/screenshot2.png" alt="drawing" width="400"/>
   <img src="images/screenshot3.png" alt="drawing" width="400"/>
</p>
