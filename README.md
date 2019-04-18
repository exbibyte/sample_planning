# Briefing
Sample Based Motion Planning. Work in Progress.

This project is intended as educational replication of several general ideas:
- Sparcity
- Motion Primitives
- Importance Sampling

A sub-goal of this project is to integrate and leverage benefits of several of these ideas in a hybrid solution.

Inputs to program
- system dynamics and various constraints are supplied as functions
- environment obstacles

# Progress:
- Sparcity:
  - implemented core algorithm of Stable Sparse RRT (https://www.cs.rutgers.edu/~kb572/pubs/stable_sparse_rrt_WAFR14_LLB.pdf)
- Motion Primitives:
  - [Work in Progress] lookup for feasible control for steering toward a direction (https://arxiv.org/pdf/1809.02399.pdf)
- Importance Sampling:
  - [TODO] use of entropy based sampler for shifting towards better parameterization (https://journals.sagepub.com/doi/pdf/10.1177/0278364912444543)
- Others
  - [TODO] a better nearest neighbour query with runtime add/deletion
  
# Running Planner
* build and run in release mode with:
  * cargo run --release --bin planner -- -p <problem_instance_name> (see src/prob_instances.rs)
  * cargo run --release --bin planner -- -e \<.ele file path> -n \<.node file path>
* required arguments:
  * Either:
    * -p \<problem instance name> (eg: -p obs3 ), see prob_instances.rs for predefined problem domain list
  * Or one of:
    * -o \<file_obstacle>: obstacle file path (eg: -o obstacles/obs2.txt)
    * -e \<.ele file path> -n \<.node file path> (see custom maps section)
* optional arguments:
  * -w: show witness node and witness representative pairs
      * drawn as a line(red) with end points (purple: witness), (blue: witness representative)
  * -i \<N>: max iterations
  * -m \<model>: model selection, defaults to dubins
      * \<model> variants:
      	* dubins
	* airplane (TODO)
  * -b \<N>: batch N iterations in between rendering calls
  * -h: help
* optional compile-time features:
  * usage:
    * cargo run --release --bin planner --features nn_naive,disable_pruning,(other features...) -- -p <<problem_instance_name> (other program arguments)...
  * variants:
    * motion_primitives (enables motion_primitive)
    * runge_kutta (alternative RK4 propagation method, default is Euler)
    * disable_pruning (make the propagation tree non-sparse)
    * nn_naive (use linear search for nearest neighbour query)
    * nn_sample_log (use logarithmic number of stochastic samples for nn query, must not have nn_naive to have effect, defaults to square root number of stochastic samples)
    * mo_prim_debug (render all candidates for motion primitives)
    * mo_prim_thresh_low/high (low and high neighbourhood threshold for motion primitive activation)

# Generating Random Obstacles
* build and run in release mode with: cargo run --release --bin gen_obs -- -f \<output_file_path>
* required arguments:
  * -f \<output_file_path> (eg: cargo run --release --bin gen_obs -- -f obstacles/obs99.txt)
* optional arguments:
  * -n \<N>: number of obstacles to be generated (default: 30)
  * -h: cargo run --release --bin gen_obs -- -h

# Using custom maps
* a set of maps that is mainly used for benchmarking purposes obtainable from https://www.movingai.com/benchmarks/grids.html can be used, these are located in the /maps_custom folder
* character movable space within a map are triangulated for use in the planner as the configuration free space
* triangulation is done using the awesome Triangle software from http://www.cs.cmu.edu/~quake/triangle.html
* the maps are converted into a format for Triangle to process and output is loadable into our planner, these intermediate files are stored at /maps_custom/<game>/poly
* some maps might have bad triangulation not useable for the planner (I aimed for working with Dragon Age maps)
* generating intermediate files and actualy map assets for planner:
  * run ./script_map2poly.sh (generates format for Triangle)
  * run ./script_triangulate_poly.sh (outputs 2D triangulation result as .ele and .node files)
* build and run planner with custom maps in release mode:
  * cargo run --release --bin planner -- -e \<.ele file path> -n \<.node file path>
    * (eg: cargo run --release --bin planner -- -e maps_custom/dragon_age/poly/ost100d.1.ele -n maps_custom/dragon_age/poly/ost100d.1.node -p ost100d -i 500000
  * cargo run --release --bin planner -- -p \<problem_instance_name> (see src/prob_instances.rs)

# Screenshots

<p align="center">
   <img src="images/screenshot0.png" alt="drawing" width="500" height="500"/>
</p>

## Maps from Dragon Age and its post-processed versions in planner
<p align="center">
   <img src="images/custom_map0_original.png" alt="drawing" width="500" height="500"/>
   <img src="images/custom_map0_processed.png" alt="drawing" width="500" height="500"/>
   <img src="images/custom_map1_original.png" alt="drawing" width="500" height="500"/>	
   <img src="images/custom_map1_processed.png" alt="drawing" width="500" height="500"/>		
</p>
