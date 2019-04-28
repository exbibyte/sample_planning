#!/bin/bash

max=200
# left=100

# for (( i=1; i <= $max; ++i ))
# do
#     cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance" "--" "-p" "obs3" "-i" "2000000" "-b" "5000"
# done

# for (( i=1; i <= $max; ++i ))
# do
#     cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance,motion_primitives" "--" "-p" "obs3" "-i" "2000000" "-b" "5000"
# done

# for (( i=1; i <= $max; ++i ))
# do
#     cargo  "run" "--release" "--bin" "planner" "--" "-p" "obs3" "-i" "2000000" "-b" "5000"
# done

# for (( i=1; i <= $left; ++i ))
# do
#     cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance" "--" "-p" "obs3" "-i" "2000000" "-b" "5000"
# done

# for (( i=1; i <= $max; ++i ))
# do
#     cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance" "--" "-p" "obs3" "-i" "2000000" "-b" "20000"
# done

# for (( i=1; i <= $max; ++i ))
# do
#     cargo  "run" "--release" "--bin" "planner" "--" "-p" "obs3" "-i" "2000000" "-b" "20000"
# done

# for (( i=1; i <= $max; ++i ))
# do
#     cargo  "run" "--release" "--bin" "planner" "--features" "motion_primitives" "--" "-p" "obs3" "-i" "2000000" "-b" "20000"
# done

# for (( i=1; i <= $max; ++i ))
# do
#     cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance" "--" "-p" "obs_sparse" "-i" "4000"
# done

# for (( i=1; i <= $max; ++i ))
# do
#     cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance,state_propagate_sample" "--" "-p" "obs_sparse" "-i" "4000"
# done

for (( i=1; i <= $max; ++i ))
do
    cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance,batch_propagate_sample" "--" "-p" "obs_sparse" "-i" "4000"
done

for (( i=1; i <= $max; ++i ))
do
    cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance,state_propagate_sample,batch_propagate_sample" "--" "-p" "obs_sparse" "-i" "4000"
done
