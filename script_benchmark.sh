#!/bin/bash

max=200

for (( i=1; i <= $max; ++i ))
do
    cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance" "--" "-p" "obs3" "-i" "2000000" "-b" "1000"
done

for (( i=1; i <= $max; ++i ))
do
    cargo  "run" "--release" "--bin" "planner" "--features" "disable_witness_disturbance,motion_primitives" "--" "-p" "obs3" "-i" "2000000" "-b" "1000"
done
