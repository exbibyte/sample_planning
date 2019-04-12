#!/bin/bash

for filename in maps_custom/dragon_age/poly/*.poly; do
    Triangle_v1_6/triangle "$filename"
done

for filename in maps_custom/dragon_age_2/poly/*.poly; do
    Triangle_v1_6/triangle "$filename"
done
