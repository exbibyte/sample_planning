#!/bin/bash

for filename in maps_custom/dragon_age/maps/*.map; do
    out_file="maps_custom/dragon_age/poly/"$(basename "$filename" .map)".poly"
    # echo "$filename"
    # echo "$out_file"
    cargo  "run" "--release" "--bin" "map2poly" "--" "-m" "$filename" "-f" "$out_file"
done

for filename in maps_custom/dragon_age_2/maps/*.map; do
    out_file="maps_custom/dragon_age_2/poly/"$(basename "$filename" .map)".poly"
    cargo  "run" "--release" "--bin" "map2poly" "--" "-m" "$filename" "-f" "$out_file"
done

for filename in maps_custom/sc1/maps/*.map; do
    out_file="maps_custom/sc1/poly/"$(basename "$filename" .map)".poly"
    cargo  "run" "--release" "--bin" "map2poly" "--" "-m" "$filename" "-f" "$out_file"
done
