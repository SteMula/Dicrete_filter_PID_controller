#!/bin/bash

# Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>



plot_dir=$(dirname "$(realpath "$0")")
data_dir="$plot_dir/../data"
echo "plot_dir: $plot_dir"
echo "data_dir: $data_dir"

for i in {0..4}; do
    data_file="$data_dir/scope_data_stefano_$i.tsv"
    octave-cli "$plot_dir/octave-script.m" "$data_file"
    #gp open result_stefano.png
done

# Assuming this line is correct, but ensure $data_file is the correct path
octave-cli "$plot_dir/oscillation_reduction_plot.m" "$data_file"

