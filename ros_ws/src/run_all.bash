#!/bin/bash

# Set the base folder
input_base_folder="/media/larrydong/M2-SSD/chrono"
output_base_folder="/home/larrydong/Desktop/chrono"

# Iterate over each subfolder in the input_base_folder
for subfolder in "$input_base_folder"/*; do
    if [ -d "$subfolder" ]; then
        # Extract the subfolder name
        subfolder_name=$(basename "$subfolder")
        
        # Set the input_folder and output_file
        input_folder="$subfolder/"
        output_file="$output_base_folder/$subfolder_name"

        # Run the rosrun command
        rosrun create_rosbag create_bag _input_folder:="$input_folder" _output_file:="$output_file"
    fi
done

