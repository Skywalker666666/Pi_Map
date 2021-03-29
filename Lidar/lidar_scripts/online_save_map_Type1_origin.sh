#!/bin/bash

# This script will save original lidar carto probability map.

if [[ $# -eq 0 ]]; then
    echo 'No Args. Please specifi the map name, like DEADBEEF.'
    exit 0
fi

FILE_NAME=$1

echo $FILE_NAME

# if we want Final Optimization, it is better to uncomment following line.
# if we just want the temperary result, don't use this
# rosservice call /finish_trajectory 0
####### you can use tab to auto-complete the full name of ID rather than 0

rosservice call /write_state "filename: '${HOME}/${FILE_NAME}_save.pbstream'" 
# or use following format:
#rosservice call /write_state "{filename: '${HOME}/xxxxxx_save.pbstream'}" 

rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/${FILE_NAME}_save -pbstream_filename=${HOME}/${FILE_NAME}_save.pbstream -resolution=0.05
