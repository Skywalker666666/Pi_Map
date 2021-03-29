#!/bin/bash

# This script will save original lidar carto 3 value ogm map.

if [[ $# -eq 0 ]]; then
    echo 'No Args. Please specifi the map name, like DEADBEEF.'
    exit 0
fi
FILE_NAME=$1
echo $FILE_NAME


# if we want Final Optimization, it is better to uncomment following line.
# but the difference of this rosservice call and "node.FinishAllTrajectories();"
# is remained to be seen.
# if we just want the temperary result, don't use this rosservice call, then mapping procedure
# can keep running.
#rosservice call /finish_trajectory 0
##### you can use tab to auto-complete the full name of ID rather than 0

rosservice call /write_state "filename: '${HOME}/${FILE_NAME}_save.pbstream'" 

#rosrun map_server map_saver -f ${HOME}/${FILE_NAME} --occ 55 --free 45
rosrun map_server map_saver -f ${HOME}/${FILE_NAME} --occ 55 --free 45
# the parameter --occ and --free can be adjusted.
