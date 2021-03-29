#!/bin/bash
if [[ $# -eq 0 ]]; then
    echo 'No Args. Please specifi the map name, like DEADBEEF.'
    exit 0
fi
RENAMED=$1
echo "Tag version is: $RENAMED"

if [ -f "pose_raw_data.txt" ]
then
    mv pose_raw_data.txt       ${RENAMED}_pose_raw_data.txt
    echo "${RENAMED}_pose_raw_data.txt converted."
fi

if [ -f "sonar_2d_ogm.txt" ]
then
    mv sonar_2d_ogm.txt        ${RENAMED}_sonar_2d_ogm.txt
    echo "${RENAMED}_sonar_2d_ogm.txt converted."
fi
if [ -f "our_lidar_data.txt" ]
then
    mv our_lidar_data.txt      ${RENAMED}_our_lidar_data.txt
    echo "${RENAMED}_our_lidar_data.txt converted."
fi
if [ -f "scan_raw_data_sonar.txt" ]
then
    mv scan_raw_data_sonar.txt ${RENAMED}_scan_raw_data_sonar.txt
    echo "${RENAMED}_scan_raw_data_sonar.txt converted."
fi

if ls Opti_GloPo_TX_*.txt 1> /dev/null 2>&1; then
    mv Opti_GloPo_TX_*.txt ${RENAMED}_Opti_GloPo_TX.txt
    echo "${RENAMED}_Opti_GloPo_TX.txt converted."
fi

if ls Orig_LocPo_TX_*.txt 1> /dev/null 2>&1; then
    mv Orig_LocPo_TX_*.txt ${RENAMED}_Orig_LocPo_TX.txt
    echo "${RENAMED}_Orig_LocPo_TX.txt converted."
fi

if ls Final_GloPo_TX_*.txt 1> /dev/null 2>&1; then
    mv Final_GloPo_TX_*.txt ${RENAMED}_Final_GloPo_TX.txt
    echo "${RENAMED}_Final_GloPo_TX.txt converted."
fi

if ls Lo2Map_Adjust_TX_*.txt 1> /dev/null 2>&1; then
    mv Lo2Map_Adjust_TX_*.txt ${RENAMED}_Lo2Map_Adjust_TX_.txt
    echo "${RENAMED}_Lo2Map_Adjust_TX_.txt converted."
fi

if [ -f "map.pgm" ]
then
    mv map.pgm ${RENAMED}.pgm
    echo "${RENAMED}.pgm converted."
fi
if [ -f "map.yaml" ]
then
    mv map.yaml ${RENAMED}.yaml
    echo "${RENAMED}.yaml converted."
fi
if [ -f "map.pbstream" ]
then
    mv map.pbstream ${RENAMED}_save.pbstream
    echo "${RENAMED}_save.pbstream converted."
fi



