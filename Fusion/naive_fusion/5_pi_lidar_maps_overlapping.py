"""
============================================================================================
PerceptIn
============================================================================================
Lidar Map processing

Sonar Map processing

Lidar + Sonar Map fusion

maps Display

Author: Skywalker
Date  : 03-27-2019
============================================================================================
"""

# PYTHON2 ONLY
import numpy as np 
import argparse
import sys
import cv2
import csv
import yaml

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# input arguments
#===========================================================================================

parser = argparse.ArgumentParser()

parser.add_argument('-inpath',
                    action = 'store',
                    dest   = 'in_path',
                    help   = 'path name of input files')

parser.add_argument('-lfile1',
                    action = 'store',
                    dest   = 'lidar_file_str1',
                    help   = 'file name of lidar source online')


parser.add_argument('-lfile2',
                    action = 'store',
                    dest   = 'lidar_file_str2',
                    help   = 'file name of lidar source offline')


args = parser.parse_args()


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# constants parameters
#===========================================================================================
#resolution       = 0.05  # resolution of grid
midle_intensity  = 127 # grey intensity value for unknow space, corresponding to 0.5

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read lidar .pgm source file and display
#===========================================================================================

lidar_file1 = args.in_path + args.lidar_file_str1 + '.pgm'
lidar_map_pre1 = cv2.imread(lidar_file1)  

lidar_file2 = args.in_path + args.lidar_file_str2 + '.pgm'
lidar_map_pre2 = cv2.imread(lidar_file2)  

#if np.int32(args.CARTOGRAPHER_MAP_MODE):
#    lidar_map_raw = lidar_map_pre

# There is two map modes: 
# 1. online_save_map_Type1_origin.sh
#    rosrun cartographer_ros cartographer_pbstream_to_ros_map
#    original grey image. grid value can be: 0 -- 255

# 2. online_save_map_Type2_3value.sh
#    rosrun map_server map_saver --occ 55 --free 45
#    three values, this is what we use now.

lidar_map_1 = (lidar_map_pre1 == 254) * 255
lidar_map_2 = (lidar_map_pre1 == 205) * midle_intensity 
lidar_map_3 = (lidar_map_pre1 ==   0) * 0
lidar_map_raw1 = np.uint8(lidar_map_1) + np.uint8(lidar_map_2) + np.uint8(lidar_map_3)

Height1, Width1 = lidar_map_raw1.shape[:2]


lidar_map_1 = (lidar_map_pre2 == 254) * 255
lidar_map_2 = (lidar_map_pre2 == 205) * midle_intensity 
lidar_map_3 = (lidar_map_pre2 ==   0) * 0
lidar_map_raw2 = np.uint8(lidar_map_1) + np.uint8(lidar_map_2) + np.uint8(lidar_map_3)

Height2, Width2 = lidar_map_raw2.shape[:2]


print 'Height1:' + str(Height1)
print 'Width1:' + str(Width1)

print 'Height2:' + str(Height2)
print 'Width2:' + str(Width2)


# color to grey
lidar_map_grey1 = cv2.cvtColor(lidar_map_raw1, cv2.COLOR_BGR2GRAY)
lidar_map_grey2 = cv2.cvtColor(lidar_map_raw2, cv2.COLOR_BGR2GRAY)

lidar_map_grey1_crop = lidar_map_grey1[0 : min(Height1, Height2), 0 : min(Width1, Width2)]
lidar_map_grey2_crop = lidar_map_grey2[1 : min(Height1, Height2)+1, 0 : min(Width1, Width2)]

lidar_map_overlap = np.uint8(lidar_map_grey1_crop != 0) * lidar_map_grey2_crop



cv2.imshow('lidar_onoff_line_map_overlap', lidar_map_overlap)
lidar_map_raw_file = args.in_path + args.lidar_file_str1 + 'lidar_onoff_line_map_overlap.png'
cv2.imwrite(lidar_map_raw_file, lidar_map_overlap)


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# For display
#===========================================================================================
cv2.waitKey(0)
cv2.destroyAllWindows()




