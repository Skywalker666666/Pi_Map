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

parser.add_argument('-th',
                    action = 'store',
                    dest   = 'threshold_sonar',
                    help   = 'threshold for sonar filter, quantification for OccupancyGrid. Required: > 0.5')


parser.add_argument('-in_map_mode',
                    action = 'store',
                    dest   = 'CARTOGRAPHER_MAP_MODE',
                    help   = 'the mode of input image, 1: from cartographer map saver. 0: from ros map saver')

parser.add_argument('-inpath',
                    action = 'store',
                    dest   = 'in_path',
                    help   = 'path name of input files')

parser.add_argument('-outpath',
                    action = 'store',
                    dest   = 'out_path',
                    help   = 'path name of output files')

parser.add_argument('-lfile',
                    action = 'store',
                    dest   = 'lidar_file_str',
                    help   = 'file name of lidar source data and original position, .pgm and .yaml')


parser.add_argument('-sfile',
                    action = 'store',
                    dest   = 'sonar_file_str',
                    help   = 'file name of sonar source data, .txt')

parser.add_argument('-pfile',
                    action = 'store',
                    dest   = 'lidar_pose_str',
                    help   = 'file name of lidar pose, .txt')

args = parser.parse_args()


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# constants parameters
#===========================================================================================
resolution       = 0.05  # resolution of grid
sonar_board_size = 1600 # sonar board size.
midle_intensity  = 127 # grey intensity value for unknow space, corresponding to 0.5
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read lidar .pgm source file and display
#===========================================================================================

lidar_file = args.in_path + args.lidar_file_str + '.pgm'
lidar_map_pre = cv2.imread(lidar_file)  

print np.max(lidar_map_pre)
print np.min(lidar_map_pre)

new_list = [np.sum(lidar_map_pre == i) for i in range(255) ]
print new_list
#print np.sum(lidar_map_pre == 100)



