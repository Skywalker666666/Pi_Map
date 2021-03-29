"""
============================================================================================
PerceptIn
============================================================================================
Map quality evaluation

Author: Skywalker
Date  : 08-17-2019
============================================================================================
"""

# PYTHON2 ONLY
import numpy as np 
import argparse
import sys
import cv2
import csv
import yaml

# for Ubuntu:
from skimage.measure import structural_similarity as ssim
# for Mac osx:
#from skimage import measure

import matplotlib.pyplot as plt



#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# input arguments
#===========================================================================================

parser = argparse.ArgumentParser()

parser.add_argument('-inpath',
                    action = 'store',
                    dest   = 'in_path',
                    help   = 'path name of input files')


parser.add_argument('-gtfile',
                    action = 'store',
                    dest   = 'ground_truth_map_str',
                    help   = 'file name of ground truth map, .png')

args = parser.parse_args()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# constants parameters
#===========================================================================================
resolution       = 0.05  # resolution of grid
sonar_board_size = 1600 # sonar board size.
midle_intensity  = 127 # grey intensity value for unknow space, corresponding to 0.5
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read ground truth .png 
#===========================================================================================
print 'Ground Truth Map: ' + args.ground_truth_map_str + '.png'

gt_file = args.in_path + args.ground_truth_map_str + '.png'
gt_map_raw = cv2.imread(gt_file)  

Height, Width = gt_map_raw.shape[:2]
print 'Height:' + str(Height)
print 'Width:' + str(Width)

# color to grey
gt_map_grey = cv2.cvtColor(gt_map_raw, cv2.COLOR_BGR2GRAY)
cv2.imshow('gt_map_raw', gt_map_grey)

#lidar_map_raw_file = args.out_path + args.lidar_file_str + '_lidar_raw.png'
#cv2.imwrite(lidar_map_raw_file, lidar_map_raw)
 
occ_grid = sum(sum(gt_map_grey == 0 ))
free_grid = sum(sum(gt_map_grey == 255 ))
unknown_grid = sum(sum(gt_map_grey == 127 ))

print "occ_grid:"
print occ_grid
print "free_grid:"
print free_grid
print "unknown_grid:"
print unknown_grid

print "sum of occ, free, unknown: "
print occ_grid + free_grid + unknown_grid

print "H x W: "
print Height * Width

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# For display
#===========================================================================================
cv2.waitKey(0)
cv2.destroyAllWindows()










