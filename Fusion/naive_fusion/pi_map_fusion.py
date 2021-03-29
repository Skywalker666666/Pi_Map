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

parser.add_argument('-inpath2',
                    action = 'store',
                    dest   = 'in_path2',
                    help   = '2nd path name of input files')

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

parser.add_argument('-sonar_src_mode',
                    action = 'store',
                    dest   = 'SONAR_MAP_SRC_MODE',
                    help   = 'the mode of input image, 1: from online map. 0: from offline map')


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

# There is two map modes: 
# 1. online_save_map_Type1_origin.sh
#    rosrun cartographer_ros cartographer_pbstream_to_ros_map
#    original grey image. grid value can be: 0 -- 255

# 2. online_save_map_Type2_3value.sh
#    rosrun map_server map_saver --occ 55 --free 45
#    three values, this is what we use now.

if np.int32(args.CARTOGRAPHER_MAP_MODE):
    lidar_map_1 = (lidar_map_pre >= (255 * 0.60)) * 255
    lidar_map_2 = ((lidar_map_pre > (255 * 0.42)) & (lidar_map_pre < (255 * 0.60))) * midle_intensity 
    lidar_map_3 = (lidar_map_pre <= (255 * 0.42)) * 0
    lidar_map_raw = np.uint8(lidar_map_1) + np.uint8(lidar_map_2) + np.uint8(lidar_map_3)
    print "Mode is: continueous value"
else:
    lidar_map_1 = (lidar_map_pre == 254) * 255
    lidar_map_2 = (lidar_map_pre == 205) * midle_intensity 
    lidar_map_3 = (lidar_map_pre ==   0) * 0
    lidar_map_raw = np.uint8(lidar_map_1) + np.uint8(lidar_map_2) + np.uint8(lidar_map_3)
    print "Mode is: 3 value"

Height, Width = lidar_map_raw.shape[:2]
#print 'Height:' + str(Height)
#print 'Width:' + str(Width)


# color to grey
lidar_map_grey = cv2.cvtColor(lidar_map_raw, cv2.COLOR_BGR2GRAY)

cv2.imshow('lidar_map_raw', lidar_map_raw)
lidar_map_raw_file = args.out_path + args.lidar_file_str + '_lidar_raw.png'
cv2.imwrite(lidar_map_raw_file, lidar_map_raw)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read lidar starting position .yaml file
#===========================================================================================
starting_pos_file = args.in_path + args.lidar_file_str + '.yaml'
with open(starting_pos_file, 'r') as f:
    doc = yaml.load(f)

origin_xyz = doc['origin']

# starting point (x,y) from lefttop, x:> y:^
# origin_x and origin_y is absolute value
origin_x = np.floor(-(origin_xyz[0] / resolution))
origin_y = np.floor(Height - (-(origin_xyz[1] / resolution)))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read lidar trajectory .yaml file
#===========================================================================================
lidar_pose_file = args.in_path + args.lidar_pose_str + '.txt'

csv.register_dialect('pi_dialect',
delimiter = ' ',
skipinitialspace=True)

with open(lidar_pose_file, 'r') as f1:
    lipose_reader = csv.reader(f1, dialect = 'pi_dialect')
    lipose_data = np.array([line for line in lipose_reader]).astype(np.float32)

lipose_trajpoints = lipose_data[:,1:3]
lipose_trajpoints[:, 0] = np.floor(lipose_trajpoints[:, 0] / resolution) + origin_x
lipose_trajpoints[:, 1] = -np.ceil(lipose_trajpoints[:, 1] / resolution) + origin_y
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read sonar .txt source file and display
#===========================================================================================
if np.int32(args.SONAR_MAP_SRC_MODE):
    print "Sonar Source Mode is: online map"
    sonar_file = args.in_path + args.sonar_file_str + '.txt'
else:
    print "Sonar Source Mode is: offline map"
    sonar_file = args.in_path2 + args.sonar_file_str + '.txt'


with open(sonar_file, 'r') as f2:
    sonar_reader = csv.reader(f2)
    sonar_data = np.array([line for line in sonar_reader]).astype(np.float32)
    
rows, columns = sonar_data.shape
#print 'sonar file size: ' + str(rows) + ' x ' + str(columns)

sonar_data_col3 = sonar_data[:, 2] 
#sonar_map_raw = sonar_data_col3.reshape((1600,1600)) #this is not suitable here

sonar_map_raw = np.zeros((sonar_board_size,sonar_board_size))
for i in range(sonar_board_size):
    if i==0:
        sonar_map_raw[:,i] = sonar_data_col3[ sonar_board_size * (i + 1) - 1 : : -1]
    else:
        sonar_map_raw[:,i] = sonar_data_col3[ sonar_board_size * (i + 1) - 1 : sonar_board_size * i - 1: -1]

#print np.amin(sonar_map_raw)
#print np.amax(sonar_map_raw)

sonar_map_raw_f = np.floor((1 - sonar_map_raw) * 255)
sonar_map_raw_u8 = sonar_map_raw_f.astype(np.uint8)
#cv2.imshow('sonar_map_raw', sonar_map_raw_u8) #1600 x 1600 too big
sonar_map_raw_file = args.out_path + args.sonar_file_str + '_sonar_raw.png'
cv2.imwrite(sonar_map_raw_file, sonar_map_raw_u8)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# insert starting point and trajectory for lidar raw map
#===========================================================================================

lidar_map_mark = lidar_map_raw
# row: the index within one row, horizontal direction.
row = origin_x.astype(np.uint32) 
# col: the index within one col, vertical direction.
col = origin_y.astype(np.uint32)
 #0/0---X / row--->     |     #0/0---col--->
 # |                    |     # |
 # |                    |     # |
 # Y / col              |     #row
 # |                    |     # |
 # |                    |     # |
 # v                    |     # v
 # (opencv plot/ image) |     # (mathmatical matrix)

#starting point
center  = (row, col) 
#ending point
center2 = (lipose_trajpoints[-1, 0], lipose_trajpoints[-1, 1])
radius = 4
# BGR
# green
color1  = (0, 255, 0) 
color2  = (0, 0, 255)  
# filled circle, solid circle
thickness1 = -1  
# thickness of circle line is 1
thickness2 =  1

cv2.circle(lidar_map_mark, center, radius, color1, thickness1)
#cv2.circle(lidar_map_mark, center, radius, color2, thickness2)
cv2.circle(lidar_map_mark, center2, radius, color2, thickness1)
cv2.imshow('lidar_map_mark',lidar_map_mark)
lidar_map_mark_file = args.out_path + args.lidar_file_str + '_lidar_mark.png'
cv2.imwrite(lidar_map_mark_file, lidar_map_mark)

#pointList = np.array([[0, 0],[10, 20],[30, 31],[50, 82],[70, 19]])
# BGR
cv2.polylines(lidar_map_mark, np.int32([lipose_trajpoints]), False, (255, 0, 0), 1); 
cv2.imshow('lidar_map_traj',lidar_map_mark)
lidar_map_traj_file = args.out_path + args.lidar_file_str + '_lidar_traj.png'
cv2.imwrite(lidar_map_traj_file, lidar_map_mark)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# crop sonar block, m x n block from 1600 x 1600
#===========================================================================================
m_start = sonar_board_size/2 - origin_y
m_end   = m_start + Height
n_start = sonar_board_size/2 - origin_x
n_end   = n_start + Width

sonar_map_block = sonar_map_raw_u8[np.int32(m_start) : np.int32(m_end), np.int32(n_start) : np.int32(n_end)]
cv2.imshow('sonar_map_block', sonar_map_block)
#print sonar_map_block.shape
sonar_map_block_file = args.out_path + args.sonar_file_str + '_sonar_block.png'
cv2.imwrite(sonar_map_block_file, sonar_map_block)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Filter Sonar map
#===========================================================================================
# because sonar_map already revert 0 and 1 for display in the above, so here threshold also need revert.
threshold_sonar_oppo = 1 - np.float32(args.threshold_sonar)
blk_1 = (sonar_map_block > midle_intensity) * 255
blk_2 = (sonar_map_block == midle_intensity) * midle_intensity  
blk_3 = np.uint8(np.logical_and((sonar_map_block > (255 * threshold_sonar_oppo)), (sonar_map_block < midle_intensity))) * 255
blk_4 = (sonar_map_block <= (255 * threshold_sonar_oppo)) * 0

sonar_map_blk_filtered = np.uint8(blk_1) + np.uint8(blk_2) + np.uint8(blk_3) + np.uint8(blk_4)
cv2.imshow('sonar_map_block_filtered', sonar_map_blk_filtered)
sonar_map_filter_file = args.out_path + args.sonar_file_str + '_sonar_filter.png'
cv2.imwrite(sonar_map_filter_file, sonar_map_blk_filtered)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Fusion
#===========================================================================================

fusion_lid_son = np.uint8(sonar_map_blk_filtered != 0) * lidar_map_grey
cv2.imshow('lidar sonar fusion map', fusion_lid_son)
fusion_map_file = args.out_path + args.lidar_file_str + '_' + args.sonar_file_str + '_fusion.png'
cv2.imwrite(fusion_map_file, fusion_lid_son)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# For display
#===========================================================================================
cv2.waitKey(0)
cv2.destroyAllWindows()

