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

parser.add_argument('-in_map_mode',
                    action = 'store',
                    dest   = 'CARTOGRAPHER_MAP_MODE',
                    help   = 'the mode of input image, 1: from cartographer map saver.(contineous values)  0: from ros map saver.(3 values)')

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


parser.add_argument('-orig_pfile',
                    action = 'store',
                    dest   = 'orig_pose_file_str',
                    help   = 'original local pose file, .txt')

parser.add_argument('-opti_pfile',
                    action = 'store',
                    dest   = 'opti_pose_file_str',
                    help   = 'optimized global pose file, .txt')

parser.add_argument('-fina_pfile',
                    action = 'store',
                    dest   = 'fina_pose_file_str',
                    help   = 'final global pose file, .txt')


parser.add_argument('-traj_shape_mode',
                    action = 'store',
                    dest   = 'TRAJ_SHAPE_MODE',
                    help   = 'the mode of trajectory shape')

args = parser.parse_args()


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# constants parameters
#===========================================================================================
resolution       = 0.05  # resolution of grid
sonar_board_size = 1600 # sonar board size.
midle_intensity  = 127 # grey intensity value for unknow space, corresponding to 0.5
#midle_intensity  = 220
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


if np.int32(args.CARTOGRAPHER_MAP_MODE): # 0.60, 0.45
    lidar_map_1 = (lidar_map_pre >= (255 * 0.60)) * 255
    lidar_map_2 = ((lidar_map_pre > (255 * 0.45)) & (lidar_map_pre < (255 * 0.60))) * midle_intensity 
    lidar_map_3 = (lidar_map_pre <= (255 * 0.45)) * 0
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

#cv2.imshow('lidar_map_raw', lidar_map_raw)

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
# read lidar trajectory .txt files
#===========================================================================================
def ReadTrajectory(pose_file_str, origin_x, origin_y, resolution):
    pose_file = args.in_path + args.lidar_file_str + '_' + pose_file_str + '.txt'
    
    # self-defined dialect for .csv reading.
    csv.register_dialect('pi_dialect',
    delimiter = ' ',
    skipinitialspace=True)
    
    with open(pose_file, 'r') as f1:
        pose_reader = csv.reader(f1, dialect = 'pi_dialect')
        pose_data = np.array([line for line in pose_reader]).astype(np.float32)
    
    trajpoints = pose_data[:,3:5]
    trajpoints[:, 0] = np.floor(trajpoints[:, 0] / resolution) + origin_x
    trajpoints[:, 1] = -np.ceil(trajpoints[:, 1] / resolution) + origin_y
    return trajpoints


origpo_trajpoints = ReadTrajectory(args.orig_pose_file_str, origin_x, origin_y, resolution)

optipo_trajpoints = ReadTrajectory(args.opti_pose_file_str, origin_x, origin_y, resolution)

finapo_trajpoints = ReadTrajectory(args.fina_pose_file_str, origin_x, origin_y, resolution)

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
center2 = (finapo_trajpoints[-1, 0], finapo_trajpoints[-1, 1])
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


if np.int32(args.TRAJ_SHAPE_MODE):
    #pointList = np.array([[0, 0],[10, 20],[30, 31],[50, 82],[70, 19]])
    # BGR
    # lineType = 8, 4, cv2.LINE_AA
    cv2.polylines(lidar_map_mark, np.int32([origpo_trajpoints]), False, (255, 0, 0), 1); 
    
    cv2.polylines(lidar_map_mark, np.int32([optipo_trajpoints]), False, (0, 192, 0), 1); 
    
    cv2.polylines(lidar_map_mark, np.int32([finapo_trajpoints]), False, (255, 0, 255), 1); 
else:
    radius_tj = 1
    for origpo in origpo_trajpoints:
        cv2.circle(lidar_map_mark, (np.int32(origpo[0]), np.int32(origpo[1])), radius_tj, (255, 0, 0), -1)
    for optipo in optipo_trajpoints:
        cv2.circle(lidar_map_mark, (np.int32(optipo[0]), np.int32(optipo[1])), radius_tj, (0, 192, 0), -1)
    for finapo in finapo_trajpoints:
        cv2.circle(lidar_map_mark, (np.int32(finapo[0]), np.int32(finapo[1])), radius_tj, (255, 0, 255), -1)




cv2.imshow('lidar_map_traj',lidar_map_mark)
lidar_map_traj_file = args.out_path + args.lidar_file_str + '_lidar_multi_trajs.png'
cv2.imwrite(lidar_map_traj_file, lidar_map_mark)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# For display
#===========================================================================================
cv2.waitKey(0)
cv2.destroyAllWindows()

