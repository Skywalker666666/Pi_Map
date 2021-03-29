"""
============================================================================================
PerceptIn
============================================================================================
Lidar Map processing
Sonar Map processing

Lidar + Sonar Map fusion based on trajectory

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
import time
import matplotlib.pyplot as plt

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# input arguments
#===========================================================================================

parser = argparse.ArgumentParser()

#parser.add_argument('-th',
#                    action = 'store',
#                    dest   = 'threshold_sonar',
#                    help   = 'threshold for sonar filter, quantification for OccupancyGrid. Required: > 0.5')
#
#
#parser.add_argument('-in_map_mode',
#                    action = 'store',
#                    dest   = 'CARTOGRAPHER_MAP_MODE',
#                    help   = 'the mode of input image, 1: from cartographer map saver. 0: from ros map saver')

parser.add_argument('-inpath',
                    action = 'store',
                    dest   = 'in_path',
                    help   = 'path name of input files')

parser.add_argument('-inpath2',
                    action = 'store',
                    dest   = 'in_path2',
                    help   = 'path name 2 of input files')

parser.add_argument('-outpath',
                    action = 'store',
                    dest   = 'out_path',
                    help   = 'path name of output files')

parser.add_argument('-lfile',
                    action = 'store',
                    dest   = 'lidar_file_str',
                    help   = 'file name of lidar map, .png')


parser.add_argument('-sfile',
                    action = 'store',
                    dest   = 'sonar_file_str',
                    help   = 'file name of sonar map, .png')

parser.add_argument('-pfile',
                    action = 'store',
                    dest   = 'lidar_pose_str',
                    help   = 'file name of lidar pose, .txt')

parser.add_argument('-yfile',
                    action = 'store',
                    dest   = 'startps_file_str',
                    help   = 'file name of starting point, .yaml')

parser.add_argument('-mfile',
                    action = 'store',
                    dest   = 'map_para_file_str',
                    help   = 'file name of map global parameters, .yaml')


parser.add_argument('-traj_mode',
                    action = 'store',
                    dest   = 'TRAJ_MODE',
                    help   = 'trajectory mode, .txt')

args = parser.parse_args()


def convDis2Pix(xy_m, origin_xy, resolution):
    xy_pix    = xy_m
    xy_pix[0] =  np.floor(xy_m[0] / resolution) + origin_xy[0]
    xy_pix[1] = -np.floor(xy_m[1] / resolution) + origin_xy[1] #changed to floor, 05-23-2019
    return xy_pix


def BLineFloat1(xSt, ySt, xEd, yEd):
    """
    Bresenham's line algorithm, float number
    Bitmap Style: https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm 
    
    xSt: Start Point x coordinate value
    ySt: Start Point y coordinate value
    xEd: End Point x coordinate value
    yEd: End Point y coordinate value
    """
    pixLine = []
    deltax = abs(xEd - xSt)
    deltay = abs(yEd - ySt)
    
    x, y = xSt, ySt

    if xSt > xEd:
        stepx = -1
    else:
        stepx = 1 

    if ySt > yEd:
        stepy = -1
    else:
        stepy = 1 

    if deltay < deltax: # Low octant
        err = deltax / 2.0
        while x != xEd:
            pixLine.append([x, y])
            err = err - deltay
            if err < 0:
                y = y + stepy
                err = err + deltax
            x = x + stepx
    else:                  # High octant
        err = deltay / 2.0
        while y != yEd:
            pixLine.append([x, y])
            err = err - deltax
            if err < 0:
                x = x + stepx
                err = err + deltay
            y = y + stepy
    pixLine.append([x, y])  # add End point
    return pixLine


def BLineFloat2(xSt, ySt, xEd, yEd):
    """
    Bresenham's line algorithm, float number
    Wikipedia Style
    
    xSt: Start Point x coordinate value
    ySt: Start Point y coordinate value
    xEd: End Point x coordinate value
    yEd: End Point y coordinate value
    """

    pixLine = []
    deltax = abs(xEd - xSt)
    deltay = abs(yEd - ySt)
    
    x, y = xSt, ySt

    if xSt > xEd:
        stepx = -1
    else:
        stepx = 1 

    if ySt > yEd:
        stepy = -1
    else:
        stepy = 1 

    if deltay < deltax: # Low octant
        err = -deltax / 2.0
        while x != xEd:
            pixLine.append([x, y])
            err = err + deltay
            if err > 0:
                y = y + stepy
                err = err - deltax
            x = x + stepx
    else:                  # High octant
        err = - deltay / 2.0
        while y != yEd:
            pixLine.append([x, y])
            err = err + deltax
            if err > 0:
                x = x + stepx
                err = err - deltay
            y = y + stepy
    pixLine.append([x, y])  # add End point
    return pixLine

def BLineFloat2Cut(xSt, ySt, xEd, yEd, map_raw):
    """
    Bresenham's line algorithm, float number
    Wikipedia Style
    
    xSt: Start Point x coordinate value
    ySt: Start Point y coordinate value
    xEd: End Point x coordinate value
    yEd: End Point y coordinate value
    """
    Height, Width = map_raw.shape[:2]

    pixLine = []
    deltax = abs(xEd - xSt)
    deltay = abs(yEd - ySt)
    
    x, y = xSt, ySt

    if xSt > xEd:
        stepx = -1
    else:
        stepx = 1 

    if ySt > yEd:
        stepy = -1
    else:
        stepy = 1 

    if deltay < deltax: # Low octant
        err = -deltax / 2.0
        while x != xEd:
            if map_raw[np.int32(y), np.int32(x)] == 0 or y == 0 or y == (Height - 1) or x == 0 or x == (Width - 1):
                #print "touch the wall!!!"
                break
            pixLine.append([x, y])
            err = err + deltay
            if err > 0:
                y = y + stepy
                err = err - deltax
            x = x + stepx
    else:                  # High octant
        err = - deltay / 2.0
        while y != yEd:
            if map_raw[np.int32(y), np.int32(x)] == 0 or y == 0 or y == (Height - 1) or x == 0 or x == (Width - 1):
                #print "touch the wall!!!"
                break
            pixLine.append([x, y])
            err = err + deltax
            if err > 0:
                x = x + stepx
                err = err - deltay
            y = y + stepy
    pixLine.append([x, y])  # add End point
    return pixLine



def BLineInt(xSt_pre, ySt_pre, xEd_pre, yEd_pre):
    """
    Bresenham's line algorithm, int number
    
    xSt: Start Point x coordinate value
    ySt: Start Point y coordinate value
    xEd: End Point x coordinate value
    yEd: End Point y coordinate value
    """
    xSt = np.int32(xSt_pre)
    ySt = np.int32(ySt_pre)
    xEd = np.int32(xEd_pre)
    yEd = np.int32(yEd_pre)

    pixLine = []
    deltax = abs(xEd - xSt)
    deltay = abs(yEd - ySt)
    
    x, y = xSt, ySt

    if xSt > xEd:
        stepx = -1
    else:
        stepx = 1 

    if ySt > yEd:
        stepy = -1
    else:
        stepy = 1 

    if deltay < deltax : # Low octant
        err = -deltax
        while x != xEd:
            pixLine.append([x, y])
            err = err + 2 * deltay 
            if err > 0:
                y = y + stepy
                err = err - 2 * deltax
            x = x + stepx
    else:                  # High octant
        err = -deltay
        while y != yEd:
            pixLine.append([x, y])
            err = err + 2 * deltax
            if err > 0:
                x = x + stepx
                err = err - 2 * deltay
            y = y + stepy
    pixLine.append([x, y])  # add End point
    return pixLine

def BLineIntCut(xSt_pre, ySt_pre, xEd_pre, yEd_pre, map_raw):
    """
    Bresenham's line algorithm, int number
    Add (1) object boundary detect (2) image boundary check
    
    xSt: Start Point x coordinate value
    ySt: Start Point y coordinate value
    xEd: End Point x coordinate value
    yEd: End Point y coordinate value
    """
    xSt = np.int32(xSt_pre)
    ySt = np.int32(ySt_pre)
    xEd = np.int32(xEd_pre)
    yEd = np.int32(yEd_pre)

    Height, Width = map_raw.shape[:2]

    pixLine = []
    deltax = abs(xEd - xSt)
    deltay = abs(yEd - ySt)
    
    x, y = xSt, ySt

    if xSt > xEd:
        stepx = -1
    else:
        stepx = 1 

    if ySt > yEd:
        stepy = -1
    else:
        stepy = 1 

    if deltay < deltax : # Low octant
        err = -deltax
        while x != xEd:
            if y <= 0 or y >= (Height - 1) or x <= 0 or x >= (Width - 1):
                #print "touch the wall!!!"
                break
            if map_raw[y, x] == 0:
                break
            pixLine.append([x, y])
            err = err + 2 * deltay 
            if err > 0:
                y = y + stepy
                err = err - 2 * deltax
            x = x + stepx
    else:                # High octant
        err = -deltay
        while y != yEd:
            if y <= 0 or y >= (Height - 1) or x <= 0 or x >= (Width - 1):
                #print "touch the wall!!!"
                break
            if map_raw[y, x] == 0:
                break
            pixLine.append([x, y])
            err = err + 2 * deltax
            if err > 0:
                x = x + stepx
                err = err - 2 * deltay
            y = y + stepy
    pixLine.append([x, y])  # add End point
    # print "shape of one point in BLine: ", np.shape([x, y])
    # [[], []]
    # (2, 1)
    # 1st dimension is 2, 2nd dimension is 1
    return pixLine


def BLineIntCutExt(xSt_pre, ySt_pre, xEd_pre, yEd_pre, map_raw):
    """
    Bresenham's line algorithm, int number
    Add (1) object boundary detect (2) image boundary check

    Add extension function. (Temporary method.) deal with double lines problem.
    
    xSt: Start Point x coordinate value
    ySt: Start Point y coordinate value
    xEd: End Point x coordinate value
    yEd: End Point y coordinate value
    """
    xSt = np.int32(xSt_pre)
    ySt = np.int32(ySt_pre)
    xEd = np.int32(xEd_pre)
    yEd = np.int32(yEd_pre)

    Height, Width = map_raw.shape[:2]

    pixLine = []
    pixLineExt = []

    deltax = abs(xEd - xSt)
    deltay = abs(yEd - ySt)
    
    x, y = xSt, ySt

    if xSt > xEd:
        stepx = -1
    else:
        stepx = 1 

    if ySt > yEd:
        stepy = -1
    else:
        stepy = 1 

    if deltay < deltax : # Low octant
        err = -deltax
        while x != xEd:
            if y <= 0 or y >= (Height - 1) or x <= 0 or x >= (Width - 1):
                #print "touch the wall!!!"
                break
            if map_raw[y, x] == 0:
                break
            pixLine.append([x, y])
            err = err + 2 * deltay 
            if err > 0:
                y = y + stepy
                err = err - 2 * deltax
            x = x + stepx
        pixLine.append([x, y])  # add End point
        # extension
        for q in range(3):
            if not(y <= 0 or y >= (Height - 1) or x <= 0 or x >= (Width - 1)):
                err = err + 2 * deltay 
                if err > 0:
                    y = y + stepy
                    err = err - 2 * deltax
                x = x + stepx
        pixLineExt.append([x, y])  # add End point


    else:                # High octant
        err = -deltay
        while y != yEd:
            if y <= 0 or y >= (Height - 1) or x <= 0 or x >= (Width - 1):
                #print "touch the wall!!!"
                break
            if map_raw[y, x] == 0:
                break
            pixLine.append([x, y])
            err = err + 2 * deltax
            if err > 0:
                x = x + stepx
                err = err - 2 * deltay
            y = y + stepy
        pixLine.append([x, y])  # add End point
        # extension
        for q in range(3):
            if not(y <= 0 or y >= (Height - 1) or x <= 0 or x >= (Width - 1)):
                err = err + 2 * deltax
                if err > 0:
                    x = x + stepx
                    err = err - 2 * deltay
                y = y + stepy
        pixLineExt.append([x, y])  # add End point
    # print "shape of one point in BLine: ", np.shape([x, y])
    # [[], []]
    # (2, 1)
    # 1st dimension is 2, 2nd dimension is 1
    return pixLine, pixLineExt





def FusionAlgorithm(ssid_vec, Diff_range_pix_vec, Lidar_pose_pix_vec, Lidar_obstacle_pix_vec, Sonar_pose_pix_vec, Sonar_obstacle_pix_vec, fusion_map_pre):
    Height, Width = fusion_map_pre.shape[:2]
    a = 0
    for ssid in ssid_vec:
        a = a + 1
        print a
        for k in range(len(Diff_range_pix_vec[ssid])):
            diff_rg_pix = Diff_range_pix_vec[ssid][k]
            if diff_rg_pix <= ThreNormObstacle: # default 10 grids.
                # obstacles, free space
                if Lidar_obstacle_pix_vec[ssid][k][0] != 0 and Lidar_obstacle_pix_vec[ssid][k][0] != Width - 1 and Lidar_obstacle_pix_vec[ssid][k][1] != 0 and Lidar_obstacle_pix_vec[ssid][k][1] != Height - 1:
                    LineSecFillFree = BLineIntCut(Lidar_pose_pix_vec[ssid][k][0], Lidar_pose_pix_vec[ssid][k][1], Lidar_obstacle_pix_vec[ssid][k][0], Lidar_obstacle_pix_vec[ssid][k][1], fusion_map_pre)
                    for FillPt in LineSecFillFree:
                        fusion_map_pre[FillPt[1], FillPt[0]] = 255
                    fusion_map_pre[Lidar_obstacle_pix_vec[ssid][k][1], Lidar_obstacle_pix_vec[ssid][k][0]] = 0
            elif diff_rg_pix > ThreGlass:  # default 30 grids. 0.05 m per grid, 30 grids is 1.5 m
                #Glass panels, windows
                #LineSecFill = BLineIntCut(Sonar_pose_pix_vec[ssid][k][0], Sonar_pose_pix_vec[ssid][k][1], Sonar_obstacle_pix_vec[ssid][k][0], Sonar_obstacle_pix_vec[ssid][k][1], fusion_map_pre)
                # following is for double lines removal of sonar, but this is right way to do, cuz boundary check have been applied in previous stage.
                LineSecFill = BLineInt(Sonar_pose_pix_vec[ssid][k][0], Sonar_pose_pix_vec[ssid][k][1], Sonar_obstacle_pix_vec[ssid][k][0], Sonar_obstacle_pix_vec[ssid][k][1])
                for FillPt in LineSecFill:
                    # reasonable behavior
                    fusion_map_pre[FillPt[1], FillPt[0]] = 255
                    # if have to extend, use following method:
                    #if( FillPt == LineSecFill[-1] or FillPt == LineSecFill[-2] ):
                    #    fusion_map_pre[FillPt[1], FillPt[0]] = 0
                    #    #print "----------------- FillPt[-1]: "
                    #    #print FillPt
               
                #print "xxxxxxxxxxxxxxx LineSecFill[-1]: "
                #print LineSecFill[-1]
 
                fusion_map_pre[Sonar_obstacle_pix_vec[ssid][k][1], Sonar_obstacle_pix_vec[ssid][k][0]] = 0

    fusion_map = fusion_map_pre.copy()
    return fusion_map 




#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# constants parameters
#===========================================================================================
resolution       = 0.05  # resolution of grid
sonar_board_size = 1600  # sonar board size.
midle_intensity  = 127   # grey intensity value for unknow space, corresponding to 0.5

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read map global parameter .yaml file
#===========================================================================================
global_map_file = args.map_para_file_str + '.yaml'
with open(global_map_file , 'r') as f:
    gl_para = yaml.load(f)
    OurSonarNum = gl_para['OurSonarNum']
    PI_PI   = gl_para['PI_PI']
    SonarX = gl_para['sonarX']
    SonarY = gl_para['sonarY']
    SonarTh = gl_para['sonarTh'] # sonar installation angle, degree
    ScanLen = gl_para['ScanLen'] # length of 2nd scan, in meters
    ThreGlass = gl_para['ThreGlass'] # Threshold for glass, if L-S gap is larger than this, assume glass. in grids
    ThreNormObstacle = gl_para['ThreNormObstacle'] # Threshold for normal obstacles, if L-S gap is smaller than this, normal obstacle. in grids

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read lidar .pgm source file and display
#===========================================================================================
lidar_file    = args.in_path + args.lidar_file_str + '.png'
lidar_map_raw = cv2.imread(lidar_file)  
lidar_map_gray = cv2.cvtColor(lidar_map_raw, cv2.COLOR_BGR2GRAY)
Height, Width = lidar_map_raw.shape[:2]
print 'Height:' + str(Height)
print 'Width:'  + str(Width)
cv2.imshow('lidar_map_raw', lidar_map_raw)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read sonar .txt source file and display
#===========================================================================================
sonar_file    = args.in_path + args.sonar_file_str + '.png'
sonar_map_raw = cv2.imread(sonar_file)  
sonar_map_gray = cv2.cvtColor(sonar_map_raw, cv2.COLOR_BGR2GRAY)
Height, Width = sonar_map_raw.shape[:2]
print 'Height:' + str(Height)
print 'Width:'  + str(Width)
cv2.imshow('sonar_map_raw', sonar_map_raw)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read lidar starting position .yaml file
#===========================================================================================
starting_pos_file = args.in_path2 + args.startps_file_str + '.yaml'
with open(starting_pos_file, 'r') as f:
    doc = yaml.load(f)

origin_xyz = doc['origin']
#print origin_xyz[1]

# starting point compared to lefttop, x:> y:^
# origin_x and origin_y is absolute value
origin_x = np.floor(-(origin_xyz[0] / resolution))
origin_y = np.floor(Height - (-(origin_xyz[1] / resolution)))
# starting point (origin_x, origin_y) lefttop, x:> y:v

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read lidar trajectory .txt file
#===========================================================================================
lidar_pose_file = args.in_path2 + args.lidar_pose_str + '.txt'

csv.register_dialect('pi_dialect',
delimiter = ' ',
skipinitialspace=True)

with open(lidar_pose_file, 'r') as f1:
    lipose_reader = csv.reader(f1, dialect = 'pi_dialect')
    lipose_data = np.array([line for line in lipose_reader]).astype(np.float32)

# method 1:
#num_positions, _ = lipose_data.shape[:2]  
# it is equal to lipose_data.shape[0], lipose_data.shape[1]
# method 2:
num_positions, _ = lipose_data.shape

if np.int32(args.TRAJ_MODE) == 0:
    # pose trajectory is from what we received.
    lipose_trajpoints_m = lipose_data[:,1:4]
elif np.int32(args.TRAJ_MODE) == 1:
    # pose trajectory is from nodes list
    lipose_trajpoints_m = lipose_data[:,3:6]
elif np.int32(args.TRAJ_MODE) == 2:
    # pose trajectory is from final nodes interpolation
    lipose_trajpoints_m = lipose_data[:,2:5]
else:
    print "TRAJ_MODE is wrong."

lipose_trajpoints_pix = np.zeros((num_positions, 2), dtype = np.float32)
# convert meter value in world-coordinate to pix index in cvplot-coordinate
lipose_trajpoints_pix[:, 0] =  np.floor(lipose_trajpoints_m[:, 0] / resolution) + origin_x
lipose_trajpoints_pix[:, 1] = -np.floor(lipose_trajpoints_m[:, 1] / resolution) + origin_y #changed to floor, 05-23-2019

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# insert starting point and trajectory for lidar raw map
#===========================================================================================
# lidar_map_mark = lidar_map_raw

lidar_map_mark = lidar_map_raw.copy()
# cv2, use .copy() method will create a copy of the array.
# otherwise it will produce only a view of the object.

# row: the index within one row, horizontal direction.
row = origin_x.astype(np.uint32) #fixed type bug, 0520
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

center = (row, col)
radius = 2
# BGR
# green
color1  = (255, 255, 0) 
# filled circle
thickness1 = -1 
# /////////////////////TO DO, a small bug here, center is not right in certain circumtances
cv2.circle(lidar_map_mark, center, radius, color1, thickness1)
cv2.imshow('lidar_map_mark',lidar_map_mark)

######+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
###### traverse all points on the trajectory
######===========================================================================================
lidar_map_mark2 = lidar_map_mark.copy()
sonar_map_mark2 = sonar_map_raw.copy()

start_time = time.clock()
last_x_m = 0
last_y_m = 0
last_th  = 0
pose_cnt = 0
real_pose_cnt = 0

Lidar_range_pix_vec = [[] for i in range(OurSonarNum)]
Sonar_range_pix_vec = [[] for i in range(OurSonarNum)]
Diff_range_pix_vec  = [[] for i in range(OurSonarNum)]

Lidar_pose_pix_vec = [[] for i in range(OurSonarNum)]
Sonar_pose_pix_vec = [[] for i in range(OurSonarNum)]
Lidar_obstacle_pix_vec = [[] for i in range(OurSonarNum)]
Sonar_obstacle_pix_vec = [[] for i in range(OurSonarNum)]

# control the sampling ratio
#for ps in range(num_positions):
for ps in range(0, num_positions, 1):
#for ps in range(0, num_positions, 100):
#for ps in range(0, 1000, 300):
    #ps = 50
    pose_cnt += 1
    x_m = lipose_trajpoints_m[ps, 0]
    y_m = lipose_trajpoints_m[ps, 1]
    th  = lipose_trajpoints_m[ps, 2]   # yaw angle, rad
    #traj_pix = convDis2Pix([x_m, y_m], [origin_x, origin_y], resolution)
    #repetition check
    if x_m == last_x_m and y_m == last_y_m and th == last_th:
        continue
    
    last_x_m = x_m 
    last_y_m = y_m 
    last_th  = th  
    real_pose_cnt += 1

    #for SonId in range(OurSonarNum):
    for SonId in [0, 7]:
        #SonId = 1
        
        #startPtMapX = SonarX[SonId] * np.cos(th) - SonarY[SonId] * np.sin(th) + x_m
        #startPtMapY = SonarX[SonId] * np.sin(th) + SonarY[SonId] * np.cos(th) + y_m
        # Matrix multiplication 
        SonarSensor = np.array([[SonarX[SonId]], [SonarY[SonId]], [1]])
        TransMat    = np.array([[np.cos(th), -np.sin(th), x_m],
                                [np.sin(th),  np.cos(th), y_m]])
        # 2 x 3 matmul 3 x 1 -> 2 x 1
        StartPt_m  = np.matmul(TransMat, SonarSensor)  # Rotate + translate
        CurHeading = th + (SonarTh[SonId] * PI_PI / 180)
        ScanOffset = [[ScanLen * np.cos(CurHeading)], [ScanLen * np.sin(CurHeading)]]
        EndPt_m    = np.add(StartPt_m, ScanOffset)
        
        StartPt_pix = convDis2Pix(StartPt_m, [origin_x, origin_y], resolution)
        EndPt_pix   = convDis2Pix(EndPt_m  , [origin_x, origin_y], resolution)
        
        #LineSec = BLineInt(StartPt_pix[0], StartPt_pix[1], EndPt_pix[0], EndPt_pix[1])
        
        # BGR
        LineSecL = BLineIntCut(StartPt_pix[0], StartPt_pix[1], EndPt_pix[0], EndPt_pix[1], lidar_map_gray)
        LineSecS = BLineIntCut(StartPt_pix[0], StartPt_pix[1], EndPt_pix[0], EndPt_pix[1], sonar_map_gray)

        # following is for double lines removal of sonar
        # LineSecL = BLineIntCut(StartPt_pix[0], StartPt_pix[1], EndPt_pix[0], EndPt_pix[1], lidar_map_gray)
        # LineSecS, LineSecSExt = BLineIntCutExt(StartPt_pix[0], StartPt_pix[1], EndPt_pix[0], EndPt_pix[1], sonar_map_gray)

        x_pix = lipose_trajpoints_pix[ps, 0] # pix index in cvplot-coordinate
        y_pix = lipose_trajpoints_pix[ps, 1] # pix index in cvplot-coordinate
        Lidar_pose_pix_vec[SonId].append([x_pix, y_pix])
        Sonar_pose_pix_vec[SonId].append([x_pix, y_pix])
        # if use following method, there will be empty (unknow) area for chassis
        #Lidar_pose_pix_vec[SonId].append(LineSecL[0])
        #Sonar_pose_pix_vec[SonId].append(LineSecS[0])
 
        Lidar_obstacle_pix_vec[SonId].append(LineSecL[-1])
        Sonar_obstacle_pix_vec[SonId].append(LineSecS[-1])
        
        # following is for double lines removal of sonar
        #Sonar_obstacle_pix_vec[SonId].append(LineSecSExt[-1])

        Lidar_range_pix_vec[SonId].append(len(LineSecL))
        Sonar_range_pix_vec[SonId].append(len(LineSecS))

        cv2.polylines(lidar_map_mark, np.int32([LineSecL]), False, (0, 255, 0), 1); 
        cv2.polylines(sonar_map_raw, np.int32([LineSecS]), False, (0, 0, 255), 1); 
        

######+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
###### Plot and show results:
######===========================================================================================

#---------------------------------------------------------------
# Time and counter, show
#---------------------------------------------------------------
print time.clock() - start_time, "seconds"
print "pose counter: ", pose_cnt
print "real pose counter: ", real_pose_cnt
cv2.imshow('lidar_map_2nd_scan', lidar_map_mark)
cv2.imshow('sonar_map_2nd_scan', sonar_map_raw)

lidar_scan_map_file = args.out_path + args.lidar_file_str + '_LIDAR_SCAN.png'
cv2.imwrite(lidar_scan_map_file, lidar_map_mark)

sonar_scan_map_file = args.out_path + args.sonar_file_str + '_SONAR_SCAN.png'
cv2.imwrite(sonar_scan_map_file, sonar_map_raw)

#---------------------------------------------------------------
# show detected obstacles points.
#---------------------------------------------------------------
# BGR
color2  = (0, 255, 0) 
color3  = (0, 0, 255) 
thickness2 = 1
for ObstaclePt in Lidar_obstacle_pix_vec[0]:
    cv2.circle(lidar_map_mark2, tuple(ObstaclePt), radius, color2, thickness2)
for ObstaclePt in Lidar_obstacle_pix_vec[7]:
    cv2.circle(lidar_map_mark2, tuple(ObstaclePt), radius, color2, thickness2)
#cv2.polylines(lidar_map_mark2, np.int32([Lidar_obstacle_pix_vec[0]]), False, (0, 255, 0), 1); 
cv2.imshow('lidar_map_2nd_scan obstacles point', lidar_map_mark2)

lidar_obstacle_map_file = args.out_path + args.lidar_file_str + '_LIDAR_OBSTAC.png'
cv2.imwrite(lidar_obstacle_map_file, lidar_map_mark2)


for ObstaclePt in Sonar_obstacle_pix_vec[0]:
    cv2.circle(sonar_map_mark2, tuple(ObstaclePt), radius, color3, thickness2)
for ObstaclePt in Sonar_obstacle_pix_vec[7]:
    cv2.circle(sonar_map_mark2, tuple(ObstaclePt), radius, color3, thickness2)
#cv2.polylines(lidar_map_mark2, np.int32([Lidar_obstacle_pix_vec[0]]), False, (0, 255, 0), 1); 
cv2.imshow('sonar_map_2nd_scan obstacles point', sonar_map_mark2)

sonar_obstacle_map_file = args.out_path + args.sonar_file_str + '_SONAR_OBSTAC.png'
cv2.imwrite(sonar_obstacle_map_file, sonar_map_mark2)

#---------------------------------------------------------------
# show range in pixels
#---------------------------------------------------------------
Diff_range_pix_vec[0] = np.array(Lidar_range_pix_vec[0]) - np.array(Sonar_range_pix_vec[0])
Diff_range_pix_vec[7] = np.array(Lidar_range_pix_vec[7]) - np.array(Sonar_range_pix_vec[7])
plt_H_id = np.arange(len(Sonar_range_pix_vec[7]))

plt.figure(1)
plt.subplot(311)
plt.plot(plt_H_id, Lidar_range_pix_vec[0], color='g', lw=1.0, ls='-', mec = 'g', mfc = 'none', marker = 'o')
plt.xlabel('Pose Id')
plt.ylabel('Range (Grid)')
plt.title('Range to obstacles from Lidar')
plt.subplot(312)
plt.plot(plt_H_id, Sonar_range_pix_vec[0], color='r', lw=1.0, ls='-', mec = 'r', mfc = 'none', marker = 'o')
plt.xlabel('Pose Id')
plt.ylabel('Range (Grid)')
plt.title('Range to obstacles from Sonar')
plt.subplot(313)
plt.plot(plt_H_id, Diff_range_pix_vec[0],  color='k', lw=1.0, ls='-', mec = 'k', mfc = 'none', marker = 'o')
plt.xlabel('Pose Id')
plt.ylabel('Range (Grid)')
plt.title('Liar Range subtracts Sonar Range: direction 0 ')
plt.subplots_adjust(hspace=0.30, wspace=0.35)
plt.show()

plt.figure(2)
plt.subplot(311)
plt.plot(plt_H_id, Lidar_range_pix_vec[7], color='g', lw=1.0, ls='-', mec = 'g', mfc = 'none', marker = 'o')
plt.xlabel('Pose Id')
plt.ylabel('Range (Grid)')
plt.title('Range to obstacles from Lidar')
plt.subplot(312)
plt.plot(plt_H_id, Sonar_range_pix_vec[7], color='r', lw=1.0, ls='-', mec = 'r', mfc = 'none', marker = 'o')
plt.xlabel('Pose Id')
plt.ylabel('Range (Grid)')
plt.title('Range to obstacles from Sonar')
plt.subplot(313)
plt.plot(plt_H_id, Diff_range_pix_vec[7],  color='k', lw=1.0, ls='-', mec = 'k', mfc = 'none', marker = 'o')
plt.xlabel('Pose Id')
plt.ylabel('Range (Grid)')
plt.title('Liar Range subtracts Sonar Range: direction 7 ')
plt.subplots_adjust(hspace=0.30, wspace=0.35)
plt.show()
######+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
###### Fuse two maps. second hand scan
######===========================================================================================
fusion_map_pre = lidar_map_gray.copy()
ssid_vec = [0 , 7]
# call fusion function


fusion_map = FusionAlgorithm(ssid_vec, Diff_range_pix_vec, Lidar_pose_pix_vec, Lidar_obstacle_pix_vec, Sonar_pose_pix_vec, Sonar_obstacle_pix_vec, fusion_map_pre)
        
cv2.imshow('Fusion_map_2nd_scan', fusion_map)
fusion_map_file = args.out_path + args.lidar_file_str + args.sonar_file_str + '_2nd_FUSION.png'
cv2.imwrite(fusion_map_file, fusion_map)




######+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
###### crop sonar block, m x n block from 1600 x 1600
######===========================================================================================
#####m_start = sonar_board_size/2 - origin_y
#####m_end   = m_start + Height
#####n_start = sonar_board_size/2 - origin_x
#####n_end   = n_start + Width
#####
#####sonar_map_block = sonar_map_raw_u8[np.int32(m_start) : np.int32(m_end), np.int32(n_start) : np.int32(n_end)]
#####cv2.imshow('sonar_map_block', sonar_map_block)
######print sonar_map_block.shape
#####sonar_map_block_file = args.out_path + args.lidar_file_str + '_sonar_block.png'
#####cv2.imwrite(sonar_map_block_file, sonar_map_block)
#####
######+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
###### Filter Sonar map
######===========================================================================================
###### because sonar_map already revert 0 and 1 for display in the above, so here threshold also need revert.
#####threshold_sonar_oppo = 1 - np.float32(args.threshold_sonar)
#####blk_1 = (sonar_map_block > midle_intensity) * 255
#####blk_2 = (sonar_map_block == midle_intensity) * midle_intensity  
#####blk_3 = np.uint8(np.logical_and((sonar_map_block > (255 * threshold_sonar_oppo)), (sonar_map_block < midle_intensity))) * 255
#####blk_4 = (sonar_map_block <= (255 * threshold_sonar_oppo)) * 0
#####
#####sonar_map_blk_filtered = np.uint8(blk_1) + np.uint8(blk_2) + np.uint8(blk_3) + np.uint8(blk_4)
#####cv2.imshow('sonar_map_block_filtered', sonar_map_blk_filtered)
#####sonar_map_filter_file = args.out_path + args.lidar_file_str + '_sonar_filter.png'
#####cv2.imwrite(sonar_map_filter_file, sonar_map_blk_filtered)
#####
######+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
###### Fusion
######===========================================================================================
#####
#####fusion_lid_son = np.uint8(sonar_map_blk_filtered != 0) * lidar_map_grey
#####cv2.imshow('lidar sonar fusion map', fusion_lid_son)
#####fusion_map_file = args.out_path + args.lidar_file_str + '_fusion.png'
#####cv2.imwrite(fusion_map_file, fusion_lid_son)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# For display
#===========================================================================================
cv2.waitKey(0)
cv2.destroyAllWindows()
