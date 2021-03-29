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

parser.add_argument('-gtfile',
                    action = 'store',
                    dest   = 'ground_truth_map_str',
                    help   = 'file name of ground truth map, .png')


parser.add_argument('-efile',
                    action = 'store',
                    dest   = 'evaluation_map_str',
                    help   = 'file name of evaluation map, .png')

parser.add_argument('-pfile',
                    action = 'store',
                    dest   = 'lidar_pose_str',
                    help   = 'file name of lidar pose, .txt')

parser.add_argument('-sonar_src_mode',
                    action = 'store',
                    dest   = 'SONAR_MAP_SRC_MODE',
                    help   = 'the mode of input image, 1: from online map. 0: from offline map')


args = parser.parse_args()


def mse(imageA, imageB):
    # 'Mean Squared Error' between the two images is the
    # sum of the squared difference between the two images
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1])
    return err

def structure_simi(imageA, imageB):
    # Structural Similarity Measure, SSIM attempts to model the 
    # perceived change in the structural information of the image
   
    # for Ubuntu 
    s_value = ssim(imageA, imageB)
    # for Mac osx
    #s_value = measure.compare_ssim(imageA, imageB)
    return s_value


def ncc(imageA, imageB):
    # Normalized cross-correlation
    # this one is a little tricky
    gt_bar = imageA.mean()
    ev_bar = imageB.mean()
    gt_std = imageA.std()
    ev_std = imageB.std()

    product = np.mean((imageA - gt_bar) * (imageB - ev_bar))
    stds = gt_std * ev_std
    if stds == 0:
        print "ERROR with std."
        return 0
    else:
        ncc_value = product / stds
        return ncc_value



#Compute the peak signal to noise ratio (PSNR) for an image.




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

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# read evaluation .png 
#===========================================================================================
print 'Evaluation Map: ' + args.evaluation_map_str  + '.png'
e_file = args.in_path2 + args.evaluation_map_str + '.png'
e_map_raw = cv2.imread(e_file)  

Height, Width = e_map_raw.shape[:2]
print 'Height: ' + str(Height)
print 'Width: ' + str(Width)

# color to grey
e_map_grey = cv2.cvtColor(e_map_raw, cv2.COLOR_BGR2GRAY)
cv2.imshow('e_map_raw', e_map_grey)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# evaluation metrics
#===========================================================================================

# MSE, the lower the error value, the more "similar" the two images are
MSE_error = mse(gt_map_grey, e_map_grey)
print 'MSE value: '
print MSE_error


# SSIM, the closer to 1.0 the similarity value is, the more "similar" the two images are
s_sim = structure_simi(gt_map_grey, e_map_grey)
print 'SSIM value: '
print s_sim 

# NCC, the lower the 
# normalized cross correlation, or Baron's cross-correlation coefficient
NCC_value = ncc(gt_map_grey, e_map_grey)
print 'NCC value: '
print NCC_value

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# For display
#===========================================================================================
cv2.waitKey(0)
cv2.destroyAllWindows()










