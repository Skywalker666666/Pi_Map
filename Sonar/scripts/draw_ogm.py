#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import collections as mc
import numpy as np
import sys
import os
import subprocess
import math

COLORS = ['-b', '-r', '-g', '-m', '-c']
RAD2DEG = 180 / math.pi

'''
    print("mm: " + m(0, 1))
'b' : (0.0, 0.0, 1.0)
'c' : (0.0, 0.75, 0.75)
'm' : (0.75, 0, 0.75)
'y' : (0.75, 0.75, 0)
'k' : (0.0, 0.0, 0.0)
    print("aa: " + sonar_maps(0, 1))
    for i in range(0, 1599):
        for j in range(0, 1599):
            #sonar_maps(i, j) = m(i * 1600 + j, 2)
'''

def load_ogm_file(filename):
    m = np.loadtxt(filename, delimiter=",")

    ogm_x = [vec[0] for vec in m]
    ogm_y = [vec[1] for vec in m]
    ogm_val = [vec[2] for vec in m]
    return m, ogm_x, ogm_y, ogm_val 

def main(name):
    mpl.rcParams['legend.fontsize'] = 10

    m, ogm_x, ogm_y, ogm_val = load_ogm_file(name)
    min_ogm_val = min(ogm_val)
    max_ogm_val = max(ogm_val)
    ogm_val_range = max_ogm_val - min_ogm_val
    print("ogm_val: [" + str(min_ogm_val) + ", " + str(max_ogm_val) + "] ---- " + str(ogm_val_range))

    sonar_maps_origin = np.zeros((1600, 1600), dtype=np.float32)
    sonar_maps = np.zeros((1600, 1600), dtype=np.float32)

    for i in range(0, 1599):
        for j in range(0, 1599):
            index_origin = i * 1600 + j
            sonar_maps_origin[i, j] = np.uint8(math.floor(m[index_origin, 2] * 256.0))

    for i in range(0, 1599):
        for j in range(0, 1599):
            index = i * 1600 + j
            if (m[index, 2] >= 0.0 and m[index, 2] < 0.5):
                sonar_maps[i, j] = 0
            elif (m[index, 2] == 0.5):
                sonar_maps[i, j] = np.uint8(math.floor(m[index, 2] * 256.0))
            elif (m[index, 2] > 0.5 and m[index, 2] <= 0.8):
                sonar_maps[i, j] = 0
            elif (m[index, 2] > 0.8):
                sonar_maps[i, j] = 255

    #sonar_maps_img = np.float32(math.floor(np.vectorize(sonar_maps)))
    #sonar_maps_img = np.uint8(sonar_maps)

    plt.figure(1)
    plt.imshow(sonar_maps_origin, cmap='gray_r')
    plt.title("Origin")
    plt.figure(2)
    plt.imshow(sonar_maps, cmap='gray_r')
    plt.title("Flitered")
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) == 2:
        main(sys.argv[1])
    else:
        print(
            'Usage[2]:' + sys.argv[0] + ' [gps_MMDD_HHMMSS.txt] |for given file')
