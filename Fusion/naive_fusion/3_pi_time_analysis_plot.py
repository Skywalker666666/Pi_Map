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

import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()

parser.add_argument('-filename',
                    action = 'store',
                    dest   = 'filename',
                    help   = '')
args = parser.parse_args()

filename = args.filename
X=[]
with open(filename, 'r') as f:
    lines = f.readlines()
    for line in lines:
        value = [float(s) for s in line.split()]
        X.append(value[0])

plt.plot(X)
plt.show()




