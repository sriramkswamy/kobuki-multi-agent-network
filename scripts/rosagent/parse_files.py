"""
This module contains the functions for post processing for reading files
"""

import os
import sys
import inspect
import csv
import json
from math import cos, sin, acos, asin, atan, pi
import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint

from transform_variables import *
# import rospy


def csv_timestamps(filename):
    """extracts the timestamps from the csv file"""
    time_stamps = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            secs = int(row[0])
            nsecs = int(row[1])
            # time_stamps.append(rospy.Time(secs, nsecs))
    return time_stamps


def csv_orientation(filename):
    """extracts all the quaternions from the csv file"""
    phi = []
    theta = []
    psi = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            phi.append(float(row[3]))
            theta.append(float(row[4]))
            psi.append(float(row[5]))
    return psi


def csv_quaternions(filename):
    """extracts all the quaternions from the csv file"""
    w_values = []
    x_values = []
    y_values = []
    z_values = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            w = float(row[0])
            x = float(row[1])
            y = float(row[2])
            z = float(row[3])
            w_values.append(w)
            x_values.append(x)
            y_values.append(y)
            z_values.append(z)
    return w_values, x_values, y_values, z_values


def csv_matrix(filename):
    """extracts all the matrices from the csv file"""
    mat = []
    mat_count = 0
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        row_count = 0
        for row in odom_data:
            if len(row) > 0:
                row = [float(row[0]), float(row[1]), float(row[2])]
                if row_count == 0:
                    csv_mat = np.matrix([row])
                else:
                    csv_mat = np.vstack([csv_mat, row])
                row_count += 1
            else:
                row_count = 0
                mat.append(csv_mat)
    return mat


def csv_3d_cartesian(filename, polar=False):
    """extract 2d cartesian coordinates from a file"""
    x_values = []
    y_values = []
    z_values = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            if polar:
                x = float(row[0]) * cos(float(row[1])) * sin(float(row[2]))
                y = float(row[0]) * cos(float(row[1])) * cos(float(row[2]))
                z = float(row[0]) * sin(float(row[2]))
            else:
                x = float(row[0])
                y = float(row[1])
                z = float(row[2])
            x_values.append(x)
            y_values.append(y)
            z_values.append(y)
    return x_values, y_values, z_values


def csv_2d_cartesian(filename, polar=False, scan=False):
    """extract 2d cartesian coordinates from a file"""
    x_values = []
    y_values = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            # if scan:
            #     row[1] = pi/2 + float(row[1])
            if polar:
                x = float(row[0]) * cos(float(row[1]))
                y = float(row[0]) * sin(float(row[1]))
            else:
                x = float(row[0])
                y = float(row[1])
            x_values.append(x)
            y_values.append(y)
    return x_values, y_values


def csv_2d_polar(filename, polar=False, scan=False):
    """extract 2d polar coordinates from a file"""
    r_values = []
    t_values = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            # if scan:
            #     row[1] = pi/2 + float(row[1])
            if polar:
                r = float(row[0])
                t = float(row[1])
            else:
                r = np.sqrt(float(row[0]) + float(row[1]))
                t = np.arctan2(float(row[1]), float(row[0]))
            r_values.append(r)
            t_values.append(t)
    return r_values, t_values
