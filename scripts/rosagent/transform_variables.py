"""
This module contains helper functions for transforming variables
"""

import os
import sys
import inspect
import csv
import json
from math import cos, sin, acos, asin, atan, pi, sqrt
import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint


def metres_to_inches(measure_metres, measure_is_list=False):
    """converts measurements from metres to inches"""
    if measure_is_list:
        measure_inches = []
        for measurement in measure_metres:
            measure_inches.append(measurement / 0.0254)
    else:
        measure_inches = measure_metres / 0.0254
    return measure_inches


def inches_to_metres(measure_inches, measure_is_list=False):
    """converts measurements from inches to metres"""
    if measure_is_list:
        measure_metres = []
        for measurement in measure_inches:
            measure_metres.append(measurement * 0.0254)
    else:
        measure_metres = measure_inches * 0.0254
    return measure_metres


def get_quat_angles(quat):
    """gets the euler angles from the quaternions"""
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    phi = atan(
        ((2*((q0*q1) + (q2*q3))) /
         (1 - 2*(q1**2 + q2**2)))
    )
    theta = asin(
        2*((q0*q2) - (q3*q1))
    )
    psi = atan(
        ((2*((q0*q3) + (q1*q2))) /
         (1 - 2*(q2**2 + q3**2)))
    )
    return phi, theta, psi


def get_dcm_angle(dcm):
    """gets the theta from the given matrix"""
    theta = 0
    cos_theta = dcm[0, 0]
    sin_theta = dcm[1, 0]
    if (cos_theta > 0) and (sin_theta > 0):
        theta = acos(cos_theta)
    elif (cos_theta < 0) and (sin_theta > 0):
        theta = acos(cos_theta)
    elif (cos_theta > 0) and (sin_theta < 0):
        theta = asin(sin_theta)
    else:
        theta = asin(sin_theta)
    return theta


def rotation_2d(theta_value):
    """gives the rotational matrix for a given angle"""
    cos_theta = np.cos(theta_value)
    sin_theta = np.sin(theta_value)
    rot_mat = np.matrix([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
    return rot_mat


def odom_offset(x_values, y_values, initial_positions):
    """offsets the odometry values based on initial positions"""
    total_values = len(x_values)
    count = 0
    new_x = [None] * total_values
    new_y = [None] * total_values

    while count < total_values:
        new_x[count] = x_values[count] + initial_positions[0]
        new_y[count] = y_values[count] + initial_positions[1]
        count += 1

    return new_x, new_y


def polar_to_cartesian(polar_values, two_dim=True):
    """Converts 2d polar coordinates to 2d cartesian coordinates"""
    cartesian_values = polar_values
    if two_dim:
        cartesian_values[0] = polar_values[0] * cos(polar_values[1])
        cartesian_values[1] = polar_values[0] * sin(polar_values[1])
    return cartesian_values


def cartesian_to_polar(cartesian_values, two_dim=True):
    """Converts 2d cartesian coordinates to 2d polar coordinates"""
    polar_values = cartesian_values
    if two_dim:
        polar_values[0] = sqrt(cartesian_values[0]**2 + cartesian_values[1]**2)
        polar_values[1] = atan(cartesian_values[1]/cartesian_values[0])
    return polar_values
