"""
This module contains the functions for post processing data for inconsistencies
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

import rosagent.parse_files as pf
import rosagent.transform_variables as tv
# import rospy


def get_time_diff(old_time, new_time):
    """Returns the difference in time between two rospy times"""
    time_diff = new_time.to_sec() - old_time.to_sec()
    return time_diff


def get_pos_diff(old_pos, new_pos):
    """Returns the difference in position between two rospy positions"""
    pos_diff = new_pos - old_pos
    return pos_diff


def check_occlusion(current_sim_data):
    """Checks if there is a data point obstructing another data point"""
    data_folder = current_sim_data['data_folder']
    plots_folder = current_sim_data['plots_folder']
    scan_folder = current_sim_data['scan_folder']

    tolerance = 1e-3
    path, dirs, files = next(os.walk(data_folder + scan_folder))
    time_steps = len(files)

    try:
        os.mkdir(os.path.join(plots_folder, scan_folder))
    except OSError:
        print 'Either folder exists or it cannot be created'

    scan_x = [None] * time_steps
    scan_y = [None] * time_steps
    step = 0

    while step < time_steps:
        flag = False

        index = step + 1
        format_string = '%04d' % index
        scan_x[step], scan_y[step] = pf.csv_2d_cartesian(
            os.path.join(data_folder, scan_folder, format_string + '.csv'), True)

        for x_value in scan_x[step]:
            current_y = scan_y[step][scan_x[step].index(x_value)]
            break_y = None

            for y_value in scan_y[step]:
                if (np.abs(current_y - y_value) < tolerance
                        and scan_x[step].index(x_value) is
                        not scan_y[step].index(y_value)):
                    flag = True
                    break_y = y_value
                    break  # breaks from the inner for loop

            if flag:
                index = step + 1
                format_string = '%04d' % index
                print(
                    'Collusion by y value ' + str(scan_y[step].index(break_y))
                    + ' and y value ' + str(scan_x[step].index(x_value)) +
                    ' in time step ' + format_string)
                break  # breaks from the outer for loop

        return 'No collusion in time step ' + str(step + 1)


def check_range_bias(current_sim_data):
    """Checks the given tests to see if there were any range biases"""
    pass


def check_angle_bias(current_sim_data):
    """Checks the given tests to see if there were any angle biases"""
    pass

def process_odom_file(data_folder, odom_time, odom_pos):
    """Process a single odometry file for differences"""
    time_stamps = pf.csv_timestamps(os.path.join(data_folder, odom_time))
    x_values, y_values = pf.csv_2d_cartesian(os.path.join(data_folder, odom_pos))
    orient_values = pf.csv_orientation(os.path.join(data_folder, odom_pos))

    time_diff = []
    x_diff = []
    y_diff = []
    orient_diff = []
    x_velocity = []
    y_velocity = []
    angular_velocity = []

    for time_index in xrange(1, len(x_values)):
        current_diff = get_time_diff(time_stamps[time_index - 1],
                                     time_stamps[time_index])
        time_diff.append(current_diff)

        current_diff = get_pos_diff(x_values[time_index - 1],
                                    x_values[time_index])
        x_diff.append(current_diff)

        current_diff = get_pos_diff(y_values[time_index - 1],
                                    y_values[time_index])
        y_diff.append(current_diff)

        current_diff = get_pos_diff(orient_values[time_index - 1],
                                    orient_values[time_index])
        orient_diff.append(current_diff)

    for diff in time_diff:
        x_velocity.append(x_diff[time_diff.index(diff)] / diff)
        y_velocity.append(y_diff[time_diff.index(diff)] / diff)
        angular_velocity.append(orient_diff[time_diff.index(diff)] / diff)

    return x_velocity, y_velocity, angular_velocity


def differentiate_position(current_sim_data):
    """Integrates the position values to compute the velocity

    :current_sim_data: dict containing all current data
    :returns: checks if velocity values are as expected
    """
    data_folder = current_sim_data['data_folder']
    odom_time_files = current_sim_data['odom_timestamps']
    odom_pos_files = current_sim_data['odom_positions']

    if type(odom_time_files) is list:
        x_vel = []
        y_vel = []
        ang_vel = []
        for bot_times in odom_time_files:
            bot_index = odom_time_files.index(bot_times)
            bot_pos = odom_pos_files[bot_index]
            bot_x, bot_y, bot_ang = process_odom_file(data_folder, bot_times, bot_pos)
            x_vel.append(bot_x)
            y_vel.append(bot_y)
            ang_vel.append(bot_ang)
    else:
        x_vel, y_vel, ang_vel = process_odom_file(data_folder, odom_time_files, odom_pos_files)

    return x_vel, y_vel, ang_vel


def get_rms_error(measured_param, true_param):
    """gets the rms values between"""
    rms_error = 0
    for param in measured_param:
        param_index = measured_param.index(param)
        rms_error += (param - true_param[param_index])**2

    rms_error = sqrt(rms_error)/len(measured_param)
    return rms_error


def check_differentiation(current_sim_data):
    """Checks if the provided velocities match with the integrated"""
    data_folder = current_sim_data['data_folder']
    odom_time_files = current_sim_data['odom_timestamps']
    odom_pos_files = current_sim_data['odom_positions']
    odom_vel_files = current_sim_data['odom_velocities']

    x_from_pos, y_from_pos, ang_from_pos = differentiate_position(current_sim_data)

    if type(odom_vel_files) is list:
        for vel_file in odom_vel_files:
            bot_index = odom_vel_files.index(vel_file)
            bot_x, bot_y = pf.csv_2d_cartesian(os.path.join(data_folder, vel_file))
            bot_ang = pf.csv_orientation(os.path.join(data_folder, vel_file))

            x_rms_error = get_rms_error(bot_x, x_from_pos[bot_index])
            y_rms_error = get_rms_error(bot_y, y_from_pos[bot_index])
            ang_rms_error = get_rms_error(bot_ang, ang_from_pos[bot_index])

            out_string = ('For bot ' + str(bot_index + 1) + ':\n' +
                          'x error: ' + str(x_rms_error) + '\n' +
                          'y error: ' + str(y_rms_error) + '\n' +
                          'ang error: ' + str(ang_rms_error) + '\n')
            print(out_string)
    else:
        x_from_file, y_from_file = pf.csv_2d_cartesian(os.path.join(data_folder, odom_vel_files))
        ang_from_file = pf.csv_orientation(os.path.join(data_folder, odom_vel_files))

        x_rms_error = get_rms_error(x_from_file, x_from_pos)
        y_rms_error = get_rms_error(y_from_file, y_from_pos)
        ang_rms_error = get_rms_error(ang_from_file, ang_from_pos)

        out_string = ('x error: ' + str(x_rms_error) + '\n' +
                      'y error: ' + str(y_rms_error) + '\n' +
                      'ang error: ' + str(ang_rms_error) + '\n')
        print(out_string)
